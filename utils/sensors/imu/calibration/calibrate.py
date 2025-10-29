#!/usr/bin/env python3
import sys
import time
import math
import argparse
import numpy as np
from smbus2 import SMBus
from mpu9250_jmdev.registers import *
from mpu9250_jmdev.mpu_9250 import MPU9250

# ---------- Helpers ----------
G0 = 9.80665  # m/s^2
DEG2RAD = math.pi / 180.0

def allan_deviation(x, dt, taus):
    """
    Allan deviation for a 1D zero-mean stationary series x (rate or accel).
    Uses the overlapping Allan variance via integrated second-differences.
    x: array [N], in SI units (rad/s or m/s^2)
    dt: sample period (s)
    taus: array of cluster times (s)
    returns: sigma (same length as taus)
    """
    N = len(x)
    y = np.cumsum(x) * dt  # integrate once
    sigmas = np.empty_like(taus, dtype=float)
    sigmas[:] = np.nan
    for i, tau in enumerate(taus):
        m = int(round(tau / dt))
        if m < 1 or 2*m >= N:
            continue
        z = (y[2*m:] - 2*y[m:-m] + y[:-2*m])
        sigmas[i] = np.sqrt(np.mean(z**2) / (2.0 * (tau**2)))
    return sigmas

def pick_band(log_tau, log_sigma, target_slope, min_span=6):
    """
    Slide a window to find a segment whose slope ~ target_slope
    Returns indices slice (start, end) inclusive
    """
    best = None
    n = len(log_tau)
    for w in range(min_span, max(min_span+1, n//2)):
        for s in range(0, n - w):
            e = s + w
            X = np.vstack([log_tau[s:e], np.ones(e-s)]).T
            y = log_sigma[s:e]
            if np.any(~np.isfinite(y)):
                continue
            a, b = np.linalg.lstsq(X, y, rcond=None)[0]  # y = a*log_tau + b
            err = abs(a - target_slope)
            if best is None or err < best[0]:
                best = (err, s, e, a, b)
    return best  # (err, s, e, slope, intercept)

def fit_white_noise(taus, sigma):
    # For white noise: sigma ~ K / sqrt(tau) => log(sigma) = logK - 0.5 log(tau)
    log_tau = np.log(taus)
    log_sig = np.log(sigma)
    band = pick_band(log_tau, log_sig, target_slope=-0.5)
    if band is None:
        return np.nan, (0, len(taus))
    _, s, e, slope, intercept = band
    K = math.exp(intercept)  # sigma ≈ K * tau^{-1/2}
    q = math.sqrt(2.0) * K   # noise density (SI / √Hz)
    return q, (s, e)

def fit_bias_rw(taus, sigma):
    # For bias random walk: sigma ~ sqrt(C * tau) => log(sigma)= 0.5 log(tau) + 0.5 log C
    log_tau = np.log(taus)
    log_sig = np.log(sigma)
    band = pick_band(log_tau, log_sig, target_slope=+0.5)
    if band is None:
        return np.nan, (0, len(taus))
    _, s, e, slope, intercept = band
    C = math.exp(2.0 * intercept)   # since intercept = 0.5*log C
    q_rw = math.sqrt(3.0 * C)       # bias RW density (SI/sec^(1/2))
    return q_rw, (s, e)

def to_opencv_matrix_4x4(M):
    flat = ", ".join(f"{v:.12g}" for v in M.reshape(-1))
    return (
        "!!opencv-matrix\n"
        "   rows: 4\n"
        "   cols: 4\n"
        "   dt: f\n"
        f"   data: [{flat}]"
    )

def parse_tbcmat(arg):
    """
    --tbcmat can be:
      - 16 comma/space-separated numbers (row-major)
      - a single keyword 'identity'
    Returns 4x4 ndarray (float)
    """
    if arg.strip().lower() == "identity":
        return np.eye(4, dtype=float)
    # split by comma or space
    parts = [p for p in arg.replace(",", " ").split() if p]
    if len(parts) != 16:
        raise ValueError("Expected 16 numbers for --tbcmat (row-major 4x4).")
    vals = [float(p) for p in parts]
    return np.array(vals, dtype=float).reshape(4, 4)

# ---------- Main ----------
def main():
    ap = argparse.ArgumentParser(description="Capture MPU9250 noise params and print YAML with !!opencv-matrix.")
    ap.add_argument("--i2c-bus", type=int, default=1, help="I2C bus index (default: 1)")
    ap.add_argument("--mpu-addr", type=lambda x: int(x, 0), default=MPU9050_ADDRESS_68,
                    help="MPU I2C address (e.g., 0x68). Default: 0x68")
    ap.add_argument("--duration", type=float, default=300.0, help="Record time in seconds (stationary). Default: 300")
    ap.add_argument("--tau-min", type=float, default=None, help="Minimum tau (s) for Allan; default auto (≈ 3*dt)")
    ap.add_argument("--tau-max", type=float, default=None, help="Maximum tau (s) for Allan; default auto (≈ duration/10)")
    ap.add_argument("--taus-per-decade", type=int, default=20, help="Logspace density for taus. Default: 20")
    ap.add_argument("--tbcmat", type=str, default="identity",
                    help="4x4 T_b_c1 as 16 numbers (row-major) or 'identity'. Default: identity")
    ap.add_argument("--gfs", type=int, default=GFS_1000, help="Gyro full-scale reg (default: 1000 dps)")
    ap.add_argument("--afs", type=int, default=AFS_8G, help="Accel full-scale reg (default: ±8g)")
    args = ap.parse_args()

    # Configure IMU
    bus = SMBus(args.i2c_bus)
    mpu = MPU9250(
        address_ak=AK8963_ADDRESS,
        address_mpu_master=args.mpu_addr,
        address_mpu_slave=None,
        bus=args.i2c_bus,
        gfs=args.gfs,
        afs=args.afs,
        mfs=AK8963_BIT_16,
        mode=AK8963_MODE_C100HZ
    )
    mpu.configure()

    print(f"IMU initialized on I2C bus {args.i2c_bus}, address 0x{args.mpu_addr:02X}")
    print("Place the unit STILL on the jig. Recording stationary data...")
    sys.stdout.flush()

    t0 = time.time()
    ts = []
    gx, gy, gz = [], [], []
    ax, ay, az = [], [], []

    try:
        while True:
            a = mpu.readAccelerometerMaster()  # g
            g = mpu.readGyroscopeMaster()      # deg/s

            ts.append(time.time())
            ax.append(a[0] * G0)
            ay.append(a[1] * G0)
            az.append(a[2] * G0)

            gx.append(g[0] * DEG2RAD)
            gy.append(g[1] * DEG2RAD)
            gz.append(g[2] * DEG2RAD)

            if ts[-1] - t0 >= args.duration:
                break
            # sample as fast as possible; you can add time.sleep(0.0x) if needed
    except KeyboardInterrupt:
        print("\nStopped early by user.", file=sys.stderr)
    finally:
        bus.close()

    ts = np.array(ts, dtype=float)
    if len(ts) < 100:
        print("Not enough samples collected. Try increasing --duration.", file=sys.stderr)
        sys.exit(1)

    # Compute nominal dt and frequency
    dts = np.diff(ts)
    dt = np.median(dts)
    freq = 1.0 / dt

    gx = np.array(gx); gy = np.array(gy); gz = np.array(gz)
    ax = np.array(ax); ay = np.array(ay); az = np.array(az)

    # Remove means (stationary)
    gx -= np.mean(gx); gy -= np.mean(gy); gz -= np.mean(gz)
    ax -= np.mean(ax); ay -= np.mean(ay); az -= np.mean(az)

    # Build taus
    tau_min = args.tau_min if args.tau_min else max(3*dt, 0.01)
    tau_max = args.tau_max if args.tau_max else max(args.duration/10.0, tau_min*10)
    decades = max(1, int(math.log10(tau_max) - math.log10(tau_min) + 0.999))
    num = max(10, args.taus_per_decade * decades)
    taus = np.logspace(math.log10(tau_min), math.log10(tau_max), num)

    # Allan deviation per axis
    adev_gx = allan_deviation(gx, dt, taus)
    adev_gy = allan_deviation(gy, dt, taus)
    adev_gz = allan_deviation(gz, dt, taus)

    adev_ax = allan_deviation(ax, dt, taus)
    adev_ay = allan_deviation(ay, dt, taus)
    adev_az = allan_deviation(az, dt, taus)

    # Fit white noise (q) and bias RW (q_rw) per axis
    qg = []
    qbg = []
    for adev in (adev_gx, adev_gy, adev_gz):
        q, _ = fit_white_noise(taus, adev)
        rw, _ = fit_bias_rw(taus, adev)
        qg.append(q); qbg.append(rw)

    qa = []
    qba = []
    for adev in (adev_ax, adev_ay, adev_az):
        q, _ = fit_white_noise(taus, adev)
        rw, _ = fit_bias_rw(taus, adev)
        qa.append(q); qba.append(rw)

    # Averages across axes (robust median)
    NoiseGyro = float(np.nanmedian(qg))     # rad/s/√Hz
    GyroWalk  = float(np.nanmedian(qbg))    # rad/s^2/√Hz
    NoiseAcc  = float(np.nanmedian(qa))     # m/s^2/√Hz
    AccWalk   = float(np.nanmedian(qba))    # m/s^3/√Hz

    # Optional: show per-axis (comment lines)
    per_axis = {
        "gyro_noise_rad_s_sqrtHz": qg,
        "gyro_bias_rw_rad_s2_sqrtHz": qbg,
        "acc_noise_m_s2_sqrtHz": qa,
        "acc_bias_rw_m_s3_sqrtHz": qba,
    }

    # T_b_c1
    try:
        Tbc = parse_tbcmat(args.tbcmat)
    except Exception as e:
        print(f"Error parsing --tbcmat: {e}", file=sys.stderr)
        Tbc = np.eye(4, dtype=float)

    # ---------- Print YAML snippet ----------
    print("\n# ---------- COPY BELOW INTO YOUR CONFIG ----------")
    print("# IMU-to-body (camera) extrinsics")
    print("IMU.T_b_c1:", to_opencv_matrix_4x4(Tbc))

    print("\n# IMU noise (continuous-time densities)")
    print(f"IMU.NoiseGyro: {NoiseGyro:.6g}    # rad/s/√Hz")
    print(f"IMU.NoiseAcc:  {NoiseAcc:.6g}    # m/s^2/√Hz")
    print(f"IMU.GyroWalk:  {GyroWalk:.6g}    # rad/s^2/√Hz")
    print(f"IMU.AccWalk:   {AccWalk:.6g}    # m/s^3/√Hz")
    print(f"IMU.Frequency: {freq:.3f}")

    # Per-axis (comments)
    print("\n# Per-axis (for reference):")
    for k, v in per_axis.items():
        vals = ", ".join(f"{x:.6g}" if np.isfinite(x) else "nan" for x in v)
        print(f"# {k}: [{vals}]")

    # Sanity notes
    print("\n# Notes:")
    print("# - Ensure the sensor is perfectly STILL during capture; longer duration -> more reliable fits.")
    print("# - If the auto-picked bands look off, rerun with --tau-min / --tau-max.")
    print("# - Pass your known 4x4 with --tbcmat 'r11 r12 ... r44' to embed it into the output.")
    print("#   Example:")
    print("#   --tbcmat '0.014865543 -0.99988093 0.004140297 -0.021640145  0.999557249 0.014967213 0.02571553 -0.064676987  -0.025774437 0.003756188 0.999660727 0.009810731  0 0 0 1'")

if __name__ == "__main__":
    main()
