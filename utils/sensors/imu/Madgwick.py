# ---- Minimal built-in Madgwick filter implementation (no external dependency) ----
import numpy as np

class Madgwick:
    def __init__(self, sampleperiod=1/256, beta=0.1):
        self.sampleperiod = sampleperiod
        self.beta = beta
        self.quaternion = np.array([1.0, 0.0, 0.0, 0.0])

    def update(self, q, gyr, acc, mag=None):
        q1, q2, q3, q4 = q
        gx, gy, gz = np.radians(gyr)
        ax, ay, az = acc

        # Normalize accelerometer
        norm = np.linalg.norm(acc)
        if norm == 0: 
            return q
        ax, ay, az = acc / norm

        # Reference direction of Earth's magnetic field (optional)
        if mag is not None:
            mx, my, mz = mag
            norm = np.linalg.norm(mag)
            if norm != 0:
                mx, my, mz = mag / norm
        else:
            mx, my, mz = 0, 0, 0

        # Auxiliary variables
        _2q1 = 2.0 * q1
        _2q2 = 2.0 * q2
        _2q3 = 2.0 * q3
        _2q4 = 2.0 * q4
        _4q1 = 4.0 * q1
        _4q2 = 4.0 * q2
        _4q3 = 4.0 * q3
        _8q2 = 8.0 * q2
        _8q3 = 8.0 * q3
        q1q1 = q1 * q1
        q2q2 = q2 * q2
        q3q3 = q3 * q3
        q4q4 = q4 * q4

        # Gradient descent algorithm corrective step
        s1 = _4q1 * q3q3 + _2q3 * ax + _4q1 * q2q2 - _2q2 * ay
        s2 = _4q2 * q4q4 - _2q4 * ax + 4.0 * q1q1 * q2 - _2q1 * ay - _4q2 + _8q2 * q2q2 + _8q2 * q3q3 + _4q2 * az
        s3 = 4.0 * q1q1 * q3 + _2q1 * ax + _4q3 * q4q4 - _2q4 * ay - _4q3 + _8q3 * q2q2 + _8q3 * q3q3 + _4q3 * az
        s4 = 4.0 * q2q2 * q4 - _2q2 * ax + 4.0 * q3q3 * q4 - _2q3 * ay
        norm_s = np.linalg.norm([s1, s2, s3, s4])
        if norm_s != 0:
            s1, s2, s3, s4 = s1/norm_s, s2/norm_s, s3/norm_s, s4/norm_s

        # Rate of change of quaternion
        qDot1 = 0.5 * (-q2 * gx - q3 * gy - q4 * gz) - self.beta * s1
        qDot2 = 0.5 * ( q1 * gx + q3 * gz - q4 * gy) - self.beta * s2
        qDot3 = 0.5 * ( q1 * gy - q2 * gz + q4 * gx) - self.beta * s3
        qDot4 = 0.5 * ( q1 * gz + q2 * gy - q3 * gx) - self.beta * s4

        # Integrate to yield quaternion
        q1 += qDot1 * self.sampleperiod
        q2 += qDot2 * self.sampleperiod
        q3 += qDot3 * self.sampleperiod
        q4 += qDot4 * self.sampleperiod

        q = np.array([q1, q2, q3, q4])
        q /= np.linalg.norm(q)
        self.quaternion = q
        return q

def q2R(q):
    """Convert quaternion to rotation matrix."""
    q0, q1, q2, q3 = q
    return np.array([
        [1 - 2*(q2*q2 + q3*q3),     2*(q1*q2 - q0*q3),     2*(q1*q3 + q0*q2)],
        [    2*(q1*q2 + q0*q3), 1 - 2*(q1*q1 + q3*q3),     2*(q2*q3 - q0*q1)],
        [    2*(q1*q3 - q0*q2),     2*(q2*q3 + q0*q1), 1 - 2*(q1*q1 + q2*q2)]
    ])
# --------------------------------------------------------------------------

