import evdev
from evdev import InputDevice, categorize, ecodes, list_devices
import socket
import subprocess
import os
import signal

# List all input devices
devices = [InputDevice(path) for path in list_devices()]

device_path = None

# Print all available devices for debugging purposes
print("Available input devices:")
for device in devices:
    print(f"Device: {device.name}, Path: {device.path}")
    user_input = input("Use this device?: [Y/N]")
    if user_input == "Y" or user_input == "y":
        print(f"Using {device.name}")
        device_path = device.path
        break

if device_path is None:
    print("Quitting since no device was chosen")
    exit(1)

# Create an InputDevice instance for the joypad
device = InputDevice(device_path)

# Constants for the R2 and L2 trigger events
R2_TRIGGER = ecodes.ABS_RZ
L2_TRIGGER = ecodes.ABS_Z

# Print device information
print(f"\nSelected device:\nName: {device.name}\nPath: {device.path}\nPhys: {device.phys}\n")

# Event loop to read input events
print("Listening for input events...")

# Local Host
host='127.0.0.1'
# Throttle Port
port_t=65432
# Steering Port
port_s=65433

# Setting up communication to throttle process
# Create a TCP/IP socket
throttle_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
throttle_server = (host, port_t)
print(f'Connecting to {host}:{port_t} for throttle communication')
throttle_socket.connect(throttle_server)

# Setting up communication to steering process
# Create a TCP/IP socket
steering_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
steering_server = (host, port_s)
print(f'Connecting to {host}:{port_t} for steering communication')
steering_socket.connect(steering_server)

gas = 0
brake = 0
min_throttle = "31.5"
min_reverse = "27.1"
cam_pid = None

def joystick_to_throttle(gas, brake, sensitivity=0.357):
    x = gas - brake
    return 30 + (sensitivity * ((x / 255) * 6)) + ((1-sensitivity) * (((x / 255)**3 * 6)))

def joystick_to_steering(val):
    return (((val - 127.5) / 256) * 12) + 30

try:
    for event in device.read_loop():
        #R2 (BRAKE/REVERSE)
        if event.type == ecodes.EV_ABS and event.code == 4:
            brake = event.value
            throttle = f"{joystick_to_throttle(gas, brake)}\n"
            throttle_socket.sendall(throttle.encode())
            #throttle_obj.speed(throttle)
        
        #L2 (GAS)
        if event.type == ecodes.EV_ABS and event.code == 3:
            gas = event.value
            throttle = f"{joystick_to_throttle(gas, brake)}\n"
            throttle_socket.sendall(throttle.encode())
            #throttle_obj.speed(throttle)

        #Right JoyStick (STEER)
        if event.type == ecodes.EV_ABS and event.code == 2:
            angle = f"{joystick_to_steering(event.value)}\n"
            steering_socket.sendall(angle.encode())

        #Left JoyStick
        #if event.type == ecodes.EV_ABS and event.code == 1:
        #    print(f"L JoyStick: {event.value}")

        #□ Button (Take picture)
        if event.type == ecodes.EV_KEY and event.code == 304 and event.value == 1:
            if cam_pid is None:
                print("Starting camera")
                command = ["sudo", "python2", "/home/craters/craters-inference/utils/CSI-Camera/capture_pics.py"]
                try:
                    process = subprocess.Popen(command)
                    cam_pid = process.pid
                    print(cam_pid)
                except subprocess.CalledProcessError as e:
                    print("An error occurred:", e.stderr)
            else:
                print("Stopping camera")
                os.kill(cam_pid, signal.SIGTERM)
                cam_pid = None

        #Up Arrow (Creep)
        if event.type == ecodes.EV_ABS and event.code == 17:
            if event.value == -1:
                throttle_socket.sendall(min_throttle.encode())
            elif event.value == 1:
                throttle_socket.sendall(min_reverse.encode())
            else:
                throttle_socket.sendall("30".encode())

        #X Button (Kill Switch)
        if event.type == ecodes.EV_KEY and event.code == 305:
            print(f"STOP")
            throttle_socket.sendall("30".encode())
            command = ["sudo", "pkill", "-f", "python"]
            try:
                subprocess.run(command, check=True, capture_output=True, text=True, input="dhoom2\n")
            except subprocess.CalledProcessError as e:
                print("An error occurred:", e.stderr)
            throttle_socket.close()
            steering_socket.close()
            break
       
        #print(f"Event: {event}")
except KeyboardInterrupt:
    print("Exiting...")

