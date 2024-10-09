
import subprocess
import time
import numpy as np

subprocess.run(['sudo', 'bash', 'registerpwm'], check=True, text=True, capture_output=True)

import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BOARD)
GPIO.setup(33, GPIO.OUT)
pwm = GPIO.PWM(33, 200)

# try:

#     # Generate values from 7 to 6 with precision 0.01
#     values_7_to_6 = np.arange(7, 5.99, -0.01)

#     # Add the value 7.5
#     value_7_5 = [7.5]

#     # Generate values from 7.5 to 9 with precision 0.01
#     values_7_5_to_9 = np.arange(7.5, 9.01, 0.01)

#     # Concatenate all the values into a single list
#     final_values = np.concatenate((values_7_to_6, value_7_5, values_7_5_to_9))

#     # Add the value 7.5
#     value_7_5 = [7.5]

#     # Convert the numpy array to a list (optional)
#     values = final_values.tolist()


#         # DO THINGS

#     print("start")
#     pwm.start(7.5)

#     time.sleep(0.2)

#     for i in values:
#         print(f"{i}")
#         pwm.ChangeDutyCycle(i)
#         time.sleep(0.2)

# except KeyboardInterrupt:
#     pwm.ChangeDutyCycle(7.5)
#     sys.exit()

try: #24-36

    # Generate values from 7 to 6 with precision 0.01
    values_30_36 = np.arange(31.5, 36, 0.1)
    values_24 = [24]
    values_30_24 = np.arange(27.5, 24, -0.1)
    values_24_36 = np.concatenate((np.flip(values_30_24), [30], values_30_36))


    print("start")
    pwm.start(30)

    time.sleep(2)

    for i in values_30_36.tolist():
        print(f"{i}")
        pwm.ChangeDutyCycle(i)
        time.sleep(0.5)

    print(24)
    pwm.ChangeDutyCycle(24)
    time.sleep(2)

    print(30)
    pwm.ChangeDutyCycle(30)
    time.sleep(3)

    for i in values_30_24.tolist():
        print(f"{i}")
        pwm.ChangeDutyCycle(i)
        time.sleep(0.5)

    for i in values_24_36.tolist():
        print(f"{i}")
        pwm.ChangeDutyCycle(i)
        time.sleep(0.5)

    

except KeyboardInterrupt:
    print(30)
    pwm.ChangeDutyCycle(30)
    exit()