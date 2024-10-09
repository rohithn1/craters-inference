import RPi.GPIO as GPIO
import time
import numpy as np

nap = 0.05

GPIO.setmode(GPIO.BOARD)
GPIO.setup(32, GPIO.OUT)
pwm = GPIO.PWM(32, 50)
pwm.start(7.5)
time.sleep(nap*4)
#angles = [6, 6.25, 6.5, 6.75, 7, 7.25, 7.5, 7.75, 8, 8.25, 8.5, 8.75, 9]
angles = np.linspace(6, 9, 30)

while True:
    for i in angles:
        pwm.ChangeDutyCycle(i)
        time.sleep(nap)
        print(i)
    for i in reversed(angles):
        pwm.ChangeDutyCycle(i)
        time.sleep(nap)
        print(i)
