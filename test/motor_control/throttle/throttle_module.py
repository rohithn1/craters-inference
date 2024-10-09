import RPi.GPIO as GPIO
import time
import subprocess
import os
import sys
import math
import numpy as np

class ThrottleController:
    def __init__(self, init_sleep_factor, pwm_pin=33, verbose=False):
        GPIO.setwarnings(False)
        GPIO.cleanup()
        
        self.verbose = verbose
        self.init_sleep_factor = abs(init_sleep_factor)
        
        self.FULL_REVERSE = 24
        self.FULL_FORWARD = 36
        self.NEUTRAL = 30
        self.NAP_TIME = 0.05

        subprocess.run(['sudo', 'bash', 'registerpwm'], check=True, text=True, capture_output=True)

        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(pwm_pin, GPIO.OUT)
        self.PWM = GPIO.PWM(pwm_pin, 200)
        self.PWM.start(self.NEUTRAL)
        time.sleep(self.NAP_TIME*self.init_sleep_factor)

    def __del__(self):
        time.sleep(self.NAP_TIME*self.init_sleep_factor)
        if hasattr(self, 'PWM'):
            self.PWM.stop()    
        GPIO.cleanup()   

    def apply_neutral(self):
        self.speed(self.NEUTRAL)

    def map_to_range(self, mapped_value):
        if 27.5 < mapped_value < 31.5:
            mapped_value = 30
            #if mapped_value < 30:
            #    self.PWM.ChangeDutyCycle(24)
            #elif mapped_value > 30:
            #    self.PWM.ChangeDutyCycle(36)
                
        if mapped_value < self.FULL_REVERSE:
            mapped_value = self.FULL_REVERSE
        if mapped_value > self.FULL_FORWARD:
            mapped_value = self.FULL_FORWARD
        return mapped_value
   
    def speed(self, speed):
        pwm_value = self.map_to_range(speed)
        if self.verbose:
            print(pwm_value)
        self.PWM.ChangeDutyCycle(pwm_value)

    def get_throttle_guide(self):
        print("R            N           D")
        print("24          30          36")
