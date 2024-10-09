import RPi.GPIO as GPIO
import time
import subprocess
import os
import sys
import math
import numpy as np

class ThrottleController:
    def __init__(self, init_sleep_factor, pwm_pin=33, verbose=False, percent=1, lower_lim=True):
        GPIO.setwarnings(False)
        GPIO.cleanup()
        
        self.verbose = verbose
        self.percent = percent
        self.lower_lim = lower_lim
        self.init_sleep_factor = abs(init_sleep_factor)
        
        self.FULL_REVERSE = 24
        self.FULL_FORWARD = 36
        self.NEUTRAL = 30
        self.CREEP_REVERSE = 27.5
        self.CREEP_FORWARD = 31.5
        self.NAP_TIME = 0.05

        if percent < 1:
            f = self.FULL_FORWARD - self.CREEP_FORWARD
            self.FULL_FORWARD = self.FULL_FORWARD - (f*percent)
            r = self.FULL_REVERSE - self.CREEP_REVERSE
            self.FULL_REVERSE = self.FULL_REVERSE + (r*percent)

        subprocess.run(['sudo', 'bash', 'registerpwm'], check=True)

        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(pwm_pin, GPIO.OUT)
        self.PWM = GPIO.PWM(pwm_pin, 200)
        self.PWM.start(self.NEUTRAL)
        time.sleep(self.NAP_TIME*self.init_sleep_factor)

    def __del__(self):
        self.PWM.ChangeDutyCycle(self.NEUTRAL)
        time.sleep(self.NAP_TIME*self.init_sleep_factor)
        if hasattr(self, 'PWM'):
            self.PWM.stop()    
        GPIO.cleanup()   

    def apply_neutral(self):
        self.speed(self.NEUTRAL)

    def map_to_range(self, mapped_value):
        if self.CREEP_REVERSE < mapped_value < self.CREEP_FORWARD and self.lower_lim:
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
        try:
            pwm_value = self.map_to_range(speed)
            if self.verbose:
                print(pwm_value)
            self.PWM.ChangeDutyCycle(pwm_value)
        except Exception as e:
            print("can't use throttle:", e)

    def get_throttle_guide(self):
        print("R            N           D")
        print("24          30          36")
