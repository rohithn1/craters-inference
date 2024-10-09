import RPi.GPIO as GPIO
import time
import subprocess
import os
import sys
import numpy as np

class SteeringController:
    def __init__(self, init_sleep_factor=1, pwm_pin=32):
        GPIO.setwarnings(False)
        GPIO.cleanup()

        self.init_sleep_factor = abs(init_sleep_factor)
        
        self.LEFT_LIMIT = 24
        self.HALF_LEFT = 27
        self.CENTER = 30
        self.HALF_RIGHT = 33
        self.RIGHT_LIMIT = 36

        self.NAP_TIME = 0.01

        subprocess.run(['sudo', 'bash', 'registerpwm'], check=True)

        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(pwm_pin, GPIO.OUT)
        self.PWM = GPIO.PWM(pwm_pin, 200)
        self.PWM.start(self.CENTER)
        time.sleep(self.NAP_TIME*self.init_sleep_factor)

        return

    def __del__(self):
        time.sleep(self.NAP_TIME*self.init_sleep_factor)
        if hasattr(self, 'PWM'):
            self.PWM.stop()    
        GPIO.cleanup()

    def _steer(self, new_direction=None):
        if new_direction is None:
            new_direction = self.CENTER
        if new_direction > self.RIGHT_LIMIT:
            new_direction = self.RIGHT_LIMIT
        if new_direction < self.LEFT_LIMIT:
            new_direction = self.LEFT_LIMIT
        try:
            self.PWM.ChangeDutyCycle(new_direction)
        except Exception as e:
            print("can't steer:", e)

    def move_towards(self, start, end, n=16):
        # Calculate the difference between start and end
        difference = abs(end - start)
        # Determine the direction of movement
        direction = 1 if end > start else -1
        # Calculate the increment for each step
        increment = difference * 0.8 / n  
        # Initialize the current position to start
        current_position = start
        # Loop through n steps
        for step in range(1, n + 1):
            # Calculate the new position
            new_position = start + (increment * step * direction)
            # If the direction is negative and the new position exceeds the end, or vice versa,
            # set it to end
            if (direction == 1 and new_position > end) or (direction == -1 and new_position < end):
                new_position = end
            self._steer(new_direction=new_position)
            time.sleep(self.NAP_TIME)
            # Update the current position
            current_position = new_position
        self._steer(new_direction=end)

    def range_test(self, steps=16):
        steps = abs(steps)
        self.full_right()
        time.sleep(1)
        self.full_left()
        time.sleep(1)
        self.move_towards(self.LEFT_LIMIT, self.RIGHT_LIMIT, steps)
        time.sleep(1)
        self.move_towards(self.RIGHT_LIMIT, self.LEFT_LIMIT, steps)
        time.sleep(1)
        self.move_towards(self.LEFT_LIMIT, self.CENTER, steps)
    
    def full_left(self):
       self._steer(new_direction=self.LEFT_LIMIT)

    def full_right(self):
        self._steer(new_direction=self.RIGHT_LIMIT)

    def center(self):
        self._steer()

    def get_steering_angle_guide(self):
        print("L           C           R")
        print("24         30          36")

        
