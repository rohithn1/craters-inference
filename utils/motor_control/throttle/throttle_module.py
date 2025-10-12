import RPi.GPIO as GPIO
import time
import subprocess
import os
import sys
import math
import numpy as np
import threading
import queue

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
        self.CURRENT_SPEED = self.NEUTRAL
        self.TARGET_SPEED = self.NEUTRAL
        self.THROTTLE_PICKUP_RATE = 0.01
        self.THROTTLE_INCREMENT = 0.1
        
        # Threading primitives
        self._stop_event = threading.Event()
        self._lock = threading.Lock()
        self._speed_queue = queue.Queue()

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
        
        # Start worker threads
        self._target_thread = threading.Thread(target=self._target_speed_worker, daemon=True)
        self._pwm_thread = threading.Thread(target=self._pwm_worker, daemon=True)
        self._target_thread.start()
        self._pwm_thread.start()

    def __del__(self):
        try:
            self.close()
        except Exception:
            pass

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
            # Enqueue target speed; target worker will set TARGET_SPEED
            self._speed_queue.put(speed)

        except Exception as e:
            print("can't use throttle:", e)

    def get_throttle_guide(self):
        print("R            N           D")
        print("24          30          36")

    def _target_speed_worker(self):
        while not self._stop_event.is_set():
            try:
                requested_speed = self._speed_queue.get(timeout=0.1)
            except queue.Empty:
                continue
            mapped_speed = self.map_to_range(requested_speed)
            with self._lock:
                self.TARGET_SPEED = mapped_speed
            self._speed_queue.task_done()

    def _pwm_worker(self):
        while not self._stop_event.is_set():
            with self._lock:
                current_speed = self.CURRENT_SPEED
                target_speed = self.TARGET_SPEED

            if abs(target_speed - current_speed) < self.THROTTLE_INCREMENT:
                time.sleep(self.THROTTLE_PICKUP_RATE)
                continue

            if target_speed > current_speed:
                step = self.THROTTLE_INCREMENT
            # elif target_speed == current_speed:
            #     step = 0
            else:
                step = -self.THROTTLE_INCREMENT
            next_speed = current_speed + step
            if (step > 0 and next_speed > target_speed) or (step < 0 and next_speed < target_speed):
                next_speed = target_speed

            with self._lock:
                self.CURRENT_SPEED = next_speed
                try:
                    self.PWM.ChangeDutyCycle(self.CURRENT_SPEED)
                except Exception as e:
                    if self.verbose:
                        print("PWM.ChangeDutyCycle error:", e)

            time.sleep(self.THROTTLE_PICKUP_RATE)

    def close(self):
        self._stop_event.set()
        try:
            if hasattr(self, '_target_thread') and self._target_thread.is_alive():
                self._target_thread.join(timeout=1.0)
        except Exception:
            pass
        try:
            if hasattr(self, '_pwm_thread') and self._pwm_thread.is_alive():
                self._pwm_thread.join(timeout=1.0)
        except Exception:
            pass

        try:
            self.PWM.ChangeDutyCycle(self.NEUTRAL)
            time.sleep(self.NAP_TIME*self.init_sleep_factor)
            if hasattr(self, 'PWM'):
                self.PWM.stop()
        finally:
            GPIO.cleanup()
