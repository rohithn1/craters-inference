from throttle_module import ThrottleController
import time
import numpy as np

default_sleep = 0.2

throttle_object = ThrottleController(init_sleep_factor=4, pwm_pin=33, verbose=True)

throttle_object.get_throttle_guide()

try:
    values_30_36 = np.arange(30, 36.1, 0.1)
    values_24 = [24]
    values_30_24 = np.arange(30, 23.9, -0.1)
    values_24_36 = np.concatenate((np.flip(values_30_24), [30], values_30_36))
    

    time.sleep(2)

    for i in values_30_36.tolist():
        throttle_object.speed(i)
        time.sleep(default_sleep)

    throttle_object.speed(24)
    time.sleep(2)

    throttle_object.speed(30)
    time.sleep(3)

    for i in values_30_24.tolist():
        throttle_object.speed(i)
        time.sleep(default_sleep)

    for i in values_24_36.tolist():
        throttle_object.speed(i)
        time.sleep(default_sleep)

    throttle_object.apply_neutral()

except KeyboardInterrupt:
    print(30)
    pwm.ChangeDutyCycle(30)
    exit()
