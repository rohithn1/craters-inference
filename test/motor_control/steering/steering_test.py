from steering_module import SteeringController
import time

default_sleep = 2

steering_obj = SteeringController(init_sleep_factor=4, pwm_pin=32)

steering_obj.get_steering_angle_guide()

time.sleep(default_sleep)

steering_obj.full_right()

time.sleep(default_sleep)

steering_obj.move_towards(steering_obj.RIGHT_LIMIT, steering_obj.LEFT_LIMIT)

time.sleep(default_sleep)

steering_obj.move_towards(steering_obj.LEFT_LIMIT, steering_obj.RIGHT_LIMIT)

time.sleep(default_sleep)

steering_obj.center()

# time.sleep(default_sleep)
# steering_obj.range_test()
# time.sleep(default_sleep)


del steering_obj
