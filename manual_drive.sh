#!/usr/bin/env bash

sudo python3 utils/launch_throttle_proc.py &
sudo python3 utils/launch_steering_proc.py &
sudo python3 utils/joypad_controller.py