#!/usr/bin/env bash

sudo python3 ~/craters-inference/utils/launch_throttle_proc.py &
THROTTLE_PID=$!
echo "Throttle PID: ${THROTTLE_PID}"
sudo python3 ~/craters-infernce/utils/launch_steering_proc.py &
STEERING_PID=$!
echo "Steering PID: ${STEERING_PID}"

# --- cleanup on Ctrl+C ---
trap "echo -e '\n[INFO] Stopping Steering and Throttle processes...'; kill $BAG_PID $LAUNCH_PID; exit 0" SIGINT

# --- keep script running until Ctrl+C ---
while true; do
  sleep 2
done

