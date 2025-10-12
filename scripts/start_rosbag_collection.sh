#!/bin/bash

BAG_DIR="$HOME/rosbags"
BAG_NAME="recording_$(date +%Y%m%d_%H%M%S).bag"
WORKSPACE="$HOME/catkin_ws"
LAUNCH_FILE="imu_stream sensor_streams.launch"
ROS_TOPICS="/csi_cam_0/image_raw /csi_cam_0/camera_info /imu"
# ----------------------------------------------------------

echo "[INFO] Setting up ROS environment..."
source /opt/ros/melodic/setup.bash
source "$WORKSPACE/devel/setup.bash"

mkdir -p "$BAG_DIR"

echo "[INFO] Launching camera + IMU streams..."
roslaunch $LAUNCH_FILE &
LAUNCH_PID=$!

# --- wait for topics to appear ---
echo "[INFO] Waiting for topics to be published..."
until rostopic list | grep -q "/csi_cam_0/image_raw" && rostopic list | grep -q "/imu"; do
  sleep 1
done

echo "[INFO] Topics detected. Starting rosbag recording..."
rosbag record $ROS_TOPICS -O "$BAG_DIR/$BAG_NAME" &
BAG_PID=$!

# --- cleanup on Ctrl+C ---
trap "echo -e '\n[INFO] Stopping rosbag and ROS launch...'; kill $BAG_PID $LAUNCH_PID; exit 0" SIGINT

# --- keep script running until Ctrl+C ---
while true; do
  sleep 2
done

