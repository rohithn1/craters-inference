#!/usr/bin/env bash

set -e

BAG_DIR="$(dirname "$0")/rosbags"
BAG_PREFIX="$1"

cd "$BAG_DIR" || exit 1

while true; do
    echo "Starting playback for prefix: ${BAG_PREFIX}"
    rosbag play $(ls -1v ${BAG_PREFIX}_*.bag) \
        --rate 1.0 \
        --topics /imu /csi_cam_0/image_raw

    echo "Playback complete for ${BAG_PREFIX}. Restarting in 3 seconds..."
    sleep 3
done
