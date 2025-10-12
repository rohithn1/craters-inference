#!/bin/bash
set -ex 

cd ~/craters-inference/catkin_ws
rm -rf build devel

catkin_make \
  -DPYTHON_EXECUTABLE=/usr/bin/python3 \
  -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m \
  -DPYTHON_LIBRARY=/usr/lib/aarch64-linux-gnu/libpython3.6m.so \
  -DNUMPY_INCLUDE_DIR=/usr/lib/python3/dist-packages/numpy/core/include \
  -DBoost_NO_BOOST_CMAKE=TRUE \
  -j4

echo "Activate by running: source devel/setup.bash"
