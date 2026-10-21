#!/usr/bin/env bash
set -eo pipefail
repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
export CMAKE_PREFIX_PATH="$repo_root/deps/install:${CMAKE_PREFIX_PATH:-}"
export LD_LIBRARY_PATH="$repo_root/deps/install/lib:${LD_LIBRARY_PATH:-}"
export MAKEFLAGS=-j2
export CMAKE_BUILD_PARALLEL_LEVEL=2
cd "$repo_root/ros_ws"
colcon build --executor sequential --packages-up-to fast_lio fastlio_px4_bridge   --cmake-args -DCMAKE_BUILD_TYPE=Release -DROS_EDITION=ROS2 -DDISTRO_ROS=jazzy   -DLIVOX_LIDAR_SDK_LIBRARY="$repo_root/deps/install/lib/liblivox_lidar_sdk_shared.so"   -DLIVOX_LIDAR_SDK_INCLUDE_DIR="$repo_root/deps/install/include"
