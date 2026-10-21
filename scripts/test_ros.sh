#!/usr/bin/env bash
set -eo pipefail
repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
source "$repo_root/ros_ws/install/local_setup.bash"
cd "$repo_root/ros_ws"
colcon test --packages-select fast_lio fastlio_px4_bridge --executor sequential   --ctest-args -R 'bridge_behavior|test_gravity_alignment' --output-on-failure
colcon test-result --verbose
