#!/usr/bin/env bash
set -eo pipefail
repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$repo_root"
vcs import --recursive < dependencies.repos
if git -C ros_ws/src/livox_ros_driver2 apply --reverse --check "$repo_root/patches/livox-jazzy-hardware.patch" 2>/dev/null; then
  echo "Livox patch already applied"
else
  git -C ros_ws/src/livox_ros_driver2 apply --check "$repo_root/patches/livox-jazzy-hardware.patch"
  git -C ros_ws/src/livox_ros_driver2 apply "$repo_root/patches/livox-jazzy-hardware.patch"
fi
cp ros_ws/src/livox_ros_driver2/package_ROS2.xml ros_ws/src/livox_ros_driver2/package.xml
