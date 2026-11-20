#!/usr/bin/env bash
set -eo pipefail
if [[ ! -r /opt/ros/jazzy/setup.bash ]]; then
  printf '%s\n' 'ROS 2 Jazzy was not found at /opt/ros/jazzy. Run this script inside Ubuntu 24.04.' >&2
  exit 1
fi
source /opt/ros/jazzy/setup.bash
parking_project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$parking_project_dir"
parking_check_args=()
for parking_arg in "$@"; do
  if [[ "$parking_arg" == "rviz:=false" ]]; then
    parking_check_args+=(--no-rviz)
  fi
done
python3 scripts/check_runtime.py "${parking_check_args[@]}"
printf '%s\n' 'Starting Gazebo, SLAM and autonomous parking. Stop this terminal with Ctrl+C.'
exec ros2 launch launch/autonomous_parking.launch.py "$@"
