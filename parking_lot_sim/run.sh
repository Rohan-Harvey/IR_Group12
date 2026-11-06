#!/usr/bin/env bash
set -euo pipefail
parking_package_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if ! command -v gz >/dev/null 2>&1; then
  printf '%s\n' 'Source /opt/ros/jazzy/setup.bash first, and install ros-jazzy-ros-gz if needed.' >&2
  exit 1
fi
export GZ_SIM_RESOURCE_PATH="${parking_package_dir}/models:${GZ_SIM_RESOURCE_PATH:-}"
parking_world_file="${parking_package_dir}/worlds/parking_lot.sdf"
if [[ "${1:-}" =~ ^[Ss][1-6]$ ]]; then
  parking_world_file="${parking_package_dir}/worlds/case_${1,,}.sdf"
  shift
fi
exec gz sim -r "$parking_world_file" "$@"
