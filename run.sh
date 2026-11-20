#!/usr/bin/env bash
set -eo pipefail
if [[ -r /opt/ros/jazzy/setup.bash ]]; then
  source /opt/ros/jazzy/setup.bash
fi
set -u
parking_package_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
python3 "${parking_package_dir}/scripts/check_runtime.py" --scene
export GZ_SIM_RESOURCE_PATH="${parking_package_dir}/models:${GZ_SIM_RESOURCE_PATH:-}"
parking_world_file="${parking_package_dir}/worlds/parking_lot.sdf"
if [[ "${1:-}" =~ ^[Ss][1-6]$ ]]; then
  parking_world_file="${parking_package_dir}/worlds/case_${1,,}.sdf"
  shift
fi
exec gz sim -r "$parking_world_file" "$@"
