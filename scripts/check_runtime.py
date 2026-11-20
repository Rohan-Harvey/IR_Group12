#!/usr/bin/env python3
"""Fail with actionable diagnostics before opening a partial ROS/Gazebo session."""

import argparse
import os
from pathlib import Path
import shutil
import sys


PACKAGES = (
    "ros_gz_sim", "ros_gz_bridge", "robot_state_publisher", "slam_toolbox",
    "nav2_controller", "nav2_planner", "nav2_behaviors", "nav2_bt_navigator",
    "nav2_lifecycle_manager", "nav2_smac_planner",
    "nav2_regulated_pure_pursuit_controller", "nav2_map_server",
)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", action="store_true", help="Check the standalone scene only")
    parser.add_argument("--no-rviz", action="store_true", help="RViz will not be launched")
    args = parser.parse_args(argv)
    project = Path(__file__).resolve().parents[1]
    errors = []
    if shutil.which("gz") is None:
        errors.append("gz is unavailable. Source /opt/ros/jazzy/setup.bash.")
    for relative in ("worlds/parking_lot.sdf", "models/limo/meshes/limo_base.dae",
                     "models/limo/meshes/limo_wheel.dae",
                     "models/walking_person/meshes/walk_blue.dae"):
        if not (project/relative).is_file():
            errors.append(f"Missing project file: {relative}. Extract the entire project ZIP.")
    if not args.scene:
        if os.environ.get("ROS_DISTRO") != "jazzy":
            errors.append("This launcher requires ROS 2 Jazzy: source /opt/ros/jazzy/setup.bash.")
        try:
            import rclpy  # noqa: F401 -- verify the launch interpreter can load ROS
            import numpy  # noqa: F401
            from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
        except ImportError as exc:
            errors.append(f"ROS Python modules are unavailable: {exc}. Use Ubuntu's /usr/bin/python3.")
        else:
            required = PACKAGES + (() if args.no_rviz else ("rviz2",))
            missing = []
            for package in required:
                try:
                    get_package_share_directory(package)
                except PackageNotFoundError:
                    missing.append(package)
            if missing:
                errors.append("Missing ROS packages: " + ", ".join(missing))
        if shutil.which("ros2") is None:
            errors.append("ros2 is unavailable in this terminal.")
    if errors:
        for error in errors:
            print("ERROR: " + error, file=sys.stderr)
        print("\nInstall any missing dependencies in Ubuntu:\n"
              "sudo apt update\n"
              "sudo apt install ros-jazzy-ros-gz ros-jazzy-navigation2 "
              "ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox ros-jazzy-rviz2 "
              "ros-jazzy-robot-state-publisher python3-numpy", file=sys.stderr)
        return 1
    print("Runtime dependencies and bundled scene resources are available.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
