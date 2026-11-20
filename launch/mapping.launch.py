"""Attach laser mapping to an already running parking-lot Gazebo world.

This file also runs directly from the source tree, without a colcon build.
"""

from pathlib import Path
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    project = Path(__file__).resolve().parents[1]
    slam_launch = (
        Path(get_package_share_directory("slam_toolbox"))
        / "launch"
        / "online_async_launch.py"
    )
    # Resolve meshes without requiring this project to be installed as a package.
    description = (project / "urdf" / "parking_robot.urdf").read_text(
        encoding="utf-8"
    ).replace("package://parking_lot_sim/", project.as_uri() + "/")

    return LaunchDescription([
        DeclareLaunchArgument(
            "bridge", default_value="true", choices=["true", "false"],
            description="Bridge the existing Gazebo world, including its clock",
        ),
        DeclareLaunchArgument(
            "state_publisher", default_value="true", choices=["true", "false"],
            description="Publish the existing robot's body and sensor transforms",
        ),
        DeclareLaunchArgument(
            "rviz", default_value="true", choices=["true", "false"],
            description="Open the map and live laser display",
        ),
        LogInfo(msg="Attaching mapping to the existing Gazebo world; no world is spawned."),
        Node(
            package="ros_gz_bridge", executable="parameter_bridge",
            name="parking_mapping_bridge", output="screen",
            parameters=[{
                "config_file": str(project / "config" / "mapping_bridge.yaml"),
                "use_sim_time": True,
            }],
            condition=IfCondition(LaunchConfiguration("bridge")),
        ),
        Node(
            package="robot_state_publisher", executable="robot_state_publisher",
            name="parking_mapping_state_publisher", output="screen",
            parameters=[{"robot_description": description, "use_sim_time": True}],
            condition=IfCondition(LaunchConfiguration("state_publisher")),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(slam_launch)),
            launch_arguments={
                "use_sim_time": "true",
                "slam_params_file": str(project / "config" / "slam.yaml"),
                "autostart": "true",
                "use_lifecycle_manager": "false",
            }.items(),
        ),
        ExecuteProcess(cmd=[sys.executable, str(project/"scripts/map_support.py")],
                       output="screen"),
        Node(
            package="rviz2", executable="rviz2", name="parking_mapping_view",
            output="screen", parameters=[{"use_sim_time": True}],
            arguments=["-d", str(project / "config" / "mapping.rviz")],
            condition=IfCondition(LaunchConfiguration("rviz")),
        ),
    ])
