"""Launch the unmarked world with random parked-car angles and optional ROS clock."""

from pathlib import Path
import shlex
import json
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_world(context):
    share = Path(get_package_share_directory("parking_lot_sim"))
    scenario=LaunchConfiguration("scenario").perform(context)
    filename="parking_lot.sdf" if scenario=="overview" else f"case_{scenario.lower()}.sdf"
    gz_arguments = ["-r", "-v", "2"]
    if LaunchConfiguration("gui").perform(context).lower() not in ("true", "1", "yes"):
        gz_arguments.append("-s")
    gz_arguments.append(str(share / "worlds" / filename))
    gz_launch = Path(get_package_share_directory("ros_gz_sim")) / "launch" / "gz_sim.launch.py"
    actions = [SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", str(share / "models") + os.pathsep
                                      + os.environ.get("GZ_SIM_RESOURCE_PATH", "")),
        IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(gz_launch)),
        launch_arguments={"gz_args": shlex.join(gz_arguments),
                          "on_exit_shutdown": "True"}.items(),
    )]
    config = json.loads((share / "config" / "parking_lot.json").read_text())
    if config.get("include_robot", True):
        actions.extend([
            Node(package="ros_gz_bridge", executable="parameter_bridge", name="parking_robot_bridge",
                 parameters=[{"config_file": str(share / "config" / "robot_bridge.yaml")}], output="screen"),
            Node(package="robot_state_publisher", executable="robot_state_publisher",
                 parameters=[{"robot_description": (share / "urdf" / "parking_robot.urdf").read_text(),
                              "use_sim_time": True}], output="screen"),
        ])
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("world", default_value="unmarked", choices=["unmarked"],
                              description="Unmarked lot with randomly angled parked cars"),
        DeclareLaunchArgument("scenario",default_value="overview",choices=["overview","S1","S2","S3","S4","S5","S6"],
                              description="All cases with an entrance start, or start near one selected case"),
        DeclareLaunchArgument("gui", default_value="true", choices=["true", "false"],
                              description="Set false to run the server only"),
        DeclareLaunchArgument("bridge_clock", default_value="true", choices=["true", "false"],
                              description="Bridge Gazebo simulation time onto ROS /clock"),
        OpaqueFunction(function=launch_world),
        Node(package="ros_gz_bridge", executable="parameter_bridge", name="parking_lot_clock",
             arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
             output="screen", condition=IfCondition(LaunchConfiguration("bridge_clock"))),
    ])
