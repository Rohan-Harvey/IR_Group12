"""Start the entrance world, SLAM and a sensor-guided autonomous survey."""

import importlib.util
import json
import os
from pathlib import Path
import sys
import tempfile

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            LogInfo, RegisterEventHandler, EmitEvent)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    project = Path(__file__).resolve().parents[1]
    spec = importlib.util.spec_from_file_location("parking_navigation_config",
                                                project/"scripts/navigation_config.py")
    config_module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(config_module)
    patrol = json.loads((project/"config/patrol.json").read_text())
    robot = json.loads((project/"config/limo.json").read_text())
    # JSON is valid YAML; ROS's parameter parser reads this temporary YAML file.
    with tempfile.NamedTemporaryFile(mode="w", suffix=".yaml", prefix="parking_nav_",
                                     delete=False, encoding="utf-8") as handle:
        json.dump(config_module.parameters(project, patrol, robot), handle, indent=2)
        nav_file = handle.name
    environment = dict(os.environ)
    environment["GZ_SIM_RESOURCE_PATH"] = str(project/"models")+os.pathsep+environment.get("GZ_SIM_RESOURCE_PATH", "")
    world = ExecuteProcess(cmd=["gz", "sim", "-r", "-v", "2", str(project/"worlds/parking_lot.sdf")],
                           output="screen", additional_env=environment)
    servers = [
        ("nav2_controller", "controller_server", "controller_server"),
        ("nav2_planner", "planner_server", "planner_server"),
        ("nav2_behaviors", "behavior_server", "behavior_server"),
        ("nav2_bt_navigator", "bt_navigator", "bt_navigator"),
    ]

    def stop_on_exit(process, label):
        return RegisterEventHandler(OnProcessExit(target_action=process,
            on_exit=[EmitEvent(event=Shutdown(reason=f"{label} exited; stopping the simulation"))]))

    actions = [
        DeclareLaunchArgument("rviz", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("autostart", default_value="true", choices=["true", "false"],
                              description="Start the survey automatically after sensors and Nav2 are ready"),
        LogInfo(msg="Autonomous mapping: entrance start, survey loop, reverse return, map save."),
        world,
        RegisterEventHandler(OnProcessExit(target_action=world,
            on_exit=[EmitEvent(event=Shutdown(reason="Gazebo closed"))])),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(project/"launch/mapping.launch.py")),
            launch_arguments={"rviz": LaunchConfiguration("rviz")}.items()),
    ]
    for package, executable, name in servers:
        server = Node(package=package, executable=executable, name=name,
                      parameters=[nav_file], output="screen",
                      remappings=[("cmd_vel", "/nav_cmd_vel")])
        actions.extend([stop_on_exit(server, name), server])
    guard = ExecuteProcess(cmd=[sys.executable, str(project/"scripts/drive_guard.py")],
                           output="screen")
    mission = ExecuteProcess(cmd=[sys.executable, str(project/"scripts/patrol.py")],
                             output="screen", condition=IfCondition(LaunchConfiguration("autostart")))
    actions.extend([
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
             name="parking_navigation_lifecycle", output="screen",
             parameters=[{"use_sim_time": True, "autostart": True,
                          "node_names": [name for _, _, name in servers],
                          "bond_timeout": 15.0}]),
        stop_on_exit(guard, "Velocity guard"), guard,
        stop_on_exit(mission, "Survey mission"), mission,
        RegisterEventHandler(OnShutdown(on_shutdown=lambda event, context: Path(nav_file).unlink(missing_ok=True))),
    ])
    return LaunchDescription(actions)
