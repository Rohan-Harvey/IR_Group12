"""Run the sensor-based parking mission in ROS 2 Jazzy / Gazebo Harmonic."""

import importlib.util
import json
import os
from pathlib import Path
import sys
import tempfile
import xml.etree.ElementTree as ET

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler)
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def setup(context):
    project = Path(__file__).resolve().parents[1]
    spec = importlib.util.spec_from_file_location("parking_navigation_config",
                                                project/"scripts/navigation_config.py")
    nav_config = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(nav_config)
    patrol = json.loads((project/"config/patrol.json").read_text())
    robot = json.loads((project/"config/limo.json").read_text())
    settings = json.loads((project/"config/parking.json").read_text())
    params = nav_config.parameters(project, patrol, robot)
    planner_config = params["planner_server"]["ros__parameters"]["GridBased"]
    planner_config.update(motion_model_for_search="REEDS_SHEPP", smooth_path=False,
                          tolerance=0.13, minimum_turning_radius=settings["planning_radius_m"],
                          reverse_penalty=1.5, change_penalty=0.12, max_planning_time=3.0,
                          downsample_costmap=False, lookup_table_size=8.0)
    # The sweep audit uses its own full-resolution observed map. Keep the
    # planning obstacle footprint exact, with padding applied by the costmap.
    nav = {name: params[name] for name in ("planner_server", "global_costmap")}
    with tempfile.NamedTemporaryFile(mode="w", suffix=".yaml", prefix="parking_nav_",
                                     delete=False, encoding="utf-8") as file:
        json.dump(nav, file, indent=2)
        parameter_path = Path(file.name)
    cleanup = [parameter_path]
    scenario = LaunchConfiguration("scenario").perform(context)
    filename = "parking_lot.sdf" if scenario == "overview" else f"case_{scenario.lower()}.sdf"
    scene = project/"worlds"/filename
    if LaunchConfiguration("pedestrians").perform(context) == "false":
        tree = ET.parse(scene)
        world = tree.getroot().find("world")
        for actor in world.findall("actor"):
            world.remove(actor)
        with tempfile.NamedTemporaryFile(suffix=".sdf", prefix="parking_static_", delete=False) as file:
            tree.write(file, encoding="utf-8", xml_declaration=True)
            scene = Path(file.name)
        cleanup.append(scene)
    simulator = ["gz", "sim", "-r", "-v", "2"]
    if LaunchConfiguration("gui").perform(context) == "false":
        simulator.extend(["-s", "--headless-rendering"])
    resources = str(project/"models")+os.pathsep+os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    sim = ExecuteProcess(cmd=[*simulator, str(scene)], output="screen",
                         additional_env={"GZ_SIM_RESOURCE_PATH": resources})
    planner = Node(package="nav2_planner", executable="planner_server", name="planner_server",
                   parameters=[str(parameter_path)], output="screen")
    lifecycle = Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
                     name="parking_navigation_lifecycle", output="screen",
                     parameters=[{"use_sim_time": True, "autostart": True,
                                  "node_names": ["planner_server"], "bond_timeout": 15.0}])
    mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(project/"launch/mapping.launch.py")),
        launch_arguments={"rviz": LaunchConfiguration("rviz")}.items())
    def on_shutdown(event, launch_context):
        for path in cleanup:
            path.unlink(missing_ok=True)
    def monitored(process, name):
        def terminate(event, launch_context):
            if launch_context.is_shutdown:
                return []
            return [EmitEvent(event=Shutdown(reason=f"{name} exited"))]
        return [RegisterEventHandler(OnProcessExit(target_action=process, on_exit=terminate)), process]
    actions = [RegisterEventHandler(OnShutdown(on_shutdown=on_shutdown)),
               *monitored(sim, "Gazebo"), mapping, *monitored(planner, "Parking planner"), lifecycle]
    if LaunchConfiguration("autostart").perform(context) == "true":
        guard = ExecuteProcess(cmd=[sys.executable, str(project/"scripts/parking_guard.py")], output="screen")
        mission = Node(executable=sys.executable, package=None, output="screen",
                       arguments=[str(project/"scripts/parking_mission.py")],
                       parameters=[{"output_root": ParameterValue(LaunchConfiguration("output_root"), value_type=str),
                                    "entrance_start": scenario == "overview"}])
        actions.extend([*monitored(guard, "Motion guard"), *monitored(mission, "Parking mission")])
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("rviz", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("autostart", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("pedestrians", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("scenario", default_value="overview",
                              choices=["overview", "S1", "S2", "S3", "S4", "S5", "S6"]),
        DeclareLaunchArgument("output_root", default_value=""),
        OpaqueFunction(function=setup),
    ])
