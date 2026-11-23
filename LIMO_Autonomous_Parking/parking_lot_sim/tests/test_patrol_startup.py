"""Startup regression using ROS-shaped stubs, without claiming a live ROS test."""

import ast
import copy
import json
import math
from pathlib import Path
import sys
import subprocess
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/"scripts"))
import patrol_route
import map_grid


class Message:
    def __init__(self, **kwargs):
        self.__dict__.update(kwargs)


class Twist(Message):
    def __init__(self):
        self.linear = SimpleNamespace(x=0.0, y=0.0, z=0.0)
        self.angular = SimpleNamespace(x=0.0, y=0.0, z=0.0)


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class ParameterStub:
    def __init__(self, name, value=None, **kwargs):
        self.name, self.value = name, value

    def to_parameter_msg(self):
        return self


class Future:
    def __init__(self, value=None, done=False):
        self.value, self.complete = value, done

    def done(self):
        return self.complete

    def result(self):
        return self.value


class ServiceClient:
    def __init__(self):
        self.ready = False
        self.requests, self.futures = [], []

    def service_is_ready(self):
        return self.ready

    def call_async(self, request):
        self.requests.append(request)
        future = Future()
        self.futures.append(future)
        return future


class RosNodeStub:
    """Keep Node.handle protected, as it is in rclpy.Node."""

    @property
    def handle(self):
        return "underlying_ros_node"

    @handle.setter
    def handle(self, value):
        raise AttributeError("handle cannot be modified after node creation")

    def __init__(self, *args, **kwargs):
        self.logs = []
        self.subscriptions = []

    def get_clock(self):
        return SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=1_000_000_000))

    def get_logger(self):
        return SimpleNamespace(info=self.logs.append, error=self.logs.append)

    def create_publisher(self, *args, **kwargs):
        return Publisher()

    def create_subscription(self, *args, **kwargs):
        self.subscriptions.append(args)
        return object()

    def create_timer(self, *args, **kwargs):
        return object()

    def create_client(self, *args, **kwargs):
        return ServiceClient()


def load_node_class(filename, class_name):
    path = ROOT/"scripts"/filename
    tree = ast.parse(path.read_text(encoding="utf-8"))
    node = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == class_name)
    namespace = dict(vars(patrol_route))
    namespace.update(vars(map_grid))
    namespace.update({
        "__file__": str(path), "__name__": "node_startup_regression",
        "Node": RosNodeStub, "Parameter": ParameterStub,
        "Path": Path, "json": json, "math": math, "subprocess": subprocess, "time": time,
        "copy": copy,
        "QoSProfile": lambda **k: object(),
        "DurabilityPolicy": SimpleNamespace(TRANSIENT_LOCAL=1),
        "ReliabilityPolicy": SimpleNamespace(RELIABLE=1),
        "Buffer": lambda: object(), "TransformListener": lambda *a: object(),
        "ActionClient": lambda *a: SimpleNamespace(server_is_ready=lambda: False),
        "NavigateToPose": object, "RosPath": Message, "Bool": Message,
        "String": Message, "Int8": Message, "OccupancyGrid": Message, "Odometry": Message,
        "SetParameters": SimpleNamespace(Request=Message),
        "LaserScan": Message, "Twist": Twist, "qos_profile_sensor_data": object(),
        "GoalStatus": SimpleNamespace(STATUS_SUCCEEDED=4),
        "PointCloud2": Message,
        "PointField": type("PointField", (Message,), {"FLOAT32": 7, "UINT32": 6}),
    })
    helpers = [n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name != "main"]
    exec(compile(ast.Module(body=helpers+[node], type_ignores=[]), str(path), "exec"), namespace)
    return namespace[class_name]


class StartupTests(unittest.TestCase):
    def test_patrol_constructs_without_overwriting_node_handle(self):
        node = load_node_class("patrol.py", "Patrol")()
        self.assertEqual(node.handle, "underlying_ros_node")
        self.assertIsNone(node.goal_handle)
        node.tick()
        self.assertFalse(node.enabled_pub.messages[-1].data)

    def test_late_accepted_goal_is_cancelled_after_mission_changes(self):
        node = load_node_class("patrol.py", "Patrol")()
        cancelled = []
        goal = SimpleNamespace(accepted=True, cancel_goal_async=lambda: cancelled.append(True))
        old_generation = node.generation
        node.cancel_navigation()
        node.goal_response(SimpleNamespace(result=lambda: goal), old_generation)
        self.assertEqual(cancelled, [True])
        self.assertIsNone(node.goal_handle)
        self.assertFalse(node.active)

    def test_rejected_replacement_cancels_previous_navigation(self):
        node = load_node_class("patrol.py", "Patrol")()
        cancelled = []
        node.goal_handle = SimpleNamespace(cancel_goal_async=lambda: cancelled.append(True))
        node.active = node.sending = True
        node.goal_response(SimpleNamespace(result=lambda: SimpleNamespace(accepted=False)),
                           node.generation)
        self.assertEqual(cancelled, [True])
        self.assertFalse(node.active)
        self.assertFalse(node.sending)
        self.assertFalse(node.enabled_pub.messages[-1].data)
        self.assertGreater(node.hold_until, node.now())

    def test_previous_goal_result_does_not_stop_a_newer_goal(self):
        node = load_node_class("patrol.py", "Patrol")()
        node.generation = 2
        node.active = True
        node.goal_handle = object()
        goal = node.goal_handle
        node.goal_result(SimpleNamespace(result=lambda: SimpleNamespace(status=4)), 1)
        self.assertTrue(node.active)
        self.assertIs(node.goal_handle, goal)

    def test_missing_goal_response_times_out_and_invalidates_pending_reply(self):
        node = load_node_class("patrol.py", "Patrol")()
        node.map = SimpleNamespace(info=SimpleNamespace(width=1))
        node.map_time = node.now()
        node.client = SimpleNamespace(server_is_ready=lambda: True)
        node.sending = True
        node.last_send = node.now()-11.0
        node.tick()
        self.assertFalse(node.sending)
        self.assertEqual(node.generation, 1)
        self.assertFalse(node.enabled_pub.messages[-1].data)

    def test_final_target_stays_enabled_outside_lateral_goal_tolerance(self):
        node = load_node_class("patrol.py", "Patrol")()
        node.map = SimpleNamespace(info=SimpleNamespace(width=1))
        node.map_time = node.now()
        node.client = SimpleNamespace(server_is_ready=lambda: True)
        node.home = (0.0, 0.0, 0.0)
        node.leg = 2
        points = node.legs[node.leg][1]
        node.progress = node.last_target = len(points)-1
        end = points[-1]
        node.robot_pose = lambda: (end.x, end.y+0.5, end.yaw)
        node.in_map = lambda point: True
        node.active = True
        node.mode_direction = -1
        node.tick()
        self.assertTrue(node.enabled_pub.messages[-1].data)
        self.assertFalse(node.finished)

    def test_stale_map_holds_an_active_mission(self):
        node = load_node_class("patrol.py", "Patrol")()
        node.map = SimpleNamespace(info=SimpleNamespace(width=1))
        node.map_time = node.now()-6.0
        node.active = True
        node.tick()
        self.assertFalse(node.enabled_pub.messages[-1].data)
        self.assertIn("map is stale", node.logs[-1])

    def test_map_saver_start_failure_is_reported_once(self):
        node = load_node_class("patrol.py", "Patrol")()
        node.finished = True
        node.save_at = 0.0
        with tempfile.TemporaryDirectory() as temporary:
            node.root = Path(temporary)
            with patch.object(subprocess, "Popen", side_effect=OSError("missing ros2")) as start:
                node.tick()
                node.tick()
                self.assertEqual(start.call_count, 1)
        self.assertTrue(node.save_reported)
        self.assertIn("could not start", node.logs[-1])

    def test_drive_guard_constructs_and_holds_without_sensor_data(self):
        node = load_node_class("drive_guard.py", "DriveGuard")()
        self.assertEqual(node.handle, "underlying_ros_node")
        node.tick()
        command = node.publisher.messages[-1]
        self.assertEqual(command.linear.x, 0.0)
        self.assertEqual(command.angular.z, 0.0)


if __name__ == "__main__":
    unittest.main()
