"""Driving direction contract across mission, planner, controller and guard."""

import json
import math
from pathlib import Path
from types import SimpleNamespace as NS
import unittest
import xml.etree.ElementTree as ET

from test_patrol_startup import load_node_class, Twist
from navigation_config import parameters

ROOT = Path(__file__).resolve().parents[1]


class DrivingDirectionTests(unittest.TestCase):
    def guard(self, direction):
        node = load_node_class("drive_guard.py", "DriveGuard")()
        node.direction_received(NS(data=direction))
        node.enable_received(NS(data=True))
        node.scan_time = node.odom_time = node.now()
        node.previous = node.now()-0.1
        return node

    def command(self, node, velocity, angular=0.03):
        command = Twist()
        command.linear.x, command.angular.z = velocity, angular
        node.command_received(command)
        node.tick()
        return node.publisher.messages[-1]

    def test_negative_command_is_stopped_during_forward_survey(self):
        node = self.guard(1)
        output = self.command(node, -0.4)
        self.assertEqual(output.linear.x, 0.0)
        self.assertEqual(output.angular.z, 0.0)
        self.assertIn("wrong driving direction", node.logs[-1])

    def test_forward_command_preserves_sign_and_curvature(self):
        node = self.guard(1)
        output = self.command(node, 0.4)
        self.assertGreater(output.linear.x, 0.0)
        self.assertAlmostEqual(output.angular.z/output.linear.x, 0.03/0.4)

    def test_reverse_is_available_in_return_phase(self):
        node = self.guard(-1)
        output = self.command(node, -0.4)
        self.assertLess(output.linear.x, 0.0)
        self.assertAlmostEqual(output.angular.z/output.linear.x, 0.03/-0.4)

    def test_forward_is_stopped_in_reverse_return(self):
        node = self.guard(-1)
        self.assertEqual(self.command(node, 0.4).linear.x, 0.0)

    def test_phase_change_discards_old_commands(self):
        node = self.guard(1)
        self.assertGreater(self.command(node, 0.4).linear.x, 0.0)
        node.direction_received(NS(data=-1))
        node.tick()
        self.assertEqual(node.publisher.messages[-1].linear.x, 0.0)
        self.assertFalse(math.isfinite(node.raw_time))
        self.assertEqual(node.output_v, 0.0)

    def test_missing_or_stale_direction_heartbeat_holds_vehicle(self):
        for direction in (0, 1):
            node = self.guard(direction)
            if direction:
                node.direction_time = node.now()-3.0
            self.assertEqual(self.command(node, 0.4).linear.x, 0.0)

    def test_default_navigation_disallows_reverse_and_gradient_is_nonzero(self):
        node = load_node_class("patrol.py", "Patrol")()
        config = parameters(ROOT, node.config, node.robot)
        self.assertEqual(config["planner_server"]["ros__parameters"]["GridBased"]
                         ["motion_model_for_search"], "DUBIN")
        controller = config["controller_server"]["ros__parameters"]["FollowPath"]
        self.assertFalse(controller["allow_reversing"])
        for name in ("local_costmap", "global_costmap"):
            costmap = config[name][name]["ros__parameters"]
            footprint = json.loads(costmap["footprint"])
            padding = costmap["footprint_padding"]
            xs = [x+math.copysign(padding, x) for x, y in footprint]
            ys = [y+math.copysign(padding, y) for x, y in footprint]
            circum = max(math.hypot(x, y) for x, y in zip(xs, ys))
            # For this axis-aligned rectangle the origin is inside it and the
            # closest side determines Nav2's inscribed radius.
            inscribed = min(min(abs(x) for x in xs), min(abs(y) for y in ys))
            layer = costmap["inflation_layer"]
            self.assertGreater(layer["inflation_radius"], circum)
            cost_at_circum = int(252*math.exp(-layer["cost_scaling_factor"]*(circum-inscribed)))
            self.assertGreater(cost_at_circum, 0)
            self.assertEqual(controller["inflation_cost_scaling_factor"], layer["cost_scaling_factor"])

    def test_wheel_axes_encode_positive_roll_towards_model_x(self):
        for world in (ROOT/"worlds").glob("*.sdf"):
            robot = ET.parse(world).getroot().find("world/model[@name='parking_robot']")
            for end in ("front", "rear"):
                for side in ("left", "right"):
                    joint = robot.find(f"joint[@name='{end}_{side}_wheel_joint']")
                    axis = joint.find("axis/xyz")
                    self.assertEqual(axis.text, "0 1 0")
                    self.assertEqual(axis.get("expressed_in"),
                                     f"front_{side}_knuckle" if end == "front" else "__model__")
                    # omega=(0,+1,0); contact r=(0,0,-R) gives surface
                    # velocity (-R,0,0), hence no-slip body velocity (+R,0,0).
                    self.assertGreater(float(node_radius(robot)), 0)


def node_radius(robot):
    return robot.findtext("plugin[@name='gz::sim::systems::AckermannSteering']/wheel_radius")


class ModeHandshakeTests(unittest.TestCase):
    def node(self):
        node = load_node_class("patrol.py", "Patrol")()
        node.planner_parameters.ready = node.controller_parameters.ready = True
        return node

    def reply(self, client, successful=True):
        client.futures[-1].value = NS(results=[NS(successful=successful, reason="test rejection")])
        client.futures[-1].complete = True

    def test_forward_mode_requires_both_parameter_acknowledgements(self):
        node = self.node()
        self.assertFalse(node.ensure_direction(1))
        self.assertEqual(node.planner_parameters.requests[-1].parameters[0].value, "DUBIN")
        self.assertFalse(node.controller_parameters.requests[-1].parameters[0].value)
        self.reply(node.planner_parameters)
        self.assertFalse(node.ensure_direction(1))
        self.reply(node.controller_parameters)
        self.assertTrue(node.ensure_direction(1))
        node.enable(True)
        self.assertTrue(node.enabled_pub.messages[-1].data)
        self.assertEqual(node.direction_pub.messages[-1].data, 1)

    def test_mode_change_waits_for_stop_and_enables_reverse_only_for_return(self):
        node = self.node()
        node.mode_direction = 1
        node.leg = 2
        node.speed = 0.2
        self.assertFalse(node.ensure_direction(-1))
        self.assertEqual(node.planner_parameters.requests, [])
        node.speed = 0.0
        self.assertFalse(node.ensure_direction(-1))
        self.assertEqual(node.planner_parameters.requests[-1].parameters[0].value, "REEDS_SHEPP")
        self.assertTrue(node.controller_parameters.requests[-1].parameters[0].value)
        node.enable(True)
        self.assertFalse(node.enabled_pub.messages[-1].data)
        self.reply(node.planner_parameters)
        self.reply(node.controller_parameters)
        self.assertTrue(node.ensure_direction(-1))
        node.enable(True)
        self.assertEqual(node.direction_pub.messages[-1].data, -1)

    def test_rejected_parameter_change_never_enables_driving(self):
        node = self.node()
        node.ensure_direction(1)
        self.reply(node.planner_parameters, False)
        self.reply(node.controller_parameters)
        self.assertFalse(node.ensure_direction(1))
        node.enable(True)
        self.assertFalse(node.enabled_pub.messages[-1].data)
        self.assertIsNotNone(node.mode_error)

    def test_timed_out_change_remains_held_despite_late_replies(self):
        node = self.node()
        node.ensure_direction(1)
        node.now = lambda: 12.0
        self.assertFalse(node.ensure_direction(1))
        self.reply(node.planner_parameters)
        self.reply(node.controller_parameters)
        self.assertFalse(node.ensure_direction(1))
        node.enable(True)
        self.assertFalse(node.enabled_pub.messages[-1].data)

    def test_cannot_authorize_driving_before_any_mode_confirmation(self):
        node = self.node()
        node.enable(True)
        self.assertFalse(node.enabled_pub.messages[-1].data)
        self.assertEqual(node.direction_pub.messages[-1].data, 0)


if __name__ == "__main__":
    unittest.main()
