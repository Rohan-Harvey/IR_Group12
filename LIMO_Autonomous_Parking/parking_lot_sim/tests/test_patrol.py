"""Offline geometric and command-safety tests; not a Nav2/Gazebo runtime test."""

import json
import math
from pathlib import Path
import sys
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/"scripts"))
from patrol_route import (build_route, command_is_fresh, goal_index, limit_command, nearest_progress,
                          rear_point, route_lengths, transform, wrap)
from scene_geometry import overlaps, rectangle
from navigation_config import parameters


class PatrolTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config = json.loads((ROOT/"config/patrol.json").read_text())
        cls.robot = json.loads((ROOT/"config/limo.json").read_text())
        cls.legs = build_route(cls.config, cls.robot["wheelbase_m"])

    def test_closed_mission_and_forward_reverse_join(self):
        self.assertEqual([name for name, _ in self.legs], ["entry", "loop", "return"])
        for (_, a), (_, b) in zip(self.legs, self.legs[1:]):
            self.assertAlmostEqual(a[-1].x, b[0].x)
            self.assertAlmostEqual(a[-1].y, b[0].y)
            self.assertAlmostEqual(wrap(a[-1].yaw-b[0].yaw), 0.0)
        end = self.legs[-1][1][-1]
        self.assertAlmostEqual(end.x, 0.0)
        self.assertAlmostEqual(end.y, 0.0)
        self.assertAlmostEqual(end.yaw, 0.0)
        self.assertTrue(all(p.direction == -1 for p in self.legs[-1][1]))

    def test_ackermann_curvature_and_rear_axle_motion(self):
        maximum = math.sin(self.robot["steering_limit_rad"])/self.robot["wheelbase_m"]
        for _, points in self.legs:
            for a, b in zip(points, points[1:]):
                self.assertLessEqual(abs(a.curvature), maximum)
                ra, rb = rear_point(a, self.robot["wheelbase_m"]), rear_point(b, self.robot["wheelbase_m"])
                ds = math.hypot(rb.x-ra.x, rb.y-ra.y)
                self.assertLess(ds, 0.16)
                if ds > 1e-7:
                    tangent = math.atan2(rb.y-ra.y, rb.x-ra.x)
                    expected = a.yaw+(math.pi if a.direction < 0 else 0)
                    self.assertLess(abs(wrap(tangent-expected)), 0.025)
                    self.assertLess(abs(wrap(b.yaw-a.yaw))/ds, maximum+0.001)
        self.assertGreaterEqual(self.config["planner_min_turning_radius_m"], 1/maximum)

    def test_complete_padded_footprint_clear_of_scene(self):
        truth = json.loads((ROOT/"config/scene_ground_truth.json").read_text())
        obstacles = [(b["id"], rectangle(b["car"]["position_m"],
                      *b["car"]["envelope_m"][:2], b["car"]["yaw_rad"]))
                     for b in truth["bays"] if b["occupied"]]
        for name, xy, length, width in [
            ("west north wall", (-24.15, 7.1), .3, 25.8),
            ("west south wall", (-24.15, -16.1), .3, 7.8),
            ("east wall", (24.15, 0), .3, 40),
            ("north wall", (0, 20.15), 48.6, .3),
            ("south wall", (0, -20.15), 48.6, .3),
            ("island", (0, 0), 33.6, 1.2),
            ("north curb", (0, 17.47), 43, .15),
            ("south curb", (0, -17.47), 43, .15),
            ("S5 rear wall", (14, -17.7), 11, .2),
            ("planter", (6.8, 16), .7, 1.8),
        ]:
            obstacles.append((name, rectangle(xy, length, width, 0)))
        origin = [*self.config["home_world_pose_m_deg"][:2],
                  math.radians(self.config["home_world_pose_m_deg"][2])]
        for leg, points in self.legs:
            for i, point in enumerate(points):
                p = transform(point, origin)
                shift = self.robot["footprint_center_offset_m"][0]
                shape = rectangle((p.x+shift*math.cos(p.yaw), p.y+shift*math.sin(p.yaw)),
                                  self.robot["footprint_size_m"][0]+.24,
                                  self.robot["steered_envelope_width_m"]+.24, p.yaw)
                for name, obstacle in obstacles:
                    self.assertFalse(overlaps(shape, obstacle), (leg, i, name))

    def test_loop_progress_does_not_jump_to_end_at_start(self):
        points = self.legs[1][1]
        start = points[0]
        self.assertEqual(nearest_progress(points, (start.x, start.y, start.yaw)), 0)
        i = 0
        for p in points:
            i = nearest_progress(points, (p.x, p.y, p.yaw), i)
        self.assertEqual(i, len(points)-1)

    def test_second_lap_inspections_have_five_numbered_stops(self):
        inspections = self.config["parking_inspections"]
        self.assertEqual([item["id"] for item in inspections], [1, 2, 3, 4, 5])
        self.assertEqual([item["id"] for item in inspections if item.get("observation_probe")], [4])
        loop = [rear_point(point, self.robot["wheelbase_m"]) for point in self.legs[1][1]]
        lengths = route_lengths(loop)
        previous = 0
        for item in inspections:
            anchor = goal_index(lengths, 0, item["aisle_distance_m"])
            stop = goal_index(lengths, 0, item["check_distance_m"])
            self.assertLess(previous, anchor)
            self.assertLessEqual(anchor, stop)
            self.assertLess(stop, len(loop)-7)
            previous = stop

    def test_speed_and_steering_guard(self):
        for v in (-5.0, -0.4, 0.0, 0.4, 5.0):
            for w in (-10.0, 0.0, 10.0):
                a, b = limit_command(v, w, .75, .4, .22)
                self.assertLessEqual(a, .75)
                self.assertGreaterEqual(a, -.4)
                self.assertLessEqual(abs(b), abs(a)*.22+1e-12)
        self.assertEqual(limit_command(math.nan, 1, .75, .4, .22), (0.0, 0.0))
        self.assertEqual(limit_command(0, 1, .75, .4, .22), (0.0, 0.0))

    def test_nav2_footprint_matches_rear_axle_reference(self):
        params = parameters(ROOT, self.config, self.robot)
        cm = params["local_costmap"]["local_costmap"]["ros__parameters"]
        self.assertEqual(cm["robot_base_frame"], "rear_axle")
        footprint = json.loads(cm["footprint"])
        cx = sum(p[0] for p in footprint)/4-self.robot["wheelbase_m"]/2
        self.assertAlmostEqual(cx, self.robot["footprint_center_offset_m"][0])
        urdf = ET.parse(ROOT/"urdf/parking_robot.urdf").getroot()
        joint = urdf.find("joint[@name='rear_axle_fixed']")
        x = float(joint.find("origin").get("xyz").split()[0])
        self.assertEqual(joint.find("parent").get("link"), "rear_axle")
        self.assertEqual(joint.find("child").get("link"), "base_footprint")
        self.assertAlmostEqual(x, self.robot["wheelbase_m"]/2)
        for world in (ROOT/"worlds").glob("*.sdf"):
            drive = ET.parse(world).getroot().find(".//plugin[@name='gz::sim::systems::AckermannSteering']")
            self.assertEqual(drive.findtext("child_frame_id"), "rear_axle")
            self.assertEqual(drive.findtext("left_joint"), "rear_left_wheel_joint")
            self.assertEqual(drive.findtext("right_joint"), "rear_right_wheel_joint")
        bt = ET.parse(ROOT/"config/navigate.xml").getroot()
        self.assertFalse(bt.findall(".//Spin"))

    def test_stale_data_or_clock_reset_holds_the_robot(self):
        self.assertTrue(command_is_fresh(True, 10.0, 9.9, 9.9, 9.9, 9.9))
        self.assertFalse(command_is_fresh(False, 10.0, 9.9, 9.9, 9.9, 9.9))
        self.assertFalse(command_is_fresh(True, 10.0, 9.0, 9.9, 9.9, 9.9))
        self.assertFalse(command_is_fresh(True, 10.0, 9.9, 9.0, 9.9, 9.9))
        self.assertFalse(command_is_fresh(True, 10.0, 9.9, 9.9, 9.0, 9.9))
        self.assertFalse(command_is_fresh(True, 10.0, 9.9, 9.9, 9.9, 7.0))
        self.assertFalse(command_is_fresh(True, 0.0, 9.9, 9.9, 9.9, 9.9))
        self.assertFalse(command_is_fresh(True, 0.0, -math.inf, -math.inf, -math.inf, -math.inf))


if __name__ == "__main__":
    unittest.main()
