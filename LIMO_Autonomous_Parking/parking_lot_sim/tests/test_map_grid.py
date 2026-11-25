"""Regression for the observed entrance deadlock; no live ROS graph required."""

from array import array
import math
from pathlib import Path
import struct
import sys
from types import SimpleNamespace as NS
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/"scripts"))
from map_grid import pad_grid, padded_origin, preview_points
from navigation_config import parameters
from test_patrol_startup import load_node_class


def raw_grid(width=385, height=478, resolution=0.05, x=0.9, y=-0.05, angle=0.0):
    return NS(header=NS(frame_id="map", stamp=NS(sec=1, nanosec=0)),
              info=NS(width=width, height=height, resolution=resolution,
                      origin=NS(position=NS(x=x, y=y, z=0.0),
                                orientation=NS(x=0.0, y=0.0, z=math.sin(angle/2),
                                               w=math.cos(angle/2)))),
              data=array("b", [-1])*(width*height))


class GridTests(unittest.TestCase):
    def test_padding_preserves_all_measured_cells_and_adds_only_unknown(self):
        original = array("b", [0, 100, -1, 23, 87, 0])
        result = pad_grid(3, 2, 0.5, original, 0.8)
        self.assertEqual((result.width, result.height, result.border), (7, 6, 2))
        for row in range(result.height):
            for column in range(result.width):
                value = result.data[row*result.width+column]
                if 2 <= row < 4 and 2 <= column < 5:
                    self.assertEqual(value, original[(row-2)*3+column-2])
                else:
                    self.assertEqual(value, -1)
        self.assertEqual(original.tolist(), [0, 100, -1, 23, 87, 0])

    def test_rotated_origin_keeps_original_cells_at_same_world_positions(self):
        angle = 0.7
        x, y = padded_origin(5.0, -3.0, angle, 40, 0.05)
        for column, row in ((0, 0), (10, 25), (300, 450)):
            old_x, old_y = (column+0.5)*0.05, (row+0.5)*0.05
            new_x, new_y = (column+40+0.5)*0.05, (row+40+0.5)*0.05
            self.assertAlmostEqual(5+math.cos(angle)*old_x-math.sin(angle)*old_y,
                                   x+math.cos(angle)*new_x-math.sin(angle)*new_y)
            self.assertAlmostEqual(-3+math.sin(angle)*old_x+math.cos(angle)*old_y,
                                   y+math.sin(angle)*new_x+math.cos(angle)*new_y)

    def test_invalid_grid_and_negative_padding_are_rejected(self):
        for width, height, resolution, data, padding in (
                (0, 1, .1, [], 4), (2, 2, .1, [0], 4),
                (1, 1, 0, [0], 4), (1, 1, .1, [0], -1)):
            with self.assertRaises(ValueError):
                pad_grid(width, height, resolution, data, padding)

    def test_preview_has_correct_coordinates_colors_and_binary_layout(self):
        data, count = preview_points(3, 1, 1.0, [0, 100, -1], (1.0, 2.0, math.pi/2))
        self.assertEqual(count, 3)
        self.assertEqual(len(data), count*16)
        colors = []
        for i in range(count):
            x, y, z, color = struct.unpack_from("<fffI", data, i*16)
            self.assertAlmostEqual(x, 0.5)
            self.assertAlmostEqual(y, 2.5+i)
            self.assertAlmostEqual(z, -0.02)
            colors.append(color)
        self.assertEqual(colors, [0xFFFFFF, 0x101010, 0x808080])

    def test_preview_is_bounded_even_for_long_narrow_grids(self):
        for width, height in ((385, 478), (1000, 1), (1, 1000)):
            original = [-1]*(width*height)
            data, count = preview_points(width, height, 0.05, original, (0, 0, 0), 100)
            self.assertLessEqual(count, 100)
            self.assertEqual(len(data), 16*count)
            self.assertTrue(all(v == -1 for v in original))


class MapAdapterTests(unittest.TestCase):
    def test_adapter_preserves_raw_map_and_publishes_matching_preview(self):
        node = load_node_class("map_support.py", "MapSupport")()
        message = raw_grid(3, 2, 0.5, angle=0.7)
        message.data = array("b", [0, 100, -1, 0, 0, 100])
        node.map_received(message)
        padded = node.map_pub.messages[-1]
        preview = node.preview_pub.messages[-1]
        self.assertEqual(message.info.width, 3)
        self.assertEqual(message.data.tolist(), [0, 100, -1, 0, 0, 100])
        self.assertIsNot(padded.info, message.info)
        self.assertEqual(padded.info.width, 19)
        self.assertEqual(padded.header.frame_id, "map")
        self.assertEqual(preview.width, 6)
        self.assertEqual(preview.row_step, 96)
        self.assertEqual(len(preview.data), 96)
        self.assertEqual([field.name for field in preview.fields], ["x", "y", "z", "rgb"])

    def test_entrance_deadlock_is_reproduced_and_padded_map_allows_first_goal(self):
        # Match the observed 385 x 478 initial grid, with the vehicle beside
        # its lower boundary. No synthetic free space is inserted.
        raw = raw_grid()
        support = load_node_class("map_support.py", "MapSupport")()
        support.map_received(raw)
        node = load_node_class("patrol.py", "Patrol")()
        node.client = NS(server_is_ready=lambda: True)
        node.mode_direction = 1
        node.home = (0.0, 0.0, 0.0)
        node.robot_pose = lambda: node.home
        targets = []

        def send(point):
            targets.append(point)
            node.active = True

        node.send_target = send
        node.map_received(raw)
        node.tick()
        self.assertEqual(targets, [])
        self.assertFalse(node.enabled_pub.messages[-1].data)
        node.map_received(support.map_pub.messages[-1])
        node.tick()
        self.assertEqual(len(targets), 1)
        self.assertGreater(targets[0].x, 0.0)
        self.assertTrue(node.enabled_pub.messages[-1].data)
        configured = parameters(ROOT, node.config, node.robot)
        costmap = configured["global_costmap"]["global_costmap"]["ros__parameters"]
        self.assertEqual(costmap["static_layer"]["map_topic"], "/navigation_map")
        map_topics = [args[1] for args in node.subscriptions if args[1].endswith("map")]
        self.assertEqual(map_topics, ["/navigation_map"])

    def test_inflation_covers_rear_axle_referenced_footprint(self):
        node = load_node_class("patrol.py", "Patrol")()
        configured = parameters(ROOT, node.config, node.robot)
        costmap = configured["global_costmap"]["global_costmap"]["ros__parameters"]
        robot = node.robot
        cx = robot["footprint_center_offset_m"][0]+robot["wheelbase_m"]/2
        radius = math.hypot(cx+robot["footprint_size_m"][0]/2+0.12,
                            robot["steered_envelope_width_m"]/2+0.12)
        self.assertGreaterEqual(costmap["inflation_layer"]["inflation_radius"], radius)


if __name__ == "__main__":
    unittest.main()
