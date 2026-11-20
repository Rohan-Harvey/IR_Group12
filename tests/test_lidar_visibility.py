"""Prevent the own-body GPU-LiDAR obstacle regression without hiding scenery."""

import json
import math
from pathlib import Path
from types import SimpleNamespace as NS
import unittest
import xml.etree.ElementTree as ET

from generate_world import Builder
from scene_robot import add_robot, LIDAR_VISIBILITY_MASK, ROBOT_VISIBILITY_FLAGS
from diagnose_navigation import scan_summary

ROOT = Path(__file__).resolve().parents[1]


class LidarVisibilityTests(unittest.TestCase):
    def check_robot(self, robot):
        mask = int(robot.findtext("link[@name='lidar_link']/sensor/lidar/visibility_mask"))
        self.assertEqual(mask, LIDAR_VISIBILITY_MASK)
        visuals = robot.findall('.//visual')
        self.assertEqual(len(visuals), 5)
        for visual in visuals:
            flags = int(visual.findtext('visibility_flags'))
            self.assertEqual(flags, ROBOT_VISIBILITY_FLAGS)
            self.assertEqual(flags & mask, 0)
            self.assertNotEqual(flags & 0xFFFFFFFF, 0)  # normal GUI/RGB camera visibility

    def test_every_world_excludes_own_body_but_preserves_external_visuals(self):
        for path in (ROOT/'worlds').glob('*.sdf'):
            world = ET.parse(path).getroot().find('world')
            robot = world.find("model[@name='parking_robot']")
            self.check_robot(robot)
            for model in world.findall('model'):
                if model is robot:
                    continue
                for visual in model.findall('.//visual'):
                    self.assertNotEqual(int(visual.findtext('visibility_flags', '4294967295'))
                                        & LIDAR_VISIBILITY_MASK, 0, (path.name, model.get('name')))
            self.assertEqual([actor.get('name') for actor in world.findall('actor')], ['pedestrian_red_bay'])

    def test_regenerating_robot_keeps_the_lidar_visibility_contract(self):
        builder = Builder(1.0)
        add_robot(builder, {'suggested_robot_spawn': {'x_m': 0, 'y_m': 0, 'yaw_deg': 0}})
        self.check_robot(builder.world.find("model[@name='parking_robot']"))

    def test_read_only_scan_summary_identifies_own_body_range_cluster(self):
        robot = json.loads((ROOT/'config/limo.json').read_text())
        scan = NS(header=NS(frame_id='lidar_link'), range_min=.12, range_max=30.,
                  angle_min=-.1, angle_increment=.1, ranges=[.26, .26, math.inf])
        summary = scan_summary(scan, robot)
        self.assertEqual(summary['returns_inside_body_rectangle'], 2)
        self.assertAlmostEqual(summary['forward_30deg_nearest_m'], .26)
        self.assertEqual(scan.ranges[:2], [.26, .26])
        scan.ranges = [10., 11., math.inf]
        self.assertEqual(scan_summary(scan, robot)['returns_inside_body_rectangle'], 0)


if __name__ == '__main__':
    unittest.main()
