import math
from types import SimpleNamespace
import unittest

import numpy as np

from parking_core import Footprint, course_speed_profile, parking_observation_path
from parking_localization import CourseLocalization, MarkovGrid, ScanLikelihood, localization_hits
from patrol_route import Point, transform, wrap


class ParkingLocalizationTest(unittest.TestCase):
    def setUp(self):
        grid = np.zeros((160, 160), dtype=np.int8)
        grid[28:110, 25] = 100
        grid[106, 26:102] = 100
        grid[37:88, 119] = 100
        grid[45, 63:90] = 100
        self.model = ScanLikelihood(grid, 0.1, (-8.0, -8.0, 0.0))
        grid_y, grid_x = np.nonzero(grid == 100)
        self.map_points = np.column_stack((grid_x, grid_y))[::4]*0.1-7.95

    def scan(self, pose):
        hits = self.map_points-np.array((pose.x, pose.y))
        c, s = math.cos(pose.yaw), math.sin(pose.yaw)
        return hits @ np.array(((c, -s), (s, c)))

    def test_offset_initialization_and_tracking(self):
        localizer = CourseLocalization(self.model, (0, 0, 0), (1.25, -0.8, 0.19))
        seed = Point(0.6, -0.6, 0.22, 0)
        events = [localizer.update(seed, self.scan(seed))[0] for _ in range(7)]
        self.assertIn("markov_initialized_particles", events)
        self.assertIn("particle_tracking", events)
        for i in range(1, 15):
            odom = Point(seed.x+0.12*i, seed.y, seed.yaw, 0)
            actual = Point(odom.x-0.013*i, odom.y+0.010*i, odom.yaw-0.003*i, 0)
            event, _, _, _ = localizer.update(odom, self.scan(actual))
            self.assertEqual(event, "particle_tracking")
        estimate = transform(odom, localizer.alignment)
        self.assertLess(math.hypot(estimate.x-actual.x, estimate.y-actual.y), 0.17)
        self.assertLess(abs(wrap(estimate.yaw-actual.yaw)), math.radians(3))

    def test_low_likelihood_restarts_grid_and_reacquires(self):
        localizer = CourseLocalization(self.model, (0, 0, 0))
        pose = Point(0.6, -0.6, 0.22, 0)
        for _ in range(7):
            localizer.update(pose, self.scan(pose))
        self.assertIsNotNone(localizer.alignment)
        events = [localizer.update(pose, np.full((20, 2), 100.0))[0] for _ in range(5)]
        self.assertEqual(events[-1], "particle_relocalizing")
        self.assertIsNone(localizer.alignment)
        for _ in range(7):
            localizer.update(pose, self.scan(pose))
        self.assertIsNotNone(localizer.alignment)

    def test_markov_prediction_moves_probability_using_odometry(self):
        class UninformativeScan:
            def score(self, poses, hits):
                return np.zeros(len(poses)), np.ones(len(poses))

        grid = MarkovGrid((0, 0, 0))
        grid.weights.fill(0.0)
        grid.weights[len(grid.weights)//2] = 1.0
        estimate, _, _ = grid.update(UninformativeScan(), (0.8, 0.0, 0.0), None)
        np.testing.assert_allclose(estimate, (0.8, 0.0, 0.0), atol=1e-10)

    def test_sparse_entrance_scan_samples_only_valid_hits(self):
        ranges = np.full(720, np.inf)
        ranges[12:32] = 14.0
        ranges[101:161] = 21.0
        scan = SimpleNamespace(ranges=ranges, range_min=0.1, range_max=30.0,
                               angle_min=-math.pi, angle_increment=math.tau/len(ranges))
        hits = localization_hits(scan)
        self.assertEqual(hits.shape, (64, 2))
        self.assertAlmostEqual(np.linalg.norm(hits[0]), 14.0)
        self.assertAlmostEqual(np.linalg.norm(hits[-1]), 21.0)


class CourseInspectionTest(unittest.TestCase):
    def test_straight_can_cruise_and_bend_has_braking_distance(self):
        path = [Point(i, 0, 0, 0 if i < 8 else 0.20) for i in range(10)]
        speeds = course_speed_profile(path, 1.75, 0.22)
        self.assertEqual(speeds[0], 1.75)
        self.assertLess(speeds[7], 1.30)
        self.assertAlmostEqual(speeds[8], 1.0)

    def test_observation_probe_requires_clear_motion_outside_mouth(self):
        class Grid:
            def __init__(self, occupied, observed):
                self.result = occupied, observed

            def area(self, _pose, _bounds):
                return self.result

        entrance = Point(0, 0, 0, 0)
        pose = Point(3.2, 0, math.pi/2, 0)
        footprint = Footprint(-0.6, 2.3, 1.28, 1.06)
        legs = parking_observation_path(Grid(False, 1.0), pose, entrance, footprint, 4.75)
        self.assertEqual([direction for direction, _ in legs], [-1, 1])
        self.assertEqual(legs[0][1], list(reversed(legs[1][1])))
        for grid in (Grid(True, 1.0), Grid(False, 0.8)):
            self.assertIsNone(parking_observation_path(grid, pose, entrance, footprint, 4.75))
        too_close = Point(1.4, 0, math.pi/2, 0)
        self.assertIsNone(parking_observation_path(Grid(False, 1.0), too_close, entrance, footprint, 4.75))
