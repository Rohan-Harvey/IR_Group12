"""Exercise the map/plan/execute boundary without borrowing simulator poses."""

import math
from types import SimpleNamespace
import unittest

import numpy as np

from parking_core import (Footprint, MovingReturns, OccupancyEvidence, OccupancySnapshot,
                          PathFollower, analytical_parking, audit_path, can_stop, candidate_poses, inverse,
                          measured_entry_width,
                          observation_maneuver, retreat_path, scan_rays, slot_score, split_path,
                          surveyed_bay_score)
from patrol_route import Point, transform


class ParkingCoreTests(unittest.TestCase):
    footprint = Footprint(-0.55, 2.45, 1.28, 1.06)

    def test_mapping_hits_win_over_misses_in_same_scan(self):
        grid = OccupancyEvidence(1.0)
        grid.insert((0.2, 0.2), [(2.3, 0.2, True), (3.2, 0.2, False), (2.3, 0.2, True)])
        self.assertEqual(grid.cells[(2, 0)], 5)
        self.assertEqual(grid.cells[(1, 0)], -2)
        self.assertEqual(grid.cells[(3, 0)], -2)
        grid.insert((0.2, 0.2), [(3.2, 0.2, False)]*100)
        self.assertEqual(grid.cells[(2, 0)], 3)

    def test_empty_laser_ray_clears_and_invalid_rays_do_not(self):
        scan = SimpleNamespace(range_min=0.1, range_max=6.0, angle_min=0.0, angle_increment=0.2,
                               ranges=[math.inf, math.nan, 0.01, 2.0])
        rays = list(scan_rays(scan, Point(0.0, 0.0, 0.0, 0.0), 3.0))
        self.assertEqual(rays[0], (3.0, 0.0, False))
        self.assertTrue(rays[1][2])
        self.assertEqual(len(rays), 2)

    def test_footprint_can_fit_while_intermediate_sweep_hits(self):
        odds = np.full((120, 120), -8, dtype=int)
        grid = OccupancySnapshot(0.12, (-60, -60), odds, np.ones_like(odds, dtype=bool))
        begin, end = Point(6.0, 0, 0, 0), Point(0.0, 0, 0, 0)
        ix, iy = grid.index(4.0, 0.8)
        odds[iy, ix] = 9
        grid = OccupancySnapshot(0.12, (-60, -60), odds, np.ones_like(odds, dtype=bool))
        self.assertFalse(grid.area(end, self.footprint.bounds(0.10, parked=True))[0])
        self.assertEqual(audit_path(grid, split_path([begin, end]), self.footprint, 0.10),
                         "occupied_sweep")

    def test_narrow_entry_can_be_measured_before_space_behind_it_is_observed(self):
        evidence = OccupancyEvidence(0.10)
        for x in range(-42, 43):
            for y in range(-41, -19):
                evidence.cells[x, y] = 9 if 9 <= abs(x) <= 32 else -8
        anchor = Point(0.05, 0.05, 0, 0)
        grid = evidence.snapshot()
        self.assertLess(measured_entry_width(grid, anchor), 2*self.footprint.half_width)
        self.assertEqual(grid.at(Point(0, -7, 0, 0)), 0)
        for cell in list(evidence.cells):
            if cell[0] > 0:
                del evidence.cells[cell]
        self.assertIsNone(measured_entry_width(evidence.snapshot(), anchor))

    def test_gap_discovery_requires_both_neighbor_evidence_and_known_floor(self):
        grid = OccupancyEvidence(0.12)
        for x in range(-85, 85):
            for y in range(-85, 85):
                if y > 55:
                    continue
                grid.cells[x, y] = 8 if 23 < y < 50 and 13 < abs(x) < 32 else -8
        goals = candidate_poses(grid.snapshot(), Point(0, 0, 0, 0), self.footprint, [])
        self.assertTrue(goals)
        self.assertLess(abs(goals[0].x), 0.55)
        self.assertAlmostEqual(goals[0].yaw, -math.pi/2)
        self.assertLess(goals[0].y, 6.1)

    def test_bounded_aisle_does_not_count_as_a_parking_gap(self):
        cells = OccupancyEvidence(0.12)
        for x in range(-80, 80):
            for y in range(-80, 80):
                cells.cells[x, y] = 8 if 14 < abs(x) < 29 else -8
        self.assertEqual(candidate_poses(cells.snapshot(), Point(0, 0, 0, 0), self.footprint, []), [])

    def test_finite_facing_rows_do_not_turn_through_aisle_into_parking_bay(self):
        evidence = OccupancyEvidence(0.12)
        goal = Point(0, 0, 0, 0)
        for ix in range(-55, 55):
            for iy in range(-45, 45):
                x, y = (ix+0.5)*0.12, (iy+0.5)*0.12
                flank = -1.1 < x < 2.8 and 1.58 < abs(y) < 2.37
                evidence.cells[ix, iy] = 10 if flank else -10
        self.assertIsNone(slot_score(evidence.snapshot(), goal, self.footprint, 0.10))

        # A rear obstruction can hide the route on the current map, but
        # the survey history still shows that the aisle served through traffic.
        for iy in range(-12, 13):
            evidence.cells[-18, iy] = 10
        grid = evidence.snapshot()
        self.assertIsNotNone(slot_score(grid, goal, self.footprint, 0.10))
        across_aisle = [Point(x/5, 0.12, 0.02, 0) for x in range(-8, 21)]
        self.assertIsNone(slot_score(grid, goal, self.footprint, 0.10, across_aisle))
        partial = across_aisle[:12]
        self.assertIsNotNone(slot_score(grid, goal, self.footprint, 0.10, partial))

    def test_short_neighbor_ends_do_not_define_perpendicular_bay(self):
        cells = OccupancyEvidence(0.12)
        for ix in range(-60, 60):
            for iy in range(-40, 75):
                x, y = (ix+0.5)*0.12, (iy+0.5)*0.12
                occupied = 2.65 < y < 4.45 and 1.65 < abs(x) < 3.6
                cells.cells[ix, iy] = 9 if occupied else -9
        sideways = Point(0, 5.24, -math.pi/2, 0)
        self.assertIsNone(slot_score(cells.snapshot(), sideways, self.footprint, 0.10))

    def test_surveyed_bay_accepts_long_flanks_and_excludes_a_known_through_lane(self):
        evidence = OccupancyEvidence(0.12)
        for i in range(-45, 46):
            for j in range(-49, 50):
                longitudinal = (i+0.5)*0.12
                lateral = (j+0.5)*0.12
                evidence.cells[i, j] = 10 if -0.7 < longitudinal < 2.2 and 2.9 < abs(lateral) < 4.1 else -10
        grid = evidence.snapshot()
        target = Point(0, 0, 0, 0)
        self.assertIsNotNone(surveyed_bay_score(grid, target, self.footprint, 0.10))
        driven = [Point(i*0.2, 0, 0, 0) for i in range(-8, 19)]
        self.assertIsNone(surveyed_bay_score(grid, target, self.footprint, 0.10, driven))

    def test_upper_clear_ray_cannot_erase_low_curb(self):
        upper, lower = OccupancyEvidence(0.12), OccupancyEvidence(0.12)
        upper.insert((0., 0.), [(3.0, 0., False)])
        lower.insert((0., 0.), [(2.0, 0., True)])
        point = Point(2.0, 0.0, 0.0, 0.0)
        self.assertLess(upper.snapshot().at(point), 0)
        self.assertGreater(upper.snapshot([lower]).at(point), 0)

    def test_reversing_stops_at_cusp_and_tracks_signed_command(self):
        points = [Point(x, 0, 0, 0) for x in (0, 0.5, 1.0, 0.5, 0)]
        follower = PathFollower(split_path(points))
        result = follower.update(points[2], 0.10, 0.6, 0.34, tight=True)
        self.assertEqual(result[0], "stopping")
        self.assertEqual(follower.update(points[2], 0, 0.6, 0.34, tight=True)[0], "shift")
        action, speed, _, direction = follower.update(points[2], 0, 0.4, 0.34, tight=True)
        self.assertEqual((action, direction), ("drive", -1))
        self.assertLess(speed, 0)
        action, *_ = follower.update(points[-1], 0, 0.4, 0.34, tight=True)
        self.assertEqual(action, "settled")

    def test_stop_check_considers_speed_shift_and_rear_hazard(self):
        front = np.array([[4.2, 0.2]])
        self.assertTrue(can_stop(front, self.footprint, 0.5, (0.,), 0.34, 0.30, 0.09))
        self.assertFalse(can_stop(front, self.footprint, 1.45, (0.,), 0.34, 0.30, 0.09))
        rear = np.array([[-1.02, 0.2]])
        self.assertFalse(can_stop(rear, self.footprint, -0.4, (0.,), 0.34, 0.30, 0.09))

    def test_crossing_return_can_stop_command_before_entering_lane(self):
        tracker = MovingReturns()
        for time, lateral in ((0.0, 3.1), (0.2, 2.86), (0.4, 2.62), (0.6, 2.38)):
            tracker.update(np.array([[3.9, lateral-0.1], [4.0, lateral], [4.1, lateral+0.1]]), time)
        active = tracker.active(0.6)
        self.assertEqual(active.shape, (1, 2, 2))
        self.assertLess(active[0, 1, 1], -0.8)
        self.assertFalse(can_stop(np.empty((0, 2)), self.footprint, 1.0, (0.,),
                                  0.34, 0.30, 0.09, moving=active))
        self.assertIsNone(tracker.active(1.1))

    def test_recovery_retraces_body_headings_in_reverse(self):
        trace = [Point(i*0.2, 0, 0, 0) for i in range(12)]
        legs = retreat_path(trace, Point(2.4, 0, 0, 0))
        self.assertEqual(legs[0][0], -1)
        self.assertEqual(legs[0][1][-1].x, 0)

    def test_recovery_requires_observed_sweep_and_prefers_withdrawal(self):
        pose = Point(0, 0, 0, 0)
        odds = np.full((120, 120), -8, dtype=int)
        grid = OccupancySnapshot(0.12, (-60, -60), odds, np.ones_like(odds, dtype=bool))
        legs = observation_maneuver(grid, pose, self.footprint, 4.75, [])
        self.assertEqual(legs[0][0], -1)
        unknown = np.zeros_like(odds)
        grid = OccupancySnapshot(0.12, (-60, -60), unknown, np.zeros_like(odds, dtype=bool))
        self.assertIsNone(observation_maneuver(grid, pose, self.footprint, 4.75, []))

    def test_analytic_candidate_ends_in_observed_bay_with_reverse_motion(self):
        evidence = OccupancyEvidence(0.12)
        for ix in range(-100, 80):
            for iy in range(-70, 85):
                x, y = (ix+0.5)*0.12, (iy+0.5)*0.12
                occupied = (2.7 < y < 6.8 and (-10.0 < x < -8.0 or -1.5 < x < 0.4)
                            or 7.05 < y < 7.25)
                evidence.cells[ix, iy] = 10 if occupied else -10
        plans = analytical_parking(evidence.snapshot(), Point(0, 0, 0, 0),
                                    0.0, self.footprint, [], 4.75, 0.10)
        valid = [(goal, legs) for goal, legs, reason in plans if not reason]
        self.assertTrue(valid)
        goal, legs = valid[0]
        self.assertAlmostEqual(goal.yaw, -math.pi/2)
        self.assertEqual(legs[0][0], -1)

    def test_odom_alignment_round_trip(self):
        origin = (-17.0, 22.0, 1.14)
        point = Point(11.0, 5.0, -1.0, 0.0)
        back = transform(transform(point, origin), inverse(origin))
        self.assertAlmostEqual(back.x, point.x)
        self.assertAlmostEqual(back.y, point.y)
        self.assertAlmostEqual(back.yaw, point.yaw)


if __name__ == "__main__":
    unittest.main()
