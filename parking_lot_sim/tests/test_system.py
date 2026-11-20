import json, math, sys, unittest
from pathlib import Path
import numpy as np
import xml.etree.ElementTree as ET
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / 'scripts'))
from parking_system.mapping import Scan, OccupancyGrid, CollisionGrid, EncoderImuOdometry, PoseBuffer, MotionTracker
from parking_system.geometry import exploration_route, rectangle, intersects, bicycle, wrap
from parking_system.control import PathTracker, ObstacleGuard, StreamWatchdog
from parking_system.planning import ReversePlanner
from parking_system.perception import CandidateGenerator
import scene_scenarios as scenes
import scene_robot

class SystemTests(unittest.TestCase):

    def setUp(self):
        self.cfg = json.loads((ROOT / 'config/autonomy.json').read_text())

    def free_grid(self):
        grid = OccupancyGrid(self.cfg['mapping'])
        grid.observed[:] = True
        grid.logodds[:] = -4.6
        return grid

    def test_bay_order_and_two_closed_ccw_laps(self):
        route, entry, laps = exploration_route(self.cfg)
        self.assertEqual(len(laps), 2)
        for begin, end in laps:
            self.assertLess(np.linalg.norm(route[begin, :2] - route[end, :2]), 1e-06)
            self.assertAlmostEqual(route[end, 2] - route[begin, 2], math.tau)
            points = route[begin:end + 1]
            area = np.sum(points[:-1, 0] * points[1:, 1] - points[1:, 0] * points[:-1, 1]) / 2
            self.assertGreater(area, 0)
            indices = []
            for case in scenes.definitions():
                indices.append(int(np.argmin(np.linalg.norm(points[:, :2] - np.asarray(case['goal'][:2]), axis=1))))
            self.assertEqual(indices, sorted(indices))
            self.assertEqual(len(set(indices)), 6)
        first = next((c for c in scenes.definitions() if c['id'] == 'S1'))
        clear = next((c for c in scenes.definitions() if c['id'] == 'S6'))
        self.assertLess(np.linalg.norm(route[:entry, :2] - first['goal'][:2], axis=1).min(), np.linalg.norm(route[:entry, :2] - clear['goal'][:2], axis=1).min())

    def test_all_parked_vehicles_match_limo_dimensions(self):
        cfg = json.loads((ROOT / 'config/parking_lot.json').read_text())
        expected = [scene_robot.FOOTPRINT_LENGTH, scene_robot.FOOTPRINT_WIDTH, scenes.CAR_HEIGHT]
        for bay in scenes.make_layout(cfg):
            np.testing.assert_allclose(bay['car']['envelope_m'], expected)
        tree = ET.parse(ROOT / 'worlds/parking_lot.sdf')
        model = tree.find("world/model[@name='parking_robot']")
        self.assertEqual(len(model.findall('.//mesh')), 5)
        self.assertEqual(len(tree.findall('world/actor')), 1)

    def test_inverse_sensor_update_unknown_and_no_returns(self):
        cfg = dict(self.cfg['mapping'], origin=[0.0, 0.0], size=[6.0, 6.0], resolution=0.1, range_max=5.0)
        grid = OccupancyGrid(cfg)
        scan = Scan(0, np.array([0.0]), np.array([2.0]), 0.1, 5.0)
        grid.integrate(scan, [1.05, 2.05, 0])
        prob = grid.probability()
        self.assertAlmostEqual(float(prob[20, 20]), 0.3, places=5)
        self.assertAlmostEqual(float(prob[20, 30]), 0.8, places=5)
        self.assertFalse(grid.observed[20, 40])
        grid.integrate(Scan(1, np.array([0.0]), np.array([math.inf]), 0.1, 5.0), [1.05, 3.05, 0])
        self.assertFalse((grid.logodds[30] > 0).any())

    def test_unseen_footprint_is_rejected(self):
        grid = OccupancyGrid(self.cfg['mapping'])
        g = self.cfg['robot']
        self.assertFalse(CollisionGrid(grid, g['length'], g['width'], 0.2).footprint([0, 9, 0]))

    def test_swept_check_detects_between_endpoints(self):
        grid = self.free_grid()
        cell = grid.cells([[0, 9]])[0]
        grid.logodds[cell[1], cell[0]] = 4.6
        collision = CollisionGrid(grid, 0.1, 0.1, 0.01)
        cfg = dict(self.cfg, robot=dict(self.cfg['robot'], length=0.1, width=0.1))
        planner = ReversePlanner(cfg, collision)
        self.assertTrue(collision.footprint([-1, 9, 0]))
        self.assertTrue(collision.footprint([1, 9, 0]))
        self.assertFalse(planner.swept(np.array([[-1, 9, 0], [1, 9, 0]])))

    def test_reverse_arcs_are_steering_limited_and_reach_goal(self):
        grid = self.free_grid()
        g = self.cfg['robot']
        planner = ReversePlanner(self.cfg, CollisionGrid(grid, g['length'], g['width'], 0.2))
        for goal in [[0, -13.6, math.pi / 2], [0, 13.6, -math.pi / 2]]:
            for side in [-1, 1]:
                path = planner.quarter(goal, 5.5, side)
                self.assertLess(np.linalg.norm(path[-1, :2] - goal[:2]), 1e-08)
                self.assertLess(abs(wrap(path[-1, 2] - goal[2])), 1e-08)
                for a, b in zip(path, path[1:]):
                    rear_a = a[:2] - g['wheelbase'] / 2 * np.array([math.cos(a[2]), math.sin(a[2])])
                    rear_b = b[:2] - g['wheelbase'] / 2 * np.array([math.cos(b[2]), math.sin(b[2])])
                    self.assertLess(np.dot(rear_b - rear_a, [math.cos(a[2]), math.sin(a[2])]), 0)

    def test_sensor_guard_stops_and_resumes_forward_and_reverse(self):
        guard = ObstacleGuard(self.cfg['robot'])
        for sign in [-1, 1]:
            self.assertTrue(guard.check([(sign * 2.1, 0)], sign * 0.4, 0, 0))
            self.assertFalse(guard.check([(sign * 10, 0)], sign * 0.4, 0, 1))

    def test_stale_invalid_and_reset_streams(self):
        guard = StreamWatchdog(['scan', 'imu'])
        self.assertFalse(guard.ready(0, 0))
        guard.feed('scan', 1, 1)
        guard.feed('imu', 1, 1)
        self.assertTrue(guard.ready(1.1, 1.1))
        self.assertFalse(guard.ready(3, 1.2))
        with self.assertRaises(ValueError):
            guard.ready(0, 1.3)
        self.assertFalse(Scan(0, np.arange(30), np.full(30, np.nan)).valid())

    def test_time_aligned_pose_interpolation(self):
        buffer = PoseBuffer()
        buffer.add(0, [0, 0, math.radians(179)])
        buffer.add(1, [2, 0, math.radians(-179)])
        p = buffer.at(0.5)
        self.assertAlmostEqual(p[0], 1)
        self.assertLess(abs(abs(p[2]) - math.pi), 1e-06)
        self.assertIsNone(buffer.at(2))

    def test_encoder_imu_closed_circle(self):
        g = self.cfg['robot']
        e = EncoderImuOdometry([0, 0, 0], g['wheelbase'], g['wheel_radius'])
        e.imu(0, 0, 0)
        e.wheels(0, 0, 0)
        for i in range(1, 1001):
            a = math.tau * i / 1000
            e.imu(wrap(a), math.tau / 50, i * 0.05)
            p = e.wheels(5.5 * a / g['wheel_radius'], 5.5 * a / g['wheel_radius'], i * 0.05)
        self.assertLess(np.linalg.norm(p[:2]), 0.001)

    def test_moving_occupancy_does_not_require_scenario_labels(self):
        tracker = MotionTracker()
        for i in range(12):
            x = i * 0.12
            cloud = np.array([[x - 0.2, -0.2], [x + 0.2, -0.2], [x + 0.2, 0.2], [x - 0.2, 0.2]])
            tracker.update(cloud, i * 0.2)
        self.assertGreater(len(tracker.zones), 0)
        self.assertTrue(tracker.conflict([[1.2, 0, 0], [1.4, 0, 0]], 0.5, 0.5))
        self.assertFalse(tracker.conflict([[10, 10, 0]], 0.5, 0.5))

    def test_runtime_does_not_read_reference_labels_or_poses(self):
        for filename in ['mapping.py', 'control.py', 'perception.py', 'planning.py']:
            source = (ROOT / 'parking_system' / filename).read_text()
            for forbidden in ['scene_scenarios', 'parking_scenarios.json', 'scene_ground_truth.json', 'goal_pose_m_deg', 'model_states', 'ground_truth']:
                self.assertNotIn(forbidden, source, filename)

    def test_candidates_follow_measured_map_translation(self):
        grid = self.free_grid()
        ys = grid.origin[1] + (np.arange(grid.height) + 0.5) * grid.resolution
        xs = grid.origin[0] + (np.arange(grid.width) + 0.5) * grid.resolution
        row = (ys > 11.5) & (ys < 16.5)
        wall = (xs > -23) & (xs < 20)
        grid.logodds[np.ix_(row, wall)] = 4.6
        gap = (xs > 6) & (xs < 12)
        grid.logodds[np.ix_(row, gap)] = -4.6
        route, entry, _ = exploration_route(self.cfg)
        generator = CandidateGenerator(self.cfg)
        first = generator.generate(grid, route, entry)
        self.assertEqual(len(first), 1)
        self.assertAlmostEqual(first[0]['pose'][0], 9.0, places=4)
        grid.origin[0] += 0.7
        route[:, 0] += 0.7
        second = generator.generate(grid, route, entry)
        self.assertEqual(len(second), 1)
        self.assertAlmostEqual(second[0]['pose'][0] - first[0]['pose'][0], 0.7, places=4)

    def test_hybrid_fallback_finds_reverse_straight_path(self):
        import time
        g = self.cfg['robot']
        planner = ReversePlanner(self.cfg, CollisionGrid(self.free_grid(), g['length'], g['width'], 0.2))
        path, status = planner.hybrid([0, 9, 0], [-2, 9, 0], time.perf_counter())
        self.assertIsNotNone(path)
        self.assertEqual(status, 'VALIDATED_REVERSE_HYBRID_ASTAR')
        self.assertLess(np.linalg.norm(path[-1, :2] - [-2, 9]), 0.18)

    def test_clearance_includes_long_obstacles_with_distant_centres(self):
        from parking_system.simulation import SensorWorld
        world = SensorWorld([('long_wall', rectangle([0, 0, 0], 100, 0.2))], [])
        self.assertAlmostEqual(world.clearance([40, 1, 0], 1, 1, 0), 0.4, places=7)

    def test_first_imu_heading_preserves_settling_rotation(self):
        g = self.cfg['robot']
        for rotation in [-0.1, 0.1]:
            e = EncoderImuOdometry([0, 0, 0.2], g['wheelbase'], g['wheel_radius'])
            e.imu(rotation, 0, 0)
            p = e.wheels(0, 0, 0)
            self.assertAlmostEqual(p[2], 0.2 + rotation)
            for i in range(1, 21):
                t = i * 0.05
                e.imu(rotation, 0, t)
                p = e.wheels(i * 0.05 / g['wheel_radius'], i * 0.05 / g['wheel_radius'], t)
            np.testing.assert_allclose(p[:2], [math.cos(0.2 + rotation), math.sin(0.2 + rotation)], atol=1e-08)

    def test_imu_reference_unwrap_keeps_initial_heading(self):
        g = self.cfg['robot']
        e = EncoderImuOdometry([0, 0, 0], g['wheelbase'], g['wheel_radius'])
        e.imu(math.radians(179), 0, 0)
        e.wheels(0, 0, 0)
        e.imu(math.radians(-179), 0, 0.05)
        pose = e.wheels(0, 0, 0.05)
        self.assertAlmostEqual(pose[2], math.radians(181))

    def test_scan_matching_keeps_the_imu_yaw_reference(self):
        from parking_system.mapping import LocalSlam
        config = dict(self.cfg['mapping'], imu_yaw_locked=True)
        slam = LocalSlam(config)
        dynamics = MotionTracker()
        angles = np.linspace(-math.pi, math.pi, 180)
        scan = Scan(0, angles, np.full(180, 5.0), 0.15, 35.0)
        pose = slam.process(scan, np.array([-30.0, -9.0, 0.13]), dynamics)
        self.assertAlmostEqual(pose[2], 0.13)
        self.assertAlmostEqual(slam.current([-29.9, -9.0, 0.16])[2], 0.16)

    def test_laser_range_matches_the_offline_runner(self):
        world = ET.parse(ROOT / 'worlds/parking_lot.sdf')
        laser = world.find('.//sensor[@type="gpu_lidar"]')
        self.assertAlmostEqual(float(laser.findtext('lidar/range/max')), self.cfg['mapping']['range_max'])
        self.assertAlmostEqual(float(laser.findtext('lidar/noise/stddev')), self.cfg['noise']['range_std_m'])

    def test_obstacle_stops_are_reported_by_the_supervisor(self):
        from parking_system.control import Mission
        mission = Mission(self.cfg, None, MotionTracker())
        pose = self.cfg['start_pose']
        self.assertEqual(mission.step(pose, [(2.1, 0)], 0), (0.0, 0.0))
        self.assertEqual(mission.last_stop_reason, 'OBSTACLE_IN_STOPPING_PATH')
        self.assertTrue(any((e['event'] == 'obstacle_stop' for e in mission.events)))
        self.assertIsNotNone(mission.guard.blocking_point)
        self.assertGreater(mission.step(pose, [], 1)[0], 0)
        self.assertTrue(any((e['event'] == 'obstacle_clear' for e in mission.events)))

    def test_s5_independent_geometric_enclosure(self):
        cars = scenes.make_layout(json.loads((ROOT / 'config/parking_lot.json').read_text()))
        names = ['S5_side_-1', 'S5_side_1', 'S5_approach_-1', 'S5_approach_1']
        boxes = []
        centres = {}
        for bay in cars:
            if bay['id'] not in names:
                continue
            car = bay['car']
            x, y, _ = car['position_m']
            centres[bay['id']] = [x, y]
            boxes.append(([x, y], scenes.CAR_LENGTH / 2, scenes.CAR_WIDTH / 2, car['yaw_rad']))
        boxes.append(([0, 18.5], 5, 0.1, 0))
        curve = [centres['S5_side_-1'], centres['S5_approach_-1'], centres['S5_approach_1'], centres['S5_side_1'], [centres['S5_side_1'][0], 18.5], [centres['S5_side_-1'][0], 18.5]]

        def inside(point):
            x, y = point
            result = False
            for a, b in zip(curve, curve[1:] + curve[:1]):
                if (a[1] > y) != (b[1] > y) and x < (b[0] - a[0]) * (y - a[1]) / (b[1] - a[1]) + a[0]:
                    result = not result
            return result
        self.assertTrue(inside([0, 16.2]))
        self.assertFalse(inside(self.cfg['start_pose'][:2]))
        radius = min(scene_robot.BODY_SIZE[:2]) / 2
        for a, b in zip(curve, curve[1:] + curve[:1]):
            n = math.ceil(math.dist(a, b) / 0.01)
            worst = 0
            for point in np.linspace(a, b, n + 1):
                distances = []
                for (x, y), half_l, half_w, h in boxes:
                    dx, dy = point - [x, y]
                    px = math.cos(h) * dx + math.sin(h) * dy
                    py = -math.sin(h) * dx + math.cos(h) * dy
                    distances.append(math.hypot(max(abs(px) - half_l, 0), max(abs(py) - half_w, 0)))
                worst = max(worst, min(distances))
            self.assertLess(worst + 0.005, radius)
        manifest = scenes.make_manifest(json.loads((ROOT / 'config/parking_lot.json').read_text()), cars)
        case = next((c for c in manifest['cases'] if c['id'] == 'S5'))
        self.assertTrue(case['static_goal_fits'])
if __name__ == '__main__':
    unittest.main()
