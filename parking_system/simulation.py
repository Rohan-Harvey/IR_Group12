from pathlib import Path
import json
import math
import time
import numpy as np
from .geometry import bicycle, rectangle, intersects, wrap
from .mapping import Scan, EncoderImuOdometry, LocalSlam, MotionTracker
from .control import Mission

class SensorWorld:

    def __init__(self, scene, actor_routes):
        self.polygons = [(name, np.asarray(p, dtype=float)) for name, p in scene]
        edges = [(a, b) for name, p in self.polygons for a, b in zip(p, np.roll(p, -1, axis=0))]
        self.edges = np.asarray(edges)
        self.routes = actor_routes
        self.bounds = [(name, p, np.array([p[:, 0].min(), p[:, 1].min(), p[:, 0].max(), p[:, 1].max()])) for name, p in self.polygons]

    def people(self, now):
        result = []
        for route in self.routes:
            start = np.asarray(route['start'])
            end = np.asarray(route['end'])
            duration = float(np.linalg.norm(end - start) / route['speed'])
            cycle = 2 * duration + 2
            t = max(0, now - route.get('delay', 0)) % cycle
            if t < duration:
                center = start + (end - start) * t / duration
            elif t < duration + 1:
                center = end
            elif t < 2 * duration + 1:
                center = end + (start - end) * (t - duration - 1) / duration
            else:
                center = start
            result.append(rectangle([*center, 0], 0.8, 0.8))
        return result

    def scan(self, pose, now, angles, rng, noise=0.01, maximum=35):
        extra = [(a, b) for poly in self.people(now) for a, b in zip(poly, np.roll(poly, -1, axis=0))]
        edges = np.concatenate((self.edges, np.asarray(extra))) if extra else self.edges
        q = edges[:, 0] - np.asarray(pose[:2])
        s = edges[:, 1] - edges[:, 0]
        beams = np.column_stack((np.cos(angles + pose[2]), np.sin(angles + pose[2])))
        den = beams[:, None, 0] * s[None, :, 1] - beams[:, None, 1] * s[None, :, 0]
        den = np.where(abs(den) > 1e-12, den, np.nan)
        t = (q[None, :, 0] * s[None, :, 1] - q[None, :, 1] * s[None, :, 0]) / den
        u = (q[None, :, 0] * beams[:, None, 1] - q[None, :, 1] * beams[:, None, 0]) / den
        ranges = np.min(np.where((t > 0.15) & (u >= 0) & (u <= 1) & np.isfinite(t), t, np.inf), axis=1)
        good = np.isfinite(ranges) & (ranges < maximum)
        ranges = np.where(good, ranges + rng.normal(0, noise, len(angles)), np.inf)
        return Scan(now, np.asarray(angles), ranges, 0.15, maximum)

    def collision(self, pose, length, width, now):
        body = rectangle(pose, length, width)
        bounds = np.array([body[:, 0].min(), body[:, 1].min(), body[:, 0].max(), body[:, 1].max()])
        for name, poly, b in self.bounds:
            if bounds[2] < b[0] or b[2] < bounds[0] or bounds[3] < b[1] or (b[3] < bounds[1]):
                continue
            if intersects(body, poly):
                return name
        for poly in self.people(now):
            if intersects(body, poly):
                return 'moving_pedestrian'
        return None

    def clearance(self, pose, length, width, now):
        body = rectangle(pose, length, width)
        bounds = np.array([body[:, 0].min(), body[:, 1].min(), body[:, 0].max(), body[:, 1].max()])
        people = self.people(now)
        polygons = [p for name, p, b in self.bounds] + people
        if not polygons:
            return math.inf
        boxes = np.asarray([b for name, p, b in self.bounds] + [np.array([p[:, 0].min(), p[:, 1].min(), p[:, 0].max(), p[:, 1].max()]) for p in people])
        dx = np.maximum(np.maximum(boxes[:, 0] - bounds[2], bounds[0] - boxes[:, 2]), 0)
        dy = np.maximum(np.maximum(boxes[:, 1] - bounds[3], bounds[1] - boxes[:, 3]), 0)
        lower = np.hypot(dx, dy)
        best = math.inf
        for index in np.argsort(lower):
            if lower[index] >= best:
                break
            p = polygons[index]
            if intersects(body, p):
                return 0.0
            for first, second in [(body, p), (p, body)]:
                for a, c in zip(second, np.roll(second, -1, axis=0)):
                    d = c - a
                    t = np.clip((first - a) @ d / max(d @ d, 1e-09), 0, 1)
                    distance = np.linalg.norm(first - (a + t[:, None] * d), axis=1)
                    best = min(best, float(distance.min()))
        return best

def load_world(root):
    import sys
    scripts = str(Path(root) / 'scripts')
    if scripts not in sys.path:
        sys.path.insert(0, scripts)
    import scene_scenarios
    from scene_people import ROUTES
    cfg = json.loads((Path(root) / 'config/parking_lot.json').read_text())
    return SensorWorld(scene_scenarios.obstacle_polygons(scene_scenarios.make_layout(cfg)), ROUTES if cfg['include_pedestrians'] else [])

def run(root, seed=0, output=None, observe_only=False, scan_match=True, initial_yaw_offset=0.0):
    root = Path(root)
    cfg = json.loads((root / 'config/autonomy.json').read_text())
    cfg['mapping']['scan_match'] = scan_match
    rng = np.random.default_rng(seed)
    world = load_world(root)
    slam = LocalSlam(cfg['mapping'])
    dynamic = MotionTracker(cfg['safety']['dynamic_memory_s'])
    mission = Mission(cfg, slam, dynamic)
    g = cfg['robot']
    pose = np.asarray(cfg['start_pose'], dtype=float)
    pose[2] += initial_yaw_offset
    odom = EncoderImuOdometry(cfg['start_pose'], g['wheelbase'], g['wheel_radius'])
    speed = steering = left = right = 0.0
    angles = np.linspace(-math.pi, math.pi, 180)
    dt = 0.05
    cloud = []
    trace = []
    error = None
    contacts = []
    min_clearance = math.inf
    start = time.perf_counter()
    last_theta = pose[2]
    status = 'RUNNING'
    now = 0.0
    estimated = pose.copy()
    for tick in range(int(cfg['safety']['mission_timeout'] / dt)):
        now = tick * dt
        yaw_noise = rng.normal(0, cfg['noise']['imu_std_rad'])
        rate = wrap(pose[2] - last_theta) / dt
        last_theta = pose[2]
        odom.imu(wrap(pose[2] + yaw_noise), rate, now)
        raw = odom.wheels(left, right, now)
        if tick % 4 == 0:
            scan = world.scan(pose, now, angles, rng, cfg['noise']['range_std_m'], cfg['mapping']['range_max'])
            slam.process(scan, raw, dynamic)
            cloud = scan.points()
        estimated = slam.current(raw)
        if now < cfg['safety'].get('startup_settle_s', 0.0):
            continue
        try:
            if observe_only and mission.state == 'SETTLE':
                status = 'OBSERVED'
                break
            v, w = mission.step(estimated, cloud, now, speed)
        except ValueError as exception:
            error = str(exception)
            status = 'ABORTED'
            break
        if mission.completed:
            status = 'COMPLETED'
            break
        speed += float(np.clip(v - speed, -0.5 * dt, 0.5 * dt))
        desired = math.atan(g['wheelbase'] * w / v) if abs(v) > 1e-08 else steering
        steering += float(np.clip(5 * (desired - steering) * dt, -1.5 * dt, 1.5 * dt))
        curvature = math.tan(steering) / g['wheelbase']
        distance = speed * dt
        pose = bicycle(pose, distance, curvature, g['wheelbase'])
        relative_noise = cfg['noise']['encoder_relative_std']
        bias = 1.0 + relative_noise * 0.35
        left += distance * (1 - 0.65 * curvature) / g['wheel_radius'] * bias + rng.normal(0, abs(distance / g['wheel_radius']) * relative_noise)
        right += distance * (1 + 0.65 * curvature) / g['wheel_radius'] * bias + rng.normal(0, abs(distance / g['wheel_radius']) * relative_noise)
        collision = world.collision(pose, g['length'], g['width'], now)
        if collision:
            contacts.append({'time_s': now, 'object': collision})
            error = 'Independent evaluator detected contact with ' + collision
            status = 'COLLISION'
            break
        if tick % 5 == 0:
            min_clearance = min(min_clearance, world.clearance(pose, g['length'], g['width'], now))
        if tick % 5 == 0:
            trace.append([now, *pose, *estimated, *raw, speed, steering])
    else:
        error = 'Simulation budget exhausted'
        status = 'TIMEOUT'
    report = {'kind': 'offline 2D ray-cast sensor and acceleration/steering-limited bicycle simulation; not Gazebo', 'seed': seed, 'initial_yaw_offset_rad': initial_yaw_offset, 'status': status, 'error': error, 'simulated_time_s': round(now, 3), 'wall_time_s': round(time.perf_counter() - start, 3), 'laps': mission.laps, 'contacts': contacts, 'minimum_true_clearance_m': min_clearance, 'clearance_sample_period_s': 0.25, 'final_true_pose': pose.tolist(), 'final_estimated_pose': estimated.tolist(), 'goal_pose': mission.goal, 'decisions': mission.decisions, 'events': mission.events, 'dynamic_track_samples': len(dynamic.zones), 'map_revision': slam.grid.revision, 'scan_processing_mean_ms': float(np.mean(slam.processing_ms)), 'scan_processing_p95_ms': float(np.quantile(slam.processing_ms, 0.95)), 'simulator_truth_used_for_control': False}
    if mission.goal is not None:
        report['position_error_m'] = math.dist(pose[:2], mission.goal[:2])
        report['heading_error_deg'] = math.degrees(abs(wrap(pose[2] - mission.goal[2])))
    if trace:
        a = np.asarray(trace)
        report['slam_position_rmse_m'] = float(np.sqrt(np.mean(np.sum((a[:, 1:3] - a[:, 4:6]) ** 2, axis=1))))
        report['odometry_position_rmse_m'] = float(np.sqrt(np.mean(np.sum((a[:, 1:3] - a[:, 7:9]) ** 2, axis=1))))
    if output is not None:
        output = Path(output)
        output.mkdir(parents=True, exist_ok=True)
        (output / 'report.json').write_text(json.dumps(report, indent=2) + '\n')
        slam.grid.save(output / 'estimated_map.npz')
        if mission.assessment_grid is not None:
            mission.assessment_grid.save(output / 'assessment_map.npz')
        np.savez_compressed(output / 'trace.npz', observation_history=mission.observation_history if mission.observation_history is not None else np.asarray(mission.recorded), samples=np.asarray(trace), history=np.asarray(mission.recorded), reference=mission.path, reverse=mission.plan if mission.plan is not None else np.empty((0, 3)), dynamic=np.asarray([z[1] for z in dynamic.zones]), pose=pose, odom=raw)
    return (report, mission, slam, dynamic, world)
