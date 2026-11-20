from dataclasses import dataclass
from collections import deque
import math
import time
import numpy as np
from scipy.spatial import cKDTree
from scipy.ndimage import distance_transform_edt
from .geometry import compose, relative, transform, wrap

@dataclass
class Scan:
    stamp: float
    angles: np.ndarray
    ranges: np.ndarray
    minimum: float = 0.15
    maximum: float = 35.0

    def valid(self):
        return len(self.angles) == len(self.ranges) and len(self.ranges) >= 30 and np.isfinite(self.angles).all() and (not np.isnan(self.ranges).any()) and (not (self.ranges < 0).any()) and np.isfinite(self.stamp)

    def points(self):
        hit = np.isfinite(self.ranges) & (self.ranges >= self.minimum) & (self.ranges < self.maximum - 0.02)
        return np.column_stack((self.ranges[hit] * np.cos(self.angles[hit]), self.ranges[hit] * np.sin(self.angles[hit])))

class PoseBuffer:

    def __init__(self, size=400):
        self.samples = deque(maxlen=size)

    def add(self, stamp, pose):
        if self.samples and stamp < self.samples[-1][0] - 0.01:
            raise ValueError('Pose clock reset')
        self.samples.append((stamp, np.asarray(pose).copy()))

    def at(self, stamp, tolerance=0.12):
        if not self.samples:
            return None
        if stamp < self.samples[0][0] - tolerance or stamp > self.samples[-1][0] + tolerance:
            return None
        for (ta, a), (tb, b) in zip(reversed(list(self.samples)[:-1]), reversed(list(self.samples)[1:])):
            if ta <= stamp <= tb:
                f = (stamp - ta) / max(tb - ta, 1e-09)
                p = a + (b - a) * f
                p[2] = a[2] + wrap(b[2] - a[2]) * f
                return p
        return self.samples[-1][1].copy() if stamp >= self.samples[-1][0] else self.samples[0][1].copy()

class EncoderImuOdometry:

    def __init__(self, start, wheelbase, radius):
        self.pose = np.asarray(start, dtype=float)
        self.wb = wheelbase
        self.radius = radius
        self.previous = None
        self.previous_stamp = None
        self.imu_last = None
        self.heading = 0.0
        self.imu_stamp = None
        self.rate = 0.0
        self.start_heading = start[2]

    def imu(self, yaw, rate, stamp):
        if not np.isfinite([yaw, rate, stamp]).all():
            raise ValueError('Invalid IMU')
        if self.imu_last is None:
            self.heading = yaw
        else:
            self.heading += wrap(yaw - self.imu_last)
        self.imu_last = yaw
        self.imu_stamp = stamp
        self.rate = rate

    def wheels(self, left, right, stamp):
        if self.imu_stamp is None:
            return None
        if not np.isfinite([left, right, stamp]).all():
            raise ValueError('Invalid encoders')
        heading = self.start_heading + self.heading + self.rate * max(-0.1, min(0.1, stamp - self.imu_stamp))
        if self.previous is not None:
            if stamp < self.previous_stamp - 0.01 or stamp - self.previous_stamp > 1:
                raise ValueError('Encoder timestamp discontinuity')
            d = self.radius * (wrap(left - self.previous[0]) + wrap(right - self.previous[1])) / 2
            old = self.pose[2]
            mid = old + wrap(heading - old) / 2
            rear = self.pose[:2] - self.wb / 2 * np.array([math.cos(old), math.sin(old)])
            rear += d * np.array([math.cos(mid), math.sin(mid)])
            self.pose[:2] = rear + self.wb / 2 * np.array([math.cos(heading), math.sin(heading)])
        self.pose[2] = heading
        self.previous = (left, right)
        self.previous_stamp = stamp
        return self.pose.copy()

class OccupancyGrid:

    def __init__(self, config):
        self.config = config
        self.resolution = config['resolution']
        self.origin = np.asarray(config['origin'])
        self.width, self.height = np.ceil(np.asarray(config['size']) / self.resolution).astype(int)
        self.logodds = np.zeros((self.height, self.width), np.float32)
        self.observed = np.zeros_like(self.logodds, dtype=bool)
        self.hits = np.zeros_like(self.logodds, dtype=np.uint16)
        self.last_seen = np.full_like(self.logodds, -np.inf)
        self.free_logit = math.log(config['free_probability'] / (1 - config['free_probability']))
        self.hit_logit = math.log(config['hit_probability'] / (1 - config['hit_probability']))
        self.revision = 0
        self.ray_distances = np.arange(0, config['range_max'] + 0.01, self.resolution * 0.8, dtype=np.float32)

    def cells(self, points):
        return np.floor((np.asarray(points) - self.origin) / self.resolution).astype(int)

    def centres(self, cells):
        return self.origin + (np.asarray(cells) + 0.5) * self.resolution

    def in_bounds(self, cells):
        return (cells[:, 0] >= 0) & (cells[:, 0] < self.width) & (cells[:, 1] >= 0) & (cells[:, 1] < self.height)

    def integrate(self, scan, pose, dynamic_points=None):
        maximum = min(scan.maximum, self.config['range_max'])
        ranges = np.minimum(scan.ranges, maximum)
        valid = (ranges >= scan.minimum) & ~np.isnan(ranges)
        angles = scan.angles[valid] + pose[2]
        ranges = ranges[valid]
        ds = self.ray_distances[self.ray_distances <= maximum]
        mask = ds[None, :] < ranges[:, None] - self.resolution * 0.8
        x = pose[0] + np.cos(angles)[:, None] * ds
        y = pose[1] + np.sin(angles)[:, None] * ds
        cells = self.cells(np.column_stack((x[mask], y[mask])))
        cells = cells[self.in_bounds(cells)]
        indices = np.unique(cells[:, 1] * self.width + cells[:, 0])
        odds = self.logodds.ravel()
        odds[indices] += self.free_logit
        self.observed.ravel()[indices] = True
        self.last_seen.ravel()[indices] = scan.stamp
        hit = ranges < maximum - 0.02
        end = np.column_stack((pose[0] + np.cos(angles[hit]) * ranges[hit], pose[1] + np.sin(angles[hit]) * ranges[hit]))
        if dynamic_points is not None and len(dynamic_points) and len(end):
            tree = cKDTree(dynamic_points)
            distance = tree.query(end)[0]
            end = end[distance > 0.32]
        cells = self.cells(end)
        cells = cells[self.in_bounds(cells)]
        idx = np.unique(cells[:, 1] * self.width + cells[:, 0])
        odds[idx] += self.hit_logit
        self.observed.ravel()[idx] = True
        self.last_seen.ravel()[idx] = scan.stamp
        self.hits.ravel()[idx] = np.minimum(self.hits.ravel()[idx].astype(int) + 1, 65535)
        np.clip(odds, -4.6, 4.6, out=odds)
        self.revision += 1

    def probability(self):
        return 1 / (1 + np.exp(-self.logodds))

    def masks(self):
        p = self.probability()
        return (self.observed & (p < self.config['free_threshold']), self.observed & (p >= self.config['occupied_threshold']))

    def occupied_points(self):
        _, occ = self.masks()
        rows, cols = np.nonzero(occ)
        return self.centres(np.column_stack((cols, rows)))

    def message_data(self):
        return np.where(self.observed, np.rint(self.probability() * 100), -1).astype(np.int8).ravel()

    def save(self, path):
        np.savez_compressed(path, logodds=self.logodds, observed=self.observed, origin=self.origin, resolution=self.resolution, revision=self.revision)

class MotionTracker:

    def __init__(self, memory=700):
        self.tracks = []
        self.zones = []
        self.memory = memory
        self.next_id = 1

    def update(self, points, now):
        points = np.asarray(points)
        if len(points) < 2:
            return np.empty((0, 2))
        tree = cKDTree(points)
        groups = tree.query_ball_point(points, 0.45)
        seen = set()
        clusters = []
        for i in range(len(points)):
            if i in seen:
                continue
            queue = [i]
            seen.add(i)
            indices = []
            while queue:
                j = queue.pop()
                indices.append(j)
                for k in groups[j]:
                    if k not in seen:
                        seen.add(k)
                        queue.append(k)
            if len(indices) < 2:
                continue
            cloud = points[indices]
            extent = np.ptp(cloud, axis=0)
            if np.linalg.norm(extent) <= 1.25:
                clusters.append((cloud.mean(axis=0), cloud))
        active = []
        dynamic = []
        used = set()
        for center, cloud in clusters:
            candidates = [(np.linalg.norm(center - t['position']), i, t) for i, t in enumerate(self.tracks) if i not in used and now - t['time'] < 0.65]
            if candidates and min(candidates, key=lambda x: x[0])[0] < 0.65:
                _, i, t = min(candidates, key=lambda x: x[0])
                used.add(i)
                dt = max(now - t['time'], 0.05)
                velocity = (center - t['position']) / dt
                t['velocity'] = 0.65 * t['velocity'] + 0.35 * velocity
                t['position'] = center
                t['time'] = now
                t['count'] += 1
                t['trail'].append(center.copy())
                if len(t['trail']) > 60:
                    t['trail'].pop(0)
                displacement = np.linalg.norm(center - t['trail'][0])
                speed = np.linalg.norm(t['velocity'])
                if t['count'] >= 5 and displacement > 0.35 and (0.12 < speed < 1.8):
                    t['moving'] = True
                if t['moving']:
                    dynamic.extend(cloud.tolist())
                    self.zones.append((now, center.copy(), t['id']))
            else:
                t = {'id': self.next_id, 'position': center, 'time': now, 'velocity': np.zeros(2), 'count': 1, 'trail': [center.copy()], 'moving': False}
                self.next_id += 1
            active.append(t)
        self.tracks = active
        self.zones = [z for z in self.zones if now - z[0] <= self.memory]
        return np.asarray(dynamic).reshape(-1, 2)

    def suppress_static(self, grid):
        if not self.zones:
            return
        _, occupied = grid.masks()
        rows, cols = np.nonzero(occupied & (grid.hits >= 8))
        if not len(rows):
            return
        tree = cKDTree(grid.centres(np.column_stack((cols, rows))))
        points = np.asarray([z[1] for z in self.zones])
        distances = tree.query(points)[0]
        self.zones = [z for z, d in zip(self.zones, distances) if d > 0.24]

    def conflict(self, path, length, width, margin=0.35):
        if not self.zones:
            return False
        zones = np.asarray([z[1] for z in self.zones])
        tree = cKDTree(zones)
        radius = math.hypot(length / 2 + margin, width / 2 + margin)
        for x, y, h in np.asarray(path)[::3]:
            ids = tree.query_ball_point([x, y], radius)
            if not ids:
                continue
            local = transform(zones[ids] - np.array([x, y]), [0, 0, -h])
            if np.any((abs(local[:, 0]) < length / 2 + margin) & (abs(local[:, 1]) < width / 2 + margin)):
                return True
        return False

class LocalSlam:

    def __init__(self, config):
        self.grid = OccupancyGrid(config)
        self.config = config
        self.pose = None
        self.last_odom = None
        self.tree = None
        self.scans = 0
        self.sigma = 0.03
        self.history = []
        self.processing_ms = []

    def process(self, scan, odom, dynamics):
        start = time.perf_counter()
        if not scan.valid():
            raise ValueError('Invalid laser scan')
        odom = np.asarray(odom)
        prediction = odom.copy() if self.pose is None else compose(self.pose, relative(self.last_odom, odom))
        lock_yaw = self.config.get('imu_yaw_locked', False)
        if lock_yaw:
            prediction[2] = odom[2]
        points = scan.points()
        estimate = prediction.copy()
        residual = 0.06
        if self.config.get('scan_match', True) and self.tree is not None and (len(points) > 25):
            for _ in range(4):
                world = transform(points, estimate)
                distance, index = self.tree.query(world)
                keep = distance < 0.32
                if keep.sum() < 25:
                    break
                source = world[keep]
                target = self.tree.data[index[keep]]
                errors = target - source
                residual = float(np.sqrt(np.mean(distance[keep] ** 2)))
                offset = source - np.asarray(estimate[:2])
                jac = np.zeros((len(source) * 2, 3))
                jac[::2, 0] = 1
                jac[1::2, 1] = 1
                jac[::2, 2] = -offset[:, 1]
                jac[1::2, 2] = offset[:, 0]
                change = np.array([*errors.mean(axis=0), 0.0]) if lock_yaw else np.linalg.lstsq(jac, errors.ravel(), rcond=None)[0]
                change[:2] = np.clip(change[:2], -0.04, 0.04)
                change[2] = np.clip(change[2], -0.004, 0.004)
                estimate[:2] += change[:2] * 0.6
                estimate[2] += change[2] * 0.6
                if np.linalg.norm(change) < 0.002:
                    break
            if np.linalg.norm(estimate[:2] - prediction[:2]) > 0.12 or abs(wrap(estimate[2] - prediction[2])) > 0.025:
                estimate = prediction
        world = transform(points, estimate)
        moving = dynamics.update(world, scan.stamp)
        self.grid.integrate(scan, estimate, moving)
        self.pose = estimate
        self.last_odom = odom.copy()
        self.scans += 1
        self.sigma = max(0.02, min(0.08, residual / math.sqrt(max(1, len(points))) * 4))
        if self.scans % 8 == 0:
            dynamics.suppress_static(self.grid)
            occupied = self.grid.occupied_points()
            self.tree = cKDTree(occupied) if len(occupied) > 25 else None
        self.history.append((scan.stamp, estimate.copy()))
        self.processing_ms.append((time.perf_counter() - start) * 1000)
        return estimate.copy()

    def current(self, odom):
        pose = np.asarray(odom).copy() if self.pose is None else compose(self.pose, relative(self.last_odom, odom))
        if self.config.get('imu_yaw_locked', False):
            pose[2] = np.asarray(odom)[2]
        return pose

class CollisionGrid:

    def __init__(self, grid, length, width, margin):
        self.grid = grid
        self.length = length
        self.width = width
        self.margin = margin
        free, occupied = grid.masks()
        self.free = free
        self.occupied = occupied
        self.distance = distance_transform_edt(free) * grid.resolution
        self.radius = math.hypot(length / 2 + margin, width / 2 + margin)
        step = grid.resolution
        xs = np.arange(-length / 2 - margin, length / 2 + margin + step / 2, step)
        ys = np.arange(-width / 2 - margin, width / 2 + margin + step / 2, step)
        self.local = np.stack(np.meshgrid(xs, ys), axis=-1).reshape(-1, 2)

    def footprint(self, pose, details=False):
        cell = self.grid.cells(np.asarray(pose[:2]).reshape(1, 2))[0]
        if not (0 <= cell[0] < self.grid.width and 0 <= cell[1] < self.grid.height):
            return (False, 0.0, 'OUTSIDE_MAP') if details else False
        if self.distance[cell[1], cell[0]] > self.radius + self.grid.resolution:
            return (True, 1.0, 'CLEAR') if details else True
        points = transform(self.local, pose)
        cells = self.grid.cells(points)
        inside = self.grid.in_bounds(cells)
        if not inside.all():
            return (False, float(inside.mean()), 'OUTSIDE_MAP') if details else False
        good = self.free[cells[:, 1], cells[:, 0]]
        coverage = float(good.mean())
        occupied = self.occupied[cells[:, 1], cells[:, 0]]
        ok = bool(good.all())
        reason = 'OCCUPIED_FOOTPRINT' if occupied.any() else 'UNKNOWN_FOOTPRINT' if not ok else 'CLEAR'
        return (ok, coverage, reason) if details else ok

    def path(self, path):
        for i, pose in enumerate(path):
            if not self.footprint(pose):
                return (False, i)
        return (True, None)
