import heapq
import math
import time
import numpy as np
from .geometry import bicycle, wrap, sample_line
from .control import PathTracker

class ReversePlanner:

    def __init__(self, config, collision):
        self.config = config
        self.collision = collision
        self.robot = config['robot']
        self.stats = {}

    def quarter(self, goal, radius, side):
        wb = self.robot['wheelbase']
        h = goal[2] + side * math.pi / 2
        r = side * radius
        rear = np.asarray(goal[:2]) - wb / 2 * np.array([math.cos(goal[2]), math.sin(goal[2])])
        start_rear = rear - np.array([r * (math.sin(goal[2]) - math.sin(h)), -r * (math.cos(goal[2]) - math.cos(h))])
        start = np.array([*start_rear + wb / 2 * np.array([math.cos(h), math.sin(h)]), h])
        n = math.ceil(radius * math.pi / 2 / 0.04)
        return np.asarray([bicycle(start, -radius * math.pi / 2 * i / n, 1 / r, wb) for i in range(n + 1)])

    def swept(self, path):
        if not len(path):
            return False
        corner = math.hypot(self.robot['length'] / 2, self.robot['width'] / 2)
        for a, b in zip(path, path[1:]):
            motion = math.dist(a[:2], b[:2]) + corner * abs(wrap(b[2] - a[2]))
            n = max(1, math.ceil(motion / (self.collision.grid.resolution * 0.45)))
            for i in range(n):
                p = np.array([a[0] + (b[0] - a[0]) * i / n, a[1] + (b[1] - a[1]) * i / n, a[2] + wrap(b[2] - a[2]) * i / n])
                if not self.collision.footprint(p):
                    return False
        return self.collision.footprint(path[-1])

    def refine(self, current, reference):
        tracker = PathTracker(reference, self.robot, -1)
        pose = np.asarray(current).copy()
        out = [pose.copy()]
        max_steps = int(np.linalg.norm(np.diff(reference[:, :2], axis=0), axis=1).sum() / 0.025) + 600
        for _ in range(max_steps):
            v, w = tracker.step(pose)
            if tracker.complete:
                return np.asarray(out)
            if tracker.deviation(pose) > 0.55:
                return None
            d = -min(0.055, max(0.008, abs(v) * 0.2))
            k = w / v if abs(v) > 1e-09 else 0
            pose = bicycle(pose, d, k, self.robot['wheelbase'])
            out.append(pose.copy())
        return None

    def historical_connector(self, current, goal, history):
        if len(history) < 10:
            return None
        h = history
        for side in [-1, 1]:
            heading = goal[2] + side * math.pi / 2
            mask = np.array([abs(wrap(x - heading)) < 0.06 for x in h[:, 2]])
            near = h[mask]
            if len(near) < 5:
                continue
            lane = float(np.median(near[np.argsort(np.linalg.norm(near[:, :2] - goal[:2], axis=1))[:20], 1]))
            radius = abs(goal[1] - lane) + self.robot['wheelbase'] / 2
            radii = [radius, 5.5, 5.0, 6.0, 6.5, 7.0, 7.5, 8.0]
            for radius in radii:
                if radius < 1 / self.robot['curvature_limit'] * 1.02 or radius > 9:
                    continue
                arc = self.quarter(goal, radius, side)
                setup = arc[0]
                distance = np.linalg.norm(h[:, :2] - setup[:2], axis=1)
                angles = np.array([abs(wrap(x - setup[2])) for x in h[:, 2]])
                costs = distance + angles * 3
                ids = np.flatnonzero((distance < 0.3) & (angles < 0.06) & (costs < costs.min() + 0.1))
                if not len(ids):
                    continue
                index = int(ids[-1])
                tail = h[index:][::-1].copy()
                tail = np.vstack((current, tail[1:]))
                if np.linalg.norm(tail[-1, :2] - setup[:2]) > 0.01:
                    tail = np.vstack((tail, sample_line(tail[-1], setup, 0.04)))
                reference = np.vstack((tail, arc[1:]))
                path = self.refine(current, reference)
                if path is not None and self.swept(path):
                    return path
        return None

    def plan(self, current, goal, history):
        started = time.perf_counter()
        path = self.historical_connector(current, np.asarray(goal), np.asarray(history))
        if path is not None:
            return {'path': path, 'status': 'VALIDATED_REVERSE_HISTORY_AND_ANALYTIC_ARC', 'time_s': time.perf_counter() - started}
        path, status = self.hybrid(current, goal, started)
        return {'path': path, 'status': status, 'time_s': time.perf_counter() - started}

    def hybrid(self, start, goal, started):
        settings = self.config['planning']
        wb = self.robot['wheelbase']
        kmax = self.robot['curvature_limit']
        step = settings['step_m']
        start = np.asarray(start)
        goal = np.asarray(goal)
        queue = []
        nodes = [(start, None, 0.0)]
        best = {}
        counter = 0

        def key(p):
            return (round(p[0] / 0.3), round(p[1] / 0.3), round(wrap(p[2]) / math.radians(10)))

        def heuristic(p):
            return math.dist(p[:2], goal[:2]) + 2 * abs(wrap(p[2] - goal[2]))
        heapq.heappush(queue, (heuristic(start), counter, 0))
        best[key(start)] = 0
        while queue and len(nodes) < settings['max_nodes']:
            if time.perf_counter() - started > settings['timeout_s']:
                return (None, 'SEARCH_TIME_BUDGET')
            _, _, idx = heapq.heappop(queue)
            pose, parent, cost = nodes[idx]
            if cost > best.get(key(pose), math.inf) + 1e-06:
                continue
            if math.dist(pose[:2], goal[:2]) < 0.18 and abs(wrap(pose[2] - goal[2])) < math.radians(5):
                chain = []
                while idx is not None:
                    p, prev, c = nodes[idx]
                    chain.append(p)
                    idx = prev
                chain.reverse()
                path = np.asarray(chain)
                if self.swept(path):
                    return (path, 'VALIDATED_REVERSE_HYBRID_ASTAR')
            for curvature in [-kmax, -kmax / 2, 0, kmax / 2, kmax]:
                endpoint = bicycle(pose, -step, curvature, wb)
                cost_new = cost + step * (1 + 0.15 * abs(curvature) / kmax)
                k = key(endpoint)
                if cost_new >= best.get(k, math.inf):
                    continue
                samples = [bicycle(pose, -step * i / 10, curvature, wb) for i in range(1, 11)]
                if not all((self.collision.footprint(p) for p in samples)):
                    continue
                best[k] = cost_new
                counter += 1
                nodes.append((endpoint, idx, cost_new))
                heapq.heappush(queue, (cost_new + heuristic(endpoint), counter, len(nodes) - 1))
        return (None, 'SEARCH_NODE_BUDGET' if queue else 'NO_REVERSE_PATH_IN_SEARCH_GRAPH')
