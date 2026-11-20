import math
import time
import numpy as np

class CandidateGenerator:

    def __init__(self, config):
        self.config = config

    def generate(self, grid, route, entry_end):
        cfg = self.config['perception']
        free, occupied = grid.masks()
        r = grid.resolution
        records = []
        for side in [-1, 1]:
            depths = np.arange(cfg['outer_band_min'], cfg['outer_band_max'], r)
            for depth in depths:
                y = side * depth
                row = grid.cells([[0, y]])[0, 1]
                known_free = free[row].copy()
                runs = []
                start = None
                for i, flag in enumerate(known_free):
                    if flag and start is None:
                        start = i
                    if start is not None and (not flag or i == len(known_free) - 1):
                        end = i if not flag else i + 1
                        if start > 0 and end < len(known_free):
                            width = (end - start) * r
                            if cfg['minimum_opening'] <= width <= cfg['maximum_opening']:
                                x = grid.origin[0] + (start + end) * r / 2
                                if -25 < x < 24:
                                    runs.append((x, depth, width, side))
                        start = None
                records.extend(runs)
        clusters = []
        for side in [-1, 1]:
            rows = {}
            for record in records:
                if record[3] == side:
                    rows.setdefault(round(record[1] / r), []).append(record)
            nodes = []
            parents = []
            previous = []
            previous_row = None

            def find(i):
                while parents[i] != i:
                    parents[i] = parents[parents[i]]
                    i = parents[i]
                return i
            for row, items in sorted(rows.items()):
                current = []
                for item in items:
                    idx = len(nodes)
                    nodes.append(item)
                    parents.append(idx)
                    current.append(idx)
                    if previous_row is not None and row - previous_row <= 2:
                        for old in previous:
                            other = nodes[old]
                            overlap = min(item[0] + item[2] / 2, other[0] + other[2] / 2) - max(item[0] - item[2] / 2, other[0] - other[2] / 2)
                            if overlap > min(1.0, 0.4 * min(item[2], other[2])):
                                parents[find(idx)] = find(old)
                previous = current
                previous_row = row
            components = {}
            for i, item in enumerate(nodes):
                components.setdefault(find(i), []).append(item)
            for items in components.values():
                by_depth = {}
                for item in items:
                    key = round(item[1] / r)
                    if key not in by_depth or item[2] > by_depth[key][2]:
                        by_depth[key] = item
                points = list(by_depth.values())
                if len(points) / (1 + np.ptp([p[1] for p in points]) / r) > 0.65:
                    clusters.append(points)
        proposals = []
        g = self.config['robot']
        for points in clusters:
            depths = np.array([p[1] for p in points])
            xs = np.array([p[0] for p in points])
            widths = np.array([p[2] for p in points])
            side = points[0][3]
            if np.ptp(depths) < cfg['minimum_depth_support'] or len(points) < 6:
                continue
            central = (depths >= np.quantile(depths, 0.15)) & (depths <= np.quantile(depths, 0.85))
            x = float(np.median(xs[central]))
            depth = float((np.min(depths) + np.max(depths)) / 2)
            if not -23 < x < 22:
                continue
            heading = -side * math.pi / 2
            centre = np.array([x, side * depth])
            first_lap = route[entry_end:]
            distance = np.linalg.norm(first_lap[:, :2] - centre, axis=1)
            near = np.flatnonzero(distance < distance.min() + 0.2)
            order = int(near[0]) + entry_end
            proposals.append({'id': '', 'pose': [x, side * depth, heading], 'side': side, 'opening_width_m': float(np.median(widths)), 'depth_support_m': float(np.ptp(depths)), 'map_revision': grid.revision, 'route_index': order, 'support_points': len(points), 'depth_bounds': [float(np.min(depths)), float(np.max(depths))]})
        proposals.sort(key=lambda c: c['route_index'])
        for i, c in enumerate(proposals, 1):
            c['id'] = f'P{i:02d}'
        return proposals

class Assessor:

    def __init__(self, config, collision, planner, dynamics):
        self.config = config
        self.collision = collision
        self.planner = planner
        self.dynamics = dynamics

    def endpoint_options(self, candidate):
        pose = candidate['pose']
        lo, hi = candidate['depth_bounds']
        side = candidate['side']
        depth_center = (lo + hi) / 2
        lower = lo + self.config['robot']['length'] / 2 + 0.05
        upper = hi - self.config['robot']['length'] / 2 - 0.05
        if lower > upper:
            lower = depth_center - 0.2
            upper = depth_center + 0.2
        depths = list(np.arange(lower, upper + 0.025, 0.1))
        depths.sort(key=lambda value: abs(value - depth_center))
        options = []
        for depth in depths:
            for dx in [0, -0.15, 0.15]:
                options.append([pose[0] + dx, side * float(depth), pose[2]])
        return options

    def assess(self, candidate, current, history, now):
        started = time.perf_counter()
        options = self.endpoint_options(candidate)
        valid = []
        coverage = 0.0
        reasons = []
        for pose in options:
            ok, seen, reason = self.collision.footprint(pose, True)
            coverage = max(coverage, seen)
            reasons.append(reason)
            if ok:
                valid.append(pose)
        result = {'candidate_id': candidate['id'], 'detected_pose': candidate['pose'], 'map_revision': candidate['map_revision'], 'opening_width_m': candidate['opening_width_m'], 'endpoint_baseline_accept': bool(valid), 'manoeuvre_accept': False, 'dynamic_guard_clear': True, 'decision': 'REJECT', 'reason': '', 'goal_pose': valid[0] if valid else candidate['pose'], 'observed_free_fraction': round(coverage, 5), 'planning_status': 'NOT_ATTEMPTED'}
        if not valid:
            result['reason'] = 'INSUFFICIENT_WIDTH' if candidate['opening_width_m'] < self.config['robot']['width'] + 2 * self.collision.margin else 'FINAL_FOOTPRINT_OR_ORIENTATION' if 'OCCUPIED_FOOTPRINT' in reasons else 'INSUFFICIENT_OBSERVATION'
        else:
            lane = self.config['exploration']['bottom' if candidate['side'] < 0 else 'top']
            goal = valid[0]
            entry = np.asarray([[goal[0], y, goal[2]] for y in np.linspace(lane, goal[1], 40)])
            if self.dynamics.conflict(entry, self.config['robot']['length'], self.config['robot']['width']):
                result['dynamic_guard_clear'] = False
                result['reason'] = 'DYNAMIC_OCCUPANCY_REASSESS_LATER'
                result['temporary_rejection'] = True
                result['planning_status'] = 'DEFERRED_UNTIL_FRESH_DYNAMIC_CLEARANCE'
                result['assessment_ms'] = round((time.perf_counter() - started) * 1000, 3)
                return result
            for goal in valid[:1]:
                solution = self.planner.plan(current, goal, history)
                if solution['path'] is not None:
                    path = solution['path']
                    result['goal_pose'] = goal
                    result['manoeuvre_accept'] = True
                    result['planning_status'] = solution['status']
                    result['path_length_m'] = float(np.linalg.norm(np.diff(path[:, :2], axis=0), axis=1).sum())
                    dynamic = self.dynamics.conflict(path, self.config['robot']['length'], self.config['robot']['width'])
                    result['dynamic_guard_clear'] = not dynamic
                    if dynamic:
                        result['reason'] = 'DYNAMIC_OCCUPANCY_REASSESS_LATER'
                        result['temporary_rejection'] = True
                    else:
                        result['decision'] = 'ACCEPT'
                        result['reason'] = 'OBSERVED_FOOTPRINT_AND_REVERSE_PATH_CLEAR'
                        result['_path'] = path
                    break
                result['planning_status'] = solution['status']
            if not result['manoeuvre_accept']:
                result['reason'] = 'NO_VALIDATED_MANOEUVRE'
        result['assessment_ms'] = round((time.perf_counter() - started) * 1000, 3)
        return result
