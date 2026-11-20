import math
import numpy as np
from .geometry import wrap, exploration_route, bicycle

class StreamWatchdog:

    def __init__(self, names, timeout=15):
        self.names = set(names)
        self.samples = {}
        self.bad_since = None
        self.last_sim = None
        self.timeout = timeout

    def feed(self, name, stamp, wall, valid=True):
        self.samples[name] = (stamp, wall, valid)

    def ready(self, sim, wall):
        if self.last_sim is not None and sim < self.last_sim - 0.05:
            raise ValueError('Simulation clock reset; restart the mission')
        self.last_sim = sim
        good = self.names <= self.samples.keys() and all((valid and -0.1 <= sim - stamp < 0.75 and (wall - arrival < 15) for name, (stamp, arrival, valid) in self.samples.items() if name in self.names))
        if good:
            self.bad_since = None
            return True
        if self.bad_since is None:
            self.bad_since = wall
        if wall - self.bad_since > self.timeout:
            raise ValueError('Missing, invalid or stale sensor stream')
        return False

class PathTracker:

    def __init__(self, path, robot, direction=1):
        self.path = np.asarray(path)
        self.robot = robot
        self.direction = direction
        self.index = 0
        self.complete = False
        self.rear = self.path[:, :2] - robot['wheelbase'] / 2 * np.column_stack((np.cos(self.path[:, 2]), np.sin(self.path[:, 2])))

    def step(self, pose):
        if self.complete:
            return (0.0, 0.0)
        wb = self.robot['wheelbase']
        rear = np.asarray(pose[:2]) - wb / 2 * np.array([math.cos(pose[2]), math.sin(pose[2])])
        end = min(len(self.path), self.index + 60)
        nearest = self.index + int(np.argmin(np.linalg.norm(self.rear[self.index:end] - rear, axis=1)))
        self.index = max(self.index, nearest)
        remaining = float(np.linalg.norm(rear - self.rear[-1]))
        heading = abs(wrap(pose[2] - self.path[-1, 2]))
        if self.index >= len(self.path) - 5 and remaining < 0.045 and (heading < math.radians(3)):
            self.complete = True
            return (0.0, 0.0)
        if self.index >= len(self.path) - 3 and remaining < 0.12 and (heading > math.radians(7)):
            raise ValueError('Endpoint heading outside tolerance')
        target = self.index
        lookahead = 0.9 if self.direction > 0 else 0.75
        while target < len(self.path) - 1 and np.linalg.norm(self.rear[target] - rear) < lookahead:
            target += 1
        delta = self.rear[target] - rear
        lateral = -math.sin(pose[2]) * delta[0] + math.cos(pose[2]) * delta[1]
        curvature = float(np.clip(2 * lateral / max(float(delta @ delta), 0.01), -self.robot['curvature_limit'], self.robot['curvature_limit']))
        maximum = self.robot['max_forward_speed'] if self.direction > 0 else self.robot['max_reverse_speed']
        speed = maximum * (0.76 if abs(curvature) > 0.08 else 1.0)
        if self.index >= len(self.path) - 60:
            speed = min(speed, max(0.035, 0.6 * remaining))
        return (self.direction * speed, self.direction * speed * curvature)

    def deviation(self, pose):
        return float(np.linalg.norm(self.path[self.index, :2] - pose[:2]))

class ObstacleGuard:

    def __init__(self, robot, timeout=45):
        self.robot = robot
        self.timeout = timeout
        self.since = None
        self.events = []
        self.blocking_point = None

    def check(self, cloud, v, w, now):
        if abs(v) < 1e-08:
            return False
        g = self.robot
        horizon = 0.5 + abs(v) * 0.4 + v * v / 0.8
        k = w / v
        cloud = np.asarray(cloud).reshape(-1, 2)
        radius = math.hypot(g['length'] / 2 + 0.18, g['width'] / 2 + 0.18) + horizon * 1.3
        cloud = cloud[np.sum(cloud ** 2, axis=1) < radius ** 2]
        hit = False
        self.blocking_point = None
        for d in np.linspace(0, math.copysign(horizon, v), max(3, math.ceil(horizon / 0.05))):
            p = bicycle([0, 0, 0], d, k, g['wheelbase'])
            dx = cloud[:, 0] - p[0]
            dy = cloud[:, 1] - p[1]
            c, s = (math.cos(p[2]), math.sin(p[2]))
            inside = (abs(c * dx + s * dy) < g['length'] / 2 + 0.18) & (abs(-s * dx + c * dy) < g['width'] / 2 + 0.18)
            if np.any(inside):
                hit = True
                self.blocking_point = cloud[np.flatnonzero(inside)[0]].tolist()
                break
        if hit:
            if self.since is None:
                self.since = now
                self.events.append(('obstacle_stop', now))
            if now - self.since > self.timeout:
                raise ValueError('Obstacle did not clear within the stopping budget')
        elif self.since is not None:
            self.events.append(('obstacle_clear', now))
            self.since = None
        return hit

class Mission:

    def __init__(self, config, slam, dynamics):
        self.config = config
        self.slam = slam
        self.dynamics = dynamics
        self.path, self.entry_end, self.lap_ranges = exploration_route(config)
        self.tracker = PathTracker(self.path, config['robot'])
        self.guard = ObstacleGuard(config['robot'], config['safety']['obstacle_timeout'])
        self.state = 'OBSERVE'
        self.events = []
        self.laps = 0
        self.first_time = None
        self.stop_time = None
        self.decisions = []
        self.candidates = []
        self.plan = None
        self.goal = None
        self.completed = False
        self.assessment = None
        self.assessment_grid = None
        self.observation_history = None
        self.recorded = []
        self.last_recorded = None
        self.progress_time = None
        self.progress_pose = None
        self.last_stop_reason = None
        self.guard_event_count = 0

    def event(self, name, now, **details):
        self.events.append({'event': name, 'time_s': round(now, 4), **details})

    def step(self, pose, cloud, now, speed=0.0):
        self.last_stop_reason = None
        if self.first_time is None:
            self.first_time = now
            self.event('mission_started', now)
        if now - self.first_time > self.config['safety']['mission_timeout']:
            raise ValueError('Mission time budget exceeded')
        if self.completed:
            return (0.0, 0.0)
        if self.last_recorded is None or math.dist(pose[:2], self.last_recorded[:2]) > 0.06:
            self.recorded.append(list(pose))
            self.last_recorded = list(pose)
        if self.state == 'OBSERVE':
            v, w = self.tracker.step(pose)
            while self.laps < len(self.lap_ranges) and self.tracker.index >= self.lap_ranges[self.laps][1]:
                self.laps += 1
                self.event('lap_complete', now, lap=self.laps)
            if self.tracker.complete:
                while self.laps < len(self.lap_ranges):
                    self.laps += 1
                    self.event('lap_complete', now, lap=self.laps)
                self.state = 'SETTLE'
                self.stop_time = now
                self.event('mapping_complete', now, scans=self.slam.scans)
                return (0.0, 0.0)
        elif self.state == 'SETTLE':
            self.last_stop_reason = 'SETTLING_BEFORE_ASSESSMENT'
            if now - self.stop_time < 1.5 or abs(speed) > 0.035:
                return (0.0, 0.0)
            self.assess(pose, now)
            return (0.0, 0.0)
        elif self.state == 'PARK':
            v, w = self.tracker.step(pose)
            if self.tracker.complete:
                self.state = 'HOLD'
                self.stop_time = now
                self.event('parking_target_reached', now)
                return (0.0, 0.0)
        elif self.state == 'HOLD':
            self.last_stop_reason = 'STATIONARY_PARKING_HOLD'
            if math.dist(pose[:2], self.goal[:2]) > 0.2 or abs(wrap(pose[2] - self.goal[2])) > math.radians(5):
                raise ValueError('Parked pose moved outside tolerance during hold')
            if now - self.stop_time >= self.config['safety']['hold_s'] and abs(speed) < 0.025:
                self.completed = True
                self.state = 'COMPLETE'
                self.last_stop_reason = 'PARKING_COMPLETE'
                self.event('parked_and_held', now)
            return (0.0, 0.0)
        else:
            return (0.0, 0.0)
        if self.tracker.deviation(pose) > 0.7:
            raise ValueError('Tracking deviation exceeds the allowed corridor')
        blocked = self.guard.check(cloud, v, w, now)
        for name, timestamp in self.guard.events[self.guard_event_count:]:
            self.event(name, timestamp, blocking_point_body_m=self.guard.blocking_point)
        self.guard_event_count = len(self.guard.events)
        if blocked:
            self.last_stop_reason = 'OBSTACLE_IN_STOPPING_PATH'
            self.progress_time = now
            return (0.0, 0.0)
        if self.progress_pose is None or math.dist(pose[:2], self.progress_pose) > 0.05:
            self.progress_pose = list(pose[:2])
            self.progress_time = now
        elif self.progress_time is not None and now - self.progress_time > 15:
            raise ValueError('No measured motion despite a drive command')
        return (v, w)

    def assess(self, pose, now):
        from .perception import CandidateGenerator, Assessor
        from .planning import ReversePlanner
        from .mapping import CollisionGrid, OccupancyGrid
        self.dynamics.suppress_static(self.slam.grid)
        g = self.config['robot']
        margin = self.config['planning']['margin'] + min(0.08, 2 * self.slam.sigma)
        frozen = OccupancyGrid(self.config['mapping'])
        frozen.logodds[:] = self.slam.grid.logodds
        frozen.observed[:] = self.slam.grid.observed
        frozen.hits[:] = self.slam.grid.hits
        frozen.revision = self.slam.grid.revision
        self.assessment_grid = frozen
        self.observation_history = np.asarray(self.recorded).copy()
        collision = CollisionGrid(frozen, g['length'], g['width'], margin)
        generator = CandidateGenerator(self.config)
        self.candidates = generator.generate(frozen, self.path, self.entry_end)
        planner = ReversePlanner(self.config, collision)
        assessor = Assessor(self.config, collision, planner, self.dynamics)
        self.assessment = {'map_revision': self.slam.grid.revision, 'margin_m': margin, 'candidate_count': len(self.candidates)}
        for candidate in self.candidates:
            decision = assessor.assess(candidate, pose, np.asarray(self.recorded), now)
            self.decisions.append(decision)
            self.event('candidate_decision', now, candidate=candidate['id'], decision=decision['decision'], reason=decision['reason'])
            if decision['decision'] == 'ACCEPT':
                self.plan = decision.pop('_path')
                self.goal = decision['goal_pose']
                self.tracker = PathTracker(self.plan, g, -1)
                self.state = 'PARK'
                self.progress_pose = None
                self.progress_time = None
                self.event('reverse_parking_started', now, candidate=candidate['id'], samples=len(self.plan))
                return
        self.state = 'NO_SAFE_SPACE'
        self.event('no_validated_parking_space', now)
        raise ValueError('No candidate passed observed-space, geometry, dynamic and manoeuvre checks')
