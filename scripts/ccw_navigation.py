import math

def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi

def make_paths(spawn, goal, wb, radius, scale):
    points = [list(spawn)]

    def line(x, y):
        a, b, h = points[-1]
        n = max(1, math.ceil(math.hypot(x - a, y - b) / (0.08 * scale)))
        points.extend([[a + (x - a) * i / n, b + (y - b) * i / n, h] for i in range(1, n + 1)])

    def arc(r, angle):
        x, y, h = points[-1]
        rx = x - wb / 2 * math.cos(h)
        ry = y - wb / 2 * math.sin(h)
        n = max(30, math.ceil(abs(r * angle) / (0.08 * scale)))
        for i in range(1, n + 1):
            t = h + angle * i / n
            points.append([rx + r * (math.sin(t) - math.sin(h)) + wb / 2 * math.cos(t), ry - r * (math.cos(t) - math.cos(h)) + wb / 2 * math.sin(t), t])
    west, east, top = (-21 * scale, 21 * scale, 7.8 * scale)
    bottom = goal[1] - wb / 2 + radius
    anchor = [goal[0] + radius + wb / 2, bottom, 0]
    if top - bottom < 2 * radius:
        raise ValueError('The lap radius does not fit the aisles')
    if abs(wrap(spawn[2])) > 1e-06:
        raise ValueError('Entrance start must face east')
    drop = spawn[1] - bottom
    if drop < 0:
        raise ValueError('Entrance approach expects a downward lane change')
    if drop > 1e-08:
        entry_radius = 20 * scale
        angle = math.acos(1 - drop / (2 * entry_radius))
        arc(-entry_radius, -angle)
        arc(entry_radius, angle)
    line(anchor[0], anchor[1])
    entry_end = len(points) - 1
    lap_ranges = []
    for lap in range(2):
        begin = len(points) - 1
        line(east - radius + wb / 2, bottom)
        arc(radius, math.pi / 2)
        line(east, top - radius + wb / 2)
        arc(radius, math.pi / 2)
        line(west + radius - wb / 2, top)
        arc(radius, math.pi / 2)
        line(west, bottom + radius - wb / 2)
        arc(radius, math.pi / 2)
        line(anchor[0], bottom)
        lap_ranges.append([begin, len(points) - 1])
    forward = points[:]
    points = [points[-1]]
    arc(-radius, math.pi / 2)
    reverse = points[:]
    return (forward, reverse, {'entry_end_index': entry_end, 'lap_ranges': lap_ranges, 'lap_anchor_m_rad': anchor})

def steering_command(body, path, index, direction, wb, scale, kmax):
    x, y, yaw = body
    rear = (x - wb / 2 * math.cos(yaw), y - wb / 2 * math.sin(yaw))

    def axle(p):
        return (p[0] - wb / 2 * math.cos(p[2]), p[1] - wb / 2 * math.sin(p[2]))
    end = min(len(path), index + 60)
    index = min(range(index, end), key=lambda i: math.dist(rear, axle(path[i])))
    target = index
    while target < len(path) - 1 and math.dist(rear, axle(path[target])) < 1.0 * scale:
        target += 1
    tx, ty = axle(path[target])
    dx, dy = (tx - rear[0], ty - rear[1])
    lateral = -math.sin(yaw) * dx + math.cos(yaw) * dy
    curvature = max(-kmax, min(kmax, 2 * lateral / max(dx * dx + dy * dy, 0.01 * scale * scale)))
    remaining = math.dist(rear, axle(path[-1]))
    speed_remaining = remaining if index >= len(path) - 60 else math.inf
    speed = min(0.5 * scale if direction > 0 else 0.25 * scale, max(0.06 * scale, 0.6 * speed_remaining))
    if abs(curvature) > 0.08 / scale:
        speed *= 0.75
    return (direction * speed, direction * speed * curvature, index, remaining)

def blocked(points, v, w, wb, length, width, scale):
    if abs(v) < 1e-06:
        return False
    horizon = 0.6 * scale + abs(v) * 0.4 + v * v / (0.8 * scale)
    count = max(2, math.ceil(horizon / (0.05 * scale)))
    k = w / v
    half_length, half_width = (length / 2 + 0.18 * scale, width / 2 + 0.18 * scale)
    radius = math.hypot(half_length, half_width) + horizon * (1 + abs(k) * wb / 2)
    points = [(x, y) for x, y in points if x * x + y * y <= radius * radius]
    for i in range(count + 1):
        s = math.copysign(horizon * i / count, v)
        a = k * s
        rx = -wb / 2 + (math.sin(a) / k if abs(k) > 1e-06 else s)
        ry = (1 - math.cos(a)) / k if abs(k) > 1e-06 else 0
        x, y = (rx + wb / 2 * math.cos(a), ry + wb / 2 * math.sin(a))
        cosine, sine = (math.cos(a), math.sin(a))
        for px, py in points:
            dx, dy = (px - x, py - y)
            if abs(cosine * dx + sine * dy) < half_length and abs(-sine * dx + cosine * dy) < half_width:
                return True
    return False

def build_mission(manifest, spawn, config):
    from scene_robot import STEERING_LIMIT, WHEEL_RADIUS
    scale = config['scale']
    geometry = {'wheelbase': manifest['wheelbase_m'], 'wheel_radius': WHEEL_RADIUS * scale, 'length': manifest['robot_footprint_m'][0], 'width': manifest['robot_footprint_m'][1], 'radius': manifest['reverse_reference_radius_m'], 'steering_limit': STEERING_LIMIT, 'scale': scale}
    if geometry['radius'] < geometry['wheelbase'] / math.sin(STEERING_LIMIT) * 1.03:
        raise ValueError('Reverse radius exceeds steering limit')
    goal = next((c['goal_pose_m_deg'] for c in manifest['cases'] if c['id'] == 'S6'))
    start = [spawn['position_m'][0], spawn['position_m'][1], spawn['yaw_rad']]
    forward, reverse, sections = make_paths(start, goal, geometry['wheelbase'], geometry['radius'], scale)
    if math.dist(reverse[-1][:2], goal[:2]) > 1e-06 or abs(wrap(reverse[-1][2] - math.radians(goal[2]))) > 1e-06:
        raise ValueError('Parking arc misses S6')
    checkpoints = []
    order = ['S2', 'S1', 'S3', 'S4', 'S5', 'S6']
    for lap, (begin, end) in enumerate(sections['lap_ranges'], 1):
        last = begin
        for case_id in order:
            case = next((c for c in manifest['cases'] if c['id'] == case_id))
            point = [case['goal_pose_m_deg'][0], sections['lap_anchor_m_rad'][1] if case['goal_pose_m_deg'][1] < 0 else 7.8 * scale]
            index = min(range(last, end + 1), key=lambda i: math.dist(forward[i][:2], point))
            checkpoints.append({'id': case_id, 'lap': lap, 'path_index': index})
            last = index + 1
    return {'purpose': 'Preset entrance approach, two complete CCW laps, then reverse into S6; no automatic space selection.', 'geometry': geometry, 'start_pose_m_rad': start, 'goal_pose_m_rad': [goal[0], goal[1], math.radians(goal[2])], 'forward': forward, 'reverse': reverse, 'checkpoints': checkpoints, 'lap_count': 2, **sections, 'laser_frames': {'/scan': {'x': 0, 'y': 0, 'yaw': 0, 'frame': 'lidar_link', 'angle_min': -math.pi, 'angle_max': math.pi}}, 'disabled_sensors': ['camera', 'contact']}

class MissionFollower:

    def __init__(self, mission):
        self.mission = mission
        self.g = mission['geometry']
        self.phase = 0
        self.index = 0
        self.hold_until = None
        self.blocked_since = None
        self.first_time = None
        self.progress_time = None
        self.progress_pose = None
        self.checkpoint_index = 0
        self.completed = False
        self.completed_laps = 0
        self.entry_completed = False
        self.events = []

    def step(self, body, cloud, now, speed=0.0):
        if self.completed:
            return (0.0, 0.0)
        if self.first_time is None:
            self.first_time = now
        if now - self.first_time > 1200:
            raise ValueError('Route exceeded 1200 simulated seconds')
        if self.hold_until is not None:
            if now < self.hold_until or abs(speed) > 0.04 * self.g['scale']:
                return (0.0, 0.0)
            self.phase = 1
            self.index = 0
            self.hold_until = None
            self.progress_pose = None
            self.events.append(('reverse_started', now))
        path = self.mission['forward' if self.phase == 0 else 'reverse']
        v, w, self.index, remaining = steering_command(body, path, self.index, 1 if self.phase == 0 else -1, self.g['wheelbase'], self.g['scale'], math.sin(self.g['steering_limit']) / self.g['wheelbase'])
        if self.phase == 0 and (not self.entry_completed) and (self.index >= self.mission['entry_end_index']):
            self.entry_completed = True
            self.events.extend([('entry_complete', now), ('lap_1_started', now)])
        while self.phase == 0 and self.checkpoint_index < len(self.mission['checkpoints']):
            checkpoint = self.mission['checkpoints'][self.checkpoint_index]
            if self.index < checkpoint['path_index']:
                break
            self.events.append(('lap_' + str(checkpoint['lap']) + '_passed_' + checkpoint['id'], now))
            self.checkpoint_index += 1
        while self.phase == 0 and self.completed_laps < len(self.mission['lap_ranges']) and (self.index >= self.mission['lap_ranges'][self.completed_laps][1]):
            self.completed_laps += 1
            self.events.append(('lap_' + str(self.completed_laps) + '_complete', now))
            if self.completed_laps < 2:
                self.events.append(('lap_2_started', now))
        if remaining < 0.12 * self.g['scale'] and self.index >= len(path) - 5:
            if abs(wrap(body[2] - path[-1][2])) > math.radians(6):
                raise ValueError('Endpoint heading needs alignment')
            if self.phase == 0:
                while self.completed_laps < 2:
                    self.completed_laps += 1
                    self.events.append(('lap_' + str(self.completed_laps) + '_complete', now))
                while self.checkpoint_index < len(self.mission['checkpoints']):
                    checkpoint = self.mission['checkpoints'][self.checkpoint_index]
                    self.events.append(('lap_' + str(checkpoint['lap']) + '_passed_' + checkpoint['id'], now))
                    self.checkpoint_index += 1
                self.hold_until = now + 1.5
                self.events.append(('stopping_before_reverse', now))
            else:
                self.completed = True
                self.events.append(('odometry_goal_reached', now))
            return (0.0, 0.0)
        if blocked(cloud, v, w, self.g['wheelbase'], self.g['length'], self.g['width'], self.g['scale']):
            if self.blocked_since is None:
                self.blocked_since = now
                self.events.append(('laser_stop', now))
            if now - self.blocked_since > 30:
                raise ValueError('Obstacle did not clear within 30 simulated seconds')
            self.progress_time = now
            return (0.0, 0.0)
        if self.blocked_since is not None:
            self.events.append(('laser_clear', now))
        self.blocked_since = None
        if self.progress_pose is None or math.dist(body[:2], self.progress_pose) > 0.05 * self.g['scale']:
            self.progress_pose = body[:2]
            self.progress_time = now
        elif now - self.progress_time > 15:
            raise ValueError('Commanded motion produced no odometry progress')
        return (v, w)

class StreamWatchdog:

    def __init__(self, names, timeout=15):
        self.names = set(names)
        self.received = {}
        self.bad_since = None
        self.last_sim = None
        self.timeout = timeout

    def feed(self, name, stamp, wall, valid=True):
        self.received[name] = (stamp, wall, valid)

    def ready(self, sim, wall):
        if self.last_sim is not None and sim < self.last_sim - 0.05:
            raise ValueError('Simulation clock reset; restart the controller')
        self.last_sim = sim
        good = self.names <= self.received.keys() and all((valid and wall - arrival < 15 and (-0.1 <= sim - stamp < 0.75) for name, (stamp, arrival, valid) in self.received.items() if name in self.names))
        if good:
            self.bad_since = None
            return True
        if self.bad_since is None:
            self.bad_since = wall
        if wall - self.bad_since > self.timeout:
            raise ValueError('Missing, stale or invalid odometry/laser stream')
        return False

class WheelImuOdometry:

    def __init__(self, start, wheelbase, wheel_radius):
        self.start = list(start)
        self.wb = wheelbase
        self.radius = wheel_radius
        self.rear = [start[0] - wheelbase / 2 * math.cos(start[2]), start[1] - wheelbase / 2 * math.sin(start[2])]
        self.body = None
        self.imu_raw = None
        self.imu_relative = 0
        self.imu_stamp = None
        self.rate = 0
        self.previous = None
        self.previous_stamp = None
        self.heading = start[2]

    def imu(self, raw_yaw, angular_rate, stamp):
        if not all((math.isfinite(v) for v in [raw_yaw, angular_rate, stamp])):
            raise ValueError('Invalid IMU sample')
        if self.imu_raw is not None:
            self.imu_relative += wrap(raw_yaw - self.imu_raw)
        self.imu_raw = raw_yaw
        self.imu_stamp = stamp
        self.rate = angular_rate

    def wheels(self, left, right, stamp):
        if self.imu_stamp is None:
            return None
        if not all((math.isfinite(v) for v in [left, right, stamp])):
            raise ValueError('Invalid rear encoder sample')
        heading = self.start[2] + self.imu_relative + self.rate * max(-0.1, min(0.1, stamp - self.imu_stamp))
        if self.previous is not None:
            dt = stamp - self.previous_stamp
            if dt < -0.01 or dt > 1.0:
                raise ValueError('Rear encoder timestamps discontinuous')
            distance = self.radius / 2 * (wrap(left - self.previous[0]) + wrap(right - self.previous[1]))
            mid = self.heading + wrap(heading - self.heading) / 2
            self.rear[0] += distance * math.cos(mid)
            self.rear[1] += distance * math.sin(mid)
        self.previous = (left, right)
        self.previous_stamp = stamp
        self.heading = heading
        self.body = [self.rear[0] + self.wb / 2 * math.cos(heading), self.rear[1] + self.wb / 2 * math.sin(heading), heading]
        return self.body
