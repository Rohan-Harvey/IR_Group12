"""Geometry, probabilistic mapping and path tracking shared by the parking nodes."""

from collections import deque
from dataclasses import dataclass
import math

import numpy as np

from patrol_route import Point, route_lengths, transform, wrap


def inverse(pose):
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    return -c*x-s*y, s*x-c*y, -yaw


def distance(a, b):
    return math.hypot(a.x-b.x, a.y-b.y)


def grid_ray(x, y, end_x, end_y):
    """Integer Bresenham traversal, including its two endpoints."""
    dx, dy = abs(end_x-x), -abs(end_y-y)
    sx, sy = (1 if x < end_x else -1), (1 if y < end_y else -1)
    error = dx+dy
    while (x, y) != (end_x, end_y):
        yield x, y
        twice = 2*error
        if twice >= dy:
            error += dy
            x += sx
        if twice <= dx:
            error += dx
            y += sy
    yield end_x, end_y


class OccupancyEvidence:
    """Scaled log odds in odom. Keep raw local evidence independent of SLAM jumps."""

    def __init__(self, resolution, limit=16):
        self.resolution = resolution
        self.limit = limit
        self.cells = {}

    def cell(self, x, y):
        return math.floor(x/self.resolution), math.floor(y/self.resolution)

    def insert(self, origin, endpoints):
        start = self.cell(*origin)
        hits, misses = set(), set()
        for x, y, hit in endpoints:
            end = self.cell(x, y)
            misses.update(grid_ray(*start, *end))
            if hit:
                hits.add(end)
        for cell in misses-hits:
            self.cells[cell] = max(-self.limit, self.cells.get(cell, 0)-2)
        for cell in hits:
            self.cells[cell] = min(self.limit, self.cells.get(cell, 0)+5)

    def snapshot(self, low=()):
        if not self.cells:
            return None
        x0 = min(x for x, _ in self.cells)
        y0 = min(y for _, y in self.cells)
        x1 = max(x for x, _ in self.cells)+1
        y1 = max(y for _, y in self.cells)+1
        odds = np.zeros((y1-y0, x1-x0), dtype=np.int8)
        seen = np.zeros_like(odds, dtype=bool)
        for (x, y), count in self.cells.items():
            odds[y-y0, x-x0] = count
            seen[y-y0, x-x0] = True
        # Low-plane hits are independent of rays that pass above a curb.
        for layer in low:
            for (x, y), count in layer.cells.items():
                if count >= 5 and x0 <= x < x1 and y0 <= y < y1:
                    odds[y-y0, x-x0] = max(odds[y-y0, x-x0], count)
                    seen[y-y0, x-x0] = True
        return OccupancySnapshot(self.resolution, (x0, y0), odds, seen)


class OccupancySnapshot:
    def __init__(self, resolution, origin, odds, seen):
        self.resolution, self.origin = resolution, origin
        self.odds, self.seen = odds, seen
        self.occupied = odds >= 4
        self.free = odds <= -2
        self.height, self.width = odds.shape

    def at(self, point):
        x, y = self.index(point.x, point.y)
        if 0 <= x < self.width and 0 <= y < self.height:
            return int(self.odds[y, x])
        return 0

    def index(self, x, y):
        return math.floor(x/self.resolution)-self.origin[0], math.floor(y/self.resolution)-self.origin[1]

    def area(self, pose, bounds):
        """Occupied cell intersection and observed-free ratio of an oriented rectangle."""
        back, front, half_width = bounds
        # Expand by the cell's half diagonal so an intersecting occupied cell
        # cannot disappear between cell centres or when the rectangle rotates.
        cell_margin = self.resolution/math.sqrt(2)
        radius = math.hypot(max(abs(back), abs(front)), half_width)+cell_margin
        x0, y0 = self.index(pose.x-radius, pose.y-radius)
        x1, y1 = self.index(pose.x+radius, pose.y+radius)
        x = (np.arange(x0, x1+1)+self.origin[0]+0.5)*self.resolution-pose.x
        y = (np.arange(y0, y1+1)+self.origin[1]+0.5)*self.resolution-pose.y
        c, s = math.cos(pose.yaw), math.sin(pose.yaw)
        longitude = c*x[None, :]+s*y[:, None]
        lateral = -s*x[None, :]+c*y[:, None]
        mask = ((back-cell_margin <= longitude) & (longitude <= front+cell_margin)
                & (np.abs(lateral) <= half_width+cell_margin))
        ix0, iy0 = max(x0, 0), max(y0, 0)
        ix1, iy1 = min(x1+1, self.width), min(y1+1, self.height)
        if ix1 <= ix0 or iy1 <= iy0:
            return False, 0.0
        valid = mask[iy0-y0:iy1-y0, ix0-x0:ix1-x0]
        occupied = np.any(valid & self.occupied[iy0:iy1, ix0:ix1])
        free = np.count_nonzero(valid & self.free[iy0:iy1, ix0:ix1])
        return bool(occupied), float(free/max(1, np.count_nonzero(mask)))

    def probabilities(self):
        data = (100/(1+np.exp(-self.odds.astype(float)/4))).round().astype(int)
        data[~self.seen] = -1
        return data.ravel().tolist()


def scan_rays(scan, pose, max_range, stride=1):
    """Yield odom endpoints. A no-return ray contributes free space only."""
    usable_range = min(scan.range_max, max_range)
    for i in range(0, len(scan.ranges), stride):
        value = scan.ranges[i]
        if not value >= scan.range_min or value == -math.inf:
            continue
        angle = pose.yaw+scan.angle_min+i*scan.angle_increment
        length = min(value, usable_range)
        yield (pose.x+math.cos(angle)*length, pose.y+math.sin(angle)*length,
               math.isfinite(value) and value < usable_range)


@dataclass(frozen=True)
class Footprint:
    back: float
    front: float
    half_width: float
    parked_half_width: float

    @classmethod
    def from_robot(cls, robot):
        center = robot["wheelbase_m"]/2+robot["footprint_center_offset_m"][0]
        half_length = robot["footprint_size_m"][0]/2
        return cls(center-half_length, center+half_length,
                   robot["steered_envelope_width_m"]/2, robot["footprint_size_m"][1]/2)

    def bounds(self, margin=0.0, parked=False):
        width = self.parked_half_width if parked else self.half_width
        return self.back-margin, self.front+margin, width+margin


class MovingReturns:
    """Estimate short-term motion of compact laser clusters in fixed odom coordinates."""

    def __init__(self, minimum_interval=0.15):
        self.tracks = []
        self.time = None
        self.minimum_interval = minimum_interval

    def update(self, hits, stamp):
        if self.time is not None and stamp-self.time < self.minimum_interval:
            return
        dt = stamp-self.time if self.time is not None else math.inf
        self.time = stamp
        groups = np.split(hits, np.where(np.linalg.norm(np.diff(hits, axis=0), axis=1) > 0.27)[0]+1)
        centers = [group.mean(axis=0) for group in groups if 3 <= len(group) <= 65
                   and 0.07 < np.linalg.norm(np.ptp(group, axis=0)) < 0.85]
        old = list(self.tracks) if dt < 0.6 else []
        tracks = []
        for center in centers:
            if old:
                nearest = min(range(len(old)),
                              key=lambda i: np.linalg.norm(center-old[i][0]-old[i][1]*dt))
                previous, previous_v, count = old[nearest]
                error = np.linalg.norm(center-previous-previous_v*dt)
                velocity = (center-previous)/dt
                if error < 0.35 and np.linalg.norm(velocity) < 2.6:
                    tracks.append((center, 0.6*previous_v+0.4*velocity, count+1))
                    old.pop(nearest)
                    continue
            tracks.append((center, np.zeros(2), 1))
        self.tracks = tracks

    def active(self, now):
        if self.time is None or not 0 <= now-self.time < 0.4:
            return None
        moving = [(center+v*(now-self.time), v) for center, v, count in self.tracks
                  if count >= 3 and 0.28 < np.linalg.norm(v) < 2.2]
        return np.asarray(moving, dtype=float).reshape(-1, 2, 2)


def crosses_surveyed_aisle(point, footprint, survey_trace):
    """Detect a known passage along the proposed parking direction."""
    c, s = math.cos(point.yaw), math.sin(point.yaw)
    parallel = math.cos(math.radians(25))
    reaches_rear = crosses_body = reaches_front = False
    for pose in survey_trace:
        if abs(math.cos(pose.yaw-point.yaw)) < parallel:
            continue
        dx, dy = pose.x-point.x, pose.y-point.y
        if abs(-s*dx+c*dy) > footprint.parked_half_width:
            continue
        along = c*dx+s*dy
        reaches_rear |= footprint.back-1.2 <= along <= footprint.back-0.20
        crosses_body |= footprint.back+0.45 <= along <= footprint.front-0.45
        reaches_front |= footprint.front+0.20 <= along <= footprint.front+1.2
        if reaches_rear and crosses_body and reaches_front:
            return True
    return False


def measured_entry_width(grid, anchor):
    """Find the narrowest observed cross-section bracketed by returns."""
    origins = (anchor.x, anchor.y, anchor.yaw)
    step = grid.resolution
    half_samples = math.floor(4.2/step)
    widths = []
    # Probe the mouth and its approach, before the final footprint area.
    for depth in np.arange(2.0, 4.25, step):
        section = [grid.at(transform(Point(i*step, -depth, 0, 0), origins))
                   for i in range(-half_samples, half_samples+1)]
        left = next((i for i in range(half_samples-1, -1, -1) if section[i] >= 4), None)
        right = next((i for i in range(half_samples+1, len(section)) if section[i] >= 4), None)
        if left is None or right is None:
            continue
        gap = section[left+1:right]
        if sum(value <= -2 for value in gap) >= 0.70*len(gap):
            widths.append(len(gap)*step)
    # Require two observed sections to avoid relying on one stray cell.
    return sorted(widths)[1] if len(widths) >= 2 else None


def slot_score(grid, point, footprint, margin, survey_trace=()):
    if grid.at(point) > -2:
        return None
    occupied, free = grid.area(point, footprint.bounds(margin, parked=True))
    if occupied or free < 0.94:
        return None
    origin = (point.x, point.y, point.yaw)
    supports, shoulders = [], []
    for side in (-1, 1):
        nearest = next(
            (offset for offset in (0.85, 1.65, 2.45, 3.25, 4.05)
             if grid.area(transform(Point(0.30, side*(footprint.parked_half_width+offset), 0, 0),
                                     origin), (-0.58, 0.95, 0.52))[0]), None)
        if nearest is None:
            return None
        supports.append(nearest)
        flank = side*(footprint.parked_half_width+nearest)
        # Require observable lane-facing ends. Otherwise a long wall
        # alongside a single parked car can look like a whole parking bay.
        leading = next(
            (float(offset) for offset in np.arange(1.60, -0.60, -0.20)
             if grid.area(transform(Point(footprint.front+offset, flank, 0, 0), origin),
                          (-0.08, 0.08, 0.50))[0]), None)
        if leading is None or not -0.15 <= leading <= 1.20:
            return None
        # A perpendicular bay is bracketed by relatively long side surfaces.
        # A short car end viewed from a cross-aisle does not supply that
        # evidence. Allow small gaps for wheels and partial lidar occlusion.
        skips, side_length = 0, 0.0
        for length in np.arange(0.20, 3.01, 0.20):
            sample = transform(Point(footprint.front+leading-length, flank, 0, 0), origin)
            if grid.area(sample, (-0.11, 0.11, 0.53))[0]:
                side_length = length
                skips = 0
            else:
                skips += 1
                if skips == 3:
                    break
        if side_length < 2.19:
            return None
        shoulders.append(leading)
    if abs(shoulders[0]-shoulders[1]) > 1.0:
        return None
    # The two facing rows of a travel aisle can look like slot neighbors.
    # A known free run-out at both ends, or a previously driven through
    # route, excludes the space even when the footprint and parking path fit.
    extensions = ((footprint.back-2.3, footprint.back),
                  (footprint.front, footprint.front+2.3))
    for back, front in extensions:
        occupied, observed = grid.area(point, (back, front, footprint.parked_half_width))
        if occupied or observed < 0.94:
            break
    else:
        return None
    if crosses_surveyed_aisle(point, footprint, survey_trace):
        return None
    return free-0.023*abs(supports[0]-supports[1])


def surveyed_bay_score(grid, point, footprint, margin, survey_trace=()):
    """Recognize a surveyed perpendicular bay when flank ends are partly hidden."""
    occupied, free = grid.area(point, footprint.bounds(margin, parked=True))
    if occupied or free < 0.94 or crosses_surveyed_aisle(point, footprint, survey_trace):
        return None
    origin = (point.x, point.y, point.yaw)
    near = footprint.parked_half_width+0.45
    far = near+3.45
    for side in (-1, 1):
        supported = sum(grid.area(
            transform(Point(float(longitudinal), side*(near+far)/2, 0, 0), origin),
            (-0.18, 0.18, (far-near)/2))[0]
            for longitudinal in np.arange(-0.50, footprint.front+0.32, 0.40))
        if supported < 5:
            return None
    return free


def candidate_poses(grid, pose, footprint, excluded, margin=0.10, aisle_yaw=None, *, survey_trace=()):
    """Find perpendicular openings bounded by sensed occupancy on both sides."""
    aisle_yaw = pose.yaw if aisle_yaw is None else aisle_yaw
    c, s = math.cos(aisle_yaw), math.sin(aisle_yaw)
    scored = []
    for sign in (-1, 1):
        heading = wrap(aisle_yaw-sign*math.pi/2)
        for along in np.arange(-8.0, 7.1, 0.24):
            for depth in np.arange(2.8, 7.1, 0.40):
                x = pose.x+c*along-sign*s*depth
                y = pose.y+s*along+sign*c*depth
                point = Point(x, y, heading, 0)
                if any(distance(point, other) < 0.70 for other in excluded):
                    continue
                score = slot_score(grid, point, footprint, margin, survey_trace)
                if score is not None:
                    scored.append((score-0.011*abs(along)+0.008*depth, point))
    scored.sort(key=lambda item: item[0], reverse=True)
    selected = []
    for _, point in scored:
        if all(distance(point, other) > 0.55 for other in selected):
            selected.append(point)
    return selected


def analytical_parking(grid, pose, heading, footprint, excluded, min_radius, margin, *, survey_trace=(),
                       in_window=None, score_bay=slot_score):
    """Try direct bounded-curvature reverse arcs with an optional straight insertion."""
    if abs(wrap(pose.yaw-heading)) > (0.60 if in_window else 0.35):
        return
    offsets = (0, 0.35, 0.75, 1.05, 1.40, 1.75, 2.10, 2.45, 2.80) if in_window else (0, 0.35, 0.75)
    insertions = (0.72, 1.0, 1.26, 1.52, 0.38, 0.0) if in_window else (0.38, 0.72, 0.0, 1.08)
    for offset in offsets:
        radius = min_radius+offset
        for side in (1, -1):
            goal_yaw = wrap(heading-side*math.pi/2)
            delta = wrap(goal_yaw-pose.yaw)
            length = abs(delta)*radius
            curvature = -delta/length
            samples = math.ceil(length/0.13)
            arc = [motion(pose, -length*i/samples, curvature) for i in range(samples+1)]
            for insertion in insertions:
                goal = motion(arc[-1], -insertion, 0)
                if in_window and not in_window(goal):
                    continue
                if any(distance(goal, past) < 0.55 for past in excluded):
                    continue
                score = score_bay(grid, goal, footprint, margin, survey_trace)
                if score is None:
                    continue
                divisions = math.ceil(insertion/0.13)
                points = arc+[motion(arc[-1], -insertion*i/divisions, 0) for i in range(1, divisions+1)]
                yield goal, [(-1, points)], audit_path(grid, [(-1, points)], footprint, margin, strict=True)


def row_axis_vote(rays):
    """A fourth angular harmonic combines orthogonal straight surfaces."""
    if len(rays) < 7:
        return 0j
    vote = 0j
    for i in range(0, len(rays)-6, 6):
        group = rays[i:i+7]
        if not all(point[2] for point in group):
            continue
        a, b = group[0], group[-1]
        dx, dy = b[0]-a[0], b[1]-a[1]
        length = math.hypot(dx, dy)
        if not 0.30 <= length <= 2.4:
            continue
        if max(abs(dx*(p[1]-a[1])-dy*(p[0]-a[0])) for p in group[1:-1]) > 0.08*length:
            continue
        if max(math.hypot(x[0]-y[0], x[1]-y[1]) for x, y in zip(group, group[1:])) > 0.55:
            continue
        angle = 4*math.atan2(dy, dx)
        vote += length*complex(math.cos(angle), math.sin(angle))
    return vote


def aligned_aisle(yaw, vote):
    if abs(vote) < 1.0:
        return yaw
    axis = math.atan2(vote.imag, vote.real)/4
    return wrap(axis+round((yaw-axis)/(math.pi/2))*math.pi/2)


def survey_goals(grid, pose, footprint, visit_history, rejected):
    """Prefer fresh, observed forward space; try turns and reversals if boxed in."""
    scored = []
    for radius in (8.0, 6.0, 4.5, 3.2):
        for bearing in (0, 10, -10, 20, -20, 42, -42, 70, -70, 100, -100, 150, -150, 180):
            angle = wrap(pose.yaw+math.radians(bearing))
            for heading in (angle, pose.yaw):
                goal = Point(pose.x+radius*math.cos(angle), pose.y+radius*math.sin(angle), heading, 0)
                if grid.at(goal) > -2 or any(distance(goal, p) < 0.75 and abs(wrap(goal.yaw-p.yaw)) < 0.3
                                             for p in rejected):
                    continue
                occupied, free = grid.area(goal, footprint.bounds(0.10))
                if occupied or free < 0.88:
                    continue
                revisit = sum(max(0, 1-distance(goal, past)/3) for past in visit_history)
                score = 0.55*min(radius, 6.0)+free-0.065*abs(bearing)-2*revisit-0.28*abs(wrap(heading-angle))
                scored.append((score, goal))
    scored.sort(key=lambda item: item[0], reverse=True)
    return [goal for _, goal in scored]


def retreat_path(trace, current, radius=2.6):
    points = [current]
    for past in reversed(trace):
        if distance(points[-1], past) > 0.08:
            points.append(past)
        if distance(past, current) > radius:
            break
    if len(points) < 5 or distance(points[-1], current) < 0.85:
        return None
    try:
        legs = split_path(points)
    except ValueError:
        return None
    if legs and distance(legs[0][1][0], legs[0][1][-1]) >= 0.85:
        return legs[:1]
    return None


def observation_maneuver(grid, pose, footprint, min_radius, visited):
    """Last-resort, short bounded-curvature move into directly observed space."""
    candidates = []
    for direction in (-1, 1):
        for curvature in (0.0, -1/min_radius, 1/min_radius):
            for travel in (1.4, 2.3):
                samples = math.ceil(travel/0.12)
                points = [motion(pose, direction*travel*i/samples, curvature) for i in range(samples+1)]
                end = points[-1]
                obstacle, observed = grid.area(end, footprint.bounds(0.25))
                if obstacle or observed < 0.95:
                    continue
                legs = [(direction, points)]
                if audit_path(grid, legs, footprint, 0.02, strict=True):
                    continue
                revisit = sum(max(0, 1-distance(end, previous)/2.6) for previous in visited[-12:])
                score = observed+0.18*travel+(0.12 if direction == -1 else 0)-revisit
                candidates.append((score, legs))
    return max(candidates, key=lambda item: item[0])[1] if candidates else None


def motion(pose, length, curvature):
    """Constant curvature propagation with signed travel distance at the rear axle."""
    angle = pose.yaw+length*curvature
    if abs(curvature) < 1e-7:
        return Point(pose.x+length*math.cos(pose.yaw), pose.y+length*math.sin(pose.yaw),
                     angle, curvature)
    return Point(pose.x+(math.sin(angle)-math.sin(pose.yaw))/curvature,
                 pose.y+(math.cos(pose.yaw)-math.cos(angle))/curvature, wrap(angle), curvature)


def can_stop(points, footprint, speed, curves, brake, latency, margin, dynamic_margin=0.0, moving=None):
    """Check actual and requested braking arcs in the latest rear-axle frame."""
    if abs(speed) < 1e-4:
        return True
    speed_magnitude = abs(speed)
    reaction_distance = speed_magnitude*latency
    stop = reaction_distance+speed*speed/(2*brake)
    start = Point(0.0, 0.0, 0.0, 0.0)
    intervals = max(1, math.ceil(stop/0.09))
    # A stationary obstacle in the initial margin can be receding during
    # a retreat. Never ignore an obstacle intersecting the physical envelope.
    initial_bounds = footprint.bounds(margin+0.05)
    physically_clear = ~in_box(points, start, footprint.bounds(0.01))
    receding = speed*points[:, 0] < -0.2*speed_magnitude
    relax = physically_clear & receding & in_box(points, start, initial_bounds)
    strict_points, receding_points = points[~relax], points[relax]
    # Check the straightening arc as well as intermediate curvature while the
    # Gazebo velocity and steering limiters transition toward stopped.
    for curve in set(curves):
        for i in range(intervals+1):
            traveled = stop*i/intervals
            pose = motion(start, math.copysign(traveled, speed), curve)
            backing = speed < 0
            back, front, width = footprint.bounds(margin+0.05)
            if backing:
                back -= dynamic_margin
            else:
                front += dynamic_margin
            if (points_in_box(strict_points, pose, (back, front, width))
                    or points_in_box(receding_points, pose, footprint.bounds(0.025))):
                return False
            if moving is not None and moving.size:
                time = (traveled/speed_magnitude if traveled <= reaction_distance else
                        latency+(speed_magnitude-math.sqrt(
                            max(0.0, speed*speed-2*brake*(traveled-reaction_distance))))/brake)
                # Beyond the short tracking horizon, expand the uncertain
                # occupancy around the last reasonably predicted position.
                forecast = moving[:, 0]+moving[:, 1]*min(time, 1.8)
                spread = 0.32+0.15*time
                if points_in_box(forecast, pose, (back-spread, front+spread, width+spread)):
                    return False
    return True


def points_in_box(points, pose, bounds):
    return bool(np.any(in_box(points, pose, bounds)))


def in_box(points, pose, bounds):
    back, front, width = bounds
    x, y = points[:, 0]-pose.x, points[:, 1]-pose.y
    c, s = math.cos(pose.yaw), math.sin(pose.yaw)
    longitudinal, lateral = c*x+s*y, -s*x+c*y
    return (back <= longitudinal) & (longitudinal <= front) & (np.abs(lateral) <= width)


def split_path(points):
    """Recover motion direction from the signed displacement versus body heading."""
    legs = []
    for a, b in zip(points, points[1:]):
        length = distance(a, b)
        if length < 1e-5:
            continue
        heading = a.yaw+wrap(b.yaw-a.yaw)/2
        dot = (b.x-a.x)*math.cos(heading)+(b.y-a.y)*math.sin(heading)
        if abs(dot) < 0.45*length:
            raise ValueError("Planner path requires lateral motion")
        direction = 1 if dot > 0 else -1
        if not legs or legs[-1][0] != direction:
            legs.append((direction, [a]))
        legs[-1][1].append(b)
    return legs


def audit_path(grid, legs, footprint, margin, strict=False):
    """Check between path vertices, adding a bound on corner travel between samples."""
    max_radius = math.hypot(max(abs(footprint.back), footprint.front), footprint.half_width)
    for _, points in legs:
        for a, b in zip(points, points[1:]):
            dyaw = wrap(b.yaw-a.yaw)
            travel = distance(a, b)+max_radius*abs(dyaw)
            steps = max(1, math.ceil(travel/0.15))
            padded = footprint.bounds(margin+travel/(2*steps))
            for i in range(steps+1):
                t = i/steps
                sample = Point(a.x+t*(b.x-a.x), a.y+t*(b.y-a.y), a.yaw+t*dyaw, 0)
                obstacle, observed = grid.area(sample, padded)
                if obstacle:
                    return "occupied_sweep"
                if strict and observed < 0.88:
                    return "unobserved_sweep"
    return None


class PathFollower:
    """Track a prepared path in odom and stop at all direction changes."""

    def __init__(self, legs):
        self.legs = deque(legs)
        self.lengths = deque(route_lengths(points) for _, points in self.legs)
        self.index = 0

    def update(self, pose, speed, cruise, braking, tight, curve_acceleration=0.26):
        direction, points = self.legs[0]
        end = points[-1]
        nearest = min(range(max(0, self.index-1), min(len(points), self.index+18)),
                      key=lambda i: distance(pose, points[i]))
        if distance(pose, points[nearest]) > (0.52 if tight else 1.0):
            return "deviated", 0.0, 0.0, direction
        self.index = max(self.index, nearest)
        goal_error = distance(pose, end)
        yaw_error = abs(wrap(pose.yaw-end.yaw))
        final = len(self.legs) == 1
        threshold = (0.22 if final else 0.17) if tight else 0.28
        if goal_error <= threshold and self.index >= len(points)-4:
            if yaw_error > (0.16 if tight else 0.30):
                return "deviated", 0.0, 0.0, direction
            if abs(speed) > 0.035:
                return "stopping", 0.0, 0.0, direction
            self.legs.popleft()
            self.lengths.popleft()
            self.index = 0
            return ("settled" if not self.legs else "shift"), 0.0, 0.0, direction
        lookahead = (0.60 if tight else 0.95)+0.55*abs(speed)
        target = points[self.index]
        for target in points[self.index:]:
            if distance(pose, target) > lookahead:
                break
        x, y = target.x-pose.x, target.y-pose.y
        forward = math.cos(pose.yaw)*x+math.sin(pose.yaw)*y
        lateral = -math.sin(pose.yaw)*x+math.cos(pose.yaw)*y
        if direction*forward < -0.12:
            return "deviated", 0.0, 0.0, direction
        curvature = 2*lateral/max(x*x+y*y, 0.12**2)
        remaining = distance(pose, points[self.index])
        remaining += self.lengths[0][-1]-self.lengths[0][self.index]
        approach = math.sqrt(2*braking*max(0.015, remaining-0.7*threshold))
        curve_speed = math.sqrt(curve_acceleration/max(abs(curvature), 0.02))
        velocity = direction*min(cruise, approach, curve_speed)
        return "drive", velocity, velocity*curvature, direction
