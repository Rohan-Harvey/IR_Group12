"""Rear-axle feasible survey reference; runtime navigation uses sensed obstacles.

Coordinates are relative to the initial vehicle centre, with x facing into
the lot. The optional world pose is only for spawn/independent validation.
"""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class Point:
    x: float
    y: float
    yaw: float
    curvature: float
    direction: int = 1


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def transform(point, origin):
    c, s = math.cos(origin[2]), math.sin(origin[2])
    return Point(origin[0]+c*point.x-s*point.y,
                 origin[1]+s*point.x+c*point.y,
                 wrap(point.yaw+origin[2]), point.curvature, point.direction)


def rear_point(point, wheelbase):
    return Point(point.x-wheelbase/2*math.cos(point.yaw),
                 point.y-wheelbase/2*math.sin(point.yaw),
                 point.yaw, point.curvature, point.direction)


def route_lengths(points):
    lengths = [0.0]
    for a, b in zip(points, points[1:]):
        lengths.append(lengths[-1]+math.hypot(b.x-a.x, b.y-a.y))
    return lengths


def nearest_progress(points, pose, previous=0, window=65):
    # A bounded forward window prevents jumping to a nearby future loop leg.
    end = min(len(points), previous+window+1)
    return min(range(previous, end),
               key=lambda i: (points[i].x-pose[0])**2+(points[i].y-pose[1])**2)


def goal_index(lengths, progress, horizon):
    end = progress
    while end+1 < len(lengths) and lengths[end]-lengths[progress] < horizon:
        end += 1
    return end


def build_route(config, wheelbase):
    step = config["sample_step_m"]
    left, bottom, right, top = config["loop_rear_axle_bounds_m"]
    radius = config["corner_radius_m"]
    join = config["join_center_x_m"]-wheelbase/2
    if not (step > 0 and radius > 0 and right-left > 2*radius
            and top-bottom > 2*radius and left+radius < join < right-radius):
        raise ValueError("Invalid rounded survey loop")
    if config["planner_min_turning_radius_m"] > radius:
        raise ValueError("Survey corners are tighter than the planner allows")
    axle = wheelbase/2

    def centre(x, y, yaw, curvature=0.0, direction=1):
        return Point(x+axle*math.cos(yaw), y+axle*math.sin(yaw),
                     wrap(yaw), curvature, direction)

    # Smoothly merge from the entrance centreline into the bottom aisle.
    distance = join+axle
    entry = []
    for i in range(math.ceil(distance/step)+1):
        t = i/math.ceil(distance/step)
        h = 10*t**3-15*t**4+6*t**5
        dy = bottom*(30*t**2-60*t**3+30*t**4)/distance
        ddy = bottom*(60*t-180*t**2+120*t**3)/distance**2
        entry.append(centre(-axle+distance*t, bottom*h, math.atan(dy),
                            ddy/(1+dy*dy)**1.5))
    loop = [entry[-1]]

    def line(x, y, heading):
        a = rear_point(loop[-1], wheelbase)
        length = math.hypot(x-a.x, y-a.y)
        n = max(1, math.ceil(length/step))
        for i in range(1, n+1):
            t = i/n
            loop.append(centre(a.x+t*(x-a.x), a.y+t*(y-a.y), heading))

    def left_arc(cx, cy, start):
        n = math.ceil(radius*math.pi/2/step)
        for i in range(1, n+1):
            angle = start+math.pi/2*i/n
            loop.append(centre(cx+radius*math.cos(angle), cy+radius*math.sin(angle),
                               angle+math.pi/2, 1/radius))

    line(right-radius, bottom, 0)
    left_arc(right-radius, bottom+radius, -math.pi/2)
    line(right, top-radius, math.pi/2)
    left_arc(right-radius, top-radius, 0)
    line(left+radius, top, math.pi)
    left_arc(left+radius, top-radius, math.pi/2)
    line(left, bottom+radius, -math.pi/2)
    left_arc(left+radius, bottom+radius, math.pi)
    line(join, bottom, 0)
    reverse = [Point(p.x, p.y, p.yaw, p.curvature, -1) for p in reversed(entry)]
    return [("entry", entry), ("loop", loop), ("return", reverse)]


def limit_command(v, w, maximum_speed, reverse_speed, maximum_curvature):
    if not all(math.isfinite(n) for n in (v, w)):
        return 0.0, 0.0
    clipped = min(maximum_speed, max(-reverse_speed, v))
    if abs(v) < 1e-4:
        return 0.0, 0.0
    curvature = max(-maximum_curvature, min(maximum_curvature, w/v))
    return clipped, clipped*curvature


def command_is_fresh(enabled, now, command_time, scan_time, odom_time, enable_time):
    return bool(enabled and math.isfinite(now)
                and 0 <= now-command_time < 0.5
                and 0 <= now-scan_time < 0.6
                and 0 <= now-odom_time < 0.6
                and 0 <= now-enable_time < 2.0)


def command_matches_direction(v, direction):
    """A mission phase authorizes one sign only; zero always means hold."""
    return bool(math.isfinite(v) and direction in (-1, 1) and v*direction > 0)
