#!/usr/bin/env python3
import argparse
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET
from ccw_navigation import MissionFollower, StreamWatchdog, WheelImuOdometry, blocked, wrap
from scene_geometry import rectangle, overlaps, inside_driving_surface
import scene_scenarios as suite
ROOT = Path(__file__).resolve().parents[1]

def check(value, message):
    if not value:
        raise ValueError(message)

def validate(root=ROOT, rollout=True):
    cfg = json.loads((root / 'config/parking_lot.json').read_text())
    mission = json.loads((root / 'config/ccw_route.json').read_text())
    g = mission['geometry']
    scale = g['scale']
    obstacles = [(name, [(x * scale, y * scale) for x, y in polygon]) for name, polygon in suite.obstacle_polygons(suite.make_layout(cfg))]
    boxes = [(n, p, (min((x for x, y in p)), min((y for x, y in p)), max((x for x, y in p)), max((y for x, y in p)))) for n, p in obstacles]

    def clear(pose, margin):
        shape = rectangle(pose[:2], g['length'] + 2 * margin * scale, g['width'] + 2 * margin * scale, pose[2])
        bounds = (min((x for x, y in shape)), min((y for x, y in shape)), max((x for x, y in shape)), max((y for x, y in shape)))
        check(all((inside_driving_surface(x, y, scale) for x, y in shape)), 'Footprint leaves the entrance road or parking area')
        for name, polygon, b in boxes:
            if bounds[2] < b[0] or b[2] < bounds[0] or bounds[3] < b[1] or (b[3] < bounds[1]):
                continue
            check(not overlaps(shape, polygon), 'Footprint intersects ' + name)
    check(mission['lap_count'] == 2 and len(mission['lap_ranges']) == 2, 'Need exactly two full laps')
    check(math.dist(mission['start_pose_m_rad'][:2], [-30 * scale, -9 * scale]) < 1e-06, 'Robot does not start at the entrance')
    for phase in ['forward', 'reverse']:
        for pose in mission[phase]:
            clear(pose, 0.2)
    actor = ET.parse(root / 'worlds/parking_lot.sdf').find('world/actor')
    if actor is not None:
        base = [float(v) for v in actor.findtext('pose').split()]
        waypoints = [[float(v) for v in p.findtext('pose').split()] for p in actor.findall('script/trajectory/waypoint')]
        xs = [base[0] + p[0] for p in waypoints]
        ys = [base[1] + p[1] for p in waypoints]
        sweep = rectangle([(min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2], max(xs) - min(xs) + 0.8 * scale, max(ys) - min(ys) + 0.8 * scale, 0)
        for pose in mission['forward'] + mission['reverse']:
            body = rectangle(pose[:2], g['length'] + 0.4 * scale, g['width'] + 0.4 * scale, pose[2])
            check(not overlaps(body, sweep), 'Pedestrian route conflicts with the circulating LIMO')
    forward = mission['forward']
    for lap, (begin, end) in enumerate(mission['lap_ranges'], 1):
        check(math.dist(forward[begin][:2], forward[end][:2]) < 1e-06, 'Lap does not close at its start')
        check(abs(forward[end][2] - forward[begin][2] - math.tau) < 1e-06, 'Lap is not one complete CCW turn')
        points = forward[begin:end + 1]
        area = sum((a[0] * b[1] - b[0] * a[1] for a, b in zip(points, points[1:] + points[:1]))) / 2
        check(area > 0, 'Lap direction is not counter-clockwise')
        checkpoints = [p for p in mission['checkpoints'] if p['lap'] == lap]
        check([p['id'] for p in checkpoints] == ['S2', 'S1', 'S3', 'S4', 'S5', 'S6'], 'Open case S6 is not last')
        check(all((begin <= p['path_index'] <= end for p in checkpoints)), 'Checkpoint lies outside its lap')
        check(all((a['path_index'] < b['path_index'] for a, b in zip(checkpoints, checkpoints[1:]))), 'Case order is not strictly increasing')
    check(math.dist(mission['reverse'][-1][:2], mission['goal_pose_m_rad'][:2]) < 1e-06, 'Reverse arc misses S6')
    check(abs(wrap(mission['reverse'][-1][2] - math.pi / 2)) < 1e-06, 'S6 is not perpendicular to the south curb')
    for sign in [1, -1]:
        check(blocked([(sign * 2.2 * scale, 0)], sign * 0.4 * scale, 0, g['wheelbase'], g['length'], g['width'], scale), 'Near obstacle ignored')
        check(not blocked([(sign * 10 * scale, 0)], sign * 0.4 * scale, 0, g['wheelbase'], g['length'], g['width'], scale), 'Distant obstacle falsely blocks')
    follower = MissionFollower(mission)
    check(follower.step(mission['start_pose_m_rad'], [(0, 0)], 0) == (0, 0), 'Pedestrian/obstacle does not stop the robot')
    check(follower.step(mission['start_pose_m_rad'], [], 1)[0] > 0, 'Robot does not resume after clearance')
    watchdog = StreamWatchdog(['/scan', 'imu', 'joint_states', 'odom'], timeout=2)
    check(not watchdog.ready(0, 0), 'Missing streams accepted')
    for name in watchdog.names:
        watchdog.feed(name, 1, 1)
    check(watchdog.ready(1.1, 1.1), 'Fresh streams rejected')
    check(not watchdog.ready(3, 1.2), 'Stale simulation data accepted')
    try:
        watchdog.ready(0, 1.3)
    except ValueError:
        pass
    else:
        raise ValueError('Clock reset accepted')
    estimator = WheelImuOdometry([0, 0, 0], g['wheelbase'], g['wheel_radius'])
    estimator.imu(0, 0, 0)
    estimator.wheels(0, 0, 0)
    left = right = 0
    for i in range(1, 1001):
        angle = math.tau * i / 1000
        delta = math.tau / 1000
        left += (g['radius'] - 0.7 * scale) * delta / g['wheel_radius']
        right += (g['radius'] + 0.7 * scale) * delta / g['wheel_radius']
        estimator.imu(wrap(angle), delta / 0.05, i * 0.05)
        estimated = estimator.wheels(left, right, i * 0.05)
    check(math.dist(estimated[:2], [0, 0]) < 0.001 * scale and abs(wrap(estimated[2])) < 1e-06, 'Wheel/IMU integration failed')
    result = {'kind': 'offline geometry and bicycle dynamics, not Gazebo', 'checks_passed': True, 'laps': 2, 'last_case_each_lap': 'S6', 'pedestrian_sweep_clear_of_laps': actor is not None}
    if rollout:
        body = list(mission['start_pose_m_rad'])
        follower = MissionFollower(mission)
        speed = steer = 0
        dt = 0.05
        for tick in range(24000):
            v, w = follower.step(body, [], tick * dt, speed)
            if follower.completed:
                break
            speed += max(-0.5 * scale * dt, min(0.5 * scale * dt, v - speed))
            desired = math.atan(g['wheelbase'] * w / v) if abs(v) > 1e-09 else steer
            steer += max(-1.5 * dt, min(1.5 * dt, 5 * (desired - steer) * dt))
            k = math.tan(steer) / g['wheelbase']
            yaw = body[2]
            rx = body[0] - g['wheelbase'] / 2 * math.cos(yaw)
            ry = body[1] - g['wheelbase'] / 2 * math.sin(yaw)
            turn = speed * k * dt
            if abs(k) > 1e-09:
                rx += (math.sin(yaw + turn) - math.sin(yaw)) / k
                ry -= (math.cos(yaw + turn) - math.cos(yaw)) / k
            else:
                rx += speed * dt * math.cos(yaw)
                ry += speed * dt * math.sin(yaw)
            yaw += turn
            body = [rx + g['wheelbase'] / 2 * math.cos(yaw), ry + g['wheelbase'] / 2 * math.sin(yaw), yaw]
            clear(body, 0.15)
        else:
            raise ValueError('Two-lap rollout did not complete')
        check(follower.completed_laps == 2, 'Controller did not count both completed laps')
        expected = [f'lap_{lap}_passed_{case}' for lap in [1, 2] for case in ['S2', 'S1', 'S3', 'S4', 'S5', 'S6']]
        actual = [event for event, t in follower.events if '_passed_' in event]
        check(actual == expected, 'Controller skipped, repeated or reordered a case')
        events = [e for e, t in follower.events]
        check(events.index('lap_2_complete') < events.index('reverse_started'), 'Reverse started before the second completed lap')
        error = math.dist(body[:2], mission['goal_pose_m_rad'][:2])
        angle = abs(math.degrees(wrap(body[2] - mission['goal_pose_m_rad'][2])))
        check(error < 0.2 * scale and angle < 6, 'Parking endpoint is outside tolerance')
        result.update(duration_s=round(tick * dt, 2), position_error_m=round(error, 5), heading_error_deg=round(angle, 4), completed_laps=follower.completed_laps, checkpoint_events=actual)
    return result
if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('root', nargs='?', type=Path, default=ROOT)
    args = parser.parse_args()
    print(json.dumps(validate(args.root), indent=2))
