import math
import numpy as np

def wrap(angle):
    return (angle + math.pi) % math.tau - math.pi

def compose(a, b):
    c, s = (math.cos(a[2]), math.sin(a[2]))
    return np.array([a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], wrap(a[2] + b[2])])

def relative(a, b):
    c, s = (math.cos(a[2]), math.sin(a[2]))
    dx, dy = (b[0] - a[0], b[1] - a[1])
    return np.array([c * dx + s * dy, -s * dx + c * dy, wrap(b[2] - a[2])])

def transform(points, pose):
    points = np.asarray(points, dtype=float)
    c, s = (math.cos(pose[2]), math.sin(pose[2]))
    return points @ np.array([[c, s], [-s, c]]) + np.asarray(pose[:2])

def rectangle(pose, length, width):
    return transform([[-length / 2, -width / 2], [length / 2, -width / 2], [length / 2, width / 2], [-length / 2, width / 2]], pose)

def intersects(a, b):
    a, b = (np.asarray(a), np.asarray(b))
    for polygon in (a, b):
        for edge in np.roll(polygon, -1, axis=0) - polygon:
            axis = np.array([-edge[1], edge[0]])
            p, q = (a @ axis, b @ axis)
            if p.max() < q.min() or q.max() < p.min():
                return False
    return True

def bicycle(pose, distance, curvature, wheelbase):
    x, y, h = pose
    rx = x - wheelbase / 2 * math.cos(h)
    ry = y - wheelbase / 2 * math.sin(h)
    nh = h + distance * curvature
    if abs(curvature) > 1e-09:
        rx += (math.sin(nh) - math.sin(h)) / curvature
        ry -= (math.cos(nh) - math.cos(h)) / curvature
    else:
        rx += distance * math.cos(h)
        ry += distance * math.sin(h)
    return np.array([rx + wheelbase / 2 * math.cos(nh), ry + wheelbase / 2 * math.sin(nh), nh])

def sample_line(a, b, step=0.08):
    count = max(1, math.ceil(math.dist(a[:2], b[:2]) / step))
    delta = wrap(b[2] - a[2])
    return [[a[0] + (b[0] - a[0]) * i / count, a[1] + (b[1] - a[1]) * i / count, a[2] + delta * i / count] for i in range(1, count + 1)]

def exploration_route(config):
    p = config['exploration']
    g = config['robot']
    wb = g['wheelbase']
    west, east, bottom, top, radius = (p['west'], p['east'], p['bottom'], p['top'], p['radius'])
    points = [list(config['start_pose'])]

    def line(x, y):
        points.extend(sample_line(points[-1], [x, y, points[-1][2]], p['sample_step']))

    def turn():
        start = points[-1]
        n = math.ceil(radius * math.pi / 2 / p['sample_step'])
        points.extend([bicycle(start, radius * math.pi / 2 * i / n, 1 / radius, wb).tolist() for i in range(1, n + 1)])
    line(p['anchor_x'], bottom)
    entry_end = len(points) - 1
    laps = []
    for lap in range(config['laps']):
        begin = len(points) - 1
        line(east - radius + wb / 2, bottom)
        turn()
        line(east, top - radius + wb / 2)
        turn()
        line(west + radius - wb / 2, top)
        turn()
        line(west, bottom + radius - wb / 2)
        turn()
        line(p['anchor_x'], bottom)
        laps.append([begin, len(points) - 1])
    return (np.asarray(points), entry_end, laps)
