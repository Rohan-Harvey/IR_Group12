"""Shared two-dimensional footprint geometry; standard library only."""

import math


def rectangle(center, length, width, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return [(center[0] + c * x - s * y, center[1] + s * x + c * y)
            for x, y in [(-length / 2, -width / 2), (length / 2, -width / 2),
                         (length / 2, width / 2), (-length / 2, width / 2)]]


def overlaps(first, second):
    """Separating-axis test; touching oriented rectangles count as overlapping."""
    for polygon in (first, second):
        for a, b in zip(polygon, polygon[1:] + polygon[:1]):
            axis = (b[1] - a[1], a[0] - b[0])
            p = [axis[0] * x + axis[1] * y for x, y in first]
            q = [axis[0] * x + axis[1] * y for x, y in second]
            if max(p) < min(q) or max(q) < min(p):
                return False
    return True
