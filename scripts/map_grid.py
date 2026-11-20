"""Pure grid operations; navigation padding never changes measured cells."""

from array import array
from dataclasses import dataclass
import math
import struct


@dataclass(frozen=True)
class PaddedGrid:
    width: int
    height: int
    border: int
    data: array


def validate_grid(width, height, resolution, data):
    if (width <= 0 or height <= 0 or len(data) != width*height
            or not math.isfinite(resolution) or resolution <= 0):
        raise ValueError("Invalid occupancy-grid dimensions, resolution or data length")


def pad_grid(width, height, resolution, data, padding_m):
    """Add only unknown (-1) cells, retaining the original grid verbatim."""
    validate_grid(width, height, resolution, data)
    if not math.isfinite(padding_m) or padding_m < 0:
        raise ValueError("Map padding must be finite and nonnegative")
    border = math.ceil(padding_m/resolution)
    new_width, new_height = width+2*border, height+2*border
    result = array("b", [-1])*(new_width*new_height)
    source = array("b", data)
    for row in range(height):
        offset = (row+border)*new_width+border
        result[offset:offset+width] = source[row*width:(row+1)*width]
    return PaddedGrid(new_width, new_height, border, result)


def padded_origin(x, y, angle, border, resolution):
    distance = border*resolution
    c, s = math.cos(angle), math.sin(angle)
    return x-distance*c+distance*s, y-distance*s-distance*c


def preview_points(width, height, resolution, data, origin, max_points=60000):
    """Return an RGB PointCloud2 buffer for display, independent of navigation.

    Sampling limits display cost only. The full grid is used by pad_grid.
    RGB is a packed UINT32; x/y/z are FLOAT32, little endian.
    """
    validate_grid(width, height, resolution, data)
    if max_points < 1:
        raise ValueError("Preview point limit must be positive")
    step = max(1, math.ceil(math.sqrt(width*height/max_points)))
    while math.ceil(width/step)*math.ceil(height/step) > max_points:
        step += 1
    count = math.ceil(width/step)*math.ceil(height/step)
    output = bytearray(count*16)
    c, s = math.cos(origin[2]), math.sin(origin[2])
    offset = 0
    for row in range(0, height, step):
        for column in range(0, width, step):
            value = data[row*width+column]
            color = 0x808080 if value < 0 else (0x101010 if value >= 50 else 0xFFFFFF)
            x, y = (column+0.5)*resolution, (row+0.5)*resolution
            struct.pack_into("<fffI", output, offset,
                             origin[0]+c*x-s*y, origin[1]+s*x+c*y, -0.02, color)
            offset += 16
    return bytes(output), count
