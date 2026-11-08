"""Geometry shared by the virtual boundary, frontier map and motion guard."""

import math


def inside_rectangle(x, y, center_x, center_y, yaw, half_x, half_y):
    """Whether a point is inside the start-pose-aligned safe rectangle."""
    values = (x, y, center_x, center_y, yaw, half_x, half_y)
    if not all(math.isfinite(value) for value in values) or half_x <= 0 or half_y <= 0:
        return False
    dx, dy = x - center_x, y - center_y
    c, s = math.cos(yaw), math.sin(yaw)
    return abs(c * dx + s * dy) <= half_x and abs(-s * dx + c * dy) <= half_y


def rectangle_corners(center_x, center_y, yaw, half_x, half_y):
    c, s = math.cos(yaw), math.sin(yaw)
    return [(center_x + c * x - s * y, center_y + s * x + c * y)
            for x, y in ((-half_x, -half_y), (half_x, -half_y),
                         (half_x, half_y), (-half_x, half_y))]


def occupancy_at(data, width, height, resolution, origin_x, origin_y,
                 origin_yaw, x, y):
    """Read an occupancy or costmap cell at a point in the map frame."""
    if (width <= 0 or height <= 0 or len(data) != width * height or
            not all(math.isfinite(v) for v in (
                resolution, origin_x, origin_y, origin_yaw, x, y)) or resolution <= 0):
        return None
    dx, dy = x - origin_x, y - origin_y
    c, s = math.cos(origin_yaw), math.sin(origin_yaw)
    col = math.floor((c * dx + s * dy) / resolution)
    row = math.floor((-s * dx + c * dy) / resolution)
    if col < 0 or row < 0 or col >= width or row >= height:
        return None
    return data[row * width + col]


def observation_grid(center_x, center_y, yaw, half_x, half_y,
                     spacing, inset):
    """Evenly spaced possible observation poses inside the safe rectangle."""
    if (not all(math.isfinite(v) for v in (
            center_x, center_y, yaw, half_x, half_y, spacing, inset)) or
            spacing <= 0 or inset < 0 or min(half_x, half_y) <= inset):
        raise ValueError('Invalid observation grid geometry')
    limits = (half_x - inset, half_y - inset)
    axes = []
    for limit in limits:
        intervals = math.floor(2 * limit / spacing)
        axes.append([(index - intervals / 2) * spacing
                     for index in range(intervals + 1)])
    c, s = math.cos(yaw), math.sin(yaw)
    return [(center_x + c * x - s * y, center_y + s * x + c * y)
            for x in axes[0] for y in axes[1]]


def bounded_map_data(data, width, height, resolution, origin_x, origin_y,
                     origin_yaw, center_x, center_y, yaw, half_x, half_y):
    """Hide unknown space outside the allowed area from the frontier finder.

    Outside cells are occupied, so they cannot create artificial frontiers at
    the boundary. The original SLAM map remains untouched for Nav2 and saving.
    """
    if width <= 0 or height <= 0 or len(data) != width * height:
        raise ValueError('Invalid occupancy grid')
    if not math.isfinite(resolution) or resolution <= 0:
        raise ValueError('Invalid map resolution')
    result = list(data)
    c, s = math.cos(origin_yaw), math.sin(origin_yaw)
    for row in range(height):
        gy = (row + 0.5) * resolution
        for col in range(width):
            gx = (col + 0.5) * resolution
            wx, wy = origin_x + c * gx - s * gy, origin_y + s * gx + c * gy
            if not inside_rectangle(wx, wy, center_x, center_y, yaw, half_x, half_y):
                result[row * width + col] = 100
    return result


def keepout_mask(center_x, center_y, yaw, half_x, half_y, resolution=0.10,
                 padding=3.0):
    """Create a closed lethal ring surrounding the mission rectangle.

    The ring remains inside the published grid even if the SLAM map expands.
    A path cannot leave the rectangle without crossing lethal mask cells.
    """
    if not all(math.isfinite(v) for v in (center_x, center_y, yaw, half_x, half_y,
                                          resolution, padding)):
        raise ValueError('Nonfinite keepout geometry')
    if min(half_x, half_y, resolution, padding) <= 0:
        raise ValueError('Invalid keepout geometry')
    extent = math.hypot(half_x, half_y) + padding
    cells = math.ceil(2 * extent / resolution)
    origin_x, origin_y = center_x - extent, center_y - extent
    data = [100] * (cells * cells)
    for row in range(cells):
        y = origin_y + (row + 0.5) * resolution
        for col in range(cells):
            x = origin_x + (col + 0.5) * resolution
            if inside_rectangle(x, y, center_x, center_y, yaw, half_x, half_y):
                data[row * cells + col] = 0
    return origin_x, origin_y, cells, cells, data
