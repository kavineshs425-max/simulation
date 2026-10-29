"""2-D geometry shared by the scenario check and the spawner -- ROS-free."""

import math


def boxes_overlap(a, b) -> bool:
    """2-D oriented rectangles (cx, cy, yaw, length, width) intersect?
    Separating-axis test on the 4 edge normals."""
    def corners(r):
        cx, cy, yaw, l, w = r
        c, s = math.cos(yaw), math.sin(yaw)
        return [(cx + c * dx - s * dy, cy + s * dx + c * dy)
                for dx, dy in ((l / 2, w / 2), (l / 2, -w / 2), (-l / 2, -w / 2), (-l / 2, w / 2))]
    ca, cb = corners(a), corners(b)
    for yaw in (a[2], a[2] + math.pi / 2, b[2], b[2] + math.pi / 2):
        ax, ay = math.cos(yaw), math.sin(yaw)
        pa = [x * ax + y * ay for x, y in ca]
        pb = [x * ax + y * ay for x, y in cb]
        if max(pa) < min(pb) or max(pb) < min(pa):
            return False
    return True
