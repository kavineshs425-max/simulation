"""Land/water mask of a world (built by tools/make_land_mask.py) -- ROS-free.

Used only to WARN: an obstacle whose start is on land, or whose path runs
ashore, is almost always a start pose typed wrong. Nothing is blocked -- the
obstacle is still spawned where you said.
"""

import math
import os
from typing import List, Optional, Tuple

import numpy as np


class LandMask:
    def __init__(self, path: str):
        d = np.load(path)
        ny, nx = (int(v) for v in d['shape'])
        self.land = np.unpackbits(d['land'], axis=1)[:, :nx].astype(bool)
        self.x0, self.y0 = (float(v) for v in d['origin'])
        self.res = float(d['res'])
        self.path = path

    def land_at(self, X, Y):
        """Vectorised: bool array, True = land. Outside the mapped area counts as land."""
        j = np.floor((np.asarray(X) - self.x0) / self.res).astype(int)
        i = np.floor((np.asarray(Y) - self.y0) / self.res).astype(int)
        ok = (i >= 0) & (i < self.land.shape[0]) & (j >= 0) & (j < self.land.shape[1])
        out = np.ones(np.shape(X), bool)
        out[ok] = self.land[i[ok], j[ok]]
        return out

    def is_land(self, x: float, y: float) -> Optional[bool]:
        """True/False, or None outside the mapped area."""
        j = int(math.floor((x - self.x0) / self.res))
        i = int(math.floor((y - self.y0) / self.res))
        if 0 <= i < self.land.shape[0] and 0 <= j < self.land.shape[1]:
            return bool(self.land[i, j])
        return None


def hull_points(s) -> List[Tuple[float, float]]:
    """Centre + 4 corners of an obstacle's hull box (so a hull that scrapes
    the shore with its bow is caught, not only one whose centre is ashore)."""
    c, sn = math.cos(s.heading), math.sin(s.heading)
    pts = [(s.x, s.y)]
    for dx, dy in ((s.l / 2, s.w / 2), (s.l / 2, -s.w / 2), (-s.l / 2, s.w / 2), (-s.l / 2, -s.w / 2)):
        pts.append((s.x + c * dx - sn * dy, s.y + sn * dx + c * dy))
    return pts


def land_problems(obstacles, mask: LandMask, duration: float, step: float = 0.5):
    """[(name, 'start' | 'path', t, x, y), ...]: the first time each obstacle's
    hull touches land. 'start' = already on land before run."""
    out = []
    for ob in obstacles:
        t = 0.0
        while t <= duration + 1e-9:
            s = ob.state(t)
            hit = next(((x, y) for x, y in hull_points(s) if mask.is_land(x, y)), None)
            if hit:
                out.append((ob.spec.name, 'start' if t == 0.0 else 'path', t, hit[0], hit[1]))
                break
            if s.speed == 0.0 and t > ob.spec.start_delay:
                break                       # holding still: it cannot get any wetter
            t += step
    return out


def default_mask_path(share_dir: str, world: str) -> Optional[str]:
    p = os.path.join(share_dir, 'worlds', f'{world}_land.npz')
    return p if os.path.isfile(p) else None
