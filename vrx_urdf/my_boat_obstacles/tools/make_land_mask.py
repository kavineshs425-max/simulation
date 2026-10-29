#!/usr/bin/env python3
"""Build a land/water mask for a Gazebo world from its terrain mesh (Collada).

    python3 tools/make_land_mask.py                       # sydney_regatta, from the Fuel cache
    python3 tools/make_land_mask.py --dae X.dae --name my_world --z-offset 0.0

The spawner uses worlds/<world>_land.npz to warn when an obstacle starts on
land or its path runs ashore. Rule used (true for sydney_regatta, check it for
another world): a cell is LAND if any terrain triangle covers it above the
waterline z = 0; no triangle = water. But "no triangle" is ALSO what lies
beyond the edge of the terrain, where there is no world at all -- so only water
ENCLOSED by land counts: connected water regions that touch the edge of the
map are dropped, as are specks smaller than --min-lake (gaps between
triangles). Each remaining lake gets an id (1 = largest), stored as `lakes`.
The terrain's own <include><pose> z offset (0.2 m for sydney_regatta) is added.

Collada is parsed directly (vertex positions, <triangles>/<polylist>, node
<matrix> transforms, <unit meter=...>) -- no pycollada needed.
"""
import argparse
import glob
import os

import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from my_boat_obstacles.dae import dae_triangles  # noqa: E402


def rasterise(tri, bounds, res):
    x0, x1, y0, y1 = bounds
    nx, ny = int(round((x1 - x0) / res)), int(round((y1 - y0) / res))
    land = np.zeros((ny, nx), bool)
    mn, mx = tri[:, :, :2].min(1), tri[:, :, :2].max(1)
    keep = (mx[:, 0] > x0) & (mn[:, 0] < x1) & (mx[:, 1] > y0) & (mn[:, 1] < y1) \
        & (tri[:, :, 2].max(1) > 0.0)
    for t in tri[keep]:
        (ax, ay, az), (bx, by, bz), (cx, cy, cz) = t
        i0 = max(0, int((min(ay, by, cy) - y0) / res)); i1 = min(ny - 1, int((max(ay, by, cy) - y0) / res) + 1)
        j0 = max(0, int((min(ax, bx, cx) - x0) / res)); j1 = min(nx - 1, int((max(ax, bx, cx) - x0) / res) + 1)
        if i1 < i0 or j1 < j0:
            continue
        gy, gx = np.mgrid[i0:i1 + 1, j0:j1 + 1]
        px, py = x0 + (gx + 0.5) * res, y0 + (gy + 0.5) * res
        det = (by - cy) * (ax - cx) + (cx - bx) * (ay - cy)
        if abs(det) < 1e-12:
            continue
        l1 = ((by - cy) * (px - cx) + (cx - bx) * (py - cy)) / det
        l2 = ((cy - ay) * (px - cx) + (ax - cx) * (py - cy)) / det
        l3 = 1.0 - l1 - l2
        inside = (l1 >= -1e-9) & (l2 >= -1e-9) & (l3 >= -1e-9)
        z = l1 * az + l2 * bz + l3 * cz
        land[i0:i1 + 1, j0:j1 + 1] |= inside & (z > 0.0)
    return land


def main():
    fuel = sorted(glob.glob(os.path.expanduser(
        '~/.gz/fuel/fuel.gazebosim.org/openrobotics/models/sydney_regatta/*/meshes/sydney_regatta.dae')))
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--dae', default=fuel[-1] if fuel else None)
    ap.add_argument('--name', default='sydney_regatta', help='world name -> worlds/<name>_land.npz')
    ap.add_argument('--z-offset', type=float, default=0.2, help='terrain <include><pose> z')
    ap.add_argument('--bounds', type=float, nargs=4, default=None, metavar=('X0', 'X1', 'Y0', 'Y1'),
                    help='default: the whole terrain + 20 m')
    ap.add_argument('--res', type=float, default=1.0)
    ap.add_argument('--min-lake', type=float, default=500.0, help='[m^2] smaller water = land')
    a = ap.parse_args()
    if not a.dae:
        ap.error('no --dae given and sydney_regatta is not in the Fuel cache')
    from scipy import ndimage
    tri = dae_triangles(a.dae)
    tri[:, :, 2] += a.z_offset
    if a.bounds is None:
        v = tri.reshape(-1, 3)
        a.bounds = [float(np.floor(v[:, 0].min() - 20)), float(np.ceil(v[:, 0].max() + 20)),
                    float(np.floor(v[:, 1].min() - 20)), float(np.ceil(v[:, 1].max() + 20))]
    land = rasterise(tri, a.bounds, a.res)
    lab, n = ndimage.label(~land)                    # 4-connected water regions
    edge = set(np.unique(np.r_[lab[0], lab[-1], lab[:, 0], lab[:, -1]])) - {0}
    areas = ndimage.sum(np.ones_like(lab), lab, index=np.arange(1, n + 1)) * a.res ** 2
    keep = [k for k in range(1, n + 1) if k not in edge and areas[k - 1] >= a.min_lake]
    keep.sort(key=lambda k: -areas[k - 1])
    lakes = np.zeros(land.shape, np.uint8)
    for i, k in enumerate(keep, 1):
        lakes[lab == k] = i
    land = lakes == 0                                # outside-the-world and specks = not water
    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'worlds',
                       f'{a.name}_land.npz')
    np.savez_compressed(out, land=np.packbits(land, axis=1), shape=np.array(land.shape),
                        lakes=lakes, origin=np.array(a.bounds[::2]), res=np.array(a.res),
                        source=np.array(os.path.basename(a.dae)))
    print(f'{out}: {land.shape[1]} x {land.shape[0]} cells at {a.res} m, bounds {a.bounds}, '
          f'{os.path.getsize(out) / 1e3:.0f} kB')
    for i, k in enumerate(keep, 1):
        ys, xs = np.nonzero(lakes == i)
        print(f'  lake {i}: {areas[k - 1] / 1e6:.3f} km^2, x {a.bounds[0] + xs.min() * a.res:.0f}..'
              f'{a.bounds[0] + (xs.max() + 1) * a.res:.0f}, y {a.bounds[2] + ys.min() * a.res:.0f}..'
              f'{a.bounds[2] + (ys.max() + 1) * a.res:.0f}')


if __name__ == '__main__':
    main()
