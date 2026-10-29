#!/usr/bin/env python3
"""Chart the world's lakes into a vector map, as DEPARE (depth area) features.

    tools/chart_lakes.py maps/regatta_canals.geojson [--mask worlds/sydney_regatta_land.npz]

A planner must never read the simulator's files, so the shoreline has to be IN
the map: each lake of the land mask becomes a DEPARE polygon (outline traced
with OpenCV, simplified to 0.75 m, specks < 20 m^2 ignored) with the base
depth below. Anything outside charted water is not navigable (the compiler's
rule), exactly like an area a paper chart leaves blank. Re-running replaces
the lake_* features and leaves every other feature untouched.
"""
import argparse
import json
import os
import sys

import cv2
import numpy as np

from marine_vector_map import vmap

LAKES = {1: ('lake_north', 'north lake', 2.5, 5.0), 2: ('lake_rowing_course', 'rowing course', 3.0, 5.0),
         3: ('lake_east_pond', 'east pond', 1.5, 3.0), 4: ('lake_small_pond', 'small pond', 1.0, 2.0),
         5: ('lake_5', 'small lake', 1.0, 1.5)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('map')
    ap.add_argument('--mask', default=os.path.join(os.path.dirname(__file__), '..', 'worlds', 'sydney_regatta_land.npz'))
    a = ap.parse_args()
    fc = vmap.load(a.map)
    d = np.load(a.mask)
    L, (x0, y0), res = d['lakes'], (float(v) for v in d['origin']), float(d['res'])
    feats = []
    for k in range(1, int(L.max()) + 1):
        cs, _ = cv2.findContours((L == k).astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        c = max(cs, key=cv2.contourArea)
        if cv2.contourArea(c) < 20:
            continue
        ring = [[round(x0 + (float(p[0][0]) + 0.5) * res, 2), round(y0 + (float(p[0][1]) + 0.5) * res, 2)]
                for p in cv2.approxPolyDP(c, 0.75, True)]
        ring.append(ring[0])
        name, label, d1, d2 = LAKES.get(k, (f'lake_{k}', f'lake {k}', 1.0, 2.0))
        feats.append({'type': 'Feature', 'geometry': {'type': 'Polygon', 'coordinates': [ring]},
                      'properties': {'class': 'DEPARE', 'name': name, 'drval1': d1, 'drval2': d2,
                                     'description': f'{label}: charted water (base depth)',
                                     'source': 'shoreline traced from the world land mask'}})
    loc = {'type': 'FeatureCollection', 'properties': fc['properties'], 'features': feats}
    new = vmap.to_geojson(loc)['features']
    keep = [f for f in fc['features'] if not f['properties']['name'].startswith('lake_')]
    fc['features'] = keep + new
    vmap.validate(fc)
    with open(a.map, 'w') as fh:
        fh.write(vmap.dumps(fc))
    print(f'{len(new)} lakes charted into {a.map} ({sum(len(f["geometry"]["coordinates"][0]) for f in new)} vertices)')


if __name__ == '__main__':
    sys.exit(main())
