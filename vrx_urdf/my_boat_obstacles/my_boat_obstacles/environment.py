"""The ENVIRONMENT: canals, basins, land, docks, bridges and other fixed structures. ROS-free.

Obstacles (scenario.py) are things that move or could move. The environment is
what does NOT: the geometry a route planner would read off a chart. It has two
parts.

TERRAIN -- land and water, one rule:

        solid = (land regions  U  canal banks)  -  (canal water  U  basins)

  * a LAND region is a polygon of solid ground (reclaimed land, an island).
  * a CANAL is a centreline + a water width. Its water cuts through any land;
    where no land region covers it, BANKS of bank_width are raised on both
    sides, so a canal in open water is still a canal (the boat cannot just go
    round the outside). Ends are OPEN by default (the water runs on past the
    banks); a CLOSED end is a dead end (a backwater).
  * a BASIN is a rectangle of water cut out of the land (a marina).
  Because it is one rule, junctions need no special case: where two canals
  meet, each one's water removes the other's bank.

  The terrain is rasterised at `resolution` (only where the world itself is
  water -- land on land is pointless), merged into rectangles, and spawned as
  ONE static Gazebo model. The same raster is the environment's occupancy map.

STRUCTURES -- exact, rotatable shapes, one static model each:
  pontoon   floating walkway with finger piers; the gaps between fingers are
            BERTHS, each with the pose a stern-first docking ends at
  jetty     fixed pier on piles, reaching out from its anchor
  bridge    deck at a clearance height, abutments at both ends, piers between
  wall      quay wall / breakwater
  pile_row  a line of piles (mooring dolphins, a channel edge)
  net       fishing net: floats you can see, a barrier under the water you
            cannot (collision only -- the lidar does not see it)

Every structure is x, y (world, ENU), heading_deg (0 = east, + counter-clockwise)
and its type's parameters (TYPES, which the web page's form is built from).
A part BLOCKS NAVIGATION when it spans the band between the boat's draft and its
air draft -- a bridge deck at 3 m clearance does not, its piers do.
"""

import hashlib
import json
import math

# the geometry of the physical features lives in the vector-map library, so the
# simulator and route planner 1 cannot disagree about where the water is
from marine_vector_map.physical import *            # noqa: F401,F403
from marine_vector_map.physical import (Terrain, _poly_mask, structure_parts)   # noqa: F401


# ------------------------------------------------------------------ SDF
def _mat(c):
    r, g, b = c
    return (f'<material><ambient>{r} {g} {b} 1</ambient><diffuse>{r} {g} {b} 1</diffuse>'
            f'<specular>0.1 0.1 0.1 1</specular></material>')


def terrain_sdf(t: Terrain, st: dict, name='env_terrain') -> str:
    z0, z1 = -st['bank_depth'], st['bank_height']
    h, zc = z1 - z0, 0.5 * (z0 + z1)
    col = _mat(st['terrain_colour'])
    items = []
    for k, (a, b, c, d) in enumerate(t.rects):
        geo = f'<geometry><box><size>{c - a:.3f} {d - b:.3f} {h:.3f}</size></box></geometry>'
        pose = f'<pose>{0.5 * (a + c):.3f} {0.5 * (b + d):.3f} {zc:.3f} 0 0 0</pose>'
        items.append(f'<collision name="c{k}">{pose}{geo}</collision><visual name="v{k}">{pose}{geo}{col}</visual>')
    return (f'<?xml version="1.0"?><sdf version="1.9"><model name="{name}"><static>true</static>'
            f'<link name="terrain">{"".join(items)}</link>'
            f'<plugin filename="gz-sim-label-system" name="gz::sim::systems::Label"><label>10</label></plugin>'
            f'</model></sdf>')


def structure_sdf(s: dict, st: dict, name: str) -> str:
    parts, _ = structure_parts(s, st)
    items = []
    for k, p in enumerate(parts):
        zc, h = 0.5 * (p.z0 + p.z1), p.z1 - p.z0
        if p.kind == 'cyl':
            geo = f'<geometry><cylinder><radius>{p.l / 2:.3f}</radius><length>{h:.3f}</length></cylinder></geometry>'
        else:
            geo = f'<geometry><box><size>{p.l:.3f} {p.w:.3f} {h:.3f}</size></box></geometry>'
        pose = f'<pose>{p.x:.3f} {p.y:.3f} {zc:.3f} 0 0 {p.yaw:.4f}</pose>'
        if p.collide:
            items.append(f'<collision name="c{k}">{pose}{geo}</collision>')
        if p.visual:
            items.append(f'<visual name="v{k}">{pose}{geo}{_mat(p.colour or (0.6, 0.6, 0.6))}</visual>')
    return (f'<?xml version="1.0"?><sdf version="1.9"><model name="{name}"><static>true</static>'
            f'<pose>{s["x"]:.3f} {s["y"]:.3f} 0 0 0 {math.radians(s["heading_deg"]):.5f}</pose>'
            f'<link name="structure">{"".join(items)}</link>'
            f'<plugin filename="gz-sim-label-system" name="gz::sim::systems::Label"><label>10</label></plugin>'
            f'</model></sdf>')


# ------------------------------------------------------------------ whole environment
def fingerprint(obj) -> str:
    return hashlib.sha1(json.dumps(obj, sort_keys=True).encode()).hexdigest()[:12]


def terrain_key(env) -> str:
    return fingerprint({k: env[k] for k in ('land', 'canals', 'basins')} |
                       {'s': {k: env['settings'][k] for k in ('resolution', 'bank_height', 'bank_depth', 'terrain_colour')}})


