"""Alappuzha (Kerala) backwater traffic on the Sydney Regatta lake -- ONE source of numbers.

  python3 plan.py [out.png]   draw the plan (map + traffic + speed table)
  python3 plan.py build       write ../alappuzha.yaml (traffic) and ../../maps/alappuzha.geojson (the map:
                              sim terrain + structures AND planner 1's chart)

Speeds are typical for each craft (m/s). Rulebook IWAI NW 2002 (Alappuzha is on National Waterway 3).
Scripted traffic never gives way, so no path is laid where it would have to give way to us to stay clear."""
import math
import os
import numpy as np

CANAL_Y, CANAL_W, CANAL_X0, CANAL_X1 = 205.0, 18.0, -505.0, -230.0   # Vadai / Commercial canal
BANK_S, BANK_N = CANAL_Y - CANAL_W / 2, CANAL_Y + CANAL_W / 2         # the canal's bank lines (water edge)
# the canal stream (runs EAST, along the canal): the map's current_mps AND the canal hyacinth's drift
# speed -- one number, so the chart and the sim cannot disagree
CURRENT_MPS = 0.2
CURRENT_DEG = math.degrees(math.atan2(0.0, CANAL_X1 - CANAL_X0))      # along the canal, west -> east
LAND_NPZ = '../../worlds/sydney_regatta_land.npz'
_WORLD = np.load(os.path.join(os.path.dirname(os.path.abspath(__file__)), LAND_NPZ))
# type: (label, size LxB m, sim model, colour)
TYPES = {
    'ferry':     ('KSWTD passenger ferry', '12 m (sim mesh)', 'jarethcruz38819hs-essex', '#1565c0'),
    'houseboat': ('Houseboat (kettuvallam)', '30 x 6.6 (sim mesh)', 'frickies_yacht', '#8d6e63'),
    'cargo':     ('Cargo country boat (canal)', '8 x 3', 'obstacle_barge', '#5d4037'),
    'shikara':   ('Shikara (motor canopy boat)', '5 x 2', 'brazilian', '#ef6c00'),
    'motorboat': ('Motor country boat (fishing)', '8 x 2.9', 'bass_boat', '#6a1b9a'),
    'canoe':     ('Country canoe, poled (crossing)', '6 x 1', 'kayak', '#2e7d32'),
    'kayak':     ('Kayak', '4 x 0.8', 'kayak', '#00897b'),
    'speedboat': ('Speed boat', '10 x 3.8', 'freedom_365', '#c62828'),
    'hyacinth':  ('Water hyacinth patch (drifts)', '3 x 2 flat', 'hyacinth_patch', '#9e9d24'),
}
SIZE = {'canoe': [6.0, 1.0, 0.5],                     # primitives only (the kayak box); meshes keep their size
        'hyacinth': [3.0, 2.0, 0.15],                  # = the hyacinth_patch default
        'W1': [4.0, 2.5, 0.15], 'W4': [5.0, 3.0, 0.15]}  # per mover (id) or per type; id wins


def to_shore(p, deg, size):
    """A patch drifting from p along `deg`: [p, the last point where all of it is still afloat].
    Marches the world water mask only (the canal mouth is open water); check_scenario.py
    checks the map's terrain too."""
    L, o = _WORLD['lakes'] > 0, _WORLD['origin']
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    l, w = size[0], size[1]

    def wet(x, y):
        i, j = int(math.floor(y - o[1])), int(math.floor(x - o[0]))
        return 0 <= i < L.shape[0] and 0 <= j < L.shape[1] and bool(L[i, j])
    k = 0
    while all(wet(p[0] + (k + 1 + l / 2 + 1.0) * c - b * s, p[1] + (k + 1 + l / 2 + 1.0) * s + b * c)
              for b in (-w / 2, 0.0, w / 2)):                      # leading edge + 1 m margin
        k += 1
    return [p, (round(p[0] + k * c, 1), round(p[1] + k * s, 1))]


def kadathu(x_w, l=6.0, b=1.0, gap=1.0, along=3.0, margin=0.3, slope_deg=15.0):
    """One lap of a canoe ferry across the canal, from tied up on the SOUTH bank (index 0) to
    tied up on the NORTH bank (index 4) and back. Counter-clockwise, so each along-bank leg keeps to
    its own lane: east along the south bank, west along the north. Tied up = parallel to the bank,
    its side `gap` off the bank line. A crossing ends `l/2 + margin` off the bank (the bow stays clear
    while it turns); it slides in/out at `slope_deg`, the steepest that keeps the hull >= 0.2 m off
    the bank at the corner, then `along` metres along the bank."""
    ys, yn = BANK_S + gap + b / 2, BANK_N - gap - b / 2
    h = l / 2 + margin
    run = round((h - gap - b / 2) / math.tan(math.radians(slope_deg)), 2)
    x_e = x_w + 2 * run + along
    pts = [(x_w + run + along, ys), (x_e, BANK_S + h), (x_e, BANK_N - h), (x_e - run, yn),
           (x_e - run - along, yn), (x_w, BANK_N - h), (x_w, BANK_S + h), (x_w + run, ys)]
    return [(round(x, 2), round(y, 2)) for x, y in pts]


# kadathu dwell at each bank, s: passengers on and off. Swept with tools/check_scenario.py: no crossing
# meets the shikara S1 (~t 89-99 s) or the ferry F2 (~t 288-303 s) mid-canal for a dwell of 14-18,
# 23-31 or 62-98 s; 80 s sits in the widest band. Re-run the checker after moving S1, F2 or the canoes.
KADATHU_DWELL = 80.0
_K = kadathu(-457.0)
# id, type, path [(x, y)...], speed m/s, loop, stops [[point index, s]], start delay s, role.
# Canal lanes: eastbound y 200, westbound y 205.5
MOVERS = [
    # the canal (eastbound lane y 200, westbound y 205; the moored row narrows it from the north)
    ('F1', 'ferry', [(-478, 224), (-478, 205), (-515, 205), (-575, 290)], 2.5, False, [], 40, 'leaves the jetty westbound, astern of us'),
    ('F2', 'ferry', [(200, 160), (-190, 205), (-470, 205), (-478, 224)], 2.5, False, [], 35, 'lake -> jetty, crosses our lake leg'),
    ('S1', 'shikara', [(-240, 205), (-515, 205), (-560, 240)], 2.5, False, [], 10, 'head-on in the canal'),
    ('H1', 'cargo', [(-440, 200), (-215, 200), (-215, 290)], 1.2, False, [], 0, 'slow, ahead of us in the canal: overtake with care'),
    ('C1', 'canoe', _K, 0.8, True, [[0, KADATHU_DWELL], [4, KADATHU_DWELL]], 0, 'kadathu shuttle, ties up at each bank'),
    ('C2', 'canoe', _K[4:] + _K[:4], 0.8, True, [[0, KADATHU_DWELL], [4, KADATHU_DWELL]], 0, 'second kadathu, half a lap behind'),
    ('W1', 'hyacinth', to_shore((-480, 199), CURRENT_DEG, SIZE['W1']), CURRENT_MPS, False, [], 0, 'soft: drifts with the current to the shore'),
    ('W2', 'hyacinth', to_shore((-380, 204), CURRENT_DEG, SIZE['hyacinth']), CURRENT_MPS, False, [], 0, ''),
    ('W3', 'hyacinth', to_shore((-290, 202), CURRENT_DEG, SIZE['hyacinth']), CURRENT_MPS, False, [], 0, ''),
    # the lake: each loop in its own water, crossings in their own corridors
    ('F4', 'ferry', [(-226, 135), (-212, 192), (100, 221), (250, 205)], 2.8, False, [], 215, 'overtakes us on our port side, lake leg'),
    ('F3', 'ferry', [(50, 290), (50, 165)], 2.8, True, [], 20, 'crosses the lake north-south (x 50)'),
    ('M1', 'motorboat', [(0, 295), (0, 165)], 3.0, True, [], 0, 'crosses our lake leg north-south (x 0)'),
    ('H2', 'houseboat', [(-200, 172), (-80, 172), (-80, 245), (-200, 245)], 2.0, True, [], 0, 'lake cruise loop (west)'),
    ('H3', 'houseboat', [(272, 300), (272, 268), (310, 230), (310, 140)], 1.6, False, [], 60, 'leaves the terminal: frees our berth'),
    ('S2', 'shikara', [(-120, 280), (-40, 280), (-40, 257), (-120, 257)], 2.5, True, [], 0, 'tourist loop by the nets (north)'),
    ('B1', 'speedboat', [(95, 165), (235, 165), (235, 232), (95, 232)], 6.0, True, [], 0, 'fast circuit (east), across our lake leg'),
    ('K1', 'kayak', [(-150, 140), (160, 140)], 1.1, True, [], 0, 'south shore'),
    ('K2', 'kayak', [(-130, 146), (180, 146)], 1.1, True, [], 15, 'south shore'),
    # the chart gives only canals a current (vmap current_mps), so the lake patch drifts at the same chart value
    ('W4', 'hyacinth', to_shore((-60, 200), 0.0, SIZE['W4']), CURRENT_MPS, False, [], 0, 'lake patch, drifts east'),
]
MOORED_CANAL = [(-400, 210.4), (-366, 210.4), (-300, 210.4), (-266, 210.4)]         # north bank, 30 x 6.6
TERMINAL = [(200 + 9 * k, 300) for k in range(8)]                                     # bow-in, side by side
BRIDGES = [(-430, 'Iron bridge'), (-330, 'Kalvathi bridge')]                          # deck 6 m, 3 m clearance
NETS = [(-100, 305, 0, 305), (170, 138, 230, 138)]                                    # stake-net lines
JETTY = (-490, -462, 214, 234)                                                        # KSWTD boat jetty basin
OURS = [(-528, 186), (-505, 200), (-232, 200), (100, 215), (272, 272)]
GOAL = (272, 280)
VCLASS = {'ferry': 'power_driven', 'houseboat': 'power_driven', 'cargo': 'power_driven', 'shikara': 'power_driven', 'motorboat': 'fishing',
          'canoe': None, 'kayak': None, 'speedboat': 'power_driven', 'hyacinth': None}


def hdg(a, b):
    return round(math.degrees(math.atan2(b[1] - a[1], b[0] - a[0])), 1)


def draw(out='plan.png'):
    import os
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle, FancyArrowPatch
    d = _WORLD
    L, o = d['lakes'] > 0, d['origin']
    fig = plt.figure(figsize=(22, 13))
    ax = fig.add_axes([0.03, 0.40, 0.94, 0.57])
    ax.imshow(np.where(L, 1, 0), origin='lower', extent=[o[0], o[0] + L.shape[1], o[1], o[1] + L.shape[0]],
              cmap=matplotlib.colors.ListedColormap(['#7c9a6b', '#bfe3f5']), interpolation='nearest')
    ax.add_patch(Rectangle((CANAL_X0, 115), CANAL_X1 - CANAL_X0, 240, color='#6f8f5e'))
    ax.add_patch(Rectangle((CANAL_X0, CANAL_Y - CANAL_W / 2), CANAL_X1 - CANAL_X0, CANAL_W, color='#9fd3ef'))
    x0, x1, y0, y1 = JETTY
    ax.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0, color='#9fd3ef'))
    ax.add_patch(Rectangle((x0, y1 - 2), x1 - x0, 2, color='#555'))
    ax.text(x0, y1 + 3, 'KSWTD boat jetty', fontsize=9, weight='bold')
    for bx, name in BRIDGES:
        ax.add_patch(Rectangle((bx - 3, CANAL_Y - CANAL_W / 2 - 4), 6, CANAL_W + 8, color='#444', alpha=.8))
        ax.text(bx, CANAL_Y + CANAL_W / 2 + 6, name + '\n(3.0 m clear)', ha='center', fontsize=8)
    for (x, y) in MOORED_CANAL:
        ax.add_patch(Rectangle((x - 15, y - 3.3), 30, 6.6, color='#8d6e63', ec='k', lw=.5))
    for (x, y) in TERMINAL:
        ax.add_patch(Rectangle((x - 3.3, y - 15), 6.6, 30, color='#8d6e63', ec='k', lw=.5))
    ax.text(196, 318, 'Houseboat terminal (8 moored)', fontsize=9, weight='bold')
    for xa, ya, xb, yb in NETS:
        ax.plot(np.linspace(xa, xb, 11), np.linspace(ya, yb, 11), 'k|', ms=9)
    ax.text(-100, 311, 'stake fishing nets', fontsize=8)
    ax.text(170, 129, 'stake nets', fontsize=8)
    ax.text(-480, 170, 'VADAI / COMMERCIAL CANAL  (18 m wide, 5 kn, keep starboard)', fontsize=9, color='w', weight='bold')
    ax.text(-60, 338, 'PUNNAMADA / VEMBANAD LAKE (open water)', fontsize=11, weight='bold', color='#0d47a1')
    for mid, t, pts, v, loop, stops, delay, note in MOVERS:
        col = TYPES[t][3]
        P = np.array(pts + ([pts[0]] if loop else []))
        ax.plot(P[:, 0], P[:, 1], '-', color=col, lw=1.6, alpha=.85)
        for a, b in zip(P[:-1], P[1:]):
            m = (a + b) / 2
            ax.add_patch(FancyArrowPatch(m - (b - a) * 0.02, m + (b - a) * 0.02, arrowstyle='-|>', mutation_scale=14, color=col))
        ax.plot(*pts[0], 'o', color=col, ms=6)
        ax.annotate(f"{mid} {v:g} m/s" + (f" (t+{delay}s)" if delay else ''), pts[0], textcoords='offset points',
                    xytext=(4, 4), fontsize=8, color=col, weight='bold')
    R = np.array(OURS)
    ax.plot(R[:, 0], R[:, 1], 'r--', lw=3, label='OUR BOAT: mission route')
    ax.plot(*OURS[0], 'r*', ms=16)
    ax.annotate('START (spawn)', OURS[0], xytext=(-30, -14), textcoords='offset points', color='r', weight='bold')
    ax.plot(*GOAL, 'rX', ms=14)
    ax.annotate('GOAL (berth H3 leaves)', GOAL, xytext=(8, -12), textcoords='offset points', color='r', weight='bold')
    ax.set_xlim(-560, 340)
    ax.set_ylim(115, 350)
    ax.set_aspect('equal')
    ax.set_title('Alappuzha backwater traffic on the Regatta lake (world metres, ENU; rulebook IWAI NW 2002, NW-3)', fontsize=13)
    ax.grid(alpha=.25)
    ax.legend(loc='lower left')
    tb = fig.add_axes([0.03, 0.02, 0.94, 0.34])
    tb.axis('off')
    rows = [[mid, TYPES[t][0], ('{:g} x {:g}'.format(*SIZE[mid][:2]) if mid in SIZE else TYPES[t][1]), TYPES[t][2], f"{v:g}", f"{v * 3.6:.1f}", f"{v / 0.5144:.1f}",
             (f'shuttle, {stops[0][1]:g} s ashore' if stops else 'loop') if loop else 'once', f"{delay}", note]
            for mid, t, pts, v, loop, stops, delay, note in MOVERS]
    rows.append(['--', 'Houseboats MOORED (canal bank 4, terminal 8)', '30 x 6.6', 'frickies_yacht', '0', '0', '0', 'static', '', 'narrow the canal to 11.4 m'])
    rows.append(['US', 'Our boat', '4 x 1.7', 'my_boat', '2.5', '9.0', '4.9', '', '', 'cruise; canal limit 5 kn = 2.57 m/s'])
    tab = tb.table(cellText=rows, colLabels=['id', 'craft', 'L x B (m)', 'sim model', 'm/s', 'km/h', 'kn', 'motion',
                   'start delay s', 'role in the test'], loc='center', cellLoc='left',
                   colWidths=[.03, .2, .08, .14, .04, .045, .04, .05, .07, .26])
    tab.auto_set_font_size(False)
    tab.set_fontsize(9)
    tab.scale(1, 1.22)
    for (r, c), cell in tab.get_celld().items():
        if 0 < r <= len(MOVERS) and c == 1:
            cell.set_text_props(color=TYPES[MOVERS[r - 1][1]][3], weight='bold')
    fig.savefig(out, dpi=90)


def scenario_yaml():
    obs = []
    for mid, t, pts, v, loop, stops, delay, note in MOVERS:
        o = {'name': f'{mid.lower()}_{t}', 'model': TYPES[t][2]}
        if VCLASS[t]:
            o['vessel_class'] = VCLASS[t]
        if mid in SIZE or t in SIZE:
            o['size'] = list(SIZE.get(mid, SIZE.get(t)))
        if t == 'hyacinth':
            o['collision'] = 'none'             # soft: boats push through it (still detected and avoided)
        if delay:
            o['start_delay'] = float(delay)
        # no `start`: a waypoint path starts at its first point (a start is put IN FRONT of the points)
        o['motion'] = {'type': 'waypoints', 'speed': float(v), 'loop': bool(loop),
                       'points': [[float(x), float(y)] for x, y in pts]}
        if stops:
            o['motion']['stops'] = [[int(k), float(sec)] for k, sec in stops]
        obs.append(o)
    for k, (x, y) in enumerate(MOORED_CANAL):
        obs.append({'name': f'moored_canal_{k + 1}', 'model': 'frickies_yacht', 'vessel_class': 'power_driven',
                    'start': {'x': float(x), 'y': float(y), 'heading_deg': 0.0}})
    for k, (x, y) in enumerate(TERMINAL):
        obs.append({'name': f'moored_terminal_{k + 1}', 'model': 'frickies_yacht', 'vessel_class': 'power_driven',
                    'start': {'x': float(x), 'y': float(y), 'heading_deg': 90.0}})
    return {'scenario': {'name': 'alappuzha', 'duration_s': 1800}, 'settings': {'duration_s': 1800.0}, 'obstacles': obs}


def vector_map(template):
    import copy
    from marine_vector_map import vmap
    W = CANAL_W

    def rect(x0, x1, y0, y1):
        return [[[x0, y0], [x1, y0], [x1, y1], [x0, y1], [x0, y0]]]

    def F(geom, coords, **p):
        return {'type': 'Feature', 'geometry': {'type': geom, 'coordinates': coords}, 'properties': p}
    feats = [
        F('Polygon', rect(CANAL_X0, CANAL_X1, 100, 360), **{'class': 'LNDARE', 'name': 'alappuzha_town',
          'description': 'the town the canal is cut through'}),
        F('LineString', [[CANAL_X0, CANAL_Y], [CANAL_X1, CANAL_Y]], **{'class': 'CANALS', 'name': 'vadai_canal',
          'depth': 3.0, 'traffic': 'two_way', 'keep': 'starboard', 'overtaking': True, 'max_speed_kn': 5.0,
          'current_dir': 'along', 'current_mps': CURRENT_MPS, 'width': W, 'bank_width': 1.0, 'open_start': True,
          'open_end': True, 'description': 'town canal, west mouth by the boat jetty to the lake; the stream runs EAST'}),
        F('Point', [-476.0, 224.0], **{'class': 'HRBBSN', 'name': 'kswtd_jetty_basin', 'depth': 2.5, 'length': 28.0,
          'width': 20.0, 'heading_deg': 0.0, 'description': 'KSWTD boat jetty basin'}),
        F('Point', [-476.0, 232.5], **{'class': 'PONTON', 'name': 'kswtd_jetty', 'type': 'pontoon', 'heading_deg': 0.0,
          'length': 24.0, 'width': 2.5, 'fingers': 2, 'finger_length': 5.0, 'finger_width': 1.2,
          'finger_side': 'right', 'deck_height': 0.5, 'description': 'KSWTD passenger ferry jetty'}),
    ]
    for bx, name in BRIDGES:
        feats.append(F('Point', [float(bx), CANAL_Y], **{'class': 'BRIDGE', 'name': name.lower().replace(' ', '_'),
                       'type': 'bridge', 'heading_deg': 90.0, 'span': W + 8.0, 'width': 6.0, 'clearance': 3.0,
                       'deck_thickness': 0.8, 'piers': 0, 'pier_width': 1.5, 'abutment_length': 4.0,
                       'description': f'{name}, 3.0 m clearance, no pier'}))
    for k, (xa, ya, xb, yb) in enumerate(NETS):
        feats.append(F('Point', [(xa + xb) / 2.0, (ya + yb) / 2.0], **{'class': 'FSHFAC', 'name': f'stake_nets_{k + 1}',
                       'type': 'net', 'heading_deg': hdg((xa, ya), (xb, yb)), 'length': float(math.hypot(xb - xa, yb - ya)),
                       'float_spacing': 2.0, 'depth': 1.5, 'description': 'stake fishing nets'}))
    feats += [
        F('Polygon', rect(CANAL_X1, 340, 100, 360), **{'class': 'DEPARE', 'name': 'punnamada_lake', 'drval1': 3.0,
          'drval2': 5.0, 'description': 'Punnamada / Vembanad lake: open water'}),
        F('Polygon', rect(-620, CANAL_X0, 140, 400), **{'class': 'DEPARE', 'name': 'jetty_approach', 'drval1': 3.0,
          'drval2': 5.0, 'description': 'open water west of the town, off the canal mouth by the boat jetty'}),
        F('Polygon', rect(CANAL_X0 - 10, CANAL_X1 + 10, 190, 240), **{'class': 'SPDLIM', 'name': 'town_canal_5kn',
          'max_speed_kn': 5.0, 'description': 'the town canal'}),
    ]
    # SCENE CONTEXT (DISCUSSION_18 P7): the kadathu's two crossing legs (the canoes' lap), the jetty
    # where the ferries berth, the bank the houseboats moor along; the nets are their FSHFAC above
    for k, x in enumerate((_K[5][0], _K[1][0])):
        feats.append(F('LineString', [[x, BANK_S], [x, BANK_N]], **{'class': 'FERYRT', 'name': f'kadathu_{k + 1}',
                       'catfry': 'free_moving', 'crossing': True, 'max_speed_kn': 3.0, 'priority': 'crosser',
                       'description': 'canoe ferry crossing at the boat jetty: canoes cross and tie up'}))
    x0, x1, y0, y1 = JETTY
    feats.append(F('Polygon', rect(x0, x1, y0, y1), **{'class': 'BERTHS', 'name': 'kswtd_jetty_berths',
                   'description': 'KSWTD jetty: the ferries berth and depart'}))
    feats.append(F('Polygon', rect(MOORED_CANAL[0][0] - 15, MOORED_CANAL[-1][0] + 15, MOORED_CANAL[0][1] - 3.3, BANK_N),
                   **{'class': 'BERTHS', 'name': 'houseboat_bank', 'description': 'houseboats moor along the north bank'}))
    fc = {'type': 'FeatureCollection', 'properties': copy.deepcopy(template['properties']), 'features': feats}
    fc['properties']['name'] = 'Alappuzha backwaters (simulation): town canal + Punnamada lake'
    fc['properties']['rulebook'] = 'iwai_nw_2002'
    return vmap.validate(vmap.to_geojson(fc))


def build(share_dir):
    import os
    import yaml
    from marine_vector_map import vmap
    tpl = vmap.load(os.path.join(share_dir, 'maps', 'regatta_canals.geojson'))
    with open(os.path.join(share_dir, 'maps', 'alappuzha.geojson'), 'w') as fh:
        fh.write(vmap.dumps(vector_map(tpl)))
    with open(os.path.join(share_dir, 'scenarios', 'alappuzha.yaml'), 'w') as fh:
        fh.write('# GENERATED by scenarios/alappuzha/plan.py -- edit the plan, then: python3 plan.py build\n')
        yaml.safe_dump(scenario_yaml(), fh, sort_keys=False, default_flow_style=None, width=120)


if __name__ == '__main__':
    import os
    import sys
    here = os.path.dirname(os.path.abspath(__file__))
    if len(sys.argv) > 1 and sys.argv[1] == 'build':
        build(os.path.dirname(os.path.dirname(here)))
    else:
        draw(sys.argv[1] if len(sys.argv) > 1 else os.path.join(here, 'plan.png'))
