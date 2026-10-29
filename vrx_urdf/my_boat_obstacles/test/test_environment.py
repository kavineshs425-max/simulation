import math

import numpy as np
import pytest
import yaml

from my_boat_obstacles import environment as E
from my_boat_obstacles.ui_node import dump_environment, merge_env_file


def env(**kw):
    d = {'canals': [], 'basins': [], 'land': [], 'structures': []}
    d.update(kw)
    return E.parse_environment(d)


def straight(name='c', width=20.0, bank=8.0, **kw):
    c = {'name': name, 'points': [[0, 0], [100, 0]], 'width': width, 'bank_width': bank}
    c.update(kw)
    return c


# ---------------------------------------------------------------- terrain rule
def test_canal_in_open_water_gets_banks_and_open_ends():
    t = E.build_terrain(env(canals=[straight()]))
    assert not t.is_solid(50, 0) and not t.is_solid(50, 9.4)        # water, 20 m wide
    assert t.is_solid(50, 11) and t.is_solid(50, -17.5)               # banks, 8 m each side
    assert not t.is_solid(50, 19)                                      # open water beyond the bank
    assert not t.is_solid(-3, 0) and not t.is_solid(103, 0)           # open ends: no bank across


def test_closed_end_is_a_dead_end():
    t = E.build_terrain(env(canals=[straight(open_end=False)]))
    assert t.is_solid(100 + 12, 0)                                     # bank wraps round the end
    assert not t.is_solid(-3, 0)


def test_junction_cuts_banks_and_does_not_notch_the_far_bank():
    main = {'name': 'main', 'points': [[-100, 0], [100, 0]], 'width': 20, 'bank_width': 8}
    branch = {'name': 'branch', 'points': [[0, 0], [0, 80]], 'width': 10, 'bank_width': 6}
    t = E.build_terrain(env(canals=[main, branch]))
    assert not t.is_solid(0, 20) and not t.is_solid(0, 11)            # branch water crosses main's north bank
    assert t.is_solid(0, -14)                                          # ...but NOT main's south bank
    assert t.is_solid(8, 20) and t.is_solid(-8, 20)                    # the branch has banks


def test_land_region_with_canal_and_basin():
    land = {'name': 'l', 'points': [[-50, -50], [150, -50], [150, 50], [-50, 50]]}
    basin = {'name': 'b', 'x': 50, 'y': 25, 'length': 20, 'width': 20}
    t = E.build_terrain(env(land=[land], canals=[straight()], basins=[basin]))
    assert not t.is_solid(50, 30) and not t.is_solid(50, 0)
    assert t.is_solid(-40, 40) and t.is_solid(140, -40)


def test_world_land_is_not_duplicated():
    t = E.build_terrain(env(canals=[straight()]), lambda X, Y: Y > 12)
    assert not t.is_solid(50, 15) and t.is_solid(50, -15)


def test_rectangles_cover_exactly_the_solid_cells():
    t = E.build_terrain(env(canals=[{'name': 'k', 'points': [[0, 0], [40, 30], [90, 30]], 'width': 12, 'bank_width': 5}]))
    area = sum((c - a) * (d - b) for a, b, c, d in t.rects)
    assert area == pytest.approx(t.solid.sum() * t.res ** 2)
    assert len(t.rects) < t.solid.sum() / 4                           # actually merged


# ---------------------------------------------------------------- structures
def test_pontoon_berths_and_docking_pose():
    e = env(structures=[{'name': 'p', 'type': 'pontoon', 'x': 10, 'y': 5, 'heading_deg': 90,
                         'length': 40, 'fingers': 5}])
    b = E.berths_world(e)
    assert len(b) == 4
    # heading 90: the walkway runs north, fingers to the WEST (left of north); the
    # boat's bow points west, its centre 1.25 + 1 + 3 = 5.25 m west of the walkway
    assert b[0]['x'] == pytest.approx(10 - 5.25) and abs(E.wrap(b[0]['heading'] - math.pi)) < 1e-3
    assert b[0]['width'] == pytest.approx((40 - 1.2) / 4 - 1.2)


def test_bridge_deck_does_not_block_but_piers_do():
    e = env(structures=[{'name': 'br', 'type': 'bridge', 'x': 0, 'y': 0, 'span': 30, 'clearance': 3.0, 'piers': 1}])
    parts, _ = E.structure_parts(e['structures'][0], e['settings'])
    deck = parts[0]
    assert not E.blocks(deck, e['settings'])
    assert sum(E.blocks(p, e['settings']) for p in parts) == 3        # 2 abutments + 1 pier
    low = env(structures=[{'name': 'br', 'type': 'bridge', 'x': 0, 'y': 0, 'clearance': 1.5}])
    assert E.blocks(E.structure_parts(low['structures'][0], low['settings'])[0][0], low['settings'])


def test_net_is_invisible_but_blocks_and_is_on_the_map():
    e = env(structures=[{'name': 'n', 'type': 'net', 'x': 10, 'y': 10, 'heading_deg': 90, 'length': 8}])
    parts, _ = E.structure_parts(e['structures'][0], e['settings'])
    barrier = [p for p in parts if not p.visual]
    assert len(barrier) == 1 and E.blocks(barrier[0], e['settings'])
    assert all(not p.collide for p in parts if p.visual)              # floats: seen, not hit
    occ = E.occupancy(e, None, 0, 0, 1.0, (30, 30))
    assert occ[10:18, 10].all() and occ.sum() <= 12                    # a 5 cm net still marks its cells


def test_sdf_is_static_and_parses():
    import xml.etree.ElementTree as ET
    e = env(canals=[straight()], structures=[{'name': 'j', 'type': 'jetty', 'x': 0, 'y': 0}])
    t = E.build_terrain(e)
    for text in (E.terrain_sdf(t, e['settings']), E.structure_sdf(e['structures'][0], e['settings'], 'env_j')):
        root = ET.fromstring(text)
        assert root.find('model/static').text == 'true'


# ---------------------------------------------------------------- config
def test_bad_config_is_refused_with_a_useful_message():
    with pytest.raises(ValueError, match='type must be one of'):
        env(structures=[{'name': 'x', 'type': 'castle', 'x': 0, 'y': 0}])
    with pytest.raises(ValueError, match='unknown key'):
        env(structures=[{'name': 'x', 'type': 'wall', 'x': 0, 'y': 0, 'lenght': 3}])
    with pytest.raises(ValueError, match='duplicate'):
        env(canals=[straight('a')], basins=[{'name': 'a', 'x': 0, 'y': 0, 'length': 1, 'width': 1}])
    with pytest.raises(ValueError, match='fingers must be'):
        env(structures=[{'name': 'x', 'type': 'pontoon', 'x': 0, 'y': 0, 'fingers': 99}])


def test_default_map_builds_and_the_yaml_save_round_trips():
    import os
    from marine_vector_map import vmap
    path = os.path.join(os.path.dirname(__file__), '..', 'maps', 'regatta_canals.geojson')
    a = E.parse_environment(vmap.to_environment(vmap.load(path)))
    assert {c['name'] for c in a['canals']} == {'main_canal', 'loop_canal', 'backwater'}
    t = E.build_terrain(a)
    assert t is not None and len(t.rects) > 50
    header = '# my environment\n# comment\n'
    saved = merge_env_file(header + dump_environment(a), a)
    assert saved.startswith('# my environment')                         # the header comments are kept
    assert E.parse_environment(yaml.safe_load(saved)) == a
