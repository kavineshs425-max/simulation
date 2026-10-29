import glob
import math
import os
import re
import xml.etree.ElementTree as ET

import pytest
import yaml

from my_boat_obstacles.catalog import MODELS, RUN_TOPIC, class_id, model_spec, obstacle_sdf
from my_boat_obstacles.scenario import (
    KNOT, SETTINGS_DEFAULTS, ConstantTurn, ObstacleSpec, Pose2D, WaypointMotion, build, load_scenario,
    parse_scenario, wrap)

HERE = os.path.dirname(os.path.abspath(__file__))
CONFIG = os.path.join(HERE, '..', 'config', 'obstacles.yaml')
SCENARIOS = sorted(glob.glob(os.path.join(HERE, '..', 'scenarios', '*.yaml'))) + [CONFIG]
SPAWN = (-528.0, 186.0)          # my_boat default spawn, facing east


@pytest.fixture(autouse=True)
def _user_models_registered():
    """The default config uses models from models/, as the spawner would."""
    from my_boat_obstacles import catalog as _cat, user_models as _um
    _cat.set_user_models(_um.scan([os.path.join(HERE, '..', 'models')]))
    yield
    _cat.set_user_models({})


def cv(x, y, heading_deg, speed, **kw):
    return build(ObstacleSpec('a', start={'x': x, 'y': y, 'heading_deg': heading_deg},
                              motion={'type': 'constant_velocity', 'speed': speed}, **kw), 1)


# ---------------------------------------------------------------- start / run
def test_start_pose_is_absolute_world_coordinates():
    s = cv(-500.0, 150.0, 90.0, 1.0).state(0.0)
    assert (s.x, s.y, s.heading) == pytest.approx((-500.0, 150.0, math.pi / 2))


def test_nothing_moves_before_run():
    ob = cv(-500.0, 150.0, 90.0, 1.0)
    for tau in (-1.0, -100.0):
        s = ob.state(tau)
        assert (s.x, s.y, s.speed) == pytest.approx((-500.0, 150.0, 0.0))
    s = ob.state(10.0)
    assert (s.x, s.y, s.speed) == pytest.approx((-500.0, 160.0, 1.0))


def test_start_delay_waits_after_run_then_moves():
    ob = cv(0.0, 0.0, 0.0, 2.0, start_delay=5.0)
    assert (ob.state(4.9).x, ob.state(4.9).speed) == pytest.approx((0.0, 0.0))
    assert (ob.state(7.0).x, ob.state(7.0).speed) == pytest.approx((4.0, 2.0))


def test_static_needs_no_motion_block_and_keeps_heading():
    s = build(ObstacleSpec('b', model='barge', start={'x': 1.0, 'y': 2.0, 'heading_deg': 30.0}),
              1).state(50.0)
    assert (s.x, s.y, s.speed) == (1.0, 2.0, 0.0)
    assert s.heading == pytest.approx(math.radians(30.0))


def test_waypoints_start_at_first_point_or_at_start():
    a = build(ObstacleSpec('a', motion={'type': 'waypoints', 'speed': 1.0,
                                        'points': [[0, 0], [10, 0]]}), 1)
    b = build(ObstacleSpec('b', start={'x': -5.0, 'y': 0.0},
                           motion={'type': 'waypoints', 'speed': 1.0, 'points': [[0, 0], [10, 0]]}), 2)
    assert a.state(-1.0).x == 0.0 and b.state(-1.0).x == -5.0
    assert b.state(8.0).x == pytest.approx(3.0)


# ---------------------------------------------------------------- motion
def test_constant_turn_matches_numeric_integration():
    m = ConstantTurn(0.0, 0.0, 0.3, 1.2, math.radians(4.0))
    x = y = 0.0
    c = 0.3
    dt = 1e-3
    for _ in range(int(30.0 / dt)):
        x += 1.2 * math.cos(c + 0.5 * m.rate * dt) * dt
        y += 1.2 * math.sin(c + 0.5 * m.rate * dt) * dt
        c += m.rate * dt
    xs, ys, cs, _ = m.at(30.0)
    assert (xs, ys) == pytest.approx((x, y), abs=1e-4)
    assert wrap(cs - c) == pytest.approx(0.0, abs=1e-9)


def test_waypoints_hold_at_end_with_zero_speed():
    m = WaypointMotion([(0.0, 0.0), (10.0, 0.0), (10.0, 10.0)], 2.0)
    assert m.at(2.5)[:2] == pytest.approx((5.0, 0.0))
    assert m.at(7.5)[:3] == pytest.approx((10.0, 5.0, math.pi / 2))
    assert (m.at(100.0)[0], m.at(100.0)[1], m.at(100.0)[3]) == pytest.approx((10.0, 10.0, 0.0))


def test_waypoints_loop_closes_back_to_start():
    m = WaypointMotion([(0.0, 0.0), (10.0, 0.0), (10.0, 10.0)], 1.0, loop=True)
    perimeter = 10.0 + 10.0 + math.hypot(10.0, 10.0)
    assert m.at(perimeter + 3.0)[:2] == pytest.approx((3.0, 0.0), abs=1e-9)


def test_crab_separates_heading_from_course():
    s = cv(0.0, 0.0, 90.0, 1.0, crab_deg=15.0).state(5.0)
    assert s.course == pytest.approx(math.radians(90.0))
    assert s.heading == pytest.approx(math.radians(105.0))
    assert (s.vx, s.vy) == pytest.approx((0.0, 1.0), abs=1e-12)


def test_body_frame_for_cpa_log():
    assert Pose2D(0.0, 0.0, math.pi / 2).to_body(0.0, 10.0) == pytest.approx((10.0, 0.0))


# ---------------------------------------------------------------- shipped scenarios
@pytest.mark.parametrize('path', SCENARIOS, ids=[os.path.basename(p) for p in SCENARIOS])
def test_shipped_scenarios_load_and_build_valid_sdf(path):
    sc = load_scenario(path)
    obs = sc.build()
    assert len(obs) == len(sc.obstacles) > 0
    for o in obs:
        smp = o.samples(sc.duration_s)
        root = ET.fromstring(obstacle_sdf(o.spec.model, f'x_{o.spec.name}', o.spec.det_class,
                                          smp, '/x/libMovingObstacle.so'))
        assert root.find('model') is not None


def _designs():
    out = []
    for path in SCENARIOS:
        for line in open(path):
            m = re.match(r'#\s*design:\s*(\{.*\})\s*$', line)
            if m:
                out.append((os.path.basename(path), yaml.safe_load(m.group(1))))
    return out


@pytest.mark.parametrize('fname, d', _designs(), ids=[f'{f}:{d.get("obstacle", "")}'
                                                        for f, d in _designs()])
def test_shipped_encounters_hit_their_documented_cpa(fname, d):
    """Each '# design:' comment is a promise: if my_boat runs from its spawn
    at own_speed on own_heading_deg from the moment of run, the named obstacle
    passes at dcpa (+ = to our port) tcpa seconds after run."""
    sc = load_scenario(os.path.join(HERE, '..', 'scenarios', fname))
    obs = {o.spec.name: o for o in sc.build()}
    ob = obs[d['obstacle']] if 'obstacle' in d else next(iter(obs.values()))
    hd = math.radians(d['own_heading_deg'])
    best = (1e9, 0.0, 0.0)
    tau = 0.0
    while tau <= d['tcpa'] * 2:
        ox = SPAWN[0] + d['own_speed'] * math.cos(hd) * tau
        oy = SPAWN[1] + d['own_speed'] * math.sin(hd) * tau
        s = ob.state(tau)
        r = math.hypot(s.x - ox, s.y - oy)
        if r < best[0]:
            best = (r, tau, Pose2D(ox, oy, hd).to_body(s.x, s.y)[1])
        tau += 0.01
    r, t, side = best
    assert t == pytest.approx(d['tcpa'], abs=0.3)
    assert r == pytest.approx(abs(d['dcpa']), abs=0.3)
    if d['dcpa'] != 0.0:
        assert math.copysign(1.0, side) == math.copysign(1.0, d['dcpa'])


def test_typos_are_errors_not_silently_ignored():
    with pytest.raises(ValueError, match='unknown keys'):
        parse_scenario({'obstacles': [{'name': 'a', 'strat': {}}]})
    with pytest.raises(ValueError, match='unknown top-level'):
        parse_scenario({'detector': {}, 'obstacles': []})
    with pytest.raises(ValueError, match='unknown start keys'):
        parse_scenario({'obstacles': [{'name': 'a', 'start': {'x': 0, 'y': 0, 'yaw': 1}}]})
    with pytest.raises(ValueError, match='unknown constant_velocity keys'):
        parse_scenario({'obstacles': [{'name': 'a', 'start': {'x': 0, 'y': 0},
                                       'motion': {'type': 'constant_velocity', 'speed': 1,
                                                  'course_rel_deg': 90}}]})
    with pytest.raises(ValueError, match='start needs x and y'):
        build(ObstacleSpec('a', start={'x': 1.0}), 1)
    with pytest.raises(ValueError, match='vessel_class'):
        build(ObstacleSpec('a', vessel_class='tug', start={'x': 0, 'y': 0}), 1)
    with pytest.raises(ValueError, match='unknown model'):
        build(ObstacleSpec('a', model='titanic', start={'x': 0, 'y': 0}), 1)
    with pytest.raises(ValueError, match='unknown class'):
        build(ObstacleSpec('a', det_class='whale', start={'x': 0, 'y': 0}), 1)


def test_class_defaults_from_model_and_can_be_overridden():
    a = build(ObstacleSpec('a', model='freedom_365', start={'x': 0, 'y': 0}), 1)
    b = build(ObstacleSpec('b', model='freedom_365', det_class='boat', start={'x': 0, 'y': 0}), 2)
    assert (a.spec.det_class, b.spec.det_class) == ('ship', 'boat')


# ---------------------------------------------------------------- samples
def test_samples_start_at_run_and_match_state():
    ob = cv(10.0, 0.0, 0.0, 1.5, start_delay=4.0)
    smp = ob.samples(duration=10.0, step=0.5)
    assert smp[0][0] == 0.0 and smp[-1][0] == pytest.approx(10.0)
    assert 4.0 in [t for t, *_ in smp]                      # exact start of motion
    assert all(b[0] > a[0] for a, b in zip(smp, smp[1:]))   # plugin requires it
    for tau, x, y, yaw in smp:
        s = ob.state(tau)
        assert (x, y, yaw) == pytest.approx((s.x, s.y, s.heading))


def test_linear_playback_of_samples_stays_on_a_turn():
    """The plugin interpolates linearly between samples; that must stay close
    to the true arc (chord error R(1-cos(r*dt/2)))."""
    ob = build(ObstacleSpec('a', start={'x': 30.0, 'y': 0.0},
                            motion={'type': 'constant_turn', 'speed': 1.5,
                                    'turn_rate_deg_s': 6.0}), 1)
    smp = ob.samples(60.0, 0.5)
    worst = 0.0
    for (ta, xa, ya, _), (tb, xb, yb, _) in zip(smp, smp[1:]):
        s = ob.state(0.5 * (ta + tb))
        worst = max(worst, math.hypot(0.5 * (xa + xb) - s.x, 0.5 * (ya + yb) - s.y))
    assert worst < 0.01


# ---------------------------------------------------------------- catalog / SDF
def test_vessel_mesh_is_rotated_bow_forward_and_centred():
    """Mesh bow is at -Y. After the visual pose (translate t, yaw +90) the
    mesh bbox centre must land on the model origin and mesh -Y on model +X."""
    measured = {'bass_boat': (0.000, -0.583), 'brazilian': (-0.009, -0.497),
                'freedom_365': (0.072, 0.208)}          # STL bbox centres (x, y)
    for name in ('bass_boat', 'brazilian', 'freedom_365'):
        m = MODELS[name]
        root = ET.fromstring(obstacle_sdf(name, 'x', 'boat', [(0, 0, 0, 0), (1, 0, 0, 0)], 'p'))
        pose = [float(v) for v in root.find('.//visual/pose').text.split()]
        tx, ty, _tz, yaw = pose[0], pose[1], pose[2], pose[5]

        def to_model(px, py):
            return (math.cos(yaw) * px - math.sin(yaw) * py + tx,
                    math.sin(yaw) * px + math.cos(yaw) * py + ty)
        cx, cy = measured[name]
        assert to_model(cx, cy) == pytest.approx((0.0, 0.0), abs=1e-8)
        bow = to_model(cx, cy - 0.5 * m.size[0])     # mesh -Y end
        assert bow == pytest.approx((0.5 * m.size[0], 0.0), abs=1e-8)
        assert m.size[0] > m.size[1]                  # length is along the bow


def test_sdf_carries_label_and_samples():
    root = ET.fromstring(obstacle_sdf('kayak', 'k1', 'kayak',
                                      [(1.0, 2.0, 3.0, 0.5), (2.0, 3.0, 3.0, 0.5)], '/lib/p.so'))
    plugins = {p.get('name'): p for p in root.iter('plugin')}
    assert plugins['my_boat_obstacles::MovingObstacle'].find('run_topic').text == RUN_TOPIC
    assert plugins['gz::sim::systems::Label'].find('label').text.strip() == str(class_id('kayak'))
    mo = plugins['my_boat_obstacles::MovingObstacle']
    assert mo.get('filename') == '/lib/p.so'
    vals = [float(v) for v in mo.find('samples').text.split()]
    assert vals == pytest.approx([1.0, 2.0, 3.0, 0.5, 2.0, 3.0, 3.0, 0.5])
    assert root.find('.//link/gravity').text == 'false'


def test_parse_gz_json_pose_line_handles_omitted_zero_fields():
    from my_boat_obstacles.spawner_node import parse_pose_line
    line = ('{"header":{"stamp":{"sec":"103","nsec":744000000}},"pose":['
            '{"name":"buoy","position":{"x":-528,"y":191},"orientation":{"w":1}},'
            '{"name":"my_boat","position":{"x":-528.37,"y":183.76,"z":0.4},'
            '"orientation":{"z":-0.5408079,"w":0.8411442}}]}')
    t, x, y, yaw = parse_pose_line(line, 'my_boat')
    assert t == pytest.approx(103.744)
    assert (x, y) == pytest.approx((-528.37, 183.76))
    assert yaw == pytest.approx(2 * math.atan2(-0.5408079, 0.8411442), abs=1e-5)
    # identity orientation arrives as {} (every field is zero except w... or absent)
    t, x, y, yaw = parse_pose_line('{"pose":[{"name":"b","orientation":{}}]}', 'b')
    assert (t, x, y, yaw) == (0.0, 0.0, 0.0, 0.0)
    assert parse_pose_line('{"pose":[]}', 'b') is None
    assert parse_pose_line('not json', 'b') is None


# ---------------------------------------------------------------- set_start
def test_with_start_moves_a_moving_obstacle_and_its_heading():
    ob = cv(-500.0, 150.0, 90.0, 1.0)
    new = ob.with_start(-470.0, 120.0, 0.0)
    assert new.id == ob.id and new.spec.name == ob.spec.name
    s = new.state(10.0)
    assert (s.x, s.y, s.speed) == pytest.approx((-460.0, 120.0, 1.0))
    assert ob.state(10.0).y == pytest.approx(160.0)          # original untouched


def test_with_start_on_static_sets_pose_and_yaw():
    ob = build(ObstacleSpec('b', model='buoy', start={'x': 0.0, 'y': 0.0}), 1)
    s = ob.with_start(5.0, 6.0, 45.0).state(99.0)
    assert (s.x, s.y, s.speed) == (5.0, 6.0, 0.0)
    assert s.heading == pytest.approx(math.radians(45.0))


def test_with_start_shifts_a_whole_waypoint_path():
    ob = build(ObstacleSpec('k', model='kayak', motion={
        'type': 'waypoints', 'speed': 1.0, 'loop': True,
        'points': [[0.0, 0.0], [10.0, 0.0], [10.0, 10.0]]}), 1)
    new = ob.with_start(100.0, 50.0, 123.0)                   # heading ignored
    assert new.motion.points[:3] == [(100.0, 50.0), (110.0, 50.0), (110.0, 60.0)]
    assert new.state(5.0).x == pytest.approx(105.0)


def test_with_start_keeps_samples_valid_for_the_plugin():
    new = cv(0.0, 0.0, 0.0, 1.0, start_delay=3.0).with_start(7.0, 8.0, 180.0)
    smp = new.samples(20.0)
    assert smp[0][1:3] == (7.0, 8.0)
    assert all(b[0] > a[0] for a, b in zip(smp, smp[1:]))
    root = ET.fromstring(obstacle_sdf('brazilian', 'n', 'boat', smp, 'p'))
    assert root.find('.//samples_topic') is not None


# ---------------------------------------------------------------- config file
def test_default_config_documents_every_option_it_uses_and_loads():
    sc = load_scenario(CONFIG)
    assert sc.settings == SETTINGS_DEFAULTS            # the file shows the defaults
    names = [o.name for o in sc.obstacles]
    assert names[:6] == ['centre_crosser', 'left_kayak', 'top_ship', 'right_buoy', 'essex', 'yacht']
    statics = set(names[6:])
    assert {'gate1_red', 'gate1_green', 'drum_1', 'log_1', 'pile_1', 'post_1'} <= statics
    built = {o.spec.name: o for o in sc.build()}
    assert all(built[n].spec.motion.get('type', 'static') == 'static' for n in statics)
    ship = {o.spec.name: o for o in sc.build()}['top_ship']
    assert ship.state(10.0).speed == pytest.approx(2.9 * KNOT)


def test_settings_are_validated_and_defaulted():
    sc = parse_scenario({'settings': {'run_lead_s': 1.5}, 'obstacles': []})
    assert sc.settings['run_lead_s'] == 1.5 and sc.settings['world'] == 'sydney_regatta'
    with pytest.raises(ValueError, match='unknown settings keys'):
        parse_scenario({'settings': {'wrold': 'x'}, 'obstacles': []})
    with pytest.raises(ValueError, match='sample_step_s must be > 0'):
        parse_scenario({'settings': {'sample_step_s': 0}, 'obstacles': []})
    # older files keep working: duration_s under scenario:
    assert parse_scenario({'scenario': {'duration_s': 900}}).duration_s == 900.0


def test_speed_in_knots_and_not_both():
    ob = parse_scenario({'obstacles': [{'name': 'a', 'start': {'x': 0, 'y': 0},
                                        'motion': {'type': 'constant_velocity',
                                                   'speed_kn': 1.0}}]}).build()[0]
    assert ob.state(1.0).speed == pytest.approx(0.514444, abs=1e-6)
    with pytest.raises(ValueError, match='not both'):
        parse_scenario({'obstacles': [{'name': 'a', 'start': {'x': 0, 'y': 0},
                                       'motion': {'type': 'constant_velocity',
                                                  'speed': 1, 'speed_kn': 2}}]})


def test_resized_primitive_keeps_its_submerged_depth():
    d = MODELS['barge']
    m = model_spec('barge', size=[12.0, 4.0, 5.0])
    bottom = d.centre_z - 0.5 * d.size[2]
    assert m.size == (12.0, 4.0, 5.0)
    assert m.centre_z - 0.5 * m.size[2] == pytest.approx(bottom)
    ob = build(ObstacleSpec('b', model='barge', size=[12.0, 4.0, 5.0], start={'x': 0, 'y': 0}), 1)
    s = ob.state(0.0)
    assert (s.l, s.w, s.h, s.z) == pytest.approx((12.0, 4.0, 5.0, m.centre_z))
    root = ET.fromstring(obstacle_sdf('barge', 'b', 'barge', ob.samples(10.0), 'p',
                                      size=[12.0, 4.0, 5.0], colour=[1.0, 0.0, 0.0]))
    assert root.find('.//visual//box/size').text == '12.0 4.0 5.0'
    assert root.find('.//collision//box/size').text == '12.0 4.0 5.0'
    assert root.find('.//visual//ambient').text == '1.0 0.0 0.0 1'


def test_size_and_colour_are_refused_on_meshes_and_checked():
    with pytest.raises(ValueError, match='is a mesh'):
        build(ObstacleSpec('a', model='brazilian', size=[5, 2, 1], start={'x': 0, 'y': 0}), 1)
    with pytest.raises(ValueError, match='cylinder'):
        model_spec('buoy', size=[1.0, 2.0, 1.0])
    with pytest.raises(ValueError, match='colour'):
        model_spec('kayak', colour=[2.0, 0.0, 0.0])
    with pytest.raises(ValueError, match='size'):
        model_spec('kayak', size=[1.0, 2.0])


def test_appearance_separates_respawn_from_new_path():
    a = ObstacleSpec('a', model='kayak', start={'x': 0, 'y': 0})
    moved = ObstacleSpec('a', model='kayak', start={'x': 5, 'y': 0},
                         motion={'type': 'constant_velocity', 'speed': 2})
    resized = ObstacleSpec('a', model='kayak', size=[5.0, 1.0, 0.6], start={'x': 0, 'y': 0})
    assert a.appearance() == moved.appearance()        # -> just a new path
    assert a.appearance() != resized.appearance()      # -> respawn


# ---------------------------------------------------------------- user models
from my_boat_obstacles import catalog, user_models  # noqa: E402

MODELS_DIR = os.path.join(HERE, '..', 'models')


def _apply(R, v):
    return tuple(sum(R[i][k] * v[k] for k in range(3)) for i in range(3))


def test_example_models_are_measured_scaled_and_rotated():
    found = user_models.scan([MODELS_DIR])
    assert set(found) >= {'shipping_container', 'channel_marker'}
    c = found['shipping_container']
    assert c.size == pytest.approx((6.06, 2.44, 2.59), abs=1e-6)
    assert c.centre_z == pytest.approx(-1.9 + 2.59 / 2)          # draft 1.9
    assert c.default_class == 'debris' and c.colour is None      # keeps its own .mtl look
    m = found['channel_marker']                                  # cm, Y-up STL
    assert m.size == pytest.approx((0.9, 0.9, 3.1), abs=1e-6)    # 310 cm tall after y-up
    assert m.centre_z == pytest.approx(-1.0 + 1.55)
    assert m.mesh_scale == (0.01, 0.01, 0.01)


def test_visual_pose_puts_the_bbox_centre_on_the_origin_bow_forward_up_up():
    for fwd, up in (('+x', '+z'), ('-y', '+z'), ('+z', '+y'), ('-x', '-z')):
        R = user_models.axes_rotation(fwd, up)
        roll, pitch, yaw = user_models.rotation_to_rpy(R)
        # rebuild R from rpy exactly as SDF does: Rz(yaw) Ry(pitch) Rx(roll)
        cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch),
                                  math.sin(pitch), math.cos(yaw), math.sin(yaw))
        R2 = [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
              [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
              [-sp, cp * sr, cp * cr]]
        for i in range(3):
            assert R2[i] == pytest.approx(R[i], abs=1e-12)
        assert _apply(R, user_models._AXES[fwd]) == pytest.approx((1, 0, 0))
        assert _apply(R, user_models._AXES[up]) == pytest.approx((0, 0, 1))
    with pytest.raises(ValueError, match='perpendicular'):
        user_models.axes_rotation('+x', '-x')


def test_marker_visual_pose_centres_the_scaled_rotated_mesh():
    m = user_models.scan([MODELS_DIR])['channel_marker']
    x, y, z, roll, pitch, yaw = m.visual_pose
    R = user_models.axes_rotation('+x', '+y')
    # raw STL bbox centre in cm: (0, 30, 0) -> scaled, rotated, then shifted
    c = _apply(R, (0.0, 0.30, 0.0))
    assert (c[0] + x, c[1] + y, c[2] + z) == pytest.approx((0.0, 0.0, 0.0), abs=1e-9)


def test_broken_model_folders_are_errors_that_name_the_folder(tmp_path):
    bad = tmp_path / 'wreck'
    bad.mkdir()
    (bad / 'model.yaml').write_text('mesh: missing.stl\n')
    with pytest.raises(ValueError, match="model 'wreck'.*not found"):
        user_models.scan([str(tmp_path)])
    (bad / 'model.yaml').write_text('mesh: w.fbx\n')
    (bad / 'w.fbx').write_text('x')
    with pytest.raises(ValueError, match='cannot be measured.*size'):
        user_models.scan([str(tmp_path)])
    (bad / 'model.yaml').write_text('mesh: w.fbx\nsize: [5, 2, 1]\nmeshh: x\n')
    with pytest.raises(ValueError, match='unknown keys'):
        user_models.scan([str(tmp_path)])
    (bad / 'model.yaml').write_text('mesh: w.fbx\nsize: [5, 2, 1]\ndraft: 0.3\n')
    assert user_models.scan([str(tmp_path)])['wreck'].size == (5.0, 2.0, 1.0)
    clash = tmp_path / 'kayak'
    clash.mkdir()
    (clash / 'model.yaml').write_text('mesh: x.stl\n')
    with pytest.raises(ValueError, match='built-in'):
        user_models.scan([str(tmp_path)])


def test_user_model_usable_in_a_config_and_sdf_is_valid():
    catalog.set_user_models(user_models.scan([MODELS_DIR]))
    try:
        ob = build(ObstacleSpec('c', model='shipping_container', start={'x': 1, 'y': 2}), 1)
        assert ob.spec.det_class == 'debris' and ob.state(0).l == pytest.approx(6.06)
        root = ET.fromstring(obstacle_sdf('shipping_container', 'c', 'debris',
                                          ob.samples(5.0), 'p'))
        uri = root.find('.//visual//mesh/uri').text
        assert os.path.isabs(uri) and uri.endswith('container.obj')
        assert root.find('.//visual/material') is None           # own .mtl kept
    finally:
        catalog.set_user_models({})


# ---------------------------------------------------------------- look + collision
def test_colour_works_on_builtin_meshes_and_texture_is_checked(tmp_path):
    root = ET.fromstring(obstacle_sdf('brazilian', 'b', 'boat', [(0, 0, 0, 0), (1, 0, 0, 0)], 'p',
                                      colour=[1.0, 0.0, 0.0]))
    assert root.find('.//visual//diffuse').text == '1.0 0.0 0.0 1'
    png = tmp_path / 'hull.png'
    png.write_bytes(b'\x89PNG')
    with pytest.raises(ValueError, match='STL mesh has no texture coordinates'):
        model_spec('brazilian', texture=str(png))
    root = ET.fromstring(obstacle_sdf('barge', 'b', 'barge', [(0, 0, 0, 0), (1, 0, 0, 0)], 'p',
                                      texture=str(png)))
    assert root.find('.//albedo_map').text == str(png)
    with pytest.raises(ValueError, match='not found'):
        model_spec('barge', texture=str(tmp_path / 'nope.png'))


def test_texture_path_in_config_is_relative_to_the_config_file(tmp_path):
    (tmp_path / 'tex').mkdir()
    (tmp_path / 'tex' / 'a.png').write_bytes(b'\x89PNG')
    cfg = tmp_path / 'c.yaml'
    cfg.write_text('obstacles:\n  - {name: b, model: barge, texture: tex/a.png, '
                   'start: {x: 0, y: 0}}\n')
    ob = load_scenario(str(cfg)).build()[0]
    assert ob.model.texture == str(tmp_path / 'tex' / 'a.png')


def test_collision_modes():
    def coll(name, mode):
        root = ET.fromstring(obstacle_sdf(name, 'x', 'boat', [(0, 0, 0, 0), (1, 0, 0, 0)], 'p',
                                          collision=mode))
        return root.find('.//collision')
    assert coll('brazilian', None).find('.//box') is not None          # default box
    assert coll('brazilian', 'mesh').find('.//mesh') is not None
    assert coll('brazilian', 'none') is None
    with pytest.raises(ValueError, match='primitive'):
        model_spec('buoy', collision='mesh')
    with pytest.raises(ValueError, match='collision must be'):
        model_spec('buoy', collision='soft')


def test_boxes_overlap_separating_axis():
    from my_boat_obstacles.spawner_node import boxes_overlap
    boat = (0.0, 0.0, 0.0, 4.0, 1.7)
    assert boxes_overlap(boat, (3.9, 0.0, 0.0, 4.0, 1.7))            # nose to tail, 0.1 overlap
    assert not boxes_overlap(boat, (4.1, 0.0, 0.0, 4.0, 1.7))
    assert not boxes_overlap(boat, (0.0, 1.8, 0.0, 4.0, 1.7))
    # a diagonal box whose bounding boxes overlap but the boxes do not
    assert not boxes_overlap(boat, (3.2, 1.9, math.radians(45), 2.0, 0.4))
    assert boxes_overlap(boat, (2.3, 0.0, math.radians(45), 2.0, 0.4))


def test_metallic_glb_is_warned_about_because_gazebo_renders_it_black(tmp_path):
    import trimesh
    for name, metal in (('shiny', None), ('matte', 0.0)):
        d = tmp_path / name
        d.mkdir()
        m = trimesh.creation.box((2.0, 1.0, 1.0))
        m.visual = trimesh.visual.TextureVisuals(material=trimesh.visual.material.PBRMaterial(
            baseColorFactor=[200, 50, 50, 255], metallicFactor=metal))
        m.export(str(d / 'm.glb'))
        (d / 'model.yaml').write_text('mesh: m.glb\n')
    found = user_models.scan([str(tmp_path)])
    assert any('BLACK' in w for w in user_models.look_warnings(found['shiny']))
    assert user_models.look_warnings(found['matte']) == []


# ---------------------------------------------------------------- on_contact
def test_stopped_obstacle_is_identical_before_and_frozen_after():
    from my_boat_obstacles.scenario import StoppedMotion
    ob = cv(0.0, 0.0, 0.0, 2.0, start_delay=3.0)
    st = ob.stopped_at(8.0)                       # 5 s of motion = 10 m
    assert isinstance(st.motion, StoppedMotion) and st.is_stopped and not ob.is_stopped
    for t in (0.0, 4.0, 7.9):
        assert (st.state(t).x, st.state(t).speed) == pytest.approx((ob.state(t).x, ob.state(t).speed))
    for t in (8.0, 20.0, 500.0):
        assert (st.state(t).x, st.state(t).speed) == pytest.approx((10.0, 0.0))
    smp = st.samples(30.0)
    assert all(b[0] > a[0] for a, b in zip(smp, smp[1:]))
    assert smp[-1][1] == pytest.approx(10.0)


def test_predicted_overlaps_finds_two_obstacles_driving_through_each_other():
    from my_boat_obstacles.scenario import predicted_overlaps
    a = build(ObstacleSpec('a', start={'x': 0.0, 'y': -20.0, 'heading_deg': 90.0},
                           motion={'type': 'constant_velocity', 'speed': 1.0}), 1)
    b = build(ObstacleSpec('b', start={'x': -20.0, 'y': 0.0, 'heading_deg': 0.0},
                           motion={'type': 'constant_velocity', 'speed': 1.0}), 2)
    ghost = build(ObstacleSpec('g', model='kayak', collision='none',
                               start={'x': 20.0, 'y': 0.0, 'heading_deg': 180.0},
                               motion={'type': 'constant_velocity', 'speed': 1.0}), 3)
    hits = predicted_overlaps([a, b, ghost], 60.0)
    assert [(x, y) for x, y, _ in hits] == [('a', 'b')]       # ghost is ignored
    assert 15.0 < hits[0][2] < 20.0                           # boxes meet before the centres
    far = build(ObstacleSpec('far', start={'x': 0.0, 'y': 50.0}), 4)
    assert predicted_overlaps([a, far], 30.0) == []


def test_on_contact_is_validated_and_defaults_to_stop():
    assert parse_scenario({'obstacles': []}).settings['on_contact'] == 'stop'
    with pytest.raises(ValueError, match='on_contact'):
        parse_scenario({'settings': {'on_contact': 'bounce'}, 'obstacles': []})
    with pytest.raises(ValueError, match='on_contact'):
        parse_scenario({'obstacles': [{'name': 'a', 'on_contact': 'bounce', 'start': {'x': 0, 'y': 0}}]})
    sc = parse_scenario({'obstacles': [{'name': 'a', 'on_contact': 'pass', 'start': {'x': 0, 'y': 0}}]})
    assert sc.obstacles[0].on_contact == 'pass'


def test_default_config_has_no_obstacles_driving_through_each_other():
    from my_boat_obstacles.scenario import predicted_overlaps
    assert predicted_overlaps(load_scenario(CONFIG).build(), 600.0) == []


def test_builtin_boats_default_to_their_model_sdf_colours():
    root = ET.fromstring(obstacle_sdf('brazilian', 'b', 'boat', [(0, 0, 0, 0), (1, 0, 0, 0)], 'p'))
    assert root.find('.//visual//diffuse').text == '0.75 0.78 0.72 1'


# ---------------------------------------------------------------- speed
def test_top_level_speed_and_knots_and_errors():
    sc = parse_scenario({'obstacles': [
        {'name': 'a', 'start': {'x': 0, 'y': 0}, 'speed': 1.5, 'motion': {'type': 'constant_velocity'}},
        {'name': 'b', 'start': {'x': 0, 'y': 9}, 'speed_kn': 2.0, 'motion': {'type': 'constant_turn',
                                                                            'turn_rate_deg_s': 5}}]})
    a, b = sc.build()
    assert a.state(10.0).x == pytest.approx(15.0)
    assert b.state(1.0).speed == pytest.approx(2.0 * KNOT)
    with pytest.raises(ValueError, match='twice'):
        parse_scenario({'obstacles': [{'name': 'a', 'start': {'x': 0, 'y': 0}, 'speed': 1,
                                       'motion': {'type': 'constant_velocity', 'speed': 2}}]})
    with pytest.raises(ValueError, match='static'):
        parse_scenario({'obstacles': [{'name': 'a', 'start': {'x': 0, 'y': 0}, 'speed': 1}]})
    with pytest.raises(ValueError, match='needs a speed'):
        parse_scenario({'obstacles': [{'name': 'a', 'start': {'x': 0, 'y': 0},
                                       'motion': {'type': 'constant_velocity'}}]})


def test_speed_scale_plays_the_same_path_faster():
    turn = {'name': 't', 'start': {'x': 0, 'y': 0}, 'speed': 1.0,
            'motion': {'type': 'constant_turn', 'turn_rate_deg_s': 6.0}, 'start_delay': 2.0}
    slow = parse_scenario({'obstacles': [turn]}).build()[0]
    fast = parse_scenario({'settings': {'speed_scale': 2.0}, 'obstacles': [turn]}).build()[0]
    # same point on the circle, reached in half the motion time; delay not scaled
    s1, s2 = slow.state(2.0 + 20.0), fast.state(2.0 + 10.0)
    assert (s2.x, s2.y) == pytest.approx((s1.x, s1.y), abs=1e-9)
    assert s2.speed == pytest.approx(2.0 * s1.speed)
    assert fast.state(1.9).x == 0.0                        # still waiting out start_delay
    assert slow.with_speed_scale(2.0).state(12.0).x == pytest.approx(s2.x)
    # a stopped obstacle stays where it stopped whatever the scale
    st = fast.stopped_at(8.0)
    assert st.with_speed_scale(0.5).state(50.0).x == pytest.approx(st.state(50.0).x)
    with pytest.raises(ValueError, match='speed_scale'):
        parse_scenario({'settings': {'speed_scale': 0}, 'obstacles': []})


def test_length_sets_the_scale_and_matte_rewrites_metalness(tmp_path):
    import json
    import trimesh
    d = tmp_path / 'm'
    d.mkdir()
    box = trimesh.creation.box((0.5, 0.1, 0.2))              # unknown units, 0.5 long in x
    box.visual = trimesh.visual.TextureVisuals(material=trimesh.visual.material.PBRMaterial(
        baseColorFactor=[200, 50, 50, 255], metallicFactor=1.0))
    box.export(str(d / 'm.glb'))
    (d / 'model.yaml').write_text('mesh: m.glb\nlength: 20.0\nmatte: true\n')
    m = user_models.scan([str(tmp_path)])['m']
    assert m.size == pytest.approx((20.0, 4.0, 8.0))
    assert m.mesh.endswith('m.matte.glb') and user_models.look_warnings(m) == []
    (d / 'model.yaml').write_text('mesh: m.glb\nlength: 20.0\nscale: 2\n')
    with pytest.raises(ValueError, match='OR length'):
        user_models.scan([str(tmp_path)])


def test_the_two_pasted_models_load():
    found = user_models.scan([MODELS_DIR])
    e, y = found['jarethcruz38819hs-essex'], found['frickies_yacht']
    assert e.size[0] == pytest.approx(12.0) and y.size[0] == pytest.approx(30.0)
    assert e.size[0] > e.size[1] and y.size[0] > y.size[1]   # length is along the bow
    assert y.mesh.endswith('scene.matte.gltf')
    assert user_models.look_warnings(y) == []


# ---------------------------------------------------------------- land
def test_land_mask_knows_the_sydney_regatta_shore():
    from my_boat_obstacles import landmask
    m = landmask.LandMask(os.path.join(HERE, '..', 'worlds', 'sydney_regatta_land.npz'))
    assert m.is_land(-528.0, 186.0) is False          # my_boat's spawn is water
    assert m.is_land(-528.0, 140.0) is True           # south shore
    assert m.is_land(-5000.0, 0.0) is None            # off the map


def test_land_check_flags_a_start_ashore_and_a_path_running_ashore():
    from my_boat_obstacles import landmask
    m = landmask.LandMask(os.path.join(HERE, '..', 'worlds', 'sydney_regatta_land.npz'))
    on_land = build(ObstacleSpec('beached', start={'x': -528.0, 'y': 140.0}), 1)
    runs = build(ObstacleSpec('runner', start={'x': -528.0, 'y': 186.0, 'heading_deg': -90.0},
                              motion={'type': 'constant_velocity', 'speed': 2.0}), 2)
    ok = build(ObstacleSpec('ok', start={'x': -500.0, 'y': 200.0}), 3)
    hits = {n: (k, t) for n, k, t, _, _ in landmask.land_problems([on_land, runs, ok], m, 120.0)}
    assert hits['beached'][0] == 'start'
    assert hits['runner'][0] == 'path' and 10.0 < hits['runner'][1] < 20.0   # shore ~30 m south
    assert 'ok' not in hits


def test_default_config_stays_in_the_water():
    from my_boat_obstacles import landmask
    m = landmask.LandMask(os.path.join(HERE, '..', 'worlds', 'sydney_regatta_land.npz'))
    assert landmask.land_problems(load_scenario(CONFIG).build(), m, 600.0) == []


def test_speed_change_mid_run_is_continuous_not_a_jump():
    ob = cv(0.0, 0.0, 0.0, 1.0, start_delay=2.0)
    at = ob.state(12.0).x                                   # 10 s of motion -> x = 10
    fast = ob.with_speed_scale(2.0, tau_now=12.0)
    assert fast.state(12.0).x == pytest.approx(at)          # same place at the switch
    assert fast.state(15.0).x == pytest.approx(at + 6.0)    # then 2 m/s
    assert fast.state(15.0).speed == pytest.approx(2.0)
    slow = fast.with_speed_scale(0.5, tau_now=15.0)
    assert slow.state(15.0).x == pytest.approx(at + 6.0)
    assert slow.state(19.0).x == pytest.approx(at + 8.0)
    # before run (no tau_now) nothing is offset
    assert ob.with_speed_scale(2.0).state(7.0).x == pytest.approx(10.0)


# ---------------------------------------------------------------- UI: save / map
def test_ui_dump_round_trips_to_the_same_scenario():
    from my_boat_obstacles.ui_node import dump_config
    doc = yaml.safe_load(open(CONFIG))
    again = parse_scenario(yaml.safe_load(dump_config(doc)))
    orig = load_scenario(CONFIG)
    assert [o.name for o in again.obstacles] == [o.name for o in orig.obstacles]
    a, b = again.build(), orig.build()
    for x, y in zip(a, b):
        for t in (0.0, 13.0, 77.0):
            sa, sb = x.state(t), y.state(t)
            assert (sa.x, sa.y, sa.speed) == pytest.approx((sb.x, sb.y, sb.speed))


def test_ui_save_keeps_the_header_and_trailing_comments():
    from my_boat_obstacles.ui_node import merge_into_file
    original = open(CONFIG).read()
    doc = yaml.safe_load(original)
    doc['obstacles'][0]['start'] = {'x': -480.0, 'y': 170.0, 'heading_deg': 45.0}
    doc['obstacles'][0]['speed'] = 2.5
    text = merge_into_file(original, doc['obstacles'])
    head = original.split('\nobstacles:')[0]
    assert text.startswith(head.rstrip('\n'))                 # documented header untouched
    assert '# --- your own models: uncomment' in text          # commented examples kept
    sc = parse_scenario(yaml.safe_load(text))
    assert sc.settings == load_scenario(CONFIG).settings
    first = sc.build()[0]
    assert first.state(0.0).x == pytest.approx(-480.0) and first.state(1.0).speed == pytest.approx(2.5)


def test_ui_map_png_is_north_up_and_placed_right():
    import io as _io
    from PIL import Image
    from my_boat_obstacles.ui_node import map_png
    png, info = map_png(os.path.join(HERE, '..', 'worlds', 'sydney_regatta_land.npz'))
    img = Image.open(_io.BytesIO(png)).convert('RGB')
    assert img.size == (info['w'], info['h'])
    def pix(x, y):   # world -> image pixel (row 0 = north edge)
        return img.getpixel((int((x - info['x0']) / info['res']),
                             info['h'] - 1 - int((y - info['y0']) / info['res'])))
    assert pix(-528.0, 186.0) == (168, 205, 232)               # my_boat's spawn is water
    assert pix(-528.0, 140.0) == (226, 214, 186)               # south shore is land
    # walking south from the spawn, the first land pixel is the dark shoreline edge
    y = 186.0
    while pix(-528.0, y) == (168, 205, 232):
        y -= 1.0
    assert pix(-528.0, y) == (150, 132, 98)
    assert info['lakes'][0]['name'] == 'north lake'


def test_ui_yaml_is_written_the_way_a_person_would():
    from my_boat_obstacles.ui_node import _flow, dump_obstacles
    assert _flow('centre_crosser') == 'centre_crosser'
    assert _flow('yes') == '"yes"' and _flow('two words') == '"two words"'   # would change meaning
    assert _flow({'x': -472.87, 'y': 165.0}) == '{x: -472.87, y: 165}'
    assert _flow([0.85, 0.1, 0.1]) == '[0.85, 0.1, 0.1]' and _flow(True) == 'true'
    text = dump_obstacles([{'name': 'a', 'model': 'buoy', 'start': {'x': 1.5, 'y': 2}}])
    assert '  - name: a\n    model: buoy\n    start: {x: 1.5, y: 2}' in text
    assert yaml.safe_load(text)['obstacles'][0] == {'name': 'a', 'model': 'buoy', 'start': {'x': 1.5, 'y': 2}}


def test_dae_is_measured_and_fuel_meshes_resolve(tmp_path):
    try:
        post = user_models.resolve_fuel('fuel:openrobotics/post/meshes/post.dae')
    except ValueError:
        pytest.skip('VRX post not in this machine\'s Fuel cache')
    d = tmp_path / 'p'
    d.mkdir()
    (d / 'model.yaml').write_text('mesh: fuel:openrobotics/post/meshes/post.dae\ndraft: 0.2\n')
    m = user_models.scan([str(tmp_path)])['p']
    assert m.mesh == post
    assert m.size == pytest.approx((0.646, 0.646, 1.097), abs=0.002)   # cm-unit Collada, measured
    with pytest.raises(ValueError, match='not in the Fuel cache'):
        user_models.resolve_fuel('fuel:openrobotics/no_such_model/meshes/x.dae')


def test_first_static_field_is_in_the_config_with_its_original_geometry():
    """The five obstacles of my_boat_gazebo's original obstacles.yaml: same
    place, and shapes identical to its obstacle_*.sdf files."""
    import re as _re
    src = os.path.join(HERE, '..', '..', 'my_boat_gazebo')
    if not os.path.isdir(src):
        pytest.skip('my_boat_gazebo not next to this package')
    orig = yaml.safe_load(open(os.path.join(src, 'config', 'obstacles.yaml')))['obstacles']
    built = {o.spec.name: o for o in load_scenario(CONFIG).build()}
    for o in orig:
        b = built[o['name']].state(0.0)
        assert (b.x, b.y) == pytest.approx((o['x'], o['y']))
        assert b.heading == pytest.approx(math.radians(o.get('yaw', 0.0)))
        sdf = open(os.path.join(src, 'models', 'obstacles', f"obstacle_{o['type']}.sdf")).read()
        z = float(_re.search(r'<pose>0 0 ([\d.]+) 0 0 0</pose>', sdf).group(1))
        cyl = _re.search(r'<radius>([\d.]+)</radius><length>([\d.]+)</length>', sdf)
        size = ((2 * float(cyl.group(1)),) * 2 + (float(cyl.group(2)),) if cyl else
                tuple(float(v) for v in _re.search(r'<size>([^<]+)</size>', sdf).group(1).split()))
        assert (b.l, b.w, b.h) == pytest.approx(size)
        assert b.z == pytest.approx(z)                   # same height against the waterline
