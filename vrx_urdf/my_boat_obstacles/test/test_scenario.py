"""Scenario realism (DISCUSSION_18 P1): waypoint dwells, the Alappuzha kadathu shuttle and drifters."""
import importlib.util
import math
import os

import pytest
import yaml

from my_boat_obstacles import catalog, user_models
from my_boat_obstacles.landmask import LandMask, hull_points
from my_boat_obstacles.scenario import WaypointMotion, build, load_scenario, parse_scenario

PKG = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')
ALP = os.path.join(PKG, 'scenarios', 'alappuzha.yaml')
ALP_MAP = os.path.join(PKG, 'maps', 'alappuzha.geojson')


def _canal():
    """(centreline y, x0, x1, width, current_mps) of the generated chart's canal (runs east-west)."""
    from marine_vector_map import vmap
    f = next(f for f in vmap.to_local(vmap.load(ALP_MAP))['features'] if f['properties'].get('class') == 'CANALS')
    (x0, y0), (x1, y1) = f['geometry']['coordinates'][0], f['geometry']['coordinates'][-1]
    assert y0 == y1, 'these tests assume the east-west canal'
    p = f['properties']
    return y0, min(x0, x1), max(x0, x1), p['width'], p['current_mps']


def _alappuzha():
    catalog.set_user_models(user_models.scan([os.path.join(PKG, 'models')]))
    sc = load_scenario(ALP)
    return sc, {o.spec.name: o for o in sc.build()}


def test_waypoint_dwell_holds_then_resumes():
    m = WaypointMotion([(0, 0), (10, 0), (10, 10)], 1.0, stops=[(1, 5.0)])
    assert m.at(5.0)[:2] == pytest.approx((5.0, 0.0))
    for t in (10.0, 12.0, 14.99):                           # tied up at point 1 ...
        x, y, course, v = m.at(t)
        assert (x, y, v) == pytest.approx((10.0, 0.0, 0.0)) and course == pytest.approx(0.0)  # arrival leg
    x, y, course, v = m.at(17.0)                             # ... then on again
    assert (x, y, v) == pytest.approx((10.0, 2.0, 1.0)) and course == pytest.approx(math.pi / 2)
    assert m.at(40.0) == pytest.approx((10.0, 10.0, math.pi / 2, 0.0))
    # no stops: exactly the old timing
    a, b = WaypointMotion([(0, 0), (10, 0), (10, 10)], 1.0), WaypointMotion([(0, 0), (10, 0), (10, 10)], 1.0, stops=[])
    assert all(a.at(t) == b.at(t) for t in (0.0, 3.3, 10.0, 25.0))
    # a loop dwells every lap; index 0 = the lap's end, held on the closing leg's course
    lp = WaypointMotion([(0, 0), (10, 0)], 1.0, loop=True, stops=[(0, 3.0)])   # lap 20 s + 3 s
    for t in (21.0, 44.0):
        assert lp.at(t) == pytest.approx((0.0, 0.0, math.pi, 0.0))
    assert lp.at(24.0)[:2] == pytest.approx((1.0, 0.0)) and lp.at(24.0)[3] == 1.0
    # parsed from a scenario, and played in the samples the Gazebo plugin gets
    ob = parse_scenario({'obstacles': [{'name': 'c', 'model': 'kayak', 'motion': {
        'type': 'waypoints', 'speed': 1.0, 'points': [[0, 0], [10, 0], [10, 10]], 'stops': [[1, 5]]}}]}).build()[0]
    held = [(x, y) for tau, x, y, _h in ob.samples(30.0, 0.5) if 10.0 <= tau <= 15.0]
    assert len(held) == 11 and set(held) == {(10.0, 0.0)}
    # the UI moves points[0] into `start`: the stop must not move with it
    ui = parse_scenario({'obstacles': [{'name': 'c', 'model': 'kayak', 'start': {'x': 0, 'y': 0}, 'motion': {
        'type': 'waypoints', 'speed': 1.0, 'points': [[10, 0], [10, 10]], 'stops': [[1, 5]]}}]}).build()[0]
    assert all(ui.state(t) == ob.state(t) for t in (0.0, 5.0, 10.0, 12.0, 15.5, 17.0, 40.0))
    with pytest.raises(ValueError):
        WaypointMotion([(0, 0), (10, 0)], 1.0, stops=[(2, 5.0)])


def test_canoe_final_course_parallel_to_bank():
    cy, _x0, _x1, width, _cur = _canal()
    banks = (cy - width / 2, cy + width / 2)
    sc, obs = _alappuzha()
    for name in ('c1_canoe', 'c2_canoe'):
        ob, seen = obs[name], set()
        t = ob.spec.start_delay + 0.25
        while t < sc.duration_s:
            s = ob.state(t)
            if s.speed == 0.0:
                assert abs(math.sin(s.heading)) < 1e-9, f'{name} held at {math.degrees(s.heading):.1f} deg, t={t}'
                bank = min(banks, key=lambda b: abs(s.y - b))
                assert abs(s.y - bank) - s.w / 2 == pytest.approx(1.0, abs=0.05), f'{name} not 1 m off at t={t}'
                seen.add(bank)
            t += 0.5
        assert seen == set(banks), f'{name} must tie up at both banks'


def test_hyacinth_speed_equals_chart_current():
    cy, x0, x1, width, current = _canal()
    data = yaml.safe_load(open(ALP))['obstacles']
    patches = [o for o in data if o['model'] == 'hyacinth_patch']
    assert any(x0 <= o['motion']['points'][0][0] <= x1 and abs(o['motion']['points'][0][1] - cy) < width / 2
               for o in patches), 'no hyacinth in the canal'
    for o in patches:                                        # every patch, lake ones too
        assert o['motion']['speed'] == current, o['name']


def test_drifters_reach_map_edge():
    """Every drift path ends where the water does: afloat there, the shore one patch-length on."""
    sc, obs = _alappuzha()
    world = LandMask(os.path.join(PKG, 'worlds', 'sydney_regatta_land.npz'))
    drifters = [o for o in obs.values() if o.spec.model == 'hyacinth_patch']
    assert len(drifters) >= 4
    for ob in drifters:
        end = ob.state(1e7)
        assert end.speed == 0.0
        assert not any(world.is_land(x, y) is not False for x, y in hull_points(end)), ob.spec.name
        d = end.l / 2 + 3.0
        assert world.is_land(end.x + d * math.cos(end.heading), end.y + d * math.sin(end.heading)) is not False, \
            f'{ob.spec.name} stops in open water at ({end.x:.0f}, {end.y:.0f})'


def test_alappuzha_scenario_is_clean():
    spec = importlib.util.spec_from_file_location('check_scenario', os.path.join(PKG, 'tools', 'check_scenario.py'))
    cs = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(cs)
    dry, contacts = cs.check(ALP, ALP_MAP)
    assert not dry and not contacts, (dry, contacts)


def test_scripted_craft_turn_on_an_arc():
    """bag planner_route44: f1_ferry (12 m) swung 90 deg in 0.2 s at a waypoint corner. A corner is an arc
    now: yaw rate <= v / (1.5 L), legs too short for the radius sailed slower (stops and loops: above)."""
    ob = parse_scenario({'obstacles': [{'name': 'f', 'model': 'jarethcruz38819hs-essex', 'motion': {
        'type': 'waypoints', 'speed': 2.5, 'points': [[-478, 224], [-478, 205], [-515, 205], [-575, 290]]}}]})
    catalog.set_user_models(user_models.scan([os.path.join(PKG, 'models')]))
    ob = ob.build()[0]
    r_max = 2.5 / (1.5 * ob.size[0])
    dt, worst, prev = 0.05, 0.0, ob.state(0.0)
    for i in range(1, int(60.0 / dt)):
        s = ob.state(i * dt)
        worst = max(worst, abs(math.atan2(math.sin(s.course - prev.course), math.cos(s.course - prev.course))) / dt)
        assert math.hypot(s.x - prev.x, s.y - prev.y) <= 2.5 * dt + 1e-9      # never faster than its speed
        prev = s
    assert worst <= r_max + 1e-6, f'yaw rate {worst:.3f} rad/s > v/R {r_max:.3f}'
    assert ob.state(1e6).x == pytest.approx(-575.0) and ob.state(1e6).y == pytest.approx(290.0)


def test_stopped_obstacle_resumes_from_where_it_stopped():
    obs = parse_scenario({'obstacles': [{'name': 'f', 'model': 'brazilian', 'motion': {
        'type': 'waypoints', 'speed': 2.0, 'points': [[100, 0]]}, 'start': {'x': 0, 'y': 0}}]})
    ob = build(obs.obstacles[0], 1)
    stopped = ob.stopped_at(10.0)
    assert stopped.is_stopped and stopped.state(30.0).speed == 0.0
    go = stopped.resumed_at(30.0)            # waited 20 s
    assert not go.is_stopped
    assert math.isclose(go.state(30.0).x, stopped.state(30.0).x, abs_tol=1e-6)   # no jump
    assert go.state(31.0).x > go.state(30.0).x + 1.5                             # and on again
