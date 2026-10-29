#!/usr/bin/env python3
"""No-sim check of a scenario against its map: does every scripted craft stay afloat, and do any
two run into each other?

    tools/check_scenario.py scenarios/alappuzha.yaml maps/alappuzha.geojson [--duration s] [--step s]

Afloat = every hull point (centre + 4 corners) is world water (worlds/<world>_land.npz) and not
the map's terrain (land regions and canal banks), at every step. Contact = two hull boxes overlap
(collision: none models are ghosts and skipped). Scripted craft never give way, so either one is a
mistake in the plan. Exit code 1 on any failure.
"""
import argparse
import os
import sys

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, PKG)

from my_boat_obstacles import catalog, user_models  # noqa: E402
from my_boat_obstacles.landmask import LandMask, hull_points  # noqa: E402
from my_boat_obstacles.scenario import load_scenario, predicted_overlaps  # noqa: E402


def check(scenario_path, map_path, duration=None, step=0.25):
    """-> (dry, contacts): [(name, t, x, y)] first time each craft is not afloat,
    [(name_a, name_b, t)] first overlap of each pair."""
    from marine_vector_map import vmap
    from marine_vector_map.physical import build_terrain, parse_environment
    sc = load_scenario(scenario_path)
    catalog.set_user_models(user_models.scan(
        user_models.default_dirs(scenario_path, PKG, sc.settings['model_paths'])))
    obs = sc.build()
    duration = sc.duration_s if duration is None else float(duration)
    world = LandMask(os.path.join(PKG, 'worlds', f"{sc.settings['world']}_land.npz"))
    terrain = build_terrain(parse_environment(vmap.to_environment(vmap.load(map_path))), world.land_at)

    def afloat(x, y):
        return world.is_land(x, y) is False and not (terrain and terrain.is_solid(x, y))
    dry = []
    n = int(round(duration / step))
    for ob in obs:
        moves = ob.spec.motion.get('type', 'static') != 'static'
        for i in range(n + 1 if moves else 1):
            s = ob.state(i * step)
            hit = next(((x, y) for x, y in hull_points(s) if not afloat(x, y)), None)
            if hit:
                dry.append((ob.spec.name, i * step, hit[0], hit[1]))
                break
    return dry, predicted_overlaps(obs, duration, step)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('scenario')
    ap.add_argument('map')
    ap.add_argument('--duration', type=float, default=None, help='s (default: the scenario duration_s)')
    ap.add_argument('--step', type=float, default=0.25, help='s')
    a = ap.parse_args()
    dry, contacts = check(a.scenario, a.map, a.duration, a.step)
    for name, t, x, y in dry:
        print(f'OUT OF WATER  {name} at t={t:.2f} s, hull point ({x:.1f}, {y:.1f})')
    for na, nb, t in contacts:
        print(f'CONTACT       {na} x {nb} at t={t:.2f} s')
    print('clean' if not (dry or contacts) else f'{len(dry)} out of water, {len(contacts)} contacts')
    return 1 if dry or contacts else 0


if __name__ == '__main__':
    sys.exit(main())
