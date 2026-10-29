import math

import pytest

from my_boat_obstacles.track_view import MapAlignment


def test_alignment_recovers_the_map_to_world_transform():
    """map = world shifted to the first GPS fix (-528, 186) and rotated 1.3 deg
    (UTM grid convergence): the fit must undo exactly that."""
    th, T = math.radians(1.3), (-528.0, 186.0)
    al = MapAlignment()
    for k in range(50):
        t = k * 0.1
        wx, wy, wyaw = -528 + 2.0 * t, 186 + 0.5 * t, 0.3 + 0.05 * t          # true, Gazebo world
        c, s = math.cos(-th), math.sin(-th)
        dx, dy = wx - T[0], wy - T[1]
        mx, my = c * dx - s * dy, s * dx + c * dy                                # the EKF's map pose
        al.add_true(t, wx, wy, wyaw)
        assert al.add_ekf(t, mx, my, wyaw - th)
    assert math.degrees(al.theta) == pytest.approx(1.3, abs=1e-6)
    assert (al.tx, al.ty) == pytest.approx(T, abs=1e-6)
    assert al.error == pytest.approx(0.0, abs=1e-6)
    doc = {'stamp': 4.9, 'frame': 'map', 'tracks': [{'id': 1, 'x': 0.0, 'y': 0.0, 'vx': 1.0, 'vy': 0.0, 'yaw': 0.0,
            'course': 0.0, 'ell': [1, 0.5, 0.0], 'pred': [[10.0, 0.0, 2.0]], 'trail': [[0, 0]]}]}
    w = al.tracks_in_world(doc)['tracks'][0]
    assert (w['x'], w['y']) == pytest.approx(T, abs=0.01)
    assert w['course'] == pytest.approx(th, abs=1e-3)


def test_no_pair_without_a_true_pose_near_the_stamp():
    al = MapAlignment()
    al.add_true(10.0, 0, 0, 0)
    assert not al.add_ekf(11.0, 0, 0, 0)
    assert al.add_ekf(10.1, 0, 0, 0)
    al.add_true(1.0, 0, 0, 0)                  # sim reset: history cleared
    assert not al.ready
