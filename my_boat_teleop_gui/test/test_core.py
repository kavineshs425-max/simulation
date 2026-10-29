import math

import pytest

from my_boat_teleop_gui.core import DEFAULTS, TeleopCore


def core(**kw):
    c = TeleopCore()
    for k, v in kw.items():
        c.set_param(k, v)
    return c


def hold(c, keys, t0, t1, dt=0.01):
    t = t0
    while t <= t1 + 1e-9:
        c.update(keys, t)
        t += dt
    return t


def test_a_tap_is_one_unit_in_actuator_mode():
    c = core()
    c.update({'w'}, 0.0); c.update(set(), 0.05)
    assert c.t3 == pytest.approx(1.0)
    c.update({'s'}, 0.1); c.update(set(), 0.15)
    assert c.steer == pytest.approx(math.radians(1.0))
    c.update({'shift', 'a'}, 0.2); c.update(set(), 0.25)
    assert c.t1 == pytest.approx(-1.0)                     # Shift = negative


def test_holding_repeats_after_the_delay_and_accelerates():
    c = core()
    c.update({'w'}, 0.0)
    hold(c, {'w'}, 0.0, 0.24)
    assert c.t3 == pytest.approx(1.0)                      # still inside repeat_delay_ms
    hold(c, {'w'}, 0.25, 1.25)                             # 1 s of repeats at 25 Hz
    n = 26                                                 # repeats done in [0.25, 1.25]
    expected = 1.0 + sum(min(30.0, 1.0 + i * 0.6) for i in range(1, n + 1))
    assert c.t3 == pytest.approx(expected, rel=0.05)
    hold(c, {'w'}, 1.26, 5.0)
    assert c.t3 == pytest.approx(554.1)                    # clamped to the bollard limit


def test_zero_detent_parks_at_zero_until_the_key_is_released():
    c = core()
    c.t3 = 5.0
    c.update({'shift', 'w'}, 0.0)
    hold(c, {'shift', 'w'}, 0.0, 3.0)                      # hold reverse for 3 s
    assert c.t3 == 0.0                                     # parked, not negative
    c.update(set(), 3.1)
    c.update({'shift', 'w'}, 3.2)                          # press again -> goes negative
    assert c.t3 == pytest.approx(-1.0)


def test_direction_mode_maps_steering_onto_both_bow_thrusters_same_sign():
    c = core(control_mode='direction')
    c.steer = c.params['steer_max']
    c._sync_bow()
    assert c.t1 == pytest.approx(39.2266) and c.t2 == pytest.approx(39.2266)   # common range
    c.steer = -c.params['steer_max'] / 2
    c._sync_bow()
    assert c.t1 == pytest.approx(-39.2266 / 2) and c.t2 == c.t1
    c2 = core(control_mode='direction')
    c2.update({'d'}, 0.0)
    assert c2.steer == pytest.approx(math.radians(1.0)) and c2.t1 > 0 and c2.t2 == c2.t1
    c2.update({'d', 'w'}, 0.05)                            # two keys at once: turn + forward
    assert c2.t3 == pytest.approx(1.0)


def test_shift_is_ignored_in_direction_mode():
    c = core(control_mode='direction')
    c.update({'shift', 'w'}, 0.0)
    assert c.t3 == pytest.approx(1.0)


def test_space_is_all_stop_and_held_keys_then_do_nothing():
    c = core()
    c.t1, c.t2, c.t3, c.steer = 10.0, -5.0, 300.0, 0.2
    c.update({'space', 'w'}, 0.0)
    assert c.commands() == {'T1': 0.0, 'T2': 0.0, 'T3': 0.0, 'steer': 0.0}
    hold(c, {'w'}, 0.01, 2.0)                              # w still held from before
    assert c.t3 == 0.0


def test_limits_are_asymmetric_and_a_new_limit_clamps_now():
    c = core()
    c.set_param('thrust_step', 1000.0)
    c.update({'shift', 'd'}, 0.0)
    assert c.t2 == pytest.approx(-78.4532)
    c.update(set(), 0.1); c.update({'d'}, 0.2); c.update(set(), 0.3); c.update({'d'}, 0.4)
    assert c.t2 == pytest.approx(39.2266)
    c.set_param('t2_max_thrust', 20.0)
    assert c.t2 == pytest.approx(20.0)


def test_bad_parameters_are_refused():
    c = core()
    for k, v in (('control_mode', 'rocket'), ('thrust_step', 0), ('accel_max', 0.5), ('repeat_hz', -1)):
        with pytest.raises(ValueError):
            c.set_param(k, v)
    with pytest.raises(KeyError):
        c.set_param('nope', 1)
    assert c.params == DEFAULTS


def test_a_stalled_gui_does_not_burst_many_repeats():
    c = core()
    c.update({'w'}, 0.0)
    c.update({'w'}, 10.0)                                  # 10 s gap in one call
    assert c.t3 <= 1.0 + 5 * 30.0


# ---------------------------------------------------------------- joystick (return to zero)
def test_direction_mode_hold_ramps_to_max_and_release_returns_to_zero():
    c = core(control_mode='direction')
    c.update({'w'}, 0.0)
    hold(c, {'w'}, 0.0, 5.0)
    assert c.t3 == pytest.approx(554.1)                    # long press reaches the limit
    c.update(set(), 5.01)
    assert c.t3 == 0.0                                     # released -> 0
    c.update({'s'}, 6.0)
    hold(c, {'s'}, 6.0, 12.0)
    assert c.t3 == pytest.approx(-274.6)                   # reverse limit
    c.update(set(), 12.01)
    assert c.t3 == 0.0


def test_turning_springs_back_with_the_bow_thrusters():
    c = core(control_mode='direction')
    c.update({'d'}, 0.0)
    hold(c, {'d'}, 0.0, 3.0)
    assert c.steer == pytest.approx(c.params['steer_max']) and c.t1 == pytest.approx(39.2266)
    c.update({'w'}, 3.01)                                  # d released, w pressed
    assert (c.steer, c.t1, c.t2) == (0.0, 0.0, 0.0) and c.t3 == pytest.approx(1.0)


def test_both_keys_of_an_axis_cancel_and_the_survivor_restarts_from_zero():
    c = core(control_mode='direction')
    c.update({'w'}, 0.0)
    hold(c, {'w'}, 0.0, 1.0)
    assert c.t3 > 10
    c.update({'w', 's'}, 1.01)
    hold(c, {'w', 's'}, 1.02, 2.0)
    assert c.t3 == 0.0                                     # both held = 0, no ramp
    c.update({'w'}, 2.01)                                  # s released
    assert c.t3 == pytest.approx(1.0)                      # w ramps again from 0


def test_return_to_zero_off_keeps_the_old_throttle_behaviour_and_actuator_always_latches():
    c = core(control_mode='direction', return_to_zero=False)
    c.update({'w'}, 0.0); hold(c, {'w'}, 0.0, 1.0); v = c.t3
    c.update(set(), 1.01)
    assert c.t3 == v and v > 0                             # kept after release
    a = core(control_mode='actuator')
    a.update({'w'}, 0.0); hold(a, {'w'}, 0.0, 1.0); v = a.t3
    a.update(set(), 1.01)
    assert a.t3 == v                                       # actuator mode unchanged
    with pytest.raises(ValueError):
        a.set_param('return_to_zero', 'yes')
