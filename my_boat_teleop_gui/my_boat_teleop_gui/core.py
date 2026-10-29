"""Teleop logic -- ROS-free and GUI-free, so it is unit-testable.

A line-for-line port of my_boat_teleop/src/keyboard_teleop.cpp (same modes,
same keys, same limits, same zero detent, same direction-mode bow mapping),
with ONE deliberate change in how holding a key works:

  The terminal teleop only ever sees characters, so "holding" means "the OS
  auto-repeats the character", and it guesses a hold from the gap between
  repeats (hold_reset_ms). Over ssh/VNC auto-repeat is often slow or off and
  holding then does nothing. This GUI reads the real key-DOWN / key-UP state,
  so it runs its own repeat: a press is one tap; after repeat_delay_ms of
  holding it repeats at repeat_hz, each repeat a bigger step
  (1 + n * accel_gain, capped at accel_max). It works the same everywhere,
  and several keys can be held at once (forward + turn).

ZERO DETENT (as in the C++): a step that would cross zero stops AT zero and
resets the acceleration. Here it also LATCHES until the key is released, so
holding the opposite key really parks the value at 0 -- press again to go
through to the other sign.

RETURN TO ZERO (direction mode, `return_to_zero`, default ON) -- a joystick,
not a throttle lever: holding a key ramps its axis up (to the limit),
RELEASING it puts the axis back to 0. The axes are surge (w / s -> T3) and
turn (a / d -> steering, with T1/T2 following). Both keys of one axis held at
once = that axis at 0. With it OFF, direction mode keeps the value after
release, as my_boat_teleop does. Actuator mode always keeps its values: it
sets individual thrusters, like trim.
"""

import math
from dataclasses import dataclass, field
from typing import Dict, Iterable

DEFAULTS = {
    'boat_namespace': 'my_boat',
    'thrust_step': 1.0,          # N per tap
    'steer_step_deg': 1.0,       # deg per tap
    'accel_gain': 0.6,           # step growth per repeat
    'accel_max': 30.0,           # max step multiplier
    'repeat_delay_ms': 250.0,    # hold this long before repeating
    'repeat_hz': 25.0,           # repeats per second while held
    # bollard-measured, asymmetric (reverse ~= half of forward); T2 mirrors T1
    't1_min_thrust': -39.2266, 't1_max_thrust': 78.4532,
    't2_min_thrust': -78.4532, 't2_max_thrust': 39.2266,
    't3_min_thrust': -274.6,   't3_max_thrust': 554.1,
    'steer_max': 0.3665,         # rad (+/-21 deg)
    'publish_rate': 20.0,        # Hz
    'control_mode': 'actuator',  # actuator | direction
    'return_to_zero': True,      # direction mode: release a key -> its axis back to 0
}

MODES = ('actuator', 'direction')


def keymap(mode: str) -> Dict[str, str]:
    """Key -> what it does, for the legend."""
    if mode == 'direction':
        return {'w': 'forward (T3 +)', 's': 'reverse (T3 -)',
                'a': 'turn left (steer -, T1/T2 -)', 'd': 'turn right (steer +, T1/T2 +)',
                'space': 'ALL-STOP'}
    # (the legend adds "release = 0" when return_to_zero is on)
    return {'w': 'T3 stern +', 'W': 'T3 stern -', 'a': 'T1 port bow +', 'A': 'T1 port bow -',
            'd': 'T2 stbd bow +', 'D': 'T2 stbd bow -', 's': 'steering +', 'S': 'steering -',
            'space': 'ALL-STOP'}


@dataclass
class _Held:
    t_next: float            # when the next repeat is due
    repeat: int = 0
    latched: bool = False    # parked at zero by the detent (or all-stop)
    cancelled: bool = False  # direction+return_to_zero: its opposite key is held too


@dataclass
class TeleopCore:
    params: dict = field(default_factory=lambda: dict(DEFAULTS))
    t1: float = 0.0
    t2: float = 0.0
    t3: float = 0.0
    steer: float = 0.0       # rad
    _held: Dict[str, _Held] = field(default_factory=dict)
    _space_down: bool = False

    # ------------------------------------------------------------ params
    @property
    def mode(self) -> str:
        return self.params['control_mode']

    def set_param(self, name, value):
        if name not in DEFAULTS:
            raise KeyError(f'unknown parameter {name!r}')
        if name == 'control_mode':
            if value not in MODES:
                raise ValueError(f'control_mode must be one of {MODES}')
        elif name == 'return_to_zero':
            if not isinstance(value, bool):
                raise ValueError('return_to_zero must be true or false')
        elif name != 'boat_namespace':
            value = float(value)
            if name in ('thrust_step', 'steer_step_deg', 'repeat_hz', 'publish_rate', 'steer_max') and value <= 0:
                raise ValueError(f'{name} must be > 0')
            if name in ('accel_gain', 'repeat_delay_ms') and value < 0:
                raise ValueError(f'{name} must be >= 0')
            if name == 'accel_max' and value < 1:
                raise ValueError('accel_max must be >= 1')
        self.params[name] = value
        if name.endswith('_thrust') or name == 'steer_max':
            self._clamp_all()
        if name == 'control_mode':
            self._held.clear()

    @property
    def bow_sync_max(self) -> float:
        """Largest bow command BOTH units can honour (their common range):
        direction mode drives the pair together, so one must not saturate."""
        p = self.params
        return min(-p['t1_min_thrust'], p['t1_max_thrust'], -p['t2_min_thrust'], p['t2_max_thrust'])

    # ------------------------------------------------------------ state
    def all_stop(self):
        self.t1 = self.t2 = self.t3 = self.steer = 0.0
        for h in self._held.values():
            h.latched = True          # a key still held after all-stop does nothing

    def commands(self):
        return {'T1': self.t1, 'T2': self.t2, 'T3': self.t3, 'steer': self.steer}

    def _clamp_all(self):
        p = self.params
        self.t1 = min(max(self.t1, p['t1_min_thrust']), p['t1_max_thrust'])
        self.t2 = min(max(self.t2, p['t2_min_thrust']), p['t2_max_thrust'])
        self.t3 = min(max(self.t3, p['t3_min_thrust']), p['t3_max_thrust'])
        self.steer = min(max(self.steer, -p['steer_max']), p['steer_max'])

    @staticmethod
    def _step(val, delta, lo, hi):
        """-> (new value, crossed zero). The zero detent."""
        n = val + delta
        crossed = (val > 0.0 and n < 0.0) or (val < 0.0 and n > 0.0)
        if crossed:
            n = 0.0
        return max(lo, min(hi, n)), crossed

    def _sync_bow(self):
        # steering [-steer_max, +steer_max] -> bow [-bow_sync_max, +bow_sync_max].
        # T2 gets the SAME sign as T1: both bow units are starboard-positive and
        # at the same station, so opposite signs would cancel exactly (see the
        # C++ sync_bow() comment).
        m = self.params['steer_max']
        b = self.bow_sync_max
        self.t1 = -b + (self.steer + m) * (2 * b) / (2 * m)
        self.t2 = self.t1

    def apply(self, key: str, accel: float = 1.0) -> bool:
        """One step of `key` (w a s d, or W A S D in actuator mode).
        -> True if it crossed zero (the detent)."""
        p = self.params
        ts, ss = p['thrust_step'] * accel, math.radians(p['steer_step_deg']) * accel
        sm = p['steer_max']
        if self.mode == 'direction':
            if key == 'w':
                self.t3, c = self._step(self.t3, +ts, p['t3_min_thrust'], p['t3_max_thrust'])
            elif key == 's':
                self.t3, c = self._step(self.t3, -ts, p['t3_min_thrust'], p['t3_max_thrust'])
            elif key == 'a':
                self.steer, c = self._step(self.steer, -ss, -sm, sm); self._sync_bow()
            elif key == 'd':
                self.steer, c = self._step(self.steer, +ss, -sm, sm); self._sync_bow()
            else:
                return False
            return c
        table = {
            'w': ('t3', +ts, 't3'), 'W': ('t3', -ts, 't3'),
            'a': ('t1', +ts, 't1'), 'A': ('t1', -ts, 't1'),
            'd': ('t2', +ts, 't2'), 'D': ('t2', -ts, 't2'),
            's': ('steer', +ss, None), 'S': ('steer', -ss, None),
        }
        if key not in table:
            return False
        attr, delta, lim = table[key]
        lo, hi = ((p[f'{lim}_min_thrust'], p[f'{lim}_max_thrust']) if lim else (-sm, sm))
        v, c = self._step(getattr(self, attr), delta, lo, hi)
        setattr(self, attr, v)
        return c

    # ------------------------------------------------------------ keys
    def update(self, down: Iterable[str], now: float) -> bool:
        """Feed the set of keys currently held ('w','a','s','d','shift','space')
        at time `now` [s]. Handles taps, holds, repeats and the detent.
        -> True if a command changed."""
        down = set(down)
        changed = False
        if 'space' in down and not self._space_down:
            self.all_stop()
            changed = True
        self._space_down = 'space' in down
        shift = 'shift' in down and self.mode == 'actuator'
        active = {(k.upper() if shift else k) for k in down if k in ('w', 'a', 's', 'd')}
        for k in list(self._held):
            if k not in active:
                del self._held[k]                    # released: accel resets
        if self.mode == 'direction' and self.params['return_to_zero']:
            # joystick: an axis with no key held (or BOTH of its keys) is 0
            for pos, neg, axis in (('w', 's', 't3'), ('d', 'a', 'steer')):
                both = pos in active and neg in active
                if both or not (pos in active or neg in active):
                    if getattr(self, axis) != 0.0:
                        setattr(self, axis, 0.0)
                        if axis == 'steer':
                            self._sync_bow()
                        changed = True
                for k in (pos, neg):
                    h = self._held.get(k)
                    if both:                          # no ramp while cancelled
                        h = self._held.setdefault(k, _Held(t_next=now))
                        h.latched = h.cancelled = True
                    elif h is not None and h.cancelled:
                        # partner released: this key starts again from 0, like a fresh press
                        del self._held[k]
        delay = self.params['repeat_delay_ms'] / 1000.0
        period = 1.0 / self.params['repeat_hz']
        for k in sorted(active):
            h = self._held.get(k)
            if h is None:                            # new press = one tap
                h = self._held[k] = _Held(t_next=now + delay)
                if self._space_down:
                    h.latched = True
                    continue
                h.latched = self.apply(k, 1.0)
                changed = True
                continue
            n = 0
            while not h.latched and now >= h.t_next and n < 5:  # held: repeat, growing
                n += 1                               # (cap: a stalled GUI must not burst)
                h.repeat += 1
                accel = min(self.params['accel_max'], 1.0 + h.repeat * self.params['accel_gain'])
                h.latched = self.apply(k, accel)
                h.t_next += period
                changed = True
            if now >= h.t_next:
                h.t_next = now + period             # drop what a stall missed
        return changed
