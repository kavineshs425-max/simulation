"""Scenario description and obstacle motion -- ROS-free so it is unit-testable.

Conventions:
  world frame  Gazebo world, ENU: x east, y north, yaw/course measured CCW
               from +x (east) in degrees in the YAML. 0 = east, 90 = north.
  boat frame   x forward, y to PORT (left) -- only used for the CPA log.

Everything is given in ABSOLUTE world coordinates, so a scenario is repeatable:
the same file puts the same obstacles in the same places every time, whatever
the boat is doing. (my_boat spawns at (-528, 186) facing east by default.)

TIME: scenario time tau starts at the moment the obstacles are RUN (the
spawner's ~/run service). Before that (tau < 0) every obstacle sits still at
its start pose. An obstacle with `start_delay` waits that long after run
before it begins to move.

Heading vs course: an obstacle's `heading` (where its bow points, what a box
detector sees as yaw) may differ from its `course` (where it actually goes) by
`crab_deg`. Planner 2 uses COURSE for CPA and HEADING for the Rule 13/19
sectors, and a scenario must be able to exercise that difference.
"""

from __future__ import annotations

import copy
import math
import os
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import yaml

from .catalog import ModelSpec, class_id, model_spec

# Truth-side COLREGs vessel class, mirrored from marine_planner_msgs/Contact.
VESSEL_CLASSES = {
    'unknown': 0,
    'power_driven': 1,
    'sailing': 2,
    'fishing': 3,
    'restricted_manoeuvre': 4,
    'not_under_command': 5,
}


def wrap(a: float) -> float:
    """Wrap an angle to (-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float

    def to_body(self, xm: float, ym: float) -> Tuple[float, float]:
        """World point -> this pose's frame (x fwd, y port)."""
        dx, dy = xm - self.x, ym - self.y
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return c * dx + s * dy, -s * dx + c * dy


@dataclass
class ObstacleState:
    """Ground truth for one obstacle at one instant, world frame."""
    id: int
    name: str
    det_class: str
    vessel_class: int
    x: float
    y: float
    z: float                  # hull-box centre height
    heading: float
    course: float
    speed: float
    l: float
    w: float
    h: float

    @property
    def vx(self) -> float:
        return self.speed * math.cos(self.course)

    @property
    def vy(self) -> float:
        return self.speed * math.sin(self.course)


# ---------------------------------------------------------------------------
# Motion models, world frame, evaluated at motion time tau >= 0.
# ---------------------------------------------------------------------------

class Motion:
    def at(self, tau: float) -> Tuple[float, float, float, float]:
        """-> (x, y, course, speed) at motion time tau."""
        raise NotImplementedError


@dataclass
class StaticMotion(Motion):
    x: float
    y: float
    yaw: float

    def at(self, tau):
        return self.x, self.y, self.yaw, 0.0


@dataclass
class ConstantVelocity(Motion):
    x: float
    y: float
    course: float
    speed: float

    def at(self, tau):
        return (self.x + self.speed * math.cos(self.course) * tau,
                self.y + self.speed * math.sin(self.course) * tau,
                self.course, self.speed)


@dataclass
class ConstantTurn(Motion):
    """Constant speed, constant turn rate. Closed form, no integration drift."""
    x: float
    y: float
    course: float
    speed: float
    rate: float  # [rad/s], positive = turning to port (CCW)

    def at(self, tau):
        if abs(self.rate) < 1e-9:
            return ConstantVelocity(self.x, self.y, self.course, self.speed).at(tau)
        c1 = self.course + self.rate * tau
        k = self.speed / self.rate
        return (self.x + k * (math.sin(c1) - math.sin(self.course)),
                self.y - k * (math.cos(c1) - math.cos(self.course)),
                wrap(c1), self.speed)


@dataclass
class WaypointMotion(Motion):
    """Polyline at constant speed. Holds at the last point, or loops.

    `stops` [(point_index, seconds), ...]: hold still at that point for that
    long (speed 0, heading = the course of the leg it arrived on), then go on.
    With `loop` the dwell repeats every lap; index 0 of a loop is the lap's END
    (it arrives there on the closing leg), so the first lap sets off at once.

    `turn_radius` [m]: a corner it sails through (not a stop, not a loop's start, not a reversal) is
    rounded by an arc this tight at most -- tighter only where the legs are too short for it, and then
    sailed slower, so the yaw rate never passes speed / turn_radius. 0 = sharp corners: the hull swings
    round in one sample (bag planner_route44: a 12 m ferry turned 90 deg in 0.2 s onto our line).
    From its start, a stop or to its end it sails straight for `hull_length` (it cannot swing its
    stern alongside a berth: the 30 m houseboat's arc kicked it into the yacht moored beside it)."""
    points: List[Tuple[float, float]]
    speed: float
    loop: bool = False
    stops: List[Tuple[int, float]] = field(default_factory=list)
    turn_radius: float = 0.0
    hull_length: float = 0.0
    _cum: List[float] = field(default_factory=list, repr=False)

    def __post_init__(self):
        if len(self.points) < 2:
            raise ValueError('waypoints motion needs at least 2 points')
        if self.speed <= 0.0:
            raise ValueError('waypoints motion needs speed > 0')
        pts = list(self.points)
        if self.loop and pts[0] != pts[-1]:
            pts.append(pts[0])
        self.points = pts
        self._cum = [0.0]
        for (xa, ya), (xb, yb) in zip(pts, pts[1:]):
            self._cum.append(self._cum[-1] + math.hypot(xb - xa, yb - ya))
        dwell = [0.0] * len(pts)
        for k, sec in self.stops:
            k, sec = int(k), float(sec)
            if not 0 <= k < len(pts) or sec < 0.0:
                raise ValueError(f'stops: point index {k} must be 0..{len(pts) - 1} '
                                 f'and seconds {sec} >= 0')
            if self.loop and k == 0:
                k = len(pts) - 1                  # the closing point IS point 0
            dwell[k] += sec
        # THE ARCS: corner k rounded at radius r (cut = how far before / after it the arc starts / ends)
        n, v, big_r = len(pts), self.speed, self.turn_radius
        head = [math.atan2(yb - ya, xb - xa) for (xa, ya), (xb, yb) in zip(pts, pts[1:])]
        legs = [b - a for a, b in zip(self._cum, self._cum[1:])]
        cut, arcs = [0.0] * n, {}

        def room(j, far):          # of leg j, what the arc at its other end may take (`far`: this end)
            fixed = far in (0, n - 1) or dwell[far] > 0.0
            return max(0.0, legs[j] - self.hull_length) if fixed else 0.5 * legs[j]
        for k in range(1, n - 1):
            th = wrap(head[k] - head[k - 1])
            tn = math.tan(abs(th) / 2.0)
            if big_r <= 0.0 or dwell[k] > 0.0 or min(legs[k - 1], legs[k]) <= 0.0 or tn < 1e-6:
                continue
            r = min(big_r, min(room(k - 1, k - 1), room(k, k + 1)) / tn)
            if r > 1e-3:
                cut[k], arcs[k] = r * tn, (r, th)
        # THE TIMELINE: (t0, t1, f(t - t0) -> (x, y, course, speed)) -- leg, arc, dwell, ...; a loop's
        # lap lasts until it departs the last point
        self._tl, t = [], 0.0

        def add(dur, f):
            nonlocal t
            if dur > 0.0:
                self._tl.append((t, t + dur, f))
                t += dur

        def hold(x, y, c):
            return lambda _t: (x, y, c, 0.0)

        def line(x, y, c):
            return lambda tt: (x + v * tt * math.cos(c), y + v * tt * math.sin(c), c, v)

        def turn(x, y, c, r, th):
            sg, w = math.copysign(1.0, th), v * min(1.0, r / big_r)   # yaw rate w / r <= v / turn_radius
            cx, cy = x - sg * r * math.sin(c), y + sg * r * math.cos(c)

            def f(tt):
                h = c + sg * w * tt / r
                return cx + sg * r * math.sin(h), cy - sg * r * math.cos(h), wrap(h), w
            return f, r * abs(th) / w

        add(dwell[0], hold(pts[0][0], pts[0][1], head[0]))
        for i in range(n - 1):
            (xa, ya), (xb, yb), c = pts[i], pts[i + 1], head[i]
            add((legs[i] - cut[i] - cut[i + 1]) / v,
                line(xa + cut[i] * math.cos(c), ya + cut[i] * math.sin(c), c))
            if i + 1 in arcs:
                f, dur = turn(xb - cut[i + 1] * math.cos(c), yb - cut[i + 1] * math.sin(c), c, *arcs[i + 1])
                add(dur, f)
            add(dwell[i + 1], hold(xb, yb, c))              # tied up: on the course it arrived on
        self._lap = t
        self._end = (pts[-1][0], pts[-1][1], head[-1], 0.0)  # past the end: hold

    def at(self, tau):
        t = max(0.0, tau)
        if self.loop and self._lap > 0.0:
            t = math.fmod(t, self._lap)
        for t0, t1, f in self._tl:
            if t < t1:
                return f(t - t0)
        return self._end


@dataclass
class StoppedMotion(Motion):
    """`base` until motion time `at_m`, then held still there (a collision)."""
    base: Motion
    at_m: float

    def at(self, tau):
        if tau < self.at_m:
            return self.base.at(tau)
        x, y, course, _ = self.base.at(self.at_m)
        return x, y, course, 0.0


# ---------------------------------------------------------------------------
# Scenario spec -> obstacles
# ---------------------------------------------------------------------------

@dataclass
class ObstacleSpec:
    """One obstacle exactly as written in the scenario YAML."""
    name: str
    model: str = 'brazilian'          # what it looks like (catalog.MODELS)
    det_class: str = ''               # label; '' = the model's default class
    vessel_class: str = 'unknown'
    start_delay: float = 0.0          # [s] after RUN before it starts moving
    crab_deg: float = 0.0
    on_contact: str = ''              # stop | pass; '' = settings.on_contact
    start: dict = field(default_factory=dict)
    motion: dict = field(default_factory=lambda: {'type': 'static'})
    size: Optional[List[float]] = None     # [l, w, h], primitive models only
    colour: Optional[List[float]] = None   # [r, g, b] 0..1
    texture: Optional[str] = None          # ABSOLUTE image path (resolved at parse)
    collision: Optional[str] = None        # box | mesh | none; None = the model's

    def appearance(self):
        """What the spawned MODEL depends on (from the config side; the
        ModelSpec is compared too, for edits to a model.yaml). If this changes
        the model must be respawned; anything else is only a new path."""
        return (self.model, self.det_class,
                None if self.size is None else tuple(self.size),
                None if self.colour is None else tuple(self.colour),
                self.texture, self.collision)


@dataclass
class Obstacle:
    id: int
    spec: ObstacleSpec
    motion: Motion
    size: Tuple[float, float, float]
    centre_z: float = 0.0
    model: Optional[ModelSpec] = None       # the resolved look/collision
    # settings.speed_scale: plays the path this many times faster. The SHAPE of
    # the path is kept (a turn keeps its radius, waypoints their corners); only
    # the time along it changes. start_delay is NOT scaled.
    speed_scale: float = 1.0
    # motion time = speed_scale * (time since start_delay) + time_offset. The
    # offset is what keeps the position CONTINUOUS when speed_scale changes
    # mid-run: the obstacle speeds up from where it is instead of jumping.
    time_offset: float = 0.0

    def _motion_time(self, tau: float) -> float:
        e = max(0.0, tau - self.spec.start_delay)
        return max(0.0, e * self.speed_scale + self.time_offset) if e > 0.0 else 0.0

    def state(self, tau: float) -> ObstacleState:
        """Truth at scenario time tau (seconds since RUN; < 0 = not run yet)."""
        m = self._motion_time(tau)
        x, y, course, speed = self.motion.at(m)
        speed *= self.speed_scale
        if tau < self.spec.start_delay:
            speed = 0.0          # sitting at the start, waiting
        heading = wrap(course + math.radians(self.spec.crab_deg))
        return ObstacleState(
            id=self.id, name=self.spec.name, det_class=self.spec.det_class,
            vessel_class=VESSEL_CLASSES[self.spec.vessel_class],
            x=x, y=y, z=self.centre_z,
            heading=heading, course=course, speed=speed,
            l=self.size[0], w=self.size[1], h=self.size[2])

    def samples(self, duration: float, step: float = 0.5):
        """Timed poses for the Gazebo plugin, [(tau, x, y, heading), ...], in
        scenario time from 0 (the run instant) to `duration`. The plugin holds
        the first sample before run and the last one after.

        The path is sampled, so a corner is cut by at most speed*step/2 --
        0.4 m at 1.5 m/s and the default 0.5 s."""
        if step <= 0.0:
            raise ValueError('sample step must be > 0')
        taus = [0.0]
        if 0.0 < self.spec.start_delay < duration:
            taus.append(self.spec.start_delay)      # exact start of motion
        t = step
        while t < duration:
            taus.append(t)
            t += step
        taus.append(duration)
        taus = sorted(set(round(v, 9) for v in taus))
        out = []
        for tau in taus:
            s = self.state(tau)
            out.append((tau, s.x, s.y, s.heading))
        return out


    def stopped_at(self, tau: float) -> 'Obstacle':
        """This obstacle, halted where it is at scenario time `tau` (it hit
        something). Identical before `tau`, so the path sent to Gazebo only
        changes from the moment of contact."""
        return Obstacle(self.id, self.spec, StoppedMotion(self.motion, self._motion_time(tau)),
                        self.size, self.centre_z, self.model, self.speed_scale, self.time_offset)

    def resumed_at(self, tau: float) -> 'Obstacle':
        """A stopped obstacle going on along its script from where it stopped, at scenario time
        `tau` (the time offset absorbs the wait, so it does not jump)."""
        e = max(0.0, tau - self.spec.start_delay)
        return Obstacle(self.id, self.spec, self.motion.base, self.size, self.centre_z, self.model,
                        self.speed_scale, self.motion.at_m - e * self.speed_scale)

    @property
    def is_stopped(self) -> bool:
        return isinstance(self.motion, StoppedMotion)

    def with_start(self, x: float, y: float, heading_deg: float) -> 'Obstacle':
        """The same obstacle with a new start pose (world frame).

        static / constant_velocity / constant_turn: start becomes (x, y) and
        heading_deg becomes the heading it sets off in.
        waypoints: the WHOLE path is translated so it begins at (x, y), keeping
        its shape. heading_deg is ignored, because the legs define the heading."""
        spec = copy.deepcopy(self.spec)
        if spec.motion.get('type') == 'waypoints':
            x0, y0 = self.motion.points[0]
            dx, dy = x - x0, y - y0
            spec.motion['points'] = [[float(p[0]) + dx, float(p[1]) + dy]
                                     for p in spec.motion.get('points', [])]
            if spec.start:
                spec.start = {'x': float(spec.start['x']) + dx,
                              'y': float(spec.start['y']) + dy}
        else:
            spec.start = {'x': float(x), 'y': float(y), 'heading_deg': float(heading_deg)}
        return build(spec, self.id, self.speed_scale)

    def with_speed_scale(self, k: float, tau_now: Optional[float] = None) -> 'Obstacle':
        """The same obstacle at a new speed_scale. With `tau_now` (a run is in
        progress) it carries on from exactly where it is at tau_now."""
        k = float(k)
        if tau_now is None:
            b = 0.0
        else:
            e = max(0.0, tau_now - self.spec.start_delay)
            b = self._motion_time(tau_now) - k * e
        return Obstacle(self.id, self.spec, self.motion, self.size, self.centre_z,
                        self.model, k, b)


# a scripted craft's turning radius, in hull lengths (waypoint corners are sailed on an arc); a paddled
# craft or a swimmer turns on the spot (the kadathu canoes' 3-11 m legs are timed that way).
# ponytail: one value for every other class; per-class radii when the catalog carries them
TURN_LENGTHS = 1.5
PIVOTS = ('kayak', 'swimmer')


def _deg(m: dict, key: str, default: float = 0.0) -> float:
    return math.radians(float(m.get(key, default)))


def build(spec: ObstacleSpec, oid: int, speed_scale: float = 1.0) -> Obstacle:
    try:
        ms = model_spec(spec.model, spec.size, spec.colour, spec.collision, spec.texture)
    except ValueError as e:
        raise ValueError(f'{spec.name}: {e}') from None
    if not spec.det_class:
        spec.det_class = ms.default_class
    class_id(spec.det_class)   # validates
    if spec.vessel_class not in VESSEL_CLASSES:
        raise ValueError(f'{spec.name}: vessel_class must be one of {list(VESSEL_CLASSES)}')

    m = dict(spec.motion)
    mtype = m.get('type', 'static')
    st = dict(spec.start)
    if mtype == 'waypoints':
        pts = [(float(p[0]), float(p[1])) for p in m.get('points', [])]
        if st:
            pts.insert(0, (float(st['x']), float(st['y'])))
        try:   # stop indices count the WHOLE path, a `start` included as index 0, so the UI
            # splitting points[0] off into `start` (ui normalise) leaves every stop where it was
            stops = [(int(k), float(sec)) for k, sec in m.get('stops', []) or []]
        except (TypeError, ValueError):
            raise ValueError(f'{spec.name}: stops must be [[point_index, seconds], ...]') from None
        try:
            motion: Motion = WaypointMotion(pts, float(m.get('speed', 0.0)),
                                            bool(m.get('loop', False)), stops,
                                            0.0 if spec.det_class in PIVOTS else TURN_LENGTHS * ms.size[0],
                                            ms.size[0])
        except ValueError as e:
            raise ValueError(f'{spec.name}: {e}') from None
    else:
        if 'x' not in st or 'y' not in st:
            raise ValueError(f'{spec.name}: start needs x and y (world coordinates)')
        x0, y0 = float(st['x']), float(st['y'])
        if mtype == 'static':
            motion = StaticMotion(x0, y0, _deg(st, 'heading_deg'))
        elif mtype == 'constant_velocity':
            motion = ConstantVelocity(x0, y0, _deg(st, 'heading_deg'), float(m['speed']))
        elif mtype == 'constant_turn':
            motion = ConstantTurn(x0, y0, _deg(st, 'heading_deg'), float(m['speed']),
                                  _deg(m, 'turn_rate_deg_s'))
        else:
            raise ValueError(f'{spec.name}: unknown motion type {mtype!r}')
    return Obstacle(oid, spec, motion, ms.size, ms.centre_z, ms, float(speed_scale))


# Spawner settings that may live in the config file's `settings:` section.
SETTINGS_DEFAULTS = {
    'world': 'sydney_regatta',     # running Gazebo world name
    'boat_model': 'my_boat',       # Gazebo model name of our boat (CPA log)
    'world_frame': 'world',        # frame_id stamped on the ground truth
    'name_prefix': 'obs',          # Gazebo model name = <prefix>_<obstacle name>
    'run_lead_s': 0.5,             # run starts this long after the service call
    'sample_step_s': 0.5,          # path sampling step for the plugin
    'duration_s': 600.0,           # paths are played this long after run, then hold
    'model_paths': [],             # extra folders of user models (relative = to this file)
    # my_boat's collision box, for the collision REPORT (Gazebo does the physics
    # itself). From my_boat_base.urdf.xacro: box 4.0 x 1.7, centre 0.23 m fwd.
    'boat_box_length': 4.0,
    'boat_box_width': 1.7,
    'boat_box_offset_x': 0.23,
    'collision_release_s': 1.0,    # a hit ends after this long with no contact
    # what an obstacle does when it touches another obstacle or the boat:
    #   stop -- halts where it touched (default); pass -- keeps to its path
    'on_contact': 'stop',
    # multiplies EVERY obstacle's speed (same paths, played faster/slower).
    # Also a live ROS parameter: ros2 param set /obstacle_spawner speed_scale 2.0
    'speed_scale': 1.0,
    # land/water map used to WARN about obstacles on land or running ashore:
    # '' = worlds/<world>_land.npz if the package has one, 'none' = off, or a path
    'land_mask': '',
}


@dataclass
class Scenario:
    name: str
    obstacles: List[ObstacleSpec]
    settings: dict = field(default_factory=lambda: dict(SETTINGS_DEFAULTS))

    @property
    def duration_s(self) -> float:
        return float(self.settings['duration_s'])

    def build(self) -> List[Obstacle]:
        k = float(self.settings['speed_scale'])
        return [build(s, i + 1, k) for i, s in enumerate(self.obstacles)]


_START_KEYS = {'x', 'y', 'heading_deg'}
_MOTION_KEYS = {'static': {'type'},
                'constant_velocity': {'type', 'speed', 'speed_kn'},
                'constant_turn': {'type', 'speed', 'speed_kn', 'turn_rate_deg_s'},
                'waypoints': {'type', 'speed', 'speed_kn', 'loop', 'points', 'stops'}}
KNOT = 1852.0 / 3600.0   # m/s
ON_CONTACT = ('stop', 'pass')


def _parse_settings(sc: dict, settings: dict) -> dict:
    bad = set(settings) - set(SETTINGS_DEFAULTS)
    if bad:
        raise ValueError(f'unknown settings keys {sorted(bad)}; '
                         f'allowed {sorted(SETTINGS_DEFAULTS)}')
    out = dict(SETTINGS_DEFAULTS)
    if 'duration_s' in sc:                     # older files put it under scenario:
        out['duration_s'] = sc['duration_s']
    out.update(settings)
    for k, v in SETTINGS_DEFAULTS.items():
        if isinstance(v, list):
            if not isinstance(out[k], list):
                raise ValueError(f'settings.{k} must be a list')
            out[k] = [str(x) for x in out[k]]
        else:
            out[k] = type(v)(out[k])
    if out['on_contact'] not in ON_CONTACT:
        raise ValueError(f"settings.on_contact must be one of {ON_CONTACT}")
    for k in ('run_lead_s', 'sample_step_s', 'duration_s', 'boat_box_length',
              'boat_box_width', 'speed_scale'):
        if out[k] <= 0.0:
            raise ValueError(f'settings.{k} must be > 0')
    return out


def parse_scenario(data: dict, base_dir: str = '.') -> Scenario:
    """`base_dir` is what relative paths in the file (texture) are relative to:
    the directory of the config file."""
    extra = set(data) - {'scenario', 'settings', 'obstacles'}
    if extra:
        raise ValueError(f'unknown top-level keys {sorted(extra)}')
    sc = data.get('scenario', {}) or {}
    unknown = set(sc) - {'name', 'duration_s'}
    if unknown:
        raise ValueError(f'unknown scenario keys {sorted(unknown)}')
    specs, names = [], set()
    for i, o in enumerate(data.get('obstacles', []) or []):
        o = dict(o)
        name = str(o.pop('name', f'obstacle_{i + 1}'))
        if name in names:
            raise ValueError(f'duplicate obstacle name {name!r}')
        names.add(name)
        start = o.pop('start', {}) or {}
        motion = o.pop('motion', {'type': 'static'}) or {'type': 'static'}
        bad = set(start) - _START_KEYS
        if bad:
            raise ValueError(f'{name}: unknown start keys {sorted(bad)}')
        mtype = motion.get('type', 'static')
        if mtype not in _MOTION_KEYS:
            raise ValueError(f'{name}: unknown motion type {mtype!r}')
        bad = set(motion) - _MOTION_KEYS[mtype]
        if bad:
            raise ValueError(f'{name}: unknown {mtype} keys {sorted(bad)}')
        motion = dict(motion)
        for key in ('speed', 'speed_kn'):          # top-level speed: the easy place
            if key in o:
                if 'speed' in motion or 'speed_kn' in motion:
                    raise ValueError(f'{name}: speed is given twice (top level and in motion)')
                if mtype == 'static':
                    raise ValueError(f'{name}: has a speed but motion is static -- '
                                     f'give it motion: {{type: constant_velocity}}')
                motion[key] = o.pop(key)
        if 'speed_kn' in motion:
            if 'speed' in motion:
                raise ValueError(f'{name}: give speed (m/s) OR speed_kn, not both')
            motion['speed'] = float(motion.pop('speed_kn')) * KNOT
        if mtype != 'static':
            if 'speed' not in motion:
                raise ValueError(f'{name}: {mtype} motion needs a speed (speed: m/s or '
                                 f'speed_kn: knots)')
            if float(motion['speed']) < 0.0:
                raise ValueError(f'{name}: speed must be >= 0')
        specs.append(ObstacleSpec(
            name=name, model=str(o.pop('model', 'brazilian')),
            det_class=str(o.pop('class', '')),
            vessel_class=str(o.pop('vessel_class', 'unknown')),
            start_delay=float(o.pop('start_delay', 0.0)),
            crab_deg=float(o.pop('crab_deg', 0.0)),
            on_contact=str(o.pop('on_contact', '')),
            size=o.pop('size', None), colour=o.pop('colour', None),
            texture=_abs(o.pop('texture', None), base_dir),
            collision=(None if o.get('collision') is None else str(o.pop('collision'))),
            start=start, motion=motion))
        o.pop('collision', None)
        if specs[-1].on_contact not in ('',) + ON_CONTACT:
            raise ValueError(f'{name}: on_contact must be one of {ON_CONTACT}')
        if o:
            raise ValueError(f'{name}: unknown keys {sorted(o)}')
    return Scenario(name=str(sc.get('name', 'unnamed')), obstacles=specs,
                    settings=_parse_settings(sc, data.get('settings', {}) or {}))


def predicted_overlaps(obstacles, duration: float, step: float = 0.1):
    """Pairs of obstacles whose SCRIPTED paths make their boxes overlap:
    [(name_a, name_b, t_first), ...]. Ghosts (collision: none) are skipped.
    Used to warn at start-up -- a scenario where two obstacles drive through
    each other is almost always a mistake in the start poses."""
    from .geometry import boxes_overlap
    solid = [o for o in obstacles if o.model is None or o.model.collision != 'none']
    moving = [o.spec.motion.get('type', 'static') != 'static' for o in solid]
    # two static obstacles never move: test such a pair once, at t = 0
    pairs = [(a, b) for a in range(len(solid)) for b in range(a + 1, len(solid))]
    found = {}

    def test(a, b, t, st):
        A, B = st[a], st[b]
        if (math.hypot(A.x - B.x, A.y - B.y) <= 0.5 * (math.hypot(A.l, A.w) + math.hypot(B.l, B.w))
                and boxes_overlap((A.x, A.y, A.heading, A.l, A.w), (B.x, B.y, B.heading, B.l, B.w))):
            found[(A.name, B.name)] = t
            return True
        return False

    st0 = [o.state(0.0) for o in solid]
    live = []
    for a, b in pairs:
        if not test(a, b, 0.0, st0) and (moving[a] or moving[b]):
            live.append((a, b))
    idx = sorted({k for p in live for k in p})
    n = int(duration / step)
    for i in range(1, n + 1):
        if not live:
            break
        t = i * step
        st = {k: solid[k].state(t) for k in idx}
        live = [(a, b) for a, b in live if not test(a, b, t, st)]
    return [(a, b, t) for (a, b), t in sorted(found.items(), key=lambda kv: kv[1])]


def _abs(path, base_dir):
    if path is None:
        return None
    path = os.path.expanduser(str(path))
    return path if os.path.isabs(path) else os.path.normpath(os.path.join(base_dir, path))


def load_scenario(path: str) -> Scenario:
    """Parse only: nothing is built, so user models can be registered first
    (their folders are given by the file's own settings.model_paths)."""
    with open(path, 'r') as f:
        return parse_scenario(yaml.safe_load(f) or {},
                              os.path.dirname(os.path.realpath(path)))
