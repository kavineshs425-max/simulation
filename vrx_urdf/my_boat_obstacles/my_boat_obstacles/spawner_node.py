"""obstacle_spawner -- put a scripted obstacle scenario into the running Gazebo.

Everything comes from ONE config file (parameter config_file; default
config/obstacles.yaml): the obstacles and the spawner's own `settings:`.

Flow
  1. On start, spawn every obstacle at its START pose (world coordinates from
     the config file). They sit still.
  2. ~/run (std_srvs/Trigger): all obstacles start moving TOGETHER. The
     spawner picks t0 = now + run_lead_s in sim time and sends it to every
     obstacle's MovingObstacle plugin on run_topic (ROS Float64, bridged to
     gz.msgs.Double by the launch file). Calling ~/run again restarts the run
     from the start poses.
  3. ~/reset (std_srvs/Trigger): obstacles jump back to their start poses and
     hold (sends -1).
  3b. ~/set_start (ros_gz_interfaces/SetEntityPose): give one obstacle a new
     start pose. entity.name = the scenario name ("crosser") or the Gazebo
     model name ("obs_crosser"); pose.position.x/y = world start;
     pose.orientation = the heading it sets off in (yaw only). The model is
     NOT respawned: the new path is sent to its plugin on samples_topic. If
     the scenario is running, the obstacle jumps onto its new path at the
     current run time. For waypoints the whole path is shifted to begin at
     (x, y) and the heading is ignored. The change lasts until the node
     restarts or the next reload; the config file is not edited.
  3c. ~/reload (std_srvs/Trigger): re-read the config file and apply the
     difference. A changed start/motion/delay/crab is sent as a new path (no
     respawn); a changed model/class/size/colour respawns that obstacle; new
     names are spawned and deleted names removed. A running scenario keeps
     running, and changed obstacles join it at the current run time. An
     invalid file changes NOTHING. world / boat_model / name_prefix need a
     restart. set_start overrides are replaced by what the file says.
  4. Ground truth is published from the SAME paths at the current sim time,
     so it matches what Gazebo shows. Your own sensors and detector see the
     models; this truth is only for scoring them and for RViz.

The current run value is re-sent at 2 Hz, so a dropped message only delays
the start by up to 0.5 s and the plugins cannot miss it.

The boat's true pose (for the CPA log only) is streamed with
`gz topic -e --json-output` from /world/<world>/dynamic_pose/info.
ros_gz_bridge cannot be used for that: its Pose_V -> TFMessage conversion
drops the model NAMES and the stamps, and the boat's own PosePublisher topic
arrives empty.
"""

import json
import math
import os
import subprocess
import tempfile
import threading
import time

import yaml
from rclpy.qos import DurabilityPolicy, QoSProfile

import rclpy
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point, Quaternion, Vector3
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from ros_gz_interfaces.srv import SetEntityPose
from std_msgs.msg import ColorRGBA, Float64, String
from std_srvs.srv import Trigger
from vision_msgs.msg import Detection3D, Detection3DArray, ObjectHypothesisWithPose
from visualization_msgs.msg import Marker, MarkerArray

from . import catalog, landmask, user_models
from .catalog import RUN_TOPIC, SAMPLES_TOPIC, obstacle_sdf, samples_text
from .geometry import boxes_overlap
from .scenario import (Obstacle, Pose2D, build, load_scenario, parse_scenario,
                       predicted_overlaps)

try:  # optional: only if the planner workspace is sourced
    from marine_planner_msgs.msg import Contact, ContactSet
except ImportError:  # pragma: no cover
    Contact = ContactSet = None


def yaw_to_quat(yaw):
    return Quaternion(x=0.0, y=0.0, z=math.sin(0.5 * yaw), w=math.cos(0.5 * yaw))


def parse_pose_line(line, model):
    """One `gz topic --json-output` Pose_V line -> (t, x, y, yaw) for `model`.

    gz omits zero-valued fields from the JSON (a pose at x=0 has no "x"), so
    every field is read with a default."""
    try:
        msg = json.loads(line)
    except ValueError:
        return None
    st = msg.get('header', {}).get('stamp', {})
    t = int(st.get('sec', 0)) + int(st.get('nsec', 0)) * 1e-9
    for ps in msg.get('pose', []):
        if ps.get('name') == model:
            pos, q = ps.get('position', {}), ps.get('orientation', {})
            qx, qy, qz, qw = (float(q.get(k, 0.0)) for k in ('x', 'y', 'z', 'w'))
            if 'w' not in q and not (qx or qy or qz):
                qw = 1.0
            yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
            return t, float(pos.get('x', 0.0)), float(pos.get('y', 0.0)), yaw
    return None


def collision_box(state, model_spec):
    """Footprint used for the collision REPORT: the obstacle's collision box
    (for collision: mesh, its bounding box -- a slight over-report)."""
    return (state.x, state.y, state.heading, state.l, state.w)


class ObstacleSpawner(Node):
    def __init__(self):
        super().__init__('obstacle_spawner')
        p = self.declare_parameter
        self.config_file = p('config_file', '').value
        run_topic = p('run_topic', RUN_TOPIC).value
        self.samples_topic = p('samples_topic', SAMPLES_TOPIC).value
        if not self.config_file:
            raise RuntimeError('obstacle_spawner: parameter config_file is required')
        self.share_dir = get_package_share_directory('my_boat_obstacles')
        self.scenario = load_scenario(self.config_file)
        self._apply_settings(self.scenario.settings)
        self._register_models(self.scenario)
        self.obstacles = self.scenario.build()
        # live speed control: starts at the file's settings.speed_scale
        self.declare_parameter('speed_scale', float(self.scenario.settings['speed_scale']))
        self.add_on_set_parameters_callback(self._on_params)
        self._warn_overlaps()
        self._warn_land()

        self.plugin_path = os.path.join(get_package_prefix('my_boat_obstacles'), 'lib',
                                        'libMovingObstacle.so')
        self.create_exe = os.path.join(get_package_prefix('ros_gz_sim'), 'lib',
                                       'ros_gz_sim', 'create')
        if not os.path.isfile(self.plugin_path):
            raise RuntimeError(f'plugin not built: {self.plugin_path}')
        self.run_topic = run_topic

        self.pub_run = self.create_publisher(Float64, run_topic, 10)
        self.pub_samples = self.create_publisher(String, self.samples_topic, 10)
        self.pending_paths = {}    # entity -> [text, sends left]
        self.pub_truth = self.create_publisher(Detection3DArray, '/sim/obstacles/ground_truth', 10)
        self.pub_contacts = (self.create_publisher(ContactSet, '/sim/obstacles/contacts', 10)
                             if ContactSet else None)
        self.pub_mk = self.create_publisher(MarkerArray, '/sim/obstacles/markers', 10)
        # one line per collision START and END: "START <name> t=<s after run>" / "END ..."
        self.pub_coll = self.create_publisher(String, '/sim/obstacles/collisions', 10)
        self.touching = {}          # id -> [tau contact began, tau last in contact]
        self.collision_count = 0
        self.create_service(Trigger, '~/run', self._on_run)
        self.create_service(Trigger, '~/reset', self._on_reset)
        self.create_service(SetEntityPose, '~/set_start', self._on_set_start)
        self.create_service(Trigger, '~/reload', self._on_reload)
        # --- for the browser UI (obstacle_ui) ---
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_state = self.create_publisher(String, '/sim/obstacles/state', 10)
        self.pub_applied = self.create_publisher(String, '~/applied_config', latched)
        self.pub_apply_result = self.create_publisher(String, '~/apply_result', 10)
        self.create_subscription(String, '~/apply_yaml', self._on_apply_yaml, 10)

        self.t0 = None             # sim time of tau = 0; None = not running
        self.state = {}            # id -> 'spawning' | 'live' | 'failed'
        self.procs = {}
        # a `create` that never exits although the model IS in Gazebo (seen 2026-10-05 at RTF 0.27:
        # obs_gate2_red existed, its create process still waited for a reply) left the obstacle
        # 'spawning' for ever and every Run was refused. After spawn_timeout_s the spawner ASKS
        # Gazebo: there -> live; not there -> spawn again (max spawn_tries), then failed.
        self.spawn_timeout = float(self.declare_parameter('spawn_timeout_s', 20.0).value)
        self.spawn_max_tries = int(self.declare_parameter('spawn_tries', 3).value)
        self.spawn_t, self.spawn_n, self.checking = {}, {}, set()
        self.cpa = {}
        self.boat = None           # latest (t, x, y, yaw), CPA log only
        self._stop = False
        self.pose_proc = None
        threading.Thread(target=self._pose_stream, daemon=True).start()

        for ob in self.obstacles:
            self._spawn(ob)
        self._publish_applied(self._file_text())
        self.create_timer(0.1, self._tick)
        self.create_timer(0.5, self._send_run)
        if ContactSet is None:
            self.get_logger().warn('marine_planner_msgs not found: /sim/obstacles/contacts '
                                   'disabled (source the dynamic_positioning_ai workspace)')
        self.get_logger().info(
            f"'{self.scenario.name}' from {self.config_file}: spawning "
            f'{len(self.obstacles)} obstacles at '
            f'their start poses. They hold still until: '
            f'ros2 service call /{self.get_name()}/run std_srvs/srv/Trigger')

    # ------------------------------------------------------------ settings
    def _apply_settings(self, st):
        self.world = st['world']
        self.boat_model = st['boat_model']
        self.world_frame = st['world_frame']
        self.prefix = st['name_prefix']
        self.run_lead = st['run_lead_s']
        self.step = st['sample_step_s']

    def _on_params(self, params):
        for prm in params:
            if prm.name != 'speed_scale':
                continue
            try:
                k = float(prm.value)
            except (TypeError, ValueError):
                return SetParametersResult(successful=False, reason='speed_scale must be a number')
            if k <= 0.0:
                return SetParametersResult(successful=False, reason='speed_scale must be > 0')
            self._apply_speed_scale(k)
        return SetParametersResult(successful=True)

    def _apply_speed_scale(self, k: float):
        """Every obstacle plays its path k times faster, starting NOW: paths
        are resent, so a running scenario speeds up / slows down in place."""
        if k == self.scenario.settings['speed_scale'] and all(
                o.speed_scale == k for o in self.obstacles):
            return
        self.scenario.settings['speed_scale'] = k
        tau = self._tau() if self.t0 is not None else None
        for i, ob in enumerate(self.obstacles):
            self.obstacles[i] = ob.with_speed_scale(k, tau)
            if self.state.get(ob.id) == 'live':
                self._send_path(self.obstacles[i])
        self.get_logger().info(f'speed_scale = {k:g}: every obstacle now moves at {k:g}x '
                               f'its configured speed')

    def _warn_land(self):
        """Tell the user when an obstacle starts on land or runs ashore.
        Returns the messages too (the UI shows them)."""
        out = []
        choice = self.scenario.settings['land_mask']
        if choice == 'none':
            return out
        path = (choice if choice else landmask.default_mask_path(self.share_dir, self.world))
        if not path:
            return out
        if not os.path.isabs(path):
            path = os.path.join(os.path.dirname(os.path.realpath(self.config_file)), path)
        try:
            mask = landmask.LandMask(path)
        except (OSError, KeyError, ValueError) as e:
            self.get_logger().warn(f'land mask {path} unusable: {e}')
            return out
        for name, kind, t, x, y in landmask.land_problems(
                self.obstacles, mask, min(self.scenario.duration_s, 600.0)):
            if kind == 'start':
                msg = (f"land check: '{name}' STARTS ON LAND (hull at "
                       f'({x:.1f}, {y:.1f})) -- move its start pose')
            else:
                msg = (f"land check: '{name}' runs ashore at t={t:.1f} s after "
                       f'run, hull at ({x:.1f}, {y:.1f}) -- change its path, '
                       f'speed or settings.duration_s')
            self.get_logger().warn(msg)
            out.append(msg)
        return out

    def _warn_overlaps(self):
        """Tell the user when two SCRIPTED paths run into each other."""
        out = []
        hits = predicted_overlaps(self.obstacles, min(self.scenario.duration_s, 300.0), 0.25)
        by_name = {o.spec.name: o for o in self.obstacles}
        for a, b, t in hits:
            modes = {self._on_contact(by_name[a]), self._on_contact(by_name[b])}
            then = ('both stop there' if modes == {'stop'} else
                    'they pass through each other' if modes == {'pass'} else
                    'the "stop" one halts, the "pass" one drives on')
            msg = (f"scenario check: '{a}' and '{b}' would overlap at t={t:.1f} s after run -- "
                   f'{then} (on_contact). Move a start pose if that is not intended.')
            self.get_logger().warn(msg)
            out.append(msg)
        return out

    def _register_models(self, sc):
        """Scan the models/ folders and register every user model."""
        dirs = user_models.default_dirs(self.config_file, self.share_dir,
                                        sc.settings['model_paths'])
        found = user_models.scan(dirs)
        catalog.set_user_models(found)
        for name, spec in sorted(found.items()):
            for w in user_models.look_warnings(spec):
                self.get_logger().warn(f"model '{name}': {w}")
        if found:
            self.get_logger().info('user models: ' + ', '.join(
                f"{n} ({m.size[0]:.2f} x {m.size[1]:.2f} x {m.size[2]:.2f} m)"
                for n, m in sorted(found.items())))
        return found

    # ------------------------------------------------------------ helpers
    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _entity(self, ob):
        return f'{self.prefix}_{ob.spec.name}'

    def _tau(self):
        """Scenario time: seconds since run; -1 while not running."""
        if self.t0 is None:
            return -1.0
        return self._now() - self.t0

    def _pose_stream(self):
        topic = f'/world/{self.world}/dynamic_pose/info'
        key = f'"name":"{self.boat_model}"'
        while not self._stop:
            try:
                self.pose_proc = subprocess.Popen(
                    ['gz', 'topic', '-e', '-t', topic, '--json-output'],
                    stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
                for line in self.pose_proc.stdout:
                    if self._stop:
                        break
                    if key in line:
                        s = parse_pose_line(line, self.boat_model)
                        if s is not None:
                            self.boat = s
            except OSError as e:
                self.get_logger().error(f'cannot run gz topic: {e}', throttle_duration_sec=10.0)
            if not self._stop:
                time.sleep(2.0)   # gz went away (sim restart?): retry

    # ------------------------------------------------------------ gazebo
    def _spawn(self, ob):
        sdf = obstacle_sdf(ob.spec.model, self._entity(ob), ob.spec.det_class,
                           ob.samples(self.scenario.duration_s, self.step),
                           self.plugin_path, self.run_topic, self.samples_topic,
                           ob.spec.size, ob.spec.colour, ob.spec.collision, ob.spec.texture)
        # BY FILE, not -string: one argument may not exceed 128 KB (MAX_ARG_STRLEN), and the plugin's sampled
        # path is in the SDF -- 1800 s of a looping craft at 0.5 s did not fit (Alappuzha scenario, 2026-10-14),
        # as the terrain did not before it
        cache = os.path.join(tempfile.gettempdir(), f'my_boat_obstacles_{os.getuid()}')
        os.makedirs(cache, exist_ok=True)
        path = os.path.join(cache, self._entity(ob) + '.sdf')
        with open(path, 'w') as fh:
            fh.write(sdf)
        self.procs[ob.id] = subprocess.Popen(
            [self.create_exe, '-world', self.world, '-name', self._entity(ob), '-file', path],
            stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
        self.state[ob.id] = 'spawning'
        self.spawn_t[ob.id] = time.monotonic()
        self.spawn_n[ob.id] = self.spawn_n.get(ob.id, 0) + 1

    def _check_stuck(self, ob):
        """In a thread: is the model in Gazebo although its create has not returned?"""
        name = self._entity(ob)
        try:
            out = subprocess.run(['gz', 'model', '--list'], capture_output=True, text=True, timeout=30).stdout
            there = any(line.strip() == f'- {name}' for line in out.splitlines())
        except (OSError, subprocess.TimeoutExpired):
            there = None                                  # cannot tell: look again next timeout
        proc = self.procs.get(ob.id)
        if there is None:
            self.spawn_t[ob.id] = time.monotonic()
        elif there:
            if proc is not None and proc.poll() is None:
                proc.kill()
            self.state[ob.id] = 'live'
            self.get_logger().warn(f"'{name}' is in Gazebo but its create never returned -- marked live")
            if all(v == 'live' for v in self.state.values()):
                self.get_logger().info('all obstacles spawned -- ready to run')
        else:
            if proc is not None and proc.poll() is None:
                proc.kill()
            if self.spawn_n.get(ob.id, 0) < self.spawn_max_tries:
                self.get_logger().warn(f"'{name}' did not spawn in {self.spawn_timeout:.0f} s -- trying again")
                self._spawn(ob)
            else:
                self.state[ob.id] = 'failed'
                self.get_logger().error(f"'{name}' did not spawn after {self.spawn_max_tries} tries")
        self.checking.discard(ob.id)

    def _remove(self, name):
        subprocess.run(['gz', 'service', '-s', f'/world/{self.world}/remove',
                        '--reqtype', 'gz.msgs.Entity', '--reptype', 'gz.msgs.Boolean',
                        '--timeout', '3000', '--req', f'name: "{name}" type: MODEL'],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=10)

    def _send_run(self):
        self.pub_run.publish(Float64(data=-1.0 if self.t0 is None else self.t0))
        # a new path is sent 3 times, 0.5 s apart (replacing is idempotent),
        # so one dropped message cannot leave Gazebo on the old path
        for name, rec in list(self.pending_paths.items()):
            self.pub_samples.publish(String(data=rec[0]))
            rec[1] -= 1
            if rec[1] <= 0:
                del self.pending_paths[name]

    # ------------------------------------------------------------ services
    def _on_run(self, _req, resp):
        self.collision_count = 0
        self._unstop_all()
        pending = [self._entity(o) for o in self.obstacles if self.state[o.id] != 'live']
        if pending:
            resp.success = False
            resp.message = f'not all obstacles are spawned yet: {pending}'
            return resp
        restart = self.t0 is not None
        self.t0 = self._now() + self.run_lead
        self.cpa = {}
        self._send_run()
        resp.success = True
        resp.message = (f"{'restarted' if restart else 'running'}: "
                        f'{len(self.obstacles)} obstacles move from t0 = {self.t0:.2f} s sim')
        self.get_logger().info(resp.message)
        return resp

    def _on_reset(self, _req, resp):
        self._unstop_all()
        self.t0 = None
        self.cpa = {}
        self._send_run()
        resp.success, resp.message = True, 'obstacles back at their start poses, holding'
        self.get_logger().info(resp.message)
        return resp

    def _send_path(self, ob):
        """Give a live obstacle a new path without respawning it."""
        self.cpa.pop(ob.id, None)
        name = self._entity(ob)
        text = name + ' ' + samples_text(ob.samples(self.scenario.duration_s, self.step))
        self.pending_paths[name] = [text, 3]
        self.pub_samples.publish(String(data=text))

    def _on_reload(self, _req, resp):
        ok, msg, _ = self._apply(lambda: load_scenario(self.config_file),
                                 os.path.basename(self.config_file), self._file_text())
        resp.success, resp.message = ok, msg
        return resp

    def _file_text(self):
        try:
            with open(self.config_file) as f:
                return f.read()
        except OSError:
            return ''

    def _on_apply_yaml(self, msg):
        """The UI's Update button: apply a configuration given as YAML text,
        WITHOUT touching the file. Reply on ~/apply_result with the same id."""
        try:
            req = json.loads(msg.data)
            rid, text = req.get('id'), req['yaml']
        except (ValueError, KeyError, TypeError) as e:
            self.pub_apply_result.publish(String(data=json.dumps(
                {'id': None, 'ok': False, 'message': f'bad request: {e}', 'warnings': []})))
            return
        base = os.path.dirname(os.path.realpath(self.config_file))
        ok, message, warnings = self._apply(
            lambda: parse_scenario(yaml.safe_load(text) or {}, base), 'the UI', text)
        self.pub_apply_result.publish(String(data=json.dumps(
            {'id': rid, 'ok': ok, 'message': message, 'warnings': warnings})))

    def _publish_applied(self, text):
        self.applied_text = text
        self.pub_applied.publish(String(data=json.dumps(
            {'config_file': os.path.realpath(self.config_file), 'yaml': text})))

    def _apply(self, loader, source, text):
        """Apply a new configuration and change only what differs (reload and
        the UI both come here). -> (ok, message, warnings). On any error NOTHING
        changes."""
        saved_user = dict(catalog.USER)
        try:
            new_sc = loader()
            # models/ is re-scanned too, so a freshly pasted model is usable
            self._register_models(new_sc)
            # ids are kept per NAME so tracks stay comparable across a reload
            old = {ob.spec.name: ob for ob in self.obstacles}
            next_id = max([ob.id for ob in self.obstacles] + [0]) + 1
            new_obs = []
            for spec in new_sc.obstacles:
                if spec.name in old:
                    oid = old[spec.name].id
                else:
                    oid, next_id = next_id, next_id + 1
                new_obs.append(build(spec, oid, float(new_sc.settings['speed_scale'])))
        except (OSError, ValueError, KeyError, TypeError, yaml.YAMLError) as e:
            catalog.set_user_models(saved_user)      # nothing changes on error
            msg = f'config NOT applied: {e}'
            self.get_logger().error(msg)
            return False, msg, []

        fixed = [k for k in ('world', 'boat_model', 'name_prefix')
                 if new_sc.settings[k] != self.scenario.settings[k]]
        if fixed:
            self.get_logger().warn(f'settings {fixed} changed but need a restart; '
                                   f'keeping the running values')
            for k in fixed:
                new_sc.settings[k] = self.scenario.settings[k]
        self.scenario = new_sc
        self._apply_settings(new_sc.settings)

        new_names = {ob.spec.name for ob in new_obs}
        removed = [ob for ob in self.obstacles if ob.spec.name not in new_names]
        added, respawned, repathed = [], [], []
        for ob in removed:
            if self.state.get(ob.id) in ('spawning', 'live'):
                self._remove(self._entity(ob))
            self.state.pop(ob.id, None)
            self.cpa.pop(ob.id, None)
        for ob in new_obs:
            prev = old.get(ob.spec.name)
            if prev is None:
                added.append(ob)
            elif (prev.spec.appearance() != ob.spec.appearance()
                  or prev.model != ob.model            # its model.yaml was edited
                  or self.state.get(ob.id) != 'live'):
                if self.state.get(ob.id) in ('spawning', 'live'):
                    self._remove(self._entity(prev))
                respawned.append(ob)
            else:
                repathed.append(ob)
        self.obstacles = new_obs
        self.touching = {}
        # the new speed_scale wins; mirror it into the ROS param
        # (the callback sees nothing to change: new_obs already use it)
        self.set_parameters([rclpy.Parameter('speed_scale', rclpy.Parameter.Type.DOUBLE,
                                             float(new_sc.settings['speed_scale']))])
        warnings = self._warn_overlaps() + self._warn_land()
        for ob in added + respawned:
            self._spawn(ob)
        for ob in repathed:
            self._send_path(ob)
        msg = (f'applied from {source}: {len(repathed)} new path, {len(respawned)} respawned, '
               f'{len(added)} added, {len(removed)} removed'
               + (f' (restart needed for {fixed})' if fixed else ''))
        self.get_logger().info(msg)
        self._publish_applied(text)
        return True, msg, warnings

    def _find(self, name):
        for i, ob in enumerate(self.obstacles):
            if name in (ob.spec.name, self._entity(ob)):
                return i, ob
        return None, None

    def _on_set_start(self, req, resp):
        i, ob = self._find(req.entity.name)
        if ob is None:
            self.get_logger().error(
                f"set_start: no obstacle '{req.entity.name}'; have "
                f'{[o.spec.name for o in self.obstacles]}')
            resp.success = False
            return resp
        if self.state[ob.id] != 'live':
            self.get_logger().error(f"set_start: '{ob.spec.name}' is not spawned yet")
            resp.success = False
            return resp
        q = req.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        try:
            new = ob.with_start(req.pose.position.x, req.pose.position.y, math.degrees(yaw))
        except ValueError as e:
            self.get_logger().error(f"set_start '{ob.spec.name}': {e}")
            resp.success = False
            return resp
        self.obstacles[i] = new
        self._send_path(new)
        s0 = new.state(0.0)
        extra = ' (waypoints: whole path shifted, heading ignored)' \
            if new.spec.motion.get('type') == 'waypoints' else ''
        when = ('running -- it jumps onto its new path now' if self.t0 is not None
                else 'holding there until run')
        self.get_logger().info(
            f"set_start '{new.spec.name}': start ({s0.x:.2f}, {s0.y:.2f}) heading "
            f'{math.degrees(s0.heading):.1f} deg{extra}; {when}')
        resp.success = True
        return resp

    # ------------------------------------------------------------ tick
    def _tick(self):
        for ob in self.obstacles:
            if self.state[ob.id] != 'spawning':
                continue
            rc = self.procs[ob.id].poll()
            if rc is None:
                if (time.monotonic() - self.spawn_t.get(ob.id, time.monotonic()) > self.spawn_timeout
                        and ob.id not in self.checking):
                    self.checking.add(ob.id)
                    threading.Thread(target=self._check_stuck, args=(ob,), daemon=True).start()
                continue
            if rc == 0:
                self.state[ob.id] = 'live'
                self.get_logger().info(f"spawned '{self._entity(ob)}' ({ob.spec.model}, "
                                       f'class {ob.spec.det_class})')
                if all(s == 'live' for s in self.state.values()):
                    self.get_logger().info('all obstacles spawned -- ready to run')
            else:
                err = self.procs[ob.id].stderr.read().decode(errors='replace')[-400:]
                self.get_logger().error(f"spawn of '{self._entity(ob)}' failed: {err}")
                self.state[ob.id] = 'failed'

        tau = self._tau()
        truths = [(ob, ob.state(tau)) for ob in self.obstacles if self.state[ob.id] == 'live']
        self._publish(truths, self.get_clock().now().to_msg())
        if self.t0 is not None and tau >= 0.0:
            self._watch_cpa(truths, tau)
        self._watch_collisions(truths, tau)
        self._publish_state(tau)

    def _publish_state(self, tau):
        """Everything the UI's map draws, once per tick, as one JSON string."""
        st = self.scenario.settings
        obs = []
        for ob in self.obstacles:
            s = ob.state(tau)
            status = self.state.get(ob.id, '?')
            if status == 'live':
                status = ('stopped' if ob.is_stopped else
                          'waiting' if self.t0 is None or tau < ob.spec.start_delay else
                          'moving' if s.speed > 0.0 else 'still')
            obs.append({'name': ob.spec.name, 'model': ob.spec.model, 'class': ob.spec.det_class,
                        'x': round(s.x, 3), 'y': round(s.y, 3), 'heading': round(s.heading, 4),
                        'l': s.l, 'w': s.w, 'speed': round(s.speed, 3), 'status': status})
        boat = None
        if self.boat is not None:
            bt, bx, by, byaw = self.boat
            boat = {'name': self.boat_model, 't': round(bt, 4), 'x': round(bx, 3), 'y': round(by, 3),
                    'heading': round(byaw, 4), 'l': st['boat_box_length'],
                    'w': st['boat_box_width'], 'offset_x': st['boat_box_offset_x']}
        self.pub_state.publish(String(data=json.dumps({
            'tau': round(tau, 2), 'running': self.t0 is not None, 'boat': boat,
            'obstacles': obs, 'touching': [v[2] for v in self.touching.values()],
            'speed_scale': st['speed_scale']})))

    def _on_contact(self, ob):
        return ob.spec.on_contact or self.scenario.settings['on_contact']

    def _halt(self, ob, tau, why):
        """Halt an obstacle where it is (it touched something)."""
        if ob.is_stopped or self._on_contact(ob) != 'stop' or self.t0 is None or tau < 0.0:
            return
        i = next(k for k, o in enumerate(self.obstacles) if o.id == ob.id)
        self.obstacles[i] = ob.stopped_at(tau)
        self._send_path(self.obstacles[i])
        self.get_logger().info(f"'{ob.spec.name}' stopped at t={tau:.1f} s: touched {why}")

    def _unstop_all(self):
        """run / reset: every obstacle back on its full scripted path, from the
        start (a mid-run speed change's time offset is dropped too)."""
        for i, ob in enumerate(self.obstacles):
            if ob.is_stopped or ob.time_offset != 0.0:
                motion = ob.motion.base if ob.is_stopped else ob.motion
                self.obstacles[i] = Obstacle(ob.id, ob.spec, motion, ob.size, ob.centre_z,
                                             ob.model, ob.speed_scale, 0.0)
                self._send_path(self.obstacles[i])
        self.touching = {}

    def _resume_when_clear(self, solid, boat, tau, step=0.5):
        """A craft halted by a contact goes on along its script once the water ahead is clear for a
        hull length of its own way (others played on along their scripts, the boat held where it is).
        A real skipper stops, then carries on (bag planner_route46: f1_ferry stopped by a shikara
        stayed in the canal for good and blocked it)."""
        # ponytail: two craft stopped bow to bow block each other for good; add a back-off if a scenario needs it
        cur = {o.id: o for o in self.obstacles}
        for i, ob in enumerate(self.obstacles):
            if not ob.is_stopped or self.state.get(ob.id) != 'live' or tau < ob.spec.start_delay:
                continue
            go = ob.resumed_at(tau)
            n = int(math.ceil(ob.size[0] / max(go.state(tau).speed, 0.3) / step))
            clear = True
            for k in range(1, n + 1):           # touching NOW is fine: what counts is moving apart
                t = tau + k * step
                me = collision_box(go.state(t), ob.model)
                if (boat is not None and boxes_overlap(boat, me)) or any(
                        boxes_overlap(me, collision_box(cur[o.id].state(t), o.model))
                        for o, _ in solid if o.id != ob.id):
                    clear = False
                    break
            if clear:
                self.obstacles[i] = go
                self._send_path(go)
                self.get_logger().info(f"'{ob.spec.name}' resumed at t={tau:.1f} s: clear ahead")

    def _watch_collisions(self, truths, tau):
        """Contacts between my_boat and obstacles, and between obstacles.

        Gazebo does the boat-vs-obstacle physics itself. Obstacles are moved
        kinematically, so against EACH OTHER they would pass straight through;
        that is what on_contact: stop is for -- the obstacle halts where it
        touched. Every contact is also reported: a WARN in the log and
        START/END lines on /sim/obstacles/collisions."""
        st = self.scenario.settings
        solid = [(ob, s) for ob, s in truths
                 if ob.model is None or ob.model.collision != 'none']
        now = {}
        boat = None
        if self.boat is not None:
            _, bx, by, byaw = self.boat
            off = st['boat_box_offset_x']
            boat = (bx + off * math.cos(byaw), by + off * math.sin(byaw), byaw,
                    st['boat_box_length'], st['boat_box_width'])
            for ob, s in solid:
                if boxes_overlap(boat, collision_box(s, ob.model)):
                    now[f'boat:{ob.id}'] = (ob.spec.name, [ob], self.boat_model)
        for i in range(len(solid)):
            for j in range(i + 1, len(solid)):
                (oa, sa), (ob_, sb) = solid[i], solid[j]
                if boxes_overlap(collision_box(sa, oa.model), collision_box(sb, ob_.model)):
                    now[f'{oa.id}:{ob_.id}'] = (f'{oa.spec.name}<->{ob_.spec.name}',
                                                [oa, ob_], None)
        for key, (label, obs, boat_name) in now.items():
            if key in self.touching:
                self.touching[key][1] = tau
                continue
            self.touching[key] = [tau, tau, label]
            self.collision_count += 1
            self.pub_coll.publish(String(data=f'START {label} t={tau:.1f}'))
            what = (f"{boat_name} hit '{obs[0].spec.name}' ({obs[0].spec.model})" if boat_name
                    else f"'{obs[0].spec.name}' hit '{obs[1].spec.name}'")
            self.get_logger().warn(f'COLLISION #{self.collision_count}: {what} '
                                   f'at t={tau:.1f} s after run')
            if boat_name:
                self._halt(obs[0], tau, boat_name)
            else:
                self._halt(obs[0], tau, f"'{obs[1].spec.name}'")
                self._halt(obs[1], tau, f"'{obs[0].spec.name}'")
        self._resume_when_clear(solid, boat, tau)
        # a hit is ONE collision even though the boat bounces along the hull
        # while being shoved: it only ends after collision_release_s clear
        release = st['collision_release_s']
        for key in list(self.touching):
            if key not in now and tau - self.touching[key][1] > release:
                began, _, label = self.touching.pop(key)
                self.pub_coll.publish(String(data=f'END {label} t={tau:.1f}'))
                self.get_logger().info(f'collision {label} ended (lasted {tau - began:.1f} s)')

    def _watch_cpa(self, truths, tau):
        if self.boat is None:
            return
        _, bx, by, byaw = self.boat
        for ob, s in truths:
            r = math.hypot(s.x - bx, s.y - by)
            rec = self.cpa.setdefault(ob.id, [r, tau, False])
            if r < rec[0]:
                rec[0], rec[1], rec[2] = r, tau, False
            elif not rec[2] and r > rec[0] + 2.0:
                rec[2] = True
                self.get_logger().info(f"CPA '{ob.spec.name}': {rec[0]:.1f} m centre-to-centre "
                                       f'at t={rec[1]:.1f} s after run')

    # ------------------------------------------------------------ output
    def _publish(self, truths, stamp):
        det = Detection3DArray()
        det.header.frame_id, det.header.stamp = self.world_frame, stamp
        for ob, s in truths:
            d = Detection3D()
            d.header = det.header
            d.id = self._entity(ob)
            d.bbox.center.position = Point(x=s.x, y=s.y, z=s.z)
            d.bbox.center.orientation = yaw_to_quat(s.heading)
            d.bbox.size = Vector3(x=s.l, y=s.w, z=s.h)
            h = ObjectHypothesisWithPose()
            h.hypothesis.class_id, h.hypothesis.score = s.det_class, 1.0
            h.pose.pose = d.bbox.center
            d.results.append(h)
            det.detections.append(d)
        self.pub_truth.publish(det)

        if self.pub_contacts:
            cs = ContactSet()
            cs.header = det.header
            for ob, s in truths:
                c = Contact()
                c.id = s.id
                c.position = Point(x=s.x, y=s.y, z=s.z)
                c.velocity = Vector3(x=s.vx, y=s.vy, z=0.0)
                c.yaw, c.l, c.w, c.h = float(s.heading), float(s.l), float(s.w), float(s.h)
                c.confidence = 1.0
                c.vessel_class = s.vessel_class
                cs.contacts.append(c)
            self.pub_contacts.publish(cs)

        arr = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)
        for i, (ob, s) in enumerate(truths):
            t = Marker()
            t.header = det.header
            t.ns, t.id, t.type = 'label', i, Marker.TEXT_VIEW_FACING
            t.pose.position = Point(x=s.x, y=s.y, z=s.z + 0.5 * s.h + 1.0)
            t.pose.orientation.w = 1.0
            t.scale.z = 1.2
            t.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.9)
            t.text = f'{ob.spec.name} [{s.det_class}] {s.speed:.1f} m/s'
            t.lifetime = Duration(sec=1)
            arr.markers.append(t)
            if s.speed > 0.05:
                a = Marker()
                a.header = det.header
                a.ns, a.id, a.type = 'velocity', i, Marker.ARROW
                a.points = [Point(x=s.x, y=s.y, z=s.z),
                            Point(x=s.x + 5.0 * s.vx, y=s.y + 5.0 * s.vy, z=s.z)]
                a.scale = Vector3(x=0.25, y=0.6, z=0.6)
                a.color = ColorRGBA(r=0.1, g=0.8, b=1.0, a=1.0)
                a.pose.orientation.w = 1.0
                a.lifetime = Duration(sec=1)
                arr.markers.append(a)
        self.pub_mk.publish(arr)

    def destroy_node(self):
        self._stop = True
        if self.pose_proc:
            self.pose_proc.kill()
        for ob in self.obstacles:
            if self.state.get(ob.id) in ('spawning', 'live'):
                self._remove(self._entity(ob))
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleSpawner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
