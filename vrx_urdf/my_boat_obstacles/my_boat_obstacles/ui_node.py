"""obstacle_ui -- the SIMU browser UI for the obstacle scenario.

    ros2 launch my_boat_obstacles moving_obstacles.launch.py     # starts this too
    then open  http://localhost:8090

What it does (the page itself is ui/index.html):
  * GET  /            the page
  * GET  /map.png     the world's land/water map (worlds/<world>_land.npz), 1 px = 1 m
  * GET  /map.json    where that image sits in world coordinates
  * GET  /config      the configuration the sim is RUNNING (from the spawner)
  * GET  /models      every model an obstacle can use (built-in + models/ folders)
  * GET  /events      live state, 10 Hz, as a server-sent-event stream; plus
                      `event: tracks` -- the tracker's tracks (marine_tracker's
                      JSON topic), moved from the EKF map frame into Gazebo world
                      coordinates (track_view.MapAlignment) so they sit on this map
  * POST /check       live checks of an EDITED configuration (not applied):
                      config errors, obstacles on land / running ashore, paths
                      that run into each other -- so the map can warn as you edit
  * POST /update      apply an edited configuration to the sim   (Update button)
  * POST /save        write it to the config file (+ .bak)       (Save button)
  * POST /run, /reset the spawner's run / reset services
  * POST /ego         move my_boat in Gazebo (gz set_pose)
  ENVIRONMENT (canals, basins, land, docks, bridges -- the sim_environment node):
  * GET  /env/types   the structure types + their parameters (the page builds its form from it)
  * GET  /env/config  the environment the sim is running
  * POST /env/geometry  footprints of structures (local frame) for drawing a draft
  * POST /env/check   warnings for an edited environment (not applied)
  * POST /env/update  apply it (sim_environment ~/apply_yaml)    /env/save  write the file
                      (into the VECTOR MAP: physical features replaced, information kept)
  * GET  /route1/info, /route1/reason.png, /route1/reason.bin
                      route planner 1's COMPILED map (route_one_planner map_compiler,
                      /route_one/map/*): why each cell is not allowed, and the canal graph
  ROUTE PLANNER 1 goals (route_one_planner route_server, its NavigateRoute action):
  * POST /route1/goal   {mode: single|waypoints, waypoints: [[x, y, heading|null], ...], berth: ''}
  * POST /route1/cancel   * GET /route1/berths
    and on /events: `event: route` (a new route) / `event: route_status` (2 Hz) /
    `event: route2` (route planner 2's local plan + decision, 10 Hz)
  * GET  /vmap        the vector map in local metres, for the page's overlay: every
                      feature + its class/attributes, with depths judged against the
                      boat's draft at the current water level

Only the Python standard library serves HTTP (no extra packages). It binds to
127.0.0.1 by default: set host:=0.0.0.0 to reach it from another machine.

The spawner owns the simulation; this node never touches Gazebo obstacles
itself. Update sends YAML to the spawner's ~/apply_yaml and waits for
~/apply_result, so the UI gets exactly the checks the file would (unknown keys,
land, overlaps) as the reply.
"""

import io
import json
import math
import os
import re
import shutil
import subprocess
import threading
import time
import uuid
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from std_srvs.srv import Trigger

from . import environment as E
from .track_view import MapAlignment

# order the keys are written in when a configuration is saved
_KEY_ORDER = ['name', 'model', 'class', 'vessel_class', 'colour', 'size', 'texture', 'collision',
              'on_contact', 'start_delay', 'crab_deg', 'start', 'speed', 'speed_kn', 'motion']


_PLAIN = re.compile(r'^[A-Za-z_][A-Za-z0-9_.\-/]*$')
_RESERVED = {'true', 'false', 'yes', 'no', 'on', 'off', 'null', 'none', '~'}   # what PyYAML reads as bool/null


def _flow(v):
    """A value in YAML flow style, written the way a person would: plain words
    unquoted, {x: 1, y: 2} maps, [a, b] lists. Anything unusual falls back to
    JSON quoting, which YAML also accepts."""
    if isinstance(v, bool):
        return 'true' if v else 'false'
    if v is None:
        return 'null'
    if isinstance(v, (int, float)):
        return repr(float(v)) if isinstance(v, float) and v != int(v) else (
            str(int(v)) if float(v).is_integer() else repr(v))
    if isinstance(v, str):
        return v if _PLAIN.match(v) and v.lower() not in _RESERVED else json.dumps(v)
    if isinstance(v, (list, tuple)):
        return '[' + ', '.join(_flow(x) for x in v) + ']'
    if isinstance(v, dict):
        return '{' + ', '.join(f'{_flow(k)}: {_flow(x)}' for k, x in v.items()) + '}'
    return json.dumps(v)


def dump_obstacles(obstacles) -> str:
    """The `obstacles:` block, one readable entry per obstacle."""
    lines = ['obstacles:']
    for ob in obstacles:
        keys = [k for k in _KEY_ORDER if k in ob] + [k for k in ob if k not in _KEY_ORDER]
        first = True
        for k in keys:
            v = ob[k]
            if k == 'motion' and isinstance(v, dict) and 'points' in v:
                # a path reads better one point per line
                lines.append(('  - ' if first else '    ') + 'motion:')
                for mk, mv in v.items():
                    if mk == 'points':
                        lines.append('      points:')
                        lines += [f'        - {_flow(p)}' for p in mv]
                    else:
                        lines.append(f'      {mk}: {_flow(mv)}')
            else:
                lines.append(('  - ' if first else '    ') + f'{k}: {_flow(v)}')
            first = False
        lines.append('')
    return '\n'.join(lines) + '\n'


def dump_config(doc) -> str:
    out = []
    for key in ('scenario', 'settings'):
        if doc.get(key):
            out.append(f'{key}:')
            out += [f'  {k}: {_flow(v)}' for k, v in doc[key].items()]
            out.append('')
    return '\n'.join(out) + '\n' + dump_obstacles(doc.get('obstacles', []))


_ENV_KEY_ORDER = ['name', 'type', 'x', 'y', 'heading_deg', 'length', 'width', 'span', 'bank_width',
                  'open_start', 'open_end', 'points']


def dump_environment(doc) -> str:
    """An environment document, one readable entry per item."""
    out = []
    if doc.get('settings'):
        out.append('settings:')
        out += [f'  {k}: {_flow(v)}' for k, v in doc['settings'].items()]
        out.append('')
    for sec in ('land', 'canals', 'basins', 'structures'):
        items = doc.get(sec) or []
        out.append(f'{sec}:' + ('' if items else ' []'))
        for it in items:
            keys = [k for k in _ENV_KEY_ORDER if k in it] + [k for k in it if k not in _ENV_KEY_ORDER]
            for n, k in enumerate(keys):
                out.append(('  - ' if n == 0 else '    ') + f'{k}: {_flow(it[k])}')
        out.append('')
    return '\n'.join(out)


def merge_env_file(original: str, doc) -> str:
    """Save = keep the comment header above the first key, regenerate the rest."""
    lines = original.splitlines()
    idx = next((i for i, l in enumerate(lines) if re.match(r'^[a-z_]+:', l)), len(lines))
    head = '\n'.join(lines[:idx]).rstrip('\n')
    return (head + '\n\n' if head else '') + dump_environment(doc)


class CombinedMask:
    """World land + the environment (terrain + blocking structure parts), with
    LandMask's is_land -- so the obstacle checks see canal banks and piers too."""

    def __init__(self, world, terrain, polys):
        self.world, self.terrain = world, terrain
        self.polys = [(min(p[0] for p in poly), min(p[1] for p in poly), max(p[0] for p in poly),
                       max(p[1] for p in poly), poly) for _n, poly in polys]

    def is_land(self, x, y):
        if self.world is not None and self.world.is_land(x, y):
            return True
        if self.terrain is not None and self.terrain.is_solid(x, y):
            return True
        for a, b, c, d, poly in self.polys:
            if a <= x <= c and b <= y <= d and _inside(x, y, poly):
                return True
        return False


def _inside(x, y, poly):
    inside, n = False, len(poly)
    for i in range(n):
        (x1, y1), (x2, y2) = poly[i], poly[(i + 1) % n]
        if (y1 > y) != (y2 > y) and x < (x2 - x1) * (y - y1) / (y2 - y1) + x1:
            inside = not inside
    return inside


def merge_into_file(original: str, obstacles) -> str:
    """Save = keep everything above `obstacles:` (the documented header and
    settings) and any comment lines trailing the file; regenerate only the
    obstacles block."""
    lines = original.splitlines()
    idx = next((i for i, l in enumerate(lines) if re.match(r'^obstacles:\s*(#.*)?$', l)), None)
    if idx is None:
        return original.rstrip('\n') + '\n\n' + dump_obstacles(obstacles)
    last = max((i for i, l in enumerate(lines) if l.strip() and not l.lstrip().startswith('#')),
               default=idx)
    tail = [l for l in lines[last + 1:]]
    head = '\n'.join(lines[:idx])
    return head.rstrip('\n') + '\n\n' + dump_obstacles(obstacles) + (
        '\n'.join(tail).strip('\n') + '\n' if any(t.strip() for t in tail) else '')


def map_png(mask_path: str):
    """Land/water PNG (row 0 = NORTH edge, so it draws upright) + its placement."""
    from PIL import Image
    d = np.load(mask_path)
    ny, nx = (int(v) for v in d['shape'])
    x0, y0 = (float(v) for v in d['origin'])
    res = float(d['res'])
    lakes = d['lakes'] if 'lakes' in d else (~np.unpackbits(d['land'], axis=1)[:, :nx].astype(bool)).astype(np.uint8)
    water = lakes > 0
    rgb = np.empty((ny, nx, 3), np.uint8)
    rgb[:] = (226, 214, 186)                      # land
    rgb[water] = (168, 205, 232)                  # water
    # shoreline: land cells touching water, drawn darker so the edge reads clearly
    edge = np.zeros_like(water)
    edge[1:, :] |= water[:-1, :]; edge[:-1, :] |= water[1:, :]
    edge[:, 1:] |= water[:, :-1]; edge[:, :-1] |= water[:, 1:]
    rgb[edge & ~water] = (150, 132, 98)
    img = Image.fromarray(rgb[::-1])              # north up
    buf = io.BytesIO()
    img.save(buf, 'PNG', optimize=True)
    names = {1: 'north lake', 2: 'rowing course', 3: 'east pond', 4: 'small pond'}
    info = {'x0': x0, 'y0': y0, 'res': res, 'w': nx, 'h': ny, 'lakes': []}
    for k in range(1, int(lakes.max()) + 1):
        ys, xs = np.nonzero(lakes == k)
        if len(xs):
            info['lakes'].append({'id': k, 'name': names.get(k, f'lake {k}'),
                                  'x': x0 + (xs.mean() + 0.5) * res, 'y': y0 + (ys.mean() + 0.5) * res})
    return buf.getvalue(), info


class ObstacleUI(Node):
    def __init__(self):
        super().__init__('obstacle_ui')
        p = self.declare_parameter
        self.host = p('host', '127.0.0.1').value
        self.port = int(p('port', 8090).value)
        self.world = p('world', 'sydney_regatta').value
        self.boat_model = p('boat_model', 'my_boat').value
        self.boat_z = float(p('boat_z', 0.4).value)
        spawner = p('spawner', '/obstacle_spawner').value
        share = get_package_share_directory('my_boat_obstacles')
        mask = p('land_mask', os.path.join(share, 'worlds', f'{self.world}_land.npz')).value
        self.page_path = os.path.join(share, 'ui', 'index.html')

        self.map_png, self.map_info = (map_png(mask) if os.path.isfile(mask) else (b'', None))
        self.state_json = '{}'
        self.tracks_json, self.tracks_seq = None, 0
        self.env_applied = None                  # {'config_file', 'yaml'} from sim_environment
        self.env_cache = (None, None)            # (yaml text, CombinedMask)
        self.r1 = {'grid': None, 'raw': None, 'png': None, 'graph': None, 'summary': None, 'version': 0}
        self.rg = {'route': None, 'route_seq': 0, 'status': None, 'status_seq': 0, 'handle': None, 'last': ''}
        self.align = MapAlignment()
        self.applied = None                       # {'config_file', 'yaml'}
        self.pending = {}                         # apply id -> [Event, result]
        self.lock = threading.Lock()

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/sim/obstacles/state', self._on_state, 10)
        self.create_subscription(String, p('tracks_topic', '/perception/tracks/json').value, self._on_tracks, 10)
        self.create_subscription(String, f'{spawner}/applied_config', self._on_applied, latched)
        self.create_subscription(String, f'{spawner}/apply_result', self._on_result, 10)
        self.pub_apply = self.create_publisher(String, f'{spawner}/apply_yaml', 10)
        self.cli = {n: self.create_client(Trigger, f'{spawner}/{n}') for n in ('run', 'reset')}
        envnode = p('environment_node', '/sim_environment').value
        self.create_subscription(String, f'{envnode}/applied_config', self._on_env_applied, latched)
        self.create_subscription(String, f'{envnode}/apply_result', self._on_result, 10)
        self.pub_env_apply = self.create_publisher(String, f'{envnode}/apply_yaml', 10)
        self.js_path = os.path.join(share, 'ui', 'env.js')
        # route planner 1's compiled map (optional: only when route_one_bringup runs)
        from nav_msgs.msg import OccupancyGrid
        self.create_subscription(OccupancyGrid, '/route_one/map/reason', self._on_r1_reason, latched)
        self.create_subscription(String, '/route_one/map/graph', lambda m: self._on_r1('graph', m), latched)
        self.create_subscription(String, '/route_one/map/summary', lambda m: self._on_r1('summary', m), latched)
        # route planner 1's route + its live status, and the action that takes goals (optional package)
        self.create_subscription(String, '/route_one/route_json', self._on_rg_route, latched)
        self.create_subscription(String, '/route_one/status_json', self._on_rg_status, 10)
        # route planner 2 (route_two_planner): its local plan + decision, and the sound signals
        self.p2 = {'json': None, 'seq': 0}
        self.create_subscription(String, '/route_two/status_json', self._on_p2, 10)
        self.rg_action = self.rg_cancel = None
        try:
            from marine_planner_msgs.action import NavigateRoute
            from rclpy.action import ActionClient
            self.NavigateRoute = NavigateRoute
            self.rg_action = ActionClient(self, NavigateRoute, p('route_action', '/route_server/navigate').value)
            self.rg_cancel = self.create_client(Trigger, '/route_server/cancel')
        except ImportError:
            self.get_logger().info('marine_planner_msgs not found: the Route goal tools are off')
        self.mask_path = mask

        node = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *a):        # quiet
                pass

            def _send(self, code, body, ctype='application/json'):
                if isinstance(body, (dict, list)):
                    body = json.dumps(body).encode()
                elif isinstance(body, str):
                    body = body.encode()
                self.send_response(code)
                self.send_header('Content-Type', ctype)
                self.send_header('Content-Length', str(len(body)))
                self.send_header('Cache-Control', 'no-store')
                self.end_headers()
                self.wfile.write(body)

            def do_GET(self):
                try:
                    node.handle_get(self)
                except (BrokenPipeError, ConnectionResetError):
                    pass

            def do_POST(self):
                n = int(self.headers.get('Content-Length', 0) or 0)
                try:
                    body = json.loads(self.rfile.read(n) or b'{}')
                except ValueError:
                    return self._send(400, {'ok': False, 'message': 'bad JSON'})
                try:
                    node.handle_post(self, body)
                except (BrokenPipeError, ConnectionResetError):
                    pass

        self.httpd = ThreadingHTTPServer((self.host, self.port), Handler)
        self.httpd.daemon_threads = True
        threading.Thread(target=self.httpd.serve_forever, daemon=True).start()
        self.get_logger().info(f'SIMU UI on http://{"localhost" if self.host in ("127.0.0.1", "0.0.0.0") else self.host}:{self.port}'
                               + ('' if self.map_info else f'  (no land map at {mask})'))

    # ------------------------------------------------------------ ROS side
    def _on_state(self, msg):
        self.state_json = msg.data
        try:
            b = json.loads(msg.data).get('boat')
        except ValueError:
            return
        if b and 't' in b:
            with self.lock:
                self.align.add_true(float(b['t']), float(b['x']), float(b['y']), float(b['heading']))

    def _on_tracks(self, msg):
        """Tracks arrive in the EKF's map frame; the page draws in Gazebo world."""
        try:
            doc = json.loads(msg.data)
            e = doc['ego']
        except (ValueError, KeyError):
            return
        with self.lock:
            self.align.add_ekf(float(doc['stamp']), float(e['x']), float(e['y']), float(e['yaw']))
            if not self.align.ready:
                return
            self.tracks_json = json.dumps(self.align.tracks_in_world(doc), separators=(',', ':'))
            self.tracks_seq += 1

    def _on_applied(self, msg):
        try:
            self.applied = json.loads(msg.data)
        except ValueError:
            pass

    def _on_rg_route(self, msg):
        with self.lock:
            self.rg['route'], self.rg['route_seq'] = msg.data, self.rg['route_seq'] + 1

    def _on_p2(self, msg):
        with self.lock:
            self.p2['json'], self.p2['seq'] = msg.data, self.p2['seq'] + 1

    def _on_rg_status(self, msg):
        with self.lock:
            self.rg['status'], self.rg['status_seq'] = msg.data, self.rg['status_seq'] + 1

    def rg_goal(self, body):
        """Send a NavigateRoute goal (the action keeps running in the route server;
        progress comes back on /route_one/status_json)."""
        if self.rg_action is None:
            return {'ok': False, 'message': 'route goals need marine_planner_msgs (route_one_planner) sourced'}
        if not self.rg_action.wait_for_server(timeout_sec=2.0):
            return {'ok': False, 'message': 'route_server is not running (ros2 launch route_one_bringup route_one.launch.py)'}
        from geometry_msgs.msg import PoseStamped
        NR = self.NavigateRoute
        g = NR.Goal()
        g.mode = NR.Goal.MODE_WAYPOINTS if body.get('mode') == 'waypoints' else NR.Goal.MODE_SINGLE
        g.berth = str(body.get('berth') or '')
        for w in body.get('waypoints') or []:
            ps = PoseStamped()
            ps.header.frame_id = 'map'
            ps.pose.position.x, ps.pose.position.y = float(w[0]), float(w[1])
            if len(w) > 2 and w[2] is not None:
                ps.pose.orientation.z, ps.pose.orientation.w = math.sin(float(w[2]) / 2), math.cos(float(w[2]) / 2)
            else:
                ps.pose.orientation.w = 0.0                      # all zero = no heading requirement
            g.waypoints.append(ps)
        if not g.waypoints and not g.berth:
            return {'ok': False, 'message': 'no goal: click the map, or pick a berth'}
        fut = self.rg_action.send_goal_async(g)

        def accepted(f):
            h = f.result()
            with self.lock:
                self.rg['handle'] = h if (h and h.accepted) else None
        fut.add_done_callback(accepted)
        what = (f"berth {g.berth}" if g.berth else
                f"{'waypoint route' if g.mode == NR.Goal.MODE_WAYPOINTS else 'goal'} with {len(g.waypoints)} pose(s)")
        return {'ok': True, 'message': f'sent {what} to the route server'}

    def rg_stop(self):
        with self.lock:
            h = self.rg['handle']
        if h is not None:
            h.cancel_goal_async()
        if self.rg_cancel is not None and self.rg_cancel.service_is_ready():
            self.rg_cancel.call_async(Trigger.Request())
        return {'ok': True, 'message': 'cancel sent'}

    def _on_r1_reason(self, msg):
        i = msg.info
        with self.lock:
            self.r1.update(grid={'x0': i.origin.position.x, 'y0': i.origin.position.y, 'res': i.resolution,
                                 'nx': i.width, 'ny': i.height, 'frame': msg.header.frame_id},
                           raw=bytes(np.frombuffer(bytes(msg.data), np.int8).astype(np.uint8)), png=None)
            self.r1['version'] += 1

    def _on_r1(self, key, msg):
        try:
            v = json.loads(msg.data)
        except ValueError:
            return
        with self.lock:
            self.r1[key] = v
            self.r1['version'] += 1

    # colour per reason code (compiler.REASONS); free / uncharted / land stay transparent:
    # the page already shows water and land -- this layer shows what the PLANNER forbids
    R1_RGBA = {2: (30, 90, 200, 150), 4: (200, 20, 20, 230), 5: (40, 40, 40, 230), 6: (245, 124, 0, 220),
               7: (211, 47, 47, 120), 8: (200, 0, 200, 200)}

    def r1_png(self):
        with self.lock:
            if self.r1['png'] is not None or self.r1['raw'] is None:
                return self.r1['png']
            g, raw = self.r1['grid'], self.r1['raw']
        from PIL import Image
        a = np.frombuffer(raw, np.uint8).reshape(g['ny'], g['nx'])
        rgba = np.zeros((g['ny'], g['nx'], 4), np.uint8)
        for code, c in self.R1_RGBA.items():
            rgba[a == code] = c
        buf = io.BytesIO()
        Image.fromarray(rgba[::-1], 'RGBA').save(buf, 'PNG', optimize=True)     # row 0 = NORTH
        with self.lock:
            self.r1['png'] = buf.getvalue()
        return self.r1['png']

    def _on_env_applied(self, msg):
        try:
            self.env_applied = json.loads(msg.data)
        except ValueError:
            pass

    def env_mask(self):
        """World land + the APPLIED environment, for the obstacle checks (cached per version)."""
        from . import landmask
        text = self.env_applied['yaml'] if self.env_applied else None
        if self.env_cache[0] == text and self.env_cache[1] is not None:
            return self.env_cache[1]
        world = landmask.LandMask(self.mask_path) if os.path.isfile(self.mask_path) else None
        terrain, polys = None, []
        if text:
            try:
                env = E.parse_environment(yaml.safe_load(text) or {})
                terrain = E.build_terrain(env, world.land_at if world else None)
                polys = E.blocking_polygons(env)
            except (ValueError, yaml.YAMLError):
                pass
        m = CombinedMask(world, terrain, polys)
        self.env_cache = (text, m)
        return m

    def _apply_env(self, text, timeout=60.0):
        rid = uuid.uuid4().hex
        ev = threading.Event()
        with self.lock:
            self.pending[rid] = [ev, None]
        self.pub_env_apply.publish(String(data=json.dumps({'id': rid, 'yaml': text})))
        ok = ev.wait(timeout)
        with self.lock:
            _, r = self.pending.pop(rid)
        if not ok:
            return {'ok': False, 'message': 'sim_environment did not answer -- is it running? '
                                            '(moving_obstacles.launch.py starts it unless environment:=none)',
                    'warnings': []}
        return r

    def vmap_view(self):
        """The APPLIED vector map in local metres + what each feature means for OUR boat."""
        text = self.env_applied.get('map') if self.env_applied else None
        if not text:
            return {'ok': False, 'message': 'the environment is not a vector map (or sim_environment is not running)'}
        try:
            from marine_vector_map import vmap
            fc = vmap.validate(json.loads(text))
        except (ImportError, ValueError) as e:
            return {'ok': False, 'message': f'vector map unavailable: {e}'}
        meta = fc['properties']
        draft = float((meta.get('environment_settings') or {}).get('boat_draft', 0.5))
        loc = vmap.to_local(fc)
        feats = []
        for f in loc['features']:
            p = dict(f['properties'])
            spec = vmap.CLASSES[p['class']]
            # only these describe the WATER depth (a net's `depth` is how deep the net hangs)
            depth = {'DEPARE': p.get('drval1'), 'CANALS': p.get('depth'), 'HRBBSN': p.get('depth'),
                     'OBSTRN': p.get('valsou')}.get(p['class'])
            if depth is not None:
                p['effective_depth'] = round(vmap.effective_depth(depth, meta), 2)
                p['navigable'] = vmap.navigable(depth, meta, draft)
            feats.append({'class': p['class'], 'kind': spec['kind'], 'label': spec['label'],
                          'properties': p, 'geometry': f['geometry']})
        return {'ok': True, 'name': meta.get('name', ''), 'file': self.env_applied['config_file'],
                'datum': meta['datum'], 'chart_datum': meta.get('chart_datum'),
                'water_level_m': meta.get('water_level_m', 0.0), 'ukc_m': meta.get('ukc_m', 0.5),
                'boat_draft': draft, 'features': feats}

    def env_check(self, doc):
        try:
            env = E.parse_environment(doc)
        except ValueError as e:
            return {'ok': False, 'error': str(e), 'warnings': []}
        from . import landmask
        world = landmask.LandMask(self.mask_path) if os.path.isfile(self.mask_path) else None
        terrain = E.build_terrain(env, world.land_at if world else None)
        warnings = E.problems(env, world.land_at if world else None, terrain)
        try:
            b = json.loads(self.state_json).get('boat')
        except ValueError:
            b = None
        if b and terrain is not None and terrain.is_solid(b['x'], b['y']):
            warnings.append({'names': [], 'text': f"my_boat ({b['x']:.0f}, {b['y']:.0f}) would be inside the terrain"})
        return {'ok': True, 'warnings': warnings, 'terrain_boxes': len(terrain.rects) if terrain else 0,
                'berths': E.berths_world(env)}

    def _on_result(self, msg):
        try:
            r = json.loads(msg.data)
        except ValueError:
            return
        with self.lock:
            slot = self.pending.get(r.get('id'))
        if slot:
            slot[1] = r
            slot[0].set()

    def _call(self, name, timeout=5.0):
        cli = self.cli[name]
        if not cli.wait_for_service(timeout_sec=2.0):
            return {'ok': False, 'message': f'obstacle_spawner/{name} is not available -- is the spawner running?'}
        fut = cli.call_async(Trigger.Request())
        ev = threading.Event()
        fut.add_done_callback(lambda _f: ev.set())
        if not ev.wait(timeout):
            return {'ok': False, 'message': f'{name}: no reply'}
        r = fut.result()
        return {'ok': bool(r.success), 'message': r.message}

    def _apply(self, text, timeout=20.0):
        rid = uuid.uuid4().hex
        ev = threading.Event()
        with self.lock:
            self.pending[rid] = [ev, None]
        self.pub_apply.publish(String(data=json.dumps({'id': rid, 'yaml': text})))
        ok = ev.wait(timeout)
        with self.lock:
            _, r = self.pending.pop(rid)
        if not ok:
            return {'ok': False, 'message': 'the spawner did not answer -- is it running?', 'warnings': []}
        return r

    def _set_ego(self, x, y, heading):
        q = f'z: {math.sin(heading / 2.0):.6f} w: {math.cos(heading / 2.0):.6f}'
        req = (f'name: "{self.boat_model}" position {{x: {x:.3f} y: {y:.3f} z: {self.boat_z:.3f}}} '
               f'orientation {{{q}}}')
        try:
            r = subprocess.run(['gz', 'service', '-s', f'/world/{self.world}/set_pose',
                                '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
                                '--timeout', '3000', '--req', req],
                               capture_output=True, text=True, timeout=10)
        except (OSError, subprocess.TimeoutExpired) as e:
            return {'ok': False, 'message': f'gz set_pose failed: {e}'}
        ok = 'true' in r.stdout
        return {'ok': ok, 'message': (f'{self.boat_model} moved to ({x:.1f}, {y:.1f})' if ok
                                      else f'gz set_pose refused: {r.stdout.strip() or r.stderr.strip()}')}

    def check(self, doc):
        """The spawner's own start-up checks, run on the draft. Never touches the sim."""
        from . import catalog, landmask
        from .scenario import parse_scenario, predicted_overlaps
        self.models()                                   # (re)register the user models
        try:
            base = os.path.dirname(os.path.realpath(self.applied['config_file'])) if self.applied else '.'
            sc = parse_scenario(yaml.safe_load(dump_config(doc)) or {}, base)
            obs = sc.build()
        except (ValueError, KeyError, TypeError, yaml.YAMLError) as e:
            return {'ok': False, 'error': str(e), 'warnings': []}
        out = []
        horizon = min(sc.duration_s, 300.0)
        for a, b, t in predicted_overlaps(obs, horizon, 0.5):
            out.append({'names': [a, b], 'text': f"'{a}' and '{b}' run into each other at t={t:.0f} s"})
        mask_path = os.path.join(get_package_share_directory('my_boat_obstacles'), 'worlds',
                                 f'{self.world}_land.npz')
        if os.path.isfile(mask_path):
            mask = self.env_mask()
            for name, kind, t, x, y in landmask.land_problems(obs, mask, min(sc.duration_s, 600.0)):
                out.append({'names': [name], 'text': (f"'{name}' starts on land" if kind == 'start' else
                                                      f"'{name}' runs ashore at t={t:.0f} s near ({x:.0f}, {y:.0f})")})
        return {'ok': True, 'warnings': out}

    def models(self):
        """The palette: built-in + user models, with size/class/colour and a group."""
        from . import catalog, user_models
        extra = []
        if self.applied:
            try:
                st = (yaml.safe_load(self.applied['yaml']) or {}).get('settings', {}) or {}
                extra = [str(x) for x in st.get('model_paths', []) or []]
            except yaml.YAMLError:
                pass
            dirs = user_models.default_dirs(self.applied['config_file'],
                                            get_package_share_directory('my_boat_obstacles'), extra)
            try:
                catalog.set_user_models(user_models.scan(dirs))
            except ValueError as e:
                self.get_logger().warn(f'models/: {e}')
        group = {'buoy': 'Buoys & marks', 'marker': 'Buoys & marks',
                 'pillar': 'Poles & structures', 'structure': 'Poles & structures', 'barge': 'Poles & structures',
                 'debris': 'Floating objects', 'vegetation': 'Floating objects', 'swimmer': 'Floating objects',
                 'boat': 'Vessels', 'ship': 'Vessels', 'kayak': 'Vessels'}
        out = []
        for name, m in sorted(catalog.all_models().items()):
            out.append({'name': name, 'l': m.size[0], 'w': m.size[1], 'h': m.size[2],
                        'class': m.default_class, 'colour': list(m.colour) if m.colour else None,
                        'kind': 'mesh' if m.mesh else m.shape,
                        'group': group.get(m.default_class, 'Other'),
                        'user': name in catalog.USER})
        return out

    # ------------------------------------------------------------ HTTP side
    def handle_get(self, h):
        path = h.path.split('?')[0]
        if path in ('/', '/index.html'):
            try:
                with open(self.page_path, 'rb') as f:
                    return h._send(200, f.read(), 'text/html; charset=utf-8')
            except OSError:
                return h._send(500, f'missing {self.page_path}', 'text/plain')
        if path == '/map.png':
            return h._send(200 if self.map_png else 404, self.map_png, 'image/png')
        if path == '/map.json':
            return h._send(200, self.map_info or {})
        if path == '/config':
            if not self.applied:
                return h._send(503, {'ok': False, 'message': 'waiting for the spawner'})
            try:
                doc = yaml.safe_load(self.applied['yaml']) or {}
            except yaml.YAMLError as e:
                return h._send(500, {'ok': False, 'message': str(e)})
            return h._send(200, {'ok': True, 'config_file': self.applied['config_file'], 'doc': doc})
        if path == '/models':
            return h._send(200, self.models())
        if path in ('/env.js', '/vmap.js', '/route1.js', '/route_goal.js'):
            try:
                with open(os.path.join(os.path.dirname(self.js_path), path[1:]), 'rb') as f:
                    return h._send(200, f.read(), 'text/javascript; charset=utf-8')
            except OSError:
                return h._send(404, '', 'text/javascript')
        if path == '/env/types':
            return h._send(200, {'types': {t: {k: {'default': v[0], 'min': v[1], 'max': v[2], 'unit': v[3], 'label': v[4]}
                                               for k, v in sc.items()} for t, sc in E.TYPES.items()},
                                 'labels': E.TYPE_LABELS, 'colours': E.DEFAULT_COLOURS,
                                 'canal': E.CANAL_DEFAULTS, 'settings': E.SETTINGS_DEFAULTS})
        if path == '/route1/berths':
            berths = []
            if self.env_applied:
                try:
                    berths = E.berths_world(E.parse_environment(yaml.safe_load(self.env_applied['yaml']) or {}))
                except (ValueError, yaml.YAMLError):
                    pass
            return h._send(200, {'berths': berths})
        if path == '/route1/info':
            with self.lock:
                r = dict(ok=self.r1['grid'] is not None, grid=self.r1['grid'], graph=self.r1['graph'],
                         summary=self.r1['summary'], version=self.r1['version'])
            if not r['ok']:
                r['message'] = 'route planner 1 not running (ros2 launch route_one_bringup route_one.launch.py)'
            return h._send(200, r)
        if path == '/route1/reason.png':
            png = self.r1_png()
            return h._send(200 if png else 404, png or b'', 'image/png')
        if path == '/route1/reason.bin':
            return h._send(200 if self.r1['raw'] else 404, self.r1['raw'] or b'', 'application/octet-stream')
        if path == '/vmap':
            return h._send(200, self.vmap_view())
        if path == '/env/config':
            if not self.env_applied:
                return h._send(503, {'ok': False, 'message': 'no environment (sim_environment not running?)'})
            try:
                doc = E.parse_environment(yaml.safe_load(self.env_applied['yaml']) or {})
            except (ValueError, yaml.YAMLError) as e:
                return h._send(500, {'ok': False, 'message': str(e)})
            return h._send(200, {'ok': True, 'config_file': self.env_applied['config_file'], 'doc': doc})
        if path == '/events':
            h.send_response(200)
            h.send_header('Content-Type', 'text/event-stream')
            h.send_header('Cache-Control', 'no-store')
            h.end_headers()
            sent, rsent, ssent, p2sent = 0, 0, 0, 0
            while True:
                h.wfile.write(f'data: {self.state_json}\n\n'.encode())
                if self.rg['route_seq'] != rsent and self.rg['route']:
                    rsent = self.rg['route_seq']
                    h.wfile.write(f"event: route\ndata: {self.rg['route']}\n\n".encode())
                if self.rg['status_seq'] != ssent and self.rg['status']:
                    ssent = self.rg['status_seq']
                    h.wfile.write(f"event: route_status\ndata: {self.rg['status']}\n\n".encode())
                if self.p2['seq'] != p2sent and self.p2['json']:
                    p2sent = self.p2['seq']
                    h.wfile.write(f"event: route2\ndata: {self.p2['json']}\n\n".encode())
                if self.tracks_seq != sent and self.tracks_json:
                    sent = self.tracks_seq
                    h.wfile.write(f'event: tracks\ndata: {self.tracks_json}\n\n'.encode())
                h.wfile.flush()
                time.sleep(0.1)
        return h._send(404, {'ok': False, 'message': 'not found'})

    def handle_post(self, h, body):
        path = h.path.split('?')[0]
        if path == '/update':
            result = {'ok': True, 'message': '', 'warnings': []}
            if 'doc' in body:
                result = self._apply(dump_config(body['doc']))
            if result.get('ok') and body.get('ego'):
                e = body['ego']
                er = self._set_ego(float(e['x']), float(e['y']), float(e['heading']))
                result['message'] = (result.get('message', '') + '; ' + er['message']).strip('; ')
                result['ok'] = er['ok']
            return h._send(200, result)
        if path == '/save':
            if not self.applied:
                return h._send(503, {'ok': False, 'message': 'waiting for the spawner'})
            target = os.path.realpath(self.applied['config_file'])     # the source, not the install link
            try:
                with open(target) as f:
                    original = f.read()
                text = merge_into_file(original, body['doc'].get('obstacles', []))
                yaml.safe_load(text)                        # never write a broken file
                shutil.copyfile(target, target + '.bak')
                with open(target, 'w') as f:
                    f.write(text)
            except (OSError, KeyError, yaml.YAMLError) as e:
                return h._send(200, {'ok': False, 'message': f'not saved: {e}'})
            return h._send(200, {'ok': True, 'message': f'saved to {os.path.basename(target)} '
                                                        f'(previous version kept as {os.path.basename(target)}.bak)'})
        if path == '/check':
            return h._send(200, self.check(body.get('doc', {})))
        if path in ('/run', '/reset'):
            return h._send(200, self._call(path[1:]))
        if path == '/ego':
            return h._send(200, self._set_ego(float(body['x']), float(body['y']), float(body['heading'])))
        if path == '/route1/goal':
            return h._send(200, self.rg_goal(body))
        if path == '/route1/cancel':
            return h._send(200, self.rg_stop())
        if path == '/env/geometry':
            st = dict(E.SETTINGS_DEFAULTS)
            st.update(body.get('settings') or {})
            out = {}
            for sdoc in body.get('structures') or []:
                try:
                    one = E.parse_environment({'settings': st, 'structures': [sdoc]})
                    out[sdoc.get('name')] = E.geometry_for_ui(one['structures'][0], one['settings'])
                except ValueError as e:
                    out[sdoc.get('name')] = {'error': str(e)}
            return h._send(200, out)
        if path == '/env/check':
            return h._send(200, self.env_check(body.get('doc') or {}))
        if path == '/env/update':
            try:
                env = E.parse_environment(body.get('doc') or {})
            except ValueError as e:
                return h._send(200, {'ok': False, 'message': f'not applied: {e}', 'warnings': []})
            r = self._apply_env(dump_environment(env))
            self.env_cache = (None, None)
            return h._send(200, r)
        if path == '/env/save':
            if not self.env_applied:
                return h._send(503, {'ok': False, 'message': 'no environment file (sim_environment not running?)'})
            # the REAL file: with --symlink-install config_file is a link in install/,
            # and the .bak must land next to the source, not in install/
            target = os.path.realpath(self.env_applied['config_file'])
            try:
                env = E.parse_environment(body.get('doc') or {})
                with open(target) as f:
                    original = f.read()
                if target.endswith(('.geojson', '.json')):
                    from marine_vector_map import vmap
                    text = vmap.dumps(vmap.merge_environment(vmap.validate(json.loads(original)), env))
                    E.parse_environment(vmap.to_environment(vmap.validate(json.loads(text))))   # never write a broken file
                else:
                    text = merge_env_file(original, env)
                    E.parse_environment(yaml.safe_load(text) or {})        # never write a broken file
                shutil.copyfile(target, target + '.bak')
                with open(target, 'w') as f:
                    f.write(text)
            except (OSError, ValueError, yaml.YAMLError) as e:
                return h._send(200, {'ok': False, 'message': f'not saved: {e}'})
            return h._send(200, {'ok': True, 'message': f'environment saved to {os.path.basename(target)} '
                                                        f'(previous version kept as {os.path.basename(target)}.bak)'})
        return h._send(404, {'ok': False, 'message': 'not found'})

    def destroy_node(self):
        self.httpd.shutdown()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleUI()
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
