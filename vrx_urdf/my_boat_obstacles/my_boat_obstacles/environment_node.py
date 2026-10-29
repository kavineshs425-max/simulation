"""sim_environment -- puts the fixed environment (environment.py) into Gazebo.

    ros2 launch my_boat_obstacles moving_obstacles.launch.py      # starts this too
    (environment:=none to skip it, environment:=<file or name> for another)

Spawns ONE static model for the terrain (land + canal banks, cut by canal
water and basins) and one static model per structure. On a change (the SIMU
page's Update, or ~/reload) only what changed is replaced: the terrain when a
land region / canal / basin changed, a structure when its own entry changed.

Publishes (latched, frame = settings.world_frame):
  /sim/environment/map      nav_msgs/OccupancyGrid  world land + terrain + every part that
                            blocks navigation (bridge piers yes, a 3 m-high deck no), on the
                            world land mask's 1 m grid -- the STATIC map a route planner reads
  /sim/environment/markers  visualization_msgs/MarkerArray  RViz
  /sim/environment/berths   std_msgs/String (JSON)   every pontoon berth + its docking pose
  ~/applied_config          std_msgs/String (JSON)   {config_file, yaml}: what is running
Talks to the page through ~/apply_yaml -> ~/apply_result, like obstacle_spawner.

The config file is a VECTOR MAP (.geojson, marine_vector_map): its PHYSICAL
features are built here, its information features (depth, restricted areas,
speed limits, rules) only travel on in ~/applied_config for the page and the
planners. A plain environment .yaml (physical features only) still works.
"""

import array
import json
import math
import os
import subprocess
import tempfile
import threading

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from geometry_msgs.msg import Point, Quaternion, Vector3
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import ColorRGBA, String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

from . import environment as E
from .landmask import LandMask


def yaw_q(yaw):
    return Quaternion(x=0.0, y=0.0, z=math.sin(yaw / 2), w=math.cos(yaw / 2))


class EnvironmentNode(Node):
    def __init__(self):
        super().__init__('sim_environment')
        self.config_file = self.declare_parameter('config_file', '').value
        if not self.config_file:
            raise RuntimeError('sim_environment: parameter config_file is required')
        self.share = get_package_share_directory('my_boat_obstacles')
        self.create_exe = os.path.join(get_package_prefix('ros_gz_sim'), 'lib', 'ros_gz_sim', 'create')
        self.cache = os.path.join(tempfile.gettempdir(), f'my_boat_env_{os.getuid()}')
        os.makedirs(self.cache, exist_ok=True)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_map = self.create_publisher(OccupancyGrid, '/sim/environment/map', latched)
        self.pub_mk = self.create_publisher(MarkerArray, '/sim/environment/markers', latched)
        self.pub_berths = self.create_publisher(String, '/sim/environment/berths', latched)
        self.pub_applied = self.create_publisher(String, '~/applied_config', latched)
        self.pub_result = self.create_publisher(String, '~/apply_result', 10)
        self.create_subscription(String, '~/apply_yaml', self._on_apply_yaml, 10)
        self.create_service(Trigger, '~/reload', self._on_reload)
        self.lock = threading.Lock()
        self.live = {}                  # item name -> gz entity name (with its fingerprint)
        self.env = None
        self.mask = None
        self.map_text = None
        ok, msg, _ = self._apply(self._read_config())
        if ok:                  # two call sites: rclpy fixes a call site's log severity on first use
            self.get_logger().info(msg)
        else:
            self.get_logger().error(msg)

    def _read_config(self):
        """-> environment YAML text. A vector map is converted (its physical part)."""
        with open(self.config_file) as f:
            text = f.read()
        if self.config_file.endswith(('.geojson', '.json')):
            from marine_vector_map import vmap
            fc = vmap.validate(json.loads(text))
            self.map_text = text
            return yaml.safe_dump(vmap.to_environment(fc), sort_keys=False)
        return text

    # ------------------------------------------------------------ gz
    def _world_mask(self, world):
        if self.mask is None or self.mask[0] != world:
            p = os.path.join(self.share, 'worlds', f'{world}_land.npz')
            self.mask = (world, LandMask(p) if os.path.isfile(p) else None)
        return self.mask[1]

    def _spawn(self, world, entity, sdf):
        path = os.path.join(self.cache, f'{entity}.sdf')
        with open(path, 'w') as f:
            f.write(sdf)
        r = subprocess.run([self.create_exe, '-world', world, '-name', entity, '-file', path],
                           capture_output=True, text=True, timeout=60)
        if r.returncode != 0:
            raise RuntimeError(f'spawn of {entity} failed: {(r.stderr or r.stdout)[-300:]}')

    def _remove(self, world, entity):
        subprocess.run(['gz', 'service', '-s', f'/world/{world}/remove', '--reqtype', 'gz.msgs.Entity',
                        '--reptype', 'gz.msgs.Boolean', '--timeout', '3000',
                        '--req', f'name: "{entity}" type: MODEL'], capture_output=True, timeout=10)

    # ------------------------------------------------------------ apply
    def _apply(self, text):
        """-> (ok, message, warnings). A bad file changes NOTHING."""
        try:
            env = E.parse_environment(yaml.safe_load(text) or {})
        except (ValueError, yaml.YAMLError) as e:
            return False, f'environment not applied: {e}', []
        with self.lock:
            st = env['settings']
            world = st['world']
            mask = self._world_mask(world)
            wl = mask.land_at if mask else None
            terrain = E.build_terrain(env, wl)
            want = {}
            if terrain is not None and terrain.rects:
                want['__terrain__'] = (f'env_terrain_{E.terrain_key(env)}', lambda: E.terrain_sdf(terrain, st, want['__terrain__'][0]))
            for s in env['structures']:
                key = E.fingerprint({'s': s, 'st': {k: st[k] for k in ('bank_depth', 'boat_length', 'dock_gap')}})
                ent = f"env_{s['name']}_{key}"
                want[s['name']] = (ent, (lambda s=s, ent=ent: E.structure_sdf(s, st, ent)))
            removed, added, errors = 0, 0, []
            for name, ent in list(self.live.items()):
                if name not in want or want[name][0] != ent:
                    self._remove(world, ent)
                    del self.live[name]
                    removed += 1
            for name, (ent, make) in want.items():
                if self.live.get(name) == ent:
                    continue
                try:
                    self._spawn(world, ent, make())
                    self.live[name] = ent
                    added += 1
                except (RuntimeError, subprocess.TimeoutExpired, OSError) as e:
                    errors.append(str(e))
            self.env, self.terrain = env, terrain
            self._publish(env, terrain, mask)
            self.pub_applied.publish(String(data=json.dumps({'config_file': self.config_file, 'yaml': text,
                                                             'map': self.map_text})))
        warnings = [w['text'] for w in E.problems(env, wl, terrain)]
        n = len(env['structures'])
        msg = (f"environment: {len(env['canals'])} canals, {len(env['basins'])} basins, "
               f"{len(env['land'])} land regions ({len(terrain.rects) if terrain else 0} terrain boxes), "
               f'{n} structures; {added} (re)spawned, {removed} removed')
        if errors:
            return False, msg + ' -- ERRORS: ' + '; '.join(errors), warnings
        return True, msg, warnings

    def _on_apply_yaml(self, msg):
        try:
            req = json.loads(msg.data)
        except ValueError:
            return
        ok, text, warnings = self._apply(req.get('yaml', ''))
        self.pub_result.publish(String(data=json.dumps({'id': req.get('id'), 'ok': ok, 'message': text,
                                                        'warnings': warnings})))

    def _on_reload(self, _req, resp):
        try:
            resp.success, resp.message, _ = self._apply(self._read_config())
        except (OSError, ValueError) as e:
            resp.success, resp.message = False, str(e)
        return resp

    # ------------------------------------------------------------ outputs
    def _publish(self, env, terrain, mask):
        st = env['settings']
        frame = st['world_frame']
        stamp = self.get_clock().now().to_msg()
        # --- occupancy on the world mask's grid (or the terrain's own when there is no mask)
        if mask is not None:
            x0, y0, res, shape, wl = mask.x0, mask.y0, mask.res, mask.land.shape, mask.land
        elif terrain is not None:
            x0, y0, res, shape, wl = terrain.x0, terrain.y0, terrain.res, terrain.solid.shape, None
        else:
            x0 = y0 = 0.0
            res, shape, wl = 1.0, (1, 1), None
        occ = E.occupancy(env, terrain, x0, y0, res, shape, wl)
        g = OccupancyGrid()
        g.header.frame_id, g.header.stamp = frame, stamp
        g.info.resolution = float(res)
        g.info.width, g.info.height = int(shape[1]), int(shape[0])
        g.info.origin.position.x, g.info.origin.position.y = float(x0), float(y0)
        g.info.origin.orientation.w = 1.0
        g.data = array.array('b', (occ.astype(np.int8) * 100).tobytes())
        self.pub_map.publish(g)
        # --- berths
        self.pub_berths.publish(String(data=json.dumps({'frame': frame, 'berths': E.berths_world(env)})))
        # --- markers
        ma = MarkerArray()
        ma.markers.append(Marker(header=g.header, action=Marker.DELETEALL))
        tc = st['terrain_colour']
        z0, z1 = -st['bank_depth'], st['bank_height']
        if terrain is not None:
            for k, (a, b, c, d) in enumerate(terrain.rects):
                m = Marker(header=g.header, ns='terrain', id=k, type=Marker.CUBE, action=Marker.ADD)
                m.pose.position = Point(x=0.5 * (a + c), y=0.5 * (b + d), z=0.5 * (z0 + z1))
                m.pose.orientation.w = 1.0
                m.scale = Vector3(x=c - a, y=d - b, z=z1 - z0)
                m.color = ColorRGBA(r=tc[0], g=tc[1], b=tc[2], a=1.0)
                ma.markers.append(m)
        k = 0
        for s in env['structures']:
            parts, _ = E.structure_parts(s, st)
            h = math.radians(s['heading_deg'])
            for p in parts:
                if not p.visual and not p.collide:
                    continue
                x, y = E.to_world(s, p.x, p.y)
                m = Marker(header=g.header, ns=f"structure/{s['name']}", id=k, action=Marker.ADD,
                           type=Marker.CYLINDER if p.kind == 'cyl' else Marker.CUBE)
                m.pose.position = Point(x=x, y=y, z=0.5 * (p.z0 + p.z1))
                m.pose.orientation = yaw_q(h + p.yaw)
                m.scale = Vector3(x=p.l, y=p.l if p.kind == 'cyl' else max(p.w, 0.05), z=p.z1 - p.z0)
                c = p.colour or (0.9, 0.1, 0.1)
                m.color = ColorRGBA(r=c[0], g=c[1], b=c[2], a=1.0 if p.visual else 0.35)   # the hidden net: see-through
                ma.markers.append(m)
                k += 1
            t = Marker(header=g.header, ns='labels', id=k, type=Marker.TEXT_VIEW_FACING, action=Marker.ADD)
            t.pose.position = Point(x=s['x'], y=s['y'], z=4.0)
            t.pose.orientation.w = 1.0
            t.scale.z = 1.6
            t.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.9)
            t.text = s['name']
            ma.markers.append(t)
            k += 1
        for b in E.berths_world(env):
            a = Marker(header=g.header, ns='berths', id=k, type=Marker.ARROW, action=Marker.ADD)
            a.pose.position = Point(x=b['x'], y=b['y'], z=0.3)
            a.pose.orientation = yaw_q(b['heading'])
            a.scale = Vector3(x=3.0, y=0.4, z=0.4)
            a.color = ColorRGBA(r=0.1, g=0.9, b=0.4, a=0.9)
            ma.markers.append(a)
            k += 1
        self.pub_mk.publish(ma)

    def destroy_node(self):
        if self.env:
            for ent in self.live.values():
                try:
                    self._remove(self.env['settings']['world'], ent)
                except Exception:
                    pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = EnvironmentNode()
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
