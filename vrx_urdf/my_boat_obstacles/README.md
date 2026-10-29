# my_boat_obstacles

Scripted **moving** obstacles for the my_boat Gazebo sim. You write down where
each obstacle starts (world coordinates), how it moves, and what it is. It
appears in the running world as a real model and **holds still until you call
run**. The boat's own lidar and camera see it, so the detector, tracker and
planners are tested on real sensor data. The existing static field
(`my_boat_gazebo` `spawn_obstacles.launch.py`) is untouched and can run
alongside it.

## SIMU -- the browser UI

```bash
ros2 launch my_boat_obstacles moving_obstacles.launch.py     # starts the UI too
# open http://localhost:8090          (ui:=false to skip, ui_port:=..., ui_host:=0.0.0.0 for other machines)
```

**Map (left):** the whole world's land/water map (both lakes and the ponds)
with my_boat (green) and every obstacle drawn to scale at its live position,
coloured by state:

- **blue:** moving
- **grey:** waiting / still
- **red:** stopped on contact

Each obstacle's planned path is drawn too: solid for waypoints, dashed for a
straight line or a turn, with an arrow for direction. A dashed outline marks
where an obstacle will start.

- **Zoom:** mouse wheel (around the cursor) or `+` / `−`. `⌂` fits the whole scenario.
- **Pan:** drag empty water.
- **Select:** click an obstacle or my_boat (or pick one from the list).
- **Move:** drag an obstacle, and **its whole path moves with it** (Shift-drag
  moves only the start). Drag my_boat the same way.
- **Shapes:** vessels are drawn as hulls, buoys/poles/piles as circles, and
  other things as boxes. Labels that would collide are hidden until you zoom
  in; the selected one always shows.
- **Add obstacle** (the panel with nothing selected): pick a model from the
  palette (Buoys & marks · Poles & structures · Floating objects · Vessels),
  then click the map. Shift+click places several; Esc cancels. New obstacles
  start static.
- **Delete:** the bin button, or Del, removes the selected obstacle.

**Panel (right), for the selected object:**

- **Position** x / y in world metres.
- **Orientation:** drag the compass dial (N up, E right, like the map; hold
  Shift to snap to 15°), or type degrees. Degrees are in the file's
  convention: 0 = east, 90 = north. The compass bearing is shown next to it.
  It works for every obstacle. On a waypoint path, turning the compass
  **rotates the whole path around the start**, so the first leg points where
  you set it.
- **Motion:** static / straight line / constant turn / waypoint path.
- **Velocity** v in m/s (knots shown).
- **Waypoint:** press it, then click on the map to add points (numbered).
  Drag a point to move it, right-click removes the last one, and Esc or
  Enter finishes. **loop** returns to the start; **Clear path** removes all
  points. Pressing Waypoint on a straight/turn obstacle turns it into a path.
- **For a static obstacle, velocity and Waypoint are dimmed.**

**Buttons:**

| button | does |
|---|---|
| **Update** | applies every change to the running simulation, with the same checks as the file (unknown keys, land, overlaps), shown in the log below. Moves my_boat in Gazebo if you dragged it. |
| **Save** | writes the obstacles to the config file. The documented header, the settings and the trailing comments are kept, and the previous file is kept as `.bak`. Save does not apply; Update does. |
| **Clear** | throws away changes not yet applied and resets the obstacles to their start. |
| **Run / Reset** | the spawner's run / reset. |

The badges show whether the sim and the file are up to date with what you see.
A waypoint obstacle with no points, or a moving obstacle at 0 m/s, is refused
before sending.

**Live checks while you edit.** 0.7 s after each change, the UI runs the
spawner's own checks on your unapplied edits (`POST /check`): config errors,
obstacles that start on land or run ashore, and paths that run into each
other. A problem shows immediately, before Update:

- a `⚠ N warnings` badge;
- the obstacle's path turns red;
- a red note in its panel, e.g. `'essex' runs ashore at t=61 s near (-428, 136)`.

The checks take under 1 s for the 22-obstacle default scene.

Tested by driving the real page (offscreen WebKit) against the sim, checking
the results in Gazebo each time. Rotating a path with the compass, adding a
pile from the palette, and deleting a drum were all confirmed in Gazebo after
Update. Earlier checks:

- a drag moved an obstacle's start 20 m;
- the dial set the heading to north;
- three clicked waypoints plus loop became a path that the obstacle then followed at the typed 2 m/s;
- dragging my_boat 15 m moved it 15 m in Gazebo;
- Save wrote a file that loads back as the same scenario;
- Clear reverted an unapplied edit.

### Tracker overlay

When `marine_tracker` runs, the page also draws its **tracks**, received as an `event: tracks`
on `/events`. Each track has its own colour, with a box, a `T7 boat 2.1 m/s 045°` chip, a
2-sigma ellipse, a trail and the predicted path (one dot per second, and a 2-sigma circle at the
end). The tracks are in the EKF's `map` frame. `track_view.MapAlignment` maps them onto this
Gazebo-world map, using the boat's true pose (`boat.t` in the state) against its EKF pose. The
legend shows the alignment and the EKF error, with toggles for tracks, trails and tentative.
Parameter `tracks_topic` (default `/perception/tracks/json`).

## The environment: canals, docks, bridges (the vector map `maps/regatta_canals.geojson`)

Obstacles are things that move or could move. The **environment** is what doesn't: the fixed
world a route planner would read off a chart. It lives in a **vector map** (GeoJSON in lat/lon,
S-57-style classes, see `marine_vector_map` in ros2_ws). Its *physical* features are built in
Gazebo, and its *information* features (depth areas, restricted/no-go areas, speed limits,
caution areas, obstructions, and each canal's depth, traffic direction, keep-starboard,
overtaking and speed) are drawn on the SIMU map as the **Vector map** overlay (legend switch;
hover a feature for its details). Saving from the Environment tab rewrites only the physical
features and keeps the rest. A plain environment `.yaml` also still works.
`sim_environment` builds it in Gazebo;
`moving_obstacles.launch.py` starts it (`environment:=none` to skip, `environment:=<file>` for another).
Edit it in the SIMU page under the **Environment** tab, or in the file.

**Terrain, one rule:** `solid = (land regions ∪ canal banks) − (canal water ∪ basins)`

| item | what it is |
|---|---|
| land region | polygon of solid ground (reclaimed land, an island) |
| canal | centreline + water `width`; banks of `bank_width` rise on both sides where no land covers them. Ends are open unless `open_start`/`open_end: false` (a dead end). An end inside another canal is a **junction**; nothing special is needed |
| basin | rectangle of water cut out of the land (a marina) |

The terrain is rasterised at `resolution` (only where the world is water), merged into boxes,
and spawned as **one** static model.

**Structures** (`type`, `x`, `y`, `heading_deg` + the type's parameters, one static model each):
`pontoon` (walkway + finger piers; the gaps are **berths** with a stern-first docking pose),
`jetty` (piled pier), `bridge` (deck at `clearance`, abutments, `piers`), `wall`
(quay wall / breakwater), `pile_row`, `net` (visible floats, **invisible** net under the water:
the lidar can't see it, the boat can't pass it).

A part **blocks navigation** when it spans `boat_draft` … `boat_air_draft`. A bridge deck at 3 m
clearance doesn't block; its piers do.

**Published (latched, frame `world`):**
- `/sim/environment/map`: `nav_msgs/OccupancyGrid` (world land + terrain + blocking parts, 1 m).
  This is the static map for the route planner.
- `/sim/environment/berths`: JSON, every berth and its docking pose.
- `/sim/environment/markers`: RViz.

**On Update**, only what changed is rebuilt: the terrain if a land region, canal or basin changed;
otherwise just that structure. The obstacle checks on the page include the environment, so an
obstacle path that runs into a bank or a pier is flagged.

The default layout fills the free east arm of the north lake (x −290…340):

| name | what it is |
|---|---|
| main_canal | 24 m, runs from the west mouth to the east mouth |
| loop_canal | 16 m, the south **alternative** route between the two junctions |
| backwater | 12 m dead end |
| marina + marina_pontoon | basin with a pontoon dock, 5 berths |
| road_bridge | 3 m clearance, one pier |
| fuel_jetty | narrows the main canal |
| mooring_piles, fishing_net | along the loop canal; the net blocks half its width |
| entrance_breakwater | at the west mouth |

Saving into the vector map keeps the information features, the canal rules and the metadata.
With a `.yaml` environment, Save keeps the comment header and rewrites the rest.

### Route goals (route planner 1)

With `route_one_bringup` running, the **Route** toolbar on the map sends goals to the route
server, RViz-style:
- **Single goal:** press = position, drag = heading, release = sent.
- **Waypoints:** press-drag-release each, then Send; the last one is the goal.
- **go to berth…**, **Cancel**.

The planned route (coloured by speed), its corridor and the live status (state, distance, ETA,
canal, and the reason when it fails) are drawn on the map. The page talks to the
`/route_server/navigate` action through `obstacle_ui` (`/route1/goal`, `/route1/cancel`,
`/route1/berths`).

## Run

```bash
# terminal 1: world + boat
ros2 launch my_boat_gazebo my_boat_world_v2.launch.py
# terminal 2: obstacles from config/obstacles.yaml appear at their start poses and hold still
ros2 launch my_boat_obstacles moving_obstacles.launch.py

ros2 service call /obstacle_spawner/run   std_srvs/srv/Trigger   # all start moving together
ros2 service call /obstacle_spawner/reset std_srvs/srv/Trigger   # back to start poses, holding
ros2 service call /obstacle_spawner/run   std_srvs/srv/Trigger   # again = restart from the start
```

### Moving an obstacle after it has spawned

```bash
# name = scenario name ("centre_crosser") or model name ("obs_centre_crosser")
# orientation = the heading it sets off in (omit it for east; north = z 0.7071, w 0.7071)
ros2 service call /obstacle_spawner/set_start ros_gz_interfaces/srv/SetEntityPose \
  "{entity: {name: centre_crosser}, pose: {position: {x: -470.0, y: 150.0}, orientation: {z: 0.7071, w: 0.7071}}}"
```

This changes that obstacle's **start pose**. The model is not respawned: its
new path is sent straight to its plugin, so the change is immediate and
nothing flickers.

- **Holding:** it moves to the new start and waits for run.
- **Running:** it jumps onto its new path at the current run time.
- **After it:** `reset` returns it to the new start.
- **waypoints obstacles:** the whole path is shifted so it begins at (x, y),
  keeping its shape, and the heading is ignored.
- **Bad name:** returns `success: false` and logs the valid names.

The change lasts until the node restarts; the scenario file is not edited.

Ctrl-C removes the obstacles. `run` is refused until every obstacle has
spawned.

**The config file is `config/obstacles.yaml`.** It holds every obstacle
(position, heading, speed, motion, model, class, size, colour, delay, crab)
and the spawner's own `settings:`, and every option is documented inline. It
starts as the whiteboard sketch. To use a different file, pass
`config:=crossing_starboard` (a name from `config/` or `scenarios/`) or
`config:=/path/to/file.yaml`.

### The first static field is included

The five obstacles of the original `my_boat_gazebo/config/obstacles.yaml`
(`buoy_1`, `buoy_2`, `pillar_1`, `pillar_2`, `barge_1`) are in the default
config at their original positions, with the same shapes, heights and colours.
So they now get collisions, ground truth, the land check and the UI like
everything else. **Don't also run `my_boat_gazebo spawn_obstacles.launch.py`:**
it would put a second copy in exactly the same places.

### How fast obstacles move

- **Per obstacle:** `speed: 1.2` (m/s) or `speed_kn: 2.3` (knots) on the
  obstacle itself. Every moving obstacle needs one; leaving it out is an error
  that says so.
- **All at once:** `settings.speed_scale` (default 1.0) multiplies every
  obstacle's speed. The paths keep their shape: a turn keeps its radius and
  waypoints keep their corners. It is also a **live ROS parameter**:

  ```bash
  ros2 param set /obstacle_spawner speed_scale 2.0   # everything twice as fast, now
  ```

  Mid-run, each obstacle speeds up from where it is and doesn't jump (checked
  in the sim: 1.20 → 2.40 m/s with no step in position). Values ≤ 0 are
  refused. `reload` resets it to the file's value.

### Land check

`worlds/sydney_regatta_land.npz` is a 1 m land/water map of the WHOLE world
(both lakes of the Sydney International Regatta Centre, the north lake with
the spawn and the 2.2 km rowing course, plus the two ponds). Only water
enclosed by land counts; beyond the terrain's edge counts as not water. It was built
from its terrain mesh by `tools/make_land_mask.py`. At start-up and on every
reload, the spawner warns if an obstacle **starts on land** or if its path
**runs ashore**:

```
land check: 'essex' STARTS ON LAND (hull at (-441.5, 134.8)) -- move its start pose
land check: 'top_ship' runs ashore at t=100.0 s after run, hull at (-602.6, 191.1) -- ...
```

It checks the hull's centre and four corners, so a bow scraping the shore
counts. It only warns and never blocks. `settings.land_mask: none` turns it
off. For another world, run the tool with `--dae <terrain.dae> --name <world>`.

The default config was laid out against this map. Obstacles that would
otherwise run ashore after a few minutes use back-and-forth or looping
waypoint paths instead.

### Change it while the sim runs

Edit the file, then:

```bash
ros2 service call /obstacle_spawner/reload std_srvs/srv/Trigger
```

The workspace is `--symlink-install`, so the installed config is a link to
the source file and your edit is what gets read. Reload applies only what
changed:

| you changed | what happens |
|---|---|
| start, heading, speed, motion, start_delay, crab | new path sent to the plugin, no respawn |
| model, class, size, colour | that obstacle is respawned |
| added a new name | spawned |
| deleted a name | removed from Gazebo |
| run_lead_s, sample_step_s, duration_s, world_frame | used from now on |
| world, boat_model, name_prefix | warned about and ignored until restart |

A running scenario keeps running, and changed obstacles join it at the current
run time. **A file with any error is rejected as a whole and nothing changes**;
the reason is in the reply, e.g. `config NOT applied: left_kayak: unknown
model 'canoe'`. Reload also replaces any `set_start` overrides with what the
file says.

Example scenarios in `scenarios/`: `head_on`, `crossing_starboard`,
`crossing_port`, `overtaking`, `being_overtaken`, `crab_crossing`,
`delayed_start`, `static_field`, `harbour_mix`.

The encounter scenarios are designed for my_boat at its **default spawn
(−528, 186), running east at 1.4 m/s from the moment you call run**. Each
file's `# design:` line gives the CPA that produces, e.g. crossing_starboard
is a collision course 45 s after run. A test checks every one of those lines.
Start the boat moving, then call run as it passes the spawn point, or edit the
start poses for your own track.

## How it works

1. On launch the spawner creates one model per obstacle at its start pose.
   Each carries the `MovingObstacle` system plugin (`src/MovingObstacle.cc`)
   and that obstacle's path, pre-sampled every 0.5 s in "seconds since run".
2. `run` picks `t0 = now + 0.5 s` in sim time and publishes it on
   `/my_boat_obstacles/run` (ROS `Float64`, bridged to `gz.msgs.Double`).
   Every plugin listens there, so all obstacles start at the **same sim
   instant**. `reset` publishes −1 (hold the start pose). The value is resent at
   2 Hz, so a dropped message cannot leave an obstacle behind.
3. Every physics step, the plugin teleports its model to the interpolated pose
   and sets the matching velocity. The model is non-static with gravity off,
   so its collision moves with it and the boat can really hit it.
   `set_start` sends a replacement path on `/my_boat_obstacles/samples`
   (`std_msgs/String` → `gz.msgs.StringMsg`, text `"<model> tau x y yaw ..."`).
   Each plugin only takes messages that start with its own model name. The
   path is sent three times, 0.5 s apart; replacing a path is idempotent.
4. Ground truth is computed from the same paths and the same `t0`. In the
   integration test it matched Gazebo's own model pose to within 6 mm.

Obstacles are kinematic: they hold their hull at the floating height and do
not heave on waves.

## Config file format

(`config/obstacles.yaml` has all of this written out with comments.)

```yaml
scenario:
  name: my_test
settings:                    # all optional; defaults shown
  world: sydney_regatta
  boat_model: my_boat
  name_prefix: obs
  world_frame: world
  run_lead_s: 0.5
  sample_step_s: 0.5
  duration_s: 600            # paths play this long after run, then hold
obstacles:
  - name: crosser
    model: brazilian         # what it looks like, see the table below
    class: boat              # optional label; defaults from the model
    vessel_class: power_driven   # COLREGs class, ground truth only
    start: {x: -465.0, y: 141.0, heading_deg: 90.0}   # WORLD coordinates
    start_delay: 0.0         # optional: wait this long after run before moving
    crab_deg: 0.0            # optional: bow points course + crab
    motion: {type: constant_velocity, speed: 1.0}     # or speed_kn: 1.9
  - name: big_buoy
    model: buoy
    size: [2.0, 2.0, 3.0]    # optional, primitives only (kayak buoy swimmer barge pillar)
    colour: [0.1, 0.8, 0.1]  # optional, primitives only
    start: {x: -500.0, y: 180.0}
```

A resized primitive keeps its default submerged depth, so it sits in the water
the same way. Meshes can't be resized (it's an error).

World frame = Gazebo ENU: x east, y north, `heading_deg` 0 = east, 90 = north.
For a moving obstacle, `heading_deg` is the direction it sets off in.

**motion**
- `{type: static}` (or leave `motion` out): buoy, moored boat
- `{type: constant_velocity, speed: 1.0}`: straight line along `heading_deg`
- `{type: constant_turn, speed: 0.8, turn_rate_deg_s: 3.0}`: + turns to port (CCW)
- `{type: waypoints, speed: 0.6, loop: true, points: [[x, y], ...]}`: world
  points. It starts at the first point, or at `start` if one is given.

Unknown keys, models, classes and vessel classes are **errors**, not silently
ignored.

## Collision

Every obstacle has a collision shape, so the boat physically hits it. This
was checked in the sim both ways:

- **Obstacle into boat:** a brazilian driven into my_boat's side shoved it
  3.5 m and spun it 64°.
- **Boat into obstacle:** my_boat under 400 N of thrust stopped dead against
  a barge, bow exactly on the barge's face, and stayed pinned there.

**Obstacle vs obstacle is different.** Obstacles follow scripted paths and
are moved kinematically, so physics can't push one off its path. Without
extra handling, two of them drive straight through each other. That is what
`on_contact` handles (`settings.on_contact`, overridable per obstacle):

- **`stop`** (default): an obstacle that touches another obstacle, or my_boat,
  halts where it touched and the collision is reported. `run` and `reset`
  put every stopped obstacle back on its full path.
- **`pass`:** it keeps to its path and goes through. The contact is still
  reported.

Verified in the sim: two boats set to collide both halted at the instant they
touched. A pair set to `pass` drove through each other. A rammer stopped the
moment it touched my_boat instead of shoving it along.

At start-up and on every reload, the spawner also checks the scripted paths
and warns about any pair that will run into each other, e.g.
`scenario check: 'red_a' and 'blue_b' would overlap at t=10.8 s after run --
both stop there (on_contact)`. The original default config had exactly this
problem: `centre_crosser` drove through `top_ship` at t ≈ 26–31 s. Its start
was moved so it now passes 10.8 m astern.

Per obstacle (or per model in its `model.yaml`), `collision:` is one of:

- **`box`** (default): a box around the hull. Fast and stable.
- **`mesh`:** the exact mesh (meshes only). Slower; use a low-poly mesh.
- **`none`:** a ghost. The sensors see it, but the boat passes through.

Every hit is also **reported**, so a run can be scored as hit or no hit:

- a warning in the log: `COLLISION #1: my_boat hit 'rammer' (brazilian) at t=6.9 s after run`
- a `std_msgs/String` on `/sim/obstacles/collisions`: `START rammer t=6.9`, then `END rammer t=10.6`

The report tests my_boat's collision box (`settings.boat_box_*`, from its
URDF) against each obstacle's box. A hit only ends after
`collision_release_s` (1 s) with no contact. Without that, one shove produced
six start/end pairs, because the boat bounces along the hull while it is being
pushed. For `collision: mesh` the report uses the mesh's bounding box, so it
can over-report slightly at pointed ends.

## Your own 3-D models, colours and textures

Drop a folder with a mesh and a `model.yaml` into `models/`, then use the
folder name as `model:`. It's picked up on the next reload with no rebuild;
**`models/README.md` has the full guide.** There are two worked examples:
`shipping_container` (textured `.obj`) and `channel_marker` (a coloured
`.stl` exported Y-up in centimetres).

- **Measured automatically:** STL, OBJ, GLB/glTF and PLY are measured with
  trimesh, so the ground truth box matches the mesh. DAE and FBX need `size:`
  in their `model.yaml`.
- **`colour: [r, g, b]`:** works on every model, including the built-in boats.
  Without it, the built-in boats use the pale grey/white of their vrx_gz
  `model.sdf`, which looks uncoloured; the default config colours them red
  and blue.
- **`length:`** in `model.yaml` sets a model's real length and works out the
  scale. Use it for meshes in unknown units (Sketchfab downloads, 3-D-print
  files).
- **`matte: true`** in `model.yaml` (glTF/GLB) uses a copy with metalness 0,
  written next to the original, which is never changed.
- **`texture: image.png`:** works on primitives and on meshes with texture
  coordinates (OBJ, GLB/glTF, DAE). **Not STL**, which has none, and asking
  for it is an error that says so.
- **A mesh's own materials** (`.glb`, `.obj` + `.mtl`) are kept unless you set
  colour or texture.

Two Gazebo behaviours that cost time (both found while testing this):

- **Gazebo caches every mesh by file path until the sim restarts.** If you
  change a mesh file but keep its name, reload won't show the change. Restart
  Gazebo, or save the mesh under a new name.
- **A GLB with the glTF default `metallic = 1` renders BLACK**, because there
  is no environment map for a metal to reflect. The spawner warns about this
  at start: `material is metallic (1) ... renders a metallic surface BLACK`.
  Export with metallic 0, or set `colour`/`texture`.

## Models

| model | looks like | size l×w×h (m) | default class |
|---|---|---|---|
| `brazilian` | `vrx_gz/models/brazilian` mesh | 5.06 × 2.03 × 1.52 | boat |
| `bass_boat` | `vrx_gz/models/bass_boat` mesh | 8.15 × 2.90 × 2.11 | boat |
| `freedom_365` | `vrx_gz/models/freedom_365` mesh | 10.22 × 3.79 × 4.02 | ship |
| `kayak` | box | 4.0 × 0.8 × 0.6 | kayak |
| `buoy` | cylinder | Ø0.8 × 1.5 | buoy |
| `swimmer` | box | 0.6 × 0.5 × 0.5 | swimmer |
| `barge` | box | 8.0 × 3.0 × 3.0 | barge |
| `pillar` | cylinder | Ø1.0 × 5.0 | pillar |
| `obstacle_buoy` / `obstacle_pillar` / `obstacle_barge` | the FIRST static field's shapes (`my_boat_gazebo/models/obstacles/obstacle_*.sdf`): yellow Ø0.9 × 2.4, orange Ø1.2 × 5.0, grey 8 × 3 × 3. Same height against the waterline, checked by a test | | buoy / pillar / barge |
| `marker_buoy_red/green/black/white` | VRX marker buoy (Fuel cache) | Ø0.65 × 1.18 (0.9 above water) | buoy |
| `round_buoy_orange/black` | VRX ball buoy (Fuel cache) | Ø0.5 (0.3 above water) | buoy |
| `mooring_post` | VRX post / bollard (Fuel cache) | Ø0.65 × 1.1 | pillar |
| `pile` | pole with a yellow top band | Ø0.4 × 5.0 (3 m above water) | pillar |
| `oil_drum` | 200 L drum lying on its side | 0.9 × 0.6 (0.2 m freeboard) | debris |
| `floating_log` | timber | 5.0 × 0.5 (0.15 m freeboard) | debris |
| `channel_marker`, `shipping_container`, `frickies_yacht`, `jarethcruz38819hs-essex` | see models/README.md | | |
| *your folders in `models/`* | your mesh | measured | from `model.yaml` |

The vessel meshes have their long axis on mesh +Y with the **bow at −Y**
(measured from the STL files: that end is 25–45 % narrower and its keel rises).
The visual is therefore yawed +90° and centred on the hull bounding box, so
the model's +X is the bow and the model pose is the hull centre.

Every model carries gz-sim's **Label** system with the class id
(`catalog.CLASS_IDS`: boat 1, ship 2, kayak 3, buoy 4, swimmer 5, barge 6,
pillar 7, marker 8, debris 9, structure 10). A Gazebo segmentation or bounding-box camera will report these
classes.

## Topics (ground truth; your sensors do the real detecting)

| topic | type | content |
|---|---|---|
| `/sim/obstacles/ground_truth` | `vision_msgs/Detection3DArray` | world frame, `id` = model name, class, hull box |
| `/sim/obstacles/contacts` | `marine_planner_msgs/ContactSet` | adds true velocity + vessel class. Only when the `dynamic_positioning_ai` workspace is sourced |
| `/sim/obstacles/markers` | `visualization_msgs/MarkerArray` | labels + velocity arrows |

These are in Gazebo **world** coordinates (`world_frame:=`, default `world`),
not the EKF `map` frame. Before run they show the start poses with speed 0.
After a run the spawner logs each obstacle's closest approach to the boat,
e.g. `CPA 'crosser': 3.1 m centre-to-centre at t=45.2 s after run`. It reads
the boat's true pose from `/world/<world>/dynamic_pose/info` with
`gz topic --json-output`. It can't use ros_gz_bridge for that: its
`Pose_V → TFMessage` conversion drops the model names and stamps, and the
boat's own PosePublisher topic arrives empty.

## Tests

`python3 -m pytest test` (69 tests) covers:

- holding still before run, start_delay, and set_start (including waypoint shifting)
- the four motion models
- every shipped scenario's `# design:` CPA
- sampling and playback error on turns
- the mesh orientation
- SDF generation for every shipped scenario
- the config file: settings validation, knots, size/colour overrides, respawn-vs-new-path
- user models: measuring, scaling, axis rotation, and broken-folder errors
- colours and textures (including the STL refusal), collision modes, the
  separating-axis hit test, and the metallic-GLB warning
- stop-on-contact and the scripted-path overlap check (including that the
  default config has none)
- speed (top-level, knots, speed_scale, continuity mid-run), length, matte
- the land check (and that the default config stays in the water)
- the gz JSON pose parser

User models need `trimesh` (`pip install trimesh`; it is already installed here).
