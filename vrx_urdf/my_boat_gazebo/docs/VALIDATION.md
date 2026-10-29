# my_boat — Build & Validation

## Build

Place both packages in your VRX workspace `src/` (they already live under
`src/vrx/vrx_urdf/`) and build:

```bash
cd <VRX_WS>           # e.g. ~/kavinesh/gazebo_vfx/vrx_garden_ws
GZ_VERSION=garden colcon build --packages-select my_boat_description my_boat_gazebo
source install/setup.bash
```

`my_boat_gazebo` depends on `my_boat_description` (the body) and on `vrx_gz`
(which provides `libSurface.so`, `libSimpleHydrodynamics.so`, `libUSVWind.so`
and `libPublisherPlugin.so`).

## Two ways to run

### A. Standalone quick test (calm/wavy water + wind, no full VRX stack)

The bundled world embeds the boat with all plugins attached:

```bash
gz sim -r $(ros2 pkg prefix my_boat_gazebo)/share/my_boat_gazebo/worlds/examples/my_boat_test.sdf
```

Good for Tests 1, 2, 3, 4, 5, 6 and 7. (For a perfectly calm surface during
Tests 1–3/6/7, set the wavefield `gain` to 0.0 in that world.)

### B. Into the real VRX world (rides the same wavefield as WAM-V — Test 4)

1. **Wind:** paste the `<wind_obj>` block from
   `worlds/examples/wind_obj_snippet.sdf` into the existing `vrx::USVWind`
   plugin in your VRX world (e.g. `vrx_gz/worlds/sydney_regatta.sdf`).
2. **Run:**
   ```bash
   # Terminal 1 — start VRX
   ros2 launch vrx_gz competition.launch.py world:=sydney_regatta
   # Terminal 2 — spawn the boat into the running world
   source install/setup.bash
   ros2 launch my_boat_gazebo spawn_my_boat.launch.py z:=1.5
   ```
   Keep `name == namespace == my_boat` so the link `my_boat/base_link` matches
   the world's `<wind_obj>`.

---

## Validation checklist

| # | Test | Procedure | Pass criterion |
|---|---|---|---|
| 1 | **Spawns successfully** | Run world A, or launch B | Hull mesh appears, no plugin-load errors in console |
| 2 | **Remains above water** | Spawn at `z:=1.5`, press play | Boat drops, hits water, does NOT sink; reaches a steady draft |
| 3 | **Stable equilibrium** | Watch ~15–30 s on calm water (wave gain 0) | Comes to rest upright, level, ~5–15 cm draft; no drift on calm/no-wind |
| 4 | **Responds to waves** | World B (or A with wave gain > 0) | Visible heave; gentle roll and pitch tracking the wave field |
| 5 | **Responds to wind** | `wind_mean_velocity > 0`, play | Boat drifts/weathervanes toward the wind direction |
| 6 | **Responds to current** | Apply the current wrench (helper below) | Boat drifts along the current; drift speed limited by drag |
| 7 | **10-minute stability** | Let world A run 10 min at RTF ≈ 1 | Pose stays bounded; no NaN/explosion; still upright |

### Helpers

**Frame / CG / alignment** (also see `my_boat_description/docs/FRAME_AND_CG.md` §10):
```bash
# Pose of the model (CG-centred base_link)
gz topic -e -t /model/my_boat/pose
```
In the GUI enable *View → Center of Mass*, *Inertia*, *Collisions* and confirm:
CoM marker at the base_link origin; hull mesh surrounds it (bow along +X);
collision box envelops the lower hull; inertia ellipsoid's long axis along X.

**Test 2/3 draft check** — watch the model's world Z settle:
```bash
gz topic -e -t /model/my_boat/pose
```

**Test 4 disturbance decay** — apply an upward impulse and watch it settle:
```bash
gz topic -t /world/my_boat_test/wrench -m gz.msgs.EntityWrench \
  -p 'entity:{name:"my_boat", type:MODEL}, wrench:{force:{z:3000}}'
```

**Test 5 wind active**:
```bash
ros2 topic echo /vrx/debug/wind/speed
ros2 topic echo /vrx/debug/wind/direction
```

**Test 6 ocean current** (emulated, see `config/current.yaml`) — apply a steady
force equal to the drag a current would exert (e.g. 0.5 m/s ⇒ xU·Uc ≈ 75 N along +X):
```bash
gz topic -t /world/my_boat_test/wrench -m gz.msgs.EntityWrench \
  -p 'entity:{name:"my_boat", type:MODEL}, wrench:{force:{x:75}}'
```
The boat should drift along the force and reach a steady drift speed as
hydrodynamic drag balances it.

**Test 7 soak** — run for 10 min and confirm no NaNs:
```bash
gz sim -r -s <world>           # headless server is fine for the soak
gz topic -e -t /model/my_boat/pose | head      # values stay finite & bounded
```

## Tuning notes

* Settles too low / high → adjust `fluid_density` or `hull_radius` in
  `config/buoyancy.yaml`.
* Rolls/pitches too freely or rings too long → raise `kP`/`mQ`/`zW` (and the
  matching quadratic terms) in `config/hydrodynamics.yaml`.
* Capsizes or lists → see the **stability note** in
  `my_boat_description/docs/FRAME_AND_CG.md` §9 (user-overridden CG height);
  widen `point_y_offset` in `buoyancy.yaml` for more roll stiffness, or correct
  `cog_y` if the CG datum was keel-referenced.

All coefficients live in `config/*.yaml` and are applied automatically by the
launch file (standalone world has them inline).
