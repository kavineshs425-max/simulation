# log_gazebo — Build & Validation

## Build

Place both packages in your VRX workspace `src/` and build:

```
cd <VRX_WS>
GZ_VERSION=garden colcon build --packages-select log_description log_gazebo
source install/setup.bash
```

`log_gazebo` depends on `log_description` (the body) and on `vrx_gz` (which
provides `libSurface.so`, `libSimpleHydrodynamics.so` and `libUSVWind.so`).

## Two ways to run

### A. Standalone quick test (calm water + wind, no full VRX stack)

The bundled world embeds the log with all plugins already attached:

```
gz sim -r $(ros2 pkg prefix log_gazebo)/share/log_gazebo/worlds/examples/floating_log_test.sdf
```

Good for Tests 1, 2, 3, 4, 6 and 7.

### B. Into the real VRX world (required for wave interaction, Test 5)

Wind first: add the log to the world's USVWind plugin using
`worlds/examples/wind_obj_snippet.sdf` (paste the `<wind_obj>` block into the
existing `vrx::USVWind` plugin in your VRX world). Then:

```
# Terminal 1 — start VRX
ros2 launch vrx_gz competition.launch.py world:=sydney_regatta

# Terminal 2 — spawn the log into the running world
source install/setup.bash
ros2 launch log_gazebo spawn_floating_log.launch.py z:=1.0
```

The log subscribes to the same `/vrx/wavefield/parameters` topic as the WAM-V,
so it rides the identical wavefield.

## Validation checklist

| # | Test | Procedure | Pass criterion |
|---|---|---|---|
| 1 | Spawns | Launch world A, or run launch B | Cylinder appears, no console errors |
| 2 | Falls and floats | Spawn at `z:=1.0` and press play | Log drops, hits water, does not sink |
| 3 | Settles to equilibrium | Watch for ~15 s | Comes to rest ~1/3 submerged, axis level with surface |
| 4 | Disturbance decays | Push it: `gz topic -t /world/<world>/wrench ...` or nudge in GUI | Oscillation visibly decays to rest within seconds |
| 5 | Responds to waves | Run in VRX world (B) with waves on | Log heaves with crests/troughs, gentle pitch and roll |
| 6 | Responds to wind | Set `wind_mean_velocity > 0`, play | Log drifts/yaws in the wind direction |
| 7 | 5-minute stability | Let world A run 5 min at RTF ~1 | Pose stays bounded, no NaN, no explosion |

### Helpers

Submerged-depth check (height of the log centre relative to z = 0):

```
gz topic -e -t /model/floating_log/pose
```

Confirm wind is active:

```
ros2 topic echo /vrx/debug/wind/speed
ros2 topic echo /vrx/debug/wind/direction
```

Apply a manual disturbance for Test 4 (example: an upward impulse):

```
gz topic -t /world/floating_log_test/wrench -m gz.msgs.EntityWrench \
  -p 'entity:{name:"floating_log", type:MODEL}, wrench:{force:{z:400}}'
```

## Tuning notes

If the log settles too high or too low, adjust `fluid_density` in
`config/buoyancy.yaml`. If a disturbance rings too long, raise `zW`/`mQ`; if it
looks over-damped or sluggish, lower them. All coefficients live in
`config/*.yaml` and are applied automatically by the launch file.
