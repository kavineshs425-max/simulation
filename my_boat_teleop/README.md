# my_boat_teleop

Keyboard teleoperation for the `my_boat` USV. Publishes thrust (N) and steering
(rad) commands to the thruster topics so you can drive the boat by hand.

There are two control modes, selected with the `control_mode` parameter.

## Actuator mode (`control_mode:=actuator`, default)

One key drives one actuator directly.

| Key | Command | Effect |
|-----|---------|--------|
| `w` / `W` | T3 thrust +/− | stern drive forward / reverse |
| `a` / `A` | T1 thrust +/− | port bow thruster |
| `d` / `D` | T2 thrust +/− | starboard bow thruster |
| `s` / `S` | T3 steering +/− | azimuth angle (radians) |
| `space` | all-stop | zero every command |
| `q` | quit | (Ctrl-C also works) |

## Direction mode (`control_mode:=direction`)

Keys are driving **intent** (`wasd`). Forward/reverse drive the stern
propeller; left/right drive the steering angle, and the bow thrusters track
that angle to assist the turn.

| Key | Command | Effect |
|-----|---------|--------|
| `w` | forward | stern propeller thrust + |
| `s` | reverse | stern propeller thrust − |
| `a` | turn left | steering −, `T1` −, `T2` − |
| `d` | turn right | steering +, `T1` +, `T2` + |
| `space` | all-stop | zero every command |
| `q` | quit | (Ctrl-C also works) |

The bow thrusters are derived from the steering angle with a linear `map()`:
the steering domain `[−steer_max, +steer_max]` is mapped onto the bow thrust
range, and **`T2 = T1`** — the same sign, not the mirror.

That used to be `T2 = −T1`, which was right only while the bow pair was mounted
**anti-parallel**. Both units are now **starboard-positive** (mount yaw −π/2 on
both, matching the allocator's `t1_dir = t2_dir = −1`). With the pair parallel
*and* at the same longitudinal station `x = +1.03 m`, opposite signs cancel
exactly — `Fy = −T1 − T2 = 0` and `Mz = −l1·(T1+T2) = 0` — so the old code
contributed precisely nothing to a turn. Same sign gives a bow side-force, which
is what assisting the turn actually means.

Because T1 and T2 have mirrored limits, direction mode drives the pair over
their **common** range (±39.2266 N with the defaults) rather than either unit's
full range; otherwise one would saturate before the other and the "balanced"
pair would yaw off-axis. Actuator mode still gives each unit its full range.

**Feel:** one tap = **1 unit** (1 N of thrust, 1° of steering). **Press and
hold** to accelerate — the longer a key auto-repeats, the bigger each step (up
to `accel_max`×), so you ramp to large values fast but keep 1-unit precision on
taps. **Zero detent:** ramping the opposite way **stops at 0** (and resets the
acceleration) instead of overshooting, so you can always settle to exactly zero;
press again from 0 to go negative. `space` zeroes everything instantly.

A tap really is only **1 N**, and `hold_reset_ms` is 150 ms, so any gap longer
than that resets the ramp and the next tap is worth 1 N again. Measured: three
taps 400 ms apart = **3 N**, which a 420 kg hull does not visibly respond to;
the same key *held* at a normal ~25/s repeat reaches full scale in ~3 s. If
auto-repeat is slow or off (common over ssh/tmux/VNC — check `xset q | grep -A1
repeat`) you will stay at a crawl however long you hold. Raise
`thrust_step:=20.0` if you would rather not hold keys.

The node republishes the held value continuously (default 20 Hz). Values are
clamped to the **bollard-measured** limits, which are asymmetric (reverse ≈ half
of forward) and differ per unit because T2 is mounted mirrored:

| | reverse | forward |
|---|---|---|
| `T1` port bow | −39.2266 N | +78.4532 N |
| `T2` stbd bow | −78.4532 N | +39.2266 N |
| `T3` stern | −274.6 N | +554.1 N |
| steering | −0.3665 rad | +0.3665 rad (±21°) |

Keep these equal to `thruster_allocator/config/allocator_bollard.yaml` and
`my_boat_gazebo/urdf/thrusters/my_boat_thruster_layout.xacro`. Gazebo clamps to
the xacro values, so a limit that is too generous here does not give you more
thrust — it just makes the status line overstate what the boat is getting.

## Topics published

```
/my_boat/thrusters/T1/thrust   std_msgs/Float64  (N)
/my_boat/thrusters/T2/thrust   std_msgs/Float64  (N)
/my_boat/thrusters/T3/thrust   std_msgs/Float64  (N)
/my_boat/thrusters/T3/pos      std_msgs/Float64  (rad)
```
These are the ROS side of `my_boat_gazebo/config/thrusters.yaml`, so the boat
must be spawned with its thruster bridge running (the default in
`spawn_my_boat.launch.py`).

## Run

### Use `ros2 run`, NOT `ros2 launch`

`ros2 launch` does **not** connect your keyboard to the node. The node reads raw
`STDIN_FILENO`, but launch gives its child a stdin that is not your terminal, so
no keystroke ever arrives. It fails *silently*: the node starts, prints its help,
publishes `0.0` at 20 Hz forever and looks perfectly healthy. Measured, holding
`w` for ~4 s under otherwise identical conditions:

| | node alive | publishing | peak T3 |
|---|---|---|---|
| `ros2 run` | yes | yes | **554.10 N** |
| `ros2 launch` | yes | yes | **0.00 N** |

`emulate_tty=True` in the launch file does not help — it only gives the child a
pty for **stdout** so status output flushes correctly.

```bash
colcon build --packages-select my_boat_teleop
source install/setup.bash

# in its OWN terminal (it grabs the keyboard):
ros2 run my_boat_teleop keyboard_teleop

# direction (wasd) mode:
ros2 run my_boat_teleop keyboard_teleop --ros-args -p control_mode:=direction

# bigger steps, if you would rather not hold keys down:
ros2 run my_boat_teleop keyboard_teleop --ros-args -p thrust_step:=20.0
```

The launch file is still useful for setting many parameters at once, but only if
you drive the node some other way (e.g. `ros2 topic pub` on the thruster
topics). It cannot be used for keyboard control.

## Parameters

`boat_namespace` (my_boat), `control_mode` (actuator | direction),
`thrust_step` (1.0 N/tap), `steer_step_deg`
(1.0 °/tap), `accel_gain` (0.6), `accel_max` (30.0×), `hold_reset_ms` (150),
`t1_min_thrust` (−39.2266), `t1_max_thrust` (78.4532), `t2_min_thrust`
(−78.4532), `t2_max_thrust` (39.2266), `t3_min_thrust` (−274.6),
`t3_max_thrust` (554.1), `steer_max` (0.3665), `publish_rate` (20.0 Hz).

> The old symmetric `bow_max_thrust` / `stern_max_thrust` parameters are gone —
> one number per side could express neither the forward/reverse asymmetry nor
> T2's mirroring. Scripts passing them will now silently have no effect, so
> update them to the per-thruster names above.
