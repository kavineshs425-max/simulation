# my_boat Propulsion System — v2 (decoupled layout)

Second propulsion arrangement for the USV `my_boat` in VRX / Gazebo Harmonic:
**two centreline lateral thrusters straddling the CG + one fixed axial stern
propeller**. It exists to give the vessel an allocation matrix that is *full
rank*, so surge, sway and yaw can be commanded independently — which the v1
arrangement (see [THRUSTERS.md](THRUSTERS.md)) cannot do.

Source of truth for the geometry and the sign conventions:
`dynamic_positioning/claude/thruster_allocator_v2/` (`config/allocator_v2.yaml`,
`image/layout_v2.jpg`, `README.md`).

**v1 is untouched.** Both layouts are spawnable; you pick one by picking a
launch file.

---

## 1. Hardware → simulation mapping

| Unit | Real hardware         | Role                             | Max thrust | DOF produced       |
|------|-----------------------|----------------------------------|-----------:|--------------------|
| T1   | APISQUEEN U92 Pro     | Forward lateral, centreline, fixed | ± 98 N   | sway (+Y) & +yaw   |
| T2   | APISQUEEN U92 Pro     | Aft lateral, centreline, fixed     | ± 98 N   | sway (+Y) & −yaw   |
| P1   | Torqeedo Cruise 3.0 R | Stern propeller, **fixed axial**   | ± 632 N  | surge (+X)         |

All three are **reversible** (commands saturate symmetrically at
`±max_thrust_cmd`). There is **no steering actuator** in this layout — no
steering joint, no `α`, no `/pos` topic.

---

## 2. Geometry

`base_link` is at the **centre of gravity**; frame is ROS marine convention
**x forward, y port, z up**. Hull extent in this frame:

```
bow tip x=+2.315   transom x=−1.854   keel z=−0.49   deck z=+0.441   half-beam 0.911
```

Stations, driven by the `d1/d2/d3/p1_y` arguments (defaults = `allocator_v2.yaml`):

| Item                  | x        | y      | z      | yaw (rad) | +cmd thrust    |
|-----------------------|---------:|-------:|-------:|----------:|----------------|
| T1 (forward lateral)  | **+d1** = +1.40 | 0.00 | −0.23 | +1.5708 | +Y (port)   |
| T2 (aft lateral)      | **−d2** = −0.90 | 0.00 | −0.23 | +1.5708 | +Y (port)   |
| P1 hull mount         | **−d3** = −1.40 | p1_y = 0.00 | −0.25 | +3.1416 | (faces −X) |
| P1 propeller          | −1.40    | 0.00   | −0.65  | —         | +X (forward)   |

P1 strut length = **0.40 m** (mount z=−0.25 → propeller z=−0.65, i.e. below the
keel like a real outboard leg); propeller diameter 0.10 m, strut diameter
0.05 m.

**Retuning:** `d1/d2/d3/p1_y` are launch/xacro arguments — no file edit needed:

```bash
ros2 launch my_boat_gazebo my_boat_world_v2.launch.py d1:=1.9 d2:=1.4 d3:=1.5
xacro my_boat_gazebo_v2.urdf.xacro namespace:=my_boat d1:=1.9 > /tmp/boat.urdf
```

Whatever you pick, **make `allocator_v2.yaml` match** — the allocator's `B` and
the simulated hull must describe the same boat, or every yaw command is biased
by the ratio of the two `d`'s. Mount depths (`lat_z`, the strut length) live in
`urdf/thrusters/my_boat_thruster_layout_v2.xacro`.

### ⚠️ Hull-fit check (measured off `my_boat_hull.stl`)

At the equilibrium draft (~0.10 m for 420 kg, per `config/buoyancy.yaml`) the
waterline sits at z ≈ −0.40, so the **wetted band of the hull is only
x ∈ [−0.95, +1.65]**:

| station x | +2.0 | +1.9 | +1.8 | +1.7 | +1.6 | +1.5 | +1.4 | … | −0.9 | −1.0 |
|-----------|-----:|-----:|-----:|-----:|-----:|-----:|-----:|---|-----:|-----:|
| keel z    |−0.22 |−0.29 |−0.33 |−0.38 |−0.42 |−0.43 |−0.45 | … |−0.40 |−0.37 |
| beam      | 0.75 | 0.89 | 0.99 | 1.09 | 1.18 | 1.26 | 1.32 | … | 1.79 | 1.59 |

The current defaults put **both laterals inside that band**: T1 at x=+1.40
(keel −0.45, beam 1.32) and T2 at x=−0.90 (keel −0.40, right at the aft edge of
the wetted run). The original sketch-scaled stations (`d1=1.90`, `d2=1.40`) did
not — the forefoot at x=+1.90 sits ~0.11 m *above* the waterline — which is why
they were moved in. The trade is yaw authority: `(d1+d2)` 2.30 m instead of
3.30 m, i.e. **225 N·m instead of 323 N·m** of pure yaw at 98 N laterals.

Those sketch numbers assumed a ~6 m hull; this hull is **4.168 m**. Measuring
`d1`/`d2` on the real boat is still the right fix (the allocator README says the
same). If you do go back to the wider stations, note that Gazebo's Thruster
plugin applies the commanded force regardless of submersion, so the simulation
will still realise the ideal `B` — it just will not reproduce the ventilation a
real tunnel thruster would suffer up there.

Two depths worth knowing, independent of the station choice:

* `lat_z = −0.23` is inside the hull at both stations but ~0.17 m **above** the
  modelled waterline. At a 0.10 m draft this hull has no station deep enough to
  bury a 0.10 m tunnel; a real installation implies a deeper draft or externally
  mounted pods. The gz physics is unaffected (force is applied at the link,
  submerged or not) beyond the roll arm it creates.
* P1's leg reaches z=−0.65, ~0.25 m **below** the waterline — genuinely in the
  water, exactly like the v1 stern drive.

---

## 3. Link & joint hierarchy

Produced by the macros in `urdf/thrusters/my_boat_engine_v2.xacro`
(verified with `check_urdf`):

```
my_boat/base_link
├── T1_propeller_link        (continuous T1_propeller_joint, yaw +90°, axis x ⇒ +Y)
├── T2_propeller_link        (continuous T2_propeller_joint, yaw +90°, axis x ⇒ +Y)
└── P1_strut_link            (FIXED P1_mount_joint, yaw 180° ⇒ faces −X)
    └── P1_propeller_link    (continuous P1_propeller_joint, axis −x ⇒ +X)
```

The laterals are **bare propellers attached directly to base_link** — no
housing, no shaft. On the real hull they are tunnel thrusters running
athwartships, so they sit on the centreline (y = 0) and are hidden inside the
hull mesh; that is expected, not a placement error.

P1 keeps the v1 leg geometry with the steering joint **replaced by a fixed
joint**. `gz sdf -p` lumps the strut into `base_link` and re-parents
`P1_propeller_joint` to `base_link` at pose `(−1.5, 0, −0.65)` with yaw π and
axis `−x` — i.e. the thrust axis survives the lumping intact (verified).

Total model mass: 420 (hull) + 2×2.5 (laterals) + 12 (leg) + 1 (prop) =
**438 kg**, within 0.5 kg of v1, so buoyancy/draft results stay comparable.

---

## 4. Thruster physics (force plugins)

Unchanged from v1 — `urdf/thrusters/my_boat_thruster_config.xacro`, reused as
is. Each unit is a `gz::sim::systems::Thruster` with `velocity_control=true`:

```
T = thrust_coefficient · ρ · |n| · n · D⁴      (ρ=1000, D=0.10 m, Ct=0.004422)
```

| Unit | joint driven         | max_thrust_cmd | min_thrust_cmd | \|n\|max   |
|------|----------------------|---------------:|---------------:|-----------:|
| T1   | T1_propeller_joint   | +98 N          | −98 N          | 14.9 rad/s |
| T2   | T2_propeller_joint   | +98 N          | −98 N          | 14.9 rad/s |
| P1   | P1_propeller_joint   | +632 N         | −632 N         | 37.8 rad/s |

The `steering_actuator` macro in that file is simply **not instantiated** — v2
has nothing to steer.

---

## 5. Command (topic) interfaces

Gazebo transport topics created by the plugins, bridged to ROS 2 by
`config/thrusters_v2.yaml`:

| Quantity  | Topic                          | ROS type         | gz type        | Units |
|-----------|--------------------------------|------------------|----------------|-------|
| T1 thrust | `/my_boat/thrusters/T1/thrust` | std_msgs/Float64 | gz.msgs.Double | N     |
| T2 thrust | `/my_boat/thrusters/T2/thrust` | std_msgs/Float64 | gz.msgs.Double | N     |
| P1 thrust | `/my_boat/thrusters/P1/thrust` | std_msgs/Float64 | gz.msgs.Double | N     |

(There is no `/pos` topic — that was v1's steering command.)

### Sign conventions — matching `thruster_allocator_v2`

* `T1 > 0` → force to **port (+Y)** at x=+d1 ⇒ **+N** (bow to port)
* `T2 > 0` → force to **port (+Y)** at x=−d2 ⇒ **−N**
* `P1 > 0` → surge **forward (+X)**; `P1 < 0` → astern

> **Changed from v1:** there, `T1 > 0` pushed to *starboard*. In v2 both
> laterals push to *port* for a positive command. Any tooling that assumed the
> v1 signs must be updated.

### Allocation model (3-DOF: surge X, sway Y, yaw N)

```
X = P1
Y = T1 + T2
N = d1·T1 − d2·T2 − p1_y·P1

          T1     T2     P1
    X [    0      0      1   ]
B = Y [    1      1      0   ]      det(B) = −(d1 + d2) = −2.30
    N [  +d1    −d2   −p1_y  ]
```

Full rank ⇒ surge is `P1` alone (axial, on the centreline: no sway, no yaw) and
the 2×2 lateral block is invertible, so sway and yaw are independently
commandable. `d3` does **not** appear in `B`. `p1_y ≠ 0` deliberately models a
mounting offset and turns surge into a parasitic yaw `N = −p1_y·P1`.

**Capability at the defaults** (98 N laterals, `d1=1.40`, `d2=0.90`) — these are
the figures the allocator prints in its start-up banner:

| axis            | formula                            | value     |
|-----------------|------------------------------------|----------:|
| surge           | `P1_max`                           | 632 N     |
| pure sway (N=0) | laterals scaled so `d1·T1 = d2·T2`  | 161 N     |
| pure yaw (Y=0)  | `(d1+d2)·min(T1_max,T2_max)`        | 225 N·m   |

⚠️ 225 N·m of pure yaw is **not** a large margin: real-boat testing logged a
constant yaw bias of 160–286 N·m. Upgrading the laterals to 20 kgf (196 N)
doubles it to 451 N·m. Watch `yaw_demand_frac` in the allocator diagnostics — if
it sits near 1.0 the hardware is the limit, not the allocator.

Out-of-plane side effects the allocator does not model (they are real, and the
sim will show them): lateral thrust acts 0.23 m below the CG → ~39 N·m of roll
at full sway; P1 acts 0.65 m below the CG → ~411 N·m **bow-up** pitch at full
ahead.

---

## 6. Run instructions

1. **Build & source:**
   ```bash
   cd ~/kavinesh/gazebo_vfx/vrx_garden_ws
   colcon build --packages-select my_boat_description my_boat_gazebo
   source install/setup.bash
   ```

2. **World + boat in one shot** (v2 counterpart of `my_boat_world.launch.py`):
   ```bash
   ros2 launch my_boat_gazebo my_boat_world_v2.launch.py
   ros2 launch my_boat_gazebo my_boat_world_v2.launch.py headless:=True
   ros2 launch my_boat_gazebo my_boat_world_v2.launch.py d1:=1.9 d2:=1.4
   ```
   Or spawn into an already-running world:
   ```bash
   ros2 launch vrx_gz competition.launch.py world:=sydney_regatta
   ros2 launch my_boat_gazebo spawn_my_boat_v2.launch.py     # bridge:=false to skip bridges
   ```

3. **Drive the thrusters** (Newtons):
   ```bash
   # Ahead
   ros2 topic pub -1 /my_boat/thrusters/P1/thrust std_msgs/msg/Float64 "{data: 632.0}"
   # Pure sway to port: d1·T1 = d2·T2  ⇒  T1=63.0, T2=98  (N = 0)
   ros2 topic pub -1 /my_boat/thrusters/T1/thrust std_msgs/msg/Float64 "{data: 63.0}"
   ros2 topic pub -1 /my_boat/thrusters/T2/thrust std_msgs/msg/Float64 "{data: 98.0}"
   # Pure yaw, bow to port: T1 = −T2 = 98  (Y = 0, N = +225 N·m)
   ros2 topic pub -1 /my_boat/thrusters/T1/thrust std_msgs/msg/Float64 "{data:  98.0}"
   ros2 topic pub -1 /my_boat/thrusters/T2/thrust std_msgs/msg/Float64 "{data: -98.0}"
   ```
   Without the ROS bridge you can command Gazebo directly:
   ```bash
   gz topic -t /my_boat/thrusters/P1/thrust -m gz.msgs.Double -p 'data: 632.0'
   ```

### Validation checklist (all run against this layout)

* `check_urdf` shows the tree in §3 — **passes**.
* `gz sdf -p my_boat_v2.urdf` converts with no errors (only the pre-existing
  `gz_frame_id` sensor warnings, identical to v1) — **passes**.
* `P1 > 0` alone drives straight ahead, no yaw — surge is decoupled.
* `T1 = −T2 = 98` turns the bow to **port** on the spot with no net sway —
  pure yaw, which v1 could not do.
* `T1 = 63.0, T2 = 98` translates to port with no heading change — pure sway.

---

## 7. Wiring the allocator to this model

`thruster_allocator_v2` publishes one `thruster_allocator_v2/ThrusterCommand`
(fields `t1/t2/p1_force_n` in Newtons, **no `alpha`**) on
`/thruster_allocator_v2/command`, while this model listens on three
`std_msgs/Float64` topics. The relay that closes that gap is
**`gazebo_thruster_bridge`'s v2 node** (`thrust_bridge_v2_node`); the package's
original node targets the v1 message (`alpha_rad → T3/pos`) and does not apply
here.

```
dp_controller ──/dp/control_command──▶ thruster_allocator_v2
                                          │ /thruster_allocator_v2/command
                                          ▼
                              gazebo_thruster_bridge_v2
                                          │ /my_boat/thrusters/{T1,T2,P1}/thrust
                                          ▼
                                  ros_gz_bridge ──▶ gz Thruster plugins
```

Ready-made bring-ups (in the `dynamic_positioning` workspace):

```bash
# DP station-keeping stack against this model
ros2 launch my_boat_gazebo my_boat_world_v2.launch.py
ros2 launch dp_control_system_v2 system_bringup_v2.launch.py

# nav2 transit chain (velocity_controller -> tau_mux -> allocator v2)
ros2 launch velocity_controller transit_bringup_v2.launch.py
ros2 launch gazebo_thruster_bridge gazebo_thruster_bridge_v2.launch.py
```

---

## 8. Files

| File | Purpose |
|------|---------|
| `urdf/thrusters/my_boat_engine_v2.xacro`          | v2 link/joint macros (`lateral_thruster`, `stern_propeller`) |
| `urdf/thrusters/my_boat_thruster_layout_v2.xacro` | Places T1/T2/P1 from `d1/d2/d3/p1_y` and attaches the plugins |
| `urdf/thrusters/my_boat_thruster_config.xacro`    | Plugin macros — **shared with v1**, unchanged |
| `urdf/my_boat_gazebo_v2.xacro`                    | Hull + dynamics + v2 propulsion + sensors macro |
| `urdf/my_boat_gazebo_v2.urdf.xacro`               | Top-level spawnable model; declares `d1/d2/d3/p1_y` |
| `config/thrusters_v2.yaml`                        | ros_gz_bridge config: T1/T2/P1 thrust, no steering |
| `launch/spawn_my_boat_v2.launch.py`               | Spawns the v2 boat + bridges into a running world |
| `launch/my_boat_world_v2.launch.py`               | VRX world + v2 boat in one shot |
