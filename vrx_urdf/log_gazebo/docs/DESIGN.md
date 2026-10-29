# log_gazebo — Design Document

## Purpose

`log_gazebo` turns the inert `floating_log` body from `log_description` into a
physically realistic floating object that interacts with water, waves and wind
inside the existing VRX environment. It adds **no** custom physics code: every
behaviour comes from a VRX plugin that already ships with the workspace.

## Architecture: mirroring VRX

VRX keeps a strict separation:

```
wamv_description   (pure body: links, geometry, inertia)
        +
wamv_gazebo        (marine physics plugins layered on top)
```

This package follows the identical pattern:

```
log_description    (pure cylinder body: base_link, geometry, inertia)
        +
log_gazebo         (Surface + SimpleHydrodynamics + USVWind wiring)
```

`log_description` stays untouched and remains a pure description package.
`log_gazebo/urdf/log_gazebo.xacro` includes the description and appends the
plugin blocks, exactly as `wamv_gazebo_dynamics_plugin.xacro` does for the WAM-V.

## 1. Which VRX plugins were reused

| Concern | Plugin | Library | Scope |
|---|---|---|---|
| Buoyancy | `vrx::Surface` | `libSurface.so` | per link, in model |
| Hydrodynamic drag + added mass | `vrx::SimpleHydrodynamics` | `libSimpleHydrodynamics.so` | per link, in model |
| Waves | `vrx::Surface` (shared wavefield) | `libSurface.so` | model subscribes to world topic |
| Wind | `vrx::USVWind` | `libUSVWind.so` | in world, lists affected links |

## 2. Why they were selected

**Surface (buoyancy).** VRX attaches one Surface plugin to each WAM-V hull and
the plugin internally approximates each hull as a *cylinder* defined by
`hull_length` and `hull_radius`. The log is literally a cylinder, so this is the
most natural possible fit — the model VRX uses for an approximation is exact
here. Surface also samples wave elevation at its sample points, which is what
makes a body ride waves, so choosing it gives Phase 1 and Phase 3 from a single
plugin.

**SimpleHydrodynamics (drag).** This is the VRX implementation of Fossen's
6-DOF maritime equations (linear + quadratic drag and added mass). It is the
same plugin VRX puts on the WAM-V `base_link`. Without it a buoyant body bobs
forever; with it, disturbances decay. Reusing it keeps the log's damping model
consistent with the rest of the VRX fleet.

**USVWind (wind).** VRX's wind is a world-level plugin that applies force only
to bodies listed as `<wind_obj>`. Reusing it (rather than Gazebo's generic wind)
means the log feels the *same* wind direction and speed already configured for
the WAM-V in the world.

Choosing these three means zero new physics code, full consistency with the
WAM-V, and automatic participation in the shared wave and wind fields.

## 3. How they connect to floating_log

- **Buoyancy / waves.** The Surface plugin is bound to `base_link` with
  `hull_length = 2.0`, `hull_radius = 0.15`, `fluid_level = 0`. Five sample
  points are spread along the body X-axis (the log's long axis). Its
  `<wavefield><topic>` points at `/vrx/wavefield/parameters` — the topic the
  VRX world publishes — so the log heaves, pitches and rolls with the identical
  wavefield as the WAM-V. No separate wave simulator is introduced.

- **Hydrodynamics.** SimpleHydrodynamics is bound to the same `base_link` and
  applies drag/added-mass in all 6 DOF, opposing the Surface forces so motion
  settles.

- **Wind.** `base_link` carries `<enable_wind>true</enable_wind>`. The log is
  then registered as a `<wind_obj>` in the world's existing USVWind plugin
  (snippet in `worlds/examples/wind_obj_snippet.sdf`; pre-wired in the bundled
  `worlds/examples/floating_log_test.sdf`).

## Physics summary (documented coefficients)

### Buoyancy

```
Geometry        cylinder L = 2.0 m, r = 0.15 m
Full volume     V   = pi r^2 L            = 0.14137 m^3
Mass            m                          = 50 kg
Fluid density   rho                        = 1000 kg/m^3 (VRX fresh-water default)
Displaced vol   V_sub = m / rho            = 0.05000 m^3
Submerged frac  V_sub / V                  = 35.4 %
```

At equilibrium the log sits roughly one third submerged, axis parallel to the
surface. (Using sea water, 1025 kg/m^3, gives 34.5 %.)

### Hydrodynamic coefficients (placeholders, see config/hydrodynamics.yaml)

Shaped to the cylinder's symmetry and scaled to a 50 kg, 2 m body:

- Broadside drag (sway `yV/yVV`, heave `zW/zWW`) is larger than axial drag
  (surge `xU/xUU`) — a cylinder pushes far more water sideways than end-on.
- Heave (`zW`) and pitch (`mQ`) damping are the terms that make a disturbed
  log settle instead of oscillating forever.
- Roll damping (`kP/kPP`) and roll added inertia (`kDotP`) are deliberately
  near zero: a smooth cylinder offers almost no resistance to spinning about
  its own long axis. This is physically correct (a real log spins freely) yet
  non-zero, so the body is never frictionless.
- Added-mass terms are negative per Fossen's convention and kept below the
  ~50 kg displaced-fluid mass to stay numerically stable.

### Wind coefficients (see config/wind.yaml)

`coeff_vector = 0.5 0.5 0.33`, following the same FAU-report basis VRX uses for
the WAM-V; the lateral term dominates because the log presents far more side
area than end area to the wind.

## Extensibility

The plugin blocks are additive and never alter the `base_link` from
`log_description`. Future phases can append thruster, IMU, GPS, or controller
plugins to the same xacro without touching the body or the marine plugins
configured here.
