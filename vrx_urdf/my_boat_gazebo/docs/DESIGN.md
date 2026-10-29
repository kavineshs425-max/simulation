# my_boat_gazebo — Design Document

## Purpose

`my_boat_gazebo` turns the inert hull from `my_boat_description` into a
physically realistic floating vessel that interacts with water, waves, wind and
(emulated) current inside the existing VRX environment. It adds **no** custom
physics code — every behaviour comes from a VRX plugin that already ships in the
workspace.

## Architecture: mirroring VRX

VRX keeps a strict separation, which this project reproduces exactly:

```
wamv_description   (pure body: links, geometry, inertia)        my_boat_description
        +                                                  =>           +
wamv_gazebo        (marine physics plugins on top)              my_boat_gazebo
```

`my_boat_description` stays a pure description package. `my_boat_gazebo`'s xacro
chain includes it and appends the plugin blocks, exactly as
`wamv_gazebo_dynamics_plugin.xacro` does for the WAM-V:

```
my_boat_gazebo.urdf.xacro                 (top-level, spawnable; TF publisher)
  └─ my_boat_gazebo.xacro                 (macro: include body + dynamics)
       ├─ my_boat_description/.../my_boat_base.urdf.xacro   (the body)
       └─ dynamics/my_boat_gazebo_dynamics_plugin.xacro     (Surface + Hydro)
```

## VRX plugins reused (none custom)

| Concern | Plugin | Library | Scope |
|---|---|---|---|
| Buoyancy | `vrx::Surface` (×2, port/stbd) | `libSurface.so` | per link, in model |
| Hydrodynamic drag + added mass | `vrx::SimpleHydrodynamics` | `libSimpleHydrodynamics.so` | per link, in model |
| Waves | `vrx::Surface` shared wavefield | `libSurface.so` | subscribes `/vrx/wavefield/parameters` |
| Wind | `vrx::USVWind` | `libUSVWind.so` | in world; lists `my_boat/base_link` |
| Ocean current | *(no VRX plugin — emulated)* | — | constant wrench, see `config/current.yaml` |

## Why these, and how they map to the monohull

* **Surface (buoyancy + waves).** VRX models each WAM-V *pontoon* as a cylinder
  and samples buoyancy at points along it. A monohull has one body, but roll
  stability needs laterally separated buoyancy. So we use **two** Surface
  plugins — a port and a starboard half-hull at y = ±0.6 m — each sampling 5
  points along X at the hull bottom (z = −1.10 m). Points spread in X give
  pitch/heave restoring; the ±Y split gives roll restoring. Each Surface also
  subscribes to the shared VRX wavefield, so the boat rides the *same* waves as
  the WAM-V (no separate wave simulator). Sizing (`hull_radius`, `hull_length`,
  `fluid_density`) is derived in `config/buoyancy.yaml`.

* **SimpleHydrodynamics (drag/added mass).** VRX's Fossen 6-DOF model, the same
  plugin on the WAM-V `base_link`. It makes disturbances decay (needed for
  equilibrium and the stability soak). Coefficients are scaled from the WAM-V
  defaults to this 420 kg / 4.17 m hull in `config/hydrodynamics.yaml`.

* **USVWind (wind).** World-level plugin; the boat opts in with
  `<enable_wind>true</enable_wind>` (set in the description) and is registered as
  a `<wind_obj>` (snippet + bundled test world). Same wind field as the WAM-V.

* **Ocean current.** VRX has no current plugin and SimpleHydrodynamics measures
  drag against still water, so current is emulated as a steady body wrench (see
  `config/current.yaml`) rather than inventing a new physics system.

## Configuration is the source of truth

`config/*.yaml` hold every tunable coefficient. The spawn launch reads
`buoyancy.yaml` + `hydrodynamics.yaml`, overrides the matching `xacro:arg`s, and
spawns the model. The standalone `worlds/examples/my_boat_test.sdf` embeds the
same values inline for one-command testing.

## Extensibility (future phases)

The plugin blocks are additive and never touch the `base_link` from
`my_boat_description`. Future phases append thruster, IMU, GPS, camera, LiDAR or
controller plugins to the same xacro chain (just like `wamv_gazebo.urdf.xacro`)
without changing the body or the marine plugins configured here. The CG-centred
`base_link` is deliberately the body frame so thruster allocation, force/PWM
mapping and controllers all reference a single clean origin.
