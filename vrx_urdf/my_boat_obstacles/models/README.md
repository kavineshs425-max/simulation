# Your own obstacle models

Put a 3-D model in a folder here, and use the folder name as `model:` in
`config/obstacles.yaml`. You don't need to rebuild: call reload and it is
picked up.

```
models/
  my_ferry/            <- the folder name is the model name
    model.yaml         <- how to use the mesh (below)
    ferry.glb          <- the mesh
    hull.png           <- optional texture
```

```bash
# after adding the folder and using it in config/obstacles.yaml
ros2 service call /obstacle_spawner/reload std_srvs/srv/Trigger
```

## Mesh formats

| format | measured automatically | own colours/textures | texture: works |
|---|---|---|---|
| `.glb` / `.gltf` | yes | yes (embedded) | yes |
| `.obj` (+ `.mtl` + images) | yes | yes (via the .mtl) | yes |
| `.stl` | yes | no, use `colour` | **no** (STL has no texture coordinates) |
| `.ply` | yes | vertex colours only | depends on the file |
| `.dae` (Collada) | yes (own reader, `my_boat_obstacles/dae.py`) | yes | yes |
| `.fbx` | **no**: add `size:` | yes | yes |

Prefer `.glb`: one file with the mesh, materials and textures together.
Keep it reasonably low-poly (a few tens of thousands of triangles). Every
obstacle is re-rendered for the lidar and camera each frame, and
`collision: mesh` uses the mesh for physics too.

## model.yaml

```yaml
mesh: ferry.glb          # REQUIRED: the mesh file in this folder, or a model Gazebo
                         #   already has cached: fuel:openrobotics/post/meshes/post.dae
scale: 1.0               # a number or [sx, sy, sz]; 0.001 for mm, 0.01 for cm, 0.0254 for inches
length: 12.0             # OR the real hull length in metres: the scale is worked out.
                         #   Use this when the units are unknown (Sketchfab, 3-D-print STLs)
forward_axis: +x         # which MESH axis points to the BOW: +x -x +y -y +z -z
up_axis: +z              # which MESH axis points UP. Many .obj/.glb/.dae files are +y
draft: 0.5               # [m] how deep the lowest point sits under the water
class: boat              # default detector label:
                         #   unknown boat ship kayak buoy swimmer barge pillar marker debris structure
collision: box           # box (fast, default) | mesh (exact, slower) | none (ghost)
colour: [0.8, 0.1, 0.1]  # optional: REPLACES the mesh's own materials
texture: hull.png        # optional: image wrapped onto the mesh (needs texture coords)
size: [12.0, 4.0, 3.5]   # ONLY for .dae/.fbx: l w h in metres, after scale + rotation
matte: true              # .gltf/.glb: use a copy with metalness 0 (see "renders black" below)
```

What the spawner does with it:

- **Measures the mesh.** The bounding box after `scale` and rotation gives the
  hull box (l along the bow, w across, h up), and that box goes out on the
  ground truth.
- **Centres it.** The visual is shifted so that the box centre is the model
  origin. The mesh's own origin doesn't matter.
- **Floats it.** The bottom of the box goes `draft` metres below the waterline.
- **Points the bow forward.** It rotates the mesh so `forward_axis` points
  along the obstacle's heading and `up_axis` points up.

### Getting forward_axis / up_axis right

Open the mesh in any viewer (Blender, MeshLab, an online glTF viewer) and
note which axis runs from stern to bow and which points up. Getting it wrong
is easy to spot in Gazebo: the obstacle travels sideways, or lies on its side.

## Colour and texture per obstacle

In `config/obstacles.yaml` you can override the look for one obstacle
without touching the model:

```yaml
  - name: red_ferry
    model: my_ferry
    colour: [0.9, 0.1, 0.1]
  - name: striped_barge
    model: barge                   # primitives take textures too
    texture: textures/stripes.png  # relative to the config file
```

## Mistakes are errors, not silent skips

If a folder here has a broken `model.yaml` (a missing mesh, a typo in a key,
a `.dae` without `size`, or a folder named like a built-in model), start-up
or reload fails with a message naming the folder. A reload that fails
changes nothing in the running sim.

## Very detailed meshes

Every obstacle is re-rendered for the lidar and camera each frame. A mesh
with over ~100k triangles slows the whole sim. Reduce it once, for example
with open3d (installed here):

```python
import open3d as o3d
m = o3d.io.read_triangle_mesh('big.stl'); m.remove_duplicated_vertices()
o3d.io.write_triangle_mesh('small.stl', m.simplify_quadric_decimation(60000))
```

## Renders black?

A glTF/GLB material with metalness ≥ 0.5 renders black in Gazebo, because
there is no environment map for a metal to reflect. Sketchfab exports often
have this. The spawner warns about it at start. Add `matte: true`.

## Meshes from Gazebo's Fuel cache

`mesh: fuel:<owner>/<model>/<path>` uses the newest copy in `~/.gz/fuel`,
so nothing is copied. The VRX buoys and post work this way, e.g.
`fuel:openrobotics/mb_marker_buoy_red/meshes/mb_marker_buoy.dae`. If the model
isn't cached, start a world that uses it once and Gazebo downloads it. The
error message says so.

Collada files are measured by the package's own reader. It applies `<unit>`
(the VRX buoys are in feet) but not `<up_axis>`, because Gazebo doesn't
either. That's why the Y-up buoys use `up_axis: +y`.

## The examples

- `shipping_container/`: a 20 ft container adrift and mostly awash (only
  0.7 m of freeboard, which is hard for a lidar). It's an `.obj` with its own
  `.mtl` and `container.png`, so it needs no colour or texture settings.
- `jarethcruz38819hs-essex/`: a 12 m workboat. The original STL
  (`source/duncanboat.zip`: 1.49 M triangles, unknown units) was decimated to
  `essex.stl` (60k triangles, bounding box unchanged to 2 mm). It uses
  `length: 12.0`, `forward_axis: -y` and a `colour`.
- `frickies_yacht/`: a 30 m superyacht, glTF with 46 textures. It uses
  `length: 30.0`, is Y-up with the bow at −Z, and has `matte: true` because
  its hull material is metallic. **Licence CC-BY-NC-4.0: credit the author, no
  commercial use** (`license.txt`).
- `marker_buoy_red/green/black/white/`, `round_buoy_orange/black/`,
  `mooring_post/`: the VRX buoys and post from the Fuel cache, coloured with
  the colours from their own `model.sdf`.
- `pile/`, `oil_drum/`, `floating_log/`: generated with trimesh as matte GLB,
  covering poles and low floating debris.
- `channel_marker/`: a spar buoy with a cone top, exported Y-up and in
  centimetres, as many CAD tools do. `scale: 0.01` and `up_axis: +y` fix
  that, and `colour` gives the STL its green.
