# meshes/

The canonical hull mesh used by both the URDF and the SDF lives at:

    models/my_boat/mesh/my_boat_hull.dae

(kept under `models/` so it sits next to its `model.config`, matching the
VRX `wamv_description` layout). This `meshes/` directory is reserved for any
additional/derived mesh assets — e.g. a decimated collision mesh — referenced
via `package://my_boat_description/meshes/<file>`.

## About the imported mesh

`my_boat_hull.dae` is the CAD export (SolidWorks -> Blender -> COLLADA). Two
properties are handled entirely in the xacro, not by editing the mesh:

* **Units:** the mesh is in **millimetres** -> applied `scale = 0.001`.
* **Axes:** CAD is Y-up (X=beam, Y=up, Z=fore/aft) -> rotated into the ROS
  marine frame (X-fwd, Y-port, Z-up) with `rpy = 1.5708 0 1.5708`.

See `docs/FRAME_AND_CG.md` for the full derivation.

> Note: the DAE references a texture `metal_basecolor.jpg` that was not part of
> the export. The hull still renders (flat material); drop the texture file in
> this folder or beside the DAE if you want the PBR albedo.
