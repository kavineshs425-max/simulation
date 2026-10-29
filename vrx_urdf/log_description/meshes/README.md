# meshes/

This directory is reserved for future mesh assets (DAE / STL / OBJ).

The current log_description package uses only primitive geometry
(cylinder) for both visual and collision elements — no external meshes
are required.

If a high-fidelity mesh is added in a future phase, reference it from
`urdf/log.urdf.xacro` and `models/floating_log/model.sdf` using a
`package://log_description/meshes/<file>` URI.
