# my_boat_description — Frame, CG and Mesh Transform Derivation

Authoritative record of how the CAD assets become a CG-centred `base_link`.
Every number here is baked into `urdf/my_boat_base.urdf.xacro` and mirrored in
`models/my_boat/model.sdf` and the test world.

---

## 0. Why STL, not DAE

The original DAE (SolidWorks → Blender → COLLADA) had a transform **baked into
its scene `<node><matrix>`** (0.001 scale + a Y↔Z rotation + a translation of
≈ 0.19 / 0.57 / 0.95 m). Gazebo applies that matrix when it loads the mesh, so
the geometry was silently scaled, rotated **and shifted** — moving the origin
away from the SolidWorks frame and breaking CG alignment (the CoM marker ended
up outside the hull).

The **STL** (`models/my_boat/mesh/my_boat_hull.stl`) is raw triangles in the
original SolidWorks coordinate system — no units tag, no scene graph, no baked
transform. So the geometry, the reported CG and the inertia all live in one
frame again. Confirmation: the STL signed volume is **2.9755 m³**, matching the
CAD mass-properties report (2.98 m³) — it is the exact CAD geometry.

---

## 1. Inputs (as supplied)

From `mass_properties.pdf` (SolidWorks, *user-overridden* mass and CG):

| Quantity | Value | Used? |
|---|---|---|
| Mass (user-overridden) | **420.00 kg** | ✅ authoritative |
| Center of Mass (user-overridden) | X=0.00, Y=0.49, Z=0.47 m | ⚠ only Y is reliable (see §3) |
| Inertia @ CoM (CAD) | Lxx=606.43, Lyy=652.79, Lzz=100.22, Lyz=−5.48 | ✅ authoritative |
| Density / Volume | — | ❌ ignored (solid-body artefact) |

---

## 2. CAD origin & frame (from the STL)

The STL is in **millimetres**, SolidWorks **"Y-up"**, origin at a hull corner
(all coordinates positive):

| CAD axis | Meaning | STL range (mm) | Size (m) |
|---|---|---|---|
| CAD **X** | lateral / beam | 13.4 … 1835.5 | 1.822 |
| CAD **Y** | vertical / up (keel = 0) | 1.8 … 932.8 | 0.931 |
| CAD **Z** | longitudinal / fore–aft | 0.0 … 4168.0 | 4.168 |

Cross-check: the smallest principal moment of inertia (100.2 kg·m²) is about
CAD Z → CAD Z is the long/roll axis; CAD Y is vertical (largest, yaw). Consistent.

---

## 3. Center of gravity used

**CG = hull volume centroid (computed from the STL):**

```
CG (CAD/STL mm) = (924.5, 491.6, 2314.5)
```

Why the centroid rather than the report's literal numbers:

| Axis | Report (mm) | Volume centroid (mm) | Verdict |
|---|---|---|---|
| X (lateral) | 0 | 924.5 | report value is at the hull **edge** → placeholder, rejected |
| Y (vertical) | 490 | 491.6 | **match** → report Y is real (CG height from keel) |
| Z (longitudinal) | 470 | 2314.5 | report value is near a hull **end** → placeholder, rejected |

The report Y agreeing with the centroid Y confirms the centroid is the correct
CG; the report X and Z are artefacts of the "user-overridden" field. To force a
different CG, edit `cg_x_mm` / `cg_y_mm` / `cg_z_mm` in `my_boat_base.urdf.xacro`.

---

## 4. CAD → ROS rotation

ROS marine convention: **x = forward, y = port, z = up**. Cyclic permutation:

```
ros_x (forward) = CAD_z (longitudinal)
ros_y (port)    = CAD_x (lateral)
ros_z (up)      = CAD_y (vertical)

R = | 0 0 1 |     rpy = (π/2, 0, π/2) = "1.5708 0 1.5708"
    | 1 0 0 |
    | 0 1 0 |
```

---

## 5. Computed mesh offset

`base_link` sits at the CG, so after scaling (mm→m) and rotating we translate by
**−R·(scale·CG)**:

```
scale·CG (m) = (0.9245, 0.4916, 2.3145)         [CAD order X,Y,Z]
R·(scale·CG) = (CGz, CGx, CGy) = (2.3145, 0.9245, 0.4916)   [ROS]
mesh_offset  = −R·(scale·CG)   = (−2.3145, −0.9245, −0.4916) m
```

Verification (CG vertex 924.5, 491.6, 2314.5 mm):
`R·(0.001·CG) + mesh_offset = (0, 0, 0)` ✅ → CG lands on base_link origin.

---

## 6. Final transforms applied (visual AND collision, identical)

```
mesh scale = 0.001 0.001 0.001
rpy        = 1.5708 0 1.5708
xyz        = -2.3145 -0.9245 -0.4916
```

Resulting hull extent in `base_link` (ROS) metres — **CG inside the hull**:

| Axis | min | max | size |
|---|---|---|---|
| x (forward) | −2.315 | 1.854 | 4.168 |
| y (port) | −0.911 | 0.911 | 1.822 |
| z (up) | −0.490 | 0.441 | 0.931 |

CG (base_link origin) is at the centre in x and y, and ~53 % up the hull in z.

---

## 7. Inertia

`base_link` = CoM, so the inertial `<origin>` is `0 0 0` and the at-CoM tensor
is used, rotated to ROS (`I_ros = R·I_cad·Rᵀ`):

```
Ixx = 100.22   Ixy = 0.0     Ixz = -5.48     (roll, smallest)
               Iyy = 606.43  Iyz = 0.0       (pitch)
                             Izz = 652.79    (yaw, largest)
```

---

## 8. Collision geometry

Per the latest request, collision uses the **same STL mesh** with the **same
transform** as the visual (shared `mesh_scale` / `mesh_rpy` / `mesh_off_*`), so
visual and collision are always coincident. Note: a full triangle-mesh collision
is heavier/less stable than a box, and it is **not** used for buoyancy (the
Surface plugin samples its own points) — only for contacts. Swap to a box here
if contact stability becomes an issue.

---

## 9. Validation procedure (frame / CG / alignment)

1. **CoM at base_link:** in `gz sim` enable *View → Center of Mass* and *Inertia*.
   The CoM marker must sit at the base_link origin, **inside** the hull, centred
   in beam and length and a bit above mid-height.
2. **Visual alignment:** the hull surrounds the marker, bow along +X.
3. **Collision alignment:** *View → Collisions* — the mesh coincides with the visual.
4. **Inertia ellipsoid:** long axis along the boat fore–aft (X) axis.
5. **Pose:** `gz topic -e -t /model/my_boat/pose` returns the commanded pose.
