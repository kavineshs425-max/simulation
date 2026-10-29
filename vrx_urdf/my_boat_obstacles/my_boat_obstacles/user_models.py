"""User 3-D models: paste a folder into models/, use it as `model: <folder name>`.

    models/
      my_ferry/
        model.yaml        <- how to use the mesh (below)
        ferry.glb         <- the mesh: .stl .obj(+.mtl +images) .glb .gltf .dae
        hull.png          <- optional texture image

model.yaml:
    mesh: ferry.glb          # REQUIRED, file in this folder -- or a model Gazebo already
                             # has cached: fuel:openrobotics/post/meshes/post.dae
    scale: 1.0               # number, or [sx, sy, sz]; e.g. 0.001 for a mm model
    length: 12.0             # OR: make the hull this long [m] (scale is worked out;
                             #     for meshes in unknown units, e.g. Sketchfab/3-D print)
    forward_axis: +x         # which MESH axis points to the BOW:  +x -x +y -y +z -z
    up_axis: +z              # which MESH axis points UP (many obj/glb/dae are +y)
    draft: 0.5               # [m] how deep the lowest point sits below the water
    class: boat              # default detector class
    collision: box           # box | mesh | none
    colour: [0.8, 0.1, 0.1]  # optional, replaces the mesh's own materials
    texture: hull.png        # optional image, needs a mesh with texture coords
    matte: true              # .gltf/.glb only: use a copy with metalness 0 (Gazebo
                             # renders metallic surfaces black; the original is untouched)
    size: [l, w, h]          # ONLY for meshes this machine cannot measure (.dae,
                             # .fbx): the size after scale + rotation, in metres

Everything else is MEASURED from the mesh (trimesh): the bounding box after
scale and rotation gives the hull box (l along the bow, w, h), the visual is
shifted so that box's centre is the model origin, and `draft` puts its bottom
at z = -draft. So the ground truth box always matches what the lidar sees.

A broken model folder is an error with the folder named, not a silent skip:
it is almost always a typo the user wants to hear about.
"""

import math
import os
from typing import Dict, List, Sequence, Tuple

import yaml

from .catalog import BUILTIN, COLLISION_MODES, ModelSpec, check_texture, class_id

MESH_EXTS = ('.stl', '.obj', '.dae', '.glb', '.gltf', '.fbx', '.ply')
MEASURABLE = ('.stl', '.obj', '.glb', '.gltf', '.ply', '.dae')   # .dae via dae.py
_KEYS = {'mesh', 'scale', 'length', 'forward_axis', 'up_axis', 'draft', 'class', 'collision',
         'colour', 'texture', 'size', 'matte'}
_AXES = {'+x': (1, 0, 0), '-x': (-1, 0, 0), '+y': (0, 1, 0), '-y': (0, -1, 0),
         '+z': (0, 0, 1), '-z': (0, 0, -1)}

_cache: Dict[Tuple[str, float], Tuple[Tuple[float, ...], Tuple[float, ...]]] = {}


def axes_rotation(forward: str, up: str):
    """3x3 R mapping mesh coordinates to model coordinates (x = bow, z = up).
    Rows are the model axes expressed in mesh coordinates: [f; u x f; u]."""
    if forward not in _AXES or up not in _AXES:
        raise ValueError(f'forward_axis/up_axis must be one of {sorted(_AXES)}')
    f, u = _AXES[forward], _AXES[up]
    if sum(a * b for a, b in zip(f, u)) != 0:
        raise ValueError(f'forward_axis {forward} and up_axis {up} must be perpendicular')
    y = (u[1] * f[2] - u[2] * f[1], u[2] * f[0] - u[0] * f[2], u[0] * f[1] - u[1] * f[0])
    return [list(f), list(y), list(u)]


def rotation_to_rpy(R) -> Tuple[float, float, float]:
    """SDF roll/pitch/yaw (R = Rz(yaw) Ry(pitch) Rx(roll)) of a rotation matrix."""
    if abs(R[2][0]) < 1.0 - 1e-9:
        pitch = math.asin(-R[2][0])
        roll = math.atan2(R[2][1], R[2][2])
        yaw = math.atan2(R[1][0], R[0][0])
    else:                                   # gimbal lock: pitch = +-90 deg
        pitch = math.copysign(math.pi / 2, -R[2][0])
        roll = 0.0
        yaw = math.atan2(-R[0][1], R[1][1])
    return roll, pitch, yaw


def matte_copy(path: str) -> str:
    """A copy of a .gltf/.glb with every material's metallicFactor set to 0,
    written next to it as <name>.matte.<ext>. Rewritten only when the source
    is newer. Buffers/textures stay shared (a .gltf references them by relative
    path, so the copy must live in the same folder)."""
    import json
    import struct
    root, ext = os.path.splitext(path)
    out = f'{root}.matte{ext}'
    if os.path.isfile(out) and os.path.getmtime(out) >= os.path.getmtime(path):
        return out

    def flatten(doc):
        for m in doc.get('materials', []):
            m.setdefault('pbrMetallicRoughness', {})['metallicFactor'] = 0.0
        return doc

    if ext.lower() == '.gltf':
        with open(path) as f:
            doc = json.load(f)
        with open(out, 'w') as f:
            json.dump(flatten(doc), f)
    elif ext.lower() == '.glb':
        with open(path, 'rb') as f:
            data = f.read()
        magic, version, _ = struct.unpack_from('<III', data, 0)
        if magic != 0x46546C67:
            raise ValueError(f'{path} is not a GLB file')
        jlen, jtype = struct.unpack_from('<II', data, 12)
        doc = flatten(json.loads(data[20:20 + jlen]))
        rest = data[20 + jlen:]                       # the BIN chunk, untouched
        js = json.dumps(doc).encode()
        js += b' ' * ((4 - len(js) % 4) % 4)          # chunks are 4-byte aligned
        body = struct.pack('<II', len(js), jtype) + js + rest
        with open(out, 'wb') as f:
            f.write(struct.pack('<III', magic, version, 12 + len(body)) + body)
    else:
        raise ValueError(f'matte works on .gltf/.glb, not {ext}')
    return out


def _mesh_bounds(path: str):
    """Raw (min xyz, max xyz) of a mesh file in its own units, cached."""
    key = (path, os.path.getmtime(path))
    if key not in _cache:
        if path.lower().endswith('.dae'):
            from .dae import dae_bounds
            _cache[key] = dae_bounds(path)
        else:
            import trimesh   # heavy import, only when a user model is used
            mesh = trimesh.load(path, force='mesh')
            lo, hi = mesh.bounds
            _cache[key] = (tuple(float(v) for v in lo), tuple(float(v) for v in hi))
    return _cache[key]


FUEL_ROOTS = ('~/.gz/fuel/fuel.gazebosim.org', '~/.ignition/fuel/fuel.gazebosim.org')


def resolve_fuel(uri: str) -> str:
    """fuel:<owner>/<model>/<file inside the model>  ->  newest cached copy.
    E.g. fuel:openrobotics/mb_marker_buoy_red/meshes/mb_marker_buoy.dae. The
    Fuel cache fills when a world that uses the model is started once."""
    rest = uri[len('fuel:'):].strip('/')
    parts = rest.split('/')
    if len(parts) < 3:
        raise ValueError(f'{uri}: expected fuel:<owner>/<model>/<path>')
    owner, model, inner = parts[0], parts[1], '/'.join(parts[2:])
    for root in FUEL_ROOTS:
        base = os.path.join(os.path.expanduser(root), owner, 'models', model)
        if os.path.isdir(base):
            versions = sorted((d for d in os.listdir(base) if d.isdigit()), key=int)
            for v in reversed(versions):
                p = os.path.join(base, v, inner)
                if os.path.isfile(p):
                    return p
    raise ValueError(f'{uri} is not in the Fuel cache (~/.gz/fuel): start a world that uses '
                     f'{model} once so Gazebo downloads it, or copy the mesh into this folder')


def _scale3(scale) -> Tuple[float, float, float]:
    if isinstance(scale, (int, float)):
        s = (float(scale),) * 3
    else:
        s = tuple(float(v) for v in scale)
    if len(s) != 3 or min(s) <= 0.0:
        raise ValueError(f'scale must be a number or [sx, sy, sz] > 0, got {scale}')
    return s


def load_model_folder(folder: str) -> ModelSpec:
    name = os.path.basename(os.path.normpath(folder))
    cfg_path = os.path.join(folder, 'model.yaml')
    try:
        with open(cfg_path) as f:
            cfg = yaml.safe_load(f) or {}
    except OSError as e:
        raise ValueError(f"model '{name}': cannot read {cfg_path}: {e}") from None
    try:
        return _build(name, folder, cfg)
    except ValueError as e:
        raise ValueError(f"model '{name}' ({cfg_path}): {e}") from None


def _build(name: str, folder: str, cfg: dict) -> ModelSpec:
    bad = set(cfg) - _KEYS
    if bad:
        raise ValueError(f'unknown keys {sorted(bad)}; allowed {sorted(_KEYS)}')
    if 'mesh' not in cfg:
        raise ValueError('`mesh:` (the mesh file in this folder) is required')
    mesh = str(cfg['mesh'])
    mesh = resolve_fuel(mesh) if mesh.startswith('fuel:') else os.path.abspath(os.path.join(folder, mesh))
    ext = os.path.splitext(mesh)[1].lower()
    if ext not in MESH_EXTS:
        raise ValueError(f'mesh must be one of {MESH_EXTS}, got {cfg["mesh"]}')
    if not os.path.isfile(mesh):
        raise ValueError(f'mesh file not found: {mesh}')
    if cfg.get('matte', False):
        mesh = matte_copy(mesh)
    fwd = str(cfg.get('forward_axis', '+x'))
    R = axes_rotation(fwd, str(cfg.get('up_axis', '+z')))
    if 'length' in cfg:
        if 'scale' in cfg:
            raise ValueError('give scale OR length, not both')
        if ext not in MEASURABLE:
            raise ValueError(f'length needs a mesh that can be measured ({MEASURABLE}); '
                             f'use scale for {ext}')
        length = float(cfg['length'])
        if length <= 0.0:
            raise ValueError('length must be > 0')
        lo, hi = _mesh_bounds(mesh)
        axis = 'xyz'.index(fwd[1])
        raw = hi[axis] - lo[axis]
        if raw <= 0.0:
            raise ValueError(f'mesh has no extent along forward_axis {fwd}')
        s = _scale3(length / raw)
    else:
        s = _scale3(cfg.get('scale', 1.0))
    draft = float(cfg.get('draft', 0.0))
    if draft < 0.0:
        raise ValueError('draft is a depth below the water: it must be >= 0')
    cls = str(cfg.get('class', 'unknown'))
    class_id(cls)
    collision = str(cfg.get('collision', 'box'))
    if collision not in COLLISION_MODES:
        raise ValueError(f'collision must be one of {COLLISION_MODES}')

    if 'size' in cfg:
        size = tuple(float(v) for v in cfg['size'])
        if len(size) != 3 or min(size) <= 0.0:
            raise ValueError('size must be [l, w, h] > 0')
        centre = (0.0, 0.0, 0.0)     # trust that the mesh is already centred
    else:
        if ext not in MEASURABLE:
            raise ValueError(f'{ext} files cannot be measured on this machine; add '
                             f'`size: [l, w, h]` (metres, after scale and rotation)')
        lo, hi = _mesh_bounds(mesh)
        # all 8 corners, scaled then rotated into the model frame
        corners = [(x * s[0], y * s[1], z * s[2]) for x in (lo[0], hi[0])
                   for y in (lo[1], hi[1]) for z in (lo[2], hi[2])]
        rot = [tuple(sum(R[i][k] * c[k] for k in range(3)) for i in range(3)) for c in corners]
        mn = [min(c[i] for c in rot) for i in range(3)]
        mx = [max(c[i] for c in rot) for i in range(3)]
        size = tuple(mx[i] - mn[i] for i in range(3))
        centre = tuple(0.5 * (mx[i] + mn[i]) for i in range(3))
        if min(size) <= 0.0:
            raise ValueError(f'mesh measures {size}: it is flat or empty')
    roll, pitch, yaw = rotation_to_rpy(R)
    visual_pose = (-centre[0], -centre[1], -centre[2], roll, pitch, yaw)

    colour = cfg.get('colour')
    if colour is not None:
        colour = tuple(float(v) for v in colour)
        if len(colour) != 3 or not all(0.0 <= c <= 1.0 for c in colour):
            raise ValueError(f'colour must be [r, g, b] in 0..1, got {list(colour)}')
    texture = cfg.get('texture')
    if texture is not None:
        texture = check_texture(os.path.abspath(os.path.join(folder, str(texture))), mesh)

    return ModelSpec(size=size, centre_z=-draft + 0.5 * size[2], default_class=cls,
                     mesh=mesh, visual_pose=visual_pose, mesh_scale=s, colour=colour,
                     texture=texture, collision=collision, source=folder)


def look_warnings(spec: ModelSpec) -> List[str]:
    """Things that load fine but will LOOK wrong in Gazebo (ogre2)."""
    out = []
    if spec.mesh and os.path.splitext(spec.mesh)[1].lower() in ('.glb', '.gltf') \
            and spec.colour is None and spec.texture is None:
        try:
            import trimesh
            scene = trimesh.load(spec.mesh)
            geoms = scene.geometry.values() if hasattr(scene, 'geometry') else [scene]
            for g in geoms:
                mat = getattr(g.visual, 'material', None)
                metal = getattr(mat, 'metallicFactor', 0.0)
                # glTF's default metallicFactor is 1.0 when the file omits it
                if metal is None or metal >= 0.5:
                    out.append(f'{os.path.basename(spec.mesh)}: material is metallic '
                               f'({1.0 if metal is None else metal:g}); with no environment map '
                               f'Gazebo renders a metallic surface BLACK. Export with metallic 0, '
                               f'or set colour/texture in model.yaml.')
                    break
        except Exception as e:   # a warning must never stop the model loading
            out.append(f'could not inspect materials of {spec.mesh}: {e}')
    return out


def scan(dirs: Sequence[str]) -> Dict[str, ModelSpec]:
    """Every sub-folder with a model.yaml in `dirs`. First directory wins on a
    duplicate name, like a search path."""
    out: Dict[str, ModelSpec] = {}
    for d in dirs:
        if not os.path.isdir(d):
            continue
        for entry in sorted(os.listdir(d)):
            folder = os.path.join(d, entry)
            if not os.path.isfile(os.path.join(folder, 'model.yaml')):
                continue
            if entry in BUILTIN:
                raise ValueError(f"model folder '{folder}' has the same name as a built-in "
                                 f'model; rename the folder')
            if entry not in out:
                out[entry] = load_model_folder(folder)
    return out


def default_dirs(config_file: str, share_dir: str, extra: List[str]) -> List[str]:
    """Where to look, in order:
      1. settings.model_paths (relative ones are relative to the config file)
      2. models/ next to the config file, and ../models from it -- with
         --symlink-install the installed config is a link into the SOURCE tree,
         so this finds the source models/ folder and pasted models work with
         no rebuild
      3. the installed package's models/
    """
    base = os.path.dirname(os.path.realpath(config_file))
    dirs = [p if os.path.isabs(p) else os.path.normpath(os.path.join(base, p)) for p in extra]
    dirs += [os.path.join(base, 'models'), os.path.normpath(os.path.join(base, '..', 'models')),
             os.path.join(share_dir, 'models')]
    seen, out = set(), []
    for d in dirs:
        r = os.path.realpath(d)
        if r not in seen:
            seen.add(r)
            out.append(d)
    return out
