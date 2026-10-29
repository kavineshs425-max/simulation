"""What each obstacle LOOKS like in Gazebo, and the SDF that spawns it.

Two kinds of model:

  BUILT-IN (this file). The three vessel meshes already in vrx_gz/models, plus
  five primitives. The meshes were measured from their STL files:

    mesh         bbox centre (x, y, z)       size (x, y, z)          floating z
    bass_boat    ( 0.000, -0.583,  0.107)    2.895  8.152  2.110     +0.398
    brazilian    (-0.009, -0.497, -0.028)    2.028  5.057  1.521     +0.338
    freedom_365  ( 0.072,  0.208,  1.817)    3.794 10.222  4.022     -0.210

    * The long axis is mesh +Y and the BOW is at mesh -Y (that end is 25-45 %
      narrower in beam and its keel rises out of the water). So the visual is
      yawed +90 deg, which maps mesh -Y onto model +X: the model's +X is its
      bow, like every other body in the stack.
    * "floating z" is the pose z at which the model.sdf's own buoyancy settles
      it (from the comments in those files).

  USER models (user_models.py): a folder with a mesh + model.yaml dropped into
  a models/ directory. Registered at start and on every reload.

For every mesh model the visual is shifted so the mesh bbox centre sits on the
model origin. The model pose is then the HULL CENTRE, and (l, w, h) is the true
hull box, which is what a 3-D detector's box and the ground truth both mean.
Obstacles are kinematic, not floating: the hull is held at its floating height
and does not heave on waves.

LOOK (per model, overridable per obstacle):
  colour   [r, g, b] 0..1. Any model. On a mesh that carries its own materials
           (glb, obj+mtl, dae) this REPLACES them.
  texture  an image (png/jpg) wrapped onto the surface as the albedo map. Works
           on primitives and on meshes that have texture coordinates (obj,
           glb/gltf, dae). NOT on STL: an STL has no texture coordinates, so
           there is nothing to map the image with (that is refused).
  neither  the mesh's own materials/textures, or the model's default colour
           (the built-in boats: the pale colours of their vrx_gz model.sdf).

COLLISION (per model, overridable per obstacle):
  box   a box of the hull bbox (default). Fast and stable; slightly generous
        round the ends of a pointed hull.
  mesh  the visual mesh itself. Exact shape, slower; use a low-poly mesh.
  none  no collision: sensors see it, the boat passes straight through.

Class ids are for gz-sim's Label system, which is what Gazebo's segmentation
and bounding-box cameras read, so a simulated labelled detector sees these
classes.
"""

import math
import os
from dataclasses import dataclass, replace
from typing import Dict, Optional, Sequence, Tuple

CLASS_IDS: Dict[str, int] = {
    'unknown': 0, 'boat': 1, 'ship': 2, 'kayak': 3, 'buoy': 4,
    'swimmer': 5, 'barge': 6, 'pillar': 7, 'marker': 8, 'debris': 9,
    'structure': 10, 'vegetation': 11,
}
COLLISION_MODES = ('box', 'mesh', 'none')
TEXTURE_EXTS = ('.png', '.jpg', '.jpeg')
UV_MESH_EXTS = ('.obj', '.dae', '.glb', '.gltf', '.fbx')   # can carry texture coords


@dataclass(frozen=True)
class ModelSpec:
    size: Tuple[float, float, float]      # hull box l (along bow), w, h
    centre_z: float                        # world z of the hull-box centre
    default_class: str
    mesh: Optional[str] = None             # uri (model://.. or absolute path), else primitive
    # mesh -> model transform: x y z roll pitch yaw (the <visual><pose>)
    visual_pose: Tuple[float, ...] = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    mesh_scale: Tuple[float, float, float] = (1.0, 1.0, 1.0)
    colour: Optional[Tuple[float, float, float]] = (0.7, 0.7, 0.7)
    texture: Optional[str] = None          # absolute path to an image
    collision: str = 'box'
    shape: str = 'box'                     # primitives: box | cylinder
    source: str = 'built-in'               # where it came from (for messages)


def _vessel(name, centre, size_xyz, float_z, cls, colour):
    cx, cy, cz = centre
    sx, sy, sz = size_xyz
    # after the +90 deg yaw: model length = mesh y extent, model beam = mesh x.
    # R(+90deg)·c = (-cy, cx); translate by its negative to centre the hull.
    return ModelSpec(size=(sy, sx, sz), centre_z=float_z + cz, default_class=cls,
                     mesh=f'model://{name}/meshes/{name}.stl',
                     visual_pose=(cy, -cx, -cz, 0.0, 0.0, math.pi / 2), colour=colour)


BUILTIN: Dict[str, ModelSpec] = {
    # colours = the <diffuse> of each vrx_gz model.sdf, so they look as before
    'brazilian': _vessel('brazilian', (-0.009, -0.497, -0.028), (2.028, 5.057, 1.521), 0.338,
                         'boat', (0.75, 0.78, 0.72)),
    'bass_boat': _vessel('bass_boat', (0.000, -0.583, 0.107), (2.895, 8.152, 2.110), 0.398,
                         'boat', (0.72, 0.75, 0.82)),
    'freedom_365': _vessel('freedom_365', (0.072, 0.208, 1.817), (3.794, 10.222, 4.022), -0.210,
                           'ship', (0.80, 0.80, 0.85)),
    # primitives: box h is the part ABOVE the water, plus 0.3 m below
    'kayak': ModelSpec((4.0, 0.8, 0.6), 0.0, 'kayak', colour=(0.95, 0.55, 0.10)),
    'buoy': ModelSpec((0.8, 0.8, 1.5), 0.45, 'buoy', colour=(0.95, 0.30, 0.05), shape='cylinder'),
    'swimmer': ModelSpec((0.6, 0.5, 0.5), 0.05, 'swimmer', colour=(0.95, 0.85, 0.10)),
    'barge': ModelSpec((8.0, 3.0, 3.0), 0.75, 'barge', colour=(0.42, 0.42, 0.45)),
    'pillar': ModelSpec((1.0, 1.0, 5.0), 1.5, 'pillar', colour=(0.35, 0.33, 0.30), shape='cylinder'),
    # floating water-hyacinth mat: flat, awash, soft (boats push through it)
    'hyacinth_patch': ModelSpec((3.0, 2.0, 0.15), 0.0, 'vegetation', colour=(0.22, 0.55, 0.16),
                                collision='none'),
    # the FIRST static field (my_boat_gazebo/models/obstacles/obstacle_*.sdf):
    # same geometry, same height against the waterline, same colour
    'obstacle_buoy': ModelSpec((0.9, 0.9, 2.4), 0.5, 'buoy', colour=(0.95, 0.88, 0.10), shape='cylinder'),
    'obstacle_pillar': ModelSpec((1.2, 1.2, 5.0), 1.0, 'pillar', colour=(0.95, 0.40, 0.05), shape='cylinder'),
    'obstacle_barge': ModelSpec((8.0, 3.0, 3.0), 0.75, 'barge', colour=(0.42, 0.42, 0.45)),
}
MODELS = BUILTIN       # kept for callers that iterate the built-ins

# name -> ModelSpec for the models found in models/ folders (user_models.py)
USER: Dict[str, ModelSpec] = {}


def all_models() -> Dict[str, ModelSpec]:
    out = dict(BUILTIN)
    out.update(USER)
    return out


def set_user_models(models: Dict[str, ModelSpec]) -> None:
    clash = set(models) & set(BUILTIN)
    if clash:
        raise ValueError(f'user model name(s) {sorted(clash)} clash with built-in models')
    USER.clear()
    USER.update(models)


def check_texture(path: str, mesh: Optional[str]) -> str:
    """Validate an ABSOLUTE texture path against the surface it goes on."""
    if not os.path.isfile(path):
        raise ValueError(f'texture file not found: {path}')
    if os.path.splitext(path)[1].lower() not in TEXTURE_EXTS:
        raise ValueError(f'texture must be one of {TEXTURE_EXTS}: {path}')
    if mesh and os.path.splitext(mesh)[1].lower() == '.stl':
        raise ValueError('an STL mesh has no texture coordinates, so a texture cannot be '
                         'mapped onto it -- use colour, or export the model as .obj/.glb/.dae')
    return path


def model_spec(name: str, size: Optional[Sequence[float]] = None,
               colour: Optional[Sequence[float]] = None,
               collision: Optional[str] = None,
               texture: Optional[str] = None) -> ModelSpec:
    """The catalogue entry, optionally with a new size / colour / collision /
    texture. `texture` must already be an absolute path.

    Only the primitive models can be RESIZED (a mesh is scaled in its
    model.yaml). A resized primitive keeps the SAME submerged depth as the
    default: its bottom stays at default centre_z - default h/2."""
    models = all_models()
    if name not in models:
        raise ValueError(f'unknown model {name!r}; one of {sorted(models)}')
    m = models[name]
    if size is not None:
        if m.mesh:
            raise ValueError(f'model {name!r} is a mesh: size can only be set on the primitive '
                             f'models {sorted(k for k, v in BUILTIN.items() if not v.mesh)} '
                             f'(scale a mesh in its model.yaml)')
        size = tuple(float(v) for v in size)
        if len(size) != 3 or min(size) <= 0.0:
            raise ValueError(f'size must be [l, w, h] > 0, got {list(size)}')
        if m.shape == 'cylinder' and size[0] != size[1]:
            raise ValueError(f'{name} is a cylinder: size l and w (the diameter) must be equal')
        bottom = m.centre_z - 0.5 * m.size[2]
        m = replace(m, size=size, centre_z=bottom + 0.5 * size[2])
    if colour is not None:
        colour = tuple(float(v) for v in colour)
        if len(colour) != 3 or not all(0.0 <= c <= 1.0 for c in colour):
            raise ValueError(f'colour must be [r, g, b] in 0..1, got {list(colour)}')
        m = replace(m, colour=colour)
    if texture is not None:
        m = replace(m, texture=check_texture(texture, m.mesh))
    if collision is not None:
        if collision not in COLLISION_MODES:
            raise ValueError(f'collision must be one of {COLLISION_MODES}, got {collision!r}')
        if collision == 'mesh' and not m.mesh:
            raise ValueError(f'model {name!r} is a primitive: collision "mesh" needs a mesh '
                             f'(its box/cylinder already IS its exact shape)')
        m = replace(m, collision=collision)
    return m


def class_id(cls: str) -> int:
    if cls not in CLASS_IDS:
        raise ValueError(f'unknown class {cls!r}; one of {sorted(CLASS_IDS)}')
    return CLASS_IDS[cls]


RUN_TOPIC = '/my_boat_obstacles/run'
SAMPLES_TOPIC = '/my_boat_obstacles/samples'


def samples_text(samples) -> str:
    """[(tau, x, y, yaw), ...] -> the plugin's "tau x y yaw ..." text."""
    return ' '.join(f'{t:.4f} {x:.4f} {y:.4f} {yaw:.5f}' for t, x, y, yaw in samples)


def _primitive_geometry(m: ModelSpec) -> str:
    l, w, h = m.size
    if m.shape == 'cylinder':
        return f'<cylinder><radius>{0.5 * l}</radius><length>{h}</length></cylinder>'
    return f'<box><size>{l} {w} {h}</size></box>'


def _material(m: ModelSpec) -> str:
    """No <material> at all = the mesh keeps its own materials/textures."""
    if m.texture:
        # PBR albedo map; the colour (if any) tints it, white = the image as-is
        r, g, b = m.colour if m.colour is not None else (1.0, 1.0, 1.0)
        return (f'<material><ambient>{r} {g} {b} 1</ambient><diffuse>{r} {g} {b} 1</diffuse>'
                f'<pbr><metal><albedo_map>{m.texture}</albedo_map>'
                f'<metalness>0.0</metalness><roughness>0.8</roughness></metal></pbr></material>')
    if m.colour is None:
        return ''
    r, g, b = m.colour
    return (f'<material><ambient>{r} {g} {b} 1</ambient><diffuse>{r} {g} {b} 1</diffuse>'
            f'<specular>0.2 0.2 0.2 1</specular></material>')


def obstacle_sdf(model_name: str, entity_name: str, cls: str, samples, plugin_path: str,
                 run_topic: str = RUN_TOPIC, samples_topic: str = SAMPLES_TOPIC,
                 size: Optional[Sequence[float]] = None,
                 colour: Optional[Sequence[float]] = None,
                 collision: Optional[str] = None,
                 texture: Optional[str] = None) -> str:
    """SDF for one moving obstacle. `samples` = [(tau, x, y, yaw), ...] with
    tau in seconds since RUN."""
    m = model_spec(model_name, size, colour, collision, texture)
    l, w, h = m.size
    if m.mesh:
        pose = ' '.join(f'{v:.10g}' for v in m.visual_pose)
        scale = ' '.join(f'{v:.6g}' for v in m.mesh_scale)
        mesh_geom = f'<mesh><uri>{m.mesh}</uri><scale>{scale}</scale></mesh>'
        visual = (f'<visual name="visual"><pose>{pose}</pose>'
                  f'<geometry>{mesh_geom}</geometry>{_material(m)}</visual>')
        if m.collision == 'mesh':
            collision_xml = (f'<collision name="collision"><pose>{pose}</pose>'
                             f'<geometry>{mesh_geom}</geometry></collision>')
        else:
            collision_xml = (f'<collision name="collision"><geometry>'
                             f'<box><size>{l} {w} {h}</size></box></geometry></collision>')
    else:
        visual = (f'<visual name="visual"><geometry>{_primitive_geometry(m)}</geometry>'
                  f'{_material(m)}</visual>')
        collision_xml = (f'<collision name="collision"><geometry>{_primitive_geometry(m)}'
                         f'</geometry></collision>')
    if m.collision == 'none':
        collision_xml = ''
    # heavy, so a collision shoves the boat rather than the obstacle; the
    # teleport each step overrides any push anyway
    mass = 5000.0
    ixx = mass * (w * w + h * h) / 12.0
    iyy = mass * (l * l + h * h) / 12.0
    izz = mass * (l * l + w * w) / 12.0
    text = samples_text(samples)
    x0, y0, yaw0 = samples[0][1], samples[0][2], samples[0][3]
    return f'''<?xml version="1.0"?>
<sdf version="1.9">
  <model name="{entity_name}">
    <pose>{x0} {y0} {m.centre_z} 0 0 {yaw0}</pose>
    <link name="base_link">
      <gravity>false</gravity>
      <inertial><mass>{mass}</mass>
        <inertia><ixx>{ixx:.1f}</ixx><iyy>{iyy:.1f}</iyy><izz>{izz:.1f}</izz>
        <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
      {visual}
      {collision_xml}
    </link>
    <plugin filename="gz-sim-label-system" name="gz::sim::systems::Label">
      <label>{class_id(cls)}</label>
    </plugin>
    <plugin filename="{plugin_path}" name="my_boat_obstacles::MovingObstacle">
      <z>{m.centre_z}</z>
      <run_topic>{run_topic}</run_topic>
      <samples_topic>{samples_topic}</samples_topic>
      <samples>{text}</samples>
    </plugin>
  </model>
</sdf>
'''
