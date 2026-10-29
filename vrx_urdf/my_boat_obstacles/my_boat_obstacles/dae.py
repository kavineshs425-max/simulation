"""Minimal Collada (.dae) reader -- positions only, no pycollada needed.

Reads vertex positions, <triangles>/<polylist>, the <node><matrix> tree and
<unit meter=...>. It does NOT apply <up_axis>: Gazebo does not either (the VRX
marker buoys are Y_UP and their own model.sdf rotates them upright), so a
Y-up mesh is handled with up_axis: +y in model.yaml, exactly like any other.
"""
import xml.etree.ElementTree as ET

import numpy as np


def dae_triangles(path):
    r = ET.parse(path).getroot()
    ns = {'c': r.tag.split('}')[0].strip('{')}
    unit = r.find('.//c:asset/c:unit', ns)
    scale = float(unit.get('meter')) if unit is not None else 1.0
    geo = {}
    for g in r.iterfind('.//c:library_geometries/c:geometry', ns):
        m = g.find('c:mesh', ns)
        if m is None:
            continue
        vid = m.find('c:vertices/c:input[@semantic="POSITION"]', ns).get('source')[1:]
        pos = np.array(m.find(f'c:source[@id="{vid}"]/c:float_array', ns).text.split(),
                       float).reshape(-1, 3)
        tris = []
        for prim in list(m.findall('c:triangles', ns)) + list(m.findall('c:polylist', ns)):
            inputs = prim.findall('c:input', ns)
            stride = max(int(i.get('offset')) for i in inputs) + 1
            voff = [int(i.get('offset')) for i in inputs if i.get('semantic') == 'VERTEX'][0]
            p = np.array(prim.find('c:p', ns).text.split(), int).reshape(-1, stride)[:, voff]
            if prim.tag.endswith('polylist'):
                k = 0
                for n in np.array(prim.find('c:vcount', ns).text.split(), int):
                    for q in range(1, n - 1):
                        tris.append((p[k], p[k + q], p[k + q + 1]))
                    k += n
            else:
                tris += list(p.reshape(-1, 3))
        geo[g.get('id')] = (pos, np.array(tris, dtype=int).reshape(-1, 3))
    out = []

    def walk(node, M):
        mt = node.find('c:matrix', ns)
        if mt is not None:
            M = M @ np.array(mt.text.split(), float).reshape(4, 4)
        for ig in node.findall('c:instance_geometry', ns):
            pos, t = geo[ig.get('url')[1:]]
            if len(t):
                h = (np.c_[pos, np.ones(len(pos))] @ M.T)[:, :3] * scale
                out.append(h[t])
        for ch in node.findall('c:node', ns):
            walk(ch, M)
    for n in r.iterfind('.//c:visual_scene/c:node', ns):
        walk(n, np.eye(4))
    return np.vstack(out)


def dae_bounds(path):
    """(min xyz, max xyz) in metres, as Gazebo would load it."""
    v = dae_triangles(path).reshape(-1, 3)
    return tuple(float(x) for x in v.min(0)), tuple(float(x) for x in v.max(0))
