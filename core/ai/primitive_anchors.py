"""Primitive-anchor math — Python mirror of src/src/linkGeometry.ts (WS5).

Used to render per-primitive anchor tables into the AI's spatial context so
the positions Claude reasons about match what the TS placement compiler will
produce. Pinned against the TS implementation by the shared parity corpus
(scripts/primitive-anchor-corpus.json) — change both sides together.

Conventions: link frame, millimetres, URDF fixed-axis rotation
R = Rz(yaw) @ Ry(pitch) @ Rx(roll). Anchor axes point OUTWARD.
"""
from __future__ import annotations

import math
from typing import Dict, List, Optional, Tuple

Vec3 = Tuple[float, float, float]

_BOX_ANCHORS = ["+x_face", "-x_face", "+y_face", "-y_face", "+z_face", "-z_face"]
_CYL_ANCHORS = ["+axis_end", "-axis_end", "tangent_+x", "tangent_-x",
                "tangent_+y", "tangent_-y", "tangent_+z", "tangent_-z"]
_SPHERE_ANCHORS = ["+x_pole", "-x_pole", "+y_pole", "-y_pole", "+z_pole", "-z_pole"]

_UNIT = {"x": (1.0, 0.0, 0.0), "y": (0.0, 1.0, 0.0), "z": (0.0, 0.0, 1.0)}


def _rpy_matrix(rpy) -> List[List[float]]:
    r, p, y = float(rpy[0]), float(rpy[1]), float(rpy[2])
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def _rotate(rot: List[List[float]], v: Vec3) -> Vec3:
    return (
        rot[0][0] * v[0] + rot[0][1] * v[1] + rot[0][2] * v[2],
        rot[1][0] * v[0] + rot[1][1] * v[1] + rot[1][2] * v[2],
        rot[2][0] * v[0] + rot[2][1] * v[1] + rot[2][2] * v[2],
    )


def _signed_unit(sign: str, axis: str) -> Optional[Vec3]:
    u = _UNIT.get(axis)
    if not u:
        return None
    s = 1.0 if sign == "+" else -1.0 if sign == "-" else 0.0
    if s == 0.0:
        return None
    return (u[0] * s, u[1] * s, u[2] * s)


def resolve_anchor(prim: Dict, anchor_name: str) -> Optional[Dict]:
    """Resolve an anchor on one primitive dict (linkGeometry wire format).

    Returns {"origin_xyz_mm": [...], "axis_xyz": [...]} or None when the
    anchor is invalid/undefined for the primitive."""
    shape = prim.get("shape")
    c = tuple(float(v) for v in (prim.get("xyz_mm") or [0, 0, 0]))
    rpy = prim.get("rpy") or [0, 0, 0]
    rot = _rpy_matrix(rpy if shape != "sphere" else [0, 0, 0])

    if shape == "box":
        if len(anchor_name) != 7 or not anchor_name.endswith("_face"):
            return None
        sign, axis = anchor_name[0], anchor_name[1]
        d = _signed_unit(sign, axis)
        s = prim.get("size_mm")
        if not d or not isinstance(s, list) or len(s) != 3:
            return None
        idx = {"x": 0, "y": 1, "z": 2}[axis]
        local = [0.0, 0.0, 0.0]
        local[idx] = (abs(float(s[idx])) / 2.0) * (1.0 if sign == "+" else -1.0)
        off = _rotate(rot, (local[0], local[1], local[2]))
        ax = _rotate(rot, d)
        return {
            "origin_xyz_mm": [c[0] + off[0], c[1] + off[1], c[2] + off[2]],
            "axis_xyz": list(ax),
        }

    if shape == "sphere":
        if len(anchor_name) != 7 or not anchor_name.endswith("_pole"):
            return None
        sign, axis = anchor_name[0], anchor_name[1]
        d = _signed_unit(sign, axis)
        if not d:
            return None
        r = abs(float(prim.get("radius_mm") or 0))
        return {
            "origin_xyz_mm": [c[0] + r * d[0], c[1] + r * d[1], c[2] + r * d[2]],
            "axis_xyz": list(d),
        }

    if shape == "cylinder":
        r = abs(float(prim.get("radius_mm") or 0))
        length = abs(float(prim.get("length_mm") or 0))
        a = _rotate(rot, (0.0, 0.0, 1.0))
        if anchor_name in ("+axis_end", "-axis_end"):
            s = 1.0 if anchor_name[0] == "+" else -1.0
            half = length / 2.0
            return {
                "origin_xyz_mm": [c[0] + half * a[0] * s, c[1] + half * a[1] * s, c[2] + half * a[2] * s],
                "axis_xyz": [a[0] * s, a[1] * s, a[2] * s],
            }
        if anchor_name.startswith("tangent_") and len(anchor_name) == 10:
            sign, axis = anchor_name[8], anchor_name[9]
            d = _signed_unit(sign, axis)
            if not d:
                return None
            d_dot_a = d[0] * a[0] + d[1] * a[1] + d[2] * a[2]
            radial = (d[0] - d_dot_a * a[0], d[1] - d_dot_a * a[1], d[2] - d_dot_a * a[2])
            ln = math.hypot(*radial)
            if ln < 1e-6:
                return None
            u = (radial[0] / ln, radial[1] / ln, radial[2] / ln)
            return {
                "origin_xyz_mm": [c[0] + r * u[0], c[1] + r * u[1], c[2] + r * u[2]],
                "axis_xyz": list(u),
            }
    return None


def anchor_names_for_primitive(prim: Dict) -> List[str]:
    shape = prim.get("shape")
    if shape == "box":
        return list(_BOX_ANCHORS)
    if shape == "sphere":
        return list(_SPHERE_ANCHORS)
    if shape == "cylinder":
        return [a for a in _CYL_ANCHORS if resolve_anchor(prim, a) is not None]
    return []


def render_anchor_table(link_name: str, primitives: List[Dict]) -> str:
    """Per-primitive anchor table for the AI's spatial context. Lists each
    named primitive's valid anchors with link-frame positions (mm)."""
    lines = [f"Primitive anchors on `{link_name}` (link-frame mm; use attach_primitive + attach_anchor):"]
    any_named = False
    for prim in primitives or []:
        name = prim.get("name")
        if not isinstance(name, str) or not name.strip():
            continue
        any_named = True
        anchors = []
        for a in anchor_names_for_primitive(prim):
            res = resolve_anchor(prim, a)
            if res:
                o = res["origin_xyz_mm"]
                anchors.append(f"{a}@[{o[0]:.0f},{o[1]:.0f},{o[2]:.0f}]")
        lines.append(f"  - {name} ({prim.get('shape')}): {', '.join(anchors)}")
    if not any_named:
        return ""
    return "\n".join(lines)


__all__ = ["resolve_anchor", "anchor_names_for_primitive", "render_anchor_table"]
