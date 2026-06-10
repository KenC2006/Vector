"""Headless CompiledGraph → URDF emitter.

The browser's URDF emission lives in urdfAssembly.ts and needs a DOM + scene
graph. The eval harness (scripts/eval_generation.py) needs a URDF for the
MuJoCo settle test without booting the app, so this module emits a physics
envelope URDF straight from the placement compiler's output:

  - one <link> per CompiledLink.physicalLinks entry
  - collision/visual = the component's resolved AABB box (or cylinder)
  - inertial from massKg + bounds (solid-primitive formulas)
  - one <joint> per CompiledLink.joints entry, verbatim

This is intentionally an ENVELOPE model — no meshes, no per-primitive
link_geometry detail. Good enough for "does it stand up" settle checks; not a
visual artifact. The split-servo body/horn mass ratio mirrors urdfAssembly's
95/5 split.

Input shape: the CompiledGraph dict produced by core.ai.compiler_client
.compile_assembly() — bounds in meters, joints carry originXyz/originRpy.
"""
from __future__ import annotations

import xml.etree.ElementTree as ET
from typing import Any, Dict, List

# Mirror of urdfAssembly.ts's split-servo mass allocation.
_SERVO_BODY_MASS_FRACTION = 0.95
# Links the resolver gave no mass (massKg <= 0) still need positive inertia
# for MuJoCo to accept the model.
_FALLBACK_MASS_KG = 0.05
_MIN_HALF_EXTENT_M = 0.002


def _fmt(v: float) -> str:
    return f"{float(v):.6f}"


def _box_inertia(mass: float, sx: float, sy: float, sz: float) -> Dict[str, float]:
    return {
        "ixx": mass / 12.0 * (sy * sy + sz * sz),
        "iyy": mass / 12.0 * (sx * sx + sz * sz),
        "izz": mass / 12.0 * (sx * sx + sy * sy),
    }


def _cylinder_inertia(mass: float, radius: float, height: float) -> Dict[str, float]:
    ixx = mass / 12.0 * (3.0 * radius * radius + height * height)
    return {"ixx": ixx, "iyy": ixx, "izz": mass / 2.0 * radius * radius}


def _add_link(
    robot: ET.Element,
    name: str,
    mass: float,
    half: List[float],
    center: List[float],
    shape: str,
) -> None:
    hx = max(abs(half[0]), _MIN_HALF_EXTENT_M)
    hy = max(abs(half[1]), _MIN_HALF_EXTENT_M)
    hz = max(abs(half[2]), _MIN_HALF_EXTENT_M)
    mass = mass if mass > 0 else _FALLBACK_MASS_KG

    link = ET.SubElement(robot, "link", name=name)
    inertial = ET.SubElement(link, "inertial")
    ET.SubElement(inertial, "mass", value=_fmt(mass))
    ET.SubElement(
        inertial, "origin",
        xyz=f"{_fmt(center[0])} {_fmt(center[1])} {_fmt(center[2])}", rpy="0 0 0",
    )
    if shape == "cylinder":
        inertia = _cylinder_inertia(mass, max(hx, hy), 2 * hz)
    else:
        inertia = _box_inertia(mass, 2 * hx, 2 * hy, 2 * hz)
    # Scientific notation, NOT the fixed 6-decimal _fmt: small/light output
    # discs (split-rotary horns) have tensor entries near 1e-6 — fixed-point
    # rounding corrupted the triangle inequality (ixx+iyy >= izz) and MuJoCo
    # rejected the model. Exact-formula values always satisfy it.
    ET.SubElement(
        inertial, "inertia",
        ixx=f"{inertia['ixx']:.9e}", iyy=f"{inertia['iyy']:.9e}", izz=f"{inertia['izz']:.9e}",
        ixy="0", ixz="0", iyz="0",
    )

    for tag in ("visual", "collision"):
        el = ET.SubElement(link, tag)
        ET.SubElement(
            el, "origin",
            xyz=f"{_fmt(center[0])} {_fmt(center[1])} {_fmt(center[2])}", rpy="0 0 0",
        )
        geom = ET.SubElement(el, "geometry")
        if shape == "cylinder":
            ET.SubElement(geom, "cylinder", radius=_fmt(max(hx, hy)), length=_fmt(2 * hz))
        else:
            ET.SubElement(geom, "box", size=f"{_fmt(2 * hx)} {_fmt(2 * hy)} {_fmt(2 * hz)}")
        if tag == "visual":
            mat = ET.SubElement(el, "material", name=f"mat_{name}")
            ET.SubElement(mat, "color", rgba="0.7 0.7 0.7 1")


def compiled_graph_to_urdf(compiled: Dict[str, Any], robot_name: str = "eval_robot") -> str:
    """Emit an envelope URDF string from a CompiledGraph dict."""
    links: List[Dict[str, Any]] = compiled.get("links") or []
    if not links:
        raise ValueError("compiled_graph_to_urdf: CompiledGraph has no links")

    robot = ET.Element("robot", name=robot_name)

    for cl in links:
        physical: List[str] = cl.get("physicalLinks") or [cl["logicalName"]]
        bounds = cl.get("bounds") or {}
        half = [float(v) for v in (bounds.get("half") or [0.02, 0.02, 0.02])]
        center = [float(v) for v in (bounds.get("center") or [0.0, 0.0, 0.0])]
        shape = str(bounds.get("shape") or "box")
        mass = float(cl.get("massKg") or 0.0)
        synthetic = cl.get("syntheticRole")
        child_attach = cl.get("childAttachTarget")

        if len(physical) == 1:
            _add_link(robot, physical[0], mass, half, center, shape)
        else:
            # Split servo: parent-frame-first ordering, horn is childAttachTarget.
            # Body carries the full envelope; horn gets a thin output puck;
            # optional carrier is a near-massless wrapper.
            for phys_name in physical:
                if phys_name == child_attach:
                    horn_half = [half[0] * 0.5, half[1] * 0.5, max(half[2] * 0.12, 0.004)]
                    _add_link(
                        robot, phys_name,
                        mass * (1.0 - _SERVO_BODY_MASS_FRACTION),
                        horn_half, [0.0, 0.0, 0.0], "cylinder",
                    )
                elif "carrier" in phys_name and len(physical) > 2:
                    _add_link(robot, phys_name, 0.01, [0.005, 0.005, 0.005], [0.0, 0.0, 0.0], "box")
                else:
                    body_half = [half[0], half[1], half[2] * 0.88]
                    _add_link(robot, phys_name, mass * _SERVO_BODY_MASS_FRACTION, body_half, center, shape)

        _ = synthetic  # syntheticRole not needed for envelope emission

        for j in cl.get("joints") or []:
            jtype = str(j.get("type") or "fixed")
            joint_el = ET.SubElement(robot, "joint", name=str(j["name"]), type=jtype)
            ET.SubElement(joint_el, "parent", link=str(j["parentLink"]))
            ET.SubElement(joint_el, "child", link=str(j["childLink"]))
            oxyz = j.get("originXyz") or [0, 0, 0]
            orpy = j.get("originRpy") or [0, 0, 0]
            ET.SubElement(
                joint_el, "origin",
                xyz=f"{_fmt(oxyz[0])} {_fmt(oxyz[1])} {_fmt(oxyz[2])}",
                rpy=f"{_fmt(orpy[0])} {_fmt(orpy[1])} {_fmt(orpy[2])}",
            )
            axis = j.get("axis") or [0, 0, 1]
            ET.SubElement(joint_el, "axis", xyz=f"{axis[0]} {axis[1]} {axis[2]}")
            if jtype in ("revolute", "prismatic"):
                limits = j.get("limits") or [-1.5708, 1.5708]
                ET.SubElement(
                    joint_el, "limit",
                    lower=_fmt(limits[0]), upper=_fmt(limits[1]),
                    effort=_fmt(j.get("effort") or 10.0),
                    velocity=_fmt(j.get("velocity") or 1.0),
                )

    _indent(robot)
    return '<?xml version="1.0"?>\n' + ET.tostring(robot, encoding="unicode")


def _indent(elem: ET.Element, level: int = 0) -> None:
    pad = "\n" + level * "  "
    if len(elem):
        if not elem.text or not elem.text.strip():
            elem.text = pad + "  "
        child = None
        for child in elem:
            _indent(child, level + 1)
        if child is not None and (not child.tail or not child.tail.strip()):
            child.tail = pad
        if not elem.tail or not elem.tail.strip():
            elem.tail = pad
    elif level and (not elem.tail or not elem.tail.strip()):
        elem.tail = pad


__all__ = ["compiled_graph_to_urdf"]
