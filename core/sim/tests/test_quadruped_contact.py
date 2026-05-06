"""Phase 5b — quadruped foot-contact invariant.

Builds a 4-leg URDF programmatically (chassis + 4 legs + 4 foot pads),
runs it through ``urdf_to_mjcf``, and asserts the four lowest box-collision
geom bottoms (in world Z) are exactly the four foot pads with strict
separation from the next-lowest geom.

This guards the "feet touch the ground, not the chassis" invariant for the
canonical quadruped topology — the most common way bbox/joint-origin drift
manifests in MuJoCo is the chassis or a leg shaft becoming the lowest
contact instead of the foot.

Run as ``python -m core.sim.tests.test_quadruped_contact`` from the repo root.
"""

from __future__ import annotations

import os
import sys
import tempfile

from lxml import etree

from core.sim.urdf_to_mjcf import urdf_to_mjcf


# Geometry (metres). Chosen so foot bottoms sit well below leg bottoms.
CHASSIS_HALF = (0.080, 0.060, 0.030)   # 160x120x60 mm
LEG_HALF     = (0.010, 0.010, 0.040)   # 20x20x80 mm  (leg length = 80mm)
FOOT_HALF    = (0.020, 0.020, 0.010)   # 40x40x20 mm

LEG_DX = 0.060   # leg X offset from chassis centre
LEG_DY = 0.040   # leg Y offset from chassis centre

FOOT_LINKS = ("foot_fl", "foot_fr", "foot_rl", "foot_rr")


def _make_quadruped_urdf() -> str:
    cx, cy, cz = CHASSIS_HALF
    lx, ly, lz = LEG_HALF
    fx, fy, fz = FOOT_HALF

    # Joint origin from chassis centre to top of leg link:
    # leg sits hanging from a corner at the chassis bottom face.
    leg_corners = [
        ("fl",  LEG_DX,  LEG_DY),
        ("fr",  LEG_DX, -LEG_DY),
        ("rl", -LEG_DX,  LEG_DY),
        ("rr", -LEG_DX, -LEG_DY),
    ]

    parts = [
        '<?xml version="1.0"?>',
        '<robot name="quadruped">',
        '  <link name="chassis">',
        '    <inertial><mass value="1.0"/>',
        '      <origin xyz="0 0 0" rpy="0 0 0"/>',
        '      <inertia ixx="1e-3" iyy="1e-3" izz="1e-3" ixy="0" ixz="0" iyz="0"/></inertial>',
        f'    <collision><origin xyz="0 0 0" rpy="0 0 0"/>',
        f'      <geometry><box size="{2*cx} {2*cy} {2*cz}"/></geometry></collision>',
        '  </link>',
    ]

    for tag, dx, dy in leg_corners:
        leg = f"leg_{tag}"
        foot = f"foot_{tag}"
        # leg link: collision centred at link origin; joint puts link origin
        # at (dx, dy, -cz - lz)  → leg top flush with chassis bottom face.
        parts += [
            f'  <link name="{leg}">',
            f'    <inertial><mass value="0.05"/><origin xyz="0 0 0" rpy="0 0 0"/>',
            f'      <inertia ixx="1e-5" iyy="1e-5" izz="1e-5" ixy="0" ixz="0" iyz="0"/></inertial>',
            f'    <collision><origin xyz="0 0 0" rpy="0 0 0"/>',
            f'      <geometry><box size="{2*lx} {2*ly} {2*lz}"/></geometry></collision>',
            f'  </link>',
            f'  <joint name="hip_{tag}" type="fixed">',
            f'    <parent link="chassis"/><child link="{leg}"/>',
            f'    <origin xyz="{dx} {dy} {-cz - lz}" rpy="0 0 0"/>',
            f'  </joint>',
            f'  <link name="{foot}">',
            f'    <inertial><mass value="0.02"/><origin xyz="0 0 0" rpy="0 0 0"/>',
            f'      <inertia ixx="1e-6" iyy="1e-6" izz="1e-6" ixy="0" ixz="0" iyz="0"/></inertial>',
            f'    <collision><origin xyz="0 0 0" rpy="0 0 0"/>',
            f'      <geometry><box size="{2*fx} {2*fy} {2*fz}"/></geometry></collision>',
            f'  </link>',
            f'  <joint name="ankle_{tag}" type="fixed">',
            f'    <parent link="{leg}"/><child link="{foot}"/>',
            f'    <origin xyz="0 0 {-lz - fz}" rpy="0 0 0"/>',
            f'  </joint>',
        ]

    parts.append('</robot>')
    return "\n".join(parts)


def _parse_xyz(s: str | None, default=(0.0, 0.0, 0.0)) -> tuple[float, float, float]:
    if not s:
        return default
    try:
        v = [float(x) for x in s.split()]
        return (v[0], v[1], v[2]) if len(v) == 3 else default
    except ValueError:
        return default


def _walk_box_geoms(root: etree._Element) -> list[tuple[str, float]]:
    """Return [(body_name, world_z_of_box_bottom)] for every box geom."""
    out: list[tuple[str, float]] = []

    def visit(body: etree._Element, parent_world_z: float) -> None:
        body_pos_z = _parse_xyz(body.get("pos"))[2]
        world_z = parent_world_z + body_pos_z
        body_name = body.get("name") or "<unnamed>"
        for geom in body.findall("geom"):
            if geom.get("type") != "box":
                continue
            size = _parse_xyz(geom.get("size"))   # MuJoCo box size = half-extents
            geom_pos_z = _parse_xyz(geom.get("pos"))[2]
            bottom_z = world_z + geom_pos_z - size[2]
            out.append((body_name, bottom_z))
        for child in body.findall("body"):
            visit(child, world_z)

    worldbody = root.find(".//worldbody")
    if worldbody is None:
        return out
    for body in worldbody.findall("body"):
        visit(body, 0.0)
    return out


def main() -> int:
    urdf_text = _make_quadruped_urdf()
    with tempfile.NamedTemporaryFile(
        mode="w", suffix=".urdf", delete=False, encoding="utf-8",
    ) as fp:
        fp.write(urdf_text)
        urdf_path = fp.name
    try:
        mjcf_xml = urdf_to_mjcf(urdf_path)
    finally:
        os.unlink(urdf_path)

    root = etree.fromstring(mjcf_xml.encode("utf-8"))
    geoms = _walk_box_geoms(root)
    if len(geoms) < 5:
        print(
            f"quadruped-contact: expected ≥5 box geoms, got {len(geoms)}: {geoms}",
            file=sys.stderr,
        )
        return 1

    geoms.sort(key=lambda gb: gb[1])
    lowest_four = {name for name, _ in geoms[:4]}
    expected = set(FOOT_LINKS)
    if lowest_four != expected:
        print(
            "quadruped-contact: lowest 4 geoms are not the foot pads.\n"
            f"  expected: {sorted(expected)}\n"
            f"  got:      {sorted(lowest_four)}\n"
            f"  full sorted list: {geoms}",
            file=sys.stderr,
        )
        return 1

    foot_z = geoms[3][1]
    next_z = geoms[4][1]
    if next_z - foot_z < 1e-4:
        print(
            f"quadruped-contact: insufficient separation between feet ({foot_z:.4f}) "
            f"and next geom ({next_z:.4f})",
            file=sys.stderr,
        )
        return 1

    print(
        f"quadruped-contact: 4 foot pads are the lowest contact geoms "
        f"(foot_z={foot_z:.4f}, next={next_z:.4f})"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
