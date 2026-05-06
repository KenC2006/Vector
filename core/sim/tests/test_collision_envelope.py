"""Phase 5b — collision envelope round-trip through urdf_to_mjcf.

The TS URDF emitter writes link collision as a single ``<box>`` whose size and
origin match ``resolved.collision.bounds`` (the canonical AABB the placement
compiler reads). MuJoCo contact must agree with placement, so the Python
converter must preserve that box exactly — same half-size, same offset, no
silent re-derivation from visual primitives.

This test builds a minimal URDF programmatically with a known box collision
and asserts the generated MJCF has one ``<geom type="box">`` with matching
``size`` (half-extents) and ``pos`` (centre offset).

Run as ``python -m core.sim.tests.test_collision_envelope`` from the repo root.
"""

from __future__ import annotations

import os
import sys
import tempfile

from lxml import etree

from core.sim.urdf_to_mjcf import urdf_to_mjcf


def _make_urdf(size_xyz, origin_xyz) -> str:
    sx, sy, sz = size_xyz
    ox, oy, oz = origin_xyz
    return f"""<?xml version="1.0"?>
<robot name="probe">
  <link name="root">
    <inertial>
      <mass value="0.1"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="1e-4" iyy="1e-4" izz="1e-4" ixy="0" ixz="0" iyz="0"/>
    </inertial>
    <visual>
      <origin xyz="{ox} {oy} {oz}" rpy="0 0 0"/>
      <geometry><box size="{sx} {sy} {sz}"/></geometry>
    </visual>
    <collision>
      <origin xyz="{ox} {oy} {oz}" rpy="0 0 0"/>
      <geometry><box size="{sx} {sy} {sz}"/></geometry>
    </collision>
  </link>
</robot>
"""


def _collision_geom_for_link(mjcf_xml: str, link_name: str) -> etree._Element | None:
    root = etree.fromstring(mjcf_xml.encode("utf-8"))
    for body in root.iter("body"):
        if body.get("name") == link_name:
            for geom in body.findall("geom"):
                if geom.get("type") in ("box", "cylinder", "sphere", "mesh"):
                    return geom
    return None


def _approx(a: float, b: float, tol: float = 1e-6) -> bool:
    return abs(a - b) <= tol


def main() -> int:
    cases = [
        # (name, full_size_xyz, origin_xyz)
        ("centred_unit",      (0.040, 0.020, 0.040), (0.000,  0.000,  0.000)),
        ("offset_z",          (0.054, 0.020, 0.040), (0.000,  0.000,  0.012)),
        ("offset_xz_servo",   (0.040, 0.020, 0.038), (0.003,  0.000, -0.001)),
    ]

    failed: list[tuple[str, str]] = []
    for name, size_xyz, origin_xyz in cases:
        with tempfile.NamedTemporaryFile(
            mode="w", suffix=".urdf", delete=False, encoding="utf-8",
        ) as fp:
            fp.write(_make_urdf(size_xyz, origin_xyz))
            urdf_path = fp.name
        try:
            mjcf_xml = urdf_to_mjcf(urdf_path)
        finally:
            os.unlink(urdf_path)

        geom = _collision_geom_for_link(mjcf_xml, "root")
        if geom is None:
            failed.append((name, "no <geom> emitted under root body"))
            print(f"  FAIL  {name}: no geom", file=sys.stderr)
            continue

        if geom.get("type") != "box":
            failed.append((name, f"expected box, got {geom.get('type')}"))
            print(f"  FAIL  {name}: type={geom.get('type')}", file=sys.stderr)
            continue

        # MuJoCo box size is half-extents; URDF box size is full extents.
        size_str = geom.get("size", "")
        try:
            half = [float(v) for v in size_str.split()]
        except ValueError:
            failed.append((name, f"unparsable size={size_str!r}"))
            continue
        expected_half = [v / 2.0 for v in size_xyz]
        if len(half) != 3 or not all(_approx(h, e) for h, e in zip(half, expected_half)):
            failed.append((name, f"size half={half} expected={expected_half}"))
            print(f"  FAIL  {name}: size {half} != {expected_half}", file=sys.stderr)
            continue

        pos_str = geom.get("pos", "0 0 0")
        try:
            pos = [float(v) for v in pos_str.split()]
        except ValueError:
            failed.append((name, f"unparsable pos={pos_str!r}"))
            continue
        if len(pos) != 3 or not all(_approx(p, e) for p, e in zip(pos, origin_xyz)):
            failed.append((name, f"pos={pos} expected={list(origin_xyz)}"))
            print(f"  FAIL  {name}: pos {pos} != {list(origin_xyz)}", file=sys.stderr)
            continue

        print(f"  PASS  {name}")

    if failed:
        print(f"collision-envelope: {len(failed)} failed", file=sys.stderr)
        return 1
    print(f"collision-envelope: {len(cases)}/{len(cases)} passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
