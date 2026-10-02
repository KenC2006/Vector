"""
Designer compiler/critic regression tests.

Run: python -m core.designer.test_designer   (from the repo root)
"""
from __future__ import annotations

import math
import sys

import numpy as np

from core.designer.compile import DesignError, compile_design, extract_design
from core.designer.critic import critique
from core.designer.fk import link_frames

LEG = [
    {"name": "body", "shape": [{"name": "chest", "shape": "box", "size_mm": [300, 150, 80]}], "at": [0, 0, 250]},
    {"name": "hip_fl", "component": "actuator_servo_heavy_duty", "parent": "body",
     "at": {"ref": "body.chest.+y", "offset": [110, 0, 0]}, "align": "bottom", "mirror": True},
    {"name": "thigh_fl", "component": "structural_limb_link_slim", "length_mm": 110, "parent": "hip_fl",
     "at": {"ref": "hip_fl.shaft_out", "offset": [0, 4, 0]}, "align": "+z", "z_axis": "+z",
     "joint": {"type": "revolute", "rest_deg": -25, "lower_deg": -70, "upper_deg": 40}, "mirror": True},
    {"name": "foot_fl", "component": "mobility_rubber_foot_pad", "parent": "thigh_fl",
     "at": {"ref": "thigh_fl.-z"}, "align": "mount_top", "z_axis": "+z", "mirror": True},
]

CASES = []


def case(fn):
    CASES.append(fn)
    return fn


def design(parts):
    return {"name": "t", "summary": "", "parts": parts}


@case
def urdf_matches_compiler_world_model():
    """The emitted URDF, run through standard URDF forward kinematics, puts
    every link exactly where the compiler says the part is."""
    asm = compile_design(design(LEG))
    frames = link_frames(asm.to_urdf())
    root = asm.parts[asm.root]
    off = root.frame_p - frames[root.link][0]
    assert len(frames) == len(asm.parts), "one URDF link per part (no helper links)"
    for q in asm.parts.values():
        p, R = frames[q.link]
        assert np.linalg.norm(p + off - q.frame_p) < 1e-3, f"{q.name} position drift"
        if q.joint and "pivot" in q.joint:
            assert np.linalg.norm(q.frame_p - q.joint["pivot"]) < 1e-9, f"{q.name} link frame not on its joint"
        assert np.abs(R - q.R).max() < 1e-5, f"{q.name} rotation drift"


@case
def mirror_creates_opposite_side():
    asm = compile_design(design(LEG))
    l, r = asm.parts["thigh_fl"], asm.parts["thigh_fr"]
    assert r.parent == "hip_fr" and r.mirror_of == "thigh_fl"
    assert abs(l.p[1] + r.p[1]) < 1e-6 and l.p[1] > 0
    # Axial joint axes: both sides get the same axis so +angle is the same motion.
    assert np.allclose(l.joint["axis"], r.joint["axis"])


@case
def mate_puts_servo_flush_and_shaft_outward():
    asm = compile_design(design(LEG))
    hip = asm.parts["hip_fl"]
    assert np.allclose(hip.R[:, 2], [0, 1, 0]), "shaft should point +Y (outward)"
    body = asm.parts["body"]
    face_y = body.p[1] + 75
    assert abs((hip.p[1] - hip.size[2] / 2) - face_y) < 1e-6, "servo bottom should sit on the body side"


@case
def rest_angle_bends_subtree_and_limits_shift():
    asm = compile_design(design(LEG))
    thigh, foot = asm.parts["thigh_fl"], asm.parts["foot_fl"]
    pivot = thigh.joint["pivot"]
    assert foot.p[0] > pivot[0] + 20, "a -25 deg rest about +Y swings the foot forward"
    assert math.isclose(thigh.joint["lower"], math.radians(-70 + 25), abs_tol=1e-9)
    assert math.isclose(thigh.joint["upper"], math.radians(40 + 25), abs_tol=1e-9)


@case
def rest_outside_limits_is_rejected():
    bad = [dict(p) for p in LEG]
    bad[2] = dict(bad[2], joint={"type": "revolute", "rest_deg": 50, "lower_deg": -10, "upper_deg": 10})
    try:
        compile_design(design(bad))
    except DesignError as e:
        assert "outside its limits" in str(e)
        return
    raise AssertionError("expected DesignError")


@case
def grounded_feet_and_design_roundtrip():
    asm = compile_design(design(LEG))
    rep = critique(asm)
    assert set(rep["ground_contacts"]) == {"foot_fl", "foot_fr"}, rep["ground_contacts"]
    assert extract_design(asm.to_urdf())["parts"][0]["name"] == "body"


@case
def critic_flags_floating_part():
    parts = LEG + [{"name": "lidar", "component": "sensor_lidar_2d", "parent": "body",
                    "at": {"ref": "body.chest.+z", "offset": [0, 0, 40]}, "align": "mount_back"}]
    issues = critique(compile_design(design(parts)))["issues"]
    assert any(i.startswith("FLOATING: lidar") for i in issues), issues


@case
def critic_flags_overloaded_leg_joint():
    weak = [dict(p) for p in LEG]
    weak[1] = dict(weak[1], component="actuator_servo_micro")
    issues = critique(compile_design(design(weak)))["issues"]
    assert any(i.startswith("OVERLOADED: thigh_fl") for i in issues), issues


@case
def unknown_anchor_lists_valid_options():
    bad = [dict(p) for p in LEG]
    bad[1] = dict(bad[1], at={"ref": "body.chest.+q"})
    try:
        compile_design(design(bad))
    except DesignError as e:
        assert "+x/-x/+y/-y/+z/-z" in str(e)
        return
    raise AssertionError("expected DesignError")


# ── import / merge: the URDF and the design never disagree ──────────────────

HAND_URDF = """<?xml version="1.0"?>
<robot name="hand">
  <material name="blue"><color rgba="0 0 0.8 1"/></material>
  <link name="base_link"/>
  <link name="body"><visual><origin xyz="0 0 0.05" rpy="0.1 0.2 0.3"/><geometry><box size="0.3 0.2 0.1"/></geometry><material name="blue"/></visual>
    <inertial><mass value="2.5"/><inertia ixx="1" iyy="1" izz="1" ixy="0" ixz="0" iyz="0"/></inertial></link>
  <link name="arm"><visual><origin xyz="0 0 0.1"/><geometry><cylinder radius="0.02" length="0.2"/></geometry></visual>
     <visual><geometry><mesh filename="package://foo/claw.stl" scale="0.001 0.001 0.001"/></geometry></visual></link>
  <link name="slider"><visual><geometry><sphere radius="0.03"/></geometry></visual></link>
  <joint name="fix" type="fixed"><parent link="base_link"/><child link="body"/><origin xyz="0 0 0.2" rpy="0 0 1.2"/></joint>
  <joint name="shoulder" type="revolute"><parent link="body"/><child link="arm"/><origin xyz="0.1 0 0.1" rpy="0.4 -0.3 0.2"/><axis xyz="0 1 1"/><limit lower="0.2" upper="1.0" effort="7" velocity="2"/></joint>
  <joint name="slide" type="prismatic"><parent link="arm"/><child link="slider"/><origin xyz="0 0 0.2"/><axis xyz="0 0 1"/><limit lower="0" upper="0.05" effort="3" velocity="1"/></joint>
</robot>"""


def _visual_poses(urdf):
    import xml.etree.ElementTree as ET
    from core.designer.geometry import rpy_to_matrix
    fr = link_frames(urdf)
    root = ET.fromstring(urdf)
    out = {}
    for l in root.findall("link"):
        p, R = fr[l.get("name")]
        for k, v in enumerate(l.findall("visual")):
            o = v.find("origin")
            xyz = np.array([float(x) for x in (o.get("xyz", "0 0 0") if o is not None else "0 0 0").split()]) * 1000
            rpy = [float(x) for x in (o.get("rpy", "0 0 0") if o is not None else "0 0 0").split()]
            out[(l.get("name"), k)] = (p + R @ xyz, R @ rpy_to_matrix(rpy))
    return out


def _assert_same_geometry(a_urdf, b_urdf):
    a, b = _visual_poses(a_urdf), _visual_poses(b_urdf)
    k0 = next(iter(a))
    assert k0 in b, f"{k0} missing"
    oa, ob = a[k0][0], b[k0][0]
    for k in a:
        assert k in b, f"visual {k} missing after round trip"
        assert np.linalg.norm((a[k][0] - oa) - (b[k][0] - ob)) < 0.02, f"{k} moved"
        assert np.abs(a[k][1] - b[k][1]).max() < 1e-4, f"{k} rotated"


@case
def import_hand_written_urdf_is_lossless():
    from core.designer.importer import import_urdf
    imp = import_urdf(HAND_URDF)
    out = compile_design(imp["design"]).to_urdf("hand")
    _assert_same_geometry(HAND_URDF, out)
    assert 'name="shoulder"' in out and 'name="slide"' in out, "joint names kept"
    assert 'effort="7.000"' in out, "ratings kept"
    assert "claw.stl" in out, "mesh kept"


@case
def untouched_output_keeps_its_design():
    from core.designer.importer import import_urdf
    d = design(LEG)
    imp = import_urdf(compile_design(d).to_urdf("t"))
    assert imp["imported"] is False and imp["design"] == d


@case
def hand_edit_merges_into_design():
    """Moving one mirrored part by hand keeps that edit exactly and leaves
    every other part's mate intact."""
    import re
    from core.designer.importer import import_urdf
    urdf = compile_design(design(LEG)).to_urdf("t")
    m = re.search(r'(<joint name="foot_fl_joint"[^>]*>.*?<origin xyz=")([^"]+)(")', urdf, re.S)
    xyz = [float(v) for v in m.group(2).split()]
    xyz[0] += 0.01
    edited = urdf[:m.start(2)] + " ".join(map(str, xyz)) + urdf[m.end(2):]
    imp = import_urdf(edited)
    parts = {p["name"]: p for p in imp["design"]["parts"]}
    assert isinstance(parts["foot_fl"]["at"], list), "edited part takes its pose from the URDF"
    assert isinstance(parts["thigh_fl"]["at"], dict), "untouched part keeps its mate"
    assert "foot_fr" in parts and not parts["foot_fl"].get("mirror"), "edited pair is un-mirrored"
    _assert_same_geometry(edited, compile_design(imp["design"]).to_urdf("t"))


@case
def linear_actuator_drives_prismatic_joint():
    d = design([
        {"name": "base", "shape": [{"name": "b", "shape": "box", "size_mm": [200, 200, 20]}]},
        {"name": "lin", "component": "actuator_linear_small", "parent": "base", "at": "base.+z", "align": "-z"},
        {"name": "plate", "shape": [{"name": "p", "shape": "box", "size_mm": [40, 40, 5]}], "parent": "lin",
         "at": "lin.rod_out", "align": "-z", "joint": {"type": "prismatic"}},
    ])
    j = compile_design(d).parts["plate"].joint
    assert not j["passive"] and j["effort"] > 10 and j["upper"] > 0.02, j


@case
def every_joint_hand_edit_merges_exactly():
    """Move or turn any single joint of a real robot by hand: the merged
    design reproduces the edited URDF exactly (demo dog, every joint)."""
    import os
    import re
    from core.designer.compile import extract_design
    from core.designer.importer import import_urdf
    path = os.path.join(os.path.dirname(__file__), "..", "..", "src", "public", "demos", "dog.urdf")
    fresh = compile_design(extract_design(open(path, encoding="utf-8").read())).to_urdf("robot")
    for jn in re.findall(r'<joint name="(\w+)"', fresh):
        for attr, bump in (("xyz", (0, 0, 0.005)), ("rpy", (0.05, 0, -0.03))):
            m = re.search(r'(<joint name="%s"[^>]*>.*?<origin[^>]*? %s=")([^"]+)(")' % (jn, attr), fresh, re.S)
            vals = [float(v) + d for v, d in zip(m.group(2).split(), bump)]
            edited = fresh[:m.start(2)] + " ".join(map(str, vals)) + fresh[m.end(2):]
            try:
                _assert_same_geometry(edited, compile_design(import_urdf(edited)["design"]).to_urdf("robot"))
            except AssertionError as e:
                raise AssertionError(f"{jn} {attr}: {e}")


def main() -> int:
    failed = 0
    for fn in CASES:
        try:
            fn()
            print(f"  PASS  {fn.__name__}")
        except Exception as e:  # noqa: BLE001 — report every failure
            failed += 1
            print(f"  FAIL  {fn.__name__}: {e}")
    print(f"\ndesigner corpus: {len(CASES) - failed}/{len(CASES)} passed")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
