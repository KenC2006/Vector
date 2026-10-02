"""
Catalog-driven sim/critic behaviour: part identity, actuator ratings, contact
friction classes, servo gains, spawn height, sandbox print, and the critic's
mirrored-clash count.

Run: python -m core.sim.tests.test_catalog_physics   (from the repo root)
"""
from __future__ import annotations

import contextlib
import io
import json
import os
import sys
import tempfile

import numpy as np

from core.presets import actuator_rating, get_component, list_components
from core.presets.actuation import derive_actuation
from core.presets.identity import CONTACT_CLASSES, PartIdentity, contact_class_of

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

CASES = []


def case(fn):
    CASES.append(fn)
    return fn


def _parts_comment(mapping) -> str:
    return f"<!-- vector:parts {json.dumps(mapping, separators=(',', ':'))} -->"


def _mjcf(urdf: str, terrain=None) -> str:
    from core.sim.urdf_to_mjcf import urdf_to_mjcf
    fd, path = tempfile.mkstemp(suffix=".urdf")
    with os.fdopen(fd, "w", encoding="utf-8") as f:
        f.write(urdf)
    try:
        return urdf_to_mjcf(path, free_base=True, terrain_config=terrain)
    finally:
        os.remove(path)


def _link(name, geom='<box size="0.04 0.04 0.02"/>', z=0.0, mass=0.1):
    return (f'<link name="{name}"><inertial><mass value="{mass}"/>'
            f'<inertia ixx="1e-4" iyy="1e-4" izz="1e-4" ixy="0" ixz="0" iyz="0"/></inertial>'
            f'<collision><origin xyz="0 0 {z}"/><geometry>{geom}</geometry></collision></link>')


def _fixed(name, parent, child, xyz="0 0 0"):
    return (f'<joint name="{name}" type="fixed"><parent link="{parent}"/><child link="{child}"/>'
            f'<origin xyz="{xyz}"/></joint>')


# ── identity ────────────────────────────────────────────────────────────────

@case
def identity_uses_parts_map_then_convention():
    urdf = _parts_comment({"console": {"part": "console", "component": "mobility_rubber_foot_pad"},
                           "shell": {"part": "shell", "component": None}})
    ident = PartIdentity(urdf)
    assert ident.component_id("console") == "mobility_rubber_foot_pad"
    assert ident.contact_class("console") == "foot"
    assert ident.component_id("shell") is None
    # With a map, links outside it are custom bodies — no name guessing.
    assert ident.component_id("mobility_wheel_driven_1") is None
    # Without a map: <component_id>_<N> only for real catalog ids.
    bare = PartIdentity("<robot/>")
    assert bare.component_id("mobility_wheel_driven_3") == "mobility_wheel_driven"
    assert bare.contact_class("mobility_wheel_driven_3") == "wheel"
    assert bare.component_id("foot_fl") is None and bare.contact_class("foot_fl") == "none"
    assert bare.component_id("left_sole_2") is None
    assert bare.component_id("effector_vacuum_pad_array") == "effector_vacuum_pad_array"
    assert PartIdentity("<!-- vector:parts {not json} -->").part_map is None


@case
def catalog_contact_classes_are_valid():
    classes = {}
    for comp in list_components():
        cls = (comp.get("sim_metadata") or {}).get("contact_class")
        if cls is not None:
            assert cls in CONTACT_CLASSES + ("drivetrain",), (comp["id"], cls)
        classes[comp["id"]] = contact_class_of(comp)
    assert classes["mobility_rubber_foot_pad"] == "foot"
    assert classes["effector_vacuum_pad_array"] == "suction"
    assert classes["effector_parallel_gripper_small"] == "gripper"
    assert classes["mobility_ball_transfer_unit"] == "caster"
    assert classes["mobility_track_tread_system"] == "track"
    assert classes["drivetrain_hub_motor_80"] == "none"   # hardware, not a contact surface
    assert classes["structural_baseplate"] == "none"


# ── actuator ratings ────────────────────────────────────────────────────────

@case
def actuator_rating_covers_rotary_and_linear():
    kind, effort, speed = actuator_rating(get_component("actuator_servo_micro"))
    assert kind == "rotary" and effort == 0.18 and abs(speed - 60 * 2 * np.pi / 60) < 1e-4
    kind, effort, speed = actuator_rating(get_component("actuator_linear_small"))
    assert kind == "linear" and effort == 50.0 and abs(speed - 0.015) < 1e-9
    assert get_component("actuator_linear_small")["actuation"]["stroke_mm"] == 50.0
    # Gearmotors: rated output torque, not stall.
    assert actuator_rating(get_component("motor_gear_medium_37mm"))[1] == 2.5
    # Brushless without a torque spec: Kt = 60/(2π·Kv) × continuous current.
    kind, effort, _ = actuator_rating(get_component("motor_brushless_inrunner_medium"))
    me = get_component("motor_brushless_inrunner_medium")["mechanical_electrical"]
    assert kind == "rotary" and abs(effort - 60 / (2 * np.pi * me["kv_rpm_per_v"]) * me["max_continuous_current_a"]) < 1e-5
    # Unpowered parts: passive axles, clamps with a "holding torque", suction.
    for cid in ("drivetrain_stub_axle_passive", "structural_shaft_collar", "effector_suction_cup",
                "transmission_spur_gear_pair", "structural_baseplate"):
        assert actuator_rating(get_component(cid)) is None, cid
    assert actuator_rating(None) is None


@case
def catalog_actuation_blocks_are_current():
    for comp in list_components():
        derived = derive_actuation(dict(comp, actuation=None))
        assert comp.get("actuation") == derived, comp["id"]
        if (comp.get("sim_metadata") or {}).get("mjcf_actuator_type") in ("position", "velocity", "motor") \
                and comp["id"] not in ("drivetrain_steering_knuckle", "effector_3finger_adaptive",
                                       "effector_soft_gripper"):
            assert derived is not None, f"{comp['id']} is powered but has no rating"
    mirror = os.path.join(ROOT, "src", "public", "generic_presets.json")
    core = os.path.join(ROOT, "core", "presets", "generic_presets.json")
    with open(mirror, "rb") as a, open(core, "rb") as b:
        assert a.read() == b.read(), "src/public/generic_presets.json differs from the core catalog"


# ── MJCF contact classes / friction ─────────────────────────────────────────

def _classes_by_body(mjcf: str):
    from lxml import etree
    root = etree.fromstring(mjcf.encode())
    return {b.get("name"): [g.get("class") for g in b.findall("geom")] for b in root.iter("body")}


@case
def friction_class_comes_from_catalog_not_names():
    urdf = ("<robot name='r'>" + _parts_comment({
        "base": {"part": "base", "component": None},
        "console": {"part": "console", "component": None},
        "pad": {"part": "pad", "component": "effector_vacuum_pad_array"},
        "shoe": {"part": "shoe", "component": "mobility_rubber_foot_pad"},
        "finger_tip": {"part": "finger_tip", "component": None},
    }) + _link("base") + _link("console") + _link("pad") + _link("shoe") + _link("finger_tip")
        + _fixed("j1", "base", "console") + _fixed("j2", "base", "pad") + _fixed("j3", "base", "shoe")
        + _fixed("j4", "base", "finger_tip") + "</robot>")
    classes = _classes_by_body(_mjcf(urdf))
    assert classes["console"] == [None], classes    # 'sole' in 'console' is not a foot
    assert classes["finger_tip"] == [None], classes  # 'tip'/'finger' are not roles
    assert classes["pad"] == ["suction"], classes    # vacuum pad: not foot friction
    assert classes["shoe"] == ["foot"], classes


@case
def class_friction_wins_over_terrain():
    import mujoco
    from core.sim.urdf_to_mjcf import NOMINAL_TERRAIN_FRICTION
    urdf = ("<robot name='r'>" + _parts_comment({"w": {"part": "w", "component": "mobility_wheel_driven"}})
            + _link("w", '<sphere radius="0.03"/>') + "</robot>")
    for terrain_mu in (NOMINAL_TERRAIN_FRICTION, 0.6):
        m = mujoco.MjModel.from_xml_string(_mjcf(urdf, {"friction": terrain_mu}))
        d = mujoco.MjData(m)
        d.qpos[2] = 0.0   # centre on the floor plane → in contact
        mujoco.mj_forward(m, d)
        assert d.ncon >= 1
        mu = float(d.contact[0].friction[0])
        expect = 1.2 * terrain_mu / NOMINAL_TERRAIN_FRICTION
        assert abs(mu - expect) < 1e-6, (terrain_mu, mu, expect)


@case
def servo_gains_scale_with_rating():
    import mujoco
    from core.sim.urdf_to_mjcf import SERVO_SATURATION_RAD
    urdf = ("<robot name='r'>" + _parts_comment({
        "base": {"part": "base", "component": None},
        "servo": {"part": "servo", "component": "actuator_servo_micro"},
        "arm": {"part": "arm", "component": None},
    }) + _link("base") + _link("servo") + _link("arm", mass=0.01)
        + _fixed("j0", "base", "servo")
        # No effort in the URDF: the rating comes from the driving servo.
        + '<joint name="elbow" type="revolute"><parent link="servo"/><child link="arm"/>'
          '<axis xyz="0 1 0"/><limit lower="-1" upper="1" velocity="0"/></joint>'
        + "</robot>")
    m = mujoco.MjModel.from_xml_string(_mjcf(urdf))
    a = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, "elbow_pos")
    kp = float(m.actuator_gainprm[a][0])
    assert abs(kp - 0.18 / SERVO_SATURATION_RAD) < 1e-6, kp
    assert abs(float(m.actuator_forcerange[a][1]) - 0.18) < 1e-9
    assert float(m.actuator_biasprm[a][2]) < 0, "dampratio should give a damping bias"


# ── spawn ───────────────────────────────────────────────────────────────────

@case
def mesh_robot_spawns_on_the_floor():
    import mujoco
    from core.sim.control import load_sim_from_text
    from core.sim.mujoco_adapter import SPAWN_CLEARANCE
    mesh = "package://meshes/collision/actuator_high_speed_mini_servo_collision.obj"
    urdf = ("<robot name='r'>" + _link("body", f'<mesh filename="{mesh}"/>')
            + "</robot>")
    for free in (True, False):
        sim = load_sim_from_text(urdf, free_base=free)
        m, d = sim.model, sim.data
        mujoco.mj_kinematics(m, d)
        g = next(i for i in range(m.ngeom) if int(m.geom_type[i]) == int(mujoco.mjtGeom.mjGEOM_MESH))
        adr, num = int(m.mesh_vertadr[m.geom_dataid[g]]), int(m.mesh_vertnum[m.geom_dataid[g]])
        world = d.geom_xpos[g] + m.mesh_vert[adr:adr + num] @ d.geom_xmat[g].reshape(3, 3).T
        assert abs(float(world[:, 2].min()) - SPAWN_CLEARANCE) < 1e-6, (free, world[:, 2].min())


# ── sandbox ─────────────────────────────────────────────────────────────────

@case
def script_print_never_reaches_stdout():
    from core.sim.control import compile_script
    fn = compile_script("def step(t, state):\n    print('hello', t, sep='-')\n    return {}\n")
    out, err = io.StringIO(), io.StringIO()
    with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
        fn(1.5, {})
    assert out.getvalue() == "", out.getvalue()
    assert "hello-1.5" in err.getvalue()


# ── critic ──────────────────────────────────────────────────────────────────

@case
def critic_reports_mirrored_clash_once():
    from core.designer.compile import compile_design
    from core.designer.critic import critique
    asm = compile_design({"name": "t", "summary": "", "parts": [
        {"name": "hull", "shape": [{"name": "box", "shape": "box", "size_mm": [300, 200, 40]}], "at": [0, 0, 20]},
        {"name": "deck", "shape": [{"name": "box", "shape": "box", "size_mm": [200, 200, 20]}],
         "parent": "hull", "at": [0, 0, 50]},
        {"name": "post", "shape": [{"name": "box", "shape": "box", "size_mm": [20, 20, 60]}],
         "parent": "hull", "at": [0, 60, 60], "mirror": True},
    ]})
    clashes = [s for s in critique(asm)["issues"] if s.startswith("CLIPPING") and "deck" in s]
    assert len(clashes) == 1, clashes


def main() -> int:
    failed = 0
    for fn in CASES:
        try:
            fn()
            print(f"  PASS  {fn.__name__}")
        except Exception as e:  # noqa: BLE001
            failed += 1
            print(f"  FAIL  {fn.__name__}: {type(e).__name__}: {e}")
    print(f"\ncatalog-physics corpus: {len(CASES) - failed}/{len(CASES)} passed")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
