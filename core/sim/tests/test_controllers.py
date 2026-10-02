"""
Simulator + controller regression tests: the physics fixes that made robots
controllable, the measured robot brief, and the baseline controllers.

Run: python -m core.sim.tests.test_controllers   (from the repo root)
"""
from __future__ import annotations

import sys

from core.designer.compile import compile_design
from core.sim.control import (
    compile_script, evaluate_controller, load_sim_from_text, robot_brief_data,
)
from core.sim.controllers import baseline_controller


def _urdf(parts):
    return compile_design({"name": "t", "summary": "", "parts": parts}).to_urdf()


ROVER = _urdf([
    {"name": "hull", "shape": [{"name": "box", "shape": "box", "size_mm": [300, 180, 70]}], "at": [0, 0, 110]},
    {"name": "motor_fl", "component": "motor_gear_medium_37mm", "parent": "hull",
     "at": {"ref": "hull.box.+y", "offset": [100, 15, -30]}, "align": "shaft_out", "z_axis": "+y", "mirror": True},
    {"name": "wheel_fl", "component": "mobility_wheel_driven", "parent": "motor_fl",
     "at": {"ref": "motor_fl.shaft_out"}, "align": "hub_bore", "joint": {"type": "continuous"}, "mirror": True},
    {"name": "motor_rl", "component": "motor_gear_medium_37mm", "parent": "hull",
     "at": {"ref": "hull.box.+y", "offset": [-100, 15, -30]}, "align": "shaft_out", "z_axis": "+y", "mirror": True},
    {"name": "wheel_rl", "component": "mobility_wheel_driven", "parent": "motor_rl",
     "at": {"ref": "motor_rl.shaft_out"}, "align": "hub_bore", "joint": {"type": "continuous"}, "mirror": True},
])

TANK = _urdf([
    {"name": "hull", "shape": [{"name": "box", "shape": "box", "size_mm": [260, 150, 70]}], "at": [0, 0, 100]},
    {"name": "track_l", "component": "mobility_track_tread_system", "parent": "hull",
     "at": {"ref": "hull.box.+y", "offset": [0, 25, -30]}, "align": "center", "mirror": True},
])


def _leg(tag, x):
    return [
        {"name": f"hip_{tag}l", "component": "actuator_servo_heavy_duty", "parent": "body",
         "at": {"ref": "body.chest.+y", "offset": [x, 0, 0]}, "align": "bottom", "mirror": True},
        {"name": f"thigh_{tag}l", "component": "structural_limb_link_slim", "length_mm": 110, "parent": f"hip_{tag}l",
         "at": {"ref": f"hip_{tag}l.shaft_out", "offset": [0, 4, 0]}, "align": "+z", "z_axis": "+z",
         "joint": {"type": "revolute", "rest_deg": -30, "lower_deg": -80, "upper_deg": 40}, "mirror": True},
        {"name": f"knee_{tag}l", "component": "actuator_servo_heavy_duty", "parent": f"thigh_{tag}l",
         "at": {"ref": f"thigh_{tag}l.-z"}, "align": "center", "z_axis": "+y", "mirror": True},
        {"name": f"shin_{tag}l", "component": "structural_limb_link_slim", "length_mm": 110, "parent": f"knee_{tag}l",
         "at": {"ref": f"knee_{tag}l.shaft_out", "offset": [0, 4, 0]}, "align": "+z", "z_axis": "+z",
         "joint": {"type": "revolute", "rest_deg": 60, "lower_deg": 10, "upper_deg": 110}, "mirror": True},
        {"name": f"foot_{tag}l", "component": "mobility_rubber_foot_pad", "parent": f"shin_{tag}l",
         "at": {"ref": f"shin_{tag}l.-z"}, "align": "mount_top", "z_axis": "+z", "mirror": True},
    ]


DOG = _urdf([{"name": "body", "shape": [{"name": "chest", "shape": "box", "size_mm": [320, 150, 80]}],
              "at": [0, 0, 280]}] + _leg("f", 110) + _leg("r", -110))

ROCKER = _urdf([
    {"name": "hull", "shape": [{"name": "box", "shape": "box", "size_mm": [200, 120, 60]}], "at": [0, 0, 150]},
    {"name": "arm_l", "shape": [{"name": "bar", "shape": "box", "size_mm": [200, 20, 20]}], "parent": "hull",
     "at": [0, 80, 150], "joint": {"type": "revolute", "pivot": [0, 80, 150], "axis": "+y",
                                    "lower_deg": -20, "upper_deg": 20}},
])

CASES = []


def case(fn):
    CASES.append(fn)
    return fn


@case
def joint_limits_are_radians():
    """MJCF defaults to degrees; without <compiler angle="radian"> every limit was ±1°."""
    sim = load_sim_from_text(DOG, free_base=False)
    mj, m = sim.mujoco, sim.model
    j = mj.mj_name2id(m, mj.mjtObj.mjOBJ_JOINT, "thigh_fl_joint")
    lo, hi = m.jnt_range[j]
    assert lo < -0.5 and hi > 0.5, (lo, hi)   # read as degrees these would be ~±0.02


@case
def wheels_roll_without_braking():
    """Floor rolling friction must not override the wheel's (it braked rovers to ~40%)."""
    brief = robot_brief_data(ROVER)
    code = ("def step(t, state):\n    return {" + ", ".join(
        f"'{j['name']}': {4.0 if j['plus_drives_robot'][0] > 0 else -4.0}" for j in brief["joints"] if j.get("is_wheel"))
        + "}\n")
    ev = evaluate_controller(ROVER, code, seconds=3)
    r = next(j for j in brief["joints"] if j.get("is_wheel"))["wheel_radius"]
    ideal = 4.0 * r * 3.0
    assert ev["forward_m"] > 0.75 * ideal, (ev["forward_m"], ideal)
    assert min(v for k, v in ev["ground_contact_pct"].items() if "wheel" in k) > 90, ev["ground_contact_pct"]


@case
def brief_measures_wheel_direction():
    brief = robot_brief_data(ROVER)
    wheels = [j for j in brief["joints"] if j.get("is_wheel")]
    assert len(wheels) == 4 and {w["side"] for w in wheels} == {"left", "right"}
    assert brief["mobile"]


@case
def tracks_are_drivable():
    brief = robot_brief_data(TANK)
    drives = [j["name"] for j in brief["joints"] if j.get("is_wheel")]
    assert sorted(drives) == ["track_l_drive", "track_r_drive"], drives
    ev = evaluate_controller(TANK, baseline_controller(brief), seconds=4)
    assert ev["forward_m"] > 0.3 and abs(ev["yaw_change_deg"]) < 5, ev


@case
def passive_pivot_has_no_motor():
    sim = load_sim_from_text(ROCKER, free_base=False)
    assert "arm_l_joint" not in sim._actuator_by_joint
    brief = robot_brief_data(ROCKER)
    assert next(j for j in brief["joints"] if j["name"] == "arm_l_joint")["type"] == "passive"


@case
def rover_baseline_follows_teleop():
    brief = robot_brief_data(ROVER)
    code = baseline_controller(brief)
    ev = evaluate_controller(ROVER, code, seconds=3)
    assert ev["forward_m"] > 0.2 and abs(ev["yaw_change_deg"]) < 3, ev
    turn = evaluate_controller(ROVER, code, seconds=3, cmd={"active": True, "vx": 0.0, "yaw_rate": 1.0})
    # Skid-steer scrubs: with catalog friction and gearmotor ratings the rover
    # turns at a fraction of the commanded 1 rad/s (~170° in 3 s). What matters
    # is that it turns clearly, the right way, on the spot.
    assert turn["yaw_change_deg"] > 30 and abs(turn["forward_m"]) < 0.1, turn


@case
def quadruped_baseline_walks_forward():
    brief = robot_brief_data(DOG)
    assert len(brief["leg_groups"]) == 4, brief["leg_groups"]
    ev = evaluate_controller(DOG, baseline_controller(brief), seconds=6)
    assert ev["fell_at_s"] is None and ev["forward_m"] > 0.1, (ev["forward_m"], ev["fell_at_s"])


@case
def sandbox_rejects_imports():
    try:
        compile_script("import os\ndef step(t, state):\n    return {}\n")
    except ValueError:
        return
    raise AssertionError("import was allowed")


def main() -> int:
    failed = 0
    for fn in CASES:
        try:
            fn()
            print(f"  PASS  {fn.__name__}")
        except Exception as e:  # noqa: BLE001
            failed += 1
            print(f"  FAIL  {fn.__name__}: {e}")
    print(f"\ncontroller corpus: {len(CASES) - failed}/{len(CASES)} passed")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
