#!/usr/bin/env python3
"""Tests for arm-chain and gripper detection in _extract_sim_joint_context.

Phase 1 metadata extension — verifies that non-leg, non-wheel kinematic
chains get tagged with arm_chain_id / arm_depth / arm_role and that
finger-named joints get tagged with gripper_root / finger_id.

Runs standalone:
    python core/test_sim_joint_context.py
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from core.server import _extract_sim_joint_context


ARM_URDF = """<?xml version="1.0"?>
<robot name="test_arm">
  <link name="base_link"/>
  <link name="shoulder"/>
  <link name="upper"/>
  <link name="elbow"/>
  <link name="forearm"/>
  <link name="wrist"/>
  <link name="palm"/>
  <link name="finger_left"/>
  <link name="finger_right"/>

  <joint name="j_base" type="revolute">
    <parent link="base_link"/><child link="shoulder"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="5" velocity="2"/>
  </joint>
  <joint name="j_shoulder" type="revolute">
    <parent link="shoulder"/><child link="upper"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="5" velocity="2"/>
  </joint>
  <joint name="upper_to_elbow" type="fixed">
    <parent link="upper"/><child link="elbow"/>
    <origin xyz="0 0 0.18" rpy="0 0 0"/>
  </joint>
  <joint name="j_elbow" type="revolute">
    <parent link="elbow"/><child link="forearm"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="5" velocity="2"/>
  </joint>
  <joint name="forearm_to_wrist" type="fixed">
    <parent link="forearm"/><child link="wrist"/>
    <origin xyz="0 0 0.15" rpy="0 0 0"/>
  </joint>
  <joint name="j_wrist" type="revolute">
    <parent link="wrist"/><child link="palm"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="5" velocity="2"/>
  </joint>

  <joint name="finger_left_joint" type="revolute">
    <parent link="palm"/><child link="finger_left"/>
    <origin xyz="0 0.02 0.03" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="0" upper="0.8" effort="2" velocity="1"/>
  </joint>
  <joint name="finger_right_joint" type="revolute">
    <parent link="palm"/><child link="finger_right"/>
    <origin xyz="0 -0.02 0.03" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="0" upper="0.8" effort="2" velocity="1"/>
  </joint>
</robot>
"""


def test_arm_chain_tagged():
    _, _, joint_metadata, _, _ = _extract_sim_joint_context(ARM_URDF)
    by_name = {m["name"]: m for m in joint_metadata}

    for jn in ("j_base", "j_shoulder", "j_elbow", "j_wrist"):
        m = by_name[jn]
        assert m.get("is_arm_chain") is True, f"{jn} should be arm_chain"
        assert m.get("arm_chain_id"), f"{jn} should have arm_chain_id"

    # Same chain — base/shoulder/elbow/wrist all serial through the kinematic
    # tree (the gripper joints branch off, but they get is_gripper not arm).
    chain_ids = {by_name[j]["arm_chain_id"] for j in ("j_base", "j_shoulder", "j_elbow", "j_wrist")}
    assert len(chain_ids) == 1, f"expected one arm chain, got {chain_ids}"

    # Depth assignment (0 at base, increasing outward).
    assert by_name["j_base"]["arm_depth"] == 0
    assert by_name["j_base"]["arm_role"] == "base"
    assert by_name["j_shoulder"]["arm_role"] == "shoulder"
    assert by_name["j_elbow"]["arm_role"] == "elbow"
    assert by_name["j_wrist"]["arm_role"] == "wrist"


def test_gripper_tagged():
    _, _, joint_metadata, _, _ = _extract_sim_joint_context(ARM_URDF)
    by_name = {m["name"]: m for m in joint_metadata}

    for jn in ("finger_left_joint", "finger_right_joint"):
        m = by_name[jn]
        assert m.get("is_gripper") is True, f"{jn} should be gripper"
        assert m.get("gripper_root") == "palm", f"{jn} gripper_root mismatch"
        assert m.get("finger_count") == 2

    # Fingers ordered by Y (right has y=-0.02 < left y=+0.02).
    assert by_name["finger_right_joint"]["finger_id"] == 0
    assert by_name["finger_left_joint"]["finger_id"] == 1


def test_arm_and_gripper_are_disjoint():
    _, _, joint_metadata, _, _ = _extract_sim_joint_context(ARM_URDF)
    by_name = {m["name"]: m for m in joint_metadata}
    for jn in ("finger_left_joint", "finger_right_joint"):
        assert not by_name[jn].get("is_arm_chain"), f"{jn} should not be arm_chain"
    for jn in ("j_base", "j_shoulder", "j_elbow", "j_wrist"):
        assert not by_name[jn].get("is_gripper"), f"{jn} should not be gripper"


WHEELED_URDF = """<?xml version="1.0"?>
<robot name="rover">
  <link name="base_link"/>
  <link name="wheel_fl"/>
  <link name="wheel_fr"/>
  <link name="wheel_rl"/>
  <link name="wheel_rr"/>
  <joint name="j_fl" type="continuous">
    <parent link="base_link"/><child link="wheel_fl"/>
    <origin xyz="0.1 0.1 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit effort="3" velocity="5"/>
  </joint>
  <joint name="j_fr" type="continuous">
    <parent link="base_link"/><child link="wheel_fr"/>
    <origin xyz="0.1 -0.1 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit effort="3" velocity="5"/>
  </joint>
  <joint name="j_rl" type="continuous">
    <parent link="base_link"/><child link="wheel_rl"/>
    <origin xyz="-0.1 0.1 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit effort="3" velocity="5"/>
  </joint>
  <joint name="j_rr" type="continuous">
    <parent link="base_link"/><child link="wheel_rr"/>
    <origin xyz="-0.1 -0.1 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit effort="3" velocity="5"/>
  </joint>
</robot>
"""


def test_wheels_not_tagged_as_arm():
    _, _, joint_metadata, _, _ = _extract_sim_joint_context(WHEELED_URDF)
    for m in joint_metadata:
        assert m.get("is_wheel_drive") is True
        assert not m.get("is_arm_chain")
        assert not m.get("is_gripper")


# Wheeled rover with an arm mounted on the chassis. Tests that the
# composable default-script generator emits both wheeled and arm sections
# in one step() function.
ROVER_WITH_ARM_URDF = """<?xml version="1.0"?>
<robot name="rover_arm">
  <link name="base_link"/>
  <link name="wheel_fl"/>
  <link name="wheel_fr"/>
  <link name="wheel_rl"/>
  <link name="wheel_rr"/>
  <link name="arm_shoulder"/>
  <link name="arm_upper"/>
  <link name="arm_forearm"/>

  <joint name="j_fl" type="continuous">
    <parent link="base_link"/><child link="wheel_fl"/>
    <origin xyz="0.1 0.1 0"/><axis xyz="0 1 0"/><limit effort="3" velocity="5"/>
  </joint>
  <joint name="j_fr" type="continuous">
    <parent link="base_link"/><child link="wheel_fr"/>
    <origin xyz="0.1 -0.1 0"/><axis xyz="0 1 0"/><limit effort="3" velocity="5"/>
  </joint>
  <joint name="j_rl" type="continuous">
    <parent link="base_link"/><child link="wheel_rl"/>
    <origin xyz="-0.1 0.1 0"/><axis xyz="0 1 0"/><limit effort="3" velocity="5"/>
  </joint>
  <joint name="j_rr" type="continuous">
    <parent link="base_link"/><child link="wheel_rr"/>
    <origin xyz="-0.1 -0.1 0"/><axis xyz="0 1 0"/><limit effort="3" velocity="5"/>
  </joint>

  <joint name="j_arm_base" type="revolute">
    <parent link="base_link"/><child link="arm_shoulder"/>
    <origin xyz="0 0 0.1"/><axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="5" velocity="2"/>
  </joint>
  <joint name="j_arm_shoulder" type="revolute">
    <parent link="arm_shoulder"/><child link="arm_upper"/>
    <origin xyz="0 0 0.05"/><axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="5" velocity="2"/>
  </joint>
  <joint name="j_arm_elbow" type="revolute">
    <parent link="arm_upper"/><child link="arm_forearm"/>
    <origin xyz="0 0 0.15"/><axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="5" velocity="2"/>
  </joint>
</robot>
"""


def test_rover_with_arm_composes_both():
    """Wheeled rover with arm: deterministic default emits BOTH sections."""
    from core.ai.claude_client import _generate_default_sim_script
    names, limits, meta, leg_geom, arm_geom = _extract_sim_joint_context(ROVER_WITH_ARM_URDF)
    by_name = {m["name"]: m for m in meta}
    # Tags applied correctly
    for jn in ("j_fl", "j_fr", "j_rl", "j_rr"):
        assert by_name[jn].get("is_wheel_drive") is True
    for jn in ("j_arm_base", "j_arm_shoulder", "j_arm_elbow"):
        assert by_name[jn].get("is_arm_chain") is True
    # Arm geometry summary populated
    assert arm_geom is not None and arm_geom.get("chain_count") == 1
    assert arm_geom.get("mean_reach_m", 0) > 0.1

    script = _generate_default_sim_script(names, meta, None, leg_geom, limits, arm_geom)
    assert "wheeled" in script and "arm" in script, "expected composed section header"
    assert "WHEEL_DRIVE_JOINTS" in script and "ARM_JOINTS" in script
    # Both archetypes present in body
    assert "wheel_ramp" in script
    assert "arm_ramp" in script
    # Single step() function (composition, not multiple)
    assert script.count("def step(") == 1
    # All movable joints present in some form
    for jn in names:
        assert jn in script, f"joint {jn} missing from composed script"


def main():
    failures = 0
    for fn in [
        test_arm_chain_tagged,
        test_gripper_tagged,
        test_arm_and_gripper_are_disjoint,
        test_wheels_not_tagged_as_arm,
        test_rover_with_arm_composes_both,
    ]:
        try:
            fn()
            print(f"PASS  {fn.__name__}")
        except AssertionError as e:
            print(f"FAIL  {fn.__name__}: {e}")
            failures += 1
    if failures:
        sys.exit(1)
    print("\nAll sim-joint-context tests passed.")


if __name__ == "__main__":
    main()
