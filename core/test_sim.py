#!/usr/bin/env python3
"""
Test script for MuJoCo simulation backend.

Tests:
1. Load simple_arm.urdf into simulator
2. Convert URDF to MJCF
3. Step simulation 100 times
4. Print joint states
5. Render a frame
6. Test reset

Usage:
    python test_sim.py
"""
import json
import sys
import os

# Add core to path for imports
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from sim.mujoco_adapter import MuJoCoSimulator
from sim.urdf_to_mjcf import urdf_to_mjcf


def main():
    """Test MuJoCo simulator."""
    urdf_path = os.path.join(
        os.path.dirname(__file__),
        "test_data",
        "simple_arm.urdf"
    )

    print("=" * 60)
    print("MuJoCo Simulation Backend Test")
    print("=" * 60)
    print()

    # Test 1: URDF to MJCF conversion
    print("[Test 1] Converting URDF to MJCF...")
    try:
        mjcf_xml = urdf_to_mjcf(urdf_path)
        print(f"  SUCCESS: Generated MJCF XML ({len(mjcf_xml)} chars)")
        print()
    except Exception as e:
        print(f"  FAILED: {e}")
        import traceback
        traceback.print_exc()
        return 1

    # Test 2: Load simulator
    print("[Test 2] Loading simulator...")
    try:
        simulator = MuJoCoSimulator()
        model_info = simulator.load_urdf(urdf_path)
        print(f"  SUCCESS: Model loaded")
        print(f"    Model name: {model_info['name']}")
        print(f"    Bodies: {model_info['n_bodies']}")
        print(f"    Joints: {model_info['n_joints']}")
        print(f"    Actuators: {model_info['n_actuators']}")
        print(f"    DOFs: {model_info['n_dofs']}")
        print(f"    Timestep: {model_info['timestep']}")
        print()
    except Exception as e:
        print(f"  FAILED: {e}")
        import traceback
        traceback.print_exc()
        return 1

    # Test 3: Get initial state
    print("[Test 3] Getting initial state...")
    try:
        state = simulator.get_state()
        print(f"  SUCCESS: Got initial state")
        print(f"    Time: {state['time']}")
        print(f"    Joints: {len(state['joint_states'])}")
        print(f"    Bodies: {len(state['body_positions'])}")
        print(f"    COM: {state['com_position']}")
        print()

        # Print joint details
        print("  Joint states:")
        for joint in state["joint_states"]:
            print(f"    {joint['name']}: pos={joint['position']:.4f}, vel={joint['velocity']:.4f}")
        print()
    except Exception as e:
        print(f"  FAILED: {e}")
        import traceback
        traceback.print_exc()
        return 1

    # Test 4: Step simulation
    print("[Test 4] Stepping simulation 100 times...")
    try:
        simulator.step(100)
        state = simulator.get_state()
        print(f"  SUCCESS: Stepped simulation")
        print(f"    Time: {state['time']:.6f}")
        print()

        # Print joint states after stepping
        print("  Joint states after 100 steps:")
        for joint in state["joint_states"]:
            print(f"    {joint['name']}: pos={joint['position']:.4f}, vel={joint['velocity']:.4f}")
        print()
    except Exception as e:
        print(f"  FAILED: {e}")
        import traceback
        traceback.print_exc()
        return 1

    # Test 5: Set control and step
    print("[Test 5] Setting control and stepping...")
    try:
        # Set some control values
        controls = {
            "shoulder_joint": 1.0,
            "elbow_joint": -0.5,
            "wrist_joint": 0.2,
        }
        simulator.set_control(controls)
        print(f"  Applied controls: {controls}")

        # Step a few times
        simulator.step(50)
        state = simulator.get_state()
        print(f"  SUCCESS: Stepped with controls")
        print(f"    Time: {state['time']:.6f}")

        print("  Joint states with control:")
        for joint in state["joint_states"]:
            print(f"    {joint['name']}: pos={joint['position']:.4f}, vel={joint['velocity']:.4f}")
        print()
    except Exception as e:
        print(f"  FAILED: {e}")
        import traceback
        traceback.print_exc()
        return 1

    # Test 6: Reset
    print("[Test 6] Resetting simulation...")
    try:
        simulator.reset()
        state = simulator.get_state()
        print(f"  SUCCESS: Simulation reset")
        print(f"    Time: {state['time']}")

        print("  Joint states after reset:")
        for joint in state["joint_states"]:
            print(f"    {joint['name']}: pos={joint['position']:.4f}, vel={joint['velocity']:.4f}")
        print()
    except Exception as e:
        print(f"  FAILED: {e}")
        import traceback
        traceback.print_exc()
        return 1

    # Test 7: Render frame
    print("[Test 7] Rendering frame...")
    try:
        png_base64 = simulator.render_frame(640, 480)
        print(f"  SUCCESS: Rendered frame")
        print(f"    Base64 PNG length: {len(png_base64)} chars")
        print(f"    (Would be ~{len(png_base64) * 0.75 / 1024:.1f} KB as binary PNG)")
        print()
    except Exception as e:
        print(f"  FAILED: {e}")
        import traceback
        traceback.print_exc()
        return 1

    # Test 8: Get state JSON serialization
    print("[Test 8] Testing JSON serialization of state...")
    try:
        state = simulator.get_state()
        state_json = json.dumps(state)
        print(f"  SUCCESS: State is JSON-serializable")
        print(f"    JSON length: {len(state_json)} chars")
        print()

        # Print pretty JSON
        print("  Sample state JSON:")
        sample_state = {
            "time": state["time"],
            "joint_states": state["joint_states"][:2] if state["joint_states"] else [],
            "body_positions": state["body_positions"][:2] if state["body_positions"] else [],
        }
        print(json.dumps(sample_state, indent=2))
        print()
    except Exception as e:
        print(f"  FAILED: {e}")
        import traceback
        traceback.print_exc()
        return 1

    print("=" * 60)
    print("All tests passed!")
    print("=" * 60)

    return 0


if __name__ == "__main__":
    sys.exit(main())
