#!/usr/bin/env python3
"""
Quick test script for URDF parsing and kinematic graph serialization.

Usage:
    python test_hello.py
"""
import json
import sys
import os

# Add core to path for imports
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from model.urdf_parser import parse_urdf
from model.kinematic_graph import KinematicGraph


def main():
    """Test URDF parsing and kinematic graph."""
    urdf_path = os.path.join(
        os.path.dirname(__file__),
        "test_data",
        "simple_arm.urdf"
    )

    print(f"Loading URDF from: {urdf_path}")
    print()

    # Parse the URDF
    try:
        kg = parse_urdf(urdf_path)
    except Exception as e:
        print(f"Error parsing URDF: {e}")
        import traceback
        traceback.print_exc()
        return 1

    # Print summary
    print(f"Kinematic Graph: {kg}")
    print(f"Root link: {kg.root_link}")
    print()

    # Print links
    print("Links:")
    for link_name in kg.get_links():
        link_data = kg.get_link_data(link_name)
        print(f"  - {link_name}")
        print(f"      mass: {link_data.mass}")
        if link_data.inertia:
            print(f"      inertia: ixx={link_data.inertia.ixx}, iyy={link_data.inertia.iyy}, izz={link_data.inertia.izz}")
        if link_data.collision_geometry:
            print(f"      collision: {link_data.collision_geometry['type']}")

    print()

    # Print joints
    print("Joints:")
    for u, v in kg.graph.edges():
        joint_data = kg.graph[u][v]["data"]
        print(f"  - {joint_data.name}")
        print(f"      type: {joint_data.joint_type}")
        print(f"      parent: {joint_data.parent_link} -> child: {joint_data.child_link}")
        print(f"      axis: {joint_data.axis}")
        if joint_data.limits:
            print(f"      limits: [{joint_data.limits.lower}, {joint_data.limits.upper}]")

    print()

    # Serialize to JSON
    kg_json = kg.to_json()

    print("Kinematic Graph JSON (pretty-printed):")
    print(json.dumps(kg_json, indent=2))

    print()

    # Verify round-trip
    print("Testing round-trip (JSON -> KinematicGraph -> JSON)...")
    kg_restored = KinematicGraph.from_json(kg_json)
    kg_json_restored = kg_restored.to_json()

    if kg_json == kg_json_restored:
        print("✓ Round-trip successful!")
    else:
        print("✗ Round-trip failed!")
        print("Original JSON:")
        print(json.dumps(kg_json, indent=2))
        print("\nRestored JSON:")
        print(json.dumps(kg_json_restored, indent=2))
        return 1

    return 0


if __name__ == "__main__":
    sys.exit(main())
