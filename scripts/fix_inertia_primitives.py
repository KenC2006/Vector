"""Fix inertia_primitive mismatches: components labelled `cylinder` whose bbox
X/Y are unequal (i.e. they aren't cylindrical at all), and components labelled
`box` that should clearly be cylinders.

Reads the live preset JSON, applies a small set of explicit corrections, and
writes back. Idempotent.
"""
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
PRESETS = ROOT / "core" / "presets" / "generic_presets.json"

# Components whose true shape is a box even if bbox X==Y. Listed because the
# physical part is a hand/cluster/finger shape that doesn't approximate a
# cylinder for inertia or visual fallback.
FORCE_BOX = {
    # Three-finger underactuated gripper — fingers spread laterally, body is
    # a faceted cluster, not a rotund mass.
    "effector_3finger_adaptive",
    # 130-size DC motor: rectangular can with flat sides, not round.
    "motor_dc_small_130",
    # 3D lidar after reshape is 149×73×104 — a Velodyne/Ouster style puck with
    # asymmetric housing, not a cylinder.
    "sensor_lidar_3d",
    # Force/torque sensor reshaped to 44×75×15 — flattened disc with
    # asymmetric mounting plate, box approximates better.
    "sensor_force_torque_6axis",
    # Rigid shaft coupling after reshape is 25×21×21 — not symmetric on X/Y.
    "transmission_rigid_shaft_coupling",
}

# Components currently `box` whose canonical part is a cylinder.
FORCE_CYL = {
    # Planetary gearbox: round housing with cylindrical output.
    "transmission_planetary_gearbox",
}


def main():
    data = json.loads(PRESETS.read_text())
    changed = []
    for cat in data["categories"].values():
        for comp in cat.get("components", []):
            cid = comp["id"]
            phys = comp.get("physical", {})
            current = phys.get("inertia_primitive")
            target = None
            if cid in FORCE_BOX and current != "box":
                target = "box"
            elif cid in FORCE_CYL and current != "cylinder":
                target = "cylinder"
            if target:
                phys["inertia_primitive"] = target
                changed.append((cid, current, target))

    for cid, old, new in changed:
        print(f"  {cid}: {old} -> {new}")

    if not changed:
        print("(no changes)")
        return

    PRESETS.write_text(json.dumps(data, indent=2) + "\n")
    print(f"\nWrote {len(changed)} change(s) to {PRESETS}")


if __name__ == "__main__":
    main()
