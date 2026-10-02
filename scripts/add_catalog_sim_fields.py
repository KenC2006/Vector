"""
Write the data-driven sim fields into the component catalog:

  * sim_metadata.contact_class for every part that touches the ground or grips
    (the sim and the geometry critic key friction / "may touch the ground" on
    it instead of guessing from link names or id prefixes);
  * actuation {kind, rated_effort, rated_speed, stroke_mm} for every powered
    part, derived from its mechanical_electrical datasheet fields
    (core.presets.actuation.derive_actuation).

Idempotent. Writes core/presets/generic_presets.json and keeps
src/public/generic_presets.json byte-identical to it.

Run from the repo root:  python -m scripts.add_catalog_sim_fields   (or python scripts/add_catalog_sim_fields.py)
      --check  only verify the catalog is up to date (exit 1 if not)
"""
from __future__ import annotations

import json
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from core.presets.actuation import derive_actuation  # noqa: E402
from core.presets.identity import CONTACT_CLASSES  # noqa: E402

CATALOG = os.path.join(ROOT, "core", "presets", "generic_presets.json")
MIRROR = os.path.join(ROOT, "src", "public", "generic_presets.json")

# Parts whose contact role is not yet authored. Existing values (wheel /
# track / drivetrain) are left alone — the frontend's placement capabilities
# read "wheel" and "drivetrain".
CONTACT = {
    "mobility_rubber_foot_pad": "foot",
    "mobility_ball_transfer_unit": "caster",
    "mobility_swerve_drive_module": "wheel",
    "effector_parallel_gripper_small": "gripper",
    "effector_parallel_gripper_large": "gripper",
    "effector_3finger_adaptive": "gripper",
    "effector_soft_gripper": "gripper",
    "effector_suction_cup": "suction",
    "effector_vacuum_pad_array": "suction",
    "effector_magnetic_tool": "suction",   # adhesion contact, like suction
}


def dump(data) -> str:
    # Matches the existing file byte-for-byte: indent=2, ASCII escapes, CRLF.
    return json.dumps(data, indent=2).replace("\n", "\r\n") + "\r\n"


def update(data) -> int:
    changed = 0
    for cat in data["categories"].values():
        for comp in cat["components"]:
            sim = comp.setdefault("sim_metadata", {})
            cls = CONTACT.get(comp["id"])
            if cls and sim.get("contact_class") != cls:
                assert cls in CONTACT_CLASSES, cls
                sim["contact_class"] = cls
                changed += 1
            act = derive_actuation(dict(comp, actuation=None))
            if act and comp.get("actuation") != act:
                comp["actuation"] = act
                changed += 1
            elif not act and "actuation" in comp:
                del comp["actuation"]
                changed += 1
    return changed


def main() -> int:
    with open(CATALOG, encoding="utf-8", newline="") as f:
        raw = f.read()
    data = json.loads(raw)
    changed = update(data)
    out = dump(data)
    if "--check" in sys.argv:
        mirror_ok = open(MIRROR, encoding="utf-8", newline="").read() == raw
        ok = changed == 0 and out == raw and mirror_ok
        print(f"catalog sim fields: {'up to date' if ok else 'STALE'}"
              f" ({changed} field changes pending, mirror {'identical' if mirror_ok else 'DIFFERS'})")
        return 0 if ok else 1
    for path in (CATALOG, MIRROR):
        with open(path, "w", encoding="utf-8", newline="") as f:
            f.write(out)
    print(f"catalog sim fields: {changed} field changes written to both catalogs")
    return 0


if __name__ == "__main__":
    sys.exit(main())
