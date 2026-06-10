"""One-shot WS4 migration: author port classes onto catalog connectors.

Port classes (shaft / bore / mount_face / generic) used to be guessed at
runtime from component-id string prefixes (resolveComponentPortsForBounds).
WS4 derives ports FROM connectors, so the class becomes authored connector
data in core/presets/generic_presets.json. This script writes it once, from
the same signals the old heuristics used (connector ids + mounting_logic),
and reports anything ambiguous for manual review.

Rules:
  - cylindrical connector, id contains 'hole'/'bore'        -> bore  (single)
  - cylindrical connector, id contains 'shaft'              -> shaft (single)
  - cylindrical, preset mounting_logic output ~ axial_shaft -> shaft (single)
  - cylindrical, preset mounting_logic primary/input ~ bore -> bore  (single)
  - other cylindrical                                       -> generic (reported)
  - planar/point connectors: left untouched (runtime default mount_face/generic)

Old-heuristic parity: presets whose TOP face used to be reclassified 'shaft'
(drivetrain_*, motor_*, actuator_bldc*, actuator_servo*, actuator_continuous*,
or mounting_logic.output == axial_shaft) must still expose a shaft-classed
connector on their +Z face. If none of their cylindrical connectors covers it,
the authored 'top' planar connector gets cls='shaft'; if they author no 'top'
connector at all, one is added at [0, 0, +hz].

Idempotent — re-running produces no further changes.
"""
import json
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CATALOG = os.path.join(ROOT, "core", "presets", "generic_presets.json")


def resolve_bbox(phys):
    for key in ("bbox_mm", "bounding_box_mm"):
        v = phys.get(key)
        if isinstance(v, list) and len(v) == 3:
            return v
    par = phys.get("parametric")
    if isinstance(par, dict) and isinstance(par.get("cross_section_mm"), list):
        cs = par["cross_section_mm"]
        axis = par.get("axis", "z")
        if axis == "x":
            return [100, cs[0], cs[1]]
        if axis == "y":
            return [cs[0], 100, cs[1]]
        return [cs[0], cs[1], 100]
    cs = phys.get("cross_section_mm")
    if isinstance(cs, list) and len(cs) == 2:
        return [cs[0], cs[1], 100]
    if isinstance(cs, list) and len(cs) == 3:
        return cs
    return [40, 40, 40]


def wants_top_shaft(comp):
    cid = comp["id"]
    ml = comp.get("mounting_logic") or {}
    out = str(ml.get("output") or "")
    return (
        cid.startswith(("drivetrain_", "motor_", "actuator_bldc",
                        "actuator_servo", "actuator_continuous"))
        or out.startswith("axial_shaft") or out == "axial_shaft"
    )


def classify_cylindrical(conn_id, ml):
    cid = conn_id.lower()
    if "hole" in cid or "bore" in cid:
        return "bore"
    if "shaft" in cid:
        return "shaft"
    out = str(ml.get("output") or "")
    if out.startswith("axial_shaft"):
        return "shaft"
    prim = str(ml.get("primary") or "") + " " + str(ml.get("input") or "")
    if "bore" in prim:
        return "bore"
    return "generic"


def main():
    with open(CATALOG, encoding="utf-8") as f:
        data = json.load(f)

    changed = 0
    review = []
    for cat in data["categories"].values():
        for comp in cat["components"]:
            ml = comp.get("mounting_logic") or {}
            conns = comp.get("connectors")
            has_z_shaft = False

            for conn in conns or []:
                if conn.get("type") != "cylindrical":
                    continue
                cls = classify_cylindrical(conn.get("id", ""), ml)
                if cls == "generic":
                    review.append(f"{comp['id']}.{conn.get('id')}: cylindrical, no shaft/bore signal -> generic")
                if conn.get("cls") != cls:
                    conn["cls"] = cls
                    changed += 1
                if cls in ("shaft", "bore") and conn.get("single") is not True:
                    conn["single"] = True
                    changed += 1
                axis = conn.get("axis_xyz") or [0, 0, 1]
                if cls == "shaft" and abs(axis[2]) > 0.999 and axis[2] > 0:
                    has_z_shaft = True

            if wants_top_shaft(comp) and not has_z_shaft:
                top = next((c for c in conns or [] if c.get("id") == "top"), None)
                if top is not None:
                    if top.get("cls") != "shaft":
                        top["cls"] = "shaft"
                        top["single"] = True
                        changed += 1
                else:
                    hz = resolve_bbox(comp.get("physical") or {})[2] / 2
                    new_conn = {
                        "id": "top",
                        "origin_xyz_mm": [0, 0, hz],
                        "axis_xyz": [0, 0, 1],
                        "type": "planar",
                        "cls": "shaft",
                        "single": True,
                    }
                    if conns is None:
                        comp["connectors"] = [new_conn]
                    else:
                        conns.append(new_conn)
                    changed += 1
                    review.append(f"{comp['id']}: added authored 'top' connector (cls=shaft) — had none")

    with open(CATALOG, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2, ensure_ascii=True)
        f.write("\n")

    print(f"wrote {changed} field changes to {CATALOG}")
    if review:
        print("manual review:")
        for line in review:
            print("  -", line)


if __name__ == "__main__":
    sys.exit(main())
