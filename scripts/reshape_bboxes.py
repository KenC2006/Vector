"""
reshape_bboxes.py — One-shot migration that aligns preset bboxes to the
runtime-effective GLB mesh extent for single-mesh single-preset components.

Skips:
  - Components whose GLB is shared across multiple presets (per MESH_OVERRIDES).
  - Components in PROCEDURAL_VISUAL_ONLY (already routed to bbox primitive).
  - Components blacklisted from rendering.

For each in-scope component:
  1. Compute new_bbox = round(runtime_extent, nearest 1mm), with a small +1mm
     padding on each axis for connector clearance.
  2. Rescale connectors with hardcoded origin_xyz_mm by axis: if old origin had
     |x|≈old_bbox_x/2, replace with new_bbox_x/2 (preserving sign). Likewise Y, Z.

Usage:
    python scripts/reshape_bboxes.py            # dry run, prints proposed diff
    python scripts/reshape_bboxes.py --apply    # writes back to JSON
"""
import argparse
import json
import math
import re
from collections import defaultdict
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
PRESETS_PATH = ROOT / "core" / "presets" / "generic_presets.json"
EXTENTS_PATH = ROOT / "src" / "public" / "meshExtents.generated.json"
OVERRIDES_PATH = ROOT / "src" / "src" / "richVisuals" / "meshOverrides.ts"


def parse_ts_string_map(ts: str, name: str) -> dict[str, str]:
    m = re.search(rf"export const {name}[^{{]*\{{(.*?)^\}}", ts, re.DOTALL | re.MULTILINE)
    if not m:
        return {}
    return dict(re.findall(r"['\"]([\w_]+)['\"]\s*:\s*['\"]([^'\"]+)['\"]", m.group(1)))


def parse_ts_set(ts: str, name: str) -> set[str]:
    m = re.search(rf"export const {name}[^=]*=\s*new Set\(\[(.*?)\]\)", ts, re.DOTALL)
    if not m:
        return set()
    return set(re.findall(r"['\"]([\w_]+)['\"]", m.group(1)))


def parse_rotations(ts: str) -> dict[str, list[float]]:
    m = re.search(r"export const ROTATION_OVERRIDES[^{]*\{(.*?)^\}", ts, re.DOTALL | re.MULTILINE)
    if not m:
        return {}
    out = {}
    for em in re.finditer(r"['\"]([\w_]+)['\"]\s*:\s*\[([^\]]+)\]", m.group(1)):
        cid = em.group(1)
        vals = []
        for v in em.group(2).split(","):
            vstr = v.replace("Math.PI", str(math.pi)).strip()
            if not re.fullmatch(r"[\d\.\s+\-*/()]+", vstr):
                continue
            vals.append(float(eval(vstr)))  # noqa: S307
        out[cid] = vals
    return out


def apply_rotation(extent, rpy):
    if not rpy or all(abs(v) < 1e-9 for v in rpy):
        return list(extent)
    rx, ry, rz = rpy
    cx, sx = math.cos(rx), math.sin(rx)
    cy, sy = math.cos(ry), math.sin(ry)
    cz, sz = math.cos(rz), math.sin(rz)
    R = [
        [cy * cz, sx * sy * cz - cx * sz, cx * sy * cz + sx * sz],
        [cy * sz, sx * sy * sz + cx * cz, cx * sy * sz - sx * cz],
        [-sy, sx * cy, cx * cy],
    ]
    return [
        abs(R[i][0]) * extent[0] + abs(R[i][1]) * extent[1] + abs(R[i][2]) * extent[2]
        for i in range(3)
    ]


def round_clean(x: float, padding: float = 1.0) -> float:
    """Round up to nearest mm with small padding for connector clearance."""
    return float(round(x + padding))


def rescale_connector(origin: list[float], old_bbox: list[float], new_bbox: list[float]) -> list[float]:
    """If a connector origin component is at ±old_bbox/2, snap to ±new_bbox/2.

    Otherwise leave as-is — connectors at off-center positions (e.g. tread_outer
    at [0, -30, 0]) are explicit geometry, not bbox-faces.
    """
    out = list(origin)
    for i in range(3):
        half_old = old_bbox[i] / 2
        half_new = new_bbox[i] / 2
        if abs(abs(origin[i]) - half_old) < 0.5:  # within 0.5mm of half-bbox
            sign = 1 if origin[i] >= 0 else -1
            out[i] = sign * half_new
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--apply", action="store_true", help="write changes")
    args = ap.parse_args()

    # Components whose bbox was authored to a real-world spec (named size,
    # standard form factor, or product line). For these the spec is
    # authoritative and a mesh mismatch should be handled by replacing the GLB
    # (or routing to PROCEDURAL_VISUAL_ONLY), not by inflating the bbox.
    SKIP_SPEC_AUTHORITATIVE = {
        # NEMA standard form factors — width is fixed by the standard.
        "actuator_stepper_nema17",  # 42.3mm NEMA17 face
        "actuator_stepper_nema23",  # 57.15mm NEMA23 face
        # 100mm driven wheel — diameter is the spec.
        "mobility_wheel_driven",
        # Spec is "small dev board"; mesh is a different (larger) board family.
        "compute_fpga_dev_board",
    }

    presets = json.loads(PRESETS_PATH.read_text())
    extents = json.loads(EXTENTS_PATH.read_text())["components"]
    ts = OVERRIDES_PATH.read_text(encoding="utf-8")

    mesh_for = parse_ts_string_map(ts, "MESH_OVERRIDES")
    procedural = parse_ts_set(ts, "PROCEDURAL_VISUAL_ONLY")
    blacklisted = parse_ts_set(ts, "SLOW_MESH_BLACKLIST")
    rotations = parse_rotations(ts)

    # Find which GLBs are shared across multiple presets
    glb_users: dict[str, list[str]] = defaultdict(list)
    for cid, mesh in mesh_for.items():
        glb_users[mesh].append(cid)
    shared = {mesh for mesh, users in glb_users.items() if len(users) > 1}

    changes = []
    for cat in presets["categories"].values():
        for comp in cat.get("components", []):
            cid = comp["id"]
            if cid in procedural or cid in blacklisted:
                continue
            if cid in SKIP_SPEC_AUTHORITATIVE:
                continue
            mesh = mesh_for.get(cid)
            if not mesh or mesh in shared:
                continue
            ext = extents.get(cid)
            if not ext:
                continue
            raw = ext.get("visual", {}).get("raw_extent_mm")
            if not raw:
                continue
            rot = rotations.get(cid, [0, 0, 0])
            runtime_extent = apply_rotation(raw, rot)

            old_bbox = comp.get("physical", {}).get("bounding_box_mm")
            if not old_bbox:
                continue

            # Don't reshape if magnitude divergence is already <30%.
            ratios = [
                runtime_extent[i] / old_bbox[i] if old_bbox[i] > 0 else 1
                for i in range(3)
            ]
            score = max(max(r, 1 / r if r > 0 else 999) for r in ratios)
            if score < 1.3:
                continue

            new_bbox = [round_clean(v) for v in runtime_extent]

            # Update connectors
            connector_changes = []
            for con in comp.get("connectors", []):
                origin = con.get("origin_xyz_mm")
                if not origin or not isinstance(origin, list):
                    continue
                new_origin = rescale_connector(origin, old_bbox, new_bbox)
                if new_origin != origin:
                    connector_changes.append((con["id"], list(origin), new_origin))
                    con["origin_xyz_mm"] = new_origin

            comp["physical"]["bounding_box_mm"] = new_bbox
            changes.append((cid, old_bbox, new_bbox, connector_changes, score))

    if not changes:
        print("No reshape candidates found.")
        return

    print(f"Proposed reshape for {len(changes)} component(s):\n")
    for cid, old_bb, new_bb, cons, score in sorted(changes, key=lambda x: -x[4]):
        print(f"  {cid}")
        print(f"    bbox: {old_bb} -> {new_bb}  (was {score:.2f}x off)")
        for con_id, old, new in cons:
            print(f"    {con_id} origin: {old} -> {new}")
        print()

    if args.apply:
        PRESETS_PATH.write_text(json.dumps(presets, indent=2) + "\n")
        print(f"Wrote {PRESETS_PATH}")
    else:
        print("(dry run; pass --apply to write)")


if __name__ == "__main__":
    main()
