"""
divergence_report.py — Audit GLB-vs-bbox alignment across the preset library.

For each component with both a declared bounding_box_mm and a backing GLB,
compares preset bbox against post-rotation GLB extent (factoring in runtime
rotation overrides from src/src/richVisuals/visualOverrides.json).

Reports two distinct failure modes:
  1. axis_mismatch  — the GLB's long axis lands on a different preset axis than
                      the bbox claims. Root cause: missing or wrong rotation
                      override.
  2. magnitude_mismatch — even after best-case axis permutation, at least one
                      axis differs by >50%. Root cause: bbox authored
                      independently of mesh, or one GLB shared by multiple
                      presets at different physical sizes.

Usage:
    python scripts/divergence_report.py            # human-readable report
    python scripts/divergence_report.py --json     # machine-readable
    python scripts/divergence_report.py --threshold 0.5
"""

import argparse
import json
import math
import sys
from itertools import permutations
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
EXTENTS_PATH = ROOT / "scripts" / "mesh-extents.generated.json"
OVERRIDES_PATH = ROOT / "src" / "src" / "richVisuals" / "visualOverrides.json"
PRESETS_PATH = ROOT / "core" / "presets" / "generic_presets.json"


def apply_rotation(extent: list[float], rpy: list[float]) -> list[float]:
    """Apply XYZ Euler rotation to an axis-aligned box's extent vector.

    Treat extent as an unsigned magnitude on each axis; the post-rotation
    extent is |R| applied to (ex, ey, ez) where |R| is the matrix of absolute
    values of R. For the n*pi/2 multiples that appear in overrides this is
    exact; for off-axis rotations it's an upper-bound AABB, which is what we
    want for divergence comparison.
    """
    if not rpy or all(abs(v) < 1e-9 for v in rpy):
        return list(extent)

    rx, ry, rz = rpy
    cx, sx = math.cos(rx), math.sin(rx)
    cy, sy = math.cos(ry), math.sin(ry)
    cz, sz = math.cos(rz), math.sin(rz)

    # three.js Euler 'XYZ' (what meshVisual.ts applies): R = Rx * Ry * Rz.
    R = [
        [cy * cz, -cy * sz, sy],
        [cx * sz + sx * sy * cz, cx * cz - sx * sy * sz, -sx * cy],
        [sx * sz - cx * sy * cz, sx * cz + cx * sy * sz, cx * cy],
    ]
    Rabs = [[abs(v) for v in row] for row in R]
    out = [
        Rabs[i][0] * extent[0] + Rabs[i][1] * extent[1] + Rabs[i][2] * extent[2]
        for i in range(3)
    ]
    return out


def best_perm_score(bbox: list[float], extent: list[float]) -> tuple[float, tuple[int, ...], list[float]]:
    """Return (worst_axis_ratio, best_perm, permuted_extent).

    Tries all 6 axis permutations of the mesh extent against the preset bbox
    and picks the one minimizing the worst per-axis ratio (max(extent/bbox,
    bbox/extent)).
    """
    best = None
    for perm in permutations(range(3)):
        permed = [extent[perm[i]] for i in range(3)]
        ratios = [
            (permed[i] / bbox[i] if bbox[i] > 0 else float("inf"))
            for i in range(3)
        ]
        score = max(max(r, 1.0 / r if r > 0 else float("inf")) for r in ratios)
        if best is None or score < best[0]:
            best = (score, perm, permed)
    return best


def axis_ratio_score(bbox: list[float], extent: list[float]) -> float:
    """Worst per-axis ratio with the IDENTITY permutation. If this is much
    worse than best_perm_score, the bbox needs a rotation override (axis
    mismatch). If they're roughly equal, the bbox/mesh sizes themselves
    diverge (magnitude mismatch).
    """
    ratios = [
        (extent[i] / bbox[i] if bbox[i] > 0 else float("inf"))
        for i in range(3)
    ]
    return max(max(r, 1.0 / r if r > 0 else float("inf")) for r in ratios)


def collect_components() -> list[dict]:
    extents = json.loads(EXTENTS_PATH.read_text())
    overrides = json.loads(OVERRIDES_PATH.read_text(encoding="utf-8"))

    rotations = {cid: e["rpy"] for cid, e in overrides["rotationOverrides"].items()}
    procedural_only = set(overrides["proceduralVisualOnly"])
    scale_policy = {cid: e["policy"] for cid, e in overrides["scalePolicy"].items()}

    # Read live bboxes from generic_presets.json — mesh-extents.generated.json's
    # declared_bbox_mm is a build-time snapshot and drifts whenever the preset
    # JSON changes. The mesh extents (raw_extent_mm) in that file remain valid
    # because GLB files don't change.
    presets_root = json.loads(PRESETS_PATH.read_text())
    live_bboxes: dict[str, list[float]] = {}
    for cat in presets_root["categories"].values():
        for comp in cat.get("components", []):
            bb = comp.get("physical", {}).get("bounding_box_mm")
            if bb:
                live_bboxes[comp["id"]] = bb

    rows: list[dict] = []
    components = extents.get("components", {})
    for cid, data in components.items():
        if not isinstance(data, dict):
            continue
        live_bb = live_bboxes.get(cid)
        if live_bb is not None:
            data = {**data, "declared_bbox_mm": live_bb}
        row = _row(cid, data, rotations, procedural_only, scale_policy)
        if row:
            rows.append(row)
    return rows


def _row(cid, data, rotations, procedural_only, scale_policy):
    visual = data.get("visual") or {}
    bbox = data.get("declared_bbox_mm")
    raw = visual.get("raw_extent_mm")
    if not bbox or not raw:
        return None

    rot_override = rotations.get(cid, [0, 0, 0])
    glb_internal_rot = visual.get("rotation_rpy") or [0, 0, 0]

    # Source of truth: raw extent + the rotation override from
    # visualOverrides.json (re-applied so a stale mesh-extents file can't hide
    # an override edit).
    runtime_extent = apply_rotation(raw, rot_override)

    identity_score = axis_ratio_score(bbox, runtime_extent)
    perm_score, perm, permed = best_perm_score(bbox, runtime_extent)

    axis_mismatch = identity_score > perm_score * 1.3 and identity_score > 1.5
    magnitude_mismatch = perm_score > 1.5
    has_rotation = any(abs(v) > 1e-9 for v in rot_override)

    return {
        "component_id": cid,
        "bbox": bbox,
        "raw_extent": raw,
        "runtime_extent": [round(v, 2) for v in runtime_extent],
        "rotation_override": rot_override if has_rotation else None,
        "scale_policy": scale_policy.get(cid),
        "procedural_only": cid in procedural_only,
        "identity_axis_score": round(identity_score, 2),
        "best_perm_score": round(perm_score, 2),
        "axis_mismatch": bool(axis_mismatch),
        "magnitude_mismatch": bool(magnitude_mismatch),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--json", action="store_true", help="emit machine-readable JSON")
    ap.add_argument("--threshold", type=float, default=1.5, help="ratio threshold (default 1.5)")
    ap.add_argument("--only", choices=["axis", "magnitude", "both"], default="both")
    args = ap.parse_args()

    rows = collect_components()

    flagged = []
    for r in rows:
        if r["procedural_only"]:
            continue
        if args.only in ("axis", "both") and r["axis_mismatch"]:
            flagged.append(r)
            continue
        if args.only in ("magnitude", "both") and r["magnitude_mismatch"]:
            flagged.append(r)

    if args.json:
        print(json.dumps({"total": len(rows), "flagged": len(flagged), "components": flagged}, indent=2))
        return

    sys.stdout.reconfigure(encoding="utf-8")  # type: ignore[attr-defined]

    print(f"Audited {len(rows)} components")
    axis_only = [r for r in flagged if r["axis_mismatch"] and not r["magnitude_mismatch"]]
    mag_only = [r for r in flagged if r["magnitude_mismatch"] and not r["axis_mismatch"]]
    both = [r for r in flagged if r["axis_mismatch"] and r["magnitude_mismatch"]]

    print(f"  axis-mismatch only:        {len(axis_only)}")
    print(f"  magnitude-mismatch only:   {len(mag_only)}")
    print(f"  both:                      {len(both)}")
    print(f"  clean:                     {len(rows) - len(flagged)}")
    print()

    def fmt(r):
        bb = r["bbox"]
        rt = r["runtime_extent"]
        rot = r["rotation_override"]
        rot_str = f"rotated" if rot else "NO ROTATION"
        flags = []
        if r["axis_mismatch"]:
            flags.append("AXIS")
        if r["magnitude_mismatch"]:
            flags.append("MAG")
        return (
            f"  {r['component_id']:42} bbox={bb} extent={rt}\n"
            f"  {'':42} idScore={r['identity_axis_score']:.2f}  permScore={r['best_perm_score']:.2f}  "
            f"{rot_str}  scale={r['scale_policy'] or 'per-axis(default)'}  flags={','.join(flags)}"
        )

    if both:
        print("=" * 78)
        print("BOTH axis AND magnitude mismatch (worst):")
        print("=" * 78)
        for r in sorted(both, key=lambda x: -x["best_perm_score"]):
            print(fmt(r))
            print()

    if axis_only:
        print("=" * 78)
        print("AXIS mismatch only (needs rotation override):")
        print("=" * 78)
        for r in sorted(axis_only, key=lambda x: -x["identity_axis_score"]):
            print(fmt(r))
            print()

    if mag_only:
        print("=" * 78)
        print("MAGNITUDE mismatch only (bbox vs mesh size disagreement):")
        print("=" * 78)
        for r in sorted(mag_only, key=lambda x: -x["best_perm_score"]):
            print(fmt(r))
            print()


if __name__ == "__main__":
    main()
