"""Generation-quality metrics over a CompiledGraph (+ source AssemblyGraph).

All geometry here is pure math on the placement compiler's output contract
(CompiledLink: bounds {half, center, shape} in meters, worldXyz/worldRpy,
physicalWorldXyz/Rpy, joints, massKg). Nothing reads authoring fields, so the
harness survives schema changes to AssemblyComponent.

Conventions:
  - URDF fixed-axis RPY: R = Rz(yaw) @ Ry(pitch) @ Rx(roll)
  - world AABB half-extents of an oriented box: hA[i] = sum_j |R[i][j]| * half[j]
"""
from __future__ import annotations

import math
import re
from typing import Any, Dict, List, Optional, Tuple

Vec3 = Tuple[float, float, float]

GAP_TOL_M = 0.003          # floating threshold (3 mm)
# Designed standoff idioms get wider tolerances: chained servos mount across a
# carrier/yoke bracket (60 mm min offset in servoSplit.ts), drivetrains hang
# with tire-radius clearance (assembled_outer_radius vs body half-extent).
GAP_TOL_SERVO_CHILD_M = 0.065
GAP_TOL_DRIVETRAIN_CHILD_M = 0.015
BURIED_EPS_M = 0.001
GROUND_BAND_M = 0.010      # links whose bottom is within this of min-z count as "lowest"
SYMMETRY_TOL_M = 0.015

_TERMINAL_LOWEST_OK = re.compile(
    r"^(mobility_|drivetrain_caster|structural_baseplate)"
)
_TIRE_RE = re.compile(r"^(mobility_wheel_|mobility_mecanum_|mobility_omni_|mobility_caster_)")
_FOOT_ID = "mobility_rubber_foot_pad"


# ── small linear algebra (no numpy dependency for the geometric metrics) ────

def rpy_matrix(rpy: List[float]) -> List[List[float]]:
    r, p, y = float(rpy[0]), float(rpy[1]), float(rpy[2])
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    # Rz(y) @ Ry(p) @ Rx(r)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def mat_vec(m: List[List[float]], v: List[float]) -> List[float]:
    return [
        m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
        m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
        m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2],
    ]


def mat_mul(a: List[List[float]], b: List[List[float]]) -> List[List[float]]:
    return [
        [sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)]
        for i in range(3)
    ]


class WorldBox:
    """Axis-aligned world bound of an oriented box."""

    __slots__ = ("center", "half", "label")

    def __init__(self, center: List[float], half: List[float], label: str = ""):
        self.center = center
        self.half = half
        self.label = label

    @staticmethod
    def from_oriented(
        pose_xyz: List[float],
        pose_rpy: List[float],
        local_half: List[float],
        local_center: List[float],
        label: str = "",
    ) -> "WorldBox":
        rot = rpy_matrix(pose_rpy)
        c_world = mat_vec(rot, list(local_center))
        center = [pose_xyz[i] + c_world[i] for i in range(3)]
        half = [
            sum(abs(rot[i][j]) * abs(local_half[j]) for j in range(3))
            for i in range(3)
        ]
        return WorldBox(center, half, label)

    def separation(self, other: "WorldBox") -> float:
        """max-axis separation: > 0 means disjoint by at least that much."""
        return max(
            abs(self.center[k] - other.center[k]) - (self.half[k] + other.half[k])
            for k in range(3)
        )

    def contains(self, other: "WorldBox", eps: float = BURIED_EPS_M) -> bool:
        return all(
            abs(other.center[k] - self.center[k]) + other.half[k]
            <= self.half[k] + eps
            for k in range(3)
        )

    def overlap_volume(self, other: "WorldBox") -> float:
        vol = 1.0
        for k in range(3):
            o = (self.half[k] + other.half[k]) - abs(self.center[k] - other.center[k])
            if o <= 0:
                return 0.0
            vol *= o
        return vol

    def min_z(self) -> float:
        # NOTE: per-axis support of an oriented box is exact (|R|·h), so this
        # is the true lowest corner, not a conservative bound.
        return self.center[2] - self.half[2]


# ── CompiledGraph indexing helpers ───────────────────────────────────────────

def _phys_index(compiled: Dict[str, Any]) -> Dict[str, Tuple[Dict[str, Any], int]]:
    """physical link name -> (CompiledLink, index into physicalLinks)."""
    out: Dict[str, Tuple[Dict[str, Any], int]] = {}
    for cl in compiled.get("links") or []:
        for i, name in enumerate(cl.get("physicalLinks") or []):
            out[name] = (cl, i)
    return out


def _logical_parent_map(compiled: Dict[str, Any]) -> Dict[str, Optional[str]]:
    phys = _phys_index(compiled)
    out: Dict[str, Optional[str]] = {}
    for cl in compiled.get("links") or []:
        joints = cl.get("joints") or []
        if not joints:
            out[cl["logicalName"]] = None
            continue
        parent_phys = joints[0].get("parentLink")
        owner = phys.get(parent_phys)
        out[cl["logicalName"]] = owner[0]["logicalName"] if owner else None
    return out


def _tree_distance(parents: Dict[str, Optional[str]], a: str, b: str) -> int:
    def chain(x: str) -> List[str]:
        seen = [x]
        while parents.get(x) is not None:
            x = parents[x]  # type: ignore[assignment]
            seen.append(x)
            if len(seen) > 200:
                break
        return seen

    ca, cb = chain(a), chain(b)
    set_a = {n: i for i, n in enumerate(ca)}
    for j, n in enumerate(cb):
        if n in set_a:
            return set_a[n] + j
    return 999


def _link_geometry_boxes(component: Dict[str, Any], pose_xyz: List[float], pose_rpy: List[float]) -> List[WorldBox]:
    """Per-primitive world boxes for a link_geometry component."""
    boxes: List[WorldBox] = []
    rot_link = rpy_matrix(pose_rpy)
    for prim in component.get("link_geometry") or []:
        if not isinstance(prim, dict):
            continue
        shape = prim.get("shape")
        xyz_mm = prim.get("xyz_mm") or [0, 0, 0]
        c_local = [float(v) / 1000.0 for v in xyz_mm]
        prpy = [float(v) for v in (prim.get("rpy") or [0, 0, 0])]
        if shape == "box":
            size = prim.get("size_mm") or [10, 10, 10]
            half = [float(s) / 2000.0 for s in size]
        elif shape == "cylinder":
            r = float(prim.get("radius_mm") or 5) / 1000.0
            length = float(prim.get("length_mm") or 10) / 1000.0
            half = [r, r, length / 2.0]
        elif shape == "sphere":
            r = float(prim.get("radius_mm") or 5) / 1000.0
            half = [r, r, r]
            prpy = [0, 0, 0]
        else:
            continue
        rot = mat_mul(rot_link, rpy_matrix(prpy))
        c_world = mat_vec(rot_link, c_local)
        center = [pose_xyz[i] + c_world[i] for i in range(3)]
        h_world = [
            sum(abs(rot[i][j]) * half[j] for j in range(3)) for i in range(3)
        ]
        boxes.append(WorldBox(center, h_world, label=str(prim.get("name") or shape)))
    return boxes


def _components_by_name(graph: Optional[Dict[str, Any]]) -> Dict[str, Dict[str, Any]]:
    out: Dict[str, Dict[str, Any]] = {}
    for c in (graph or {}).get("components") or []:
        if isinstance(c, dict) and c.get("link_name"):
            out[str(c["link_name"])] = c
    return out


# ── metrics ──────────────────────────────────────────────────────────────────

def metric_compile(compiled: Dict[str, Any]) -> Dict[str, Any]:
    skipped = compiled.get("skippedClasses") or []
    diags = compiled.get("diagnostics") or []
    errors = [d for d in diags if d.get("severity") == "error"]
    n_links = len(compiled.get("links") or [])
    ok = n_links > 0 and not skipped and not errors
    return {
        "score": 1.0 if ok else (0.3 if n_links > 0 else 0.0),
        "links": n_links,
        "skipped_classes": skipped,
        "error_diagnostics": [d.get("message", "") for d in errors][:10],
    }


def metric_contact(compiled: Dict[str, Any], graph: Optional[Dict[str, Any]]) -> Dict[str, Any]:
    phys = _phys_index(compiled)
    comps = _components_by_name(graph)
    rows: List[Dict[str, Any]] = []
    n_floating = n_buried = n_contact = 0

    for cl in compiled.get("links") or []:
        joints = cl.get("joints") or []
        if not joints:
            continue  # root
        parent_phys_name = joints[0].get("parentLink")
        parent_entry = phys.get(parent_phys_name)
        if not parent_entry:
            continue
        parent_cl, parent_idx = parent_entry

        # child contact pose: parent-facing physical link (index 0)
        c_xyz = (cl.get("physicalWorldXyz") or [cl["worldXyz"]])[0]
        c_rpy = (cl.get("physicalWorldRpy") or [cl["worldRpy"]])[0]
        cb = cl.get("bounds") or {}
        child_box = WorldBox.from_oriented(
            list(c_xyz), list(c_rpy),
            list(cb.get("half") or [0.01] * 3),
            list(cb.get("center") or [0.0] * 3),
            label=cl["logicalName"],
        )

        parent_comp = comps.get(parent_cl["logicalName"]) or {}
        if parent_comp.get("link_geometry"):
            parent_boxes = _link_geometry_boxes(
                parent_comp,
                list(parent_cl["worldXyz"]),
                list(parent_cl["worldRpy"]),
            )
        else:
            p_xyz = (parent_cl.get("physicalWorldXyz") or [parent_cl["worldXyz"]])[parent_idx]
            p_rpy = (parent_cl.get("physicalWorldRpy") or [parent_cl["worldRpy"]])[parent_idx]
            pb = parent_cl.get("bounds") or {}
            parent_boxes = [WorldBox.from_oriented(
                list(p_xyz), list(p_rpy),
                list(pb.get("half") or [0.01] * 3),
                list(pb.get("center") or [0.0] * 3),
                label=parent_cl["logicalName"],
            )]
        if not parent_boxes:
            continue

        gap = min(child_box.separation(b) for b in parent_boxes)
        child_is_shell = bool((comps.get(cl["logicalName"]) or {}).get("link_geometry"))
        buried = (not child_is_shell) and any(b.contains(child_box) for b in parent_boxes)

        # Designed-standoff idioms: chained rotary actuators sit across a
        # carrier/yoke bracket; drivetrains hang with tire-radius clearance.
        # Use the wider tolerance so the metric flags genuinely detached
        # parts, not idioms. "Split rotary" is the STRUCTURAL signal —
        # physicalLinks > 1 means the compiler emitted body+horn — so any
        # actuator the capability predicates split (servo, BLDC, stepper,
        # gearmotor) gets the standoff tolerance without an id list here.
        child_id = str(cl.get("componentId") or "")
        child_is_split_rotary = len(cl.get("physicalLinks") or []) > 1
        if child_is_split_rotary:
            gap_tol = GAP_TOL_SERVO_CHILD_M
        elif child_id.startswith("drivetrain_"):
            gap_tol = GAP_TOL_DRIVETRAIN_CHILD_M
        else:
            gap_tol = GAP_TOL_M

        if gap > gap_tol:
            status = "floating"
            n_floating += 1
        elif buried:
            status = "buried"
            n_buried += 1
        else:
            status = "contact"
            n_contact += 1
        rows.append({
            "link": cl["logicalName"],
            "parent": parent_cl["logicalName"],
            "status": status,
            "gap_mm": round(gap * 1000, 2),
        })

    total = len(rows)
    score = 1.0 if total == 0 else max(0.0, 1.0 - (n_floating + n_buried) / total)
    worst = max((r["gap_mm"] for r in rows), default=0.0)
    return {
        "score": score,
        "children": total,
        "floating": n_floating,
        "buried": n_buried,
        "contact": n_contact,
        "worst_gap_mm": worst,
        "offenders": [r for r in rows if r["status"] != "contact"][:20],
    }


def metric_interpenetration(compiled: Dict[str, Any]) -> Dict[str, Any]:
    parents = _logical_parent_map(compiled)
    links = compiled.get("links") or []
    boxes: List[Tuple[str, WorldBox]] = []
    for cl in links:
        b = cl.get("bounds") or {}
        boxes.append((
            cl["logicalName"],
            WorldBox.from_oriented(
                list(cl["worldXyz"]), list(cl["worldRpy"]),
                list(b.get("half") or [0.01] * 3),
                list(b.get("center") or [0.0] * 3),
                label=cl["logicalName"],
            ),
        ))
    total_vol = 0.0
    worst: Tuple[float, str, str] = (0.0, "", "")
    n_pairs = 0
    for i in range(len(boxes)):
        for j in range(i + 1, len(boxes)):
            a_name, a_box = boxes[i]
            b_name, b_box = boxes[j]
            if _tree_distance(parents, a_name, b_name) < 2:
                continue
            vol = a_box.overlap_volume(b_box)
            if vol > 1e-9:
                n_pairs += 1
                total_vol += vol
                if vol > worst[0]:
                    worst = (vol, a_name, b_name)
    link_vol = sum(8 * b.half[0] * b.half[1] * b.half[2] for _, b in boxes) or 1.0
    frac = total_vol / link_vol
    score = max(0.0, 1.0 - frac * 20.0)  # 5% mutual overlap → score 0
    return {
        "score": score,
        "overlapping_pairs": n_pairs,
        "total_overlap_cm3": round(total_vol * 1e6, 2),
        "overlap_fraction": round(frac, 5),
        "worst_pair": {"links": [worst[1], worst[2]], "cm3": round(worst[0] * 1e6, 2)} if worst[0] > 0 else None,
    }


def metric_ground(compiled: Dict[str, Any], graph: Optional[Dict[str, Any]]) -> Dict[str, Any]:
    links = compiled.get("links") or []
    lowest: List[Tuple[float, str, str]] = []  # (min_z, logicalName, componentId)
    root_bottom = None
    for cl in links:
        b = cl.get("bounds") or {}
        box = WorldBox.from_oriented(
            list(cl["worldXyz"]), list(cl["worldRpy"]),
            list(b.get("half") or [0.01] * 3),
            list(b.get("center") or [0.0] * 3),
        )
        lowest.append((box.min_z(), cl["logicalName"], cl["componentId"]))
        if not (cl.get("joints") or []):  # root link
            root_bottom = box.min_z()
    if not lowest:
        return {"score": 0.0, "reason": "no links"}
    min_z = min(z for z, _, _ in lowest)
    offenders = []
    for z, name, comp_id in lowest:
        if z - min_z > GROUND_BAND_M:
            continue
        # Only links that hang BELOW the root body count as ground-contact
        # candidates — a flat tabletop robot has everything within a few mm of
        # the plate and none of it is "wrongly touching the ground".
        if root_bottom is not None and z >= root_bottom - 0.002:
            continue
        ok = (
            bool(_TERMINAL_LOWEST_OK.match(comp_id))
            or comp_id == _FOOT_ID
            or _TIRE_RE.match(comp_id) is not None
        )
        if not ok:
            offenders.append({"link": name, "component": comp_id, "z_mm": round(z * 1000, 1)})
    score = 1.0 if not offenders else max(0.0, 1.0 - 0.34 * len(offenders))
    return {
        "score": score,
        "min_z_mm": round(min_z * 1000, 1),
        "non_foot_lowest_links": offenders[:10],
    }


def metric_structure(
    compiled: Dict[str, Any],
    graph: Optional[Dict[str, Any]],
    expectations: Dict[str, Any],
) -> Dict[str, Any]:
    links = compiled.get("links") or []
    actuated = 0
    for cl in links:
        for j in cl.get("joints") or []:
            if j.get("type") in ("revolute", "prismatic", "continuous"):
                actuated += 1
    checks: List[Tuple[str, bool, str]] = []

    dof_range = expectations.get("dof_range")
    if dof_range:
        ok = dof_range[0] <= actuated <= dof_range[1]
        checks.append(("dof", ok, f"actuated={actuated} expected {dof_range}"))

    comps = (graph or {}).get("components") or []
    n_feet = sum(1 for c in comps if c.get("component_id") == _FOOT_ID)
    n_tires = sum(1 for c in comps if _TIRE_RE.match(str(c.get("component_id") or "")))
    if expectations.get("expect_wheels"):
        lo = expectations.get("min_wheels", 1)
        hi = expectations.get("max_wheels", 99)
        checks.append(("wheels", lo <= n_tires <= hi, f"tires={n_tires} expected [{lo},{hi}]"))
    if expectations.get("min_legs") is not None:
        lo = expectations.get("min_legs", 0)
        hi = expectations.get("max_legs", 99)
        # legs ≈ ground-terminal chains: foot pads, or (for creatures) terminal mobility parts
        n_legs = n_feet if n_feet > 0 else n_tires
        checks.append(("legs", lo <= n_legs <= hi, f"feet={n_feet} expected [{lo},{hi}]"))

    sym_detail: List[Dict[str, Any]] = []
    if expectations.get("expect_symmetry"):
        # Geometric (name-independent) symmetry: every off-center link must have
        # SOME link of the same component type at its y-mirrored position. The
        # compiler assigns multi-child slots by input order, so name-pairing
        # (fl↔fr) doesn't reflect geometry.
        positions = [
            (cl["componentId"], cl["worldXyz"]) for cl in links
            if abs(cl["worldXyz"][1]) > 0.005
        ]
        unmatched = 0
        for comp_id, xyz in positions:
            mirror = (xyz[0], -xyz[1], xyz[2])
            best = min(
                (
                    math.dist(mirror, tuple(other))
                    for other_id, other in positions
                    if other_id == comp_id
                ),
                default=999.0,
            )
            if best > SYMMETRY_TOL_M:
                unmatched += 1
                if len(sym_detail) < 10:
                    sym_detail.append({"component": comp_id, "xyz": xyz, "nearest_mirror_dist_m": round(best, 4)})
        if positions:
            checks.append((
                "symmetry", unmatched == 0,
                f"{unmatched}/{len(positions)} off-center links lack a mirrored counterpart",
            ))

    passed = sum(1 for _, ok, _ in checks if ok)
    score = 1.0 if not checks else passed / len(checks)
    return {
        "score": score,
        "actuated_joints": actuated,
        "checks": [{"name": n, "pass": ok, "detail": d} for n, ok, d in checks],
        "asymmetric_pairs": sym_detail[:10],
    }


def metric_mass(compiled: Dict[str, Any]) -> Dict[str, Any]:
    links = compiled.get("links") or []
    masses = [(cl["logicalName"], float(cl.get("massKg") or 0.0)) for cl in links]
    zero = [n for n, m in masses if m <= 0.0]
    total = sum(m for _, m in masses)
    ok_total = 0.2 <= total <= 100.0
    score = (0.5 if not zero else max(0.0, 0.5 - 0.1 * len(zero))) + (0.5 if ok_total else 0.0)
    return {
        "score": score,
        "total_kg": round(total, 3),
        "zero_mass_links": zero[:10],
    }


def metric_settle(compiled: Dict[str, Any], seed: int = 7, passive_stable: bool = True) -> Dict[str, Any]:
    """MuJoCo settle test. Gracefully skipped when mujoco is unavailable.

    passive_stable=False (bipeds/humanoids): an uncontrolled biped falling over
    is physically correct, so the height-retention and drift checks are
    skipped; only no-NaN and bounded velocities are scored."""
    try:
        import mujoco  # noqa: F401
    except Exception as e:  # pragma: no cover
        return {"score": None, "skipped": True, "reason": f"mujoco unavailable: {e}"}

    import os
    import tempfile

    from core.sim.compiled_graph_urdf import compiled_graph_to_urdf
    from core.sim.mujoco_adapter import MuJoCoSimulator

    try:
        urdf = compiled_graph_to_urdf(compiled)
    except Exception as e:
        return {"score": 0.0, "reason": f"URDF emit failed: {e}"}

    path = None
    try:
        with tempfile.NamedTemporaryFile("w", suffix=".urdf", delete=False, encoding="utf-8") as f:
            f.write(urdf)
            path = f.name
        sim = MuJoCoSimulator()
        sim.load_urdf(path, free_base=True)
        model, data = sim.model, sim.data

        import numpy as np
        np.random.seed(seed)
        z0 = float(data.qpos[2]) if model.nq >= 7 else 0.0
        com0 = data.qpos[:2].copy() if model.nq >= 7 else None

        dt = model.opt.timestep
        n_steps = max(1, int(2.5 / dt))
        sim.step(n_steps)

        if not bool(np.isfinite(data.qpos).all() and np.isfinite(data.qvel).all()):
            return {"score": 0.0, "reason": "NaN/inf in state", "steps": n_steps}
        z1 = float(data.qpos[2]) if model.nq >= 7 else 0.0
        height_ratio = (z1 / z0) if z0 > 1e-6 else 1.0
        drift = 0.0
        if com0 is not None:
            drift = float(np.linalg.norm(np.asarray(data.qpos[:2]) - np.asarray(com0)))
        max_qvel = float(np.max(np.abs(data.qvel))) if data.qvel.size else 0.0

        checks = {
            "no_nan": True,
            "velocities_settled": max_qvel < 20.0,
        }
        if passive_stable:
            checks["height_retained"] = height_ratio > 0.5
            checks["drift_ok"] = drift < 0.5
        score = sum(1.0 for v in checks.values() if v) / len(checks)
        return {
            "score": score,
            "height_ratio": round(height_ratio, 3),
            "com_drift_m": round(drift, 4),
            "max_qvel": round(max_qvel, 3),
            "checks": checks,
        }
    except Exception as e:
        return {"score": 0.0, "reason": f"sim failed: {e}"}
    finally:
        if path:
            try:
                os.unlink(path)
            except OSError:
                pass


WEIGHTS = {
    "compile": 0.25,
    "contact": 0.20,
    "settle": 0.20,
    "interpenetration": 0.10,
    "ground": 0.10,
    "structure": 0.10,
    "mass": 0.05,
}


def score_graph(
    compiled: Dict[str, Any],
    graph: Optional[Dict[str, Any]],
    expectations: Dict[str, Any],
    run_sim: bool = True,
) -> Dict[str, Any]:
    metrics: Dict[str, Dict[str, Any]] = {
        "compile": metric_compile(compiled),
        "contact": metric_contact(compiled, graph),
        "interpenetration": metric_interpenetration(compiled),
        "ground": metric_ground(compiled, graph),
        "structure": metric_structure(compiled, graph, expectations),
        "mass": metric_mass(compiled),
    }
    metrics["settle"] = (
        metric_settle(compiled, passive_stable=expectations.get("passive_stable", True))
        if run_sim else {"score": None, "skipped": True, "reason": "--no-sim"}
    )

    weight_total = 0.0
    weighted = 0.0
    for name, w in WEIGHTS.items():
        s = metrics.get(name, {}).get("score")
        if s is None:
            continue  # skipped metric: renormalize
        weight_total += w
        weighted += w * float(s)
    overall = weighted / weight_total if weight_total > 0 else 0.0
    return {"metrics": metrics, "score": round(overall, 4)}
