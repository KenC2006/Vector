"""
Vector Component Preset Library
================================
Loads and indexes the generic_presets.json registry so the AI layer
and validation pipeline can look up component specs by ID.
"""

import json
import os
from typing import Dict, List, Optional, Tuple
import math

_PRESETS_DIR = os.path.dirname(os.path.abspath(__file__))
_PRESETS_FILE = os.path.join(_PRESETS_DIR, "generic_presets.json")

# Lazy-loaded singleton
_registry: Optional[Dict] = None


def _load() -> Dict:
    global _registry
    if _registry is None:
        with open(_PRESETS_FILE, "r", encoding="utf-8") as f:
            _registry = json.load(f)
    return _registry


def get_all_categories() -> List[str]:
    """Return list of category names (actuators, sensors, ...)."""
    return list(_load()["categories"].keys())


def get_category(name: str) -> Dict:
    """Return full category dict including description and components list."""
    return _load()["categories"][name]


def get_component(component_id: str) -> Optional[Dict]:
    """Look up a single component by its unique id across all categories."""
    for cat in _load()["categories"].values():
        for comp in cat["components"]:
            if comp["id"] == component_id:
                return comp
    return None


def list_components(category: Optional[str] = None) -> List[Dict]:
    """Return flat list of components, optionally filtered by category."""
    data = _load()
    results = []
    cats = [category] if category else data["categories"].keys()
    for cat_name in cats:
        if cat_name in data["categories"]:
            results.extend(data["categories"][cat_name]["components"])
    return results


def search_components(query: str) -> List[Dict]:
    """Simple keyword search across component id, name, and description."""
    query_lower = query.lower()
    return [
        c for c in list_components()
        if query_lower in c["id"].lower()
        or query_lower in c["name"].lower()
        or query_lower in c.get("description", "").lower()
    ]


def get_system_instruction() -> str:
    """Return the AI system instruction block from the presets file."""
    return _load()["system_instruction"]["role"]


def get_mounting_rules() -> List[str]:
    """Return the mounting validation rules."""
    return _load()["system_instruction"]["mounting_validation"]


# ---------------------------------------------------------------------------
# Physics helpers — used by the AI and validation pipeline to compute
# mass properties from a set of selected components.
# ---------------------------------------------------------------------------

def compute_box_inertia(mass_kg: float, x_m: float, y_m: float, z_m: float) -> Dict:
    """Uniform-density box inertia about its center of mass."""
    ixx = mass_kg / 12.0 * (y_m**2 + z_m**2)
    iyy = mass_kg / 12.0 * (x_m**2 + z_m**2)
    izz = mass_kg / 12.0 * (x_m**2 + y_m**2)
    return {"ixx": ixx, "ixy": 0, "ixz": 0, "iyy": iyy, "iyz": 0, "izz": izz}


def compute_cylinder_inertia(mass_kg: float, radius_m: float, height_m: float) -> Dict:
    """Uniform-density cylinder inertia (axis along Z) about its center of mass."""
    ixx = mass_kg / 12.0 * (3 * radius_m**2 + height_m**2)
    iyy = ixx
    izz = mass_kg / 2.0 * radius_m**2
    return {"ixx": ixx, "ixy": 0, "ixz": 0, "iyy": iyy, "iyz": 0, "izz": izz}


def compute_sphere_inertia(mass_kg: float, radius_m: float) -> Dict:
    """Uniform-density sphere inertia about its center of mass."""
    i = 2.0 / 5.0 * mass_kg * radius_m**2
    return {"ixx": i, "ixy": 0, "ixz": 0, "iyy": i, "iyz": 0, "izz": i}


def _number_tuple(value, length: int) -> bool:
    return (
        isinstance(value, list)
        and len(value) == length
        and all(isinstance(item, (int, float)) and math.isfinite(item) for item in value)
    )


def _resolved_bbox_mm(phys: Dict, instance: Optional[Dict] = None) -> List[float]:
    if _number_tuple(phys.get("bbox_mm"), 3):
        return list(phys["bbox_mm"])
    if _number_tuple(phys.get("bounding_box_mm"), 3):
        return list(phys["bounding_box_mm"])

    instance_length = None
    if isinstance(instance, dict):
        raw = instance.get("length_mm")
        if isinstance(raw, (int, float)) and math.isfinite(raw) and raw > 0:
            instance_length = float(raw)

    parametric = phys.get("parametric")
    if isinstance(parametric, dict) and parametric.get("axis") in ("x", "y", "z") and _number_tuple(parametric.get("cross_section_mm"), 2):
        cross = parametric["cross_section_mm"]
        length = instance_length if instance_length is not None else phys.get("length_mm", 100)
        if parametric["axis"] == "x":
            return [length, cross[0], cross[1]]
        if parametric["axis"] == "y":
            return [cross[0], length, cross[1]]
        return [cross[0], cross[1], length]

    cross_section = phys.get("cross_section_mm")
    if _number_tuple(cross_section, 3):
        return list(cross_section)
    if _number_tuple(cross_section, 2):
        length = instance_length if instance_length is not None else 40
        return [cross_section[0], cross_section[1], length]
    return [40, 40, 40]


def resolve_component_bounds_mm(preset: Dict, instance: Optional[Dict] = None) -> List[float]:
    """Single source of truth for component outer-envelope dimensions on the
    Python side (Phase 3 of COMPONENT_UNIFICATION_PLAN.md). Mirrors the TS
    `resolveComponentHalfBoundsMm` contract: bbox_mm wins, parametric splice
    consumes instance.length_mm, legacy cross_section_mm falls through.

    Returns full extents in mm: [x, y, z]. Use resolve_component_bounds_m
    for meters."""
    if not isinstance(preset, dict):
        return [10, 10, 10]
    return _resolved_bbox_mm(preset.get("physical", {}) or {}, instance)


def resolve_component_bounds_m(preset: Dict, instance: Optional[Dict] = None) -> List[float]:
    """Meters convenience wrapper around resolve_component_bounds_mm."""
    return [v / 1000.0 for v in resolve_component_bounds_mm(preset, instance)]


def is_parametric_spec(preset: Dict) -> bool:
    """True when the preset defines a per-instance length axis. Mirrors the TS
    `isParametricSpec` predicate (Phase 3 of COMPONENT_UNIFICATION_PLAN.md)."""
    if not isinstance(preset, dict):
        return False
    phys = preset.get("physical", {}) or {}
    parametric = phys.get("parametric")
    if (
        isinstance(parametric, dict)
        and parametric.get("axis") in ("x", "y", "z")
        and _number_tuple(parametric.get("cross_section_mm"), 2)
    ):
        return True
    if _number_tuple(phys.get("cross_section_mm"), 2):
        return True
    return False


def inertia_for_component(component: Dict) -> Dict:
    """Auto-calculate inertia tensor for a component from its physical spec."""
    phys = component["physical"]
    mass = phys.get("mass_kg") or phys.get("mass_kg_per_100mm", 0)
    shape = phys.get("inertia_primitive", "box")
    bb = _resolved_bbox_mm(phys)

    if shape == "box":
        dims_m = [d / 1000.0 for d in bb]
        return compute_box_inertia(mass, *dims_m[:3])
    elif shape == "cylinder":
        r = bb[0] / 2000.0
        h = bb[2] / 1000.0
        return compute_cylinder_inertia(mass, r, h)
    elif shape == "sphere":
        r = bb[0] / 2000.0
        return compute_sphere_inertia(mass, r)
    else:
        # Fallback: treat as box
        dims_m = [d / 1000.0 for d in bb]
        return compute_box_inertia(mass, *dims_m[:3])


def parallel_axis_shift(inertia: Dict, mass_kg: float,
                        dx: float, dy: float, dz: float) -> Dict:
    """Apply parallel-axis theorem to shift inertia tensor by (dx, dy, dz) meters."""
    d_sq = dx**2 + dy**2 + dz**2
    return {
        "ixx": inertia["ixx"] + mass_kg * (d_sq - dx**2),
        "ixy": inertia["ixy"] - mass_kg * dx * dy,
        "ixz": inertia["ixz"] - mass_kg * dx * dz,
        "iyy": inertia["iyy"] + mass_kg * (d_sq - dy**2),
        "iyz": inertia["iyz"] - mass_kg * dy * dz,
        "izz": inertia["izz"] + mass_kg * (d_sq - dz**2),
    }


def composite_mass_properties(
    components: List[Tuple[Dict, Tuple[float, float, float]]]
) -> Dict:
    """
    Given a list of (component_dict, position_xyz_meters) tuples,
    compute composite mass, center of mass, and inertia tensor.

    Returns dict with keys: total_mass_kg, com_xyz_m, inertia_at_com.
    """
    total_mass = 0.0
    weighted_pos = [0.0, 0.0, 0.0]

    entries = []
    for comp, pos in components:
        m = comp["physical"].get("mass_kg") or comp["physical"].get("mass_kg_per_100mm", 0)
        total_mass += m
        for i in range(3):
            weighted_pos[i] += m * pos[i]
        entries.append((comp, pos, m))

    if total_mass == 0:
        return {"total_mass_kg": 0, "com_xyz_m": [0, 0, 0], "inertia_at_com": compute_box_inertia(0, 0, 0, 0)}

    com = [w / total_mass for w in weighted_pos]

    # Sum inertias shifted to composite CoM
    composite_inertia = {"ixx": 0, "ixy": 0, "ixz": 0, "iyy": 0, "iyz": 0, "izz": 0}
    for comp, pos, m in entries:
        local_inertia = inertia_for_component(comp)
        dx = pos[0] - com[0]
        dy = pos[1] - com[1]
        dz = pos[2] - com[2]
        shifted = parallel_axis_shift(local_inertia, m, dx, dy, dz)
        for key in composite_inertia:
            composite_inertia[key] += shifted[key]

    return {
        "total_mass_kg": round(total_mass, 6),
        "com_xyz_m": [round(c, 6) for c in com],
        "inertia_at_com": {k: round(v, 9) for k, v in composite_inertia.items()},
    }


def check_static_stability(
    com_xyz_m: Tuple[float, float, float],
    contact_points_xy: List[Tuple[float, float]],
) -> Dict:
    """
    Check if projected CoM falls within the convex hull of ground contact points.

    Returns dict with: stable (bool), margin_m (float), warnings (list).
    """
    if len(contact_points_xy) < 3:
        return {
            "stable": len(contact_points_xy) >= 1,
            "margin_m": 0.0,
            "warnings": ["Fewer than 3 contact points — limited stability analysis."],
        }

    # Compute convex hull (simple gift-wrapping for small point sets)
    from functools import reduce

    def cross_2d(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    pts = sorted(contact_points_xy, key=lambda p: (p[0], p[1]))
    lower = []
    for p in pts:
        while len(lower) >= 2 and cross_2d(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    upper = []
    for p in reversed(pts):
        while len(upper) >= 2 and cross_2d(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    hull = lower[:-1] + upper[:-1]

    # Distance from projected CoM to nearest hull edge
    px, py = com_xyz_m[0], com_xyz_m[1]
    min_dist = float("inf")
    inside = True
    n = len(hull)
    for i in range(n):
        x1, y1 = hull[i]
        x2, y2 = hull[(i + 1) % n]
        # Signed distance (positive = inside for CCW hull)
        edge_len = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
        if edge_len < 1e-12:
            continue
        dist = ((x2 - x1) * (py - y1) - (y2 - y1) * (px - x1)) / edge_len
        if dist < 0:
            inside = False
        min_dist = min(min_dist, abs(dist))

    warnings = []
    if not inside:
        warnings.append("CoM projection is OUTSIDE the support polygon — robot will tip over.")
    com_height = com_xyz_m[2]
    hull_xs = [p[0] for p in hull]
    hull_ys = [p[1] for p in hull]
    polygon_width = max(
        max(hull_xs) - min(hull_xs),
        max(hull_ys) - min(hull_ys),
    )
    if polygon_width > 0 and com_height / polygon_width > 3.0:
        warnings.append(
            f"CoM height / support width ratio = {com_height/polygon_width:.1f} "
            f"(> 3.0) — high tip-over risk."
        )

    return {
        "stable": inside,
        "margin_m": round(min_dist, 6),
        "warnings": warnings,
    }
