"""
Archetype normalizer (Phase 3.

Deterministic post-processing for AI-emitted assembly graphs. Runs *after* the
LLM produces a semantic graph (link names, component_ids, parents, faces) and
*before* the placement compiler turns that into URDF poses.

Scope: invariants the LLM should not have to remember on every turn — e.g.
"quadrupeds don't get a default tail", "rubber foot pads are terminal leaves".
This module operates only on the semantic graph (component_id + link_name +
attach_to + attach_face). It does not touch xyz/rpy.

Returns structured diagnostics so the caller can route them per the plan:
  - archetype/topology issues -> AI redesign prompt
  - schema issues             -> component spec
  - placement issues          -> compiler/exporter

Usage:
    from core.ai.archetype_normalizer import normalize_assembly
    components, diagnostics = normalize_assembly(components, requested_features={...})
"""

from typing import Dict, List, Optional, Set, Tuple


def _children_of(components: List[Dict], parent_link: str) -> List[Dict]:
    return [c for c in components if c.get("attach_to") == parent_link]


def _subtree_link_names(components: List[Dict], root_link: str) -> Set[str]:
    out: Set[str] = set()
    pending = [root_link]
    while pending:
        name = pending.pop()
        if name in out:
            continue
        out.add(name)
        for child in _children_of(components, name):
            ln = child.get("link_name")
            if ln:
                pending.append(ln)
    return out


def _subtree_component_ids(components: List[Dict], root_link: str) -> Set[str]:
    names = _subtree_link_names(components, root_link)
    return {c.get("component_id", "") for c in components if c.get("link_name") in names}


def detect_archetype(components: List[Dict]) -> Optional[str]:
    """Return the archetype label, or None if unrecognized.

    Quadruped = >=4 rubber foot pads.
    Arm       = has an effector at the chain tip + chain of revolute servos
                + no rubber feet (otherwise quadruped wins first).
    """
    foot_count = sum(1 for c in components if c.get("component_id") == "mobility_rubber_foot_pad")
    if foot_count >= 4:
        return "quadruped"

    has_effector = any(c.get("component_id", "").startswith("effector_") for c in components)
    revolute_count = sum(
        1 for c in components
        if c.get("joint_type") == "revolute"
        and c.get("component_id", "").startswith("actuator_")
    )
    if has_effector and revolute_count >= 3 and foot_count == 0:
        return "arm"
    return None


def _baseplate_link_names(components: List[Dict]) -> Set[str]:
    return {
        c.get("link_name", "")
        for c in components
        if c.get("component_id", "").startswith("structural_baseplate")
    }


def _normalize_quadruped(
    components: List[Dict],
    requested_features: Dict[str, bool],
) -> Tuple[List[Dict], List[Dict]]:
    """Enforce quadruped invariants. Returns (new_components, diagnostics)."""
    diagnostics: List[Dict] = []

    allow_tail = bool(requested_features.get("tail"))
    base_names = _baseplate_link_names(components)

    # Invariant: no tail subtree off the baseplate's back face unless requested.
    # A "tail-like" subtree is one parented to the baseplate (typically via
    # attach_face=back or named with "tail") that contains no functional
    # terminal (foot pad, sensor, compute, power) — i.e. it's cosmetic.
    to_remove: Set[str] = set()
    for comp in list(components):
        link_name = comp.get("link_name", "")
        cid = comp.get("component_id", "")
        if comp.get("attach_to") not in base_names:
            continue
        if comp.get("attach_face") != "back" and "tail" not in link_name.lower():
            continue

        subtree_ids = _subtree_component_ids(components, link_name)
        has_functional_terminal = any(
            sid == "mobility_rubber_foot_pad"
            or sid.startswith("sensor_")
            or sid.startswith("compute_")
            or sid.startswith("power_")
            for sid in subtree_ids
        )
        tail_like = (
            cid.startswith("actuator_servo")
            or cid.startswith("actuator_high_speed")
            or cid == "structural_limb_link_slim"
            or "tail" in link_name.lower()
        )
        if not (tail_like and not has_functional_terminal):
            continue

        if allow_tail:
            diagnostics.append({
                "code": "quadruped_tail_present",
                "severity": "info",
                "component": link_name,
                "message": f"tail subtree '{link_name}' kept (requested_features.tail=true)",
            })
            continue

        # Mark the whole subtree for removal.
        to_remove |= _subtree_link_names(components, link_name)
        diagnostics.append({
            "code": "quadruped_tail_forbidden",
            "severity": "repaired",
            "component": link_name,
            "repair": "remove_subtree",
            "message": f"removed default tail chain '{link_name}' (no foot/sensor/compute/power terminal)",
        })

    new_components = [c for c in components if c.get("link_name") not in to_remove]

    # Invariant (warn-only): rubber foot pads are terminal leaves.
    # topologyValidation.ts already enforces this in the TS side and auto-repairs,
    # but surfacing it here too lets the AI see structured feedback before the
    # graph hits the resolver.
    for comp in new_components:
        if comp.get("component_id") != "mobility_rubber_foot_pad":
            continue
        link_name = comp.get("link_name", "")
        children = _children_of(new_components, link_name)
        if children:
            diagnostics.append({
                "code": "foot_pad_has_children",
                "severity": "warning",
                "component": link_name,
                "children": [c.get("link_name") for c in children],
                "repair": "reparent_children_to_shin",
                "message": (
                    f"rubber foot pad '{link_name}' has {len(children)} child(ren); "
                    "foot pads are terminal leaves"
                ),
            })

    # Invariant (warn-only): exactly one root.
    roots = [c for c in new_components if not c.get("attach_to")]
    if len(roots) > 1:
        diagnostics.append({
            "code": "multiple_roots",
            "severity": "warning",
            "components": [c.get("link_name") for c in roots],
            "message": f"{len(roots)} root components; quadruped expects exactly one base",
        })

    return new_components, diagnostics


def _normalize_arm(
    components: List[Dict],
    requested_features: Dict[str, bool],
) -> Tuple[List[Dict], List[Dict]]:
    """Enforce arm-archetype invariants. Returns (new_components, diagnostics).

    Doesn't auto-mutate the topology — it surfaces structured diagnostics so
    the caller (AI redesign loop) can flag-and-retry. Auto-repair would risk
    cascading placement breakage; topology critiques drive redesign instead.
    """
    diagnostics: List[Dict] = []
    by_link = {c.get("link_name", ""): c for c in components}
    base_names = _baseplate_link_names(components)

    user_wants_camera = bool(requested_features.get("camera"))
    user_wants_lidar = bool(requested_features.get("lidar"))

    # Find the base servo (parented to baseplate, axis z) and the next-stage
    # shoulder servo (parented to base servo's horn or compound parent, axis y).
    base_servo: Optional[Dict] = None
    shoulder_servo: Optional[Dict] = None
    for c in components:
        if (c.get("attach_to") in base_names
            and c.get("component_id", "").startswith("actuator_servo")
            and c.get("joint_axis", "").lower() == "z"):
            base_servo = c
            break
    if base_servo is not None:
        base_link = base_servo.get("link_name", "")
        for c in components:
            if (c.get("attach_to") == base_link
                and c.get("component_id", "").startswith("actuator_servo")
                and c.get("joint_axis", "").lower() == "y"):
                shoulder_servo = c
                break

    # Invariant 1: a structural extrusion stem must sit between base_servo
    # and shoulder_servo. Without it the shoulder hangs off the base via
    # a tiny compound bracket and the arm reads as stubby.
    if base_servo is not None and shoulder_servo is not None:
        diagnostics.append({
            "code": "arm_missing_torso_stem",
            "severity": "error",
            "component": shoulder_servo.get("link_name"),
            "message": (
                f"shoulder servo '{shoulder_servo.get('link_name')}' attaches directly "
                f"to base servo '{base_servo.get('link_name')}' — insert a "
                "`structural_extrusion_2020` (length_mm=60–100) torso stem between them."
            ),
            "fixable_by": "topology",
        })

    # Invariant 2: arm bones (between Y-axis revolute joints) must use
    # structural_extrusion_2020, not structural_limb_link_slim. Slim links
    # are 6×6mm and look like twigs under chunky shoulder/elbow servos.
    y_servo_links = {
        c.get("link_name") for c in components
        if c.get("joint_type") == "revolute"
        and c.get("joint_axis", "").lower() == "y"
        and c.get("component_id", "").startswith("actuator_servo")
    }
    for c in components:
        if c.get("component_id") != "structural_limb_link_slim":
            continue
        parent = c.get("attach_to")
        if parent not in y_servo_links:
            continue
        # Slim link is parented to a Y-revolute servo => it's an arm bone.
        diagnostics.append({
            "code": "arm_slim_limb_used_as_bone",
            "severity": "error",
            "component": c.get("link_name"),
            "message": (
                f"arm bone '{c.get('link_name')}' uses structural_limb_link_slim — "
                "swap to `structural_extrusion_2020` (180–220mm upper arm, "
                "130–170mm forearm). Slim is for legs, not arms."
            ),
            "fixable_by": "topology",
        })

    # Invariant 3: no default sensors on a bare arm. Only flag if requested
    # features explicitly disclaim them.
    for c in components:
        cid = c.get("component_id", "")
        if not (cid.startswith("sensor_depth_camera") or cid.startswith("sensor_lidar")):
            continue
        is_camera = cid.startswith("sensor_depth_camera")
        if is_camera and user_wants_camera: continue
        if (not is_camera) and user_wants_lidar: continue
        diagnostics.append({
            "code": "arm_unrequested_sensor",
            "severity": "warning",
            "component": c.get("link_name"),
            "message": (
                f"sensor '{c.get('link_name')}' ({cid}) added to a bare arm — "
                "remove it unless the user explicitly asked for "
                f"{'a camera' if is_camera else 'a lidar'}."
            ),
            "fixable_by": "topology",
        })

    return components, diagnostics


_NORMALIZERS = {
    "quadruped": _normalize_quadruped,
    "arm": _normalize_arm,
}


def normalize_assembly(
    components: List[Dict],
    requested_features: Optional[Dict[str, bool]] = None,
) -> Tuple[List[Dict], List[Dict]]:
    """Run the appropriate archetype normalizer on a semantic assembly graph.

    requested_features lets the user opt in to optional parts:
        {"tail": True, "articulated_head": True}

    Returns (normalized_components, diagnostics). The diagnostics list contains
    structured records {code, severity, component, message, repair?} suitable
    for routing to the AI redesign prompt or the user-facing chat panel."""
    feats = requested_features or {}
    archetype = detect_archetype(components)
    if archetype is None:
        return components, []
    normalizer = _NORMALIZERS.get(archetype)
    if normalizer is None:
        return components, [{
            "code": "archetype_unknown",
            "severity": "info",
            "message": f"detected archetype '{archetype}' has no normalizer registered",
        }]
    return normalizer(components, feats)
