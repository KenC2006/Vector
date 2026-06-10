"""Semantic-graph contract validator.

Splits AI output ownership cleanly:

    The LLM output IS allowed to contain:
      component_ids, logical link_names, parent link_names, semantic joint
      axes, rest poses (attach_rpy), raw placement (xyz/rpy), link_geometry
      body shells, requested_features.

    The LLM output is NOT allowed to contain:
      `_body`/`_horn`/`_mount`/carrier link names (compiler-owned), the
      origin_xyz/world_xyz aliases (only bare `xyz` is the supported field —
      aliases would create writer races), attach_rpy on auto-leveled foot
      pads.

Returns diagnostics in the shape used elsewhere in the assembler:
  {code, severity, component, message, repair?}
"""

from typing import Dict, List

# Link-name infixes that indicate the LLM tried to author a backend-only link
# (split-servo body/horn, carrier, mount). The placement compiler owns those
# names; the AI must use the bare logical link_name only.
_FORBIDDEN_LINK_INFIXES = ("_body", "_horn", "_mount_joint", "_carrier")

# Authoring fields that were removed from the schema. Strip them defensively
# (old saved graphs / stale clients may still carry them) so the placement
# compiler never sees them.
_REMOVED_FIELDS = ("placement_offset_mm", "splay_angle_deg", "archetype_mode", "_archetype_mode")


def _is_passive_limb(component_id: str) -> bool:
    return (
        component_id.startswith("structural_limb_link")
        or component_id.startswith("structural_extrusion_")
    )


def validate_semantic_graph(components: List[Dict]) -> List[Dict]:
    """Inspect an LLM-emitted assembly graph and return ownership-tagged
    diagnostics for anything that crosses into the placement compiler's lane.
    Read-only — the caller decides whether to strip offending fields or just
    route the feedback into the next AI turn."""
    diagnostics: List[Dict] = []
    all_names = {
        str(c.get("link_name", "")) for c in components or [] if isinstance(c, dict)
    }
    for comp in components or []:
        if not isinstance(comp, dict):
            continue
        link_name = str(comp.get("link_name", ""))
        cid = str(comp.get("component_id", ""))

        # 1. Backend link names. The compiler emits `<link>_body`/`<link>_horn`
        # for split rotary actuators — an authored name only collides when its
        # PREFIX is another link in this graph (e.g. 'hip_1_horn' next to
        # 'hip_1'). Benign creative names like 'torso_body_1' or
        # 'scorpion_body_1' are fine; flagging every '*_body*' burned a retry
        # turn on most creature designs (live-3 finding, 2026-06-10).
        for infix in _FORBIDDEN_LINK_INFIXES:
            idx = link_name.find(infix)
            if idx <= 0:
                continue
            prefix = link_name[:idx]
            if prefix in all_names:
                diagnostics.append({
                    "code": "semantic_graph_split_servo_link_name",
                    "severity": "error",
                    "component": link_name,
                    "message": (
                        f"link_name '{link_name}' collides with the compiler-"
                        f"emitted '{infix}' link of '{prefix}'. Reference the "
                        "bare logical link; the compiler routes children to "
                        "the correct emitted link."
                    ),
                })
                break

        # 2. Foot pads must not carry attach_rpy — the assembler auto-levels
        # them flat to the world floor.
        if cid == "mobility_rubber_foot_pad":
            rpy = comp.get("attach_rpy")
            if isinstance(rpy, list) and any(abs(float(v)) > 1e-9 for v in rpy if isinstance(v, (int, float))):
                diagnostics.append({
                    "code": "semantic_graph_foot_pad_attach_rpy",
                    "severity": "error",
                    "component": link_name,
                    "repair": "drop_attach_rpy",
                    "message": (
                        f"foot pad '{link_name}' has explicit attach_rpy={rpy}. "
                        "Foot pads are auto-leveled — drop attach_rpy."
                    ),
                })

        # 3. Passive limb links (extrusions / limb beams) inherit orientation
        # from their actuator parent's joint frame; rest pose belongs on the
        # driving servo, not the bone.
        if _is_passive_limb(cid):
            rpy = comp.get("attach_rpy")
            if isinstance(rpy, list) and any(abs(float(v)) > 1e-9 for v in rpy if isinstance(v, (int, float))):
                diagnostics.append({
                    "code": "semantic_graph_forbidden_field",
                    "severity": "warning",
                    "component": link_name,
                    "repair": "drop_attach_rpy",
                    "message": (
                        f"passive limb '{link_name}' carries attach_rpy={rpy}. "
                        "Rest pose belongs on the driving servo's attach_rpy, "
                        "not on the passive bone."
                    ),
                })

        # 4. Only the bare `xyz`/`rpy` field names are supported for raw
        # placement. The aliases would let two writers race.
        for forbidden in ("origin_xyz", "world_xyz"):
            if forbidden in comp:
                diagnostics.append({
                    "code": "semantic_graph_forbidden_field",
                    "severity": "error",
                    "component": link_name,
                    "repair": f"drop_{forbidden}",
                    "message": (
                        f"link '{link_name}' set '{forbidden}' — use bare 'xyz' "
                        "for raw placement."
                    ),
                })

    return diagnostics


def normalize_and_validate(assembly: Dict) -> Dict:
    """Run the post-LLM contract pipeline on a raw `design_robot` /
    `modify_topology` assembly dict. Mutates `assembly` in place: strips
    forbidden fields and stashes routed diagnostics on
    `assembly['_diagnostics']`. Returns the same dict.

    Single shared entry point so every site that consumes a fresh AI-emitted
    assembly applies the same contract."""
    import sys
    from core.ai.compiler_diagnostics import route as route_diagnostics, format_for_ai

    if not isinstance(assembly, dict):
        return assembly

    components = assembly.get("components", [])

    semantic_diags = validate_semantic_graph(components)
    strip_forbidden_fields(components)

    routed = route_diagnostics(list(semantic_diags))
    for owner, diags in routed.items():
        for diag in diags:
            sev = diag.get("severity", "info")
            msg = diag.get("message", diag.get("code", "diagnostic"))
            print(f"[assembly][{owner}:{sev}] {msg}", file=sys.stderr)

    ai_feedback = format_for_ai(list(semantic_diags))
    if ai_feedback or semantic_diags:
        assembly["_diagnostics"] = {
            "routed": routed,
            "ai_feedback": ai_feedback,
        }
    return assembly


def strip_forbidden_fields(components: List[Dict]) -> List[Dict]:
    """In-place safety net: drop fields the AI is not allowed to author.

    Raw `xyz`/`rpy` and `link_geometry` are KEPT — they are first-class
    creative-authoring fields. Only the alias spellings, removed legacy
    fields, and foot-pad rest poses are stripped.

    Returns the same list reference for chaining."""
    for comp in components or []:
        if not isinstance(comp, dict):
            continue
        comp.pop("origin_xyz", None)
        comp.pop("world_xyz", None)
        for k in _REMOVED_FIELDS:
            comp.pop(k, None)
        cid = str(comp.get("component_id", ""))
        if cid == "mobility_rubber_foot_pad":
            comp.pop("attach_rpy", None)
    return components
