"""
Semantic-graph contract validator (Phase 3.

The plan splits AI output cleanly:

    The LLM output IS allowed to contain:
      archetype, component_ids, logical link_names, parent link_names, roles,
      semantic joint axes, high-level pose intents, requested_features.

    The LLM output is NOT allowed to contain:
      final world/local xyz, final physical rpy, `_body`/`_horn`/`_mount`/
      carrier link names, hand-mirrored knee signs, foot-pad leveling rotations,
      MJCF geom/contact parameters, collision mesh choices.

This module checks the second list. While the LLM tool schema still accepts
`attach_rpy` (we can't remove that until the shared placement compiler exists in
Phase 3b), this validator surfaces structured diagnostics whenever the AI uses
fields it shouldn't, so they can be routed via `compiler_diagnostics.route` to
the AI redesign loop instead of silently leaking through.

Returns diagnostics in the shape used elsewhere in the assembler:
  {code, severity, component, message, repair?}
"""

from typing import Dict, List, Optional


# Link-name infixes that indicate the LLM tried to author a backend-only link
# (split-servo body/horn, carrier, mount). The placement compiler owns those
# names; the AI must use the bare logical link_name only.
_FORBIDDEN_LINK_INFIXES = ("_body", "_horn", "_mount_joint", "_carrier")


def _is_passive_limb(component_id: str) -> bool:
    return (
        component_id.startswith("structural_limb_link")
        or component_id.startswith("structural_extrusion_")
    )


def validate_semantic_graph(
    components: List[Dict],
    declared_archetype: Optional[str] = None,
) -> List[Dict]:
    """Inspect an LLM-emitted assembly graph and return ownership-tagged
    diagnostics for anything that crosses into the placement compiler's lane.
    Read-only — the caller decides whether to strip offending fields or just
    route the feedback into the next AI turn.

    `declared_archetype="novel"` relaxes the raw-xyz/raw-rpy ban — Tier-B
    "complete control" lets Claude author full geometry for non-standard
    creatures. Standard mode keeps the strict ban.
    """
    is_novel = declared_archetype == "novel"
    diagnostics: List[Dict] = []
    for comp in components or []:
        if not isinstance(comp, dict):
            continue
        link_name = str(comp.get("link_name", ""))
        cid = str(comp.get("component_id", ""))

        # 1. Backend link names. The compiler emits `_body`/`_horn`; the AI
        # must reference the bare logical servo only.
        for infix in _FORBIDDEN_LINK_INFIXES:
            if infix in link_name:
                diagnostics.append({
                    "code": "semantic_graph_split_servo_link_name",
                    "severity": "error",
                    "component": link_name,
                    "message": (
                        f"link_name '{link_name}' uses a backend-internal suffix "
                        f"'{infix}'. Reference the bare logical servo link; the "
                        "compiler routes children to the correct emitted link."
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

        # 3. Passive limb links (extrusions / limb beams) must not carry
        # attach_rpy either; they inherit orientation from their actuator
        # parent's joint frame, and any nonzero rpy is a placement compiler
        # concern (rest pose), not an AI authoring concern.
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
                        "Rest pose belongs to the placement compiler — express "
                        "intent via pose_intent ('dog_crouch', etc.) instead."
                    ),
                })

        # 4. Raw xyz is never permitted from the AI in standard mode. Novel
        # mode relaxes this — Claude can author xyz directly to express
        # geometries the deterministic placement compiler can't reach.
        # `origin_xyz`/`world_xyz` aliases stay forbidden in both modes (only
        # `xyz` is the supported field name; aliases would create writer races).
        for forbidden in ("origin_xyz", "world_xyz"):
            if forbidden in comp:
                diagnostics.append({
                    "code": "semantic_graph_forbidden_field",
                    "severity": "error",
                    "component": link_name,
                    "repair": f"drop_{forbidden}",
                    "message": (
                        f"link '{link_name}' set '{forbidden}' — use bare 'xyz' "
                        "(novel mode only) or let the compiler place it."
                    ),
                })
        if not is_novel and "xyz" in comp:
            diagnostics.append({
                "code": "semantic_graph_forbidden_field",
                "severity": "error",
                "component": link_name,
                "repair": "drop_xyz",
                "message": (
                    f"link '{link_name}' set 'xyz' in standard mode. "
                    "Direct geometry authoring is novel-mode only — set "
                    "archetype_mode='novel' to use raw xyz/rpy authoring."
                ),
            })

    return diagnostics


def normalize_and_validate(assembly: Dict) -> Dict:
    """Run the full Phase 3 post-LLM pipeline on a raw `design_robot` /
    `modify_topology` assembly dict. Mutates `assembly` in place: strips
    forbidden fields, applies archetype normalization, stashes routed
    diagnostics on `assembly['_diagnostics']`. Returns the same dict.

    Single shared entry point so every site that consumes a fresh AI-emitted
    assembly applies the same contract — no duplication between tool-result
    extraction, legacy URDF assembler, and corpus tests."""
    import sys
    from core.ai.archetype_normalizer import normalize_assembly
    from core.ai.compiler_diagnostics import route as route_diagnostics, format_for_ai

    if not isinstance(assembly, dict):
        return assembly

    components = assembly.get("components", [])
    features_in = assembly.get("requested_features") or {}
    requested_features = {
        "tail": bool(features_in.get("tail")),
        "articulated_head": bool(features_in.get("articulated_head")),
    }

    declared_archetype = assembly.get("_archetype_mode")
    semantic_diags = validate_semantic_graph(components, declared_archetype=declared_archetype)
    strip_forbidden_fields(components, declared_archetype=declared_archetype)
    components, arch_diags = normalize_assembly(
        components, requested_features, declared_archetype=declared_archetype,
    )
    assembly["components"] = components

    all_diags = list(semantic_diags) + list(arch_diags)
    routed = route_diagnostics(all_diags)
    for owner, diags in routed.items():
        for diag in diags:
            sev = diag.get("severity", "info")
            msg = diag.get("message", diag.get("code", "diagnostic"))
            print(f"[assembly][{owner}:{sev}] {msg}", file=sys.stderr)

    ai_feedback = format_for_ai(all_diags)
    if ai_feedback or all_diags:
        assembly["_diagnostics"] = {
            "routed": routed,
            "ai_feedback": ai_feedback,
        }
    return assembly


def strip_forbidden_fields(
    components: List[Dict],
    declared_archetype: Optional[str] = None,
) -> List[Dict]:
    """In-place safety net: drop fields the AI is not allowed to author.

    Used in shadow mode so that even when validate_semantic_graph emits a
    diagnostic, the offending value can't leak into the placement compiler.

    `declared_archetype`: when not 'novel', strips Tier-A creative-authority
    fields (placement_offset_mm, splay_angle_deg). Those fields are
    novel-mode-only — exposing them in standard mode would risk regressing
    dog/arm/wheeled designs that depend on the deterministic placement
    pipeline staying byte-identical.

    Returns the same list reference for chaining."""
    is_novel = declared_archetype == "novel"
    for comp in components or []:
        if not isinstance(comp, dict):
            continue
        # Phase-3 contract: raw xyz / world coordinates were forbidden because
        # earlier-phase Claude produced invalid URDF when given direct geometry
        # control. In novel mode, Tier-B "complete control" reverses that —
        # Claude can author full xyz/rpy explicitly to express designs the
        # deterministic placement compiler can't reach (asymmetric anatomy,
        # exact creature poses, sculpture-style robots). Standard mode keeps
        # the strip — dog/arm/wheeled stay bulletproof.
        if not is_novel:
            for k in ("xyz", "origin_xyz", "world_xyz", "rpy"):
                comp.pop(k, None)
            comp.pop("placement_offset_mm", None)
            comp.pop("splay_angle_deg", None)
            # Custom primitive composition is novel-mode only: standard
            # archetypes have well-tuned templates that custom geometry would
            # dilute (e.g. a dog with a primitive-composed torso would lose the
            # baseplate-driven mate-connector math).
            comp.pop("link_geometry", None)
        else:
            # Novel mode: drop the world_xyz / origin_xyz aliases (placement
            # compiler only consumes `xyz` and `rpy` directly). Keeping all
            # three would let two writers race.
            comp.pop("origin_xyz", None)
            comp.pop("world_xyz", None)
        cid = str(comp.get("component_id", ""))
        if cid == "mobility_rubber_foot_pad":
            comp.pop("attach_rpy", None)
    return components
