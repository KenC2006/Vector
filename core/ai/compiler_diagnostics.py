"""
Structured diagnostic taxonomy for the AI assembly pipeline.

Phase 3 of docs/COMPONENT_UNIFICATION_PLAN.md §3.8 (Structured repair loop):

    > Validation feedback is routed to the owning layer:
    >   - topology/archetype errors -> AI semantic graph
    >   - resolver/schema errors    -> component spec
    >   - placement/compiler errors -> deterministic code fixes
    >   - exporter transform errors -> exporter tests
    > The AI should not be asked to guess new RPY values for a compiler bug.

This module defines the owner taxonomy and a single `route` entry point so any
producer (archetype normalizer, semantic-graph validator, resolver, placement
compiler, exporter) can emit diagnostics in a uniform shape and the dispatcher
sends each one to the right consumer:

  Owner                        Consumer
  ─────                        ────────
  AI_TOPOLOGY                  fed back into the next AI redesign turn
  COMPONENT_SPEC               raised to the human spec editor (or CI)
  PLACEMENT_COMPILER           printed as a developer log; never shown to AI
  EXPORTER                     printed as a developer log; never shown to AI

Diagnostics produced today (e.g. by archetype_normalizer) are dicts shaped like
`{code, severity, component, message, repair?}`. This module preserves that
shape and adds an `owner` field, plus a small registry mapping codes -> owners
so existing producers don't need to know the routing rules.
"""

from enum import Enum
from typing import Dict, Iterable, List, Optional


class Owner(str, Enum):
    AI_TOPOLOGY = "ai_topology"
    COMPONENT_SPEC = "component_spec"
    PLACEMENT_COMPILER = "placement_compiler"
    EXPORTER = "exporter"


# Code -> owning layer. Anything not listed defaults to PLACEMENT_COMPILER (the
# safest default — surfaces a developer log without polluting the AI prompt).
_OWNER_BY_CODE: Dict[str, Owner] = {
    # archetype_normalizer.py
    "quadruped_tail_forbidden": Owner.AI_TOPOLOGY,
    "quadruped_tail_present": Owner.AI_TOPOLOGY,
    "foot_pad_has_children": Owner.AI_TOPOLOGY,
    "multiple_roots": Owner.AI_TOPOLOGY,
    "archetype_unknown": Owner.PLACEMENT_COMPILER,

    # semantic_graph.py
    "semantic_graph_forbidden_field": Owner.AI_TOPOLOGY,
    "semantic_graph_foot_pad_attach_rpy": Owner.AI_TOPOLOGY,
    "semantic_graph_split_servo_link_name": Owner.AI_TOPOLOGY,
    "semantic_graph_mirrored_sign": Owner.AI_TOPOLOGY,

    # component spec / resolver
    "missing_bbox": Owner.COMPONENT_SPEC,
    "mesh_vs_bbox_divergence": Owner.COMPONENT_SPEC,
    "connector_outside_bounds": Owner.COMPONENT_SPEC,

    # placement compiler
    "placement_overlap": Owner.PLACEMENT_COMPILER,
    "joint_origin_unresolved": Owner.PLACEMENT_COMPILER,

    # exporter
    "urdf_mjcf_transform_mismatch": Owner.EXPORTER,
    "rpy_quaternion_drift": Owner.EXPORTER,
}


def owner_for(code: str) -> Owner:
    """Return the owning layer for a diagnostic code (default placement)."""
    return _OWNER_BY_CODE.get(code, Owner.PLACEMENT_COMPILER)


def annotate(diagnostics: Iterable[Dict]) -> List[Dict]:
    """Tag a stream of raw diagnostics with their owning layer. Idempotent."""
    out: List[Dict] = []
    for d in diagnostics:
        if not isinstance(d, dict):
            continue
        if "owner" in d:
            out.append(d)
            continue
        code = str(d.get("code", ""))
        out.append({**d, "owner": owner_for(code).value})
    return out


def route(diagnostics: Iterable[Dict]) -> Dict[str, List[Dict]]:
    """Group diagnostics by owning layer. Every owner key is always present."""
    bucket: Dict[str, List[Dict]] = {o.value: [] for o in Owner}
    for d in annotate(diagnostics):
        bucket[d["owner"]].append(d)
    return bucket


def format_for_ai(diagnostics: Iterable[Dict]) -> Optional[str]:
    """Render the AI_TOPOLOGY-owned subset as a redesign-prompt fragment.

    Returns None when there's nothing the AI is responsible for. Other owners
    are intentionally suppressed — feeding placement/exporter bugs back to the
    AI is exactly the failure the plan calls out."""
    routed = route(diagnostics)
    ai_diags = routed[Owner.AI_TOPOLOGY.value]
    if not ai_diags:
        return None
    lines = ["The previous assembly had topology issues you should fix:"]
    for d in ai_diags:
        sev = d.get("severity", "info")
        msg = d.get("message", d.get("code", ""))
        comp = d.get("component")
        suffix = f" (link='{comp}')" if comp else ""
        lines.append(f"  [{sev}] {msg}{suffix}")
    return "\n".join(lines)
