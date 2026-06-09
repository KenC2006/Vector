#!/usr/bin/env python3
"""
Phase 3 corpus tests for the semantic-graph contract validator and the
diagnostic router.

Runs as a script (no test framework dep): each invariant is a function that
returns True on pass / raises on fail. Wire into CI with:

    python core/test_semantic_graph.py
"""
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from core.ai.semantic_graph import (
    validate_semantic_graph,
    strip_forbidden_fields,
    normalize_and_validate,
)
from core.ai.compiler_diagnostics import (
    Owner,
    owner_for,
    route,
    format_for_ai,
)


def _codes(diags):
    return sorted(d.get("code") for d in diags)


def test_clean_graph_emits_no_diagnostics():
    graph = [
        {"link_name": "base_1", "component_id": "structural_baseplate", "attach_to": None},
        {"link_name": "servo_1", "component_id": "actuator_servo_standard", "attach_to": "base_1", "attach_face": "top"},
    ]
    assert validate_semantic_graph(graph) == [], "clean graph must produce zero diagnostics"


def test_split_servo_link_name_flagged():
    graph = [
        {"link_name": "servo_1_horn", "component_id": "actuator_servo_standard", "attach_to": "base_1"},
    ]
    diags = validate_semantic_graph(graph)
    assert _codes(diags) == ["semantic_graph_split_servo_link_name"], _codes(diags)
    assert diags[0]["severity"] == "error"


def test_foot_pad_attach_rpy_flagged_and_stripped():
    graph = [
        {
            "link_name": "foot_1",
            "component_id": "mobility_rubber_foot_pad",
            "attach_to": "shin_1",
            "attach_face": "bottom",
            "attach_rpy": [0.5, 0, 0],
        },
    ]
    diags = validate_semantic_graph(graph)
    assert "semantic_graph_foot_pad_attach_rpy" in _codes(diags), _codes(diags)
    strip_forbidden_fields(graph)
    assert "attach_rpy" not in graph[0], "strip must remove foot pad attach_rpy"


def test_passive_limb_attach_rpy_warned_not_stripped():
    graph = [
        {
            "link_name": "extr_1",
            "component_id": "structural_extrusion_2020",
            "attach_to": "base_1",
            "attach_face": "top",
            "attach_rpy": [1.0, 0, 0],
        },
    ]
    diags = validate_semantic_graph(graph)
    assert "semantic_graph_forbidden_field" in _codes(diags), _codes(diags)
    assert diags[0]["severity"] == "warning"
    strip_forbidden_fields(graph)
    assert "attach_rpy" in graph[0], "passive-limb attach_rpy must remain (warn-only, shadow mode)"


def test_raw_xyz_is_first_class_authoring():
    # Raw xyz/rpy are always-available creative authoring — no diagnostic,
    # never stripped. Only the alias spellings are forbidden.
    graph = [
        {"link_name": "wing_1", "component_id": "sensor_imu_9dof", "attach_to": "base_1",
         "xyz": [0.1, 0, 0], "rpy": [0, 0.5, 0]},
    ]
    assert validate_semantic_graph(graph) == []
    strip_forbidden_fields(graph)
    assert "xyz" in graph[0] and "rpy" in graph[0]


def test_xyz_aliases_flagged_and_stripped():
    graph = [
        {"link_name": "rogue_1", "component_id": "sensor_imu_9dof", "attach_to": "base_1",
         "origin_xyz": [0.1, 0, 0]},
    ]
    diags = validate_semantic_graph(graph)
    assert _codes(diags) == ["semantic_graph_forbidden_field"]
    strip_forbidden_fields(graph)
    assert "origin_xyz" not in graph[0]


def test_removed_legacy_fields_stripped_silently():
    graph = [
        {"link_name": "leg_1", "component_id": "structural_limb_link_slim", "attach_to": "base_1",
         "placement_offset_mm": [10, 0, 0], "splay_angle_deg": 30, "archetype_mode": "novel"},
    ]
    strip_forbidden_fields(graph)
    assert "placement_offset_mm" not in graph[0]
    assert "splay_angle_deg" not in graph[0]
    assert "archetype_mode" not in graph[0]


def test_owner_for_known_codes():
    assert owner_for("semantic_graph_forbidden_field") is Owner.AI_TOPOLOGY
    assert owner_for("quadruped_tail_forbidden") is Owner.AI_TOPOLOGY
    assert owner_for("mesh_vs_bbox_divergence") is Owner.COMPONENT_SPEC
    assert owner_for("urdf_mjcf_transform_mismatch") is Owner.EXPORTER
    # Unknown codes default to placement compiler (developer-only).
    assert owner_for("totally_made_up_code") is Owner.PLACEMENT_COMPILER


def test_router_buckets_diagnostics_by_owner():
    diags = [
        {"code": "semantic_graph_forbidden_field", "severity": "error", "message": "x"},
        {"code": "mesh_vs_bbox_divergence", "severity": "warning", "message": "y"},
        {"code": "urdf_mjcf_transform_mismatch", "severity": "error", "message": "z"},
        {"code": "joint_origin_unresolved", "severity": "error", "message": "w"},
    ]
    routed = route(diags)
    assert len(routed[Owner.AI_TOPOLOGY.value]) == 1
    assert len(routed[Owner.COMPONENT_SPEC.value]) == 1
    assert len(routed[Owner.EXPORTER.value]) == 1
    assert len(routed[Owner.PLACEMENT_COMPILER.value]) == 1


def test_format_for_ai_includes_only_ai_owned():
    diags = [
        {"code": "semantic_graph_forbidden_field", "severity": "error", "message": "ai-fix me", "component": "rogue_1"},
        {"code": "joint_origin_unresolved", "severity": "error", "message": "compiler bug", "component": "joint_3"},
    ]
    text = format_for_ai(diags)
    assert text is not None
    assert "ai-fix me" in text
    assert "compiler bug" not in text, "compiler-owned diagnostics must NOT leak into the AI prompt"


def test_format_for_ai_returns_none_when_nothing_for_ai():
    diags = [
        {"code": "joint_origin_unresolved", "severity": "error", "message": "compiler bug"},
    ]
    assert format_for_ai(diags) is None


def test_normalize_and_validate_pipeline_stashes_diagnostics():
    assembly = {
        "base_link": "base_1",
        "components": [
            {"link_name": "base_1", "component_id": "structural_baseplate", "attach_to": None},
            {"link_name": "rogue_1", "component_id": "sensor_imu_9dof", "attach_to": "base_1", "origin_xyz": [0.1, 0, 0]},
        ],
    }
    out = normalize_and_validate(assembly)
    assert out is assembly, "normalize_and_validate mutates in place"
    assert "_diagnostics" in assembly, "diagnostics must be stashed"
    diag = assembly["_diagnostics"]
    assert "routed" in diag and "ai_feedback" in diag
    assert diag["ai_feedback"] is not None and "rogue_1" in diag["ai_feedback"]
    # And the forbidden alias should be gone from the graph.
    assert "origin_xyz" not in assembly["components"][1]


def test_normalize_and_validate_clean_assembly_has_no_diagnostics_key():
    assembly = {
        "base_link": "base_1",
        "components": [
            {"link_name": "base_1", "component_id": "structural_baseplate", "attach_to": None},
        ],
    }
    normalize_and_validate(assembly)
    assert "_diagnostics" not in assembly, "clean assemblies must not carry an empty _diagnostics payload"


_TESTS = [
    test_clean_graph_emits_no_diagnostics,
    test_split_servo_link_name_flagged,
    test_foot_pad_attach_rpy_flagged_and_stripped,
    test_passive_limb_attach_rpy_warned_not_stripped,
    test_raw_xyz_is_first_class_authoring,
    test_xyz_aliases_flagged_and_stripped,
    test_removed_legacy_fields_stripped_silently,
    test_owner_for_known_codes,
    test_router_buckets_diagnostics_by_owner,
    test_format_for_ai_includes_only_ai_owned,
    test_format_for_ai_returns_none_when_nothing_for_ai,
    test_normalize_and_validate_pipeline_stashes_diagnostics,
    test_normalize_and_validate_clean_assembly_has_no_diagnostics_key,
]


def main():
    failed = 0
    for fn in _TESTS:
        try:
            fn()
            print(f"  PASS  {fn.__name__}")
        except AssertionError as e:
            failed += 1
            print(f"  FAIL  {fn.__name__}: {e}")
        except Exception as e:
            failed += 1
            print(f"  ERROR {fn.__name__}: {e!r}")
    total = len(_TESTS)
    print(f"\nsemantic-graph corpus: {total - failed}/{total} passed")
    return 0 if failed == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
