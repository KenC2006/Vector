#!/usr/bin/env python3
"""Tests for prompt-level archetype classification + freedom-mode wiring.

Runs standalone:
    python core/test_prompt_archetype.py
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from core.ai.prompt_archetype import classify_prompt, is_novel
from core.ai.archetype_normalizer import normalize_assembly, detect_archetype


# ── Classifier ──────────────────────────────────────────────────────────────

def test_standard_archetypes():
    for prompt in [
        "build a robot dog",
        "design a quadruped",
        "Spot-style quadruped",
        "Go1 quadruped",
        "make a robotic arm with a gripper",
        "6-DOF arm",
        "build a rover with 4 wheels",
        "design a Mars rover",
        "make a humanoid",
        "build a car",
        "design a biped",
    ]:
        assert classify_prompt(prompt) == "standard", f"{prompt!r} should be standard"


def test_novel_archetypes():
    for prompt in [
        "build a crab robot",
        "design a spider",
        "make an octopus",
        "snake robot",
        "starfish-like robot",
        "build a centipede",
        "scorpion robot",
        "build a 6-legged thing",
        "8-legged sprawled walker",
        "weird custom robot",
        "non-standard hexapod",
    ]:
        assert classify_prompt(prompt) == "novel", f"{prompt!r} should be novel"
        assert is_novel(prompt), f"is_novel({prompt!r}) should be True"


def test_standard_wins_over_novel():
    # An arm with a "crab claw" gripper is still primarily an arm.
    assert classify_prompt("robotic arm with crab-claw gripper") == "standard"
    # A robot dog with a snake-like tail is still a quadruped.
    assert classify_prompt("robot dog with a snake tail") == "standard"


def test_unknown_prompts_dont_trigger_freedom_mode():
    for prompt in [
        "",
        "build something cool",
        "make a robot",
        "design me a thing",
    ]:
        assert classify_prompt(prompt) == "unknown", f"{prompt!r} should be unknown"
        assert not is_novel(prompt)


def test_word_boundary_matches():
    # "scar" must not match "car"; "darm" must not match "arm".
    assert classify_prompt("a scar in the testbed") == "unknown"


# ── Normalizer opt-out ──────────────────────────────────────────────────────

def _eight_leg_assembly():
    """Synthetic 8-legged crab-like topology — 8 foot pads.

    Pre-fix this would tag as quadruped (>=4 foot pads) and force-fit it into
    the 12-DOF Go1 template via _normalize_quadruped. Post-fix: detect_archetype
    returns None (==4 only), and declared_archetype='novel' fully short-circuits.
    """
    components = [
        {"link_name": "base", "component_id": "structural_baseplate_large",
         "attach_to": None},
    ]
    for i in range(8):
        components.append({
            "link_name": f"shin_{i}", "component_id": "structural_limb_link_slim",
            "attach_to": "base", "joint_type": "fixed",
        })
        components.append({
            "link_name": f"foot_{i}", "component_id": "mobility_rubber_foot_pad",
            "attach_to": f"shin_{i}", "joint_type": "fixed",
        })
    return components


def test_eight_legs_no_longer_classified_as_quadruped():
    """Tightened detection: ==4 foot pads, not >=4."""
    components = _eight_leg_assembly()
    assert detect_archetype(components) is None, (
        "8-legged designs must not auto-classify as quadruped — that's the "
        "exact regression that produced 'crab = robot dog with more legs'."
    )


def test_four_legs_still_classified_as_quadruped():
    """Real quadrupeds must still trigger the dog scaffolding (no regression)."""
    components = [
        {"link_name": "base", "component_id": "structural_baseplate_large",
         "attach_to": None},
    ]
    for i in range(4):
        components.append({
            "link_name": f"shin_{i}", "component_id": "structural_limb_link_slim",
            "attach_to": "base", "joint_type": "fixed",
        })
        components.append({
            "link_name": f"foot_{i}", "component_id": "mobility_rubber_foot_pad",
            "attach_to": f"shin_{i}", "joint_type": "fixed",
        })
    assert detect_archetype(components) == "quadruped"


def test_tier_a_fields_stripped_in_standard_mode():
    """Tier-A creative-authority fields (placement_offset_mm, splay_angle_deg)
    must not survive into the placement compiler in standard mode.

    Standard archetypes (dog/arm/wheeled) depend on the deterministic
    placement pipeline staying byte-identical. If these fields leaked into
    standard-mode runs, we'd risk regressing the dog/arm visual quality."""
    from core.ai.semantic_graph import strip_forbidden_fields
    components = [
        {
            "link_name": "leg_1", "component_id": "actuator_servo_high_torque",
            "attach_to": "base", "attach_face": "bottom", "joint_type": "revolute",
            "joint_axis": "z",
            "placement_offset_mm": [10, -5, 0],
            "splay_angle_deg": 25,
        },
    ]
    strip_forbidden_fields(components, declared_archetype=None)
    assert "placement_offset_mm" not in components[0], "must strip in standard mode"
    assert "splay_angle_deg" not in components[0], "must strip in standard mode"


def test_tier_a_fields_preserved_in_novel_mode():
    """Novel mode is the explicit opt-in for Tier-A creative authority. The
    fields must survive validation untouched so the placement compiler can
    apply the offset and splay override."""
    from core.ai.semantic_graph import strip_forbidden_fields
    components = [
        {
            "link_name": "leg_1", "component_id": "actuator_servo_high_torque",
            "attach_to": "base", "attach_face": "bottom", "joint_type": "revolute",
            "joint_axis": "z",
            "placement_offset_mm": [10, -5, 0],
            "splay_angle_deg": 25,
        },
    ]
    strip_forbidden_fields(components, declared_archetype="novel")
    assert components[0].get("placement_offset_mm") == [10, -5, 0]
    assert components[0].get("splay_angle_deg") == 25


def test_tier_b_xyz_stripped_in_standard_mode():
    """Tier-B raw xyz/rpy authoring is novel-only — must be stripped in
    standard mode so dog/arm/wheeled keep their deterministic placement.
    Standard mode also emits a diagnostic explaining how to opt in."""
    from core.ai.semantic_graph import strip_forbidden_fields, validate_semantic_graph
    components = [
        {"link_name": "leg_1", "component_id": "actuator_servo_high_torque",
         "attach_to": "base", "attach_face": "bottom", "joint_type": "revolute",
         "joint_axis": "z",
         "xyz": [0.1, 0.05, -0.02],
         "rpy": [0.0, 0.0, 1.5708]},
    ]
    diags = validate_semantic_graph(components, declared_archetype=None)
    assert any(d["code"] == "semantic_graph_forbidden_field" for d in diags), \
        "standard mode must emit forbidden-field diagnostic for raw xyz"
    strip_forbidden_fields(components, declared_archetype=None)
    assert "xyz" not in components[0]
    assert "rpy" not in components[0]


def test_tier_b_xyz_preserved_in_novel_mode():
    """Novel mode is the explicit opt-in for full geometry authority — both
    xyz and rpy must survive validation untouched."""
    from core.ai.semantic_graph import strip_forbidden_fields, validate_semantic_graph
    components = [
        {"link_name": "leg_1", "component_id": "actuator_servo_high_torque",
         "attach_to": "base", "attach_face": "bottom", "joint_type": "revolute",
         "joint_axis": "z",
         "xyz": [0.1, 0.05, -0.02],
         "rpy": [0.0, 0.0, 1.5708]},
    ]
    # Novel mode: validator does NOT emit the forbidden-xyz diagnostic.
    diags = validate_semantic_graph(components, declared_archetype="novel")
    forbidden_xyz_diags = [
        d for d in diags
        if d["code"] == "semantic_graph_forbidden_field"
        and "xyz" in d.get("message", "")
    ]
    assert not forbidden_xyz_diags, \
        f"novel mode must not emit forbidden-xyz diagnostic; got {forbidden_xyz_diags}"
    # And strip preserves the fields.
    strip_forbidden_fields(components, declared_archetype="novel")
    assert components[0].get("xyz") == [0.1, 0.05, -0.02]
    assert components[0].get("rpy") == [0.0, 0.0, 1.5708]


def test_tier_b_origin_xyz_aliases_still_forbidden():
    """`origin_xyz` and `world_xyz` aliases stay forbidden in BOTH modes —
    only bare `xyz` is the supported authoring field. Two writers (xyz +
    origin_xyz) would race and silently produce wrong placement."""
    from core.ai.semantic_graph import strip_forbidden_fields
    for mode in (None, "novel"):
        components = [
            {"link_name": "x", "component_id": "y", "attach_to": "z",
             "origin_xyz": [1, 2, 3], "world_xyz": [4, 5, 6]},
        ]
        strip_forbidden_fields(components, declared_archetype=mode)
        assert "origin_xyz" not in components[0], f"origin_xyz must strip in mode={mode}"
        assert "world_xyz" not in components[0], f"world_xyz must strip in mode={mode}"


def test_tier_b_schema_advertises_xyz_rpy():
    """The design_robot schema must expose xyz and rpy so Claude can author
    them. Without this, the prompt-side guidance for novel-mode raw authoring
    would be unreachable."""
    from core.ai.claude_client import DESIGN_ROBOT_TOOL
    item_schema = DESIGN_ROBOT_TOOL["input_schema"]["properties"]["components"]["items"]["properties"]
    assert "xyz" in item_schema, "missing xyz in design_robot schema"
    assert "rpy" in item_schema, "missing rpy in design_robot schema"


def test_tier_a_schema_advertises_fields():
    """The design_robot tool schema must expose placement_offset_mm and
    splay_angle_deg so Claude can author them. Without this, the prompt-
    side guidance for novel mode would be dead letters."""
    from core.ai.claude_client import DESIGN_ROBOT_TOOL
    item_schema = DESIGN_ROBOT_TOOL["input_schema"]["properties"]["components"]["items"]["properties"]
    assert "placement_offset_mm" in item_schema, "missing placement_offset_mm in design_robot schema"
    assert "splay_angle_deg" in item_schema, "missing splay_angle_deg in design_robot schema"


def test_link_geometry_stripped_in_standard_mode():
    """Custom primitive composition (`link_geometry`) is novel-mode only.
    In standard mode the field must strip so it can't leak into the placement
    pipeline and dilute the well-tuned quadruped/arm/wheeled templates."""
    from core.ai.semantic_graph import strip_forbidden_fields
    components = [
        {"link_name": "torso", "component_id": "structural_baseplate_large",
         "attach_to": None,
         "link_geometry": [{"shape": "box", "size_mm": [200, 100, 300]}]},
    ]
    strip_forbidden_fields(components, declared_archetype=None)
    assert "link_geometry" not in components[0], \
        "link_geometry must strip in standard mode"


def test_link_geometry_preserved_in_novel_mode():
    """In novel mode the field flows through to the placement / visual
    pipeline. Without preservation, the AI's authored body shells would be
    silently discarded before reaching the resolver."""
    from core.ai.semantic_graph import strip_forbidden_fields
    components = [
        {"link_name": "torso", "component_id": "structural_baseplate_large",
         "attach_to": None,
         "link_geometry": [
             {"shape": "box", "size_mm": [200, 100, 300]},
             {"shape": "sphere", "radius_mm": 80, "xyz_mm": [0, 0, 360]},
         ]},
    ]
    strip_forbidden_fields(components, declared_archetype="novel")
    assert components[0].get("link_geometry"), \
        "link_geometry must survive in novel mode"
    assert len(components[0]["link_geometry"]) == 2


def test_link_geometry_schema_advertised():
    """The design_robot tool schema must expose link_geometry so Claude can
    use it; the modify_topology schema too so iterative edits don't silently
    drop the field."""
    from core.ai.claude_client import DESIGN_ROBOT_TOOL, MODIFY_TOPOLOGY_TOOL
    design_props = DESIGN_ROBOT_TOOL["input_schema"]["properties"]["components"]["items"]["properties"]
    assert "link_geometry" in design_props, "missing link_geometry in design_robot schema"
    modify_props = MODIFY_TOPOLOGY_TOOL["input_schema"]["properties"]["operations"]["items"]["properties"]
    assert "link_geometry" in modify_props, "missing link_geometry in modify_topology schema"


def test_declared_novel_skips_normalization():
    """A 4-leg assembly with declared_archetype='novel' must NOT be normalized."""
    components = [
        {"link_name": "base", "component_id": "structural_baseplate_large",
         "attach_to": None},
        {"link_name": "tail_servo", "component_id": "actuator_servo_standard",
         "attach_to": "base", "attach_face": "back", "joint_type": "revolute"},
    ]
    for i in range(4):
        components.append({
            "link_name": f"shin_{i}", "component_id": "structural_limb_link_slim",
            "attach_to": "base", "joint_type": "fixed",
        })
        components.append({
            "link_name": f"foot_{i}", "component_id": "mobility_rubber_foot_pad",
            "attach_to": f"shin_{i}", "joint_type": "fixed",
        })

    # Without declaration: quadruped normalizer runs and removes the tail.
    out_default, diags_default = normalize_assembly(components, {"tail": False})
    tail_present_default = any(c["link_name"] == "tail_servo" for c in out_default)
    assert not tail_present_default, "control: quadruped normalizer should remove the tail"

    # With novel declaration: nothing runs, tail survives.
    out_novel, diags_novel = normalize_assembly(
        components, {"tail": False}, declared_archetype="novel",
    )
    tail_present_novel = any(c["link_name"] == "tail_servo" for c in out_novel)
    assert tail_present_novel, "novel mode must skip normalization (tail must survive)"
    assert diags_novel == [], "novel mode must emit no diagnostics"


def main():
    failures = 0
    for fn in [
        test_standard_archetypes,
        test_novel_archetypes,
        test_standard_wins_over_novel,
        test_unknown_prompts_dont_trigger_freedom_mode,
        test_word_boundary_matches,
        test_eight_legs_no_longer_classified_as_quadruped,
        test_four_legs_still_classified_as_quadruped,
        test_tier_a_fields_stripped_in_standard_mode,
        test_tier_a_fields_preserved_in_novel_mode,
        test_tier_b_xyz_stripped_in_standard_mode,
        test_tier_b_xyz_preserved_in_novel_mode,
        test_tier_b_origin_xyz_aliases_still_forbidden,
        test_tier_b_schema_advertises_xyz_rpy,
        test_tier_a_schema_advertises_fields,
        test_link_geometry_stripped_in_standard_mode,
        test_link_geometry_preserved_in_novel_mode,
        test_link_geometry_schema_advertised,
        test_declared_novel_skips_normalization,
    ]:
        try:
            fn()
            print(f"PASS  {fn.__name__}")
        except AssertionError as e:
            print(f"FAIL  {fn.__name__}: {e}")
            failures += 1
    if failures:
        sys.exit(1)
    print("\nAll prompt-archetype tests passed.")


if __name__ == "__main__":
    main()
