#!/usr/bin/env python3
"""
Fixtures for the dynamic preset-catalog selector (Workstream #4).

Covers the "Done when" criteria from docs/IMPROVEMENT_PLAN.md §4:
- A request with no actuator keywords against a fresh graph does NOT
  include actuator-only presets beyond the core floor.
- System prompt size drops measurably for a typical request vs. the
  full-catalog baseline.
- Core floor is always present.
- On retry, previously-tried presets are always included.

Usage:
    python core/test_dynamic_catalog.py
"""
from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from core.ai.catalog_selector import (
    CORE_FLOOR_IDS,
    build_scoped_catalog,
    dynamic_catalog_enabled,
    render_connector_hint,
    reset_index_for_tests,
)
from core.ai.claude_client import _ALLOWED_COMPONENT_IDS, _build_component_catalog


failures: list[str] = []


def expect(cond: bool, label: str, detail: str = "") -> None:
    if cond:
        print(f"  PASS: {label}")
    else:
        print(f"  FAIL: {label}")
        if detail:
            print(f"        {detail}")
        failures.append(label)


def scoped(**kwargs) -> str:
    reset_index_for_tests()
    return build_scoped_catalog(allowed_ids=_ALLOWED_COMPONENT_IDS, **kwargs)


def ids_in(catalog: str) -> set[str]:
    """Extract the preset ids listed in a catalog string."""
    out: set[str] = set()
    for line in catalog.splitlines():
        stripped = line.strip()
        if stripped.startswith("- "):
            body = stripped[2:].split(":", 1)[0].strip()
            out.add(body)
    return out


def test_core_floor_always_present() -> None:
    print("\n[1] Core floor always present")
    cat = scoped(user_prompt="add a camera")
    present = ids_in(cat)
    missing = [cid for cid in CORE_FLOOR_IDS if cid in _ALLOWED_COMPONENT_IDS and cid not in present]
    expect(not missing, "all core-floor ids injected", f"missing: {missing}")


def test_structural_only_prompt_no_actuators() -> None:
    """Spec fixture: request with no actuator keywords against a fresh graph
    should not inject actuator-only presets beyond the core floor."""
    print("\n[2] No-actuator prompt against fresh graph")
    cat = scoped(user_prompt="add a depth camera pointing at the floor", kg_json=None)
    present = ids_in(cat)
    actuator_ids = {
        pid for pid in present
        if pid.startswith("actuator_") or pid.startswith("motor_")
    }
    # Core floor ships actuator_servo_standard — that's the allowed exception.
    extra_actuators = actuator_ids - CORE_FLOOR_IDS
    expect(
        not extra_actuators,
        "no actuator presets beyond core floor",
        f"unexpected actuators: {sorted(extra_actuators)}",
    )
    expect(
        "sensor_depth_camera_small" in present,
        "depth camera preset surfaced from prompt keyword",
    )


def test_graph_category_boosts_related_presets() -> None:
    print("\n[3] Existing graph categories steer selection")
    graph = {"components": [
        {"link_name": "structural_baseplate_large_1", "component_id": "structural_baseplate_large"},
        {"link_name": "actuator_servo_high_torque_1", "component_id": "actuator_servo_high_torque"},
        {"link_name": "mobility_wheel_driven_1", "component_id": "mobility_wheel_driven"},
    ]}
    cat = scoped(user_prompt="add another one", kg_json=graph)
    present = ids_in(cat)
    expect(
        any(pid.startswith("mobility_") for pid in present),
        "mobility presets surface when mobility is already on graph",
    )


def test_graph_category_from_prefix_mismatch() -> None:
    """Regression: motor_* ids live in category=actuators; effector_* ids
    live in category=end_effectors. Graph-category extraction must use the
    index, not the id prefix, or these graphs produce zero category
    boost."""
    print("\n[3b] Motor/effector ids route to correct category")
    graph = {"components": [
        {"link_name": "structural_baseplate_1", "component_id": "structural_baseplate"},
        {"link_name": "motor_dc_small_130_1", "component_id": "motor_dc_small_130"},
        {"link_name": "effector_suction_cup_1", "component_id": "effector_suction_cup"},
    ]}
    cat = scoped(user_prompt="extend this", kg_json=graph)
    present = ids_in(cat)
    # actuators and end_effectors should both see a graph-category boost,
    # which in turn surfaces at least one additional actuator + effector
    # beyond what's in the graph already.
    actuator_other = {pid for pid in present
                      if (pid.startswith("actuator_") or pid.startswith("motor_"))
                      and pid != "motor_dc_small_130"}
    effector_other = {pid for pid in present
                      if pid.startswith("effector_")
                      and pid != "effector_suction_cup"}
    expect(
        len(actuator_other) >= 1,
        "graph with motor_* boosts other actuator-category presets",
    )
    expect(
        len(effector_other) >= 1,
        "graph with effector_* boosts other end_effectors-category presets",
    )


def test_tried_presets_always_surface_on_retry() -> None:
    print("\n[4] Retry: tried presets always included")
    tried = ["sensor_lidar_2d", "effector_suction_cup"]
    cat = scoped(
        user_prompt="redesign the robot — validator complained about components",
        tried_preset_ids=tried,
    )
    present = ids_in(cat)
    missing = [pid for pid in tried if pid in _ALLOWED_COMPONENT_IDS and pid not in present]
    expect(not missing, "all tried presets injected on retry", f"missing: {missing}")


def test_scoped_smaller_than_full() -> None:
    """Measure the token-size win — primary motivation per the plan."""
    print("\n[5] Scoped catalog is smaller than the full dump")
    full = _build_component_catalog()
    scoped_cat = scoped(user_prompt="build a robotic arm with a camera")
    scoped_len = len(scoped_cat)
    full_len = len(full)
    ratio = scoped_len / full_len if full_len else 1.0
    print(f"    full={full_len} chars, scoped={scoped_len} chars, ratio={ratio:.2f}")
    expect(
        scoped_len < full_len,
        "scoped is strictly smaller than full",
        f"scoped={scoped_len}, full={full_len}",
    )
    # Plan target is >=40% reduction on a typical request. Allow some
    # slack — arm requests recruit several categories, so we accept >=15%
    # for the fixture and leave the tighter target to empirical testing.
    expect(
        scoped_len < full_len * 0.85,
        "scoped reduces catalog size >=15% on arm request",
    )


def test_omitted_hint_present_when_truncated() -> None:
    print("\n[6] 'More presets available' hint surfaces when truncated")
    cat = scoped(user_prompt="add a lidar")
    expect(
        "additional presets exist but were omitted" in cat,
        "hint footer present in scoped output",
    )


def test_explicit_preset_id_mention_wins() -> None:
    print("\n[7] Explicit preset-id mention dominates")
    cat = scoped(user_prompt="I want the transmission_leadscrew_8mm on this build")
    present = ids_in(cat)
    expect(
        "transmission_leadscrew_8mm" in present,
        "explicit id mention surfaces the preset",
    )


def test_no_signal_fallback() -> None:
    """Vague first-turn prompts ("build this" + image, "make something",
    etc.) produce no category hints. On no-signal, fall back to the full
    catalog rather than guessing at a baseline — we can't see the image
    or infer intent, so hiding categories (mobility/effectors/power/etc.)
    risks the "build this + rover image -> no wheels" failure mode."""
    print("\n[9] No-signal fallback returns full catalog")
    reset_index_for_tests()
    full_cat = _build_component_catalog()
    full_ids = ids_in(full_cat)
    for prompt in ("build this", "make something cool", "", "go"):
        cat = scoped(user_prompt=prompt)
        present = ids_in(cat)
        missing = full_ids - present
        expect(
            not missing,
            f"no-signal on prompt={prompt!r} returns full catalog",
            f"missing: {sorted(missing)}",
        )


def test_feature_flag_gate() -> None:
    print("\n[8] Feature-flag gate")
    old = os.environ.get("VECTOR_DYNAMIC_CATALOG")
    try:
        os.environ.pop("VECTOR_DYNAMIC_CATALOG", None)
        expect(not dynamic_catalog_enabled(), "flag defaults OFF")
        os.environ["VECTOR_DYNAMIC_CATALOG"] = "1"
        expect(dynamic_catalog_enabled(), "flag ON when value='1'")
        os.environ["VECTOR_DYNAMIC_CATALOG"] = "0"
        expect(not dynamic_catalog_enabled(), "flag OFF when value='0'")
        os.environ["VECTOR_DYNAMIC_CATALOG"] = "false"
        expect(not dynamic_catalog_enabled(), "flag OFF when value='false'")
    finally:
        if old is None:
            os.environ.pop("VECTOR_DYNAMIC_CATALOG", None)
        else:
            os.environ["VECTOR_DYNAMIC_CATALOG"] = old


def test_connector_hint_renders_authored_only() -> None:
    """render_connector_hint exposes authored connectors to Claude in a
    compact shape. Defaults (top/bottom/front/back/left/right) stay implicit —
    they exist on every preset and are described once in the system prompt."""
    print("\n[10] render_connector_hint: authored connectors surface, defaults don't")
    # No connectors -> empty string (every default-connector preset goes this path).
    expect(render_connector_hint({}) == "", "absent `connectors` -> empty hint")
    expect(render_connector_hint({"connectors": []}) == "", "empty `connectors` -> empty hint")

    # Cylindrical: must include diameter (port-mismatch signal).
    servo = {"connectors": [
        {"id": "shaft_out", "type": "cylindrical", "axis_xyz": [0, 0, 1], "diameter_mm": 8}
    ]}
    hint = render_connector_hint(servo)
    expect("shaft_out" in hint, "cylindrical id surfaces", hint)
    expect("cyl" in hint, "cylindrical short-type surfaces", hint)
    expect("8mm" in hint, "cylindrical diameter surfaces", hint)

    # Planar + point: no diameter, short-type only.
    camera = {"connectors": [
        {"id": "mount_back",    "type": "planar", "axis_xyz": [-1, 0, 0]},
        {"id": "optical_front", "type": "point",  "axis_xyz": [ 1, 0, 0]},
    ]}
    hint_c = render_connector_hint(camera)
    expect("mount_back(plan)" in hint_c, "planar short-type surfaces", hint_c)
    expect("optical_front(pt)" in hint_c, "point short-type surfaces", hint_c)
    expect("mm" not in hint_c, "no diameter on planar/point entries", hint_c)


def test_catalog_includes_authored_connectors() -> None:
    """The full catalog dump that lands in Claude's system prompt carries
    the authored-connector hint on every preset that opts in. Presets
    without authored connectors stay byte-identical to the pre-Phase-4
    shape."""
    print("\n[11] _build_component_catalog: connector hint on authored presets only")
    cat = _build_component_catalog()
    # Authored connectors on the allowed-id presets (Phase 3 landed these):
    # three servo variants + camera + L-bracket. The coupler disc and heavy-
    # duty servo aren't in _ALLOWED_COMPONENT_IDS, so the catalog won't
    # render them at all — the test only checks presets Claude can see.
    must_have_hints = [
        ("actuator_servo_micro",      "shaft_out(cyl"),
        ("actuator_servo_standard",   "shaft_out(cyl"),
        ("actuator_servo_high_torque","shaft_out(cyl 8mm)"),
        ("sensor_depth_camera_small", "mount_back(plan), optical_front(pt)"),
        ("structural_bracket_l",      "plate_top(plan), wall_inner(plan), wall_outer(plan)"),
    ]
    for pid, needle in must_have_hints:
        line = next((l for l in cat.splitlines() if l.strip().startswith(f"- {pid}:")), None)
        expect(line is not None, f"{pid} present in catalog", "")
        if line is not None:
            expect(needle in line, f"{pid} line carries '{needle}'", line)

    # Presets WITHOUT authored connectors must NOT carry a `conn=` suffix
    # (byte-identical to the pre-Phase-4 layout on those lines).
    baseplate_line = next(
        (l for l in cat.splitlines() if l.strip().startswith("- structural_baseplate:")),
        None,
    )
    expect(baseplate_line is not None, "baseplate line present")
    if baseplate_line is not None:
        expect(" conn=" not in baseplate_line, "baseplate has no conn= suffix", baseplate_line)


def main() -> int:
    test_core_floor_always_present()
    test_structural_only_prompt_no_actuators()
    test_graph_category_boosts_related_presets()
    test_graph_category_from_prefix_mismatch()
    test_tried_presets_always_surface_on_retry()
    test_scoped_smaller_than_full()
    test_omitted_hint_present_when_truncated()
    test_explicit_preset_id_mention_wins()
    test_no_signal_fallback()
    test_feature_flag_gate()
    test_connector_hint_renders_authored_only()
    test_catalog_includes_authored_connectors()

    print()
    if failures:
        print(f"FAILED: {len(failures)} check(s)")
        for name in failures:
            print(f"  - {name}")
        return 1
    print("All dynamic-catalog fixtures passed.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
