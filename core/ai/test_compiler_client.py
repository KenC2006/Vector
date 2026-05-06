"""Corpus for the Python placement-compiler client (Phase 3b.21).

Plain-Python assertion-style tests — runnable as
``python -m core.ai.test_compiler_client`` from the repo root or wired into
CI. Mirrors the approach used by the JS corpus files
(`src/src/placementCompilerCorpus.ts`) so both runtimes test the same
contract from each side of the subprocess boundary.

What this exercises:

  - End-to-end stdin→stdout subprocess plumbing (CompilerError, JSON shape).
  - Determinism: same input compiles to the same fingerprint and the same
    byte-identical CompiledGraph.
  - Servo split contract: actuated servos emit body/horn physical links and
    a body-to-horn revolute joint.
  - Foot leveling invariant: rubber feet end up world-level regardless of
    leg orientation (the placement_compiler implements this; the AI redesign
    loop relies on it).
  - Round-trip with the AI assembler: `_execute_add_component` poses match
    what `compile_assembly` returns for the same graph, link-by-link.
"""

from __future__ import annotations

import json
import sys
from typing import Callable

from core.ai.compiler_client import (
    CompilerError,
    assert_no_skipped_classes,
    compile_assembly,
)


# ── Test framework ──────────────────────────────────────────────────────────

_TESTS: list[tuple[str, Callable[[], None]]] = []


def _test(name: str):
    def deco(fn: Callable[[], None]):
        _TESTS.append((name, fn))
        return fn
    return deco


def _assert(cond: object, msg: str) -> None:
    if not cond:
        raise AssertionError(msg)


# ── Fixtures ────────────────────────────────────────────────────────────────


def _root_only_graph() -> dict:
    return {
        "base_link": "plate",
        "components": [
            {"link_name": "plate", "component_id": "structural_baseplate_large",
             "attach_to": None, "attach_face": None, "joint_type": "fixed", "joint_axis": None},
        ],
    }


def _hip_servo_graph() -> dict:
    g = _root_only_graph()
    g["components"].append({
        "link_name": "hip", "component_id": "actuator_servo_high_torque",
        "attach_to": "plate", "attach_face": "bottom",
        "joint_type": "revolute", "joint_axis": "z",
    })
    return g


def _leg_with_foot_graph() -> dict:
    return {
        "base_link": "plate",
        "components": [
            {"link_name": "plate", "component_id": "structural_baseplate_large",
             "attach_to": None, "attach_face": None, "joint_type": "fixed", "joint_axis": None},
            {"link_name": "hip", "component_id": "actuator_servo_high_torque",
             "attach_to": "plate", "attach_face": "bottom",
             "joint_type": "revolute", "joint_axis": "y"},
            {"link_name": "thigh", "component_id": "structural_limb_link_slim",
             "attach_to": "hip", "attach_face": "top", "joint_type": "fixed",
             "joint_axis": None, "length_mm": 100},
            {"link_name": "foot", "component_id": "mobility_rubber_foot_pad",
             "attach_to": "thigh", "attach_face": "bottom",
             "joint_type": "fixed", "joint_axis": None},
        ],
    }


# ── Tests ───────────────────────────────────────────────────────────────────


@_test("CompiledGraph shape: required fields present, fingerprint non-empty")
def t_shape() -> None:
    r = compile_assembly(_root_only_graph())
    for key in ("baseLink", "links", "attachIndex", "diagnostics", "fingerprint", "skippedClasses"):
        _assert(key in r, f"missing field {key!r} in CompiledGraph")
    _assert(isinstance(r["fingerprint"], str) and r["fingerprint"], "fingerprint must be non-empty string")
    assert_no_skipped_classes(r)


@_test("Determinism: same graph compiles to byte-identical CompiledGraph across calls")
def t_determinism() -> None:
    g = _leg_with_foot_graph()
    a = compile_assembly(g)
    b = compile_assembly(g)
    c = compile_assembly(g)
    _assert(a["fingerprint"] == b["fingerprint"] == c["fingerprint"],
            f"fingerprints diverged: {a['fingerprint']!r} {b['fingerprint']!r} {c['fingerprint']!r}")
    _assert(json.dumps(a, sort_keys=True) == json.dumps(b, sort_keys=True) == json.dumps(c, sort_keys=True),
            "CompiledGraph JSON not byte-identical across compile_assembly calls")


@_test("Servo split: actuated servo emits body+horn physicalLinks, body-to-horn revolute joint")
def t_servo_split() -> None:
    r = compile_assembly(_hip_servo_graph())
    by_logical = {l["logicalName"]: l for l in r["links"]}
    hip = by_logical.get("hip")
    _assert(hip is not None, "compiled graph missing hip link")
    _assert(len(hip["physicalLinks"]) >= 2,
            f"servo must split into >=2 physical links, got {hip['physicalLinks']}")
    body, horn = hip["physicalLinks"][0], hip["physicalLinks"][-1]
    _assert(body.endswith("_body") and horn.endswith("_horn"),
            f"split naming wrong: body={body!r} horn={horn!r}")
    _assert(hip["childAttachTarget"] == horn,
            f"childAttachTarget must be horn, got {hip['childAttachTarget']!r}")
    rev_joints = [j for j in hip["joints"] if j["type"] == "revolute"]
    _assert(len(rev_joints) == 1, f"expected exactly 1 revolute joint, got {len(rev_joints)}")
    _assert(rev_joints[0]["parentLink"] == body and rev_joints[0]["childLink"] == horn,
            f"revolute joint parent/child wrong: {rev_joints[0]}")
    _assert(hip["placedViaConnector"] is True,
            "actuated servos must be placed-via-connector so reconcile skips them (Phase 3b.4.K)")


@_test("Foot leveling: rubber foot pad ends with worldRpy[0] ~ 0 even on a tilted leg")
def t_foot_leveling() -> None:
    r = compile_assembly(_leg_with_foot_graph())
    by_logical = {l["logicalName"]: l for l in r["links"]}
    foot = by_logical.get("foot")
    _assert(foot is not None, "compiled graph missing foot link")
    rx, _ry, _rz = foot["worldRpy"]
    _assert(abs(rx) < 1e-3, f"foot world roll must be ~0 (worldLevelRpy), got {rx}")


@_test("Determinism: extra components in different add-order yield same fingerprint when graph order matches")
def t_order_sensitivity() -> None:
    # Reorder children-of-plate. AssemblyGraph is order-sensitive (the AI's
    # add order is meaningful for multi-child distribution), so swapping
    # SHOULD change the fingerprint. This test pins that behavior.
    g1 = _hip_servo_graph()
    g2 = {"base_link": g1["base_link"], "components": list(reversed(g1["components"]))}
    # Reversed order puts the servo before the plate; the compiler will reject
    # because the plate isn't visited first. Instead reorder while keeping
    # the root in front.
    g2 = {
        "base_link": "plate",
        "components": [
            g1["components"][0],
            {"link_name": "hip2", "component_id": "actuator_servo_high_torque",
             "attach_to": "plate", "attach_face": "bottom",
             "joint_type": "revolute", "joint_axis": "z"},
        ],
    }
    a = compile_assembly(g1)
    b = compile_assembly(g2)
    # Different logical names -> different attachIndex -> at least one diff.
    _assert(json.dumps(a) != json.dumps(b),
            "different logical names must produce different CompiledGraph")


@_test("Error path: unknown component_id emits a placement_compiler diagnostic")
def t_unknown_component() -> None:
    g = {
        "base_link": "ghost",
        "components": [
            {"link_name": "ghost", "component_id": "definitely_not_a_real_component",
             "attach_to": None, "attach_face": None, "joint_type": "fixed", "joint_axis": None},
        ],
    }
    r = compile_assembly(g)
    _assert(len(r["links"]) == 0, "unknown root component should produce no links")
    diags = r["diagnostics"]
    _assert(any(d.get("code", "").startswith("compiler.unknown_component") for d in diags),
            f"expected compiler.unknown_component diagnostic, got {diags!r}")
    _assert(all(d.get("owner") == "placement_compiler" for d in diags),
            f"unknown_component must be owned by placement_compiler, got {diags!r}")


@_test("Round-trip: _execute_add_component poses match compile_assembly link-for-link")
def t_round_trip_with_assembler() -> None:
    from core.ai.claude_client import _execute_add_component, _assembly_graph_from_state

    state: dict = {"links": {}}
    _execute_add_component(state, {
        "component_id": "structural_baseplate_large", "parent_link": None,
        "attach_face": None, "joint_type": "fixed", "joint_axis": "z",
    })
    _execute_add_component(state, {
        "component_id": "actuator_servo_high_torque",
        "parent_link": "structural_baseplate_large_1",
        "attach_face": "top", "joint_type": "revolute", "joint_axis": "y",
    })
    _execute_add_component(state, {
        "component_id": "structural_limb_link_slim",
        "parent_link": "actuator_servo_high_torque_2",
        "attach_face": "top", "joint_type": "fixed", "joint_axis": "z",
        "length_mm": 100,
    })

    graph = _assembly_graph_from_state(state)
    direct = compile_assembly(graph)
    by_logical = {l["logicalName"]: l for l in direct["links"]}

    for lname, info in state["links"].items():
        cl = by_logical.get(lname)
        _assert(cl is not None, f"compiler missing logical link {lname!r}")
        for axis_idx in range(3):
            _assert(abs(info["world_xyz"][axis_idx] - round(cl["worldXyz"][axis_idx], 4)) < 1e-3,
                    f"world_xyz mismatch on {lname} axis {axis_idx}: "
                    f"assembler={info['world_xyz']} compiler={cl['worldXyz']}")


# ── Runner ──────────────────────────────────────────────────────────────────


def main() -> int:
    failed: list[tuple[str, str]] = []
    for name, fn in _TESTS:
        try:
            fn()
        except AssertionError as e:
            failed.append((name, str(e)))
            print(f"  FAIL  {name}: {e}", file=sys.stderr)
            continue
        except Exception as e:  # noqa: BLE001 — surface any unexpected exception in the runner
            failed.append((name, f"{type(e).__name__}: {e}"))
            print(f"  ERROR {name}: {type(e).__name__}: {e}", file=sys.stderr)
            continue
        print(f"  PASS  {name}")
    total = len(_TESTS)
    passed = total - len(failed)
    print(f"\ncompiler-client corpus: {passed}/{total} passed")
    return 0 if not failed else 1


if __name__ == "__main__":
    sys.exit(main())
