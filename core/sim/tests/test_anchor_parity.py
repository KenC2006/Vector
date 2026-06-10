"""Primitive-anchor parity corpus (Python side).

Mirrors src/src/primitiveAnchorCorpus.ts over the shared fixture file
scripts/primitive-anchor-corpus.json. The Python resolver feeds the AI's
spatial-context anchor tables; the TS resolver feeds compile-time placement —
they must agree to 1e-6 or Claude designs against positions the compiler
won't produce.

Run: python -m core.sim.tests.test_anchor_parity
"""
import json
import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..")))

from core.ai.primitive_anchors import anchor_names_for_primitive, resolve_anchor

EPS = 1e-6


def _close(a, b):
    return len(a) == len(b) and all(abs(x - y) < EPS for x, y in zip(a, b))


def main() -> int:
    corpus_path = os.path.abspath(os.path.join(
        os.path.dirname(__file__), "..", "..", "..", "scripts", "primitive-anchor-corpus.json"))
    with open(corpus_path, encoding="utf-8") as f:
        corpus = json.load(f)

    failed = 0
    for case in corpus["cases"]:
        res = resolve_anchor(case["primitive"], case["anchor"])
        if res is None:
            failed += 1
            print(f"  FAIL  {case['name']}: expected a pose, got None")
            continue
        if not _close(res["origin_xyz_mm"], case["origin_xyz_mm"]):
            failed += 1
            print(f"  FAIL  {case['name']}: origin {res['origin_xyz_mm']} != {case['origin_xyz_mm']}")
            continue
        if not _close(res["axis_xyz"], case["axis_xyz"]):
            failed += 1
            print(f"  FAIL  {case['name']}: axis {res['axis_xyz']} != {case['axis_xyz']}")
            continue
        print(f"  PASS  {case['name']}")

    for case in corpus["invalid_cases"]:
        res = resolve_anchor(case["primitive"], case["anchor"])
        if res is not None:
            failed += 1
            print(f"  FAIL  {case['name']}: expected None, got {res}")
            continue
        if case["anchor"] in anchor_names_for_primitive(case["primitive"]):
            failed += 1
            print(f"  FAIL  {case['name']}: anchor_names still lists {case['anchor']}")
            continue
        print(f"  PASS  {case['name']}")

    total = len(corpus["cases"]) + len(corpus["invalid_cases"])
    print(f"\nprimitive-anchor parity (Python): {total - failed}/{total} passed")
    return 0 if failed == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
