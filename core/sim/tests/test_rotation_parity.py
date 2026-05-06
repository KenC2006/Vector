"""Phase 5b — Python half of the rotation parity contract.

Loads ``scripts/rotation-corpus.json`` and verifies that
``core.sim.urdf_to_mjcf._rpy_to_quat`` matches the corpus exactly. The
TypeScript ``rpyToQuat`` is verified against the same corpus from the
frontend side (``src/src/rotationParityCorpus.ts``), so any divergence
between the two converters fails this test on at least one runtime.

Run as ``python -m core.sim.tests.test_rotation_parity`` from the repo
root, or via pytest.
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

from core.sim.urdf_to_mjcf import _rpy_to_quat

REPO_ROOT = Path(__file__).resolve().parents[3]
CORPUS_PATH = REPO_ROOT / "scripts" / "rotation-corpus.json"


def _matrix_from_quat(w: float, x: float, y: float, z: float) -> list[list[float]]:
    return [
        [1 - 2*(y*y + z*z),   2*(x*y - z*w),     2*(x*z + y*w)],
        [2*(x*y + z*w),       1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w),       2*(y*z + x*w),     1 - 2*(x*x + y*y)],
    ]


def main() -> int:
    with CORPUS_PATH.open("r", encoding="utf-8") as fp:
        corpus = json.load(fp)
    tol = float(corpus["tolerance"])

    failed: list[tuple[str, str]] = []
    for case in corpus["cases"]:
        name = case["name"]
        rpy = case["rpy"]
        got = _rpy_to_quat(*rpy)
        exp = case["quat_wxyz"]
        # Quaternion sign ambiguity: q and -q encode the same rotation.
        dot = sum(g * e for g, e in zip(got, exp))
        sign = -1.0 if dot < 0 else 1.0
        q_err = max(abs(sign * g - e) for g, e in zip(got, exp))

        m_got = _matrix_from_quat(*got)
        m_exp = case["matrix_row_major"]
        m_err = max(
            abs(m_got[i][j] - m_exp[i][j]) for i in range(3) for j in range(3)
        )

        if q_err > tol or m_err > tol:
            failed.append((name, f"qErr={q_err:.2e} mErr={m_err:.2e}"))
            print(f"  FAIL  {name}: qErr={q_err:.2e} mErr={m_err:.2e}", file=sys.stderr)
        else:
            print(f"  PASS  {name}")

    total = len(corpus["cases"])
    passed = total - len(failed)
    print(f"\nrotation-parity (Python): {passed}/{total} passed")
    return 0 if not failed else 1


if __name__ == "__main__":
    sys.exit(main())
