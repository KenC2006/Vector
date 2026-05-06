"""Python client for the shared placement compiler (Phase 3b.5).

Spawns `node --experimental-strip-types src/src/compileAssemblyCli.ts` as a
single-shot subprocess, sends an AssemblyGraph as JSON on stdin, and parses
the resulting CompiledGraph from stdout.

The frontend URDF assembler and the AI redesign loop go through the same
compiler — Python and TypeScript must never disagree on placement, servo
splits, or RPY conventions. If you find yourself replicating geometry rules
in Python, route the work back through this client instead.
"""

from __future__ import annotations

import json
import subprocess
from pathlib import Path
from typing import Any, Optional

REPO_ROOT = Path(__file__).resolve().parents[2]
SRC_DIR = REPO_ROOT / "src"
CLI_REL = "src/compileAssemblyCli.ts"  # relative to SRC_DIR

DEFAULT_TIMEOUT_S = 30


class CompilerError(RuntimeError):
    """Raised when the compiler subprocess exits non-zero or returns malformed JSON."""


def compile_assembly(
    graph: dict,
    *,
    use_mate_connectors: bool = True,
    instance_overrides: Optional[dict] = None,
    timeout_s: float = DEFAULT_TIMEOUT_S,
) -> dict:
    """Compile an AssemblyGraph into a CompiledGraph via the TS placement compiler.

    Parameters
    ----------
    graph
        AssemblyGraph dict: ``{ "base_link": str, "components": [...], "ground_offset"?: float }``.
        Component dicts mirror the TypeScript AssemblyComponent shape
        (``link_name``, ``component_id``, ``attach_to``, ``attach_face``,
        ``joint_type``, ``joint_axis`` and the optional connector / length / rpy fields).
    use_mate_connectors
        Forwarded to the compiler. Default True matches the frontend.
    instance_overrides
        Per-link length_mm overrides for parametric extrusions; same shape as
        the compiler's ``instanceOverrides`` option.
    timeout_s
        Hard cap on the subprocess (single graphs compile in <1s, but a
        runaway shouldn't hang the AI loop).

    Returns
    -------
    dict
        CompiledGraph: ``{ "baseLink", "links", "attachIndex", "diagnostics",
        "fingerprint", "skippedClasses" }``. See
        ``src/src/placementCompiler/index.ts`` for field semantics.

    Raises
    ------
    CompilerError
        Subprocess exited non-zero or stdout was not valid JSON.
    """
    request = {
        "graph": graph,
        "useMateConnectors": use_mate_connectors,
        "instanceOverrides": instance_overrides or {},
    }
    proc = subprocess.run(
        ["node", "--experimental-strip-types", CLI_REL],
        input=json.dumps(request),
        capture_output=True,
        text=True,
        cwd=str(SRC_DIR),
        timeout=timeout_s,
    )
    if proc.returncode != 0:
        raise CompilerError(
            f"compileAssemblyCli exited {proc.returncode}: "
            f"{proc.stderr.strip() or proc.stdout.strip()}"
        )
    try:
        return json.loads(proc.stdout)
    except json.JSONDecodeError as e:
        raise CompilerError(
            f"compileAssemblyCli stdout was not JSON ({e}): {proc.stdout[:400]!r}"
        ) from e


def assert_no_skipped_classes(compiled: dict) -> None:
    """Sanity check — after Phase 3b.4.K cutover the compiler implements every
    placement class. A non-empty ``skippedClasses`` means an unexpected input
    or a regression in the compiler."""
    skipped = compiled.get("skippedClasses") or []
    if skipped:
        raise CompilerError(
            f"placement compiler reported skipped classes: {skipped} — "
            "either a regression or an unsupported AssemblyGraph shape"
        )


__all__ = ["CompilerError", "compile_assembly", "assert_no_skipped_classes"]
