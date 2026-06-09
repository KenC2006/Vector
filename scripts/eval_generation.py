"""Generation-quality eval harness.

Two modes:

  LIVE — generate robots from the fixed prompt set via the real AI path,
  save the AssemblyGraphs, compile, score:
      python scripts/eval_generation.py --live --out eval/runs/<name>
  Requires ANTHROPIC_API_KEY. Pass --out eval/baselines to (re)capture the
  committed baseline set.

  REPLAY — re-compile + re-score previously saved graphs (no API, no cost).
  This is the per-workstream regression gate:
      python scripts/eval_generation.py --replay eval/baselines \
          --compare eval/baselines/scores.json --out eval/runs/<name>

Scores are weighted (compile 25 / contact 20 / settle 20 / interpenetration 10
/ ground 10 / structure 10 / mass 5). Settle uses MuJoCo and is skipped
gracefully when unavailable (weights renormalize). Exit code: 0 on success,
2 when --compare shows the overall mean dropped by more than --tolerance
(default 0.05), 1 on harness errors.
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

# Windows consoles default to cp1252 — the report uses arrows/box characters.
for _stream in (sys.stdout, sys.stderr):
    try:
        _stream.reconfigure(encoding="utf-8", errors="replace")
    except (AttributeError, ValueError):
        pass

# Same .env convention as core/server.py — live mode needs ANTHROPIC_API_KEY.
try:
    from dotenv import load_dotenv
    load_dotenv(REPO_ROOT / ".env")
except ImportError:
    pass

from scripts.eval_lib.metrics import score_graph  # noqa: E402
from scripts.eval_lib.report import render_report  # noqa: E402


def _load_prompts(only: Optional[List[str]]) -> List[Dict[str, Any]]:
    data = json.loads((REPO_ROOT / "eval" / "prompts.json").read_text(encoding="utf-8"))
    prompts = data["prompts"]
    if only:
        wanted = set(only)
        prompts = [p for p in prompts if p["id"] in wanted]
        missing = wanted - {p["id"] for p in prompts}
        if missing:
            raise SystemExit(f"unknown prompt ids: {sorted(missing)}")
    return prompts


def _compile(graph: Dict[str, Any]) -> Dict[str, Any]:
    from core.ai.compiler_client import compile_assembly
    return compile_assembly(graph)


def _generate_live(prompt_spec: Dict[str, Any], model: Optional[str]) -> Dict[str, Any]:
    """One live generation. Returns {graph, mode_probe, explanation}."""
    from core.ai import claude_client

    kwargs: Dict[str, Any] = {}
    if model:
        kwargs["model"] = model

    # Mode probe (pre-WS1 only): record which archetype mode this prompt runs
    # under so baseline-vs-refactor deltas are interpretable. Defensive getattr
    # — the function is deleted in WS1.
    mode_probe = None
    probe_fn = getattr(claude_client, "_maybe_freedom_mode", None)
    if callable(probe_fn):
        try:
            _, is_novel = probe_fn(prompt_spec["prompt"])
            mode_probe = "novel" if is_novel else "standard"
        except Exception:
            mode_probe = "probe_failed"

    result = claude_client.generate_edit(
        prompt_spec["prompt"],
        current_urdf="",
        kinematic_graph_json={},
        session_id=f"eval_{prompt_spec['id']}_{int(time.time())}",
        **kwargs,
    )
    graph = result.get("assembly_graph")
    if not isinstance(graph, dict) or not graph.get("components"):
        raise RuntimeError(
            f"generation for '{prompt_spec['id']}' returned no assembly_graph "
            f"(stats={result.get('stats')!r})"
        )
    return {
        "graph": graph,
        "mode_probe": mode_probe,
        "explanation": (result.get("explanation") or "")[:500],
    }


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    mode = ap.add_mutually_exclusive_group(required=True)
    mode.add_argument("--live", action="store_true", help="generate via the AI path (needs ANTHROPIC_API_KEY)")
    mode.add_argument("--replay", metavar="DIR", help="re-score saved *.graph.json from DIR")
    ap.add_argument("--out", required=True, help="output dir for graphs/scorecards/report")
    ap.add_argument("--compare", metavar="SCORES_JSON", help="baseline scores.json to diff against")
    ap.add_argument("--prompts", help="comma-separated prompt ids (default: all)")
    ap.add_argument("--model", help="model override for --live")
    ap.add_argument("--no-sim", action="store_true", help="skip the MuJoCo settle metric")
    ap.add_argument("--tolerance", type=float, default=0.05,
                    help="max allowed drop in mean overall score vs --compare (default 0.05)")
    args = ap.parse_args()

    only = [s.strip() for s in args.prompts.split(",")] if args.prompts else None
    prompts = _load_prompts(only)
    out_dir = (REPO_ROOT / args.out).resolve() if not os.path.isabs(args.out) else Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)

    scorecards: List[Dict[str, Any]] = []
    failures: List[str] = []

    for spec in prompts:
        pid = spec["id"]
        graph: Optional[Dict[str, Any]] = None
        meta: Dict[str, Any] = {}
        try:
            if args.live:
                print(f"[eval] generating '{pid}' ...", file=sys.stderr)
                gen = _generate_live(spec, args.model)
                graph = gen["graph"]
                meta = {"mode_probe": gen["mode_probe"], "explanation": gen["explanation"]}
                (out_dir / f"{pid}.graph.json").write_text(
                    json.dumps(graph, indent=1), encoding="utf-8"
                )
            else:
                graph_path = Path(args.replay) / f"{pid}.graph.json"
                if not graph_path.is_absolute():
                    graph_path = REPO_ROOT / graph_path
                if not graph_path.exists():
                    print(f"[eval] no saved graph for '{pid}' — skipping", file=sys.stderr)
                    continue
                graph = json.loads(graph_path.read_text(encoding="utf-8"))

            print(f"[eval] compiling '{pid}' ...", file=sys.stderr)
            compiled = _compile(graph)
            result = score_graph(compiled, graph, spec, run_sim=not args.no_sim)
            card = {
                "prompt_id": pid,
                "prompt": spec["prompt"],
                "model": args.model,
                **meta,
                **result,
            }
        except Exception as e:
            failures.append(f"{pid}: {e}")
            card = {
                "prompt_id": pid,
                "prompt": spec["prompt"],
                "error": str(e),
                "score": 0.0,
                "metrics": {},
            }
        scorecards.append(card)
        print(f"[eval] {pid}: score={card.get('score')}", file=sys.stderr)

    compare_map: Optional[Dict[str, Dict[str, Any]]] = None
    if args.compare:
        cmp_path = Path(args.compare)
        if not cmp_path.is_absolute():
            cmp_path = REPO_ROOT / cmp_path
        baseline = json.loads(cmp_path.read_text(encoding="utf-8"))
        compare_map = {sc["prompt_id"]: sc for sc in baseline.get("scorecards", [])}

    (out_dir / "scores.json").write_text(
        json.dumps({"generated_at": time.strftime("%Y-%m-%dT%H:%M:%S"), "scorecards": scorecards}, indent=1),
        encoding="utf-8",
    )
    report = render_report(scorecards, compare_map)
    (out_dir / "report.md").write_text(report, encoding="utf-8")
    print(report)

    if failures:
        print(f"[eval] FAILURES: {failures}", file=sys.stderr)
        return 1

    if compare_map:
        cur = [sc["score"] for sc in scorecards if sc.get("score") is not None]
        prev = [compare_map[sc["prompt_id"]]["score"] for sc in scorecards
                if sc["prompt_id"] in compare_map and compare_map[sc["prompt_id"]].get("score") is not None]
        if cur and prev:
            drop = (sum(prev) / len(prev)) - (sum(cur) / len(cur))
            if drop > args.tolerance:
                print(f"[eval] REGRESSION: mean score dropped {drop:.3f} (> {args.tolerance})", file=sys.stderr)
                return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
