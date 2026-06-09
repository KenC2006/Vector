"""Markdown report rendering for eval scorecards."""
from __future__ import annotations

from typing import Any, Dict, List, Optional

METRIC_ORDER = ["compile", "contact", "settle", "interpenetration", "ground", "structure", "mass"]


def _fmt_score(s: Any) -> str:
    if s is None:
        return "skip"
    return f"{float(s):.2f}"


def render_report(
    scorecards: List[Dict[str, Any]],
    compare_to: Optional[Dict[str, Dict[str, Any]]] = None,
) -> str:
    """scorecards: [{prompt_id, model?, score, metrics: {...}}].
    compare_to: {prompt_id: scorecard} from a baseline scores.json."""
    lines: List[str] = ["# Generation eval report", ""]

    header = "| prompt | overall |" + "".join(f" {m} |" for m in METRIC_ORDER)
    sep = "|---|---|" + "---|" * len(METRIC_ORDER)
    lines.append(header)
    lines.append(sep)
    for sc in scorecards:
        pid = sc.get("prompt_id", "?")
        base = (compare_to or {}).get(pid)
        cells = [_cell(sc.get("score"), base.get("score") if base else None)]
        for m in METRIC_ORDER:
            cur = (sc.get("metrics") or {}).get(m, {}).get("score")
            prev = ((base or {}).get("metrics") or {}).get(m, {}).get("score") if base else None
            cells.append(_cell(cur, prev))
        lines.append(f"| {pid} | " + " | ".join(cells) + " |")
    lines.append("")

    # Per-prompt detail: offenders and failed checks only.
    for sc in scorecards:
        pid = sc.get("prompt_id", "?")
        m = sc.get("metrics") or {}
        details: List[str] = []
        contact = m.get("contact") or {}
        if contact.get("offenders"):
            details.append(
                f"- contact: {contact.get('floating', 0)} floating / {contact.get('buried', 0)} buried — "
                + "; ".join(
                    f"`{o['link']}` {o['status']} (gap {o['gap_mm']}mm)"
                    for o in contact["offenders"][:6]
                )
            )
        inter = m.get("interpenetration") or {}
        if inter.get("worst_pair"):
            wp = inter["worst_pair"]
            details.append(
                f"- interpenetration: {inter.get('total_overlap_cm3', 0)}cm³ total; worst "
                f"`{wp['links'][0]}`×`{wp['links'][1]}` ({wp['cm3']}cm³)"
            )
        ground = m.get("ground") or {}
        if ground.get("non_foot_lowest_links"):
            details.append(
                "- ground: non-foot lowest links: "
                + ", ".join(f"`{o['link']}`" for o in ground["non_foot_lowest_links"][:6])
            )
        structure = m.get("structure") or {}
        failed = [c for c in structure.get("checks") or [] if not c.get("pass")]
        if failed:
            details.append("- structure: " + "; ".join(f"{c['name']}: {c['detail']}" for c in failed))
        settle = m.get("settle") or {}
        if settle.get("score") is not None and settle.get("score") < 1.0 and not settle.get("skipped"):
            reason = settle.get("reason") or ", ".join(
                k for k, v in (settle.get("checks") or {}).items() if not v
            )
            details.append(f"- settle: {reason} (height_ratio={settle.get('height_ratio')}, drift={settle.get('com_drift_m')})")
        mass = m.get("mass") or {}
        if mass.get("zero_mass_links"):
            details.append(f"- mass: zero-mass links: {', '.join(mass['zero_mass_links'][:6])}")

        if details:
            lines.append(f"## {pid}")
            lines.extend(details)
            lines.append("")

    return "\n".join(lines)


def _cell(cur: Any, prev: Any) -> str:
    if cur is None:
        return "skip"
    cur_f = float(cur)
    if prev is None:
        return f"{cur_f:.2f}"
    delta = cur_f - float(prev)
    arrow = "→" if abs(delta) < 0.005 else ("↑" if delta > 0 else "↓")
    return f"{cur_f:.2f} {arrow}{abs(delta):.2f}"
