"""
Preset-catalog rendering helpers for the Claude system prompt.

Exposes a single helper, `render_connector_hint`, which summarizes a preset's
authored mate connectors as a compact one-line string for the component
catalog. (The earlier dynamic/scoped-catalog scorer + keyword/RAG index that
also lived here was removed once the full-catalog path became unconditional —
see git history if you need it back.)
"""
from __future__ import annotations

from typing import Any


def render_connector_hint(comp: dict[str, Any]) -> str:
    """Compact summary of a preset's AUTHORED mate connectors for the catalog.

    Returns an empty string when no connectors are authored. Defaults
    (top/bottom/front/back/left/right) are intentionally omitted — they
    exist on every preset and are described once in the system prompt
    rather than repeated per-line.

    Shape: ` conn=[shaft_out(cyl 8mm out:+Z@18mm), top_face(plan)]`.
    Cylindrical entries include their diameter because it's the
    port-mismatch signal Claude needs to decide between `concentric` and
    `fastened`. Shaft/bore-classed connectors additionally carry the
    physical output/input axis (`out:`/`in:`) with its origin height —
    the model's only window into HOW a rotary part sits in its local
    frame, so it can reason "shaft exits the +Z face at 18mm" instead
    of guessing.
    """
    conns = comp.get("connectors") or []
    if not conns:
        return ""

    def _axis_label(c: dict[str, Any]) -> str:
        ax = c.get("axis_xyz")
        if not isinstance(ax, (list, tuple)) or len(ax) != 3:
            return ""
        for i, name in enumerate(("X", "Y", "Z")):
            v = ax[i]
            if isinstance(v, (int, float)) and abs(abs(v) - 1) < 1e-6:
                sign = "+" if v > 0 else "-"
                origin = c.get("origin_xyz_mm")
                at = ""
                if isinstance(origin, (list, tuple)) and len(origin) == 3 and isinstance(origin[i], (int, float)):
                    at = f"@{round(origin[i])}mm"
                return f"{sign}{name}{at}"
        return ""

    parts: list[str] = []
    for c in conns:
        if not isinstance(c, dict):
            continue
        cid = c.get("id")
        if not isinstance(cid, str) or not cid:
            continue
        ctype = c.get("type", "")
        short = {"cylindrical": "cyl", "planar": "plan", "point": "pt"}.get(ctype, ctype[:4])
        bits = [short]
        if ctype == "cylindrical" and c.get("diameter_mm") is not None:
            bits.append(f"{c['diameter_mm']}mm")
        cls = c.get("cls")
        if cls in ("shaft", "bore"):
            axis = _axis_label(c)
            if axis:
                bits.append(f"{'out' if cls == 'shaft' else 'in'}:{axis}")
        parts.append(f"{cid}({' '.join(bits)})")
    if not parts:
        return ""
    return f" conn=[{', '.join(parts)}]"
