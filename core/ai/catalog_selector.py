"""
Dynamic preset-catalog selector for the Claude system prompt.

Scores presets by relevance to the user request and the current graph,
then returns a compact subset (top-N + a hardcoded "core floor") instead
of dumping the full ~40-preset catalog on every call.

Design notes:
- Category comes from the preset JSON's top-level category key (normalized
  to singular: actuators → actuator). The preset_id prefix is used as a
  secondary signal.
- Core floor is hand-picked: the components Claude needs for any topology
  (baseplate root, one basic servo, one basic extrusion, an L-bracket,
  a flexible coupler). If the scorer fails, these still let Claude build
  a working robot.
- On retry, tried_preset_ids is merged in so Claude sees the presets it
  previously chose even if keyword match is weak. A "more presets
  available" hint is appended so Claude can ask for extras by name.

Controlled by the VECTOR_DYNAMIC_CATALOG env var — when unset/empty, the
caller uses the full-catalog path. This keeps a clean A/B baseline
against Workstreams #1-#3.
"""
from __future__ import annotations

import os
import re
from typing import Any, Iterable, Optional

# ── Core floor: always included, regardless of score ─────────────────────────
#
# Rationale: Claude's minimum viable topology is "baseplate + actuator +
# structural link." These five presets cover that floor. The two baseplate
# variants are listed explicitly because topology rule #1 in SYSTEM_PROMPT
# depends on both being visible (small-vs-large plate choice).
CORE_FLOOR_IDS: frozenset[str] = frozenset({
    "structural_baseplate",
    "structural_baseplate_large",
    "structural_extrusion_2020",
    "structural_bracket_l",
    "actuator_servo_standard",
    "transmission_flexible_coupling_jaw",
})

# Default cap on the scored portion. Plus the core floor. Plus any tried
# presets on retry. Typical final count is 20-30 presets.
DEFAULT_MAX_SCORED = 24

# ── Category aliases ─────────────────────────────────────────────────────────
#
# Keyword → category mapping for prompt-level category inference. When the
# user writes "arm with a gripper", we boost actuator + end_effectors.
#
# These lists intentionally err on the side of catching more tokens. Over-
# matching inflates the top-N into adjacent categories (cheap); under-
# matching hides a needed preset (expensive).
_PROMPT_CATEGORY_KEYWORDS: dict[str, tuple[str, ...]] = {
    "actuator": (
        "servo", "motor", "actuator", "joint", "revolute", "hinge",
        "axis", "torque", "dof", "drive", "spin", "rotate",
    ),
    "sensor": (
        "camera", "lidar", "imu", "sensor", "ultrasonic", "range",
        "force", "encoder", "depth", "vision", "see", "eye", "eyes",
        "scanner", "rangefinder",
    ),
    "mobility": (
        "wheel", "wheels", "caster", "mecanum", "rover", "car", "vehicle",
        "truck", "buggy", "cart", "bike", "motorcycle", "unicycle",
        "foot", "feet", "pad",
    ),
    "structural": (
        "bracket", "brackets", "extrusion", "frame", "plate", "baseplate",
        "rail", "body", "torso", "chassis", "beam", "strut", "shaft",
    ),
    "compute": (
        "mcu", "sbc", "microcontroller", "pi", "raspberry", "arduino",
        "controller", "electronics", "brain", "compute", "computer",
        "driver",
    ),
    "power": (
        "battery", "batteries", "lipo", "converter", "power", "pdu",
        "distribution", "regulator", "buck",
    ),
    "transmission": (
        "belt", "leadscrew", "bearing", "coupling", "coupler", "pulley",
        "gear", "transmission",
    ),
    "end_effectors": (
        "gripper", "grippers", "suction", "clamp", "claw", "tool",
        "finger", "fingers", "hand", "effector", "end-effector",
    ),
}

# Robot-archetype hints — these trigger extra categories even when
# individual keywords are absent.
_ARCHETYPE_CATEGORIES: dict[str, tuple[str, ...]] = {
    "arm": ("actuator", "structural", "end_effectors", "sensor"),
    "robotic arm": ("actuator", "structural", "end_effectors", "sensor"),
    "manipulator": ("actuator", "structural", "end_effectors"),
    "quadruped": ("actuator", "structural", "mobility", "sensor"),
    "dog": ("actuator", "structural", "mobility", "sensor"),
    "spot": ("actuator", "structural", "mobility", "sensor"),
    "go1": ("actuator", "structural", "mobility", "sensor"),
    "biped": ("actuator", "structural", "mobility", "sensor", "end_effectors"),
    "humanoid": ("actuator", "structural", "mobility", "sensor", "end_effectors"),
    "hexapod": ("actuator", "structural", "mobility", "sensor"),
    "rover": ("mobility", "sensor", "structural", "power", "compute"),
    "car": ("mobility", "sensor", "structural", "power"),
    "vehicle": ("mobility", "sensor", "structural", "power"),
    "drone": ("actuator", "sensor", "power", "compute"),
    # "robot" is the bare-signal archetype — used both on explicit match and
    # as the no-signal fallback below. Keeps vague first-turn prompts from
    # collapsing to only the core floor.
    "robot": ("actuator", "structural", "sensor"),
}

_WORD_RE = re.compile(r"[a-z0-9]+")


def _tokenize(text: str) -> list[str]:
    """Lowercase, strip punctuation, split into word-like tokens."""
    if not text:
        return []
    return _WORD_RE.findall(text.lower())


def _normalize_category(cat_name: str) -> str:
    """Map the preset JSON category name to the singular form used in scoring."""
    # end_effectors → effector feels cleaner but the JSON key is the
    # source of truth and Topology Rules reference it as "effector" singular.
    # Keep both pluralities stable: sensors→sensor, actuators→actuator, etc.
    if cat_name.endswith("s") and cat_name != "transmission" and cat_name != "end_effectors":
        return cat_name[:-1]
    return cat_name


# ── Index ────────────────────────────────────────────────────────────────────
#
# Built once, lazily. Each entry is a dict with pre-tokenized fields so
# scoring is a constant-time set lookup per preset.

_INDEX: Optional[list[dict[str, Any]]] = None


def _build_index(allowed_ids: Iterable[str]) -> list[dict[str, Any]]:
    """Enumerate allowed presets with tokens, category, and raw spec.

    allowed_ids is the caller's filter (matches _ALLOWED_COMPONENT_IDS in
    claude_client.py — we take it as a param instead of importing to keep
    this module free of circular deps).
    """
    from core.presets import get_all_categories, get_category
    allowed = set(allowed_ids)
    index: list[dict[str, Any]] = []
    for cat_name in get_all_categories():
        cat = get_category(cat_name)
        normalized = _normalize_category(cat_name)
        for comp in cat.get("components", []):
            if comp["id"] not in allowed:
                continue
            tokens = set(_tokenize(comp["id"]))
            tokens.update(_tokenize(comp.get("name", "")))
            tokens.update(_tokenize(comp.get("description", "")))
            index.append({
                "id": comp["id"],
                "name": comp.get("name", comp["id"]),
                "category_raw": cat_name,
                "category": normalized,
                "tokens": tokens,
                "comp": comp,
            })
    return index


def _get_index(allowed_ids: Iterable[str]) -> list[dict[str, Any]]:
    global _INDEX
    if _INDEX is None:
        _INDEX = _build_index(allowed_ids)
    return _INDEX


def reset_index_for_tests() -> None:
    """Drop the cached index. Only intended for unit tests."""
    global _INDEX
    _INDEX = None


# ── Scoring ──────────────────────────────────────────────────────────────────

def _graph_categories(
    kg_json: Optional[dict],
    index: list[dict[str, Any]],
) -> set[str]:
    """Return the set of normalized categories currently present in the graph.

    Used so mid-conversation edits ("add a sensor") still surface structural
    + actuator presets even when the prompt only names "sensor" — the graph
    establishes the archetype context.

    Category is resolved via the pre-built index rather than the id prefix
    because the two don't always agree: `motor_*` ids live in the
    `actuators` category, `effector_*` ids in `end_effectors`. Prefix-based
    inference would under-boost those.
    """
    if not kg_json:
        return set()
    id_to_cat = {e["id"]: e["category"] for e in index}
    cats: set[str] = set()
    # kg_json shape varies between input paths. Check both "links" (full
    # KinematicGraph.to_json) and "components" (assembly-style dump).
    candidates = []
    if isinstance(kg_json, dict):
        links = kg_json.get("links")
        if isinstance(links, list):
            candidates.extend(links)
        elif isinstance(links, dict):
            candidates.extend(links.values())
        comps = kg_json.get("components")
        if isinstance(comps, list):
            candidates.extend(comps)
    for entry in candidates:
        if not isinstance(entry, dict):
            continue
        cid = entry.get("component_id") or entry.get("preset_id") or entry.get("id")
        if not cid or not isinstance(cid, str):
            continue
        cat = id_to_cat.get(cid)
        if cat is None:
            # Id not in the allowed index — fall back to prefix so unknown
            # components still contribute a weak signal rather than nothing.
            cat = _normalize_category(cid.split("_", 1)[0])
        cats.add(cat)
    return cats


def _prompt_categories(prompt_tokens: set[str], prompt_lower: str) -> set[str]:
    """Infer which categories the user's words imply.

    Two signals: single-word keyword match (fast) and multi-word archetype
    phrases (robotic arm, mars rover, etc.).
    """
    cats: set[str] = set()
    for cat, keywords in _PROMPT_CATEGORY_KEYWORDS.items():
        if any(kw in prompt_tokens for kw in keywords):
            cats.add(cat)
    for phrase, archetype_cats in _ARCHETYPE_CATEGORIES.items():
        if phrase in prompt_lower:
            cats.update(archetype_cats)
    return cats


def _score_preset(
    entry: dict[str, Any],
    prompt_tokens: set[str],
    graph_cats: set[str],
    prompt_cats: set[str],
    tried_ids: set[str],
) -> int:
    """Relevance score. Higher = more likely to be injected.

    Weights are tuned so that:
    - explicit id mention dominates (user said the preset by name)
    - retry-tried presets are always selected
    - prompt category match outranks graph category match (user intent
      beats existing state when they disagree)
    - token overlap is the long-tail tie-breaker
    """
    score = 0
    pid = entry["id"]
    if pid in tried_ids:
        score += 1000
    if pid in prompt_tokens:
        # "structural_baseplate_large" tokenizes to structural/baseplate/large,
        # so an explicit id mention triggers both this branch AND the token
        # branch below. That's intentional — it bubbles the exact id above
        # same-category siblings.
        score += 100
    overlap = entry["tokens"] & prompt_tokens
    score += 5 * len(overlap)
    if entry["category"] in prompt_cats:
        score += 15
    if entry["category"] in graph_cats:
        score += 10
    return score


# ── Rendering ────────────────────────────────────────────────────────────────

def render_connector_hint(comp: dict[str, Any]) -> str:
    """Compact summary of a preset's AUTHORED mate connectors for the catalog.

    Returns an empty string when no connectors are authored. Defaults
    (top/bottom/front/back/left/right) are intentionally omitted — they
    exist on every preset and are described once in the system prompt
    rather than repeated per-line.

    Shape: ` conn=[shaft_out(cyl 8mm), top_face(plan)]`. Cylindrical
    entries include their diameter because it's the port-mismatch signal
    Claude needs to decide between `concentric` and `fastened` — planar
    and point entries are name-only (type alone disambiguates).
    """
    conns = comp.get("connectors") or []
    if not conns:
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
        if ctype == "cylindrical" and c.get("diameter_mm") is not None:
            parts.append(f"{cid}({short} {c['diameter_mm']}mm)")
        else:
            parts.append(f"{cid}({short})")
    if not parts:
        return ""
    return f" conn=[{', '.join(parts)}]"


def _render_preset_line(comp: dict[str, Any]) -> str:
    """Render one preset in the compact format used by _build_component_catalog.

    Kept byte-identical to the full-catalog path so switching between modes
    doesn't perturb Claude's reading of individual entries — only which
    entries are present changes.
    """
    phys = comp["physical"]
    mass = phys.get("mass_kg") or phys.get("mass_kg_per_100mm")
    if mass is None:
        mass_str = "?"
    elif mass >= 1:
        mass_str = f"{mass}kg"
    else:
        mass_str = f"{round(mass*1000)}g"
    bb = phys.get("bounding_box_mm", [])
    bb_str = f"{bb[0]}x{bb[1]}x{bb[2]}mm" if len(bb) >= 3 else ""
    shape = phys.get("inertia_primitive", "box")
    me = comp.get("mechanical_electrical", {})
    spec = ""
    if "max_torque_nm" in me: spec = f"{me['max_torque_nm']}Nm"
    elif "holding_torque_nm" in me: spec = f"{me['holding_torque_nm']}Nm"
    elif "max_force_n" in me: spec = f"{me['max_force_n']}N"
    elif "grip_force_n" in me: spec = f"{me['grip_force_n']}N"
    elif "capacity_mah" in me: spec = f"{me['capacity_mah']}mAh"
    elif "fov_h_deg" in me: spec = f"{me['fov_h_deg']}°FOV"
    elif "range_m" in me: spec = f"{me['range_m']}m"
    conn_hint = render_connector_hint(comp)
    return f"  - {comp['id']}: {comp['name']} [{mass_str}, {bb_str}, {shape}]{(' ' + spec) if spec else ''}{conn_hint}"


def _render_catalog(selected_ids: set[str], index: list[dict[str, Any]]) -> str:
    """Group selected presets by category and render in a stable order.

    Uses the same category ordering as the preset JSON (preserved by
    `get_all_categories`) so Claude sees a consistent layout across calls.
    """
    from core.presets import get_all_categories
    by_cat: dict[str, list[dict[str, Any]]] = {}
    for entry in index:
        if entry["id"] in selected_ids:
            by_cat.setdefault(entry["category_raw"], []).append(entry)
    lines: list[str] = []
    for cat_name in get_all_categories():
        entries = by_cat.get(cat_name)
        if not entries:
            continue
        label = cat_name.replace("_", " ").title()
        lines.append(f"\n{label} ({len(entries)}):")
        for entry in entries:
            lines.append(_render_preset_line(entry["comp"]))
    return "\n".join(lines)


# ── Public API ───────────────────────────────────────────────────────────────

def dynamic_catalog_enabled() -> bool:
    """Feature flag: on when VECTOR_DYNAMIC_CATALOG is set to a truthy value.

    Kept as a separate predicate so call sites can cheaply skip building
    the scoped catalog when the flag is off — the full-catalog path is
    identical to pre-WS4 behavior.
    """
    val = os.environ.get("VECTOR_DYNAMIC_CATALOG", "").strip().lower()
    return val not in ("", "0", "false", "off", "no")


def build_scoped_catalog(
    user_prompt: str,
    allowed_ids: Iterable[str],
    kg_json: Optional[dict] = None,
    tried_preset_ids: Optional[Iterable[str]] = None,
    max_scored: int = DEFAULT_MAX_SCORED,
) -> str:
    """Return a catalog string scoped to presets relevant to this request.

    Always includes the core floor. Always includes any preset in
    tried_preset_ids (so retry context surfaces Claude's previous picks).
    Fills the remaining budget with the top-scoring presets.

    Appends a "more-presets-available" hint so Claude knows the catalog
    is intentionally truncated — mitigates the hidden-preset failure mode.
    """
    index = _get_index(allowed_ids)
    if not index:
        return "(Component catalog unavailable)"

    prompt_tokens = set(_tokenize(user_prompt))
    prompt_lower = (user_prompt or "").lower()
    graph_cats = _graph_categories(kg_json, index)
    prompt_cats = _prompt_categories(prompt_tokens, prompt_lower)
    tried_ids = set(tried_preset_ids or [])

    # No-signal fallback: terse text prompts like "build this" (especially
    # paired with an image), "make something", or an empty string produce
    # no category hints. When the scorer has nothing to narrow on, return
    # the full catalog rather than guessing at a baseline — safer than
    # hiding categories the user might need (a "build this" + rover image
    # still needs wheels; + arm image still needs grippers). We can't read
    # the image, so we shouldn't pretend to scope.
    if not prompt_cats and not graph_cats and not tried_ids:
        all_ids = {e["id"] for e in index}
        return _render_catalog(all_ids, index)

    selected: set[str] = set(CORE_FLOOR_IDS) & {e["id"] for e in index}
    selected.update(tried_ids & {e["id"] for e in index})

    scored: list[tuple[int, str]] = []
    for entry in index:
        if entry["id"] in selected:
            continue
        s = _score_preset(entry, prompt_tokens, graph_cats, prompt_cats, tried_ids)
        if s > 0:
            scored.append((s, entry["id"]))
    # Sort by score desc, then id asc for stable output (Claude sees the
    # same order across runs with the same prompt — prompt cache stays warm).
    scored.sort(key=lambda t: (-t[0], t[1]))
    for _, pid in scored[:max_scored]:
        selected.add(pid)

    body = _render_catalog(selected, index)
    omitted = sum(1 for e in index if e["id"] not in selected)
    if omitted > 0:
        body += (
            f"\n\n_Note: {omitted} additional presets exist but were omitted as "
            f"unlikely to be relevant to this request. If none of the above fits, "
            f"mention the component type you need in your explanation and the full "
            f"catalog will be surfaced on retry._"
        )
    return body
