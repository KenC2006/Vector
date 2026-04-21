"""
Critique classifier for the Gemini validation loop (Workstream #3b).

Gemini's critiques sometimes ask for things Vector's catalog can't satisfy
("add a rocker-bogie", "widen feet to 100mm", "add a display screen"). Sending
those back to Claude as a redesign target produces loops that never converge.

Flow per failed checklist item:
  1. Deterministic pre-filter — extracts capability phrases from the critique
     and matches them against the component catalog. Three verdicts:
       - drop   → phrase names a component/concept with no catalog match
       - keep   → phrase resolves cleanly to an existing preset
       - ambig  → critique doesn't name parts directly ("legs too thin")
  2. LLM stage — Sonnet (not Haiku; Haiku is too agreeable for nuanced
     feasibility calls) is invoked on the ambiguous remainder in a single
     batched JSON call.

Classifier verdict lands on the checklist item as
  {"classifier_drop": bool, "classifier_reason": str}.

The TS side reads `classifier_drop` to skip items when deciding whether to
trigger a Claude redesign.
"""

from __future__ import annotations

import json
import os
import re
import sys
from typing import Dict, List, Optional, Set, Tuple

try:
    from core.ai.claude_client import _get_client, _parse_json_response
except Exception as _e:  # pragma: no cover — import ordering guard
    _get_client = None
    _parse_json_response = None

# Phrases Gemini has historically emitted that map to components/concepts the
# catalog does not carry. Matched on lowercased critique detail with word
# boundaries. Additions should come from real critique logs, not speculation.
INFEASIBLE_PHRASES: List[str] = [
    "rocker-bogie",
    "rocker bogie",
    "display screen",
    "display panel",
    "face screen",
    "monitor",
    "lcd",
    "pedestal spread",
    "spreading base",
    "spreading pedestal",
    "tank tread",
    "continuous track",
    "legged track",
    "cable routing",
    "wire management",
    "heat sink",
    "cooling fan",
    "dust cover",
    "enclosure panel",
]

# Regex patterns for critiques that don't reduce to a single phrase. These
# catch dimension-specific asks ("widen feet to 100mm", "baseplate 60-100mm
# thick") that the catalog can't satisfy even though they name real parts.
# Kept narrow — each pattern must require both a part hint and an infeasible
# qualifier (unsupported dimension / reshape verb).
INFEASIBLE_PATTERNS: List[Tuple[str, str]] = [
    # Verb-pattern: extended to include singular "foot" (Gemini sometimes refers
    # to a single specific foot pad by name).
    (r"\b(widen|wider|enlarge|broaden|spread)\s+(?:the\s+)?(foot|feet|foot pads?)\b", "reshape feet (fixed preset dimension)"),
    # Singular "foot" + dimension within ~40 chars catches the real-world
    # "foot should be at least ~40mm" pattern that the original tight regex
    # ("feet (should be|to|at) \d mm") missed. CATALOG_KEYWORDS would otherwise
    # short-circuit "foot" to KEEP and let the redesign loop chase a 40mm foot
    # preset that doesn't exist (drove the broken quadruped loop on 2026-04-20).
    (r"\b(?:foot|feet|foot pads?)\b[^.]{0,40}?\b\d+\s*mm\b", "foot resized to a specific mm (no such preset)"),
    (r"\bbaseplate\s+(?:should\s+be\s+|thick(?:ness)?\s+of\s+)?\d+(?:\s*-\s*\d+)?\s*mm", "baseplate thickness re-spec (preset is fixed)"),
    (r"\btorso\s+(?:should\s+be\s+)?\d+\s*mm", "torso resized to a specific mm (no such preset)"),
]

# Anatomical / aesthetic critiques ported from the prior TS regex (removed in
# this workstream from viewportChat.ts). Restored as deterministic drops so
# the pipeline still rejects them when the Sonnet LLM stage is unavailable —
# without this, the quadruped bugfix (mammal-like leg-mirror redesign loop)
# regresses whenever ANTHROPIC_API_KEY is missing. The old regex paired these
# with `actionableHints` (see GATE_KEEP_PATTERNS below) so a critique that
# names a missing component wins over aesthetic language in the same sentence.
INFEASIBLE_AESTHETIC_PATTERNS: List[Tuple[str, str]] = [
    (r"\b(boxy|integrated body|mammal(?:-|\s)like|spot(?:-|\s)style|dachshund)\b", "aesthetic/anatomical critique (no preset action available)"),
    (r"\bknees?\s+(?:point|bend|face|angle)\w*", "knee-angle critique (not a preset-selectable property)"),
    (r"\bmirror(?:ed)?\s+\w*\s*(?:pitch|leg|knee|hip|limb|orient|front|rear)\b", "leg-mirror critique (same-sign rpy is the Spot look; reshuffling regresses design)"),
    (r"\b(?:leg|knee|hip|limb)s?\s+\w*\s*mirror(?:ed)?\b", "leg-mirror critique (same-sign rpy is the Spot look; reshuffling regresses design)"),
    (r"\b(?:wrong|off|bad|incorrect)\s+(?:proportions?|ratio|aspect)\b", "proportion critique (no preset reshapes existing components)"),
    (r"\b(?:too\s+(?:thin|thick|narrow|wide)|thin(?:ness)?|thick(?:ness)?)\s+(?:chassis|torso|baseplate|body)\b", "dimension critique on structural preset (fixed dimensions)"),
]

# If any of these match, the aesthetic patterns above are disarmed — the
# critique names a concrete missing/wrong component or a specific connection
# change that Claude CAN act on, even if it's wrapped in anatomical language.
# Mirrors the `actionableHints` guard in the old TS regex.
_ACTIONABLE_PARTS = r"(?:gripper|sensor|servo|wheel|battery|leg|head|arm|hip|knee|foot|imu|camera|extrusion|bracket|lidar|coupler|coupling|bearing)"
GATE_KEEP_PATTERNS: List[str] = [
    rf"\b(missing|absent|forgot|no)\s+(?:a\s+|an\s+|the\s+)?{_ACTIONABLE_PARTS}\b",
    r"\bshould\s+(?:be\s+)?(?:attach|connect|added|reoriented|swapped|replaced)",
    r"\bwrong\s+(?:component|connection|attach|part|preset)\b",
    rf"\b(add|attach|mount|insert|swap|replace)\s+(?:a\s+|an\s+|the\s+)?{_ACTIONABLE_PARTS}\b",
]

# Component keywords mined from _ALLOWED_COMPONENT_IDS + preset names. Used by
# the fuzzy-match pass to decide whether a noun phrase in the critique
# resolves to a catalog entry. Kept deliberately narrow — general words
# ("base", "frame") are excluded because they match too liberally.
CATALOG_KEYWORDS: Set[str] = {
    "servo", "motor", "actuator", "gripper", "wheel", "caster",
    "mecanum", "foot pad", "foot", "baseplate", "extrusion", "bracket",
    "l-bracket", "u-bracket", "shaft collar", "bearing", "belt",
    "leadscrew", "coupling", "coupler", "camera", "lidar", "imu",
    "ultrasonic", "force torque", "encoder", "mcu", "sbc", "driver",
    "battery", "lipo", "buck converter", "distribution", "suction",
}


def _normalize(text: str) -> str:
    return (text or "").lower().strip()


def _deterministic_verdict(detail: str) -> Tuple[str, str]:
    """
    Return (verdict, reason) where verdict in {"drop", "keep", "ambig"}.
    The reason is a short human-readable tag for logs.
    """
    t = _normalize(detail)
    if not t:
        return ("ambig", "empty detail")

    # 1. Hard drops — known-infeasible phrases and dimension-specific regex
    #    win over everything else. Even if the critique name-drops a real
    #    component alongside the bad phrase ("rocker-bogie wheels") the
    #    load-bearing ask is infeasible.
    for phrase in INFEASIBLE_PHRASES:
        if re.search(rf"\b{re.escape(phrase)}\b", t):
            return ("drop", f"infeasible phrase: {phrase!r}")
    for pattern, label in INFEASIBLE_PATTERNS:
        if re.search(pattern, t):
            return ("drop", label)

    # 2. Aesthetic/anatomical drops — gated by GATE_KEEP_PATTERNS. If the
    #    critique also names a concrete missing/wrong component or connection
    #    change, prefer KEEP (it's actionable even if wrapped in anatomical
    #    language). Mirrors the old TS regex's `actionableHints` guard.
    has_actionable_hint = any(re.search(p, t) for p in GATE_KEEP_PATTERNS)
    if not has_actionable_hint:
        for pattern, label in INFEASIBLE_AESTHETIC_PATTERNS:
            if re.search(pattern, t):
                return ("drop", label)

    # 3. Catalog matches — if the critique names a catalog concept directly
    #    it's actionable. Use word-boundary regex so "motor" doesn't fire on
    #    "motorway" etc.
    for kw in CATALOG_KEYWORDS:
        if re.search(rf"\b{re.escape(kw)}\b", t):
            return ("keep", f"catalog match: {kw!r}")

    # 4. Everything else — ambiguous proportions/shape language ("legs too
    #    thin", "torso is wrong proportion"). Hand off to LLM.
    return ("ambig", "no direct catalog match")


def _build_sonnet_prompt(items: List[Dict]) -> str:
    """
    Few-shot prompt for the Sonnet ambiguity judge. Each item is a dict with
    keys {index, check, detail}. Output: strict JSON array, one entry per
    index with {index, actionable, reason}.
    """
    examples = """Examples of critiques and the right call:

- "widen feet to 100mm for stability"            → DROP (no preset offers 100mm feet; current foot pad is 24mm)
- "add a rocker-bogie suspension"                → DROP (not in catalog)
- "baseplate should be 60-100mm thick"           → DROP (baseplate presets are fixed dimensions; can't be re-thicknessed)
- "missing a camera on the head"                 → KEEP (sensor_depth_camera_small exists)
- "use a wider baseplate"                        → KEEP (structural_baseplate_large is wider)
- "add a bracket between the stacked servos"     → KEEP (structural_bracket_l/u exist)
- "legs are mounted mirrored when they should all face the same way" → KEEP (topology change, achievable via attach_rpy)
- "torso is wrong proportion"                    → DROP (aesthetic, no preset dimension change satisfies)
- "the shoulder points up instead of down"       → KEEP (attach_rpy adjustment is actionable)

A critique is KEEP only when Claude can satisfy it with existing presets and topology moves (add/remove/swap/reorient a component). Vague aesthetic or dimension-specific asks that require parts we don't have are DROP. When in doubt, prefer DROP — a missed redesign opportunity is cheaper than a loop that burns Gemini quota and never converges."""

    item_lines = []
    for it in items:
        check = _normalize(it.get("check", ""))
        detail = (it.get("detail") or "").strip().replace("\n", " ")
        item_lines.append(f'{{"index": {it["index"]}, "check": "{check}", "detail": "{detail}"}}')
    items_block = "[\n  " + ",\n  ".join(item_lines) + "\n]"

    return f"""You are a feasibility judge for a robot-design AI's validation critiques. Decide, per critique, whether a redesign AI constrained to a fixed preset catalog could act on the critique.

{examples}

Classify each of the following critiques. Return a JSON array with exactly one object per input index, each shaped:
{{"index": <int>, "actionable": <true|false>, "reason": "<short reason>"}}

Critiques:
{items_block}

Return ONLY the JSON array. No prose, no markdown fences."""


def _llm_classify(items: List[Dict]) -> Dict[int, Tuple[bool, str]]:
    """
    Run the Sonnet batch classifier. Returns {index: (actionable, reason)}.
    On any failure returns an empty dict — caller defaults ambiguous items to
    actionable=True (fail-open, preserving current behavior).
    """
    if not items:
        return {}
    if _get_client is None or _parse_json_response is None:
        return {}
    if not os.environ.get("ANTHROPIC_API_KEY"):
        print("[critique_classifier] ANTHROPIC_API_KEY not set, skipping LLM stage", file=sys.stderr)
        return {}
    try:
        client = _get_client()
        prompt = _build_sonnet_prompt(items)
        response = client.messages.create(
            model="claude-sonnet-4-6",
            max_tokens=600,
            temperature=0,
            messages=[{"role": "user", "content": prompt}],
        )
        text = "".join(getattr(b, "text", "") for b in response.content) or ""
        parsed = _parse_json_response(text)
        if isinstance(parsed, dict):
            parsed = [parsed]
        if not isinstance(parsed, list):
            return {}
        out: Dict[int, Tuple[bool, str]] = {}
        for entry in parsed:
            if not isinstance(entry, dict):
                continue
            idx = entry.get("index")
            if not isinstance(idx, int):
                continue
            actionable = bool(entry.get("actionable", True))
            reason = str(entry.get("reason", ""))[:120]
            out[idx] = (actionable, reason)
        return out
    except Exception as e:
        print(f"[critique_classifier] LLM stage failed: {e}", file=sys.stderr)
        return {}


def classify_checklist(checklist: List[Dict], original_prompt: str = "") -> List[Dict]:
    """
    Enrich each failed checklist item with classifier_drop + classifier_reason.
    Passing items are untouched. Items already matching the hard-drop list or
    clear catalog match bypass the LLM; ambiguous items go through Sonnet in
    a single batched call.
    """
    if not checklist:
        return checklist

    enriched: List[Dict] = []
    ambiguous: List[Dict] = []
    for idx, item in enumerate(checklist):
        new_item = dict(item)
        if item.get("pass", True):
            enriched.append(new_item)
            continue
        verdict, reason = _deterministic_verdict(item.get("detail", ""))
        if verdict == "drop":
            new_item["classifier_drop"] = True
            new_item["classifier_reason"] = reason
        elif verdict == "keep":
            new_item["classifier_drop"] = False
            new_item["classifier_reason"] = reason
        else:
            ambiguous.append({"index": idx, "check": item.get("check", ""), "detail": item.get("detail", "")})
            # default fail-open; may be overwritten below by LLM
            new_item["classifier_drop"] = False
            new_item["classifier_reason"] = "ambiguous — pending LLM"
        enriched.append(new_item)

    if ambiguous:
        llm_verdicts = _llm_classify(ambiguous)
        for idx, (actionable, reason) in llm_verdicts.items():
            if 0 <= idx < len(enriched):
                enriched[idx]["classifier_drop"] = not actionable
                enriched[idx]["classifier_reason"] = f"llm: {reason}" if reason else "llm"

    # Log summary
    dropped = [i for i, c in enumerate(enriched) if c.get("classifier_drop")]
    if dropped:
        print(f"[critique_classifier] Dropped {len(dropped)} critique(s) as infeasible", file=sys.stderr)
        for i in dropped:
            print(f"[critique_classifier]   [{enriched[i].get('check')}] ({enriched[i].get('classifier_reason')}): {enriched[i].get('detail','')[:140]}", file=sys.stderr)
    return enriched


# ── Self-test fixtures (run with: python -m core.ai.critique_classifier) ─────
if __name__ == "__main__":  # pragma: no cover
    fixtures = [
        {"check": "proportions", "pass": False, "detail": "add a rocker-bogie suspension to the chassis",
         "expect_drop": True},
        {"check": "proportions", "pass": False, "detail": "use a wider baseplate preset",
         "expect_drop": False},
        {"check": "shape_match", "pass": False, "detail": "widen feet to 100mm for stability",
         "expect_drop": True},
        # Regression: real critique from 2026-04-20 quadruped session — singular
        # "foot" + "should be at least ~Nmm" was leaking through as KEEP and
        # driving a redesign that swapped foot pads for L-brackets.
        {"check": "proportions", "pass": False,
         "detail": "the mobility_rubber_foot_pad is undersized; the foot should be at least ~40mm to match the leg profile",
         "expect_drop": True},
        {"check": "completeness", "pass": False, "detail": "missing a camera on the head",
         "expect_drop": False},
        # Regression: bracket-floating-camera critique from same session was
        # correctly KEPT (actionable: swap bracket for extrusion). Lock that.
        {"check": "completeness", "pass": False,
         "detail": "the camera mount structural_bracket_l_6 is floating ~20mm in front of the chassis; the bracket is too small to bridge the gap, requiring a structural_extrusion or longer mount",
         "expect_drop": False},
        # Aesthetic-pattern regression coverage (from old TS regex, commits
        # df74d83 / 5f82e8a). These must drop even without the Sonnet stage.
        {"check": "proportions", "pass": False,
         "detail": "the chassis looks too boxy for a quadruped, should be more mammal-like",
         "expect_drop": True},
        {"check": "shape_match", "pass": False,
         "detail": "the rear knees point forward instead of backward like a dog",
         "expect_drop": True},
        {"check": "shape_match", "pass": False,
         "detail": "front legs should be mirrored pitch vs rear legs",
         "expect_drop": True},
        # GATE_KEEP bypass: aesthetic language wrapped around an actionable ask.
        {"check": "completeness", "pass": False,
         "detail": "the legs are mirrored but missing a servo on the front-left hip",
         "expect_drop": False},
        {"check": "shape_match", "pass": True, "detail": "ok"},
    ]
    enriched = classify_checklist(fixtures, "mars rover with pan-tilt mast")
    ok = True
    for inp, out in zip(fixtures, enriched):
        if not inp.get("pass", True):
            actual = bool(out.get("classifier_drop"))
            expected = bool(inp.get("expect_drop"))
            mark = "PASS" if actual == expected else "FAIL"
            if actual != expected:
                ok = False
            print(f"  [{mark}] detail={inp['detail']!r} expected_drop={expected} got_drop={actual} reason={out.get('classifier_reason')}")
    sys.exit(0 if ok else 1)
