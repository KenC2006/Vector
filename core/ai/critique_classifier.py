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


# Magnitude parsing — used by the Layer-2 measurement-feedback logic
#. Validator details frequently include
# numeric distance claims like "floating ~45mm", "shin shifted 38mm", "camera
# detached by 1.5cm". We pull the LARGEST such magnitude in the text (in mm)
# so we can compare against the engine's ICP gap for the same joint.
_MAGNITUDE_PATTERN = re.compile(
    r"(~\s*)?(\d+(?:\.\d+)?)\s*(mm|cm|m)\b",
    re.IGNORECASE,
)


def _extract_largest_magnitude_mm(detail: str) -> Optional[float]:
    """Return the largest dimensional magnitude in `detail` in mm, or None.

    Matches mm, cm, and m. Ignores percents and unitless numbers. Used as a
    cheap stand-in for "is the validator claiming a large placement error?"
    """
    if not detail:
        return None
    max_mm: Optional[float] = None
    for m in _MAGNITUDE_PATTERN.finditer(detail):
        try:
            value = float(m.group(2))
        except (TypeError, ValueError):
            continue
        unit = m.group(3).lower()
        if unit == "cm":
            value_mm = value * 10.0
        elif unit == "m":
            value_mm = value * 1000.0
        else:
            value_mm = value
        if max_mm is None or value_mm > max_mm:
            max_mm = value_mm
    return max_mm


def _find_icp_entry_for_critique(detail: str, icp_gaps: List[Dict]) -> Optional[Dict]:
    """Return the first ICP entry whose `linkName` appears in `detail`.

    Link names are unique per build (e.g. `sensor_depth_camera_small_5`,
    `shin_extrusion_rl`), so substring search is safe. Matching is longest-
    first so that `structural_baseplate_large_1` wins over `structural_baseplate_1`
    when both exist.
    """
    if not detail or not icp_gaps:
        return None
    sorted_entries = sorted(
        (e for e in icp_gaps if isinstance(e, dict) and e.get("linkName")),
        key=lambda e: len(e.get("linkName", "")),
        reverse=True,
    )
    for entry in sorted_entries:
        name = entry.get("linkName")
        if name and name in detail:
            return entry
    return None


# Thresholds for the Layer-2 reclassification logic. Kept as module-level
# constants so they're easy to tune and reference in fixtures / docs.
#
# - CRITIQUE_MAGNITUDE_MIN_MM: critiques below this magnitude don't trigger
# reclassification. 10mm is the cap calls out as the
# typical adaptive-confident-cap bound — real placement residuals should be
# below 10mm, so a >10mm complaint is suspicious.
# - ICP_FLUSH_MAX_MM: below this gap, an ICP entry is "flush" and a large
# magnitude complaint on the same joint is almost certainly a visual misread.
CRITIQUE_MAGNITUDE_MIN_MM = 10.0
ICP_FLUSH_MAX_MM = 2.0


def _deterministic_verdict(detail: str) -> Tuple[str, str]:
    """
    Return (verdict, reason) where verdict in {"drop", "keep", "ambig"}.
    The reason is a short human-readable tag for logs.
    """
    t = _normalize(detail)
    if not t:
        return ("ambig", "empty detail")

    # 1. Hard drops — known-infeasible phrases and dimension-specific regex
    # win over everything else. Even if the critique name-drops a real
    # component alongside the bad phrase ("rocker-bogie wheels") the
    # load-bearing ask is infeasible.
    for phrase in INFEASIBLE_PHRASES:
        if re.search(rf"\b{re.escape(phrase)}\b", t):
            return ("drop", f"infeasible phrase: {phrase!r}")
    for pattern, label in INFEASIBLE_PATTERNS:
        if re.search(pattern, t):
            return ("drop", label)

    # 2. Aesthetic/anatomical drops — gated by GATE_KEEP_PATTERNS. If the
    # critique also names a concrete missing/wrong component or connection
    # change, prefer KEEP (it's actionable even if wrapped in anatomical
    # language). Mirrors the old TS regex's `actionableHints` guard.
    has_actionable_hint = any(re.search(p, t) for p in GATE_KEEP_PATTERNS)
    if not has_actionable_hint:
        for pattern, label in INFEASIBLE_AESTHETIC_PATTERNS:
            if re.search(pattern, t):
                return ("drop", label)

    # 3. Catalog matches — if the critique names a catalog concept directly
    # it's actionable. Use word-boundary regex so "motor" doesn't fire on
    # "motorway" etc.
    for kw in CATALOG_KEYWORDS:
        if re.search(rf"\b{re.escape(kw)}\b", t):
            return ("keep", f"catalog match: {kw!r}")

    # 4. Everything else — ambiguous proportions/shape language ("legs too
    # thin", "torso is wrong proportion"). Hand off to LLM.
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


def _apply_measurement_feedback(item: Dict, icp_gaps: List[Dict]) -> None:
    """
    In-place Layer-2 reclassification.

    Two transformations on placement-fixable critiques whose detail contains
    a dimensional claim > CRITIQUE_MAGNITUDE_MIN_MM:

      1. If the named joint has a high-confidence ICP gap < ICP_FLUSH_MAX_MM,
         the part IS flush — the validator misread the screenshot. Drop the
         critique (classifier_drop=True, reason mentions the ICP refutation).

      2. If NO ICP entry names the joint, the placement engine cannot close the
         reported gap (ICP caps out at ~3mm, adaptive-cap ~15mm). A >10mm
         complaint with no ICP presence is a topology-consequence claim
         (rotation up the ancestor chain, wrong component, etc.), so flip
         `fixable_by` from placement → topology so the AI gets a redesign
         attempt instead of hitting `Skipping redesign — placement-fixable and
         retry cap reached`.

    Non-placement critiques and magnitudes ≤ 10mm are left alone.
    """
    if item.get("pass", True):
        return
    if item.get("classifier_drop"):
        return  # already dropped by upstream deterministic/LLM stage — respect it
    if item.get("fixable_by") != "placement":
        return
    detail = item.get("detail", "")
    magnitude = _extract_largest_magnitude_mm(detail)
    if magnitude is None or magnitude <= CRITIQUE_MAGNITUDE_MIN_MM:
        return

    entry = _find_icp_entry_for_critique(detail, icp_gaps) if icp_gaps else None
    if entry is not None:
        gap = entry.get("gapP50Mm")
        confidence = entry.get("confidence")
        # Gap is "flush" when absolute p50 < 2mm AND the engine is confident.
        # `abs` catches the negative (overlap) case — those are ALSO flush,
        # not floating.
        if (
            confidence == "high"
            and isinstance(gap, (int, float))
            and abs(float(gap)) < ICP_FLUSH_MAX_MM
        ):
            item["classifier_drop"] = True
            item["classifier_reason"] = (
                f"measurement_feedback: critique claims ~{magnitude:.0f}mm offset on "
                f"`{entry.get('linkName')}` but ICP reports gap_p50={float(gap):.2f}mm at high "
                f"confidence (paired={entry.get('pairedCount')}/{entry.get('sampleCount')}). "
                f"Validator misread — parts are flush."
            )
            return
        # Matched an ICP entry, but either low-confidence or gap is genuinely
        # large: leave the critique alone. The engine's own ICP telemetry
        # confirms there's something to look at.
        return

    # No ICP entry for this joint (e.g. pair lives on the Step-1 authored
    # connector path, or link name is absent from the detail). A >10mm
    # placement-fixable complaint cannot be resolved by the placement engine
    # (ICP nudge caps at ~15mm via confident-cap, 3mm default). Escalate to
    # topology so the AI gets a redesign rather than hitting the
    # placement-fixable retry cap.
    item["fixable_by"] = "topology"
    item["classifier_reason"] = (
        f"measurement_feedback: ~{magnitude:.0f}mm placement claim exceeds ICP's "
        f"placement-reach budget with no matching ICP entry; escalated to topology "
        f"so the AI can attempt a structural fix."
    )


def classify_checklist(checklist: List[Dict], original_prompt: str = "",
                       engine_summary: Optional[Dict] = None) -> List[Dict]:
    """
    Enrich each failed checklist item with classifier_drop + classifier_reason.
    Passing items are untouched. Items already matching the hard-drop list or
    clear catalog match bypass the LLM; ambiguous items go through Sonnet in
    a single batched call.

    engine_summary (optional): Layer-2 feedback input of the shape
    {"placements": [...], "icpGaps": [...]} as produced by
    urdfAssembly.ts. When present, placement-fixable critiques with large mm
    magnitudes are cross-checked against the ICP table — a critique that
    contradicts a high-confidence flush gap is dropped; one with no matching
    ICP entry is escalated to topology-fixable so the AI can attempt a
    structural redesign instead of hitting the placement-retry cap.
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

    # Layer-2 measurement feedback — applied AFTER the deterministic/LLM stage
    # so catalog-infeasible drops still win over measurement reclassification.
    icp_gaps: List[Dict] = []
    if engine_summary and isinstance(engine_summary, dict):
        raw_gaps = engine_summary.get("icpGaps")
        if isinstance(raw_gaps, list):
            icp_gaps = raw_gaps
    for item in enriched:
        _apply_measurement_feedback(item, icp_gaps)

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
        # df74d83 / 5f82e8a). Vague body/proportion asks still drop, but
        # concrete knee/mirror critiques are actionable through attach_rpy.
        {"check": "proportions", "pass": False,
         "detail": "the chassis looks too boxy for a quadruped, should be more mammal-like",
         "expect_drop": True},
        {"check": "shape_match", "pass": False,
         "detail": "the rear knees point forward instead of backward like a dog",
         "expect_drop": False},
        {"check": "shape_match", "pass": False,
         "detail": "front legs should be mirrored pitch vs rear legs",
         "expect_drop": False},
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

    # ── Layer 2 measurement-feedback fixtures (require engine_summary) ────────
    # Scenario: image-24 quadruped from 2026-04-22. Validator hallucinates
    # "camera floating 45mm" and "shin detached 38mm" on a build where ICP
    # reports flush high-confidence gaps for those joints.
    layer2_icp = [
        {"linkName": "sensor_depth_camera_small_5", "parentConnector": "structural_baseplate_large.front",
         "childConnector": "sensor_depth_camera_small.mount_back",
         "pairedCount": 80, "sampleCount": 80, "gapP50Mm": -1.72, "gapP90Mm": -0.74,
         "gapMinMm": -2.0, "gapMaxMm": -0.5, "nudgeMm": 0.0, "reason": "flush", "confidence": "high"},
        {"linkName": "shin_extrusion_rl", "parentConnector": "actuator_servo_high_torque.bottom",
         "childConnector": "structural_extrusion.top",
         "pairedCount": 78, "sampleCount": 80, "gapP50Mm": 0.80, "gapP90Mm": 1.10,
         "gapMinMm": 0.4, "gapMaxMm": 1.3, "nudgeMm": 0.0, "reason": "flush", "confidence": "high"},
    ]
    layer2_summary = {"placements": [], "icpGaps": layer2_icp}
    layer2_fixtures = [
        # Should DROP — validator claim > 10mm, but ICP says flush high-confidence.
        {"check": "grounded", "pass": False, "fixable_by": "placement",
         "detail": "sensor_depth_camera_small_5 is floating ~45mm past the +X edge of the baseplate",
         "expect_drop": True, "expect_fixable_by": "placement"},
        {"check": "overlap", "pass": False, "fixable_by": "placement",
         "detail": "shin_extrusion_rl shifted 38mm along X-axis from where it should sit",
         "expect_drop": True, "expect_fixable_by": "placement"},
        # Should NOT drop — magnitude present but joint has no ICP entry.
        # Should ESCALATE to topology-fixable so AI gets a redesign.
        {"check": "direction", "pass": False, "fixable_by": "placement",
         "detail": "thigh_extrusion_fl is angled 30mm off-axis relative to its hip joint",
         "expect_drop": False, "expect_fixable_by": "topology"},
        # Should NOT drop, NOT escalate — magnitude below threshold.
        {"check": "overlap", "pass": False, "fixable_by": "placement",
         "detail": "shin_extrusion_rl clips into its parent by ~3mm",
         "expect_drop": False, "expect_fixable_by": "placement"},
        # Should NOT drop, NOT escalate — topology-fixable already.
        {"check": "completeness", "pass": False, "fixable_by": "topology",
         "detail": "missing a gripper at the end of forearm_extrusion_1 (gap ~150mm)",
         "expect_drop": False, "expect_fixable_by": "topology"},
        # Should NOT drop — ICP entry exists but confidence is low, so we
        # can't refute the screenshot claim.
        {"check": "grounded", "pass": False, "fixable_by": "placement",
         "detail": "imu_board_1 is floating 20mm above the baseplate",
         "expect_drop": False, "expect_fixable_by": "placement"},
    ]
    # Seed the low-confidence case.
    layer2_icp.append({
        "linkName": "imu_board_1", "parentConnector": "structural_baseplate_large.top",
        "childConnector": "imu_board.bottom",
        "pairedCount": 6, "sampleCount": 80, "gapP50Mm": -0.1, "gapP90Mm": 2.0,
        "gapMinMm": -1.0, "gapMaxMm": 8.0, "nudgeMm": 0.0, "reason": "sparse", "confidence": "low",
    })
    layer2_enriched = classify_checklist(layer2_fixtures, "quadruped smoke test", layer2_summary)
    for inp, out in zip(layer2_fixtures, layer2_enriched):
        actual_drop = bool(out.get("classifier_drop"))
        expected_drop = bool(inp.get("expect_drop"))
        actual_fixable = out.get("fixable_by")
        expected_fixable = inp.get("expect_fixable_by")
        pass_ok = actual_drop == expected_drop and actual_fixable == expected_fixable
        mark = "PASS" if pass_ok else "FAIL"
        if not pass_ok:
            ok = False
        print(f"  [{mark}] (layer2) drop={actual_drop}(exp={expected_drop}) "
              f"fixable_by={actual_fixable}(exp={expected_fixable}) reason={out.get('classifier_reason')}")

    # Magnitude parsing direct unit coverage
    magnitude_cases = [
        ("the camera floats ~45mm from the chassis", 45.0),
        ("shin offset 1.5cm along X", 15.0),
        ("foot sunk 0.05m into ground", 50.0),
        ("biggest offset is 120 mm, smaller ~5mm", 120.0),
        ("no numeric claim here", None),
    ]
    for text, expected in magnitude_cases:
        actual = _extract_largest_magnitude_mm(text)
        pass_ok = (actual == expected) or (
            expected is not None and actual is not None and abs(actual - expected) < 1e-6
        )
        mark = "PASS" if pass_ok else "FAIL"
        if not pass_ok:
            ok = False
        print(f"  [{mark}] (magnitude) text={text!r} expected={expected} got={actual}")

    sys.exit(0 if ok else 1)
