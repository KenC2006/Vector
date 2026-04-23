# Validator Measurement Feedback — Live Test Results (2026-04-22)

First live quadruped test after shipping Layers 1, 2, and 3a. Branch: `engine-gaps-integration`.

## Test setup

- Prompt: standard quadruped generation (same canonical Go1-style chain that produced image-24's crossed-leg trash on 2026-04-22).
- Build: 42 components placed, 33 ICP nudges (total 121.87mm), 4 reconcile shifts for foot pads.
- Validator: Gemini 3 Flash, 3 × 768px screenshots + URDF + new engine summary block.

## What landed on the wire

Frontend console confirmed the engine summary reached the validator:

```
[AI] Assembly result: urdf=68387 chars, topologyErrors=[], engineSummary=41 placements / 41 ICP gaps
[AI] Running 2nd-pass AI validation with visual feedback...
```

41 placements + 41 ICP gaps = one row per non-root component, matching the 42-link build.

## Pass 1 validator response

Gemini returned `ok=false, needs_redesign=true`. Six checklist items:

| check | pass | fixable_by | classifier_drop | notes |
|---|---|---|---|---|
| shape_match | ✓ | topology | — | "quadruped with four legs attached to a central chassis" |
| direction | ✗ | placement | false | "legs angled sharply inward due to -1.05 rad pitch, unstable crouched stance" |
| overlap | ✗ | placement | false | "camera clipping into front edge by ~2mm; ICP table shows 3-5mm gaps in nearly all leg joints" |
| completeness | ✓ | topology | — | "all requested features present" |
| proportions | ✗ | topology | **true** | "baseplate 350mm long but only 8mm thick... dachshund profile" — dropped by deterministic classifier as aesthetic/anatomical |
| grounded | ✓ | placement | — | "all four foot pads correctly on ground plane" |

**Key observation — Gemini is reading the ICP tables directly.** The `overlap` critique literally says *"the ICP table shows significant gaps of 3-5mm in nearly all leg joints (e.g., mobility_rubber_foot_pad_15 to structural_extrusion_2020_14)"*. Layer 1 is demonstrably working: the tables reach Gemini and it parses them.

## Hallucination class killed

The image-24 failure mode was "camera floating 45mm" / "shin detached 38mm" — fabricated magnitudes on joints that ICP reported as flush. On this run:

- Camera (`sensor_depth_camera_small_6`): ICP `gap_p50=-1.72mm, p90=-0.74mm, paired=80/80, confidence=high`. Gemini said *"clipping by ~2mm"* — that matches reality within rounding. **No fabricated magnitude.**
- Shins / legs: ICP reports `p90=3-5mm` on several mates (confident-cap on low-paired-ratio small faces — documented residual in `docs/ICP_RESIDUALS.md`). Gemini echoed *"3-5mm gaps in nearly all leg joints"* — also a faithful reading of the table.

Every numeric claim in Pass 1 matched or came close to an engine row. Zero hallucinated >10mm floating/detached claims.

## Layer 2 behavior on this run

No >10mm placement-fixable critiques fired. Consequence: Layer 2's flush-refute and topology-escalate branches did not exercise. **This is the intended healthy-build outcome** — Layer 1 suppresses the kind of critique that would trigger Layer 2. The fixture suite (`python -m core.ai.critique_classifier`) proves the paths work when the >10mm case shows up; we just don't need them on a build where Gemini's magnitudes are grounded.

Both Pass 1 placement-fixable items (`direction` at "sharply inward" and `overlap` at ~2mm) came back from the LLM stage as `classifier_drop: false` with actionable reasons — appropriate, since the LLM judge decided the AI *could* act (adjust knee `attach_rpy`, adjust camera `attach_xyz`).

The `proportions` dachshund critique was correctly dropped by the deterministic pre-filter as `aesthetic/anatomical critique (no preset action available)` — this is existing behavior, not Layer 2.

## Layer 3a behavior

**No specific-geometry prescriptions.** Pass 1 described symptoms ("sharply inward", "unstable crouched stance", "elongated dachshund profile") without prescribing "mirror in pairs", "rotate by N°", or "flip sign". Pass 2's AI redesign reflected a *judgment* call (reduce the magnitudes: hip_pitch 0.52→0.35, knee -1.05→-0.52) rather than a literal-interpretation spiral.

This is the fix for the image-24 root cause. On 2026-04-22's image-24, Gemini's "mirror in pairs" prescription was taken literally and produced the crossed-leg stance. Here, Gemini gave no such prescription and the redesign stayed kinematically sane.

## Pass 2 result

Redesign ran. AI adjusted:
- All 4 `hip_pitch` servos: `attach_rpy = [0, 0.35, 0]` (was 0.52)
- All 4 `knee` servos: `attach_rpy = [0, -0.52, 0]` (was -1.05)
- Camera: added explicit `mate_connector: "mount_back"` — camera now sits at `xyz=0.2200 0.0000 0.0000` (elevation removed from xyz, angles cleaner)

Gemini 429 RESOURCE_EXHAUSTED'd on validation (free-tier quota). Fallback path returned `ok=true`, build shipped.

Visual inspection of the captured screenshot: upright quadruped, four legs straight, foot pads on the ground, chassis visible with components on top. **Not the image-24 crossed-leg trash.** Stance is a bit stiff/upright (slight over-correction on the angles) but topologically sound and recognizable as a quadruped.

## Residual observations

1. **Sub-2mm interpenetration still generates complaints.** The camera's `-1.72mm` p50 gap is *intentional flush contact* (ICP treats negative-and-tiny as "touching"), but Gemini still flagged it as `~2mm clipping`. Layer 2's 10mm floor means the classifier doesn't suppress it. Options if this bothers us later:
   - (a) Tighten the prompt to explicitly say "sub-2mm interpenetration IS the flush contact state, do not flag as overlap".
   - (b) Add a separate Layer-2 branch: for `overlap` checks specifically, drop critiques when magnitude ≤ 2mm AND a high-confidence ICP entry shows |gap| ≤ 2mm.
   - Not urgent: the critique was actionable enough that the LLM stage kept it, and the redesign responded reasonably.

2. **Free-tier 429 on Pass 2.** Expected; ship-mode concern only. The prompt additions (two new sections + tables) grew per-request tokens — may want to watch the per-day call count on heavy usage.

3. **Confident-cap residuals still appear.** Mates like extrusion-on-servo show `p90=4-5mm` with 8-20% paired ratio — documented as known-acceptable in `docs/ICP_RESIDUALS.md` residual #2 / #3. Gemini reads these faithfully from the table but flags them as a problem. Could be suppressed by tagging these low-paired-coverage entries as `confidence=low-coverage` and adding prompt guidance to treat them as sample artifacts, not real gaps. Not blocking.

## Verdict

Ship-ready. The workstream's specific goal — kill the "camera floating Nmm / shin detached Nmm" hallucinations that produced crossed-leg quadruped trash on 2026-04-22 — is achieved. Every magnitude Gemini cited on Pass 1 traced to an actual ICP row. Pass 2 produced a topologically reasonable quadruped. No new failure modes uncovered.
