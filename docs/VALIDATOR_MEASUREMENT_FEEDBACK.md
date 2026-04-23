# Validator Measurement Feedback Plan

**Status:** **Superseded — Layers 1, 2, and 3a all shipped 2026-04-22 in commit `4f45289`.** See [`VALIDATOR_MEASUREMENT_FEEDBACK_LANDED.md`](./VALIDATOR_MEASUREMENT_FEEDBACK_LANDED.md) for what actually landed (including deviations from this plan — the validator prompt was inline in `claude_client.py`, not a separate file; Layer 2 was not deferred). Layer 3b (canonical pose references) remains open per original plan.

This doc is preserved as the original problem statement and design rationale.

---

**Original status:** Open. Triggered by 2026-04-22 quadruped smoke test (image 24) where the AI produced a crossed-leg "trash" assembly after a redesign cycle. Engine logs were clean; the failure was entirely in the AI/validator interaction.

## What happened (image 24)

User generated a quadruped. The placement engine and ICP did exactly what they should. The AI/validator loop produced an unusable output anyway. Three separable causes; none is an engine bug.

### Pass 1 (initial design)
- Quadruped with all four hip_pitch=`+0.52`, all four knees=`-1.05` — same direction on every leg (canonical pose).
- Engine placed everything correctly. ICP fired 33 nudges, total 121.87mm. Camera placed via `parent=front, child=mount_back, fastened` at `xyz=(0.220, 0, -0.030)` (with elevation_angle=-10° applied).
- Validator hallucinated: *"sensor_depth_camera_small_5 is floating in mid-air ~47mm past the +X edge."*  ICP for that joint reported `paired=80/80, p50=-1.72mm, p90=-0.74mm` — flush, sub-mm overlap.
- Validator also said: *"All legs pitched ~30° toward -X; should be mirrored in pairs (front vs back)."*  This is geometry advice that's wrong for quadrupeds — Spot/Go1 keep all four hip_pitch the same direction.
- Verdict: `needs_redesign=true` → AI redesigns.

### Pass 2 (AI's "fix")
- AI obeyed literally: rear hip_pitch=`-0.52`, rear knee=`+1.05` (signs flipped).
- Result: front legs lean back, rear legs lean forward — they cross in the middle. Image 24's X-shape.
- Validator now says: *"shin shifted 38mm along X-axis, foot floating 75mm above ground, camera floating 45mm."*  All three are visual misreads of the rotated chain. Engine placement xyz is identical to the working FL/FR/RL legs (`xyz=0.0000 0.0000 -0.0770`).
- Validator tags all 3 failures `fixable_by: placement`.
- Loop: `Skipping redesign — all 3 failure(s) are placement-fixable and retry cap reached.` AI never gets a third chance.

### Why "the old system never made it this bad"
Earlier runs either skipped the redesign cycle entirely, or the validator's regex-based classifier dropped Gemini's geometry suggestions before they reached the AI. The new classifier (workstream/gemini-loop-hardening) is more permissive — it lets specific suggestions through. So the AI is now *more* responsive to validator advice, which is good when the advice is right and bad when it isn't.

## What the validator currently gets

`viewportChat.ts:1131` calls `ai_validate_assembly` with:
- `urdfContent` (URDF XML, ~67k chars on this run)
- `originalPrompt` (user's text)
- `screenshots` (3 angles, 768×768, base64 PNG)
- `referenceImages` (any images the user attached)

That's it. **No engine-computed ground truth.** Every distance/gap claim is from pixel-counting on the screenshots.

## Three-layer fix

### Layer 1: feed engine telemetry to the validator (~1 hr, biggest leverage)

Bundle the data the engine already computed and pass it alongside the screenshots. Two structured tables:

**ICP gap table** — already logged at `urdfAssembly.ts:1809`:
```
joint                                        | parent_face   | child_face       | gap_p50  | gap_p90  | nudge   | confidence
sensor_depth_camera_small_5                  | front         | mount_back       | -1.72mm  | -0.74mm  | 0.00mm  | high (paired=80/80)
power_lipo_4s_5000_2                         | top           | bottom           | -2.65mm  | 12.52mm  | 3.00mm  | low (paired=7/80)
actuator_servo_high_torque_7                 | bottom (disc) | top (servo)      |  3.34mm  |  3.36mm  | 3.36mm  | high (confident-cap)
...
```

**Placement table** — from `[assembly] ✓ Placed ...` lines:
```
link                            | parent                          | xyz (m)                 | rpy (rad)
shin_extrusion_rl               | actuator_servo_high_torque_30   | 0.0000 0.0000 -0.0770   | 0 0 0
shin_extrusion_rr               | actuator_servo_high_torque_39   | 0.0000 0.0000 -0.0770   | 0 0 0
```

Add to the validator system prompt:
> The tables below are ground-truth measurements from the placement engine. Each row is the actual gap (in mm) between two contact faces and the actual XYZ placement applied. **Do not contradict these numbers from screenshot inspection alone.** If the screenshot looks like a part is floating but the engine reports a gap < 2mm, the issue is mesh visual offset or proportions, not placement. If two siblings have identical XYZ but the screenshot shows one displaced, the displacement comes from rotation accumulated up the chain (attach_rpy on an ancestor) — call out the ancestor, don't claim the child is "detached."

This kills the entire "floating camera" / "detached shin" hallucination class. The engine *knows* whether parts are touching; we just weren't telling Gemini.

**Files:**
- `src/src/urdfAssembly.ts` — collect the per-joint ICP results into a serializable summary, return alongside the URDF
- `src/src/viewportChat.ts` — thread the summary into the `ai_validate_assembly` call
- `src-tauri/src/lib.rs` `ai_validate_assembly` IPC handler — accept new parameter
- `core/server.py` validator endpoint — forward to Gemini prompt
- `core/ai/validator_prompt.py` (or wherever) — add the ground-truth section

### Layer 2: escalate large-distance placement complaints to topology (~30 min)

The classifier currently treats every `fixable_by: placement` failure as something the placement engine should resolve. But ICP caps nudges at 3mm (default) or 15mm (confident-cap). A "shin offset 38mm" complaint is impossible to fix at the placement layer — it's a topology consequence (rotation chain) or a validator misread.

Add to `core/ai/critique_classifier.py`:
- Parse magnitude from the critique detail (`/(\d+)\s*mm/` patterns).
- If `placement-fixable` AND magnitude > 10mm AND the affected joint's ICP entry is high-confidence and < 2mm → reclassify as `validator_misread` (drop the critique, don't trigger redesign).
- If `placement-fixable` AND magnitude > 10mm AND no high-confidence ICP entry exists → reclassify as `topology-fixable` (let AI redesign).

This stops the `Skipping redesign — placement-fixable and retry cap reached` dead-end on cases where the engine can't actually fix it.

**Files:**
- `core/ai/critique_classifier.py` — magnitude parsing + reclassification logic
- Test fixture in `core/tests/test_critique_classifier.py`

### Layer 3: stop Gemini from prescribing specific geometry (prompt change, ~15 min)

The "mirror legs in pairs" suggestion was confidently wrong. Real quadrupeds (Spot, Go1, ANYmal) keep all four hip_pitch the *same* sign. Mirroring inverts to a crossed-leg stance.

Two options:

**(a) Ban specific geometry prescriptions** — change the validator prompt:
> When a stance/orientation looks wrong, describe *what* is wrong (e.g., "legs collide in the centerline") but DO NOT prescribe specific angle changes, sign flips, or "mirror in pairs" instructions. The downstream AI will choose the corrective change. Specific prescriptions risk steering toward a worse pose.

**(b) Pass canonical reference poses** — for each robot class (quadruped, biped, arm), include a one-line description of the canonical stance:
> Quadruped reference pose: all four hip_pitch servos rotate the same direction (legs all lean back into a "praying mantis" stance, knees fold forward). Mirroring front vs back creates a crossed-leg stance.

(b) is more work but more durable. (a) is a one-line prompt edit.

**Files:** wherever the validator system prompt lives (likely `core/ai/validator_prompt.py` or inline in `core/server.py`).

## Recommended sequence

1. **Layer 1 first** — biggest win, smallest blast radius, data already in logs. Ships in one session.
2. **Layer 3 (a)** — one-line prompt change, prevents future bad-prescription incidents. Ships in same session.
3. **Layer 2** — defer until Layer 1 is in production. With Layer 1, most "shin detached" hallucinations should already be filtered out by the validator using the gap table; Layer 2 becomes a backstop for cases the validator misses.
4. **Layer 3 (b)** — pose reference table — separate session, do it when adding new robot classes (biped, arm) that need their own canonical poses anyway.

## What this plan does NOT do

- **Doesn't fix the AI camera-bracket regression** (`docs/AI_CAMERA_BRACKET_REGRESSION.md`) — that's a system-prompt fix on the *design* AI, separate from the validator.
- **Doesn't address Gemini's catalog-impossible suggestions** (G2 in `project_known_issues.md`) — already partially handled by the classifier hardening on `workstream/gemini-loop-hardening`.
- **Doesn't add new validator capabilities** (e.g. tipping analysis, COG checks) — out of scope; this is purely about feeding the validator the ground truth it's currently missing.

## Test cases that would catch regression

- Generate a quadruped. Verify validator does NOT say "camera floating" when ICP gap < 2mm.
- Generate a quadruped, force the AI to produce mirrored legs (via prompt). Verify validator describes "legs cross in centerline" rather than "shin detached 38mm."
- Generate a robot where the validator's previous rule of thumb would have over-fired. Verify the new ground-truth-aware prompt produces accurate critique.

## Files involved (summary)

| File | Layer | Change |
|---|---|---|
| `src/src/urdfAssembly.ts` | 1 | Collect ICP summary into return payload |
| `src/src/viewportChat.ts` | 1 | Thread summary to validator call |
| `src-tauri/src/lib.rs` | 1 | IPC param pass-through |
| `core/server.py` | 1 | Validator endpoint forwards summary |
| `core/ai/validator_prompt.py` (or inline) | 1, 3 | Ground-truth section + ban geometry prescriptions |
| `core/ai/critique_classifier.py` | 2 | Magnitude parsing + reclassification |
| `core/tests/test_critique_classifier.py` | 2 | Fixtures |

## Related docs

- `docs/AI_CAMERA_BRACKET_REGRESSION.md` — separate AI-side regression (design AI, not validator)
- `docs/ICP_RESIDUALS.md` — what ICP can and can't fix; informs Layer 2's threshold choice
- `docs/ENGINE_NEXT_STEPS.md` — engine plan now complete; this doc is post-engine work
