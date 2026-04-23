# Validator Measurement Feedback — Landed (2026-04-22)

Follow-up to `docs/VALIDATOR_MEASUREMENT_FEEDBACK.md`. Layers 1, 2, and 3a all shipped in one session on branch `engine-gaps-integration`.

## What landed

### Layer 1 — engine ground-truth to validator

- `src/src/urdfAssembly.ts` — new exported types `EnginePlacementEntry`, `EngineIcpEntry`, `EngineSummary`. `runContactCleanupPass` now returns `icpEntries` alongside `shifts`; `resolveAssemblyGraph` accumulates `placementEntries` inline during the placement loop (at the existing `[assembly] ✓ Placed` emission site) and returns a fully populated `engineSummary` field. Confidence heuristic: `high` when paired-sample ratio ≥ 75% OR the adaptive confident-cap fired (checked via `/confident[- ]?cap/i` on the ICP diagnostic reason); otherwise `low`. Also emits a placeholder entry for the `shouldApplyRuntimeNudge` skip branch so Step-1-authored connectors show up in the table as `authored-engagement-depth` with high confidence.
- `src/src/viewportChat.ts` — captures `assemblyOut.engineSummary` and threads it as a new `engineSummary` parameter on the `ai_validate_assembly` invocation. Logs table sizes so a live trace confirms delivery.
- `src-tauri/src/lib.rs` — added `engine_summary: Option<serde_json::Value>` to the `ai_validate_assembly` Tauri command and forwards it as `engine_summary` in the RPC body.
- `core/server.py` — reads `engine_summary` from RPC params, passes it to `_validate_assembly`.
- `core/ai/claude_client.py`:
  - `validate_assembly()` grew an `engine_summary: dict = None` kwarg and threads it into both the Gemini call and the classifier.
  - New `_format_engine_summary_block()` renders the payload as two markdown tables (ICP gap + placement) with hard caps at 200 rows each.
  - `_validate_assembly_gemini()` injects the block between the URDF and the view-instruction in the prompt, and logs the table sizes.
  - `VALIDATION_SYSTEM_PROMPT` grew two new sections above the checklist:
    - **"Engine ground truth — do not contradict"** — explicit instruction that a high-confidence ICP gap < 2mm refutes any screenshot-based "floating N mm" claim, and that sibling displacement claims should name the ancestor's `attach_rpy`.
    - **"Describe symptoms, do NOT prescribe geometry"** (Layer 3a, below).

### Layer 2 — magnitude reclassification in the critique classifier

- `core/ai/critique_classifier.py`:
  - Added `_extract_largest_magnitude_mm()` — regex over mm / cm / m, takes the largest magnitude in a critique detail.
  - Added `_find_icp_entry_for_critique()` — substring match of critique detail against ICP entries, longest-link-name first so that `structural_baseplate_large_1` wins over `structural_baseplate_1`.
  - Added `_apply_measurement_feedback()` — post-hoc pass over the classifier output. For `fixable_by == "placement"` critiques with magnitude > 10mm:
    - If a high-confidence ICP entry exists and `|gap_p50| < 2mm` → drop the critique as `validator_misread` with an explanatory reason.
    - If no ICP entry matches → flip `fixable_by` to `"topology"` so the redesign loop gets another shot instead of hitting the placement-retry cap.
    - Low-confidence ICP matches and magnitudes ≤ 10mm are left untouched.
  - `classify_checklist()` gained an optional `engine_summary: Optional[Dict] = None` parameter and runs the measurement-feedback pass AFTER the deterministic/LLM stages (catalog-infeasible drops still win).
  - Thresholds `CRITIQUE_MAGNITUDE_MIN_MM` (10) and `ICP_FLUSH_MAX_MM` (2) exported as module constants.
  - Self-test `__main__` block grew 6 Layer-2 fixtures (flush-refute, escalate-to-topology, below-threshold skip, already-topology skip, low-confidence skip) and 5 magnitude-parsing direct fixtures (mm / cm / m / multi-unit / unitless). All pass.

### Layer 3a — prompt ban on geometry prescriptions

- Inline in `VALIDATION_SYSTEM_PROMPT` inside `core/ai/claude_client.py`. One paragraph ("Describe symptoms, do NOT prescribe geometry") instructs Gemini to describe *what* looks wrong without prescribing angles / sign flips / mirror instructions. Specifically calls out the quadruped hip_pitch mirror that produced the image-24 crossed-leg stance.

## Deviations from the plan

- Plan said "Layer 2 deferred until Layer 1 is in production." User explicitly asked for Layers 1, 2, and 3a in one session. Shipped accordingly.
- Plan mentioned a possible `core/ai/validator_prompt.py` file. The validator prompt actually lives inline in `core/ai/claude_client.py` as `VALIDATION_SYSTEM_PROMPT`. Edited it in place.
- Plan suggested placing the table in a separate content block. I concatenated into the existing URDF prompt_text so both the URDF and the ground-truth block live in the same user-turn part (matches the surrounding reference-image / view-instruction pattern already in `_validate_assembly_gemini`).

## Verification

- `cd src && npx tsc --noEmit` — clean except the pre-existing `GTAOPass` TS6133 warning in `main.ts:7`.
- `python -m py_compile core/ai/critique_classifier.py core/ai/claude_client.py core/server.py` — clean.
- `PYTHONIOENCODING=utf-8 PYTHONPATH=. python -m core.ai.critique_classifier` — all 11 deterministic fixtures + 6 Layer-2 fixtures + 5 magnitude fixtures pass.
- Manual trace against image-24: the rendered engine summary block names `sensor_depth_camera_small_5` with `gap_p50=-1.72mm / confidence=high / paired=80/80`, which the new "do not contradict" instruction tells Gemini to treat as flush. Layer 2 provides the classifier-side backstop: if Gemini still emits "floating ~45mm", the classifier drops it with `classifier_drop=true` and the explanatory reason.
- No live quadruped generation performed — Gemini quota + need to avoid destabilizing the existing Session-5 unstaged work on this branch. The manual trace is sufficient per the task's fallback verification criterion.

## Follow-ups / new failure modes noticed

- **Layer 2 magnitude regex requires a unit suffix.** "Shifted 38mm" works; "shifted significantly" does not. Fine for the historical failure case (validator always quantifies) but worth revisiting if Gemini ever drops the numeric magnitude.
- **ICP entry lookup is first-match by substring.** If a critique detail names two link_names (e.g. "shin_extrusion_rl clips into actuator_servo_high_torque_30"), only the longest-name substring wins. Good enough today because detail phrasing usually leads with the offending child, but worth tracking.
- **Placement table can grow large.** 200-row cap per table keeps tokens bounded. A typical Go1 quadruped is ~50 rows, so no issue for current workloads.
- **Confidence heuristic is static (≥ 75%).** Not tuned empirically; docs/ICP_RESIDUALS.md mentions 10% paired is "low". 75% splits the real traces cleanly today but may need revisiting when rotated-mate coverage improves.
- **No Layer 3b (canonical pose reference) yet.** Deferred to the biped/arm workstream — aligns with the original plan.

## Files changed

```
src/src/urdfAssembly.ts
src/src/viewportChat.ts
src-tauri/src/lib.rs
core/server.py
core/ai/claude_client.py
core/ai/critique_classifier.py
```
