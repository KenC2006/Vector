# AI Camera-Bracket Regression (landed)

**Status:** Easy + Medium fixes from `AI_CAMERA_BRACKET_REGRESSION.md` shipped on `engine-gaps-integration` 2026-04-23. Orthogonal Gemini `fix_kind` schema extension remains deferred.

## Symptom recap

Across redesign cycles the AI would drop a valid bracket + tilted camera on the baseplate front and replace it with a flat, un-bracketed camera on an arbitrary face (top or right). Visible in the session image: depth camera hovering off the right side of the quadruped chassis with no bracket between it and the baseplate, same height as the baseplate top.

## What landed

### Easy — assembly system prompt rule 13 (core/ai/claude_client.py)

Added a canonical rule to `SYSTEM_PROMPT` (design_robot / modify_topology path). Inserted after the electronics rule and before the Common Patterns section so redesign retries see the same constraint:

> 13. **Cameras and lidars NEVER mount directly to a baseplate face.** Always interpose a `structural_bracket_l`: the bracket mates to the baseplate via `mate_connector: "wall_outer"` on the chosen face (front/back/left/right), then the sensor mates to the bracket via `attach_connector: "wall_inner"` (or `"plate_top"` for an upward-facing sensor) + `mate_connector: "mount_back"` + `mate_type: "fastened"`. See the L-bracket + camera example in the Mate Connectors section above for the exact fields. This rule is canonical — it holds across redesign cycles. If a validator says the camera is "floating", "not visibly mounted", or "offset from the baseplate edge", the fix is to ADD a bracket or reposition the existing bracket's mount face — NEVER move the sensor onto `attach_face: "top"` of the baseplate to "make it sit flat". A top-face baseplate mount with no bracket is the same bug in a different orientation.

Why the wording is load-bearing:
- "NEVER" + "canonical" + "holds across redesign cycles" anticipates the exact failure mode (pass-2 degeneration).
- Explicitly calls out the known anti-pattern ("move the sensor onto attach_face: top to make it sit flat") so Gemini feedback pressure doesn't produce that response.
- Points back at the already-in-prompt example for the exact component-field values rather than duplicating them — keeps token cost low.

Only `SYSTEM_PROMPT` was touched. `ASSEMBLY_SYSTEM_PROMPT` (the tool-call-per-component variant) is a different code path used by `generate_assembly_with_tools` and is not the regression site; added there would just burn tokens.

### Medium — elevation_angle on top/bottom faces (src/src/urdfAssembly.ts)

Before: `computeFacePlacement` silently dropped `elevation_angle` on top/bottom faces. Side faces honored it but top/bottom always emitted `rpy: "0 0 0"`.

After: top emits `rpy: "0 -elevRad yawRad"`, bottom adds `elevRad` onto the existing `pitchRad` (so splay/wheel rotations still compose). Both sit next to the existing side-face branches for consistency.

**Sign convention deviation from the plan doc:** the plan prescribed `top: 0 elevRad 0` / `bottom: 0 -elevRad 0`. The plan didn't trace the existing side-face math. Front uses `rpy: "0 -elevRad 0"` so elev=−10° → pitch=+10° around Y → sensor's +X rotates toward −Z (tilted down-forward). To keep the user-facing semantic "negative elevation = tilt forward-down" stable across every face, top must also use `-elevRad`, not `+elevRad`. Bottom mirrors with `+elevRad`. Comments at each new branch record the convention so the next reader doesn't re-derive it.

## Deviations from the plan

1. **Sign convention flipped** from the doc (see above). Landing the doc sign would have produced top-mounted cameras that pitched backward for negative elevation values, contradicting the side-face convention. The plan doc has been updated with a superseded-pointer header.
2. **Fixture pair skipped.** The plan called for a fixture pair in `alignmentCorpus.ts`. That harness exercises `reconcileNodePlacement` (mesh/bbox alignment), not `computeFacePlacement`. `computeFacePlacement` is a closure inside `initUrdfAssembly` (urdfAssembly.ts:2028, inside the exported factory at :344) and isn't importable from any corpus. Options considered: (a) skip and rely on smoke test; (b) extract `computeFacePlacement` to module scope (~100-line refactor capturing parent-bounds lookup and MateConnector types); (c) end-to-end test through `initUrdfAssembly` (needs DOM + catalog + XML parsing). Shipped (a). Regression coverage for this change = manual smoke test (see below). Test debt is real; tracked as a follow-up below.

## Verification

- `cd src && npx tsc --noEmit` → only the pre-existing `GTAOPass TS6133` warning on `src/main.ts:7`. Expected per the session brief.
- `python -m py_compile core/ai/claude_client.py` → OK.
- `cd src && npm run test:alignment-corpus` → 14/14 passed (confirms the elevRad edits didn't break reconcile's scene-level expectations).
- `cd src && npm run test:mate-corpus` → 47/47 passed (confirms the closed-form resolveMate parity against legacy bbox math still holds).
- **Live smoke test — user to run.** Start the app, generate a quadruped with explicit prompt: "include a depth camera mounted to the front of the baseplate with -10° downward tilt for navigation." Expected pass-1 output: bracket inserted between baseplate.front and camera, camera tilted ~10° downward, no Gemini complaint about floating. After one redesign cycle: the bracket should still be present. If the AI still mounts the camera directly (no bracket), the prompt wording needs iteration — surface the failing critique text and we tune the rule, not declare done.

## Follow-ups / known gaps

- **Sign-convention smoke check for top-face elev.** The side-face-consistent sign is reasoned, not measured. If a user reports "camera on top with elev=-10 tilts the wrong way," it's a single-character flip (`-elevRad` → `elevRad` on the top branch, matching sign on bottom). The comment at the branch records the intent so the flip is safe.
- **No direct test for `computeFacePlacement` rpy output.** Either extract the function to module scope, or add an end-to-end harness, to cover the elevRad branches. The closure also holds all the other placement-engine branches (wheels, splay, orientation parsing) that are currently untestable the same way — the refactor would pay off broadly, not just for this change.
- **Orthogonal Gemini `fix_kind: "insert_bracket"` schema extension** (~half day) remains deferred. The prompt rule is the immediate fix; if redesign cycles still regress after a few field sessions, the structured fix_kind is the next lever.

## Files touched

- `core/ai/claude_client.py` — `SYSTEM_PROMPT` rule 13 inserted.
- `src/src/urdfAssembly.ts` — `computeFacePlacement` top and bottom branches honor `elevRad`.
- `docs/AI_CAMERA_BRACKET_REGRESSION.md` — superseded header added.
