# AI Camera-Bracket Regression

**Status:** **Superseded — Easy (system-prompt rule) and Medium (elev on top/bottom) fixes landed 2026-04-23.** See [`AI_CAMERA_BRACKET_REGRESSION_LANDED.md`](./AI_CAMERA_BRACKET_REGRESSION_LANDED.md) for what actually shipped (including sign-convention correction vs. the plan and the skipped fixture). The Orthogonal Gemini `fix_kind` schema extension remains deferred.

This doc is preserved as the original problem statement and design rationale.

---

**Original status:** Open. Defer until engine plan (Sessions 1-5) lands. Tracked in memory: `project_camera_bracket_regression.md`.

## Symptom

When the AI generates a quadruped with a depth camera, redesign cycles can degrade the camera placement instead of improving it. Observed pattern across multiple smoke runs:

- **Pass 1 output:** `attach_face: "front"` + `elevation_angle: -10` on baseplate. Camera body extends 90mm forward; Gemini correctly sees this as "floating ~47mm in front" because the camera's mesh-back is past the baseplate edge with no visible mount.
- **Pass 2 (AI's "fix" after Gemini critique):** AI moves camera to `attach_face: "top"` + `orientation: "horizontal"`. Drops the elevation_angle. Doesn't insert a bracket. Camera now sits flat on the baseplate corner, oriented along Y-axis instead of X-axis. Gemini's new critique: "oriented incorrectly, missing -10° elevation."

In a separate earlier run the AI did the right thing (baseplate.front → structural_bracket_l → camera.mount_back, fastened mate, elevation preserved). It just doesn't preserve that pattern across redesign cycles.

## Three contributing causes

1. **AI heuristic for cameras is weak.** When Gemini complains about a sensor floating, the AI's instinct is to move it to a flat surface, not "add a bracket between the floating thing and its mount." The bracket pattern works but isn't reinforced as the canonical fix.

2. **`elevation_angle` only fires on side faces.** `computeFacePlacement` honors the parameter for `front`/`back`/`left`/`right` but ignores it for `top`/`bottom`. So when the AI moves a camera from front-mount to top-mount to "fix" floating, the requested tilt silently drops to zero. The tilt request is preserved in the URDF but the placement engine doesn't apply it.

3. **Redesign loses earlier successful patterns.** The AI's redesign input is "previous URDF + Gemini critique." If a successful pattern (camera on bracket on front) existed in pass 0 but the redesign starts fresh from a minimal URDF, the bracket choice gets re-derived from scratch and can regress.

## Fixes ranked by effort

### Easy: strengthen the system prompt (~30 min)

In `core/ai/claude_client.py` (or wherever the assembly system prompt lives):

> Cameras and lidars must NEVER mount directly on a baseplate face. They mount on a `structural_bracket_l` whose `wall_outer` connector mates to the baseplate face, with the sensor on the bracket's `plate_top` connector via fastened mate. This applies whether the sensor faces forward, sideways, or upward.

This single instruction should fix Pass 1's "no bracket" issue and survive redesign cycles because the rule is canonical, not contextual.

### Medium: extend `elevation_angle` to top/bottom faces (~1 hr)

In `urdfAssembly.ts` `computeFacePlacement`, the top/bottom cases don't read `elevRad`. Either:
- Add an `elevRad` rotation to the top/bottom rpy output (so a sensor on top.face with elevation_angle=-10 gets pitched down 10°), OR
- Document the limitation and force the AI prompt to never combine `top`/`bottom` with `elevation_angle`

The first option is more user-friendly. Implementation: top-face gets `rpy: 0 elevRad 0` (pitch around Y for forward tilt), bottom-face gets `rpy: 0 -elevRad 0`. Add fixture in `alignmentCorpus.ts`.

### Orthogonal: Gemini critique categorization (~half day)

Currently Gemini outputs flat checklist items with `fixable_by` enum (placement / topology). Extend the schema so a "missing bracket" finding is tagged distinctly from a "wrong placement" finding:
- Add `fix_kind: "insert_bracket"` to Gemini's vocabulary
- When Claude consumes a `fix_kind: insert_bracket` critique, the system prompt instructs it to insert the bracket rather than reroute the topology

This is more invasive but fixes the redesign-loses-pattern problem at the root.

## Recommended sequence

1. Ship the prompt fix first (easy, immediate win, survives redesigns).
2. After engine Sessions 1-5 land, fold in the elevation_angle-on-top change as a small follow-up.
3. Defer the Gemini schema extension to a separate session focused on validation/critique improvements.

## Files involved

- `core/ai/claude_client.py` — assembly system prompt
- `src/src/urdfAssembly.ts` — `computeFacePlacement` top/bottom branches
- `core/ai/critique_classifier.py` — Gemini's `fixable_by` taxonomy

## Test cases that would catch regression

- Generate a quadruped with explicit prompt: "include a depth camera mounted to the front of the baseplate with -10° downward tilt for navigation." Expected output: bracket inserted, camera tilted, no Gemini complaint about floating.
- Generate a robot with multiple sensors. Expected: each sensor on a bracket; no direct baseplate mounts.
- After any AI redesign cycle: camera placement should not REGRESS from front+bracket+tilt to top+no-bracket+no-tilt.
