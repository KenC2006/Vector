# Engine Architecture: Two Engines or One?

Investigates whether Vector's "two engines" (Three.js render vs URDF placement) are causing the rotation/placement bugs we keep seeing, and whether unifying them is the right next move.

## TL;DR

It's not really two engines — it's **three systems with an incomplete sync layer**:

1. **Preset spec** (`generic_presets.json`) — declared bounding boxes, ports, axes
2. **Placement engine** (`src/src/urdfAssembly.ts`) — pure spatial algebra computing joint origins from preset specs and rendered mesh dims
3. **Rich visuals** (`src/src/richVisuals/index.ts`) — loads real GLBs, applies rotation overrides, caches actual extents

The intent is for #3 to feed back its real measurements to #2 via `getRenderedMeshDims()`. It does — but **only for components with a loaded GLB**. Parametric fallbacks and slow-blacklisted components bypass this, creating incoherence. ROTATION_OVERRIDES is a manual sync table that requires ongoing maintenance per new component.

The bugs you're seeing (P3 mesh/bbox mismatch, M1 torso offset, P1 L-bracket rotation, auto-bracket visual seams) are all incomplete-sync symptoms — not "Three.js disagrees with the engine," but "three systems sometimes don't share the same numbers."

**"Combine them" isn't quite the right move.** The right move is to make the sync complete and one-directional, not to fuse the systems. Recommendation at the bottom.

## The Actual Architecture

### System 1 — Preset Spec
- Source: `core/presets/*.json` (the canonical catalog)
- Carries: `bounding_box_mm`, ports (mount_face / shaft), axes, masses, materials
- Authority: when a GLB hasn't loaded yet, this is the only data the engine has

### System 2 — Placement Engine (`urdfAssembly.ts`)
- Pure symbolic math — computes joint xyz/rpy from parent + child dimensions
- Key entrypoints: `_performPlacement`, `computeFacePlacement`, `getParentBounds`
- Hybrid input: prefers rendered mesh dims (System 3), falls back to preset bbox (System 1) when no mesh is loaded
- Output: assembly graph → URDF XML

### System 3 — Rich Visuals (`richVisuals/index.ts`)
- Loads STEP-converted GLBs at runtime via Three.js
- Applies `ROTATION_OVERRIDES` (`meshOverrides.ts`) to realign mesh axes to match preset axis convention
- Per-axis scaling so the GLB extents match `bounding_box_mm`
- Caches final dimensions; subsequent placement queries hit `getRenderedMeshDims()` and get the corrected numbers

### The sync layer
- `getParentBounds()` (urdfAssembly.ts) is the bridge — checks `getRenderedMeshDims()` first, falls through to URDF primitives if no cache hit
- `ROTATION_OVERRIDES` is the manual escape hatch when GLB axes don't match the preset's declared axis convention
- `groundRobot()` in `main.ts` is a third-party adjustment — post-render Y-shift that neither System 1 nor System 2 knows about

## Where Disagreement Becomes a Bug

### Asymmetric mesh vs preset bbox (P3 / M1)
- `structural_servo_coupler_disc` preset declares `[32, 32, 8]` but GLB has `[31.96, 8, 31.98]` — axes swapped
- Placement engine asks `getParentBounds()` for the first child placement → mesh hasn't loaded → returns preset bbox
- GLB then loads, dimensions update; second child on same face placed using *different* extents
- **Effect**: visible seams between siblings on the same parent face

### L-bracket rotation (P1)
- `ROTATION_OVERRIDES` rotates the mesh geometry in `applyMeshToLink()` via `applyMatrix4()`
- This realigns the rendered mesh's local axes
- BUT: placement engine computes xyz/rpy from `bounding_box_mm` preset values, which were authored against an *assumed* axis orientation
- If the override rotates the mesh into agreement with the preset, fine. If it doesn't, placement is geometrically inconsistent with the visual.
- **Root cause**: the rotation override exists only in the visual layer; the placement engine never sees it

### Auto-bracket visual seams
- Auto-repair inserts `structural_bracket_auto_*` between port-mismatched servos
- These brackets are placed using the same `getParentBounds()` path
- Because they're inserted *during* the placement pass and their parent's GLB may or may not have loaded yet, the bracket attaches at a different point than the eventual rendered position of the parent

### Grounding ambiguity (P2)
- `groundRobot()` shifts the assembly root by `-minY` after Three.js computes world bounds
- Placement engine has no knowledge of this shift — every joint origin is computed relative to the unshifted root
- If a tall pedestal robot has visual mesh extents that don't match its preset bbox, the shift uses Three.js's view of the world but joints were computed against a different one

### Camera depth orientation (M2)
- The depth camera preset places lenses at `rpy=[π/2, 0, 0]` in the visual primitives
- This is hardcoded in the preset's visual definition, not in the placement engine's port semantics
- The mount port says "back is -X" but the visual lens points sideways — disagreement between port semantics (System 1) and visual authoring (baked into System 3 inputs)

**Common pattern**: every one of these traces to "the spatial number that placement engine used isn't the spatial number that ended up in the rendered scene." Different parts of the pipeline see different numbers.

## "One Engine" — What Would It Even Mean?

Three viable architectures:

### Option A — Three.js authority
Make the placement engine *always* read post-load AABBs from Three.js. Remove preset-bbox fallback in `getParentBounds()`.

**Cost**: Placement becomes async-dependent. Must wait for GLB loads before placing children. Refactor introduces queueing, ~200 lines in `urdfAssembly.ts`.
**Benefit**: Single source of truth — no preset-vs-mesh mismatches possible.
**Risk**: Slow-blacklisted meshes block assembly flow. Initial design generation gets slower (need GLBs warm). Doesn't fix the rotation override problem (still a separate sync layer).

### Option B — Presets bake the GLBs (build-time)
Run a build-time script that loads every STEP/GLB, measures real extents, and rewrites `generic_presets.json` with the actual numbers. Placement engine then reads only presets; richVisuals just applies materials.

**Cost**: One-time tooling (~500 lines of Node.js build script). Pipeline complexity — preset updates become coupled to GLB updates.
**Benefit**: Presets become gospel, self-documenting truth. `ROTATION_OVERRIDES` can be eliminated over time as new GLBs are authored to match the preset axis convention from the start.
**Risk**: Existing GLBs with axis swaps still need overrides until they're re-authored. Migration is gradual, not instant.

### Option C — Render-time alignment pass
Keep placement engine pure. After URDF parse + richVisuals load, run a `reconcileNodePlacement()` pass that compares actual mesh AABBs to what placement engine assumed and applies in-scene corrective offsets.

**Cost**: ~300 lines bridging `urdfAssembly` and `richVisuals` with a callback. No preset schema changes.
**Benefit**: Preserves placement engine purity. Fixes the visual disagreement after the fact. `ROTATION_OVERRIDES` stays in the visual layer where it belongs.
**Risk**: Corrective adjustments cascade — if a parent's adjustment shifts, every child needs re-reconciliation. Needs careful topological ordering (root → leaf).

## Is Combining Them Smart?

Honest answer: **not as a single move, and not by collapsing them into one module**. The systems have legitimate separation of concerns:
- Placement engine should be pure (testable from Node, no THREE imports — see `urdfGraphEquivalence.ts` for the model)
- Renderer should handle visuals (materials, lighting, mesh loading)
- Presets should describe the catalog independent of either

What needs to combine isn't the *systems* — it's the *numbers they all agree on*. Right now each has its own slightly different view of dimensions and orientations. The fix is to converge those views, not to merge the modules.

## Recommendation

**Hybrid path: C now, B over time, A maybe later.**

### Near term — Option C (render-time alignment pass)
- Stops visible seams and rotation disagreement immediately
- No preset schema changes, no GLB re-authoring needed
- This is what the deferred P3 anchor-to-rendered-mesh entry in `IMPROVEMENT_PLAN.md` is gesturing at — Option C is essentially that fix done correctly (alignment pass) instead of via per-component overrides
- Connects cleanly with the "Deferred Engine Work" trigger criteria in the improvement plan

### Medium term — Option B infrastructure (presets bake the GLBs)
- Author the build-time measurement script
- Regenerate presets from real GLB extents
- New presets added going forward go through this pipeline → no manual `ROTATION_OVERRIDES` entries needed
- Existing overrides retire one-by-one as GLBs are re-authored

### Long term — Option A becomes viable
- Once presets and meshes agree (via B), Three.js-authority becomes risk-free
- At that point, fallback paths in `getParentBounds()` can be removed and the engine simplifies

### What NOT to do
- **Don't merge `urdfAssembly.ts` into `main.ts` or richVisuals.** Pure placement math is the right design — the corpus tests in `graphPreservationCorpus.ts` only work because the engine is pure.
- **Don't add more `ROTATION_OVERRIDES` / `TRANSLATION_OVERRIDES` entries as the long-term fix.** Each entry is a band-aid that needs maintenance. They're acceptable as interim mitigations but the override table itself is a code smell — if it keeps growing, that's the signal to ship Option C.
- **Don't try to make placement async (Option A) until presets and meshes agree.** Async refactor on top of disagreement just hides the bug behind timing.

## Connection to the Improvement Plan

The deferred `Anchor-to-Rendered-Mesh Placement (P3 root cause)` entry in `docs/IMPROVEMENT_PLAN.md` is a partial Option C — it teaches the engine to anchor to rendered mesh extents instead of preset bbox declarations. That's the right direction. This doc reframes it as: **the trigger criteria are correct, but the execution should be a render-time alignment pass (cleaner architecture), not a per-component override table (band-aid that doesn't scale).**

When you eventually pick up the deferred P3 work, read this doc first. The `TRANSLATION_OVERRIDES` interim mentioned in IMPROVEMENT_PLAN.md is fine as a stopgap, but the real fix is Option C.

## Files Referenced

- `src/src/urdfAssembly.ts` — placement engine, `getParentBounds`, `_performPlacement`, `computeFacePlacement`
- `src/src/richVisuals/index.ts` — GLB loading, `getRenderedMeshDims`, `applyMeshToLink`
- `src/src/meshOverrides.ts` — `ROTATION_OVERRIDES` table (the manual sync layer)
- `src/main.ts` — Three.js entry, URDF parse, `groundRobot` post-render Y-shift
- `src/src/rotationIO.ts` — quat ↔ rpy conversion utilities
- `src/src/urdfGraphEquivalence.ts` — pure module pattern (precedent for keeping placement engine pure)
- `core/presets/*.json` — preset spec (System 1)
