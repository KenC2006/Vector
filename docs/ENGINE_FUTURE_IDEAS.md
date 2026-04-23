# Engine Future Ideas — Post-2026-04-23

Captured 2026-04-23 after a survey of what's actually shipped (Sessions 1-5, validator measurement feedback, camera-bracket fix, all of WS1-WS5 from the original IMPROVEMENT_PLAN). These are ideas that are NOT in any other plan doc and NOT yet implemented. Some may already have implicit precondition work shipped that makes them cheap; others are genuine future directions.

Order is rough leverage / cost ratio, highest first.

## 1. Self-tuning preset connectors using ICP feedback (~half day, high leverage)

**Idea:** the validator-measurement-feedback work (Layer 1, commit `4f45289`) now flows engine-computed ICP gaps to Gemini per joint. The same data could be aggregated across many robots into a "preset health report" that flags connector positions consistently producing >Nmm gaps.

**Concrete:** add a build-time aggregator that reads recent session logs, groups ICP gap entries by `parent_preset.connector_id` → child preset, and surfaces:
- connectors whose median gap across N≥10 robots is > 2mm (likely authored origin off by that delta)
- connectors whose paired-sample ratio is consistently low (likely sample radius wrong relative to the smaller face — this is also Adaptive Sample Radius, see #4 below)

**Why this is realistic now:** the data is already being collected. The validator-feedback session showed that ICP gap entries reach Gemini per joint. Persisting them to disk and aggregating is a separate concern — no engine math changes needed.

**Why it matters:** every preset author currently hand-measures connector origins from the GLB. This pipeline would tell you which presets are wrong without anyone needing to look at a render. Closes the long-pole "import any STEP file and it just works" goal indirectly — by making preset authoring quality a measurable metric.

**Files (when picked up):** new `scripts/aggregate-icp-feedback.mjs`, persistent log writer somewhere in `viewportChat.ts` or `core/server.py`. New `docs/PRESET_HEALTH_REPORT.md` schema.

## 2. Multi-pass ICP (~30 min, marginal but trivial)

**Idea:** `docs/ICP_RESIDUALS.md` says "marginal win, not worth it" but that was when ICP itself was a new thing. Now that contactCleanup is a proven module, calling it twice is a one-line change.

**Concrete:** in `runContactCleanupPass`, after the first pass commits its nudges to placement xyz, re-run sample-and-project on the new positions. Cap total iterations at 2 (third pass would over-correct). If the second pass produces nudges < 0.5mm everywhere, skip it.

**Why marginal:** most remaining gaps are <2mm after pass 1. The handful of cases where pass 2 would help (rotated mates with multiple contact regions) are also the same cases where 3D ICP would actually solve it. Pass 2 is a band-aid.

**Why worth doing anyway:** trivial cost (one outer loop), measurable improvement on the ~6 mates per quadruped that currently sit at the cap. Honest expectation: ~30% reduction in "leftover gap after ICP" log lines.

**Files:** `src/src/contactCleanup.ts`. Add a fixture in `alignmentCorpus.ts` covering a parent that needs two passes to converge.

## 3. Body-vs-shaft auto-detect on multi-primitive GLBs (~1 day, unlocks catalog)

**Idea:** Layer 5 of the engine plan added a procedural shaft overlay for `actuator_servo_high_torque` / `heavy_duty` (single-primitive GLBs). It can't run on `actuator_servo_standard` (16-primitive GLB) because we can't auto-detect which primitives form the body vs the shaft / horn / mounting. The engine-next-steps doc explicitly calls this out as "not blocking, smarter measurement is a separate piece of work."

**Concrete:** in `scripts/measure-servo-body-top.js` (or a new `scripts/cluster-glb-primitives.mjs`):
1. Load every primitive in the GLB as a separate AABB.
2. Build an overlap graph (primitives whose AABBs intersect are connected).
3. The largest connected cluster by total volume = body. Everything outside = shaft / horn / accessory.
4. Body top = max-Z face of the cluster. Shaft length = (overall max-Z) − (body max-Z).

**Why now:** the engine path that consumes this output (Layer 5's `SHAFT_OVERLAYS` map + `applyMeshToLink` shaft injection) already exists. This adds the auto-measurement so that table can be populated automatically for new components.

**Why it matters:** unlocks every multi-primitive servo GLB in the catalog (~12 actuator presets currently can't get the shaft-overlay treatment because their measurements are wrong). Reduces the per-preset hand-authoring burden.

**Files:** new `scripts/cluster-glb-primitives.mjs`, populate `SHAFT_OVERLAYS` in `meshOverrides.ts` for the previously-skipped presets, regenerate the overlay test corpus.

## 4. Adaptive sample radius for ICP (~1 hour, fixes documented residuals)

**Idea:** documented in `docs/ICP_RESIDUALS.md` Section B+C as a deferred fix. ICP currently uses 0.8× of the *parent's* face extent as the sample radius. When the child's face is much smaller (extrusion top on baseplate top), 88-92% of child samples miss the parent surface. Tight percentile spread says the real gap is small but the confident-cap won't fire on <75% paired ratio.

**Concrete:** sample radius = `0.8 × min(parent_face_extent, child_face_extent)`. One line in `contactCleanup.ts`'s sampling logic.

**Why now:** validator measurement feedback now surfaces these "low-pair-coverage" cases as informational ICP entries. Fixing them eliminates a class of inconsistent confidence labels in the Gemini feedback table.

**Why it might wait:** ICP residuals doc says "ship if extrusion-edge gaps become a complaint." User hasn't complained yet. Would still be a good cleanup pass.

**Files:** `src/src/contactCleanup.ts` (1-line fix + comment), add an alignmentCorpus fixture for the small-child-on-large-parent case.

## 5. Connector authoring UI (~multi-day, ambitious)

**Idea:** the long-pole for "import any STEP file" is that every new component needs hand-authored connector origins/axes by an engineer. A click-on-face tool inside the existing C3 viewport overlay (Shift+C) would let users define connectors without code changes.

**Concrete:** extend the C3 overlay with an "author mode":
1. Shift+C shows existing connectors (current behavior).
2. Author mode: click a face → engine snaps to nearest mesh face → shows axis arrow → user can rotate axis with arrow keys → confirm → connector entry appended to `generic_presets.json`.
3. Reload + ICP comparison: existing C3 overlay shows a green/red gauge of pre-vs-post-author gap.

**Why this matters most for non-engineers:** anyone could add a new STEP file to the catalog without touching `generic_presets.json` manually. This is the missing layer between "drop a STEP in the meshes folder" and "the AI can use it cleanly."

**Why deferred:** UI work, not engine work. Genuine multi-day. Would need design discussion before implementation.

**Files (rough scope):** `src/src/viewportOverlays/connectorAuthor.ts` (new), updates to `meshOverrides.ts` for live-edit support, `core/presets/` write path from the frontend.

## 6. Compound housings as sub-assemblies (architectural, deferred)

**Idea:** `ENGINE_NEXT_STEPS.md` flagged this as "out of scope, future work." Hip housings, pan-tilt modules, gripper modules contain multiple connectors that should move together as a kinematic group but currently must be authored as a flat list of components.

**Concrete:** a new preset type `compound_assembly` containing:
- a list of internal components with their relative poses
- a single "external" connector exposed to mating
- internal joints fixed (or kinematically linked, e.g. pan-tilt with two revolute joints)

When mated, the engine treats the compound as a single rigid (or semi-rigid) component for placement, but expands to its internal structure for rendering and joint-state.

**Why deferred:** requires schema changes, placement-engine changes, render-time changes, and a sub-assembly authoring tool. Earliest sensible time is after #5 (connector authoring) so the same UI can author compounds.

**Why it matters eventually:** lots of off-the-shelf robotics parts ARE compounds (Dynamixel hip housings, pan-tilt heads, off-the-shelf grippers). Treating them as monolithic loses kinematic detail; treating them as flat lists makes the AI's job harder.

## What's NOT here

These are deliberately omitted because they're already covered in other docs:

- **AssemblyGraph preservation, tool-call edit surface, dynamic catalog, Gemini hardening, render-time alignment** — all shipped as WS1-WS5, see corresponding `_landed.md` memory entries.
- **Camera-bracket regression, validator measurement feedback** — shipped, see `_landed.md` memories.
- **3D ICP for rotated mates** — `docs/ICP_RESIDUALS.md` says ~10× effort for marginal benefit; revisit only when rotated mates become a primary visual concern.
- **L-bracket auto-rotation (P1), grounding on tall pedestals (P2), arm rest-pose over-fires (P4), pan-tilt vs sensor-on-actuator (D1)** — tracked in `memory/project_known_issues.md`. These are bug-fix work, not new ideas.
- **Sim realism** — entire separate plan in `docs/SIM_REALISM_PLAN.md`. Engine-adjacent but a distinct workstream.

## Recommended pick order

If you want one thing to ship: **#2 multi-pass ICP** (30 min, measurable win, no risk).

If you have a half day: **#1 self-tuning preset feedback** (highest leverage on quality, uses data already being collected).

If you want a real engine investment: **#3 body-vs-shaft auto-detect** unlocks the rest of the multi-primitive servo catalog and removes a per-preset authoring step.

#5 and #6 are real investments — discuss before committing.
