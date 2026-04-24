# Engine Next Steps — Post-Layer-5 Plan

## What this doc is

Synthesis of three research agent surveys (run on `engine-gaps-integration` after Layer 5 landed) plus the Layer-1-through-5 lessons. Covers two intertwined problems:

1. **Visible mating-face gaps** that survive perfect connector placement (chamfered cube top vs flat disc bottom — Layer 5's `actuator_servo_high_torque` close-up shows this).
2. **The remaining `bounding_box_mm` authoring drift surface** — the same class of bug as Layer 1 (`structural_baseplate_large` 8mm bbox vs 30mm rendered mesh), still latent on every other component.

Goal of the plan: get to a state where **importing an arbitrary STEP file** and asking the AI to mate it produces clean visuals **without per-component manual prep**, and the entire bbox-vs-mesh drift class of bugs is permanently eliminated.

## Where we are now (after Layer 1-5)

Branch: `engine-gaps-integration`. Local-only, not pushed.

| Layer | Commit | What it did |
|-------|--------|-------------|
| 1 | `9fda2b9` | Parametric plate generators (`generateBaseplate` etc.) render at exactly `bbox.z` instead of auto-thickening to `max(d, min(w,h)*0.12)`. Killed the 21mm uniform-shift bug on baseplate top-face children. |
| 2 | `4635c0e` | `computeFacePlacement` accepts `parentConnectors` — when authored, the connector origin overrides bbox half-extents for face-normal positioning. Multi-child distributions now route through the connector path. |
| 3 | `fc1a0ab` | `placed_via_connector: boolean` flag on `AssemblyComponent`. Set by `computeMatePlacement` and Layer-2's path. `reconcileAlignment.ts` skips flagged children — connector positions are authoritative, bbox-derived deltas can't stomp them. |
| 4 | `b29663a` | `computeFacePlacement` also consults child's authored connectors (`top`/`bottom` aliases on coupler, body-style connectors on servos). `matePlacement` caller splices `length_mm` into bbox for parametric parents (extrusions). Killed the 26mm coupler-on-extrusion gap. |
| 5 | `f5f489d` | `SHAFT_OVERLAYS` map + `applyMeshToLink` shaft cylinder injection. For `actuator_servo_high_torque`/`heavy_duty` (single-primitive GLBs that don't separate body/shaft), scales body to `bbox.z - shaft_length` and adds a procedural metal cylinder above. Body-flush mating becomes possible. |

What still bites visually:
- Chamfered top edges on servos vs flat disc bottoms on couplers → ~1-3mm visible gap at every leg joint (`Image 12`).
- Body-vs-shaft estimates baked per-preset (`shaft_length_mm: 5` is a guess; correct value would be measured per GLB).
- Future imported components have no authored connectors at all — they fall through the bbox path and break in the same ways the original baseplate broke.

What still bites architecturally:
- `bounding_box_mm` is an authored field that drifts from the rendered mesh whenever someone forgets to keep them in sync. Layer 1 was an instance of this; the audit in `scripts/audit-preset-bboxes.js` already surfaces 61 more instances on GLB-based presets.
- Sim collision is emitted as primitive `<box>` from `bounding_box_mm` — wrong by definition for any non-rectangular component, and not what production tools (Drake, Mujoco, Isaac) use.

## The architectural insight

**Drop *authored* bbox. Keep *derived* bbox.**

This was the framing the user landed on after the research came back. It's correct.

- **Authored bbox** = the typed `bounding_box_mm` field in `generic_presets.json`. This is the drift source. Every Layer-1-class bug starts here. Every imported STEP file requires a human to type one in.
- **Derived bbox** = `getRenderedMeshDims(component_id)` — already exists in `richVisuals/index.ts`. Walks the loaded mesh once, caches the AABB. Single source of truth: the mesh.

Replacing the authored field with on-demand derivation removes the entire drift surface. AI prompts get the same "this is 46×36×34mm" they currently get; validators get the same size info; reconcile fallback uses the same numbers. Just no possibility of drift.

This is what Onshape, Drake, Isaac Sim, and Mujoco all do in practice (per Agent 3). Authored AABB is a legacy ROS-URDF convention that the modern stack has moved away from — production uses convex-hull collision and on-demand mesh AABB for spatial reasoning.

## Six-step plan, prioritized

### Step 1 — `engagement_depth_mm` on connector schema (~30 min)

The smallest immediate win. Adds a signed scalar field to mate connectors:

```jsonc
{
  "id": "top",
  "origin_xyz_mm": [0, 0, 12],
  "axis_xyz": [0, 0, 1],
  "type": "planar",
  "engagement_depth_mm": 1.5  // child translates -1.5mm INTO parent along axis
}
```

Solver semantics: after the standard frame-alignment transform places the child, translate it by `-engagement_depth_mm` along `connector.axis_xyz` (in parent frame). Default 0, fully backward-compatible.

**Naming chosen** per Agent 2's survey: `engagement_depth_mm` is novel as a *named field* but the underlying mechanism (offset baked into connector frame) is universal — Onshape, Fusion, Unreal sockets all do equivalent things via different surface APIs. Closest precedent: Unreal's `RelativeLocation` on a SkeletalMeshSocket. We're just exposing what other tools bake invisibly.

Use it on chamfered-top connectors — `actuator_servo_high_torque.top: engagement_depth_mm: 1.5` lets the disc edge sit just below the chamfer line, hiding the visible gap.

**Files:** `mateConnectors.ts` (schema), `urdfAssembly.ts` (apply in `computeMatePlacement` and Layer-2's `computeFacePlacement` path), `generic_presets.json` (author per-connector values).

### Step 2 — Runtime ICP-along-normal nudge (~3-4 hr)

The "no per-component authoring required" lever. **This is the one that delivers "import any component, get clean visuals."**

For each face mate where `engagement_depth_mm` isn't authored:

1. Sample ~64 points on the parent's contact face (raycast into `linkLocalAABBExcludingPivots` mesh near the connector origin)
2. Sample ~64 points on the child's contact face
3. Project both sets onto the connector axis (collapse to 1D scalars)
4. Compute the percentile of signed distance that gives ~0.2mm visible interpenetration (Agent 1's recommendation)
5. Cap the nudge at 3mm to bound damage from bad inputs
6. Apply as a translation along `connector.axis` in the placement xyz

Sub-millisecond per joint. Run once at assembly time, bake the nudge into the placement so reconcile and persistence stay clean.

**Honest caveat from Agent 1:** "Auto-clean visuals from any imported component" is **not solved as a first-class feature** in any production tool surveyed (Onshape, SolidWorks, Fusion, Isaac Sim, Gazebo, Drake, MoveIt, Unreal, Unity, Mujoco, RViz, ROS-Industrial). All of them rely on curated authoring. ICP-along-normal is a credible algorithmic path no one has shipped — implementing it puts Vector slightly ahead of the industry, not catching up. Plan for tuning iterations and edge cases.

**Files:** new module `src/src/contactCleanup.ts`, hooked in `urdfAssembly.ts` after `computeMatePlacement`/`computeFacePlacement` returns.

### Step 3 — Demote `bounding_box_mm` from authored to derived (~1 day)

The architectural cleanup that kills the entire drift class.

1. Add `getOrComputeBbox(component_id, preset)` helper that returns `getRenderedMeshDims(component_id)` if mesh is loaded, else falls back to `preset.physical.bounding_box_mm` for legacy. Cached.
2. Replace every read of `preset.physical.bounding_box_mm` (32 sites currently) with `getOrComputeBbox(...)`. Targets: `urdfAssembly.ts`, `topologyValidation.ts`, `reconcileAlignment.ts` fallback path, MJCF export, AI prompt builders, mount node placement.
3. Once all reads route through the helper, delete `bounding_box_mm` from non-parametric components in `generic_presets.json` (parametric ones still need it as input to the generator). Keep it as an optional override field for legacy.
4. Audit script (`scripts/audit-preset-bboxes.js`) becomes a static check: any preset declaring `bounding_box_mm` AND a `meshOverride` gets flagged unless the bbox is explicitly marked `legacy_override: true`.

After this, the "three numbers must agree" problem (declared bbox, authored connectors, rendered mesh) is reduced to "two numbers must agree" (authored connectors and rendered mesh) — and Step 1+2 close that gap.

**Files:** new helper in `urdfAssembly.ts` or extracted to `componentDims.ts`, mass replacement across the listed sites, `generic_presets.json` cleanup.

### Step 4 — Convex-hull collision instead of primitive `<box>` (~half day)

Production standard per Agent 3. Boston Dynamics ships mesh collision on Spot. Isaac Sim defaults to convex hull. ANYbotics hand-authors primitives but that's the legacy ROS pattern, not the modern one.

1. At component import time (one-shot), run CoACD or three-mesh convex hull on the loaded GLB → produces 1-N convex pieces.
2. Cache as `_collision.obj` next to the GLB.
3. URDF emission switches from `<collision><box size="..."/></collision>` (current) to `<collision><mesh filename="package://meshes/{id}_collision.obj"/></collision>`.

This unlocks proper sim physics (rounded corners actually round in Mujoco), removes the bbox dependency in collision export, and matches what Spot/Isaac/SAPIEN ship.

**Library options:** [CoACD](https://colin97.github.io/CoACD/) (modern V-HACD replacement) or rolling our own three-mesh convex hull (simpler but produces a single hull, less faithful for concave shapes).

**Files:** new `scripts/generate-collision-meshes.js` (one-shot per GLB), new field `collisionMeshFile` on preset, `urdfAssembly.ts` collision emission.

### Step 5 — Multi-child face distribution from connector tangent extent (~half day)

Currently Layer 2's `_computeMultiChildOffsets` distributes children using parent bbox face dimensions. Replace with: sample mesh extent along the connector's tangent axes (the u/v plane perpendicular to the connector normal). Same source of truth as everything else — the mesh.

For a `top` connector with axis `[0,0,1]`, tangent axes are `[1,0,0]` and `[0,1,0]`. Sample mesh AABB along those two axes near the connector origin. That's the face extent.

**Files:** `urdfAssembly.ts` `_buildMultiChildPositions` and `_computeMultiChildOffsets`.

### Step 6 — Retire reconcile (passive, when catalog hits 100% connector authoring)

No active work. As components get connectors authored, `placed_via_connector` is true everywhere, reconcile is a no-op everywhere, and we delete it. Don't rush — let it exist as the safety net for a few releases first to catch edge cases.

## Recommended sequence

1. **Step 1 first** (~30 min) — closes the immediate visible gap on the current quadruped. Authored `engagement_depth_mm: 1-2` on `actuator_servo_high_torque.top` covers the chamfer mismatch.
2. **Step 3 in parallel** (~1 day) — eliminates the entire drift bug class permanently. Doesn't depend on anything else.
3. **Step 2 next** (~3-4 hr) — unlocks the "import any component" goal. Best done after Step 3 because the contact-face sampling consumes derived bbox for sample density.
4. **Step 4** (~half day) — production-standard polish for sim. Not user-visible but matters for downstream Mujoco/Drake users.
5. **Step 5** (~half day) — cleanup pass; eliminates the last bbox dependency in the placement path.
6. **Step 6** — passive.

Total active engineering: ~2-3 days for steps 1-5. Step 4-5 can ship together.

## What this plan does NOT do

- **Does not adopt B-rep.** Per the prior research in `docs/ENGINE_EXECUTION_PLAN.md` and confirmed by Agent 1's survey, no production URDF/robot tool uses B-rep at assembly time. Zoo.dev's clean look comes from B-rep + procedurally-generated parts, neither of which fits an "assemble imported third-party CAD" use case.
- **Does not auto-detect body-vs-shaft on multi-primitive GLBs.** The current `scripts/measure-servo-body-top.js` heuristic (largest-primitive-is-the-body) is wrong for `actuator_servo_standard` (16-primitive GLB where body is multiple primitives). A smarter measurement (cluster primitives by overlap, treat largest cluster as body) is a separate piece of work, not blocking.
- **Does not solve compound housings** (hip housings, pan-tilt modules). Same caveat as the original `ENGINE_EXECUTION_PLAN.md` "What this plan will NOT fix" table — needs sub-assembly concept, future work.
- **Does not promise production-quality "any imported component looks perfect"**. No tool in the industry promises this. Steps 1+2 push the floor much higher than current; tuning per-component-class will still be needed for the long tail.

## Research agent findings (sources)

Three agents ran on 2026-04-21, all completed. Outputs preserved in this section's quotes.

### Agent 1: auto-clean visuals in production tools

> No tool surveyed implements automatic, runtime, mesh-aware visual gap closure as a first-class feature. The closest is per-author offset annotations on the connector itself.
>
> "Import any component, get clean visuals automatically" is not solved in any production tool we surveyed, and is not actively pitched as a research goal in CAD AI work either. The universal industry practice is curated authoring.

Tools surveyed: Onshape, SolidWorks/Composer, Fusion 360, NVIDIA Isaac Sim/Omniverse, Gazebo (modern), Drake, MoveIt, Unreal Engine sockets, Unity Robotics URDF Importer, MuJoCo viewer, RViz, ROS-Industrial.

Algorithmic levers identified: ICP point-to-plane registration along the connector normal, surface wrapping (STAR-CCM+ style, expensive), engagement_depth offsets (universal but always user-authored), VHACD/CoACD convex decomposition (used for collision, doesn't address visual gaps).

### Agent 2: engagement_depth schema conventions

> There is a converged convention, but it's not what you're proposing: bake the offset into the connector's coordinate frame at authoring time (Onshape, Fusion, Unreal sockets, MJCF sites). The other converged pattern is per-mate-instance distance offsets (SolidWorks Distance Mate, Unity anchors, URDF joint origin tweaks). Nobody ships a named scalar called "engagement depth" on the connector schema.
>
> Mildly novel as a named field, but the underlying idea is well-trodden. What every tool actually does: authoring-time, move the connector's Z-origin into the parent body by N mm. Done.

Recommended schema (adopted in Step 1):

```jsonc
{
  "name": "top_mount",
  "origin": [0, 0, 25],
  "axis":   [0, 0, 1],
  "engagement_offset_mm": -3.0  // signed; negative = child sinks INTO parent
}
```

Closest existing precedent: Unreal SkeletalMeshSocket `RelativeLocation` (declarative on the socket, but a vector not a semantic depth).

### Agent 3: bbox necessity for non-placement uses

> Don't drop bbox; demote it from a stored field to a derived projection of the mesh.
>
> Sim collision: No production stack uses raw AABB collision. Modern default = convex hull or convex decomposition (V-HACD, CoACD).
> AI spatial reasoning: VLM/VLA stacks (RT-2, OpenVLA, ECoT) consume per-object AABB + label, not raw mesh — bbox is load-bearing here, but should be derived from mesh.
> UI snap: Onshape pattern is mesh BVH raycast + connector-frame snap. Bbox not in the loop.
> Sim primitives: ANYmal hand-tunes primitive collision (legacy), Spot ships mesh collision (modern), Isaac defaults to convex-hull-from-mesh (modern).

Production examples cited: [Spot URDF (mesh collision)](https://github.com/heuristicus/spot_ros/blob/master/spot_description/urdf/spot.urdf.xacro), [ANYmal D URDF (primitive collision)](https://github.com/ANYbotics/anymal_d_simple_description/blob/master/urdf/anymal.urdf), [Mujoco geom types](https://mujoco.readthedocs.io/en/stable/XMLreference.html), [CoACD modern decomposition](https://colin97.github.io/CoACD/), [Drake hydroelastic](https://drake.mit.edu/doxygen_cxx/group__hydroelastic__user__guide.html).

VLM citations: [ECoT (bbox-based VLA reasoning)](https://embodied-cot.github.io/), [OpenVLA](https://openvla.github.io/), [SP-VLA point/box grounding](https://sp-vla-anonymous.vercel.app/).

## Status of related docs

| Doc | Status |
|---|---|
| `docs/ENGINE_EXECUTION_PLAN.md` | Phase 1 (reconcile gate) + C1+C4 done. Phase 2-Core partial. Phase 3 retire-reconcile = Step 6 here. |
| `docs/ENGINE_RESIDUAL_GAP_FIX.md` | Layers 1-3 closed every issue it documented. Superseded for active work. |
| `docs/MATE_CONNECTOR_MIGRATION.md` | Background reference. Step 1 of this plan extends the connector schema. |
| `docs/ENGINE_ALIGNMENT_PLAN.md` | Superseded since the original 2026-04-17 reconcile-as-primary-fix doc. Keep as history. |

This doc is the active plan from 2026-04-21 onward.
