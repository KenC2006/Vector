# Mate Connector Migration — Placement Engine v3

Execution plan for evolving Vector's placement engine from `attach_face` + bbox-half-extent math to **named mate connectors** (frame-to-frame composition). Follow-up to the render-time reconciliation pass from `docs/ENGINE_ALIGNMENT_PLAN.md` (WS5, shipped).

Read the background first if you're new to this:
- `docs/ENGINE_ARCHITECTURE.md` — the three-option analysis; mate connectors are the "get smarter about poses" evolution of Option C
- `docs/ENGINE_ALIGNMENT_PLAN.md` — the render-time reconcile we shipped as WS5
- Research digest from four parallel agents (CAD fundamentals, JS/TS solvers, URDF ecosystem, LLM-CAD prior art) — all converged on this architecture

This doc is about *execution*: schema, math, migration sequence, test criteria.

## Why now — trigger criteria met

Post-WS5 testing confirms Option C closes mesh-vs-bbox **depth** gaps (submerged electronics on raised-lip baseplates, asymmetric-centroid torso). It cannot close:

- **Face-ambiguity** (P1 — L-bracket "top" means plate-top or wall-face?)
- **Concentric mates** (servo shaft into coupler hole — we treat both as boxes)
- **Auto-bracket seams** (placeholder geometry doesn't match real bracket shape)
- **Camera mount orientation** (M2 — lens faces wrong direction post-mount)

These are *semantic* problems, not measurement problems. Render-time reconciliation cannot reach them because the face identity is the ambiguity, not the face position.

Research verdict (unanimous across four agents): LLMs shouldn't do spatial math; the industry pattern is **named frames + typed mates, resolved by closed-form composition**. See AIDL (arxiv 2502.09819), Onshape Mate Connectors, Drake's F/M frame pairs, Rhoban `onshape-to-robot`.

## The model — mate connectors + frame composition

### Connector

A **mate connector** is a named local frame on a preset. Schema:

```jsonc
{
  "id": "shaft_out",           // stable, semantic, globally unique within preset
  "origin_xyz_mm": [0, 0, 13], // frame origin in preset-local coords
  "axis_xyz":      [0, 0, 1],  // primary axis (out-of-face normal, shaft direction, etc.)
  "type":          "cylindrical", // planar | cylindrical | point
  "diameter_mm":   6.0         // optional — only for cylindrical
}
```

Three connector types cover >90% of robot assembly (per CAD-fundamentals research):

| Type | Geometry | Pairs-with |
|---|---|---|
| `planar` | a point + a normal — a face patch | another `planar` |
| `cylindrical` | a point + an axis — a shaft or hole axis | another `cylindrical` |
| `point` | just a point | `point` or any |

### Mate

A **mate** pairs two connectors with a type:

```jsonc
{
  "parent_connector": "servo_1.shaft_out",
  "child_connector":  "coupler_2.shaft_hole",
  "type":             "concentric",
  "offset_mm":        0,     // axial slide for cylindrical, optional
  "rotation_rad":     0      // axial spin, optional
}
```

Three mate types cover the same >90%:

| Mate | DOF remaining | Use case |
|---|---|---|
| `fastened` | 0 | Default "weld" — same as today's `attach_face: fixed` |
| `planar` | 3 (2 in-plane + 1 spin) — pinned by `offset_uv` and `rotation` params | Face-to-face with optional xy offset |
| `concentric` | 2 (axial slide + spin) — pinned by `offset` and `rotation` params | Shaft-in-hole, wheel-on-axle |

### Resolver — closed form, no iteration

```
// Given parent world transform T_p, parent connector local pose C_p,
// child connector local pose C_c, and desired mate:
child_world = T_p · C_p · Mate(type, params) · inverse(C_c)
```

No Newton, no DOF graph, no dependency on an external solver. The math is matrix multiplication — this is *why* the three-type vocabulary was chosen.

## Worked example — servo → coupler

### Preset JSON additions

```jsonc
// actuator_servo_high_torque preset — add:
"connectors": [
  { "id": "shaft_out",    "origin_xyz_mm": [0, 0,  13], "axis_xyz": [0, 0, 1], "type": "cylindrical", "diameter_mm": 6 },
  { "id": "bracket_mount","origin_xyz_mm": [0, 0, -13], "axis_xyz": [0, 0,-1], "type": "planar" }
]

// structural_servo_coupler_disc preset — add:
"connectors": [
  { "id": "shaft_hole",   "origin_xyz_mm": [0, 0,  4],  "axis_xyz": [0, 0, 1], "type": "cylindrical", "diameter_mm": 6 },
  { "id": "bottom_face",  "origin_xyz_mm": [0, 0, -4],  "axis_xyz": [0, 0,-1], "type": "planar" }
]
```

### LLM-emitted assembly component (new form)

```jsonc
{
  "link_name":      "coupler_1",
  "component_id":   "structural_servo_coupler_disc",
  "attach_to":      "servo_1",
  "mate_connector": "shaft_hole",       // the child connector
  "attach_connector":"shaft_out",        // the parent connector  (optional — inferred from attach_to's default)
  "mate_type":      "concentric",
  "joint_type":     "revolute",
  "joint_axis":     "z"
}
```

### LLM-emitted assembly component (legacy form — still works)

```jsonc
{
  "link_name":    "coupler_1",
  "component_id": "structural_servo_coupler_disc",
  "attach_to":    "servo_1",
  "attach_face":  "top",                 // maps to servo_1's default "top" connector + coupler_1's default "bottom" connector + mate_type=fastened
  "joint_type":   "fixed",
  "joint_axis":   "z"
}
```

Both produce valid assemblies. The new form fixes the concentric-mate seam at the visual layer and encodes the shaft-hole relationship structurally.

## Backward compatibility — load-bearing design choice

Every preset gets **6 auto-generated default connectors** on first load: `top`, `bottom`, `front`, `back`, `left`, `right`, `type: planar`, origin at face center + outward normal. These are derived from the preset bbox exactly like today's `attachmentNodes.ts` does.

Consequence: `attach_face: "top"` is literally equivalent to `attach_connector: "top"`, `mate_connector: "bottom"` (auto-inferred opposite), `mate_type: "fastened"`. Every existing assembly graph continues to resolve bit-identically.

Authored connectors in the preset JSON **override** defaults of the same name. A preset can keep the default `top` and add a new `shaft_out`.

## Files in scope

| File | What changes |
|---|---|
| `core/presets/*.json` | Optional `connectors` array per preset component |
| `src/src/mateConnectors.ts` | **NEW.** Connector/mate types, default generation, frame composition math |
| `src/src/mateCorpus.ts` | **NEW.** Fixture harness for resolver correctness |
| `src/src/urdfAssembly.ts` | `AssemblyComponent` adds optional `attach_connector`/`mate_connector`/`mate_type`. `_performPlacement` gains a connector path alongside the existing bbox path (feature-flagged) |
| `src/src/attachmentNodes.ts` | Default-connector generation lives here (extends current face-node logic) |
| `src/src/reconcileAlignment.ts` | Unchanged. Stays as permanent safety-net for any residual depth mismatches |
| `core/ai/claude_client.py` | System prompt teaches Claude when to emit the connector form (cylindrical mates, face-ambiguous parts) |
| `src/package.json` | New `test:mate-corpus` script |

## Phased checklist

### Phase 1 — foundation (pure, no engine swap yet) — WS6

1. Define `MateConnector`, `MateConstraint`, `ConnectorType`, `MateType` types in `mateConnectors.ts`.
2. `generateDefaultConnectors(bbox) → MateConnector[6]` — identical semantics to today's `defaultFaceNodesForBoxDims`.
3. `resolveMate(parentWorld, parentConn, childConn, mateType, params) → Matrix4` — closed-form composition for `fastened` / `planar` / `concentric`.
4. `mateCorpus.ts` — ≥10 fixtures: fastened equivalence with today's bbox path; planar with xy offset; concentric shaft-hole; concentric with axial offset; cylindrical joint DOF preservation; face-ambiguity on L-bracket mock; backward-compat parity test (same graph, old path vs. new path → same transform).

### Phase 2 — engine integration (feature-flagged) — WS6

5. Extend `AssemblyComponent` with optional `attach_connector`, `mate_connector`, `mate_type`.
6. `_performPlacement` gets a connector branch: if any of the new fields are set, resolve via connectors; otherwise fall through to existing bbox math.
7. Feature flag `USE_MATE_CONNECTORS` defaulting to on; per-preset opt-out for rollback.
8. `urdfGraphEquivalence.ts` round-trip for the new fields (lossy is OK — they can default to `attach_face` on reverse-parse).

### Phase 3 — author problem-child connectors — WS7+

One preset per focused commit. Each gets: preset JSON diff, one-line description in preset's `description` field, smoke-test render before/after screenshot:

9. `structural_bracket_l` — `plate_top`, `wall_inner`, `wall_outer` (three named planar connectors disambiguating the two distinct surfaces).
10. `actuator_servo_*` family — `shaft_out` (cylindrical) + `bracket_mount` (planar). Retires most of `ROTATION_OVERRIDES` entries for servos.
11. `structural_servo_coupler_disc` — `shaft_hole` (cylindrical) + `bottom_face` (planar).
12. `sensor_depth_camera_small` — `mount_back` (planar) + `optical_front` (point, with axis for aim direction).
13. `structural_torso_panel` — connectors authored against the *rendered* GLB centroid, fixing M1 without re-authoring the mesh.
14. `structural_bracket_auto_N` auto-insertion path — ensure auto-brackets get connectors copied from the parent's shaft port.

### Phase 4 — LLM integration — WS7+

15. Preset catalog sent to Claude includes `connectors` array per component.
16. System prompt addition: "When attaching to a servo shaft, use `mate_type: concentric` with `parent_connector: shaft_out`. When attaching to a face, `attach_face` is fine."
17. Few-shot examples in the prompt for the concentric form.
18. Gemini critic loop (per CADSmith research) gets structured measurement feedback: for each placed child, emit `{link, expected_face_distance, actual_face_distance, delta_mm}` alongside screenshots.

### Phase 5 — cleanup — WS8+

19. Audit `ROTATION_OVERRIDES` — each entry asks: "does the connector on this preset make this entry redundant?" Most should retire.
20. Decide Option C's fate. Lean: keep as permanent safety-net (it costs nothing when aligned, catches catalog-data drift).

## Test / done criteria

- **Empirical**: quadruped, wheeled rover, arm, dual-arm stationary all render without submerged electronics (Option C already), *and* without shaft seams at servo↔coupler, *and* without L-bracket rotation errors, *and* without camera-facing-wrong-direction.
- **Fixture**: `test:mate-corpus` ≥10 passing.
- **Backward compat**: every existing session URDF in `session_data/` round-trips to an assembly graph and resolves to bit-identical joint origins under the connector path (when no explicit connectors are authored on any referenced preset).
- **No regression**: `test:tool-dispatch` 17/17, `test:graph-preservation` 11/11, `test:validator` 19/19, `test:alignment-corpus` 6/6.
- **Perf**: connector resolution ≤0.5ms per component (closed form — this is free).

## Risks

- **LLM doesn't adopt the new surface**. Claude keeps emitting `attach_face` for cases where `mate_type: concentric` would be better. *Mitigate*: keep legacy path working indefinitely; add emission quality into Gemini's critic loop ("you could have used shaft_out here"); few-shot the prompt with concrete examples.
- **Preset authoring burden**. Asymmetric meshes (torso_panel) need manual measurement of connector origins. *Mitigate*: 5–10 problem-child presets total, not catalog-wide. One-time cost per preset.
- **Multi-child distribution breaks**. If 4 legs attach via `mate_connector: "bottom"`, they all resolve to the same world point. The current `_computeMultiChildOffsets` handles this via face-plane spreading. *Mitigate*: retain distribution logic as an orthogonal post-pass — "connectors give the anchor, distribution gives the grid within the anchor face." Requires care where the math composes.
- **Topology-naming problem** (flagged by CAD research — SolidWorks bleeds users on this for 25 years). If we rename `shaft_out` later, every saved assembly referencing it breaks. *Mitigate*: version the preset catalog, fail loudly on unknown connectors, never silently fall back to a guessed connector.
- **Cylindrical mate has a free axial DOF**. Concentric shaft-coupler leaves slide direction undetermined. *Mitigate*: default the axial offset to the child connector's origin (so by default the coupler's shaft_hole origin sits at the servo's shaft_out origin — flush), allow `offset_mm` param to override.

## Non-goals

- No full constraint solver. No Newton iteration. No DOF-graph decomposition. Closed-form composition only. *(We revisit SolveSpace WASM only if we hit real over-constrained cases, which the three-mate vocabulary shouldn't produce.)*
- No runtime B-Rep extraction. `opencascade.js` is 13MB gzipped — prohibitive for the browser viewport.
- No URDF format extensions. URDF stays dumb-pose-serialization. Richness lives in preset JSON and resolver.
- No LLM-emits-code (CadQuery/FeatureScript). Structured assembly graph stays.
- No cross-preset inference. If a preset doesn't declare `shaft_out`, no heuristic invents one.
- No full catalog rewrite. Phase 3 touches ~5–10 presets; the rest ride on default connectors.

## Session prompt — paste into a fresh Claude Code session on `main`

```
You are working on `main` in C:/Users/ahmad/Downloads/Vector. No worktree —
WS5 already merged, this is a single-branch cycle.

## Your task
Implement Phase 1 + Phase 2 of the mate-connector migration in
`docs/MATE_CONNECTOR_MIGRATION.md` (THIS FILE — read it fully first).
Do NOT do Phases 3–5 here. Those need per-preset domain judgment and
belong in follow-up sessions.

## Core objective
- MateConnector / MateConstraint types + schema
- Default-connector generation (6 face-center per preset, bbox-derived)
- Closed-form frame composition math (fastened / planar / concentric)
- Fixture harness ≥10 fixtures including legacy-parity tests
- Feature-flagged dual code path in `_performPlacement`
- Zero regression vs. WS5 output (same URDFs for quadruped/rover/arm)

## Context to read before coding
1. `docs/MATE_CONNECTOR_MIGRATION.md` — this file
2. `docs/ENGINE_ALIGNMENT_PLAN.md` — WS5 (shipped) for architectural continuity
3. `docs/ENGINE_ARCHITECTURE.md` — why not Options A or B
4. `CLAUDE.md` — project conventions, quality gates
5. `src/src/reconcileAlignment.ts` — WS5 pure module, stays untouched
6. `src/src/attachmentNodes.ts` — defaultFaceNodesForBoxDims is the mold for default-connector generation
7. `C:/Users/ahmad/.claude/projects/C--Users-ahmad-Downloads-Vector/memory/MEMORY.md` + linked entries as needed

## Quality gates (from CLAUDE.md)
- cd src && npx tsc --noEmit — clean (only pre-existing GTAOPass warning)
- test:tool-dispatch 17/17, test:graph-preservation 11/11, test:validator 19/19,
  test:alignment-corpus 6/6 — no regression
- test:mate-corpus ≥10 passing — new suite
- Python compile clean

## Smoke test
1. Build a quadruped, a wheeled rover, an arm. All three MUST render
   bit-identically to the WS5-merged main output — no explicit connectors
   authored yet, so every preset uses default face-center connectors, which
   are mathematically equivalent to the current bbox path.
2. Console: `[mate] default connectors: 6` per preset load.
3. Feature-flag toggle (USE_MATE_CONNECTORS=false) also produces identical
   output (the legacy path still works).

## Do NOT commit
edit → type-check → fixture → smoke test → wait for "commit."
```

## Related

- `docs/ENGINE_ALIGNMENT_PLAN.md` — predecessor (Option C, WS5, shipped)
- `docs/ENGINE_ARCHITECTURE.md` — the three-option analysis; this plan is "Option C + smarter poses," not a replacement for Option C
- `docs/IMPROVEMENT_PLAN.md` — master plan; update its Deferred Engine Work section to reference this file when WS6 completes
- `memory/project_known_issues.md` — P1 (L-bracket) and M2 (camera) can be marked resolved when Phase 3 lands
- AIDL paper (arxiv 2502.09819) — strongest academic precedent for LLM-emits-constraints + solver-resolves
- Onshape Mate Connectors — product-scale precedent for named frames
- Drake `MultibodyPlant` F/M frame pairs — robotics-specific precedent
- Rhoban `onshape-to-robot` — CAD→URDF prefix-naming convention prior art
