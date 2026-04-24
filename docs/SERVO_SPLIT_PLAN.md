# Servo Split Implementation Plan

## Problem

Today, a servo is modeled as a single URDF link. When the assembler attaches a servo between parent and child, it emits one revolute joint at the mating face — which causes:

1. **The whole servo body swings** on that joint, instead of being bolted fixed.
2. **The rotation axis is located at the mating face center**, not at the horn centerline (which is physically offset by ~`hz/2` from the body's mount face).
3. **A servo cannot simultaneously be bolted to a bracket on one face AND drive a child from its horn** — one link means one parent.
4. **`joint_axis` is a logical label, not a geometric fact** — a servo with `joint_axis: x` stays visually upright, even though a real servo with rotation about world X must have its horn physically pointing along world X.

## Goal

Model each servo as **two URDF links joined by an internal revolute joint**:

- `servo_N_body` — the housing, ears, mount surfaces. Carries ~90% of the mass.
- `servo_N_output` — the horn + output shaft. Carries ~10% of the mass.
- Internal joint: `type=revolute`, origin at horn center (roughly `[0, 0, hz*0.42]` in body-local frame), axis = body-local `[0, 0, 1]`. This is the actuated DOF.

External mates then become **fixed** joints targeting the appropriate sub-link:

- `attach_face=top` on a servo → fixed joint into `servo_N_output` (the horn).
- Any other face → fixed joint into `servo_N_body` (bracket/ear mount).

The AI-facing interface does not change. `component_id=actuator_servo_*`, `attach_to=servo_N`, `attach_face`, `joint_axis`, `attach_rpy` all stay. Only the URDF emitter, preset catalog, visual generator, load/save, and a handful of topology helpers are aware of the split.

## Current-state reference (files to read before starting)

- `src/src/attachmentNodes.ts:84-150` — port definitions; servos get `top=shaft`, `bottom=mount_face`.
- `src/src/urdfAssembly.ts:135-164` — `AssemblyJointType`, `normalizeJointType`, `axisNameToTuple`.
- `src/src/urdfAssembly.ts:303-352` — `reconcileJointAxis` (axis re-expression on drag).
- `src/src/urdfAssembly.ts:4200-4290` — the connection site where `resolveConnectionJoint` chooses joint type and port occupancy is tracked. Coupler-bridging exception at `:4211`.
- `src/src/urdfAssembly.ts:4369-4380` — per-component placement log + joint type override.
- `src/src/urdfAssembly.ts:2762-2785` — `addComponentCore` joint/link emission (single-link today).
- `src/src/richVisuals/generators/actuators.ts:32-144` — `generateServo` produces one `THREE.Group`.
- `src/src/topologyCorpus.ts` — training examples; many reference `actuator_servo_*` with `attach_face: top`.
- `core/ai/claude_client.py:430-510` — prompt text that teaches the AI about servo shafts and named connectors.
- `core/sim/urdf_to_mjcf.py:1139-1170` — MJCF converter emits a position actuator per revolute joint.
- `core/presets/generic_presets.json` and `src/public/generic_presets.json` — servo preset entries (`actuator_servo_micro`, `actuator_servo_standard`, `actuator_servo_high_torque`, `actuator_continuous_rotation_servo`).

## Design

### Preset schema additions

For each servo preset in `core/presets/generic_presets.json` and `src/public/generic_presets.json`, add:

```json
"split_link": {
  "body_mass_frac": 0.9,
  "output_origin_xyz": [0, 0, 0.0]   // filled per-preset; ≈ hz * 0.42
  "output_axis_xyz":   [0, 0, 1],
  "output_half_extents": [x, y, z],  // horn bounding box, body-local
  "output_visual_tag": "horn"        // see visual generator change
}
```

Existing fields (mass, bbox, connectors) stay as the **body** values. Only the horn is broken out.

### URDF emission

When the assembler is about to add a servo link, emit **two `<link>` elements and three `<joint>` elements**:

```
<link name="servo_1_body">     ... mass = total * body_mass_frac ...
<link name="servo_1_output">   ... mass = total * (1 - body_mass_frac) ...

<joint name="servo_1_mount"    type="fixed"    parent=PARENT        child=servo_1_body   origin=<mount placement>
<joint name="servo_1_joint"    type="revolute" parent=servo_1_body  child=servo_1_output origin=<output_origin_xyz> axis=<output_axis_xyz> limit=...>
```

Downstream children that the AI attached with `attach_face=top` route to `servo_1_output`; all other faces route to `servo_1_body`. Their joints are `fixed`.

Entry point: the single-link emission path today in `addComponentCore` (`urdfAssembly.ts:2762`) and the topology-add path around `:4369-4470`. Factor a `emitServoLinks(doc, preset, comp, parentLink, placement)` helper and call it from both sites when `preset.id.startsWith('actuator_servo')` or `preset.id.startsWith('actuator_continuous')`.

### Axis-to-body-pose solver + bracket auto-insertion

**This is the key piece and what makes the AI's existing `joint_axis` declaration still work.**

Today the AI writes `joint_axis: "x"` and the engine writes `<axis xyz="1 0 0"/>` on the joint. Under the split, the internal axis is baked as body-local `+Z`, so we must instead **rotate the servo body** so that its local `+Z` aligns with the AI's desired world axis.

A naive solver lands the body in a correct but visually awkward pose — e.g. a hip-abduction servo hanging sideways off the chassis. Real robots never look like this because real robots use **brackets** to absorb the rotation. The chassis bolts to a bracket, the bracket bolts to the servo, the bracket carries the 90° twist, and the servo sits naturally on the bracket's inner wall. Vector already has `structural_bracket_l` and `structural_bracket_u` in the catalog and auto-repair infrastructure that inserts them. We reuse that here.

Inputs:
- `parentFaceNormal` (world) — from `attach_face` on the parent.
- `desiredWorldAxis` (world) — from `joint_axis` + any `attach_rpy` from the AI.

Output: the servo body's `<origin rpy=...>` on the mount joint.

Constraints:
1. Body's mount-face normal = `-parentFaceNormal` (flat against parent).
2. Body's local `+Z` (horn direction) = `desiredWorldAxis`.

Solver:
```
// 1. Pick which body face is the mount face (usually "bottom" = local -Z,
//    but if desired horn axis matches parent face normal then mount must
//    be "side" instead — the body lies down).
const mountFaceLocal = pickMountFace(parentFaceNormal, desiredWorldAxis, preset)
// mountFaceLocal ∈ {±X, ±Y, -Z}; +Z reserved for horn.

// 2. Build rotation that maps (mountFaceLocal → -parentFaceNormal) AND
//    (localZ → desiredWorldAxis) simultaneously. Two orthogonal constraints
//    uniquely determine a rotation (cross product gives the third axis).
const bodyWorldQuat = solveTwoAxisRotation(
  { from: mountFaceLocal,       to: parentFaceNormal.clone().negate() },
  { from: new Vector3(0,0,1),   to: desiredWorldAxis },
)

// 3. Convert to RPY relative to the parent link frame.
const localQuat = parentWorldQuat.clone().invert().multiply(bodyWorldQuat)
const rpy = eulerFromQuat(localQuat, 'XYZ')
```

`solveTwoAxisRotation` uses the standard "align two vectors" trick: rotate first pair with `Quaternion.setFromUnitVectors`, then roll about the now-aligned first axis to satisfy the second pair.

If the two constraints are inconsistent (`mountFaceLocal` × desired horn axis not orthogonal enough — i.e. the AI asked for a configuration no rigid body can satisfy), fall back to prioritizing **horn axis**, mount face gets closest feasible. Log a warning for the topology validator to surface.

**Bracket auto-insertion.** After the solver returns a pose, measure the tilt from the servo's natural mount face (usually local `-Z` bolted against parent face normal). If tilt ≤ 45°, emit the servo directly — it fits naturally. If tilt > 45°, **do not tilt the servo**; instead insert a bracket between parent and servo:

```
Solver says body tilt = 90°
  → pick structural_bracket_l (90° bracket)
  → orient bracket so its inner wall faces the desired horn direction
  → emit:  parent --(fixed)--> bracket --(fixed)--> servo_body --(revolute)--> servo_output
  → servo_body pose relative to bracket is near-identity; bracket absorbs the rotation
```

Rules:
- Tilt 45–135° → `structural_bracket_l` (L-bracket, ~90° turn).
- Tilt 135–225° → `structural_bracket_u` (U-bracket, ~180° turn, horn faces back at parent).
- Tilt < 45° → direct mount, no bracket.
- Bracket orientation is deterministic from the target horn direction and parent face normal (same two-axis solver, applied to the bracket instead of the servo).
- Cutoff is per-preset (`split_link.bracket_tilt_threshold_deg`, default 45) so exotic servos that are designed to mount sideways can opt out.

This step is invisible to the AI — it still writes `{attach_to: chassis, attach_face: bottom, joint_axis: x}`. The emitter decides whether a bracket is needed based on geometry alone. The resulting graph has one extra link per non-Z-axis servo, but matches how the robot would actually be built in hardware.

Add one new validation rule in `topologyCorpus.ts` / `topologyValidation.ts`: soft-info (not warn) that tags which mates will get an auto-inserted bracket, so users browsing the preview can see where brackets will appear.

### Port system

In `attachmentNodes.ts:componentPortsForPreset`, tag each port with the sub-link it belongs to:

```ts
interface AttachmentNodeDef {
  // ...existing fields
  subLink?: 'body' | 'output'   // new; default 'body'
}
```

For a servo:
- `top` → `subLink: 'output'`, keep `cls: 'shaft'`.
- `bottom`, `x_plus`, `x_minus`, `y_plus`, `y_minus` → `subLink: 'body'`.

Port-occupancy map (`urdfAssembly.ts` around `:4225`) keys on the resolved URDF link name — since `body` and `output` are distinct links, occupancy separates naturally. The "single-use shaft" warning at `:4228` now triggers only on `servo_N_output`, which is the correct semantics.

### Auto-repair coupler

`urdfAssembly.ts:4211-4215` — the `isCouplingPair` exception for `structural_servo_coupler` can be **retired for servos** (the horn is already a proper fixed attachment, no bridging disc needed). Keep the coupler path for real shaft-bore mates on drivetrain hubs and motors without a pre-modeled horn.

In the topology graph-mutation passes that currently inject a coupler after a servo's revolute, early-return when the parent is a servo preset — the coupler is no longer needed.

### Rich visuals

In `src/src/richVisuals/generators/actuators.ts`, split `generateServo` into two THREE.Group builders:

```ts
export function generateServoBody(id, dims): THREE.Group   // housing, ears, mount holes, cable, vents, label
export function generateServoOutput(id, dims): THREE.Group // horn, bolt circle, center screw, output shaft
```

The URDF loader that maps link → scene node will call `generateServoBody` for `servo_N_body` and `generateServoOutput` for `servo_N_output`. When the internal revolute joint rotates in sim or live control, only the output group moves — matching reality.

Update the registration in `src/src/richVisuals/generators/index.ts` so the generator dispatch can produce per-sublink groups (inspect how it currently keys on component id; add a sublink suffix).

### Simulator / MJCF

`core/sim/urdf_to_mjcf.py:1139-1170` iterates URDF revolute joints and emits a position actuator per joint. After the split:
- Joints named `servo_N_joint` (internal revolute) remain actuated with the servo's effort/velocity limits.
- Joints named `servo_N_mount` (external fixed) emit no actuator.

No converter change is strictly required — it already filters on joint type — but verify that `claude_client.py:1158` gain-scaling reads the effort from the correct joint (it keys on joint name/effort, both of which now live on the internal revolute). Name the internal joint `{servo_link}_joint` to keep naming stable.

**DOF count is invariant.** Same number of actuated revolute joints as before; they just live inside the servo instead of between servo and parent.

**Inertia improvement.** Splitting mass so the horn is ~10% of total means the actuated sub-link has low rotational inertia, which is physically correct and should make PD tuning more realistic.

### Load / migrate existing URDFs

Old saves (`dog2.urdf`, `robotDog.urdf`, `test.urdf`, anything users have on disk) contain one-link servos. On URDF load:

1. Scan `<link>` elements whose name matches `actuator_servo_*_\d+` or `actuator_continuous_*_\d+`.
2. For each, synthesize a second `<link>` (`..._output`) and rewrite:
   - Original link → `..._body`, mass × `body_mass_frac`, keep housing visuals only.
   - New output link → mass × `(1 - body_mass_frac)`, horn visuals only.
   - Find the **child** revolute joint (where the old servo link was the parent) and **retype it to fixed**, re-parent it to `..._output`.
   - Insert a new internal revolute joint `..._joint` between `..._body` and `..._output` with the preset's baked origin + axis.
3. Body's `rpy` on its mount joint needs the axis-to-pose solver applied so the horn points along the (preserved) world rotation axis.

Put this migration behind a one-shot function `migrateLegacyServos(doc)` called from the URDF loader. After migration the document is in new form and round-trips cleanly.

### AI prompt + topology corpus

**No prompt changes required.** The AI continues to emit `{component_id: actuator_servo_*, attach_to: X, attach_face: top|bottom|..., joint_axis: x|y|z, attach_rpy: [...]}`. The topology corpus examples all remain valid.

Optional improvements (do last, may help AI pick better poses once visuals shift):
- Update a couple of prompt examples to show the physically-correct side-mounted servo for `joint_axis: y` arm/leg joints, so the AI's intuition matches new visuals.
- Add one new lint rule to `topologyValidation.ts` that warns when `joint_axis` is inconsistent with `attach_face`'s natural horn direction (e.g. `attach_face: top, joint_axis: x` — the body must tilt 90°).

## Implementation order

1. **Preset schema.** Add `split_link` block to the four servo presets in both `core/presets/generic_presets.json` and `src/public/generic_presets.json`. Fill `output_origin_xyz`, `output_half_extents`, `body_mass_frac = 0.9` for each.
2. **Axis-to-pose solver.** New module `src/src/servoPose.ts` with `solveServoBodyPose(parentFaceNormal, desiredWorldAxis, preset) → { rpy, mountFaceLocal }`. Unit-test against the four canonical cases: (top, z) = identity; (top, y) = 90° tilt; (bottom, x) = hang + roll 90°; (bottom, z) = hang.
3. **Visual split.** Refactor `generateServo` into `generateServoBody` + `generateServoOutput`. Update the generators/index dispatch to accept a `subLink` argument.
4. **Port tagging.** Add `subLink` field to `AttachmentNodeDef`. Tag servo ports.
5. **URDF emit path.** Add `emitServoLinks` helper. Call from `addComponentCore` (`urdfAssembly.ts:2762`) and from the topology-add loop (`:4369-4470`). Child routing: `attach_face=top` ⇒ child's joint parent = `servo_N_output`; else = `servo_N_body`.
5b. **Bracket auto-insertion.** When `solveServoBodyPose` returns tilt > threshold, route the mount through a bracket: `parent → bracket(fixed) → servo_body(fixed)`. Pick `structural_bracket_l` for ~90° rotations and `structural_bracket_u` for ~180°. Bracket orientation is solved by the same two-axis routine, with the bracket replacing the servo as the tilted element. Give each servo preset a `bracket_tilt_threshold_deg` field (default 45) so this is tunable.
6. **Retire coupler auto-insert for servos.** Remove the `isCouplingPair` exception for `structural_servo_coupler` at `:4211-4215`; remove any graph-mutation pass that inserts a coupler disc after a servo.
7. **Legacy migration.** `migrateLegacyServos(doc)` in the URDF loader. Test against `dog2.urdf`, `robotDog.urdf`, `test.urdf`.
8. **Sim smoke test.** Convert a migrated quadruped URDF to MJCF, confirm actuator count and joint names match expectations, run `robotdog_trot.py` equivalent.
9. **Topology lint (optional).** Add a soft-warn rule for inconsistent `(attach_face, joint_axis)` pairs.

## Acceptance checklist

- [ ] A freshly assembled quadruped has 12 actuated revolute joints (same as before), all named `*_joint`, all internal to their servos.
- [ ] Each hip-abduction servo sits upright on an auto-inserted L-bracket; the bracket carries the 90° rotation so the horn still aligns with world X. Servo body itself is not tilted past the `bracket_tilt_threshold_deg`.
- [ ] A servo with a bracket mate on `x_plus` and a limb on `top` produces both as `fixed` external joints — the bracket goes to `_body`, the limb goes to `_output`, and only the horn spins in sim.
- [ ] `dog2.urdf` loads, migrates, and visualizes with correct horn-aligned axes without any hand edit.
- [ ] MJCF actuator count and effort/velocity limits are unchanged vs. pre-split for the same robot.
- [ ] No changes to AI prompt required; existing `topologyCorpus.ts` examples still pass validation.

## Known risks

- **Visual churn.** With bracket auto-insertion enabled, servos stay upright and the visual shift is small — parents gain an L-bracket between them and each non-Z-axis servo. Still eyeball the arm demo and quadruped before merging; extra brackets change the chain length and may shift overall robot height slightly.
- **Link count inflation.** Every non-Z-axis servo gains a bracket link. For a 12-DOF quadruped that's up to 8 extra links (hips + knees). MJCF handles this fine; just be aware the link count roughly doubles on servo-heavy robots.
- **Roll ambiguity around the horn axis.** The two-axis solver leaves one DOF (rotation about the horn); resolve deterministically by picking the mount face that minimizes body tilt, or by honoring `orientation`/`attach_rpy` when supplied.
- **Horn-axis inconsistency edge cases.** Some AI outputs combine `attach_face` and `joint_axis` in geometrically forced ways. Solver must fall back gracefully (prioritize horn alignment) and the new lint rule should surface these.
- **Mass split tuning.** 90/10 is a guess; real servos are closer to 95/5. Pick a value and document it in the preset — don't hardcode in the emitter.
