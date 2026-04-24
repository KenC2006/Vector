# Servo Split — Implementation Steps

Companion to `SERVO_SPLIT_PLAN.md` (design). This doc is the execution checklist: ordered tasks, exact files to touch, what to verify after each step, and what to hand off next.

Work top-to-bottom. Each step is independently committable; downstream steps assume upstream steps landed.

### Worktree setup (for implementors)
Run this once to get an isolated working copy on the `servo_split` branch:
```bash
git worktree add ~/Downloads/vector-servo-split -b servo_split origin/axis_fixation
cd ~/Downloads/vector-servo-split
```
Work entirely inside `~/Downloads/vector-servo-split`. The main repo is unaffected. When done, open a PR from `servo_split` → `axis_fixation`.

---

## Phase 0 — Prep ✅

### Step 0.1: Branch + snapshot ✅
- Staying on `axis_fixation` branch (no separate branch needed per user decision).
- `dog2.urdf` copied to `fixtures/legacy_servos/dog2.urdf` (only valid reference fixture — `robotDog.urdf` and `test.urdf` were broken and have been deleted).

### Step 0.2: Audit servo preset entries ✅
- Both preset files (`core/presets/generic_presets.json`, `src/public/generic_presets.json`) are **identical** — no sync issue.
- Five servo presets found (plan listed four; `actuator_servo_heavy_duty` exists alongside `actuator_servo_high_torque` and needs `split_link` too):

| Preset | bbox [w, d, h] mm | mass_kg | hornR mm | hornH mm | output_origin_z mm |
|---|---|---|---|---|---|
| actuator_servo_micro | [23, 12.2, 29] | 0.009 | 3.66 | 2.03 | 12.18 |
| actuator_servo_standard | [40, 20, 37] | 0.055 | 6.00 | 2.59 | 15.54 |
| actuator_servo_high_torque | [46.5, 36, 34] | 0.165 | 10.80 | 2.38 | 14.28 |
| actuator_servo_heavy_duty | [54, 42, 54] | 0.350 | 12.60 | 3.78 | 22.68 |
| actuator_continuous_rotation_servo | [40, 20, 37] | 0.055 | 6.00 | 2.59 | 15.54 |

Geometry formulas from `actuators.ts:66-92`: `hornR = min(w,d)*0.3`, `hornH = h*0.07`, horn center at `h*0.42` (THREE Y-up maps to URDF Z as `[0, 0, h*0.42]`).

**Exit criteria:** ✅ both preset files open cleanly, dimensions table recorded above.

---

## Phase 1 — Schema + data (no code yet)

### Step 1.1: Add `split_link` block to each servo preset
For each of the four servo presets, in **both** `core/presets/generic_presets.json` and `src/public/generic_presets.json`, add:

```json
"split_link": {
  "body_mass_frac": 0.9,
  "output_origin_xyz_mm": [0, 0, HZ_MM * 0.42],
  "output_axis_xyz": [0, 0, 1],
  "output_half_extents_mm": [HORN_R_MM, HORN_R_MM, HORN_H_MM],
  "bracket_tilt_threshold_deg": 45
}
```

Fill `HZ_MM`, `HORN_R_MM`, `HORN_H_MM` from the dimensions table you built in 0.2. Use the same geometry formulas as `generateServo` in `actuators.ts:66-92` (`hornR = min(w,d)*0.3`, `hornH = h*0.07`, shaft offset ≈ `h*0.48`).

### Step 1.2: Extend the preset type
- `src/src/urdfAssembly.ts:131+` defines `PresetData` / `PresetCategory`. Follow the type chain to `PresetComponent` and add the optional field:

```ts
interface SplitLink {
  body_mass_frac: number
  output_origin_xyz_mm: [number, number, number]
  output_axis_xyz: [number, number, number]
  output_half_extents_mm: [number, number, number]
  bracket_tilt_threshold_deg: number
}

// on PresetComponent:
split_link?: SplitLink
```

- Mirror in any Python-side preset schema (`core/ai/catalog_selector.py`, `core/ai/critique_classifier.py`) only if they parse the field — skim first; if they just pass-through, leave alone.

**Exit criteria:** `npm run typecheck` (or whatever your TS check command is) passes. No runtime behavior change yet.

---

## Phase 2 — Pure logic module (no integration)

### Step 2.1: Create `src/src/servoPose.ts`
New file. Implements the axis-to-body-pose solver as a pure function (no THREE scene dependencies, just math). Signature:

```ts
import * as THREE from 'three'

export interface SolveInput {
  parentFaceNormalWorld: THREE.Vector3   // unit
  desiredHornAxisWorld:  THREE.Vector3   // unit
  parentWorldQuat:       THREE.Quaternion
  preset: { split_link: SplitLink }
}

export interface SolveResult {
  bodyLocalRpy: [number, number, number]   // applied to body mount joint
  bodyTiltDeg:  number                     // angle between body's -Z (natural mount) and parentFaceNormal (negated)
  mountFaceLocal: 'bottom' | 'x_plus' | 'x_minus' | 'y_plus' | 'y_minus'
  feasible: boolean                        // false if solver fell back to horn-priority
}

export function solveServoBodyPose(input: SolveInput): SolveResult
```

Implementation:
1. `pickMountFace`: prefer `-Z` (bottom). If `desiredHornAxisWorld ≈ parentFaceNormalWorld` (would put horn INTO parent), pick a side face.
2. `solveTwoAxisRotation({from: mountFaceLocal, to: -parentFaceNormalWorld}, {from: +Z, to: desiredHornAxisWorld})`:
   - First rotation: `Quaternion.setFromUnitVectors(mountFaceLocal, -parentFaceNormalWorld)`.
   - Apply that to local `+Z`, then compute roll about `-parentFaceNormalWorld` to align rotated `+Z` with `desiredHornAxisWorld`.
3. `bodyTiltDeg = angleBetween(localMinusZ_afterRotation, -parentFaceNormalWorld)` in degrees.
4. Convert world quat → parent-local → RPY (XYZ euler).
5. If the two constraints cannot both be satisfied (e.g. user forced the impossible), prioritize horn alignment and set `feasible=false`.

### Step 2.2: Unit tests for the solver
Create `src/src/servoPose.test.ts` (or co-locate). Canonical cases:

| Case                                    | parentFaceNormal | desiredHornAxis | Expected tilt | Expected mountFace |
|-----------------------------------------|------------------|-----------------|---------------|---------------------|
| Stacked up: `(top, z)`                  | `(0,0,1)`        | `(0,0,1)`       | 0°            | `bottom`            |
| Stacked up, Y-axis: `(top, y)`          | `(0,0,1)`        | `(0,1,0)`       | 90°           | `bottom`            |
| Hip abduction: `(bottom, x)`            | `(0,0,-1)`       | `(1,0,0)`       | 90°           | `bottom`            |
| Dangling straight: `(bottom, z)`        | `(0,0,-1)`       | `(0,0,-1)`      | 0°            | `bottom`            |
| Horn into parent (impossible direct)    | `(0,0,1)`        | `(0,0,-1)`      | any           | side face           |

Each test: call `solveServoBodyPose`, construct the resulting world quat from the returned rpy + identity parentWorldQuat, verify that applying it to `(0,0,1)` gives `desiredHornAxis` within 1e-6 and that the negated mountFace axis gives `parentFaceNormal`.

**Exit criteria:** all five cases green. Solver is standalone — the rest of the codebase has not been touched yet.

---

## Phase 3 — Visual split

### Step 3.1: Split `generateServo`
In `src/src/richVisuals/generators/actuators.ts:32-144`:

- Keep `generateServo` as a thin wrapper for backwards compatibility (it composes the two below into one group). Useful for the interim while step 5 is incomplete.
- Add:

```ts
export function generateServoBody(id: string, dims: GeneratorDims): THREE.Group {
  // everything EXCEPT: horn, hornBolts, center screw, output shaft, bearing ring
  // i.e. housing + ears + ear holes + cable + strain relief + label + ribs + vents
}

export function generateServoOutput(id: string, dims: GeneratorDims): THREE.Group {
  // horn (nurbsServoHorn) + hornBolts (boltCircle) + center screw + output shaft + bearing ring
  // positions relative to the horn's center, NOT the body's center
  // so when the output group is attached to the output link, its origin is the horn center
}
```

Critical: the output group is authored in the **output link's local frame**, so its geometric origin is the horn. That's the frame the internal revolute joint spins.

### Step 3.2: Plumb sublink through the generator dispatch
- `src/src/richVisuals/generators/index.ts` — inspect how component id maps to generator. Add a `subLink?: 'body' | 'output'` arg. When the preset has `split_link` and `subLink === 'output'`, return `generateServoOutput`; otherwise `generateServoBody`.
- Find the call site(s) in the URDF-to-scene loader (grep for generator dispatch usage). Update to pass `subLink` based on link name suffix (`_body` / `_output`).

**Exit criteria:** with a hand-authored URDF containing two servo sublinks, the viewport shows body and output as separate visual groups. No other flow is using sublinks yet — that's next.

---

## Phase 4 — Ports

### Step 4.1: Add `subLink` to `AttachmentNodeDef`
- `src/src/attachmentNodes.ts` near the interface definition (search for `AttachmentNodeDef`). Add optional field:
  ```ts
  subLink?: 'body' | 'output'
  ```

### Step 4.2: Tag servo ports
- In `componentPortsForPreset` (`attachmentNodes.ts:121-127`), when setting up servo ports:
  - `top` → `subLink: 'output'`
  - `bottom`, `x_plus`, `x_minus`, `y_plus`, `y_minus` → `subLink: 'body'`
- Port occupancy keying (`urdfAssembly.ts` ~line 4225) should now key on the **resolved sublink link name**, not the servo's AI-declared `link_name`. Add a helper `resolveSublinkName(compLinkName, port)` that returns `compLinkName + '_' + (port.subLink ?? 'body')`.

**Exit criteria:** port validation still passes for all topologyCorpus examples. No emission change yet.

---

## Phase 5 — URDF emitter

### Step 5.1: Write `emitServoLinks`
New function in `src/src/urdfAssembly.ts`, near `addComponentCore` (~line 2762). Signature:

```ts
function emitServoLinks(
  doc: Document,
  preset: PresetComponent,
  comp: AssemblyComponent,
  parentLinkName: string,
  mountPlacement: { xyz: [number,number,number], rpy: [number,number,number] },
  opts: { insertBracket?: BracketSpec }
): { bodyLinkName: string, outputLinkName: string }
```

Emits:
- `<link name="${comp.link_name}_body">` with mass × `body_mass_frac`, body visuals, body collision.
- `<link name="${comp.link_name}_output">` with mass × `(1 - body_mass_frac)`, output visuals, output collision.
- `<joint name="${comp.link_name}_mount" type="fixed">` parent=`parentLinkName`, child=`_body`, origin=`mountPlacement`.
- `<joint name="${comp.link_name}_joint" type="revolute">` parent=`_body`, child=`_output`, origin=`output_origin_xyz_mm / 1000`, axis=`output_axis_xyz`, limit from preset effort/velocity.

If `opts.insertBracket` is set, skip the direct `_mount` joint and instead emit bracket-first (Step 5.2).

### Step 5.2: Integrate the solver and bracket decision
Where the single-link path currently emits a servo (two call sites: `addComponentCore` ~2762 and the topology-add loop ~4369–4470):

```ts
if (preset.split_link) {
  const solved = solveServoBodyPose({
    parentFaceNormalWorld: ...,     // from attach_face
    desiredHornAxisWorld: ...,      // from joint_axis + attach_rpy
    parentWorldQuat: ...,           // from parent link's world transform
    preset,
  })

  const needsBracket = solved.bodyTiltDeg > preset.split_link.bracket_tilt_threshold_deg

  if (needsBracket) {
    const bracketSpec = pickBracket(solved.bodyTiltDeg, solved)  // 45–135 → L, 135–225 → U
    emitBracketLink(doc, bracketSpec, parentLinkName, comp, mountPlacement)
    // bracket becomes new parent; servo body mount quat is identity
    emitServoLinks(doc, preset, comp, bracketSpec.linkName, identityPlacement, {})
  } else {
    const mountPlacement = applyBodyRpy(mountPlacement, solved.bodyLocalRpy)
    emitServoLinks(doc, preset, comp, parentLinkName, mountPlacement, {})
  }
} else {
  // existing single-link path (motors, etc)
}
```

### Step 5.3: Child routing
In the topology-add loop (`~4369-4470`), when resolving a child's parent link name:
- If the parent is a servo with `split_link` and `attach_face === 'top'`: set the child's parent link to `${servoLinkName}_output`.
- Otherwise: set to `${servoLinkName}_body`.
- Force `joint_type = 'fixed'` for any external mate to a split-link servo (the DOF is already internal). Log when overriding the AI's declared joint_type so it's traceable.

### Step 5.4: Write `emitBracketLink` + `pickBracket`
- `pickBracket(tiltDeg, solveResult)`:
  - 45–135° → `structural_bracket_l`
  - 135–225° → `structural_bracket_u`
  - Otherwise fall back to direct mount (shouldn't reach here since Step 5.2 checks threshold first).
- `emitBracketLink` inserts a bracket link with `plate_top` (or `wall_outer`) bolted to parent, and orients it so `wall_inner` faces the desired horn direction. Use the same `solveTwoAxisRotation` from `servoPose.ts` but applied to the bracket's own frame. Use a synthetic link name: `${servoLinkName}_mount_bracket`.

**Exit criteria:**
- A fresh assembly of a stacked vertical arm (all Z-axis joints) produces servos with **no** brackets, bodies upright, internal revolute joints named `${servoLinkName}_joint`.
- A fresh assembly of a quadruped produces L-brackets under each hip-abduction servo and each knee pitch servo. Horns align with world X/Y as appropriate.
- `npm run dev` loads the viewport without errors.

---

## Phase 6 — Retire servo-specific coupler path

### Step 6.1: Remove coupler exception for servos
- `src/src/urdfAssembly.ts:4211-4215` — delete the `isCouplingPair` branch that targets `structural_servo_coupler`. Keep any branches that handle drivetrain/motor shaft→bore (those still need the coupler because they don't have split_link).
- Search the repo for topology graph-mutation passes that explicitly inject `structural_servo_coupler_disc` after a servo (grep `structural_servo_coupler`). Add an early-return when the parent preset has `split_link`.

### Step 6.2: Update topologyCorpus expectations
- `src/src/topologyCorpus.ts` — examples that previously asserted a coupler was auto-inserted after a servo-shaft mate (search for `structural_servo_coupler` in test expectations) need updating: the split-link servo no longer needs a coupler. Update the expected topology graphs accordingly. Do NOT change the input topology — that's the AI's output and it stays the same. Only the expected post-processing output changes.

**Exit criteria:** `topologyCorpus` tests pass. Inspect one failing case pre-update to confirm the diff is "coupler removed" rather than "something else broke."

---

## Phase 7 — Legacy migration

### Step 7.1: Write `migrateLegacyServos`
New function in the URDF loader (find the loader by grepping for where `DOMParser` parses incoming URDFs into the scene). Signature:

```ts
function migrateLegacyServos(doc: Document, presets: PresetData): { migrated: number }
```

For each `<link>` whose name matches `/^(actuator_servo_\w+|actuator_continuous_\w+)_\d+$/`:
1. Look up the preset; if no `split_link`, skip (not a migratable servo).
2. Rename the link to `${original}_body`. Remove horn-only visuals/collisions (you'll need a visual tag or a geometric rule — the easiest is to regenerate both visual groups from the preset, discarding the old merged visuals).
3. Scale mass by `body_mass_frac`.
4. Create a new `<link name="${original}_output">` with output mass and output visuals.
5. Find every `<joint>` whose `<child link="${original}"/>` — rewrite its child to `${original}_body`.
6. Find every `<joint>` whose `<parent link="${original}"/>` — determine which face it mated on. If `top`-ish (axis along original's +Z and origin near top face), rewrite `<parent>` to `${original}_output` and `<type>` to `fixed`; axis element can be removed. Otherwise rewrite `<parent>` to `${original}_body` and `<type>` to `fixed`.
7. Insert the new internal `<joint name="${original}_joint" type="revolute">` between `_body` and `_output` with preset-baked origin and axis; carry over effort/velocity limits from whichever legacy joint was actuated.
8. **Run the bracket pipeline on the body's mount joint** (the one whose child is now `_body`): if the solver reports tilt > threshold, insert a bracket there. Matches fresh-assembly behavior so migrated files look like new ones.

### Step 7.2: Wire migration into load
- Call `migrateLegacyServos(doc)` right after URDF parse, before scene construction. Log `[servo_migrate] converted N legacy servos`.
- Add an idempotency check: if a link named `${X}_body` already exists, skip — the file is already in new format.

### Step 7.3: Fixture tests
Load `fixtures/legacy_servos/dog2.urdf`. After migration:
- Count revolute joints matches pre-migration count (DOF preserved).
- All revolute joints are named `*_joint` and their parent links end in `_body`.
- Bracket links were inserted where expected (hip abduction, knee pitch in the dog).

**Exit criteria:** fixture migrates cleanly, visual diff vs. pre-migration shows brackets inserted but overall robot silhouette similar.

---

## Phase 8 — Simulator verification

### Step 8.1: MJCF conversion smoke test
- Convert a migrated quadruped URDF via `core/sim/urdf_to_mjcf.py`.
- Check: actuator count matches number of `*_joint` revolute joints (same count as pre-split).
- Check: actuator names follow the new `${servo}_joint` convention. Anything in `claude_client.py:1158` that reads joint names by pattern needs to match the new names.
- Check: effort/velocity limits on the internal revolute joint were carried over from the legacy revolute in Step 7.1.6.

### Step 8.2: Runtime sim check
- Run `robotdog_trot.py` (or its current equivalent) against the migrated quadruped.
- Expected: same gait behavior as before the split. PD response may be slightly different because inertia is now concentrated in the body link rather than distributed — retune gains if the trot is visibly worse. Document any gain changes.

**Exit criteria:** MJCF file generates without errors, trot script produces qualitatively similar behavior, any gain retuning is noted in the commit message.

---

## Phase 9 — Polish (optional, do last)

### Step 9.1: Topology lint
- `src/src/topologyValidation.ts` — add a soft-info rule that flags `(attach_face, joint_axis)` combinations that will trigger a bracket insertion, so users previewing topology in the UI see a "bracket will be auto-inserted" hint on those mates.

### Step 9.2: Prompt nudge (only if AI behavior looks off)
- Skim `claude_client.py:430-510` for examples that directly contradict new visuals. Most should stay as-is. If the AI is now generating robots that look worse (e.g. requesting unnecessary brackets by picking weird axes), add one example in the prompt showing the preferred face+axis pattern for arm and leg joints.

### Step 9.3: Rip out deprecated single-link servo path (cleanup)
Once confident the split path is stable and legacy migration is reliable, delete the single-link code branch in `addComponentCore` and the wrapper `generateServo`. This is irreversible — do it only after a week of daily use without issues.

---

## Rollback plan

Each phase lands as its own commit. If a later phase fails in a way that can't be fixed quickly:
- Phases 1–4 are no-ops at runtime (schema, pure logic, visual helpers, port metadata) — safe to keep.
- Phase 5 is the behavior change. Revert that commit to restore the old emitter. Brackets disappear, servos go back to single-link on new assemblies.
- Phase 7 migration can be gated behind a feature flag for a release cycle: `if (FLAGS.servo_split) migrateLegacyServos(doc)`. Leave the flag in through Phase 8 at least.

## Dependencies between steps

```
0 → 1 → 2 ─┐
          ├→ 5 → 6 → 7 → 8 → 9
      3 ──┤
      4 ──┘
```

Steps 2, 3, 4 can proceed in parallel once Phase 1 lands. Step 5 needs all three. Steps 6, 7, 8, 9 are strictly sequential after 5.

## Open questions to resolve during Phase 2

- Exact `output_origin_xyz` per servo preset — measure from `actuators.ts` geometry formulas or eyeball in the viewport with a debug overlay?
- Mass split: 90/10 default is a guess. If you have a vendor spec sheet for any of the four servos (horn + shaft vs. body), use that for that preset. Otherwise commit to 90/10 and note in the preset.
- Bracket choice at exactly 135°: falls between L and U — pick L as default and document.
