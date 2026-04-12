# Rotation Implementation Plan

End-to-end plan to make rotations first-class across placement, serialization, and simulation.

## Goals

1. Rotations are authoritative as **quaternions** in-memory; URDF `rpy` is a serialization concern only.
2. Round-trip (parse → edit → save → reparse) is bit-stable within float tolerance.
3. Carry/placement mode supports manual rotation with snap, in addition to mount-driven auto-orientation.
4. Joint axes survive rotation of their parent link (already partly working — formalize and extend to fixed joints).
5. Sim layer (MuJoCo) sees the same rotations the user authored — no Y-up visual hack leaking into export.

## Current State (summary)

- Internal: `THREE.Quaternion` on pivot groups (`urdfParser.ts:374`).
- Write path: quat → Euler **XYZ** → `rpy` (`urdfAssembly.ts:1677,1701`).
- Read path: `rpy` → Euler **ZYX** → quat (`urdfParser.ts:330,380`). **Round-trip bug.**
- Visual hack: global `group.rotation.x = -π/2` to convert URDF Z-up to Three.js Y-up (`main.ts:1044`). Not persisted, breaks re-import.
- Snap commit reconciles joint axes for revolute/prismatic (`urdfAssembly.ts:2054`); fixed joints unhandled.
- Carry-mode manual rotation: keyboard nudges only (`urdfAssembly.ts:1478`); no gizmo, no snap-to-angle UI.
- Sim animation (`main.ts:3206`) uses `setFromAxisAngle` per joint — fine for 1-DOF, fine going forward.

## Phase 1 — Convention Lockdown (foundation, ~half day)

Cannot build on rotations until the math is consistent.

1. **Pick one Euler order for URDF I/O.** URDF spec is fixed-axis RPY = intrinsic XYZ ≡ extrinsic ZYX. Standardize on **`'ZYX'` extrinsic** in Three.js (matches reading path). Fix the writer in `urdfAssembly.ts:1677,1701`.
2. **Add a single helper module** `src/src/rotationIO.ts` exporting `quatToRpy(q)` and `rpyToQuat(rpy)`. Replace all inline Euler conversions with these. One place to audit.
3. **Round-trip test fixture**: tiny script that loads `sampleurdf/`, walks every joint quaternion, serializes, reparses, asserts `quat.angleTo(quat') < 1e-6`. Run on every save.
4. **Kill the Y-up hack.** Remove `main.ts:1044`. Instead, wrap the robot in a `worldGroup` whose only purpose is the Z-up→Y-up rotation **for the camera/scene**, not the robot data. Robot link transforms stay in URDF (Z-up) space. This is the single most important fix — without it, exports drift.

**Exit criteria**: load → save → diff produces zero changes on the sample URDFs.

## Phase 2 — Placement-Time Rotation (~1–2 days)

Make rotation a first-class authoring action in carry mode.

1. **Carry-mode rotation state**: extend the carry struct in `urdfAssembly.ts` with a `userQuat: Quaternion` applied *after* mount alignment. Mount snap computes the base orientation; user rotation composes on top.
2. **Hotkeys** (extend existing keyboard nudges at `urdfAssembly.ts:1478`):
   - `R` + drag: free rotate around the snap normal (most common case — spinning a part on its mount face).
   - `Shift+R`: snap to 15° increments.
   - `X / Y / Z`: constrain rotation axis (in the snap-target's local frame).
3. **Gizmo** (optional, Phase 2.5): Three.js `TransformControls` in rotate mode, anchored at the ghost's mount point. Only shown when carry is in "rotate" sub-mode.
4. **Ghost preview update**: the existing `Box3Helper` ghost (lines 196–224) already decomposes a world matrix — just feed it the composed quaternion. No structural change.
5. **Commit path**: `commitCarry()` (line 1655) already converts to local-relative — make sure the user rotation is folded into the local quat *before* the existing axis-reconciliation block at line 2054, not after.

**Exit criteria**: pick a part, snap to a face, press R, rotate, place — re-open the URDF and the rotation is preserved.

## Phase 3 — Joint Axis & Fixed-Joint Reconciliation (~half day)

The existing reconciliation at `urdfAssembly.ts:2054` is the right shape but incomplete.

1. **Extend to fixed joints**: even though a fixed joint has no DOF, its child frame's orientation must be expressed correctly. Currently the code skips axis recompute when `joint.type === 'fixed'`; it should still rewrite `<origin rpy>` from the composed quaternion. (Probably already does — verify.)
2. **Joint axis stays unit-length**: after `applyQuaternion`, renormalize. Float drift over many edits otherwise accumulates.
3. **Multi-axis future-proofing**: not in scope, but leave a TODO marker — if we ever add ball/floating joints, the single `<axis>` vector won't suffice; we'd need a per-DOF quaternion stack.

## Phase 4 — Inspector Editing (~half day)

Right now the inspector shows `xyz rpy` text fields (`urdfAssembly.ts:825`). Make them respect Phase 1's helpers.

1. Inspector edits write through `rpyToQuat` → set on pivot group → mark dirty. No direct Euler manipulation on the group.
2. Add a small "reset rotation" button per joint.
3. Display rotation in degrees in the UI, store in radians. Avoid the classic mistake of mixing.

## Phase 5 — Sim Layer Compatibility (~1 day, blocks SIM_PLAN.md Layer 1)

This is where Phase 1's hack-removal pays off.

1. **URDF→MJCF converter** (`scripts/urdf_to_mjcf.py` per SIM_PLAN.md) consumes URDF with correct Z-up rpy. No special-casing needed once Phase 1 is done.
2. **Sim → render readback**: MuJoCo gives joint angles (scalars for 1-DOF). The existing `setFromAxisAngle` path at `main.ts:3206` already handles this — confirm it still works after the worldGroup refactor.
3. **Origin transforms**: MuJoCo bodies inherit parent frame; verify that joint `origin rpy` from URDF maps to MJCF `<body pos quat>` and not to the joint element itself. This is a converter concern but worth flagging.
4. **Test**: load sample robot, run sim 1s, pause, compare visual joint angles to MuJoCo's `qpos` — should match within 1e-4 rad.

## Phase 6 — Snapping UX Polish (~half day, optional)

1. Expose snap-angle tolerance (currently hardcoded `π/4` at `urdfAssembly.ts:189`) as a setting.
2. When the user manually rotates in carry mode, *temporarily relax* angle snap so they can place at off-axis orientations without fighting the snap.
3. Visual indicator on the ghost: small arc showing the current rotation delta from the snap default.

## Risks & Open Questions

- **Hack removal blast radius**: removing `main.ts:1044` will visually break anything that assumed Y-up robot data. Need to grep for other consumers of `parsedRobot.group.rotation` before doing it. Do Phase 1.4 on a branch.
- **Existing saved URDFs**: anything saved under the old XYZ-write convention will reparse wrong after Phase 1.1. Acceptable since we're pre-release, but worth a one-shot migration script if there's anything we want to keep.
- **Gizmo vs hotkeys**: gizmos are nicer but `TransformControls` doesn't compose cleanly with our custom carry state. Hotkeys-first is the lower-risk path; gizmo can come later.
- **Multi-DOF joints**: out of scope. If they land, revisit Phase 3.

## Suggested Order

Phase 1 → Phase 3 → Phase 2 → Phase 4 → Phase 5 → Phase 6.

Phase 1 unblocks everything. Phase 3 is small and removes a known footgun before users start authoring rotations heavily in Phase 2. Phase 5 should wait until 1–4 are stable so the sim integration isn't chasing a moving target.
