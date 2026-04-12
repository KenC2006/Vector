# Simulation Pipeline Plan

## Goal
Connect the URDF editor to MuJoCo physics so designed robots can be simulated in real-time with the 3D viewport showing live joint states.

## Current Assets
- `core/sim/mujoco_adapter.py` — MuJoCo wrapper (load, step, reset, get_state, set_control, render)
- `core/sim/urdf_to_mjcf.py` — URDF→MJCF converter (defaults-only; does NOT yet read preset `sim_metadata`)
- `core/validation/validator.py` — URDF validation (schema, collisions, inertia, CoM)
- Tauri commands: `sim_load`, `sim_step`, `sim_reset`, `sim_set_control`, `sim_get_state`, `sim_render`
- Sim bar UI in viewport (play/pause/reset) — wired to a 60 Hz step loop in `main.ts`
- `updateRobotFromSimState` already maps MJCF joint names → `parsedRobot.joints` and applies `setFromAxisAngle`
- Component presets carry mass / inertia / joint limits / actuator specs in `sim_metadata`

## Status by Layer
| Layer | Status | Notes |
|-------|--------|-------|
| 1. URDF → Sim pipeline | **partial** | Validation gate ✅, staging file ✅, `sim_load` ✅. Actuator injection from presets ❌. |
| 2. Sim loop → viewport sync | **mostly done** | 60 Hz loop ✅, joint name map ✅, **revolute only** (no prismatic/free). |
| 3. Control interface | **not started** | No sliders, no gravity toggle, no presets, no script runner. |
| 4. Visualization extras | **not started** | Stretch. |

---

## Layer 1 — URDF → Sim Pipeline

Already done:
1. Pull URDF from editor
2. Per-link + backend validation, errors block, warnings continue
3. Write to staging file, call `sim_load`

**Remaining work:**
1. **Actuator injection from presets.** `urdf_to_mjcf.py` currently hardcodes `damping=0.1` and falls back to `ctrlrange ±10`. Read each joint's `sim_metadata` (effort, velocity, damping, armature, frictionloss, gear) and emit a matching `<motor>` or `<position>` actuator.
2. **Massless / degenerate inertia auto-fix.** MuJoCo refuses non-PD inertia tensors. Either reject in validation or substitute a small isotropic tensor with a logged warning (current converter sometimes silently emits zeros).
3. **Mesh collision approximation.** MJCF needs convex hulls for mesh collisions; URDF mesh tags should be converted (or replaced with the link's bounding primitive) by the converter.
4. **Free-floating base.** If the root link has no fixed parent and the user wants a mobile robot, emit a `<freejoint/>`. Add a per-robot toggle (default: floor-attached).
5. **Coordinate frame audit (post-rotation-plan).** ROTATION_PLAN Phase 1 moved the Z-up→Y-up wrap onto `worldGroup`. Confirm `updateRobotFromSimState` writes joint quats in URDF/Z-up space (it's inside `worldGroup`, so this should be correct — verify with a known asymmetric robot).

## Layer 2 — Sim Loop → Viewport Sync

Already done:
- 60 Hz `setInterval` calls `sim_step` + `sim_get_state`
- Joint name lookup via `parsedRobot.joints`
- Revolute readback via `setFromAxisAngle`
- Original poses cached on entry, restored on exit

**Remaining work:**
1. **Prismatic joints.** `updateRobotFromSimState` only writes `quaternion` — for prismatic joints it should translate the group along its axis by `position`.
2. **Decouple sim time from render time.** Currently 1 sim step per ~16 ms regardless of MuJoCo's `dt`. Read the model's `opt.timestep`, then run `floor((now − lastTick) / dt)` substeps per frame (capped) so playback is real-time and deterministic regardless of frame rate.
3. **Catch-up cap & overrun warning.** If substeps consistently saturate, surface a "sim falling behind" badge instead of silently drifting.
4. **Base body pose.** If the root link is free-floating, read its world pos/quat from state and apply to the robot's parent group, not just joint groups.
5. **Single-state IPC frame.** Today we round-trip `sim_step` then `sim_get_state` separately — combine into one RPC (`sim_step_and_get`) to halve IPC cost.
6. **Error overlay.** When MuJoCo throws mid-step (NaN explosion, contact blowup), pause the loop and show the error in the sim bar instead of spamming the console.

## Layer 3 — Control Interface

1. **Joint slider panel** (sidebar, visible only while sim active). One slider per actuated joint, range from URDF limits, sends `sim_set_control` on input. For position actuators this is direct; for torque motors slider becomes a setpoint and a small P controller bridges.
2. **Gravity toggle.** Add `sim_set_gravity` Tauri command → adapter setter that mutates `model.opt.gravity`. Checkbox in sim bar.
3. **Reset-to-pose menu.** Three reset modes: (a) MJCF home, (b) editor pose at sim entry, (c) saved keyframe.
4. **Keyframe capture.** Button to snapshot current `qpos`/`qvel` to a named keyframe; reset menu can pick from these. Keyframes persist in project metadata, not the URDF.
5. **Python script runner.** Mount a Monaco tab where the user writes a callback `def step(t, state) -> controls`. Backend evaluates it each tick in a sandboxed namespace. Errors surface in sim bar, not the editor.
6. **Preset controllers (stretch).** Quadruped stand/trot, arm IK target, mobile-base WASD. Out of scope until script runner works.

## Layer 4 — Visualization (stretch)

- Contact-force arrows on `parsedRobot` links from `data.contact`
- CoM trail (decaying line)
- Joint torque heatmap (color-tint link materials by `data.qfrc_actuator`)
- Self-collision toggle + offending pair highlight
- MuJoCo offscreen camera PiP via `sim_render`

---

## Cross-cutting Concerns (added in revision)

These were absent from the original plan and bite if left to last:

1. **Determinism.** Expose a seed knob; record (model hash, seed, control trace) per session so a bug is reproducible.
2. **Hot reload.** If the user edits the URDF while sim is active, do we (a) auto-pause, (b) auto-rebuild and resume from `qpos`, or (c) ignore? Pick **a** as default with a "rebuild" button.
3. **Sim ↔ build interlock.** Carry-mode placement, gizmo edits, and inspector writes must be disabled while sim is active — they'd race the readback and silently drop changes on exit.
4. **Camera follow.** Optional "track base link" camera mode for mobile robots; otherwise the robot drives off-screen.
5. **Floor / world.** Currently no explicit ground plane in the converter — confirm `urdf_to_mjcf.py` adds a default `<geom type="plane"/>` or expose a toggle.
6. **Performance budget.** With JSON-RPC over stdio, target ≤4 ms per `sim_step_and_get`. If we exceed it on a 30-link robot, move to a length-prefixed binary frame.
7. **Resource cleanup.** `lastSimStagingPath` deleted on shutdown — also clean up on app close + on `sim_load` failure (already partly done; audit).

---

## Implementation Order (revised)

Phase A — close Layer 1/2 gaps ✅ DONE:
1. ✅ Actuator injection reads `<dynamics>` damping/friction from URDF joints
2. ✅ Inertia auto-fix: massless links with collision geometry get 10g default
3. ✅ Prismatic joint readback in `updateRobotFromSimState`
4. ✅ Fixed-timestep substep loop with catch-up cap (20 substeps/frame max)
5. ✅ `sim_step` now returns state JSON directly (single RPC per frame)
6. ✅ Sim ↔ build interlock: carry mode blocked while sim active
7. ✅ Mid-sim error overlay in sim bar
   - Also added: ground plane in MJCF, ctrlrange from effort limits, `sim_load` returns model info JSON

Phase B — Layer 3 essentials ✅ DONE:
8. ✅ Joint slider panel in `panel-sim` sidebar: position display + torque sliders → `sim_set_control`
9. ✅ `sim_set_gravity` + checkbox toggle (full gravity / zero-G)
10. ✅ Reset-to-pose: "Home" (MuJoCo reset) + "Editor Pose" (restore URDF initial positions)
11. ✅ Keyframe capture: timestamp-named snapshots stored in `localStorage` per file path; load/delete UI
12. ✅ Free-floating base: `free_base` toggle → `<freejoint/>` in converter; base body pose readback applied to robot group

<!-- NEXT: Phase C -->
Phase C — power user (NEXT TO IMPLEMENT):
13. Python script runner (Monaco tab, `def step(t, state) -> controls`, sandboxed eval each tick)
14. Camera follow (track base link for mobile robots)
15. Determinism seed + control trace logging

Phase D — Layer 4 viz polish, in any order driven by demand.

## Key Risks (revised)
- **Converter quality** is still the largest unknown — actuator injection and mesh→hull are the work that decides whether real robots load at all.
- **IPC overhead** vs 60 Hz: combined-RPC + substep loop is the cheap fix; binary protocol is the expensive fallback.
- **Frame conventions** post-rotation-plan: trust but verify against an asymmetric test robot before declaring Layer 2 done.
- **Race conditions** between editor edits and sim loop: solved by the build/sim interlock, not by retry logic.
