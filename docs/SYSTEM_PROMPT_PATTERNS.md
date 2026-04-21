# System Prompt — Robot Pattern Coverage

Working doc for expanding the canonical-pattern section of `SYSTEM_PROMPT` in `core/ai/claude_client.py`. Every pattern Claude can't reproduce from first principles should have an explicit topology sketch in the prompt, otherwise the model drops limbs, inserts towers, or fans sensors off servo shafts.

## Tier list — why this order

The ordering reflects **frequency of real-world requests × topology complexity × how badly Claude fails without guidance**. Tier 1 and 2 are the request volume; Tier 3+ are the specialized / edge-case robots that users ask for occasionally but that still need to render plausibly or the product looks toy.

Each pattern in the prompt should specify: root preset, canonical chain, joint axes, face choices, rest-pose `attach_rpy` if applicable, and the one anti-pattern Claude most commonly emits.

---

## Tier 1 — already covered (baseline)

These are in `SYSTEM_PROMPT` today (lines ~429–470). Keep them. Revisit only if new palette items change the canonical chain.

| Pattern                      | Root                         | Distinguishing rule                                                             | Known-good |
| ---------------------------- | ---------------------------- | ------------------------------------------------------------------------------- | ---------- |
| Robotic arm (6-DOF)          | `structural_baseplate`       | Arm extrusions vertical + joints control angle; wrist cam on forearm, not servo | yes        |
| Wheeled rover (4-wheel)      | `structural_baseplate`       | Wheels direct on baseplate bottom, revolute y, no servos between                | yes        |
| 6-wheel rover (Mars rover)   | `structural_baseplate_large` | Backend distributes 2×3                                                         | yes        |
| 2-wheel (bike/unicycle)      | `structural_baseplate`       | Only if explicitly requested                                                    | yes        |
| Quadruped (canonical 12-DOF) | `structural_baseplate_large` | Anatomical joint order, compound hip, same-sign mirror                          | yes        |
| Quadruped (8-DOF simple)     | `structural_baseplate_large` | Drop abduction when user says "simple/cheap"                                    | yes        |
| Head / neck assembly         | baseplate (or torso)         | `neck_servo → head_bracket → camera` — keep it simple, one servo                | yes        |
| Tail                         | baseplate                    | `tail_servo(back, revolute z) → tail_extrusion(horizontal)`                     | yes        |
| Sensor mount (basic)         | any structural               | Never on servo shaft; elevation_angle on side faces                             | yes        |
| Rotated top sensor           | any structural               | `orientation="45"` for yaw on top face                                          | yes        |

---

## Tier 2 — next priorities (draft or needs canonical pattern)

These are common enough that "Claude winged it" isn't acceptable. All have prior design notes, none are landed.

### 1. Bipedal humanoid (17 DOF)

- **Root**: `structural_baseplate_large` as torso (no vertical-orient preset exists; accept the blocky look for now).
- **Chain**: head (top) + arms (left/right faces) + legs (bottom, 2-child symmetry).
- **Canonical**:
  ```
  torso (baseplate_large)
  ├── neck_servo (top, revolute y) → head_bracket → depth_camera
  ├── shoulder_L (left, revolute x) → upper_arm → elbow (y) → forearm → wrist (z) → gripper
  ├── shoulder_R (right, revolute x) → [mirror, same sign]
  ├── hip_abduction_L (bottom, revolute x) → hip_pitch (y) → thigh → knee (y) → shin → ankle_pitch (y) → foot
  └── hip_abduction_R (bottom, revolute x) → [mirror, same sign]
  ```
- **Anti-pattern to encode**: arms on top face; missing ankle; mirror-sign flip.
- **Gemini override**: "ignore tipping-risk / narrow-support critiques on bipeds."
- Source: `project_bipedal_humanoid_plan.md`.

### 2. Hexapod (6-leg insect / spider, 18 DOF)

- **Root**: `structural_baseplate_large`.
- **Chain**: 6× leg using quadruped's 3-DOF leg (hip_abduction → hip_pitch → thigh → knee → shin → foot), distributed 2×3 on bottom face.
- **Anti-pattern**: Claude will try to distribute 6 at corners (like 4 wheels) — need explicit "2 rows × 3" hint, or rely on backend auto-distribution that already handles 6.
- **Rest pose**: crouch rpy same as quadruped.

### 3. Differential-drive mobile robot (2 powered + caster)

- **Root**: `structural_baseplate`.
- **Chain**: 2× `mobility_wheel_driven` (bottom, left/right, revolute y) + 1× `mobility_caster_wheel` (bottom, back) + lidar on top + SBC.
- **Anti-pattern**: Claude emits 4 wheels by default. Need "differential drive = exactly 2 powered wheels + 1 caster."
- **Vocabulary**: "TurtleBot", "Roomba-style", "diff drive".

---

## Tier 3 — specialized but realistic

Users will ask for these. Not landing them today means a wrong-shape robot ships.

### 4. Swerve-drive platform (FRC / outdoor rover)

- **Root**: `structural_baseplate_large`.
- **Chain**: 4× `mobility_swerve_drive_module` (bottom corners) — each module is a wheel + steering unit.
- **Anti-pattern**: Claude emits servo → wheel stacks instead of the single swerve preset.

### 5. Pan-tilt sensor mount (standalone)

- **Root**: `structural_baseplate` (small).
- **Chain**: `pan_servo (top, revolute z) → tilt_bracket (top, fixed) → tilt_servo (top, revolute y) → camera (top, fixed)`.
- **Anti-pattern**: this is the one valid exception to "sensor on structural link, never on actuator"-style prompts. The current SENSOR_ON_ACTUATOR validator could false-positive here — flagged in `project_known_issues.md` D1. Need a prompt clause: "pan-tilt mounts = servo-servo-bracket-sensor is the correct pattern."

### 6. Self-balancing 2-wheel robot (Segway / inverted pendulum)

- **Root**: `structural_baseplate`.
- **Chain**: 2× wheel (bottom, left/right, revolute y) + tall vertical structural_extrusion + payload on top.
- **Anti-pattern**: Claude wants to add a caster for stability. Explicit "self-balancing = exactly 2 wheels, no caster; balance is a control problem, not a topology one."

### 7. Underwater ROV / aerial drone

- Palette gap: no thrusters / propellers. Document as **not supported** and suggest nearest analog (wheeled chassis + cameras) so the request doesn't silently fail.

---

## What to add to the prompt vs. handle in code

Prompt is probabilistic. Follow lever hierarchy from `project_system_prompt_improvements.md`:

1. If a pattern failure can be **detected deterministically** (e.g., hexapod with 6 legs but 4-corner distribution), put the fix in validator / auto-repair.
2. Only add to prompt what the validator cannot catch: vocabulary → preset mapping, rest-pose rpy, joint-order anatomy, face choice.
3. Every new pattern in the prompt should have a corresponding test-corpus fixture (`project_vector_improvement_plan.md` §1). Don't ship a pattern the prompt describes that no canonical fixture exercises.

## Template for a new pattern entry

```
<RobotName> (<approx DOF>, vocabulary: "<synonyms>"):

  <root_preset>
    -> <child>(<face>, <joint_type> <axis>[, attach_rpy=[r,p,y]])   — <purpose>
       -> ...

  Notes:
  - Root face for children: ...
  - Mirror rule: ...
  - Rest pose: ...
  - Anti-pattern: ... (what Claude commonly emits wrong)
```

Keep each entry under ~10 lines. The prompt is already ~150 lines; doubling it hurts more than it helps.

---
