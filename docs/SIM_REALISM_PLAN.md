# Simulation Realism Plan

Goal: make Vector's simulation physically faithful — real gravity, real positions, real dynamics — so that a loaded URDF behaves the way the same robot would on a bench.

This is a working plan, grouped by subsystem. Each item lists the file(s) involved and the concrete change.

---

## 1. Physics solver & integrator (MuJoCo options)

**Where:** `core/sim/urdf_to_mjcf.py` (`<option>` block, ~L539–545)

Current: `timestep=0.002`, `gravity=0 0 -9.81`, defaults otherwise.

Changes:
- [ ] Switch integrator to `implicitfast` (or `implicit` for stiff contacts). RK4 is an option but slower and worse with contact.
- [ ] Set `solver="Newton"`, `iterations="100"`, `tolerance="1e-10"` for tight constraint convergence.
- [ ] Set `cone="elliptic"` and `impratio="10"` — prevents the "ice-skating" feel of the default pyramidal friction cone.
- [ ] Set `noslip_iterations="3"` to kill residual tangential drift at rest.
- [ ] Enable `<flag override="enable" energy="enable"/>` so we can assert energy conservation in tests.
- [ ] Drop `timestep` to `0.001` for stability with stiff actuators; compensate by stepping more substeps per frame (see §6).
- [ ] Expose these as editable sim settings in the UI (`simManager.ts` — add a "Physics" collapsible in the sim panel).

---

## 2. Mass & inertia fidelity

**Where:** `core/sim/urdf_to_mjcf.py` (`_compute_inertia_from_collision_list`, ~L262–310; `_PLASTIC_DENSITY` auto-estimation ~L347)

Current: falls back to plastic-density volume estimate when URDF omits mass. Composite inertia from collision primitives is correct, but:

- [ ] Warn (surface in UI) when auto-estimation kicks in — user should know their sim is using a guessed 1.2 g/cm³.
- [ ] Let the user pick a material (`aluminum`, `steel`, `pla`, `abs`, `carbon`) per link; bake density accordingly. Store on the link via a `<vector:material>` custom tag in the URDF so it round-trips.
- [ ] When *visual* geometry is richer than *collision* geometry (common for mesh robots), compute inertia from the visual mesh using `trimesh.inertia` and use it instead. This is the biggest single accuracy win.
- [ ] Validate: compute total mass and COM, show them in the sim panel. If total mass < 10 g or COM is outside the AABB, flag it.

---

## 3. Joint dynamics (damping, friction, armature)

**Where:** `urdf_to_mjcf.py` L503–509, L645–651

Current: damping defaults to 0.1, frictionloss 0.0, no armature. These are the three biggest reasons a MuJoCo arm "feels wrong".

- [ ] Add **armature** (reflected rotor inertia) per joint. Even a small value (`1e-4`–`1e-3`) dramatically stabilises PD control and prevents jitter. Default based on joint type: revolute → `5e-4`, prismatic → `1e-3`.
- [ ] Default damping from `0.1` → something scaled to joint inertia (e.g. `0.2 × sqrt(I·kp)` for critical-ish damping).
- [ ] Add **frictionloss** default `~0.02` Nm (revolute) / `0.5 N` (prismatic) — dry friction, absent today, is why joints keep drifting after you release control.
- [ ] Respect URDF `<safety_controller k_velocity>` and `<limit effort velocity>` — currently ignored. Wire them into actuator `forcerange` and `ctrlrange`.

---

## 4. Actuators

**Where:** `urdf_to_mjcf.py` ~L717 (PD actuator creation)

Current: every joint gets a position actuator (`_pos`) plus motor (`_motor`), with a kv ≈ 0.1·kp heuristic.

- [ ] Replace the position-actuator-only path with **torque-limited PD** — i.e., position actuator with explicit `forcerange` from URDF `effort`. Today unlimited torque masks model errors.
- [ ] Compute kp from desired closed-loop bandwidth and reflected inertia, not as a hand-tuned constant. Target 10–20 Hz for joints < 1 kg·m², 3–5 Hz above.
- [ ] Support URDF `<transmission>` (gear ratios + motor rotor inertia). Without this, armature is just a guess.
- [ ] Add actuator latency (`user` custom field → first-order lag in the control loop). Real motors aren't instant.

---

## 5. Contacts & friction

**Where:** `urdf_to_mjcf.py` L544–585 (default geom class, floor, foot class)

Current: `condim=4`, friction `1.0 0.05 0.001` for links, `1.5 0.1 0.01` for floor and feet.

- [ ] Use `condim=6` on feet / contact points (torsional + rolling) — already done for "foot" class, extend to any link tagged as a contact.
- [ ] Add **solref / solimp** defaults: `solref="0.005 1"` (contact time constant of 5 ms — stiffer than default, closer to rigid), `solimp="0.9 0.95 0.001"`. This alone removes most visual "squish" on heavy robots.
- [ ] Auto-classify links with "foot", "wheel", "gripper", "finger" in the name into sensible friction classes instead of all-or-nothing defaults.
- [ ] Wheels: detect continuous joints whose child link has cylinder collision → apply rolling-friction class (low torsional, high lateral).
- [ ] Expose `floor friction` in UI as a slider for quick what-if.

---

## 6. Time stepping (real-time sync)

**Where:** `src/src/simManager.ts` L731–755, L928–930

Current: `setInterval(..., 1000/60)`; each tick substeps by wall-clock elapsed, capped at 20. Drifts under load and gives lumpy visible motion.

- [ ] Replace `setInterval` with a `requestAnimationFrame` loop.
- [ ] Track `simTime` vs `wallTime` explicitly; step the physics until `simTime ≥ wallTime − lag`. Skip ahead (not infinitely) on tab-resume.
- [ ] Expose a "sim speed" multiplier (0.1× … 2×) and a "max real-time factor" readout.
- [ ] When `simModelDt = 0.001` and the frame budget is 16 ms, we need up to 16 substeps per frame — make that the default cap, not 20.
- [ ] When the physics can't keep up, surface a warning badge rather than silently slowing down.

---

## 7. Ground-truth positions

**Where:** `mujoco_adapter.py` `_auto_lift_fixed_base` / `_auto_lift_above_floor` (L76–174); `get_state` L213+

Current: uses `geom_size.max()` as a conservative half-extent → lifts robots too high for mesh geoms.

- [ ] Use `mj_collision` or per-geom AABB (`mj_geomAABB`) for the floor-clearance probe. The current `max(size)` bound is wrong for boxes and useless for meshes.
- [ ] Report body positions in **world frame with explicit units** (metres, radians) in `get_state` and label them in the UI.
- [ ] Add full 6-DOF pose of a user-selectable "end-effector" body to state (`xpos`, `xquat`, linear + angular velocity). This is what users will actually want to read.
- [ ] Ship `mj_forward` after `set_control` but before `get_state` on the *same* tick so the UI never shows a one-step-stale pose.

---

## 8. Sensors (for closed-loop realism)

New section in `urdf_to_mjcf.py` (`<sensor>` block, not currently emitted).

- [ ] Emit `jointpos`, `jointvel`, `jointactuatorfrc` sensors per joint. Cheap, lets the UI plot real observables.
- [ ] Emit `framepos`/`framequat` for any link tagged `<vector:ee>` (end-effector).
- [ ] Emit `accelerometer`, `gyro`, `framequat` on any link named `imu*` or tagged `<vector:imu>`.
- [ ] Support Gaussian noise on sensor reads (configured in sim panel) — critical for any sim-to-real workflow.

---

## 9. Determinism & reproducibility

- [ ] Thread the existing `seed` input (`simManager.ts` L667) into `mj_setSeed` on load, and into any noise processes (§8).
- [ ] Snapshot the full `qpos`/`qvel`/`ctrl` vector in `reset()`; restore exactly on subsequent resets rather than re-running `_auto_lift_*`.
- [ ] Record every step's `qpos` to an in-memory ring buffer; add a "scrub timeline" in the UI that replays without re-stepping.

---

## 10. Validation — "is the sim actually real?"

- [ ] Gravity test: a free-falling unit box drops `½·g·t²` within 1 mm over 1 s. Add as a pytest under `core/sim/tests/test_physics.py`.
- [ ] Pendulum test: 1 m massless-rod pendulum with 1 kg bob has period `2π·√(l/g)` to 0.5 %.
- [ ] Energy conservation test: with damping=0, friction=0, integrator=implicit, total energy drifts < 1 % over 10 s.
- [ ] Contact test: stacked-box tower stays stacked for 5 s (no interpenetration, no drift > 1 mm).
- [ ] Actuator test: a 1 DOF arm with gravity and a position actuator settles to the target within one time constant ± 5 %.

These five tests are the definition of "real physics" — if they pass, the sim is credible.

---

## Priority order (what to do first)

1. **§7** (`_auto_lift` AABB fix) and **§1** (integrator + cone) — one hour, biggest immediate realism bump.
2. **§3** (armature + frictionloss defaults) — kills the "rubbery" feel.
3. **§10** (validation tests) — lock in the wins so regressions show up.
4. **§6** (real-time loop) — makes the *visible* motion match the physics.
5. **§2** (mesh-based inertia) and **§4** (transmission support) — biggest accuracy wins once the basics are solid.
6. **§8** (sensors) and **§9** (determinism) — enable the next class of workflows.
