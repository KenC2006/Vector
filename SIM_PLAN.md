# Real Physics Simulation Plan

Goal: go from "URDF loads in MuJoCo and we can push torque values" to "you can build a quadruped in Vector, press play, and watch it walk." Target demo: a small 12-DOF robot dog trotting on flat ground with working servo controllers, gravity, contacts, and a closable feedback loop.

---

## 1. Where we are today

What already works:
- `core/sim/urdf_to_mjcf.py` converts our URDF to MJCF, preserves joint axes, limits, damping/friction, and emits one `motor` per non-fixed joint.
- `core/sim/mujoco_adapter.py` loads the model, steps it, exposes `get_state()` (qpos, qvel, body poses, contacts, actuator forces), `set_control()`, `set_gravity()`, `reset()`.
- `src/src/urdfAssembly.ts` now emits per-visual `<collision>` elements — hitboxes are no longer one-box-per-link.
- Free-floating base is supported via `free_base=True` → `<freejoint/>` on root.
- Frontend `simManager.ts` drives lifecycle, keyframes, sliders, and a preview animation path; real-core stepping path exists.

Gaps that block a walking robot:
1. **Actuators are raw torque motors.** `motor` with a ctrl range is open-loop torque. You can't say "hold 30°" — you'd have to write a PD loop by hand in JS every tick. Real servos are position-controlled.
2. **Inertia is fake.** `_create_body_element` writes `diaginertia = 0.001 * mass` for everything. A 2 kg torso and a 10 g foot get geometrically similar inertia tensors. Walking controllers are very sensitive to this — the robot will feel "floaty" or "glued," and tuned gaits won't transfer.
3. **Contact model is default.** No friction tuning, no solver tuning, no condim settings. Feet will slide.
4. **No ground-truth sensors.** No IMU, no foot-contact booleans, no joint encoders exposed as an observation vector. A controller has nothing to close the loop on.
5. **No controller layer at all.** There's a keyframe player and sliders, but nothing that can run a gait. No trajectory interpolation, no PD targets, no state machine, no CPG, no RL policy hook.
6. **No legged-robot components in the preset library.** Presets are arms/grippers/extrusions. There's no "hip-abduction servo + upper leg + knee servo + lower leg + foot pad" building block, and no template quadruped.
7. **Rendering is placeholder.** `render_frame` draws a gradient — there's no real MuJoCo offscreen render. Fine for now, but means the viewport relies on the Three.js mirror of `get_state()`, which works but has to be wired carefully to avoid drift.
8. **Step loop is JS-driven.** `simStepIntervalId` style loops in the frontend mean sim speed is coupled to JS timer jitter. A stable walker usually wants the physics thread to own timing.

---

## 2. Phases

### Phase A — Real actuators (the biggest single unlock)

Swap `motor` for `position` actuators in `urdf_to_mjcf.py`:

```xml
<position name="{joint}_pos" joint="{joint}" kp="..." kv="..." forcerange="..." ctrlrange="{lower} {upper}"/>
```

- `kp` (stiffness) and `kv` (damping) come from a new `servo_spec` field on joint presets (or derived from `effort` + a stiffness ratio).
- `ctrlrange` matches the joint limits — now `set_control({joint: angle_rad})` means "go to this angle."
- Keep a fallback path: `actuator_mode: "torque" | "position" | "velocity"` on the joint, default `position`. Torque mode is still useful for debugging and for RL.
- Update `simManager.ts` `set_control` call sites: sliders currently send a scaled torque-ish number; they should send radians.
- Update keyframe playback: keyframes are already "per-joint target angle," they were just being interpreted as torques. This is a rename + unit fix.

Exit criteria: drag a slider to 45° with gravity on, joint holds 45° against the load.

### Phase B — Honest inertia

Replace the hardcoded `diag = 0.001 * mass` with a shape-based inertia from the collision primitive:
- Box → `(1/12) m (y²+z², x²+z², x²+y²)`
- Cylinder (z-aligned) → `(1/12 m (3r² + h²), same, 1/2 m r²)`
- Sphere → `2/5 m r²`

For multi-primitive links (post-per-visual-collision work), compute the parallel-axis sum and rotate into the link frame. This is ~40 lines of numpy. Also:
- Let preset authors override with an explicit `inertia` block when they know the real value (motors publish their inertia in datasheets).
- Auto-mass: if `mass` is missing, estimate from volume × density where density comes from a component-type table (steel bracket ≠ plastic shell).

Exit criteria: a dropped part falls and spins like the shape looks, not like a point mass.

### Phase C — Contact and ground tuning

- Add `<default>` block in MJCF emission: `<geom friction="1.0 0.05 0.001" condim="4" solref="0.005 1" solimp="0.9 0.95 0.001"/>`.
- Tag foot geoms specifically with higher friction and `condim=6` (includes torsional) so the robot doesn't pirouette.
- Expose a `material: "foot_rubber" | "plastic" | "metal"` field on presets that maps to friction values.
- Self-collision: URDF doesn't express collision filtering. Emit a `<contact>` section with `<exclude>` pairs for adjacent links (parent/child across every joint) to avoid jitter from touching neighbors. Leave non-adjacent self-collision on — that's what catches legs crossing.

Exit criteria: robot stands still on the floor without sliding or sinking.

### Phase D — Sensors and observations

Emit a `<sensor>` block in the MJCF with, at minimum:
- `framequat` + `framelinvel` + `frameangvel` on the root body (IMU surrogate).
- `jointpos` + `jointvel` on every actuated joint (encoders).
- `touch` on every foot geom (contact boolean/force).

Return these through `get_state()` as `state["sensors"] = {...}` so the frontend and any controller can see them. This is what a policy would consume.

Exit criteria: pick up the robot in sim via a nudge, watch the IMU quaternion update in the sim panel.

### Phase E — Controller layer

This is the real brain. Build it in Python alongside `mujoco_adapter.py` so it runs at physics rate, not JS-timer rate.

Start with the simplest thing that works and grow:

1. **PD trajectory player.** A `Controller` class that takes a `(time → joint_targets)` callable and runs it every sim step. Walking policies that ship as open-loop joint trajectories (e.g. Boston-Dynamics-style precomputed gaits for a specific quadruped) drop straight into this.

2. **Trot CPG.** A small central pattern generator: each leg has a phase oscillator, phases coupled so the diagonal pairs move together, oscillator output modulates hip/knee targets. ~80 lines of Python. Tunable parameters: stride length, stride height, frequency, duty factor. This is how you get "press W to walk" without training anything.

3. **MPC / WBC (stretch).** Not worth building ourselves. If we want real dynamic gaits later, wrap an off-the-shelf library (`mujoco_mpc`, or export observations to a policy loaded from an ONNX file and run via `onnxruntime`).

4. **RL policy slot.** Even if we don't train, expose a "load `policy.onnx`" button. The input contract is fixed by Phase D sensors; the output is joint targets. This lets us drop in policies trained externally (Isaac Gym, MuJoCo MJX) without touching our code.

Control cadence: physics at 500 Hz (`timestep=0.002`, already set), controller at 200 Hz (decimation of 2–3), observations streamed to frontend at 30–60 Hz (decimation of ~8).

Exit criteria: pick "Trot" from a dropdown, robot walks forward on flat ground.

### Phase F — Step loop ownership

Move the step loop off the JS `setInterval` and into Python. The frontend says "start sim at real-time rate" and Python runs its own loop, pushing state snapshots over IPC at the render rate. This removes jitter and means a heavy UI frame doesn't stall physics.

Tauri side: either a long-lived Python subprocess with JSON-lines over stdio, or a local websocket. The existing `core/server.py` already does request/response — extend it with a "stream" mode.

Exit criteria: UI freeze for 500 ms, sim doesn't lose time.

### Phase G — Quadruped preset + demo robot

Nothing to simulate until you can build a dog. Add to the preset library:
- **Hip servo (2-DOF module)** — servo body + abduction bracket + hip joint. Real part reference: Dynamixel XL430 or similar.
- **Upper leg link** — parameterized length, attaches servo on one end, knee servo on the other.
- **Lower leg link** — length + foot pad (sphere or puck) at the tip, tagged as a foot geom.
- **Torso plate** — rectangular chassis with four mounting points for legs.

Ship a template `robot_dog.urdf` as a starter so users can press play on day one, then modify. Store it under `src/public/` alongside the presets.

Exit criteria: `File → New → Quadruped Template` loads a complete, simulatable robot.

### Phase H — UI polish for sim

- Sim panel shows: real-time factor (sim_time / wall_time), physics step count, contact count, max actuator force, IMU tilt.
- "Tune" tab exposes global gravity, floor friction, and per-joint `kp`/`kv` sliders — lets users feel the effect of gains on stability without editing XML.
- Gait selector dropdown (wired to Phase E controllers).
- Reset teleports robot back to a spawn pose *and* zeros velocities — `mj_resetData` already does this, just needs a button that also clears the controller's internal state (oscillator phases, PD integrators).

---

## 3. Research notes — things I'd verify before building

These are assumptions I'd want to confirm with the MuJoCo docs or a quick spike before committing:

- **`position` actuator vs. `general` with `gaintype`/`biastype`.** `position` is sugar for a specific PD formulation; for servos modeling a velocity limit it may be better to use `general` with an explicit transmission. Worth 20 min of reading before locking it in.
- **URDF `effort` → MuJoCo `forcerange`.** Already doing this, but double-check that MuJoCo's force limiting on `position` actuators actually clamps at the effort ceiling rather than clamping the *control signal* (those are different things).
- **Inertia from mesh.** For GLB components, MuJoCo can compute inertia from the mesh (`inertiafromgeom="true"` on compiler). This might be simpler than our analytical approach for meshed parts — but only if the mesh is watertight and manifold, which ours may not be.
- **`condim=6` cost.** Torsional friction is expensive. Check whether we need it on every foot or just on the ones that actually have a flat contact patch.
- **Physics rate.** 500 Hz is a safe default but 1 kHz is standard for quadrupeds with stiff knee servos. Higher rate costs CPU linearly — might matter for headless render.
- **Controller language.** Python ctrl loop at 200 Hz is fine *if* the IPC isn't in the loop. If the controller lives in Python and the stepper lives in Python, no IPC — good. If we ever move the controller to JS, the IPC round-trip will be the bottleneck.
- **Existing gait libraries for small quadrupeds.** Worth 30 min searching. `mujoco_menagerie` has ANYmal, Spot, Go1, A1 with trained policies — we could load one of *their* models as a sanity check that our pipeline is physically sound before trusting it with a user-built robot.

---

## 4. Milestones

- **M1 — Static stand.** Phases A+B+C. Robot dog stands on the floor under gravity without falling, sliding, or exploding. No walking yet. ~1 week of focused work.
- **M2 — Open-loop trot.** Phases D+E1+E2+G. A hand-tuned CPG makes the template quadruped trot forward on flat ground for 10 seconds without falling. ~1 week.
- **M3 — Interactive.** Phases F+H. Real-time factor ≥ 0.8, UI doesn't jank physics, user can tune gains live. ~3 days.
- **M4 — Bring your own policy.** ONNX loader. Confirms the observation/action contract is stable enough for external trained policies. ~2 days.

M1 is the gate. If a robot can't stand, nothing else matters.

---

## 5. Risks

- **Inertia is a rabbit hole.** Honest inertia computation across multi-primitive links with arbitrary orientations is finicky. Budget double the time.
- **Tuning the CPG is empirical.** Expect to spend a day sliding `kp`/`kv`/`frequency`/`stride` before it looks right. That's normal, not a sign something's broken.
- **Per-visual collision interacts with self-collision.** Now that each link can have multiple collision geoms, the `<exclude>` list (Phase C) needs to exclude *pairs of bodies*, not pairs of geoms — verify that's how MuJoCo's contact filtering actually works.
- **MuJoCo free-base + position actuators on all joints = robot pops into the air on load** if the spawn pose violates joint limits or if `kp` is too high relative to timestep. Always spawn at a known-good pose and ramp up gains over the first N steps.
