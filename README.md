# Vector

Describe a robot in plain English and Vector builds it. Ask for a quadruped and
the AI assembles a kinematic chain from a parts catalog, then hands you a URDF
you can edit by hand and run in a MuJoCo sim.

It's a desktop app built with Tauri: a Rust shell around a Python core, with a
Three.js viewport and a Monaco editor up front. The core talks to Claude and the
placement compiler over JSON-RPC.

> Status: v0.1.0, a working prototype. Expect sharp edges.

## What it does

The AI never writes XML. It calls a small tool API (`add_component`,
`attach_to_face`, `set_joint`) that drives a placement compiler. The compiler
resolves face attachments, picks connector ports, spreads multiple children
apart so they don't collide, and emits the URDF. When the topology validator
rejects a layout, like an actuator stacked on another actuator's shaft or a
drive shaft sunk into a mount with no bracket, the model corrects itself on the
same turn.

From there the URDF is yours. Monaco on the left, live 3D on the right; save and
the scene reparses. Geometry hot-reloads, so when a STEP or binbrep finishes
loading the viewport swaps the parametric stand-in for the real CAD without
touching your camera. AI edits arrive as inline Monaco diffs: Accept keeps them,
Dismiss reverts. Chat is scoped per file, so every URDF tab keeps its own history.

## The catalog

147 presets in 10 categories: actuators, motors, sensors, compute, power,
structural, transmission, end effectors, mobility, and drivetrain. Each one
ships with authored mate connectors, a STEP file, a precomputed `.binbrep`
(OCCT's native binary B-Rep, roughly 50× faster to parse than STEP), and a
convex-hull collision mesh.

## Simulation

Hit Run and the URDF converts to MJCF and loads into MuJoCo. Drive the joints
with a Python controller you upload or have the AI write, toggle gravity, and
switch between flat, rough, and stairs terrain. A corpus test checks quadruped
foot-leveling, so the feet really do sit lowest on the ground.

## Getting started

You'll need Rust, Node, Python 3.10+, and the Tauri CLI (`cargo install tauri-cli`).

```bash
# Python core
pip install -r core/requirements.txt

# Frontend
cd src && npm install && cd ..

# Run — boots the Vite dev server and the desktop shell together
cargo tauri dev
```

AI features read keys from a `.env` in the repo root:

```env
ANTHROPIC_API_KEY=sk-...   # assembly and edits (Claude)
GEMINI_API_KEY=...         # sim-script generation
```

## Tests

`npm run check`, run from `src/`, is the full gate: 19 suites across the
placement compiler, validators, and sim. The ones that carry the most weight:

- **Topology validator** rejects illegal attachments before the compiler runs.
- **Mate corpus**: 47 hand-authored attachment scenarios that have to compile to known-good poses.
- **Rotation and anchor parity**: the TypeScript placement engine and the Python compiler have to agree to the bit.
- **Collision envelope**: each preset's `bbox_mm` has to match its convex-hull mesh within 15%.
- **Mesh extents** flags any GLB whose rendered size drifts more than 5% from its authored bbox.

A git hook at `.githooks/pre-commit` runs the gate before every commit. Install
it with `npm --prefix src run install-hooks`.

## Layout

```
src/          Tauri frontend (TypeScript, Vite)
src/src/      App code: main.ts, viewportChat.ts, urdfAssembly.ts,
              placementCompiler/, richVisuals/, simManager.ts
src/public/   Component catalog: STEPs, binbreps, GLBs, collision meshes
src-tauri/    Rust shell; owns the Python core process
core/         Python: JSON-RPC server, Claude client, placement compiler,
              URDF/MJCF converters, semantic graph
scripts/      One-off tooling: collision regen, bbox audits, catalog validators
docs/         Design notes and decision logs
```
