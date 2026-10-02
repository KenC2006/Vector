# Vector

Type what robot you want, get a URDF you can edit and simulate.

Vector is a desktop app (Tauri + Python + Three.js). You describe a robot,
Claude designs it from a catalog of real parts, and you get a URDF in an editor
next to a 3D view. From there you can keep editing it by chat, by hand, or with
the mouse, and drop it into MuJoCo to see if it actually works.

Still a prototype (v0.2.0), so expect rough edges.

[![Vector demo](docs/media/demo.gif)](https://github.com/KenC2006/Vector/releases/download/v0.2.0/vector_demo.mp4)

[Full demo video (3.5 min)](https://github.com/KenC2006/Vector/releases/download/v0.2.0/vector_demo.mp4)

## How it works

Claude doesn't write the URDF directly. It writes a design: a list of parts,
where each one goes, which way it faces, and how each joint moves. The compiler
in `core/designer/` turns that into a URDF. A checker then looks at the result
for parts that float or overlap, which parts touch the ground, whether the robot
tips over, and whether each motor can hold its load. Claude fixes whatever it
flags and tries again.

The design is saved inside the URDF as a comment, so when you ask for a change
("put an arm on its back") it edits the existing robot instead of starting over.

## Editing

You can edit a robot a few ways, and they all change the same design:

- Chat with the AI. Changes show up as a diff you can accept or reject.
- Drag parts in from the Components panel. They snap to open connectors or sit
  flat on whatever surface you're pointing at. Tab changes which side attaches,
  R rotates it, J changes the joint type.
- Select a part and move it with the gizmo, or change its fields in the
  Properties panel.
- Edit the URDF text directly. Vector notices and merges your edit back in.

You can also open a URDF that Vector didn't make and edit it the same way.

## Simulation

Click Simulate and the robot loads into MuJoCo. You can switch between flat,
rough and stairs terrain, and drive the robot with WASD while it runs.

Robots are controlled by a small Python script with a `step(t, state)` function
that runs at 200 Hz. There are two ways to get one:

- **Quick**: builds a simple controller right away (no AI) by testing what each
  joint and wheel does. Wheeled robots drive, legged robots get a basic gait.
- **Generate**: tell it what you want ("walk forward", "drive a figure 8") and
  Claude writes a controller, runs it in the sim, and keeps adjusting it until it
  works. Takes a minute or two.

## Parts catalog

147 parts across actuators, motors, sensors, compute, power, structure,
transmission, end effectors, wheels/tracks and drivetrain. Each part has
connectors that define how it attaches to other parts, and a 3D model.

## Running it

You need Rust, Node, Python 3.10+ and the Tauri CLI (`cargo install tauri-cli`).

```bash
pip install -r core/requirements.txt
cd src && npm install && cd ..
cargo tauri dev
```

Add your Anthropic key to a `.env` file in the repo root:

```env
ANTHROPIC_API_KEY=sk-...
# only needed for org keys that aren't tied to a workspace:
# ANTHROPIC_WORKSPACE_ID=wrkspc_...
```

## Tests

Run `npm run check` from `src/`. It covers the designer (generated URDFs match
the design, hand edits merge back correctly), the simulator (joints, wheels,
tracks and the baseline controllers behave), part visuals, and catalog data.

To run it before every commit: `npm --prefix src run install-hooks`.

## Project layout

```
src/          frontend (TypeScript, Vite, Three.js, Monaco)
src-tauri/    Rust shell that runs the Python core
core/         Python: designer, simulator, Claude calls
assets/       source CAD files (not shipped with the app)
scripts/      catalog and mesh tooling
```
