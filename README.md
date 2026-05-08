# Vector

A desktop IDE for designing robots. You describe what you want in plain English, the AI assembles a kinematic chain from a catalog of components (servos, hub motors, lidar, lipo packs, brackets, wheels, etc.), and you get a URDF you can simulate in MuJoCo.

It's a Tauri app — Rust shell, TypeScript + Three.js for the viewport, Monaco for URDF editing, and a Python core that talks to Claude and the placement compiler over JSON-RPC.

## What it does

Type "build me a quadruped" and the AI starts assembling. It doesn't write XML directly — it calls a tool API (`add_component`, `attach_to_face`, `set_joint`, etc.) that feeds a placement compiler. The compiler resolves face attachments, picks connector ports, runs collision-aware multi-child distribution, and emits the URDF. If the topology validator rejects something (actuator stacked on another actuator's shaft, shaft into a structural mount without a bracket), the model self-corrects in the same turn.

The URDF stays editable by hand. Monaco on the left, live 3D on the right. Save → reparse → scene update. Geometry hot-reloads too: when a STEP/binbrep finishes loading, the viewport swaps the parametric stand-in for the CAD without resetting your camera.

The catalog has 147 presets across 10 categories (actuators, motors, sensors, compute, power, structural, transmission, end effectors, mobility, drivetrain). Each one carries authored mate connectors, a STEP file, a precomputed `.binbrep` (OCCT native binary B-Rep, ~50× faster to parse than STEP), and a convex-hull collision OBJ.

Hit Run to simulate. The URDF gets converted to MJCF and handed to MuJoCo. Quadruped foot-leveling and contact pads are checked by a corpus test, so feet are actually the lowest geom.

Once placement settles you can bake the assembly. An OCCT worker fuses adjacent CAD parts (servo + coupler + extrusion → one solid) and renders the fused result. Silent during, single toast on failure.

AI edits land as Monaco inline diffs — Accept commits the new URDF, Dismiss reverts. Chat is scoped per file, so each URDF tab keeps its own history and a pending diff on tab A survives a trip to tab B.

## Pre-commit gate

`npm run check` runs 17 suites before anything lands. The ones that pull the most weight:

- **Topology validator** — catches illegal attachments before the placement compiler sees them.
- **Mate-corpus** — 47 hand-authored attachment scenarios (servo-on-baseplate, hub-motor-with-wheel, gripper-on-arm-end) that have to compile to known-good poses.
- **Visual parity** — 53 cases checking the Three.js generators agree with the resolver's authored frame.
- **Collision envelope** — each preset's `bbox_mm` has to agree with its convex-hull collision mesh within 15%. No baseline; the placement compiler trusts collision bounds as the envelope, so drift is a hard fail.
- **Rotation parity (TS + Python)** — same input through the JS placement engine and the Python compiler must produce the same world-frame rotation.
- **Mesh-extents** — measures the rendered extent of every GLB and flags >5% drift from the authored bbox (with a baseline for intentional cases).

The gate runs through a versioned git hook at `.githooks/pre-commit`. Install with `npm --prefix src run install-hooks`.

## Layout

```
src/                Tauri frontend (TypeScript, Vite)
src/src/            App code — main.ts, viewportChat.ts, urdfAssembly.ts,
                    placementCompiler/, richVisuals/, bake/
src/public/         Component catalog, STEPs, binbreps, GLBs, collision OBJs
src-tauri/          Rust shell that owns the Python core process
core/               Python — JSON-RPC server, Claude client, placement compiler,
                    URDF/MJCF converters, semantic graph
scripts/            One-shot tooling — collision regen, bbox audit, mesh extents,
                    catalog validators
docs/               Plans + decision logs
```

