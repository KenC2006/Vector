# Vector

A desktop IDE for designing real robots. You describe what you want in plain English, the AI assembles a kinematic chain from a catalog of ~150 real components (servos, hub motors, lidar, lipo packs, brackets, wheels), and you get a URDF you can simulate in MuJoCo.

It runs as a Tauri app — Rust shell, TypeScript + Three.js for the viewport, Monaco for URDF editing, and a Python core that talks to Claude and the placement compiler over JSON-RPC.

## What you can actually do with it

**Type "build me a quadruped" and watch it happen.** The AI doesn't write XML directly. It calls a tool API (`add_component`, `attach_to_face`, `set_joint`, etc.) that goes through a placement compiler. The compiler resolves face attachments, picks connector ports, runs collision-aware multi-child distribution, and emits the URDF. If it produces something the topology validator rejects (an actuator stacked on another actuator's shaft, a shaft hitting a structural mount without a bracket), it self-corrects within the same turn before you see a thing.

**Edit the URDF by hand and it stays in sync.** Monaco editor on the left, live 3D on the right. Save → reparse → scene update. Hot-reload on the geometry too — when a STEP/binbrep finishes loading, the viewport swaps the parametric placeholder for real CAD without breaking your camera.

**Real components, not placeholders.** 147 catalog presets across 10 categories (actuators, motors, sensors, compute, power, structural, transmission, end effectors, mobility, drivetrain). Each one ships with authored mate connectors, a real STEP file, a precomputed `.binbrep` (OCCT native binary B-Rep — ~50× faster to parse than STEP), and a convex-hull collision OBJ for the placement compiler to read.

**Simulate.** Hit Run. The URDF gets converted to MJCF, MuJoCo physics kicks in, and the robot walks/drives/falls over depending on what you built. Quadruped foot-leveling and contact pads are validated by a corpus test — you don't end up with a dog that ice-skates because its feet aren't actually the lowest geom.

**Bake the assembly.** Once placement settles, an OCCT worker fuses adjacent CAD parts (servo + coupler + extrusion become one solid) and renders the fused result. Quiet by default — silent during, single toast on failure.

**Inline-diff AI edits.** When the AI proposes a change, you see it as a Monaco inline diff. Accept commits the new URDF; Dismiss reverts. Per-file chat scoping means each URDF tab has its own conversation history — switching tabs doesn't blow away context, and a pending diff on tab A survives a side trip to tab B.

## How it stays correct

There's a strict pre-commit gate (`npm run check`) with 17 suites that have to pass before code lands. Among the more useful ones:

- **Topology validator** — rejects illegal attachments before they reach the placement compiler.
- **Mate-corpus** — 47 hand-authored attachment scenarios (servo-on-baseplate, hub-motor-with-wheel, gripper-on-end-of-arm) that must compile to known-good poses.
- **Visual parity** — 53 cases checking that the rich Three.js generators agree with the resolver's authored frame within tolerance.
- **Collision envelope** — every preset's `bbox_mm` must agree with its convex-hull collision mesh within 15%. No baseline mechanism for this one — collision drift is a hard fail because the placement compiler reads collision bounds as the authoritative envelope.
- **Rotation parity (TS + Python)** — same input must produce the same world-frame rotation through both the JS placement engine and the Python compiler.
- **Mesh-extents** — measures the actual rendered extent of every GLB and warns if it's drifted from the authored bbox by more than 5% (with a baseline file for known intentional drifts).

The gate is wired through a versioned git hook in `.githooks/pre-commit`. Activate with `npm --prefix src run install-hooks`.

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

