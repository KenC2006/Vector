# Vector — Tech Stack

---

## Overview

Vector is a local-first desktop application. The core loop is:
**edit robot model file → AI processes change → 3D preview updates → optionally enter sim**.

The stack is split into four layers: shell (desktop app), editor, AI/core, and simulation.

---

## Desktop Shell

**Tauri (Rust + WebView)**
- Lighter than Electron, native performance, ships a single binary
- Rust backend handles file I/O, process management, sim subprocess communication
- WebView frontend for all UI (Monaco, 3D viewport, command bar)
- Tauri's IPC bridges frontend commands to backend operations

**Alternative considered:** Electron — heavier but larger ecosystem. Defer this decision until 3D rendering performance is validated in Tauri's WebView.

---

## Editor

**Monaco Editor**
- Same editor as VS Code — syntax highlighting, inline decorations, diff view
- Custom language server for URDF/MJCF: syntax validation, hover docs on joints/links
- Inline ghost text completions (like Cursor) powered by the AI layer
- Diff rendering for every AI-generated change (accept / reject per hunk)

**3D Viewport**
- Three.js or Babylon.js running in the WebView
- Loads URDF/MJCF meshes (STL/OBJ/DAE) directly
- Shows: link frames, joint axes, collision geometry wireframe, CoM marker, sensor FOV cones
- Static preview only in edit mode — no physics running

---

## AI / Core Layer

This is the brain. Lives in a Python process that the Tauri backend spawns and communicates with over a local socket (JSON-RPC or msgpack).

**Language: Python**
- Robotics ecosystem is Python-native (urdfpy, mujoco, ROS, CadQuery all have Python APIs)
- Faster iteration than Rust for the AI/model logic

### AI

**Claude API (claude-sonnet-4-6)**
- NL → robot model generation
- Stateful edit commands ("add 2 legs") → structured URDF/MJCF diffs
- Validation feedback interpretation ("why is this unstable")
- Structured output (JSON) for diffs, not free-form text — so the editor can render them cleanly

**Prompt architecture:**
- System prompt carries the current kinematic graph + active presets as context
- Each edit command sends: current model XML + user instruction → returns: diff + explanation + validation flags
- Keep model files chunked — large URDFs get summarized into the kinematic graph to stay within context

### Robot Model Parsing

| Library | Purpose |
|---------|---------|
| `yourdfpy` | URDF parsing, validation, forward kinematics |
| `mujoco` (Python bindings) | MJCF parsing + headless physics queries |
| `networkx` | Kinematic graph representation (links = nodes, joints = edges) |
| `lxml` | XML manipulation for applying diffs to URDF |
| `numpy` | Inertia tensor math, transform chains |

### Kinematic Graph

The internal representation used by the AI layer — not the raw XML.

```
Node: Link { name, mass, inertia, mesh_path, collision_geometry }
Edge: Joint { name, type, parent, child, axis, limits, actuator }
```

Every AI edit operates on this graph first, then serializes back to URDF/MJCF. This prevents the AI from corrupting XML structure and makes diffs clean.

### Preset Library

- JSON files per component (actuator, sensor, archetype)
- Schema: name, category, physical_params, urdf_snippet, mjcf_snippet, thumbnail
- Bundled with the app; community presets pulled from a registry later
- Resolved at model-gen time — AI references preset by name, core layer injects the full spec

### Validation Agent

Runs after every accepted edit, before preview updates:

| Check | Tool |
|-------|------|
| URDF schema validity | `yourdfpy` |
| Self-collision | `yourdfpy` / `mujoco` headless query |
| Inertia tensor plausibility | `numpy` (eigenvalue check) |
| CoM height vs support polygon | `numpy` |
| Joint limit vs actuator torque | preset param lookup |
| Mesh watertightness (for CAD export) | `trimesh` |

Results returned as structured annotations → rendered as inline squiggles in Monaco.

---

## CAD Layer

**CadQuery (Python)**
- Generates parametric 3D geometry from link dimensions in the kinematic graph
- Exports STEP for manufacturing, STL for simulation meshes
- Runs in the same Python process as the core layer

**Import path (CAD → Model):**
- STEP/STL uploaded by user
- `cadquery` or `pythonocc` extracts geometry + bounding volumes
- AI infers link/joint structure from geometry + user description
- Generates matching URDF with mesh references

**OpenSCAD** — secondary option for simpler parametric output, easier for users to hand-edit.

---

## Simulation Layer

Simulation is a **mode**, launched as a subprocess when the user enters it. The core Python process sends the current model file to the sim backend and receives back joint states, contact forces, and render frames.

### MVP: MuJoCo

```
Python core  →  model.xml (MJCF)  →  mujoco.MjModel  →  MjSimulation
                                                       ↓
                                              joint states, CoM, contacts
                                                       ↓
                              render frames → viewport via shared memory / websocket
```

- `mujoco` Python bindings — clean API, fast headless
- Passive viewer or offscreen render piped to the frontend viewport
- User controls: play, pause, scrub timeline, reset

### Phase 2: Isaac Sim

- NVIDIA Omniverse Kit — Python scripted
- USD as the interchange format (convert URDF → USD on sim launch)
- GR00T integration for humanoid motion priors
- Photorealistic sensor simulation (depth, lidar)
- Requires NVIDIA GPU — gated behind hardware check

### Phase 2: Genesis (MIT)

- Differentiable physics — useful for sim-to-real and policy gradient training
- Pure Python, no GPU required for basic use
- Best for RL researchers

### Sim ↔ Editor contract

```json
{
  "model_path": "robot.mjcf",
  "command": "load" | "play" | "pause" | "reset" | "step",
  "result": {
    "joint_states": [...],
    "com_position": [...],
    "contacts": [...],
    "render_frame": "<base64 png>"
  }
}
```

---

## Data Flow (end to end)

```
User types: "add 2 legs to the robot"
        ↓
Command bar (Monaco / WebView)
        ↓
Tauri IPC → Python core (JSON-RPC)
        ↓
Claude API: kinematic graph + instruction → structured diff
        ↓
Diff applied to kinematic graph → serialized to URDF/MJCF
        ↓
Validation agent runs → annotations returned
        ↓
Diff + annotations sent back over IPC
        ↓
Monaco renders diff view + inline squiggles
Three.js reloads mesh → viewport updates
        ↓
User accepts → model file written to disk
```

---

## File Structure (app)

```
vector/
├── src-tauri/          # Rust — shell, IPC, file I/O, process management
├── src/                # TypeScript — Monaco, Three.js, command bar, UI
├── core/               # Python — AI layer, model parsing, validation, CAD, sim bridge
│   ├── ai/             # Claude API calls, prompt templates, diff parser
│   ├── model/          # Kinematic graph, URDF/MJCF read/write
│   ├── presets/        # JSON preset library
│   ├── validation/     # Validation checks
│   ├── cad/            # CadQuery export/import
│   └── sim/            # Sim backend adapters (MuJoCo, Isaac, Genesis)
└── presets/            # Bundled component presets (JSON)
```

---

## Key Dependencies

| Package | Language | Purpose |
|---------|----------|---------|
| `tauri` | Rust | Desktop shell |
| `monaco-editor` | TS | Code editor |
| `three` / `babylon.js` | TS | 3D viewport |
| `@anthropic-ai/sdk` → use Python `anthropic` | Python | Claude API |
| `yourdfpy` | Python | URDF parsing + validation |
| `mujoco` | Python | MJCF parsing + simulation |
| `networkx` | Python | Kinematic graph |
| `cadquery` | Python | Parametric CAD |
| `trimesh` | Python | Mesh processing |
| `numpy` | Python | Math |
| `lxml` | Python | XML diffing |

---

## Open Questions

- **IPC protocol:** JSON-RPC over stdio vs local Unix socket vs WebSocket — stdio is simplest for MVP
- **3D library:** Three.js (lighter, more control) vs Babylon.js (built-in physics preview, better GLTF support)
- **Mesh format in viewport:** Convert all meshes to GLTF at load time for Three.js, or support STL/OBJ/DAE natively
- **Python env management:** Bundle a Python runtime (PyApp / PyOxidizer) or require user to have Python installed
- **Preset registry:** Local-only for MVP, or build a simple HTTP registry from day one
