# Vector — Cursor for Robotics

An AI-native IDE for designing, editing, and simulating robots using natural language.

---

## Vision

Roboticists spend most of their time writing boilerplate URDF/MJCF, manually exporting CAD to simulation formats, and debugging physics issues by reading raw XML. Vector eliminates that loop. You describe what you want, Vector generates a physically grounded robot model, you iterate in plain English, and when you're ready, drop into Simulation Mode to run and inspect it.

---

## Core Interaction Model

The primary interface is a **command bar that produces diffs**, not a chatbot that rewrites everything.

```
> add 2 legs to the robot
  ↳ Added left_leg and right_leg subtrees at pelvis.
    Hip joints set to 50 Nm (matched torso mass).
    [View diff] [Accept] [Reject] [Modify...]
```

Every AI change is shown as a structured diff on the model file. The user always sees exactly what changed and why.

---

## Feature Areas

### 1. Natural Language Robot Generation
- Describe a robot from scratch in plain text
- AI generates URDF, SDF, or MJCF with inline comments explaining every design decision
- Comments encode intent: torque ratings, workspace limits, tradeoffs made

**Example commands:**
```
"create a 6-DOF arm for light assembly tasks"
"add 2 legs to the robot"
"make the arm 15cm longer"
"mirror the right arm to create a left arm"
"add a depth camera to the head, facing forward"
"replace the hip joints with ones that support 50 Nm torque"
"stiffen the knee joints — they're oscillating in sim"
```

### 2. Stateful Model Editing
- AI understands the current robot structure before making any changes
- Maintains a kinematic graph in memory:
  ```
  base_link → torso → head
                    → left_shoulder → left_elbow → left_wrist
                    → right_shoulder → ...
  ```
- Operations map onto graph mutations:
  - **Add** — insert new subtree
  - **Remove** — prune subtree
  - **Modify** — change properties on a node
  - **Mirror** — duplicate and reflect subtree
  - **Replace** — swap subtree with a different preset
- Ambiguity resolved by context, with a single clarifying question if needed

### 3. Component Preset Library
Presets ship with real physical parameters so simulation is grounded from day one.

**Actuators:**
- Dynamixel XH430, XM540
- T-Motor AK series
- Linear servo profiles
- Custom actuator definition

**Sensors:**
- Intel RealSense D435 / D455
- Velodyne VLP-16
- IMU (MPU-6050, BMI088)
- Force/torque (ATI, Robotiq)

**Kinematic Archetypes:**
- 6-DOF serial arm
- Delta robot
- SCARA
- Bipedal (humanoid proportions)
- Quadruped (Spot-style, MIT Mini Cheetah-style)
- Mobile manipulator

**Grippers:**
- Parallel jaw (2-finger)
- 3-finger adaptive (Robotiq-style)
- Soft gripper
- Magnetic end effector

### 4. CAD Integration
Two paths:

**Parametric generation (Model → CAD):**
- Generate OpenSCAD or CadQuery Python from the kinematic description
- AI writes manufacturable geometry from link/joint definitions
- Change a joint limit → CAD updates automatically

**Round-trip import (CAD → Model):**
- Accept STEP / STL from Fusion360 or Onshape
- Extract geometry, auto-generate matching URDF/MJCF
- Mesh simplification for collision geometry

### 5. Simulation Backend
Pluggable simulation targets:

| Simulator | Best For |
|-----------|----------|
| MuJoCo | RL/learning, fast headless rollouts |
| NVIDIA Isaac Sim + GR00T | Photorealism, humanoid foundation model |
| Genesis (MIT) | Differentiable physics, sim-to-real |
| Gazebo/Ignition | ROS-native deployments |
| PyBullet | Lightweight scripting |

Simulation is a mode, not a background process. The user explicitly enters Simulation Mode when they want to run it — keeping the design/edit workflow lightweight by default.

**In Simulation Mode:**
- Load the current model into the selected backend
- Run, pause, scrub, and inspect the simulation
- Exit back to the editor at any time; the model file is the source of truth

**World model features (via Isaac GR00T / Genesis):**
- Synthetic training data generation
- Domain randomization baked in
- Sensor simulation: depth, lidar, tactile, IMU

### 6. Validation Agent
Runs automatically after every edit:
- Kinematic reachability check
- Self-collision detection
- Inertia tensor plausibility (flags non-physical values)
- Center of mass stability for legged robots
- Joint limit vs actuator torque cross-check
- Mesh watertightness for CAD export

Validation results surface as inline annotations in the model file, not just a log.

---

## UI Layout

```
┌─────────────────────────┬──────────────────────────────┐
│  Robot File (URDF/MJCF) │   3D Viewport / Sim           │
│                         │                              │
│  [editable, AI inline   │   [live — updates on save]   │
│   completions active]   │                              │
│                         │                              │
├─────────────────────────┴──────────────────────────────┤
│  > add 2 legs to the robot                             │
│    ↳ Added left_leg and right_leg at pelvis.           │
│      [View diff] [Accept] [Reject] [Modify...]         │
└────────────────────────────────────────────────────────┘
```

**Left pane:** Model file with AI inline completions (like Cursor's ghost text). Completions suggest joint limits based on referenced actuator, inertia from preset mass, etc.

**Right pane:** 3D viewport. In edit mode shows a static preview — collision geometry, joint axes, CoM marker, sensor FOV cones. Switch to Simulation Mode to run physics.

**Command bar:** Not a chatbot. A command bar that produces diffs. `Cmd+K` equivalent.

---

## File Format Support

| Format | Read | Write | Notes |
|--------|------|-------|-------|
| URDF | ✓ | ✓ | ROS standard |
| MJCF | ✓ | ✓ | MuJoCo, dominant in RL |
| SDF | ✓ | ✓ | Gazebo |
| USD/USDA | planned | planned | NVIDIA Omniverse |
| STEP | ✓ | ✓ | CAD import/export |
| STL/OBJ | ✓ | ✓ | Mesh import |
| OpenSCAD | — | ✓ | Parametric CAD gen |
| CadQuery | — | ✓ | Parametric CAD gen |

---

## Build Phases

### Phase 1 — MVP
- [ ] URDF + MJCF generation from natural language
- [ ] Inline comments explaining every design decision
- [ ] Stateful edit commands ("add 2 legs", "make arm longer")
- [ ] Kinematic graph maintained in memory across edits
- [ ] Diff view for every AI change
- [ ] Preset library: 5 actuators, 3 sensors, 3 kinematic archetypes
- [ ] MuJoCo integration (Simulation Mode — user-initiated)
- [ ] Basic validation: self-collision, inertia check, CoM

### Phase 2 — CAD + Expanded Sim
- [ ] CadQuery / OpenSCAD parametric export
- [ ] STEP/STL import with auto URDF generation
- [ ] Isaac Sim integration
- [ ] Expanded preset library
- [ ] Sensor simulation (depth, lidar)
- [ ] Domain randomization config

### Phase 3 — Training + Deployment
- [ ] GR00T / Genesis world model integration
- [ ] Synthetic dataset generation from sim
- [ ] ROS 2 package export (launch files, meshes, controllers)
- [ ] Hardware-in-the-loop mode (deploy to real robot, compare to sim)
- [ ] Collaborative editing (team shares a robot model like a git repo)

---

## Competitive Landscape

| Tool | Gap |
|------|-----|
| NVIDIA Isaac Lab | No model gen, steep learning curve, NVIDIA-locked |
| Onshape / Fusion360 | CAD only, URDF export is manual and painful |
| URDF Composer | GUI only, no AI, no sim integration |
| ROS tooling | Fragmented, not AI-native, high expertise floor |
| Lerobot (HuggingFace) | Training focused, no model design tooling |

**Moat:** The stateful edit + sim feedback loop. Nobody owns this end-to-end. The hard part is not generation — it's understanding the existing model before touching it.

---

## Open Questions

- Primary file format for MVP: URDF or MJCF? (URDF is more universal; MJCF is better for learning workflows)
- Desktop app (Electron / Tauri) vs web app vs VS Code extension?
- Sim backend for MVP: MuJoCo (simplest API) vs Isaac (best long-term)?
- Preset library: curated by us vs community-contributed from day one?
- Target user for MVP: robotics researcher, hardware engineer, or student?
