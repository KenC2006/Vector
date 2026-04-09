# Simulation Pipeline Plan

## Goal
Connect the URDF editor to MuJoCo physics so designed robots can be simulated in real-time with the 3D viewport showing live joint states.

## Current Assets
- `core/sim/mujoco_adapter.py` — MuJoCo wrapper (load, step, reset, get_state, set_control, render)
- `core/sim/urdf_to_mjcf.py` — URDF to MJCF converter
- `core/validation/validator.py` — URDF validation (schema, collisions, inertia, CoM)
- Tauri commands: `sim_load`, `sim_step`, `sim_reset`, `sim_set_control`, `sim_get_state`, `sim_render`
- Sim bar UI in viewport (play/pause/reset/timeline) — wired but not connected to live data
- Component presets with full physical constants (mass, inertia, joint limits, actuator specs)

## Layer 1: URDF to Sim Pipeline
1. Take current URDF from editor
2. Run validation — block if physics-breaking issues (non-positive-definite inertia, missing mass, etc.)
3. Convert URDF to MJCF via `urdf_to_mjcf.py`
4. Inject actuator definitions from preset specs (torque limits, damping, armature from `sim_metadata`)
5. Load MJCF into MuJoCo via `sim_load`
6. **Key work**: Ensure converter handles preset-derived components correctly — multi-visual links should collapse to single collision primitives, actuator params should map to MuJoCo actuator elements

## Layer 2: Sim Loop to Viewport Sync
1. User clicks "Play" in sim bar → starts a sim loop
2. Each frame (~60Hz): call `sim_step` → `sim_get_state` returns `{joint_name: {pos, vel}}` dict
3. Map MJCF joint names to Three.js joint groups via `parsedRobot.joints`
4. Apply joint positions as rotations/translations on the Three.js groups
5. Viewport becomes a live physics visualizer
6. Play/Pause/Reset/Step controls work on the sim state
7. Timeline scrubber shows sim time
8. **Key work**: The joint name mapping (MJCF names may differ from URDF names after conversion), and the rotation axis alignment (URDF axis → Three.js rotation)

## Layer 3: Control Interface
1. **Joint sliders panel**: When sim is active, show a slider per actuated joint in a sidebar panel
2. **Gravity toggle**: Let user disable gravity to test in zero-G
3. **Preset controllers**: For common archetypes:
   - Quadruped: stand, trot gait, sit
   - Arm: home position, reach target
   - Mobile base: drive forward/turn
4. **Python script runner**: User writes a control script, it runs alongside sim and sends commands
5. **Key work**: The preset controllers need actual robotics logic (inverse kinematics for arms, gait generators for legs). Start with joint sliders, add presets later.

## Layer 4: Visualization (stretch)
- Contact forces as arrows in Three.js
- CoM trajectory trail
- MuJoCo camera PiP view (from `sim_render`)
- Joint torque heatmap on the model
- Ground reaction force visualization

## Implementation Order
1. Wire "Simulate" button → validate URDF → convert to MJCF → load into MuJoCo
2. Implement sim step loop with joint state readback
3. Apply joint states to Three.js viewport in real-time
4. Play/Pause/Reset controls
5. Joint slider panel
6. Timeline scrubber
7. Gravity toggle
8. Preset controllers (stretch)

## Key Risks
- **URDF→MJCF conversion quality**: MuJoCo is strict about inertia tensors and actuator defs. May need to auto-fix common issues (add default inertia to massless links, clamp extreme values).
- **Performance**: Python sim loop + JSON-RPC + IPC overhead. May need to batch multiple steps per IPC call or move to a faster protocol.
- **Joint axis mapping**: URDF and MuJoCo may interpret axes differently. Need careful testing per joint type.
