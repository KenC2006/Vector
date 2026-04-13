# Mesh / Node Placement Mismatch

## Summary

The ghost outline, attachment node rings, and rendered mesh all agree with **each other** — they all derive from the actual GLB bounding box. The mismatch is between the actual GLB size and what the preset's `bounding_box_mm` declares. Nodes sit correctly on the rendered mesh but in the wrong place relative to the component's stated dimensions.

---

## Problem 1 — Multiple components share one GLB (biggest impact)

**Files:** `src/src/richVisuals/meshOverrides.ts`

Many preset IDs map to the same physical file. After uniform mm→m scaling, every component sharing a file renders at **identical physical dimensions**, ignoring their individual `bounding_box_mm` values.

Worst cases:

| Components | Shared file | Declared sizes |
|---|---|---|
| `actuator_linear_small` / `actuator_linear_heavy` / `actuator_micro_linear_servo` | `linear_actuator.glb` | 16×16×130 mm vs 30×30×350 mm |
| `actuator_servo_micro` / `actuator_high_speed_mini_servo` | `servo_small.glb` | 23×12×29 mm vs different |
| `actuator_servo_heavy_duty` / `actuator_servo_high_torque` | `servo_high_torque.glb` | different masses/dims |
| `actuator_bldc_small` / `actuator_bldc_large` | `bldc_outrunner.glb` | different radii/heights |
| `motor_hub_80mm` / `motor_hub_120mm` | `motor_hub.glb` | 80 mm vs 120 mm diameter |
| `motor_harmonic_drive_compact` / `motor_harmonic_drive_large` | `motor_harmonic_drive.glb` | different |
| `sensor_depth_camera_small` / `sensor_depth_camera_wide` | `depth_camera.glb` | different FOV/size |
| `sensor_imu_6dof` / `sensor_imu_9dof` | `imu_board.glb` | same board, different dims claimed |
| `sensor_contact_switch` / `sensor_limit_switch` | `sensor_limit_switch.glb` | different |
| `power_lipo_3s_2200` / `power_lipo_4s_5000` / `power_lipo_6s_10000` | `battery_lipo.glb` | 3s vs 4s vs 6s — very different |
| `power_buck_converter_5v` / `power_buck_converter_12v` | `power_buck_converter.glb` | different |
| `power_18650_cell_holder` / `power_18650_4s2p_battery` | `power_cell_holder.glb` | different |
| `structural_hex_standoff_m3` / `structural_hex_standoff_m4` | `structural_standoff.glb` | different |
| `transmission_flexible_coupling` / `transmission_flexible_coupling_jaw` | `transmission_coupling.glb` | different |
| `effector_parallel_gripper_small` / `effector_parallel_gripper_large` | `gripper_parallel.glb` | small vs large |

---

## Problem 2 — Per-axis scaling only applied to extrusions

**File:** `src/src/richVisuals/index.ts`, `applyMeshToLink()` (~line 311)

```typescript
const isExtrusion = compId.includes('extrusion')
if (isExtrusion) {
  // scale X, Y, Z independently to match dims
}
// Everything else: only scale(0.001) is applied
```

Extrusions get rescaled to match their declared `bounding_box_mm`. Every other component (servos, motors, sensors, grippers, batteries) is scaled uniformly by 0.001 only. The `bounding_box_mm` field in the preset JSON has **no effect** on the actual rendered size of non-extrusion components — only the raw STEP geometry determines the rendered size.

**Fix direction:** Extend per-axis scaling to all components using `bounding_box_mm` as the target. Needs axis-orientation handling (the GLB's longest axis may not be Z).

---

## Problem 3 — `measureLinkDims` ignores visual group positions

**File:** `src/src/richVisuals/index.ts`, `measureLinkDims()` (~line 85)

```typescript
geometryGroup.traverse(child => {
  if (child instanceof THREE.Mesh) {
    const params = child.geometry.parameters
    // reads params.width/height/depth — NO position/offset
  }
})
```

For multi-piece components (e.g. linear actuator = body + extending rod + rear clevis), each visual piece has its own `<visual><origin>` which becomes `visualGroup.position` in Three.js. `measureLinkDims` reads raw geometry sizes but ignores these offsets. For the linear actuator:

- Main body: box depth=0.130m → `dims.z = 0.130`
- Rod: cylinder h=0.052m sitting at z=+0.091 → actual tip at z=+0.117
- Clevis: box at z=−0.067

`measureLinkDims` returns `dims.z = 0.130` but the actual parametric extent is ~0.200m. This `dims` is passed into `applyMeshToLink` where it's used as the mm→m detection threshold (`maxExpectedDim * 10`). Detection still works in practice because the raw GLB in mm is always far larger, but the hint is wrong and could cause edge-case failures.

---

## Problem 4 — Different mm→m detection thresholds in preload vs apply

**Files:**
- `src/src/richVisuals/index.ts`, `preloadMeshCache()` (~line 489): uses `maxDim > 1.0`
- `src/src/richVisuals/index.ts`, `applyMeshToLink()` (~line 302): uses `maxMeshDim > maxExpectedDim * 10`

```typescript
// preloadMeshCache:
if (maxDim > 1.0) rawSize.multiplyScalar(0.001)

// applyMeshToLink:
if (maxMeshDim > maxExpectedDim * 10) meshGroup.scale.setScalar(0.001)
```

Currently these happen to agree for all components in the library. But they could diverge for a component where `maxExpectedDim` is between 0.1m and 1.0m and the raw GLB happens to be in an intermediate range. A latent inconsistency.

---

## How the data flows (for reference)

```
preset bounding_box_mm
        │
        ▼
measureLinkDims()          ← reads URDF primitive geometry params, ignores visual origins
        │
        ▼
applyMeshToLink(dims)      ← dims only used for mm→m detection + extrusion scaling
        │                     GLB size is otherwise taken as-is after 0.001 scale
        ▼
meshDimsCache              ← stores actual GLB size (full extents in meters)
        │
        ├──► computeCarryGhostBounds()   → ghost outline + ghost face nodes
        │                                   uses meshDimsCache, cx=cy=cz=0
        │
        └──► rebuildMountNodes()         → placed component attachment rings
                │                          uses computeLinkLocalBoundingBox()
                ▼
         computeLinkLocalBoundingBox()  ← walks actual Three.js scene meshes
                                           gives actual GLB bbox in link-local space
```

Ghost and placed nodes agree with each other. Neither agrees with `bounding_box_mm` for non-extrusion components.

---

## Key file locations

| File | Function | Issue |
|---|---|---|
| `src/src/richVisuals/meshOverrides.ts` | `MESH_OVERRIDES` map | shared GLB files |
| `src/src/richVisuals/index.ts` ~L85 | `measureLinkDims()` | ignores visual origin offsets |
| `src/src/richVisuals/index.ts` ~L266 | `applyMeshToLink()` | per-axis scaling extrusion-only |
| `src/src/richVisuals/index.ts` ~L302 | mm→m detection in `applyMeshToLink` | `> maxExpectedDim * 10` |
| `src/src/richVisuals/index.ts` ~L489 | mm→m detection in `preloadMeshCache` | `> 1.0` (different threshold) |
| `src/src/urdfAssembly.ts` ~L499 | `computeLinkLocalBoundingBox()` | correct, but depends on GLB being loaded |
| `src/src/urdfAssembly.ts` ~L534 | `rebuildMountNodes()` | node placement driven by above |
| `src/src/urdfAssembly.ts` ~L1557 | `computeCarryGhostBounds()` | uses meshDimsCache with cx=cy=cz=0 |
