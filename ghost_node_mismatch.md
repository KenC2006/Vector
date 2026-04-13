# Ghost Outline / Attachment Node Mismatch

## Context

When placing a component in carry mode, the ghost outline box and the attachment node ring positions don't align with the component's actual rendered geometry. This causes snapping to feel off and nodes to appear displaced from the component's visual faces.

---

## Three Sources of Truth (the core problem)

There are three independent systems that each independently derive component dimensions, and they don't agree:

| System | Source | Used By |
|--------|--------|---------|
| Ghost outline | `generateVisuals(comp)` — parametric preset definitions | `computeCarryGhostBounds()` → `enterCarryMode()` |
| Rendered mesh | GLB/STEP file, scaled mm→m and centered | What the user sees |
| Node positions | `computeLinkLocalBoundingBox()` — walks actual Three.js scene | `rebuildMountNodes()` |

---

## Problem 1: Ghost uses preset parametric bounds, nodes use actual rendered bounds

**Where:** `urdfAssembly.ts`
- Ghost: `computeCarryGhostBounds()` (line ~1557) → calls `generateVisuals(comp)` → derives AABB from URDF primitive definitions (box sizes, cylinder radii)
- Nodes: `rebuildMountNodes()` (line ~528) → calls `computeLinkLocalBoundingBox()` (line ~493) → walks actual Three.js scene meshes

**Effect:** For any component where the GLB/rich visual differs from the URDF primitives, the ghost box and the node face positions are computed from different geometries and end up misaligned.

**Fix idea:** Make both use the same source. The cleanest approach is to have `computeCarryGhostBounds` also call `computeLinkLocalBoundingBox` on the component's link group (if it already exists in the scene) instead of re-deriving from presets. If the link group isn't in the scene yet (new placement), fall back to the `generateVisuals` path as today.

---

## Problem 2: Non-extrusion GLB meshes not scaled to match URDF dimensions

**Where:** `richVisuals/index.ts`, `applyMeshToLink()` (line ~258)

The uniform mm→m scale (`meshGroup.scale.setScalar(0.001)`) is applied to all GLB meshes, but per-axis scaling to match URDF primitive dimensions is only applied to components where `compId.includes('extrusion')` (lines ~303–315). For everything else (servos, sensors, motors, grippers), the GLB is loaded at its raw manufacturer proportions after the mm→m conversion.

Since STEP geometry rarely matches simplified URDF primitives exactly, the visible mesh and the ghost outline are different sizes. A servo GLB from the manufacturer might be 42×22×58mm after conversion, while the URDF primitive says 40×20×55mm.

**Fix idea — Option A (preferred):** After loading and scaling the GLB, compute its actual bounding box and store it alongside the link. Use this stored size as the authoritative dimension source for both the ghost and nodes. This makes the rendered mesh the source of truth.

**Fix idea — Option B:** Extend the per-axis scaling beyond extrusions to all non-articulated components (anything where distorting the shape is acceptable). Probably not right for manufacturer meshes.

**Fix idea — Option C:** Measure the post-scale GLB bounding box and write it back into the component's `physical.bounding_box_mm` so downstream dimension consumers (ghost, nodes) all read from one place.

---

## Problem 3: Async GLB loading doesn't trigger `rebuildMountNodes` (most impactful)

**Where:** `richVisuals/index.ts`, `loadMeshOverride()` (line ~344) and `urdfAssembly.ts`, `rebuildMountNodes()` (line ~528)

`loadMeshOverride` is async — GLBs arrive ~100ms+ after `applyRichVisuals` returns. At parse time, `computeLinkLocalBoundingBox` measures the parametric fallback geometry (the URDF primitive box/cylinder). When the GLB loads later, `applyMeshToLink` swaps in the real mesh — but there is no callback back into `urdfAssembly.ts` to re-run `rebuildMountNodes`. Nodes are permanently placed based on stale fallback geometry, even after the GLB is fully rendered.

This is the most impactful bug: a robot loaded with several STEP/GLB overrides will have all its nodes permanently misplaced until the URDF is manually reparsed.

**Fix idea:** `applyRichVisuals` (or `loadMeshOverride`) needs a callback. Add an optional `onMeshLoaded?: (linkName: string) => void` parameter to `applyRichVisuals`. In `urdfAssembly.ts`, pass a callback that calls `rebuildMountNodes()` (debounced — one rebuild after all GLBs settle, not one per component). Wire the callback through `initUrdfAssembly`'s deps so it can trigger the rebuild cleanly without a circular import.

---

## Problem 4: Ghost is always a box regardless of actual shape

**Where:** `urdfAssembly.ts`, `enterCarryMode()` (line ~1884)

```typescript
const geo = new THREE.BoxGeometry(hx * 2, hy * 2, hz * 2)
```

The ghost is always a box even for cylindrical components (BLDC motors, bearings, wheels). For a cylinder, the ghost is a rectangular prism that encloses the cylinder — visually wrong.

**Fix idea:** Inspect the dominant geometry type from `generateVisuals(comp)`. If all visuals are cylinders (or the component has a single dominant cylinder), use a `CylinderGeometry` (or `EdgesGeometry` of a cylinder) for the ghost instead. Could also allow a `SphereGeometry` for sphere-dominant components. The `computeCarryGhostBounds` return value would need a `shape: 'box' | 'cylinder' | 'sphere'` field.

---

## Implementation Order (suggested)

1. **Problem 3** — Add `onMeshLoaded` callback from `richVisuals` to `urdfAssembly` to re-run `rebuildMountNodes` after async GLB loads settle. This is the highest-impact fix and unblocks accurate node placement for most real components.
2. **Problem 1** — After fixing Problem 3, make `computeCarryGhostBounds` use `computeLinkLocalBoundingBox` as its source when the link group is already in the scene.
3. **Problem 2** — Cache the post-scale GLB bounding box as the authoritative size. This ensures ghost, nodes, and any future dimension consumers all agree.
4. **Problem 4** — Add shape detection to `computeCarryGhostBounds` and use the right Three.js geometry primitive for the ghost outline. Cosmetic but improves feedback for cylindrical parts.

---

## Key File Locations

| File | Relevant Function | Line (approx) |
|------|------------------|---------------|
| `src/src/urdfAssembly.ts` | `computeCarryGhostBounds()` | ~1557 |
| `src/src/urdfAssembly.ts` | `enterCarryMode()` | ~1871 |
| `src/src/urdfAssembly.ts` | `computeLinkLocalBoundingBox()` | ~493 |
| `src/src/urdfAssembly.ts` | `rebuildMountNodes()` | ~528 |
| `src/src/urdfAssembly.ts` | `getCarrySourceNodes()` | ~1927 |
| `src/src/richVisuals/index.ts` | `applyRichVisuals()` | ~161 |
| `src/src/richVisuals/index.ts` | `applyMeshToLink()` | ~258 |
| `src/src/richVisuals/index.ts` | `loadMeshOverride()` | ~344 |
| `src/src/richVisuals/index.ts` | `measureLinkDims()` | ~85 |
