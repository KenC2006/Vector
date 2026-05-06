// Rendered-mesh AABB cache. This is observed viewport data, not placement
// authority. Runtime code may record it for diagnostics and visual inspection,
// but component sizing should come from the visual resolver / authored preset
// dimensions so behavior does not depend on async mesh load order.

import * as THREE from 'three'

const meshDimsCache = new Map<string, THREE.Vector3>()

/** Record a component's rendered mesh size for diagnostics. The cache is
 *  not consulted by placement, sizing, or any non-diagnostic path — it
 *  exists so the viewport can show observed-vs-declared mesh extents. */
export function setRenderedMeshDims(compId: string, dims: THREE.Vector3): void {
  meshDimsCache.set(compId, dims.clone())
}
