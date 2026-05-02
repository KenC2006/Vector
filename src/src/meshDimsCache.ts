// Rendered-mesh AABB cache. This is observed viewport data, not placement
// authority. Runtime code may record it for diagnostics and visual inspection,
// but component sizing should come from the visual resolver / authored preset
// dimensions so behavior does not depend on async mesh load order.

import * as THREE from 'three'

const meshDimsCache = new Map<string, THREE.Vector3>()

/** Return the observed rendered mesh size (full extents, meters) for a
 *  component, or null if not yet loaded. */
export function getRenderedMeshDims(compId: string): THREE.Vector3 | null {
  return meshDimsCache.get(compId) ?? null
}

/** Record a component's rendered mesh size for diagnostics. */
export function setRenderedMeshDims(compId: string, dims: THREE.Vector3): void {
  meshDimsCache.set(compId, dims.clone())
}

/** Reset cache, used by viewport reset / preset reload paths. */
export function clearRenderedMeshDimsCache(): void {
  meshDimsCache.clear()
}
