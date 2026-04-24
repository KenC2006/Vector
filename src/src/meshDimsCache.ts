// Rendered-mesh AABB cache — populated by richVisuals/index.ts after a GLB
// or parametric mesh is scaled into the scene, read by placement / validator
// consumers that want the true rendered size rather than the authored
// bounding_box_mm field on the preset.
//
// Extracted from richVisuals/index.ts so pure/node-runnable modules
// (topologyValidation.ts, graphMutations.ts, mateCorpus.ts) can query the
// cache through componentDims without pulling in the entire rich-visual
// generator tree (which fails to resolve under native node ESM because
// `./generators` is a directory import).

import * as THREE from 'three'

const meshDimsCache = new Map<string, THREE.Vector3>()

/** Return the actual rendered mesh size (full extents, meters) for a
 *  component, or null if not yet loaded. */
export function getRenderedMeshDims(compId: string): THREE.Vector3 | null {
  return meshDimsCache.get(compId) ?? null
}

/** Record a component's rendered mesh size. Called after per-axis scaling
 *  of a GLB or after a parametric placeholder is built. */
export function setRenderedMeshDims(compId: string, dims: THREE.Vector3): void {
  meshDimsCache.set(compId, dims.clone())
}

/** Reset cache — used by viewport reset / preset reload paths. */
export function clearRenderedMeshDimsCache(): void {
  meshDimsCache.clear()
}
