// Component bounding-box accessor.
//
// Single read path for "how big is this component, in millimeters?" Step 3 of
// docs/ENGINE_NEXT_STEPS.md demoted preset.physical.bounding_box_mm from an
// authored source-of-truth to a derived helper: when the GLB has rendered,
// the actual mesh AABB is authoritative; the authored field only fills in
// for legacy presets and for the pre-load placeholder phase.
//
// Why this exists as a tiny helper rather than inline in each call site:
// the drift class that bit Layer 1 (`structural_baseplate_large` 8mm bbox vs
// 30mm rendered mesh) lives wherever a reader is allowed to consult the
// authored value directly. Funneling all reads through getOrComputeBbox()
// guarantees a single place to keep the priority order honest.

import { getRenderedMeshDims } from './meshDimsCache.ts'

export interface DimsPresetLike {
  physical: {
    bounding_box_mm?: number[]
    cross_section_mm?: number[]
  }
}

/** Returns [x, y, z] in millimeters. Priority:
 *  1. Rendered mesh AABB (after applyMeshToLink populates the cache)
 *  2. Authored bounding_box_mm
 *  3. cross_section_mm (parametric extrusions; length is supplied per-instance
 *     elsewhere — caller splices in length_mm before passing the preset)
 *  4. [40, 40, 40] hardcoded default
 *
 *  When the rendered cache is populated, the values reflect the post-scale
 *  GLB extents (in meters internally, converted to mm here). When a preset's
 *  GLB hasn't loaded yet — or in node corpus runs that never load meshes —
 *  the authored field is used. Both paths produce comparable mm tuples so
 *  callers don't need to branch.
 */
export function getOrComputeBbox(componentId: string, preset: DimsPresetLike): [number, number, number] {
  const rendered = getRenderedMeshDims(componentId)
  if (rendered && rendered.x > 0.001 && rendered.y > 0.001 && rendered.z > 0.001) {
    return [rendered.x * 1000, rendered.y * 1000, rendered.z * 1000]
  }
  const phys = preset.physical
  const bb = phys.bounding_box_mm ?? phys.cross_section_mm ?? [40, 40, 40]
  return [bb[0] ?? 40, bb[1] ?? 40, bb[2] ?? 40]
}
