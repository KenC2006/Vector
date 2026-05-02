// Component bounding-box accessor.
//
// Single read path for "how big is this component, in millimeters?"
// Placement, validation, and corpus tests should agree on authored component
// dimensions instead of depending on whether an async viewport mesh happened
// to have loaded first.

export interface DimsPresetLike {
  physical: {
    bounding_box_mm?: number[]
    cross_section_mm?: number[]
  }
}

export interface DimsInstanceLike {
  length_mm?: number
}

/** Returns [x, y, z] in millimeters. Priority:
 *  1. Authored bounding_box_mm
 *  2. cross_section_mm (parametric extrusions; length is supplied per-instance
 *     elsewhere; caller splices in length_mm before passing the preset)
 *  3. [40, 40, 40] hardcoded default
 */
export function getOrComputeBbox(_componentId: string, preset: DimsPresetLike): [number, number, number] {
  const phys = preset.physical
  const bb = phys.bounding_box_mm ?? phys.cross_section_mm ?? [40, 40, 40]
  return [bb[0] ?? 40, bb[1] ?? 40, bb[2] ?? 40]
}

export function getAuthoredHalfBoundsMm(
  preset: DimsPresetLike,
  instance?: DimsInstanceLike,
): { hxMm: number; hyMm: number; hzMm: number } {
  const phys = preset.physical
  const bb = getOrComputeBbox('', preset)
  const zMm = phys.cross_section_mm && instance?.length_mm !== undefined
    ? instance.length_mm
    : bb[2]
  return {
    hxMm: (bb[0] ?? 40) / 2,
    hyMm: (bb[1] ?? 40) / 2,
    hzMm: (zMm ?? 40) / 2,
  }
}
