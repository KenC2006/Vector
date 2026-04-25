// Per-preset bake-source overrides.
//
// When a preset's authored STEP is too hostile for OCCT booleans (see
// `memory/project_coupler_step_overmodeled.md` — coupler's 974-edge splined
// bore hangs both simplify() and fuse()), we swap in a parametric stand-in
// at bake time. The per-preset display path is unchanged.
//
// Dimensions are chosen to match the preset's authored bounding box so the
// bake output sits at the same footprint as the preset's GLB placeholder.

export type BakeSourceOverride =
  | { kind: 'disc'; odMm: number; idMm: number; thicknessMm: number }
  | {
      /** Parametric box. `sizeMm` is usually fixed for non-parametric presets.
       *  For parametric ones (extrusions), pass `lengthField: 'length_mm'`
       *  and the caller reads `component.length_mm` to fill the length
       *  dimension. `lengthAxis` picks which of x/y/z is the variable one. */
      kind: 'box'
      sizeMm?: [number, number, number]
      lengthField?: 'length_mm'
      lengthAxis?: 0 | 1 | 2
      crossSectionMm?: [number, number]  // the two non-length dims
    }
  | {
      /** T-slot aluminum extrusion: bar with a wide groove cut into each
       *  face + a thin center ridge inside each groove. Length in Z,
       *  derived from component.length_mm. */
      kind: 'extrusion'
      crossSectionMm: [number, number]
      slotWidthMm: number
      slotDepthMm: number
      ridgeWidthMm: number
      ridgeHeightMm: number
    }

/** Lookup table. Keyed by preset id. Returns undefined when the preset should
 *  be baked from its authored STEP (the default). */
export const BAKE_SOURCE_OVERRIDES: Record<string, BakeSourceOverride> = {
  // 25-tooth spline bore in the goBILDA 1908 model dominates OCCT's boolean
  // algorithm. Replace with a plain disc (same bbox: 32×32×8mm, 8mm bore).
  'structural_servo_coupler_disc': {
    kind: 'disc', odMm: 32, idMm: 8, thicknessMm: 8,
  },

  // Parametric extrusions — bake cuts a groove into each face and leaves
  // a thin center ridge inside each groove, producing an authentic 80/20-
  // style T-slot profile. Groove width MUST stay well below the cross
  // section or the four cuts overlap at the corners and disconnect the
  // bar into floating pieces. Real 2020 opening is ~6 mm on a 20 mm bar,
  // real 4040 is ~8 mm on 40 mm — keeping to roughly 40% of the cross
  // section keeps ~6 mm corner webs intact.
  //   2020: 8 mm groove × 3 mm deep, 2.5 mm ridge × 2 mm tall
  //   4040: 14 mm groove × 5 mm deep, 4 mm ridge × 3.5 mm tall
  'structural_extrusion_2020': {
    kind: 'extrusion', crossSectionMm: [20, 20],
    slotWidthMm: 8, slotDepthMm: 3, ridgeWidthMm: 2.5, ridgeHeightMm: 2,
  },
  'structural_extrusion_4040': {
    kind: 'extrusion', crossSectionMm: [40, 40],
    slotWidthMm: 14, slotDepthMm: 5, ridgeWidthMm: 4, ridgeHeightMm: 3.5,
  },

  // Parametric baseplate — no STEP source at all. Per-preset renders a URDF
  // <box> from bounding_box_mm. Without this override, the baseplate cluster
  // aborts with "no STEP source" and all its fixed-joined electronics fall
  // back to per-preset render, which defeats the point of bake.
  'structural_baseplate_large': {
    kind: 'box', sizeMm: [350, 250, 8],
  },
  'structural_baseplate': {
    kind: 'box', sizeMm: [200, 150, 6],
  },

  // LiPo batteries — meshOverrides.ts references `battery_lipo.step` but
  // that file is missing from the catalog (only the pre-converted GLB
  // shipped). OCCT falls back to parsing the 404 HTML response and fails,
  // aborting whichever cluster contains the battery. Substitute with a
  // parametric box matching each preset's bounding_box_mm.
  'power_lipo_3s_2200': { kind: 'box', sizeMm: [105, 34, 24] },
  'power_lipo_4s_5000': { kind: 'box', sizeMm: [137, 42, 30] },
  'power_lipo_6s_10000': { kind: 'box', sizeMm: [165, 65, 42] },

  // Electronics with very large STEP files — Replicad's importSTEP can't
  // finish parsing them inside the 30s cluster timeout (sbc_small.step is
  // 17.5 MB, depth_camera.step is 14.6 MB). They're rectangular boards so
  // a bbox-derived box is a fine approximation; the seams aren't load-
  // bearing visually since these mount flush on the baseplate.
  'compute_sbc_small': { kind: 'box', sizeMm: [85, 56, 17] },
  'sensor_depth_camera_small': { kind: 'box', sizeMm: [90, 25, 25] },
  'sensor_depth_camera_wide': { kind: 'box', sizeMm: [124, 29, 26] },
}

export function getBakeSourceOverride(presetId: string): BakeSourceOverride | undefined {
  return BAKE_SOURCE_OVERRIDES[presetId]
}
