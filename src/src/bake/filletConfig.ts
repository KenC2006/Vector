// Phase 4: fillet radius + selector policy, resolved per joint.
//
// The placement engine already classifies mates via `mate_type` (fastened /
// planar / cylindrical), so the bake can pick a fillet radius that reads
// sensibly for that mate geometry. Different mate types tolerate different
// radii:
//
//   fastened  — flush bolt/weld: small 0.3mm chamfer-style fillet
//   planar    — flat mating faces (brackets, plates): 0.8mm
//   cylindrical — shaft-in-bore or disc-on-shaft: 1.2mm, forgives OCCT's
//                 fondness for failing on tight radii (shrink-retry still
//                 halves aggressively)
//
// Per-preset or per-preset-pair overrides land in `FILLET_OVERRIDES`. The
// plan's "Per-joint override mechanism" (Phase 4) lives here — known
// problem topologies (bracket+disc is too small to carry a fillet) get
// `skipFillet: true` so we don't waste 4 retry passes per bake.

export type MateType = 'fastened' | 'planar' | 'cylindrical' | string

export interface FilletPolicy {
  /** Target fillet radius in mm. Worker shrinks by /2 up to 3 times on reject. */
  radiusMm: number
  /** Slab thickness around the mate plane for the inBox filter (mm).  */
  slabMm: number
  /** Lateral (tangent-plane) half-extent of the fillet-edge filter box (mm).
   *  Should cover the child footprint with a small margin. Undefined tells
   *  the caller to derive from the child preset's bounding box. */
  lateralHalfMm?: number
  /** When true, skip the fillet attempt entirely. Fuse still runs. */
  skipFillet?: boolean
}

const DEFAULTS_BY_MATE_TYPE: Record<string, FilletPolicy> = {
  fastened:    { radiusMm: 0.3, slabMm: 0.2 },
  planar:      { radiusMm: 0.8, slabMm: 0.3 },
  cylindrical: { radiusMm: 1.2, slabMm: 0.4 },
}

const FALLBACK: FilletPolicy = { radiusMm: 0.6, slabMm: 0.3 }

/** Per-preset pair overrides. Key = `${parentPresetId}::${childPresetId}`
 *  (either can be '*' as wildcard). First match wins; list is order-sensitive. */
const FILLET_OVERRIDES: Array<{ parent: string; child: string; policy: Partial<FilletPolicy> }> = [
  // Bracket plate is too thin (1.6mm) to carry a visible fillet without
  // OCCT's "fillet propagates past geometry" error. Skip for any child.
  { parent: 'structural_bracket_l', child: '*', policy: { skipFillet: true } },
  { parent: 'structural_bracket_u', child: '*', policy: { skipFillet: true } },
  // Servo + coupler: works at 0.5mm per the Phase 2 smoke. Force that as the
  // starting point (worker still shrinks if even that rejects).
  { parent: 'actuator_servo_standard', child: 'structural_servo_coupler_disc', policy: { radiusMm: 0.5 } },
  { parent: 'actuator_servo_high_torque', child: 'structural_servo_coupler_disc', policy: { radiusMm: 0.5 } },
  { parent: 'actuator_servo_heavy_duty', child: 'structural_servo_coupler_disc', policy: { radiusMm: 0.5 } },
]

export function resolveFilletPolicy(
  mateType: MateType | undefined,
  parentPresetId: string,
  childPresetId: string,
): FilletPolicy {
  const base = (mateType && DEFAULTS_BY_MATE_TYPE[mateType]) || FALLBACK
  for (const ovr of FILLET_OVERRIDES) {
    const pm = ovr.parent === '*' || ovr.parent === parentPresetId
    const cm = ovr.child === '*' || ovr.child === childPresetId
    if (pm && cm) return { ...base, ...ovr.policy }
  }
  return base
}
