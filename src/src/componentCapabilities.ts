// Data-driven component capabilities — derived from authored preset data
// (connector `cls` + `sim_metadata`), replacing component-id prefix matching.
//
// The placement engine used to decide "is this a servo / a tire / a
// drivetrain?" by string-matching component ids (`actuator_servo*`,
// `mobility_wheel_*`, `drivetrain_*`). That made the catalog data
// (connector classes authored in generic_presets.json) decorative: a BLDC
// authored an output shaft exactly like a servo, but never received the
// body/horn split because its id didn't match. Capabilities make the
// authored data the single source of truth:
//
//   - `cls: 'shaft'` connector + a `sim_metadata.mjcf_actuator_type` →
//     rotary actuator that compiles to a fixed body + rotating output
//     ("horn") split, with the output frame at the AUTHORED connector
//     origin.
//   - `sim_metadata.contact_class: 'wheel'` → rolling contact (tires,
//     casters): coin-flat local frame, stood upright by the engine.
//   - `sim_metadata.contact_class: 'drivetrain'` → chassis-to-wheel
//     hardware (hub motors, axles, knuckles): keeps the dedicated
//     outboard/side-flip mounting idiom and never splits (its shaft is
//     where the tire mates, not a separate link).
//
// Passive shaft hardware (transmissions, couplers, potentiometers) authors
// `cls: 'shaft'` but no actuator type, so it never splits — matching the
// pre-capability behavior.

import type { MateConnector } from './mateConnectors.ts'

/** Minimal slice of a preset spec needed to derive capabilities. Raw catalog
 * JSON satisfies this directly. */
export interface CapabilitySourceSpec {
  id: string
  connectors?: MateConnector[]
  sim_metadata?: Record<string, unknown>
}

export interface ComponentCapabilities {
  /** Authored rotary output connector (`cls: 'shaft'`), if any. Prefers a
   * `single: true` connector when several are classed shaft (e.g. couplers
   * author shaft_in + shaft_out; neither is single, first wins). Origin/axis
   * are in the preset's local frame, mm — by catalog convention the output
   * axis is local +Z. */
  shaftConnector: MateConnector | null
  /** Authored hub bore (`cls: 'bore'`), if any — where a shaft inserts. */
  boreConnector: MateConnector | null
  /** `sim_metadata.contact_class` verbatim (wheel / drivetrain / foot / …). */
  contactClass: string | null
  /** Rotary actuator that compiles to a body + output ("horn") split. */
  splitRotary: boolean
  /** Rolling contact hardware (tires, casters). */
  wheel: boolean
  /** Chassis-to-wheel drivetrain hardware (hub motors, axles, casters' forks). */
  drivetrain: boolean
}

const ROTARY_ACTUATOR_TYPES = new Set(['position', 'velocity', 'motor'])

export function capabilitiesForSpec(spec: CapabilitySourceSpec): ComponentCapabilities {
  const conns = spec.connectors ?? []
  const shaftConns = conns.filter(c => c.cls === 'shaft')
  const shaftConnector = shaftConns.find(c => c.single === true) ?? shaftConns[0] ?? null
  const boreConns = conns.filter(c => c.cls === 'bore')
  const boreConnector = boreConns.find(c => c.single === true) ?? boreConns[0] ?? null

  const sim = (spec.sim_metadata ?? {}) as Record<string, unknown>
  const contactClass = typeof sim.contact_class === 'string' ? sim.contact_class : null
  const actuatorType = typeof sim.mjcf_actuator_type === 'string' ? sim.mjcf_actuator_type : null

  const wheel = contactClass === 'wheel'
  const drivetrain = contactClass === 'drivetrain'
  const splitRotary =
    shaftConnector !== null &&
    actuatorType !== null &&
    ROTARY_ACTUATOR_TYPES.has(actuatorType) &&
    !wheel &&
    !drivetrain

  return { shaftConnector, boreConnector, contactClass, splitRotary, wheel, drivetrain }
}
