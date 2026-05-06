// Phase 3b — joint type / axis / limit helpers extracted from urdfAssembly.ts.
//
// Pure: no THREE, no DOM, no scene state. Preset reads are tolerant (`any`)
// because the loader normalizes catalog JSON into ComponentSpec but
// sim_metadata / mechanical_electrical fields remain loosely typed.

import type { AssemblyJointType } from './index.ts'

export const DEFAULT_REVOLUTE_LIMIT_RAD = Math.PI / 2

export function resolveJointLimitsRad(preset: any): [number, number] {
  const sim = preset?.sim_metadata || {}
  const me = preset?.mechanical_electrical || {}
  const deg = sim.mjcf_joint_limits_deg || me.angle_range_deg
  if (Array.isArray(deg) && deg.length === 2) {
    const lo = (Number(deg[0]) || 0) * Math.PI / 180
    const hi = (Number(deg[1]) || 0) * Math.PI / 180
    if (hi > lo) return [lo, hi]
  }
  return [-DEFAULT_REVOLUTE_LIMIT_RAD, DEFAULT_REVOLUTE_LIMIT_RAD]
}

export function normalizeJointType(value?: string): AssemblyJointType {
  const jointType = (value || '').toLowerCase()
  if (
    jointType === 'revolute' ||
    jointType === 'continuous' ||
    jointType === 'prismatic'
  ) {
    return jointType
  }
  return 'fixed'
}

export function axisNameToTuple(axis?: string): [number, number, number] {
  switch ((axis || 'z').toLowerCase()) {
    case 'x': return [1, 0, 0]
    case 'y': return [0, 1, 0]
    default: return [0, 0, 1]
  }
}

export function axisTupleToUrdf(axis: [number, number, number]): string {
  return axis.map(v => (Math.abs(v) < 1e-9 ? 0 : v)).join(' ')
}
