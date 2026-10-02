/**
 * The design model: an explicit-pose part list compiled to URDF by
 * core/designer/compile.py. Every editor (component placement, gizmo,
 * inspector, AI chat) edits this and recompiles; the URDF is derived.
 *
 * Units: millimetres and degrees. World frame: X forward, Y left, Z up.
 */

export type Vec3 = [number, number, number]

/** A direction: '+x' | '-y' | 'up' | … or three numbers. */
export type AxisSpec = string | Vec3

/** Where a part (or a joint pivot) goes: a world point, 'part' / 'part.anchor',
 *  or an anchor reference plus a world-frame offset. */
export type AtSpec = Vec3 | string | { ref: string; offset?: Vec3 }

export type JointType = 'fixed' | 'revolute' | 'continuous' | 'prismatic'

export interface JointSpec {
  type: JointType
  name?: string
  rest_deg?: number
  lower_deg?: number
  upper_deg?: number
  lower_mm?: number
  upper_mm?: number
  pivot?: AtSpec
  axis?: AxisSpec
  passive?: boolean
  effort?: number
  velocity?: number
}

export interface Primitive {
  name?: string
  shape: 'box' | 'cylinder' | 'sphere' | 'mesh'
  size_mm?: Vec3
  radius_mm?: number
  length_mm?: number
  xyz_mm?: Vec3
  rpy_deg?: Vec3
  color?: [number, number, number]
  filename?: string
  scale?: Vec3
}

export interface DesignPart {
  name: string
  component?: string
  length_mm?: number
  shape?: Primitive[]
  parent?: string
  at?: AtSpec
  align?: string
  z_axis?: AxisSpec
  x_axis?: AxisSpec
  spin_deg?: number
  joint?: JointSpec | JointType
  mirror?: boolean
  link?: string
  mirror_link?: string
  mass_kg?: number
  color?: [number, number, number]
}

export interface Design {
  name: string
  summary?: string
  parts: DesignPart[]
}

/** 3x3 rotation as rows (columns are the part's local axes in world). */
export type Mat3 = [Vec3, Vec3, Vec3]

export interface CompiledPart {
  link: string
  component: string | null
  parent: string | null
  mirror_of: string | null
  /** Part origin in the design world (mm). */
  p: Vec3
  R: Mat3
  /** URDF link frame origin (the joint pivot for moving parts). */
  frame_p: Vec3
  size: Vec3
  center_local: Vec3
  joint: { type: JointType; passive?: boolean; pivot?: Vec3; axis?: Vec3 } | null
}

export interface CompiledDesign {
  urdf: string
  root: string
  parts: Record<string, CompiledPart>
  issues: string[]
}
