import type { MateConnector } from './mateConnectors'
import type { UrdfVisualDesc } from './componentMeshes'

export type ComponentEnvelopeAxis = 'x' | 'y' | 'z'
export type ComponentCollisionSource = 'authored_mesh' | 'urdf_primitives' | 'preset_bbox'
export type ComponentMeshScalePolicy = 'none' | 'uniform' | 'per-axis'

export interface ComponentParametricEnvelope {
  axis: ComponentEnvelopeAxis
  cross_section_mm: [number, number]
}

export interface ComponentMeshSpec {
  visual_file?: string
  collision_file?: string
  rotation_rpy?: [number, number, number]
  units?: 'm' | 'mm'
  scale_policy?: ComponentMeshScalePolicy
}

export interface ComponentPhysicalSpec {
  mass_kg?: number
  inertia_primitive?: 'box' | 'cylinder' | 'sphere' | string

  /** Current shipped schema. Kept during the migration; `bbox_mm` is the target name. */
  bounding_box_mm?: number[]
  /** Target fixed-envelope field from the unification plan. */
  bbox_mm?: [number, number, number]

  /** Current shipped parametric field. Kept during the migration. */
  cross_section_mm?: number[]
  /** Target parametric envelope field from the unification plan. */
  parametric?: ComponentParametricEnvelope

  mesh?: ComponentMeshSpec
  collision_mesh?: string

  /** Mesh audit marker for presets whose bbox intentionally differs from raw mesh extents. */
  legacy_override?: boolean
  /** Per-100mm mass for parametric extrusions; resolveComponentMassKg multiplies by length_mm/100. */
  mass_kg_per_100mm?: number
  mass_kg_per_100x100mm?: number

  [key: string]: unknown
}

export interface ComponentSpec {
  id: string
  name?: string
  description?: string
  category?: string
  physical: ComponentPhysicalSpec
  mechanical_electrical: Record<string, unknown>
  mounting_logic?: Record<string, unknown>
  connectors?: MateConnector[]
  [key: string]: unknown
}

export interface ComponentInstanceSpec {
  length_mm?: number
}

export type AttachmentNodeClass =
  | 'mount_face'
  | 'shaft'
  | 'bore'
  | 'rail'
  | 'generic'

export interface AttachmentNodeDef {
  nodeId: string
  label: string
  cls: AttachmentNodeClass
  /** Local frame relative to the component's main link frame (URDF coordinates). */
  origin_xyz: [number, number, number]
  origin_rpy: [number, number, number]
  /** Optional joint semantics for a connection made at this node. */
  kinematic?: {
    joint_type?: 'fixed' | 'revolute' | 'continuous' | 'prismatic'
    axis_xyz?: [number, number, number]
  }
  /** If true, only one connection may attach to this node. */
  single: boolean
}

export interface ComponentBoundsMm {
  half: [number, number, number]
  center: [number, number, number]
  shape: 'box' | 'cylinder'
}

/**
 * The logical-resolver record. For visuals + previewGroup + status, callers
 * use `resolveComponentVisual` from `componentVisualResolver.ts` — it layers
 * on top of this record.
 */
export interface ResolvedComponentRecord {
  id: string
  spec: ComponentSpec
  bounds: ComponentBoundsMm
  collision: {
    source: ComponentCollisionSource
    file?: string
    bounds: ComponentBoundsMm
    descriptors?: UrdfVisualDesc[]
  }
  ports: AttachmentNodeDef[]
  connectors: MateConnector[]
  warnings: string[]
}

export const DEPRECATED_COMPONENT_PHYSICAL_FIELDS = [
  'outer_diameter_mm',
  'inner_diameter_mm',
  'wall_thickness_mm',
  'diameter_mm',
  'thickness_mm',
] as const

export type DeprecatedComponentPhysicalField = typeof DEPRECATED_COMPONENT_PHYSICAL_FIELDS[number]

export interface ComponentSpecDeprecationFinding {
  componentId: string
  field: string
  message: string
}

/** Inspects a preset for vestigial fields slated for removal in Phase 6.
 *  Returns one finding per offending field; the loader emits each at most once. */
export function auditComponentSpecDeprecations(spec: {
  id?: string
  physical?: Record<string, unknown>
}): ComponentSpecDeprecationFinding[] {
  const findings: ComponentSpecDeprecationFinding[] = []
  const phys = spec.physical
  const id = spec.id ?? '<unknown>'
  if (!phys || typeof phys !== 'object') return findings
  for (const field of DEPRECATED_COMPONENT_PHYSICAL_FIELDS) {
    if (Object.prototype.hasOwnProperty.call(phys, field) && phys[field] !== undefined) {
      findings.push({
        componentId: id,
        field,
        message: `physical.${field} is deprecated; replace with bbox_mm/parametric/mesh metadata`,
      })
    }
  }
  if (Object.prototype.hasOwnProperty.call(phys, 'legacy_override') && phys.legacy_override !== undefined) {
    findings.push({
      componentId: id,
      field: 'legacy_override',
      message: 'physical.legacy_override is a temporary mesh-audit marker; will be retired with Phase 5 mesh extents',
    })
  }
  return findings
}

const seenDeprecationKeys = new Set<string>()

/** Emits one console.warn per (componentId, field) pair across the lifetime of
 *  the page. Called from the preset loader so the Phase 0 warnings surface
 *  exactly once even when the catalog is re-fetched. */
export function reportComponentSpecDeprecations(
  catalog: { categories?: Record<string, { components?: Array<{ id: string; physical?: Record<string, unknown> }> }> },
  emit: (msg: string) => void = (msg) => console.warn(msg),
): ComponentSpecDeprecationFinding[] {
  if (!catalog || typeof catalog !== 'object' || !catalog.categories) return []
  const all: ComponentSpecDeprecationFinding[] = []
  for (const cat of Object.values(catalog.categories)) {
    if (!cat?.components) continue
    for (const comp of cat.components) {
      const findings = auditComponentSpecDeprecations(comp)
      for (const f of findings) {
        const key = `${f.componentId}::${f.field}`
        if (seenDeprecationKeys.has(key)) continue
        seenDeprecationKeys.add(key)
        emit(`[componentSpec] ${f.componentId}: ${f.message}`)
        all.push(f)
      }
    }
  }
  return all
}
