import type {
  AttachmentNodeDef,
  ComponentInstanceSpec,
  ComponentSpec,
  ResolvedComponentRecord,
} from './componentSpec.ts'
import { _resolverInternal_generateDefaultConnectors, mergeConnectors } from './mateConnectors.ts'
import type { MateConnector } from './mateConnectors.ts'
import {
  hasLinkGeometry,
  linkGeometryCollisionDescriptors,
  linkGeometryConnectors,
  linkGeometryUnionAabbMm,
  linkGeometryVolumeM3,
  LINK_GEOMETRY_DEFAULT_DENSITY_KG_M3,
} from './linkGeometry.ts'
import type { LinkPrimitive } from './linkGeometry.ts'

/**
 * Loose preset shape accepted by the resolver. Wider than `ComponentSpec` to
 * tolerate callers loading raw catalog JSON (where number tuples land as
 * `number[]` not `[number, number, number]`).
 */
export type ComponentResolverSpec = {
  id: string
  category?: string
  name?: string
  description?: string
  physical: {
    mass_kg?: number
    mass_kg_per_100mm?: number
    bbox_mm?: number[]
    bounding_box_mm?: number[]
    parametric?: {
      axis?: unknown
      cross_section_mm?: unknown
    }
    cross_section_mm?: number[]
    inertia_primitive?: string
    collision_mesh?: string
  }
  mechanical_electrical?: Record<string, unknown>
  mounting_logic?: Record<string, unknown>
  connectors?: MateConnector[]
}

export interface ResolveComponentArgs {
  spec: ComponentResolverSpec
  instance?: ComponentInstanceSpec
  category?: string
}

export function resolveComponent(args: ResolveComponentArgs): ResolvedComponentRecord {
  const spec = normalizeSpec(args.spec, args.category ?? args.spec.category ?? inferComponentCategory(args.spec.id))

  // Authored body shell: every geometric fact derives from the primitive
  // union, not the donor preset's envelope — bounds (center generally ≠ 0),
  // the 6 face connectors (so children mount on the REAL surfaces), and
  // per-primitive collision descriptors. Authored preset connectors still
  // merge over the derived defaults.
  const prims = hasLinkGeometry(args.instance)
    ? (args.instance!.link_geometry as LinkPrimitive[])
    : null
  const union = prims ? linkGeometryUnionAabbMm(prims) : null
  if (prims && union) {
    const bounds = {
      half: union.half,
      center: union.center,
      shape: 'box' as const,
    }
    // The shell REPLACES the donor preset's geometry, so the donor's authored
    // connectors (which describe the donor's surfaces — e.g. a baseplate's
    // 'top' at +2.5mm) must NOT override the union faces. Union-derived
    // connectors are the only valid mounting surfaces on a shell.
    const connectors = linkGeometryConnectors(prims)
    const ports = portsFromConnectors(connectors)
    return {
      id: spec.id,
      spec,
      bounds,
      collision: {
        source: 'urdf_primitives',
        bounds,
        descriptors: linkGeometryCollisionDescriptors(prims),
      },
      ports,
      connectors,
      warnings: [],
    }
  }

  const halfBounds = resolveComponentHalfBoundsMm(spec, args.instance)
  const shape: 'box' | 'cylinder' = spec.physical.inertia_primitive === 'cylinder' ? 'cylinder' : 'box'
  const bounds = {
    half: [halfBounds.hxMm, halfBounds.hyMm, halfBounds.hzMm] as [number, number, number],
    center: [0, 0, 0] as [number, number, number],
    shape,
  }
  const connectors = mergeConnectors(
    _resolverInternal_generateDefaultConnectors(halfBounds),
    spec.connectors,
  )
  const ports = portsFromConnectors(connectors)
  const collision = resolveCollisionRecord(spec, bounds)
  const warnings = [
    ...buildResolverWarnings(spec),
    ...buildMeasuredMeshWarnings(spec.id, [halfBounds.hxMm * 2, halfBounds.hyMm * 2, halfBounds.hzMm * 2]),
  ]

  return {
    id: spec.id,
    spec,
    bounds,
    collision,
    ports,
    connectors,
    warnings,
  }
}

function resolveCollisionRecord(
  spec: ComponentResolverSpec,
  envelopeBounds: ResolvedComponentRecord['bounds'],
): ResolvedComponentRecord['collision'] {
  // Bbox-as-source-of-truth: collision bounds always derive from the spec
  // bbox (envelopeBounds), never from the measured OBJ. See
  // componentVisualResolver.resolveCollisionEnvelope for the rationale.
  if (!spec.physical.collision_mesh) {
    return { source: 'preset_bbox', bounds: envelopeBounds }
  }
  return {
    source: 'authored_mesh',
    file: spec.physical.collision_mesh,
    bounds: envelopeBounds,
  }
}

// Phase 5 step 4: divergence reporting moved to componentVisualResolver where
// scalePolicy is in scope — visual divergence is only meaningful when
// scalePolicy === 'none'. This stub stays no-op so the call site keeps its
// shape; the visual resolver now owns both visual and collision gating.
function buildMeasuredMeshWarnings(_componentId: string, _declaredBboxMm: [number, number, number]): string[] {
  return []
}

export function resolveComponentHalfBoundsMm(
  spec: ComponentResolverSpec,
  instance?: ComponentInstanceSpec,
): { hxMm: number; hyMm: number; hzMm: number } {
  const bb = resolveComponentBboxMm(spec, instance)
  return {
    hxMm: (bb[0] ?? 40) / 2,
    hyMm: (bb[1] ?? 40) / 2,
    hzMm: (bb[2] ?? 40) / 2,
  }
}

/**
 * Resolve a component's mass in kg.
 * - Authored mass_kg wins.
 * - Parametric extrusions use mass_kg_per_100mm scaled by instance length_mm
 *   (per-100mm × length_mm/100). With no instance length, falls back to the
 *   per-100mm value as-is.
 * - Otherwise returns fallbackKg.
 */
export function resolveComponentMassKg(
  spec: ComponentResolverSpec,
  instance?: ComponentInstanceSpec,
  fallbackKg = 0.1,
): number {
  const phys = spec.physical
  // Authored body shells: derive mass from the primitive volume at shell
  // density. Takes priority over the donor preset's mass_kg — a 300mm torso
  // drawn over a baseplate must not weigh 450g.
  if (hasLinkGeometry(instance)) {
    const vol = linkGeometryVolumeM3(instance!.link_geometry as LinkPrimitive[])
    if (vol > 0) return vol * LINK_GEOMETRY_DEFAULT_DENSITY_KG_M3
  }
  if (typeof phys.mass_kg === 'number') return phys.mass_kg
  if (typeof phys.mass_kg_per_100mm === 'number') {
    if (typeof instance?.length_mm === 'number') {
      return phys.mass_kg_per_100mm * (instance.length_mm / 100)
    }
    return phys.mass_kg_per_100mm
  }
  return fallbackKg
}

/** True when the spec is a parametric extrusion whose length is supplied
 *  per-instance via `length_mm`. Centralizes the predicate that previously
 *  appeared inline as `phys.cross_section_mm` checks across the codebase. */
export function isParametricSpec(spec: ComponentResolverSpec): boolean {
  const phys = spec.physical
  if (isParametricEnvelope(phys.parametric)) return true
  if (isNumberTuple(phys.cross_section_mm, 2)) return true
  return false
}

export function resolveComponentBboxMm(
  spec: ComponentResolverSpec,
  instance?: ComponentInstanceSpec,
): [number, number, number] {
  const phys = spec.physical
  if (isNumberTuple(phys.bbox_mm, 3)) return tuple3(phys.bbox_mm)
  if (isNumberTuple(phys.bounding_box_mm, 3)) return tuple3(phys.bounding_box_mm)
  if (isParametricEnvelope(phys.parametric)) {
    return bboxFromParametricEnvelope(phys.parametric.axis, phys.parametric.cross_section_mm, instance?.length_mm)
  }
  if (isNumberTuple(phys.cross_section_mm, 3)) {
    const bb = phys.cross_section_mm
    return [bb[0], bb[1], instance?.length_mm ?? bb[2]]
  }
  if (isNumberTuple(phys.cross_section_mm, 2)) {
    const bb = phys.cross_section_mm
    // Parametric default length matches `bboxFromParametricEnvelope` (100mm)
    // so validator-time resolves without needing an instance.
    return [bb[0], bb[1], instance?.length_mm ?? 100]
  }
  // Phase 6: missing dims is a CI failure, not a silent fallback. Every
  // entry in `generic_presets.json` is required to declare bbox_mm /
  // bounding_box_mm / parametric / cross_section_mm; the schema validator
  // catches authoring gaps. If we ever reach here it means a runtime
  // injected an unauthored spec.
  throw new Error(
    `[componentResolver] '${spec.id}' missing bbox_mm/bounding_box_mm/parametric/cross_section_mm`
  )
}

function isNumberTuple(value: unknown, length: number): value is number[] {
  return Array.isArray(value)
    && value.length === length
    && value.every(item => typeof item === 'number' && Number.isFinite(item))
}

function tuple3(value: number[]): [number, number, number] {
  // Guarded by isNumberTuple(value, 3) at every call site.
  return [value[0], value[1], value[2]]
}

function isParametricEnvelope(value: unknown): value is { axis: 'x' | 'y' | 'z'; cross_section_mm: [number, number] } {
  if (!value || typeof value !== 'object') return false
  const envelope = value as { axis?: unknown; cross_section_mm?: unknown }
  return (envelope.axis === 'x' || envelope.axis === 'y' || envelope.axis === 'z')
    && isNumberTuple(envelope.cross_section_mm, 2)
}

function bboxFromParametricEnvelope(
  axis: 'x' | 'y' | 'z',
  crossSectionMm: [number, number],
  lengthMm = 100,
): [number, number, number] {
  if (axis === 'x') return [lengthMm, crossSectionMm[0], crossSectionMm[1]]
  if (axis === 'y') return [crossSectionMm[0], lengthMm, crossSectionMm[1]]
  return [crossSectionMm[0], crossSectionMm[1], lengthMm]
}

function normalizeSpec(spec: ComponentResolverSpec, category: string): ComponentSpec {
  return {
    ...spec,
    category,
    mechanical_electrical: spec.mechanical_electrical ?? {},
    mounting_logic: spec.mounting_logic ?? {},
  } as ComponentSpec
}

function buildResolverWarnings(spec: ComponentResolverSpec): string[] {
  const warnings: string[] = []
  if (!spec.physical.bbox_mm && !spec.physical.bounding_box_mm && !spec.physical.parametric && !spec.physical.cross_section_mm) {
    warnings.push('missing physical.bbox_mm/bounding_box_mm or physical.parametric/cross_section_mm; using compatibility envelope')
  }
  return warnings
}

export function inferComponentCategory(componentId: string): string {
  if (componentId.startsWith('actuator_')) return 'actuators'
  if (componentId.startsWith('motor_')) return 'motors'
  if (componentId.startsWith('sensor_')) return 'sensors'
  if (componentId.startsWith('compute_')) return 'compute'
  if (componentId.startsWith('power_')) return 'power'
  if (componentId.startsWith('structural_')) return 'structural'
  if (componentId.startsWith('transmission_')) return 'transmission'
  if (componentId.startsWith('effector_')) return 'end_effectors'
  if (componentId.startsWith('mobility_')) return 'mobility'
  return 'misc'
}

export function isTireComponentId(componentId: string): boolean {
  return (
    componentId.startsWith('mobility_wheel_') ||
    componentId.startsWith('mobility_mecanum_') ||
    componentId.startsWith('mobility_omni_') ||
    componentId.startsWith('mobility_caster_')
  )
}

export function isDrivetrainComponentId(componentId: string): boolean {
  return componentId.startsWith('drivetrain_')
}

export function isFootPadComponentId(componentId: string): boolean {
  return componentId === 'mobility_rubber_foot_pad'
}

function _defaultClsForConnectorType(type: MateConnector['type']): AttachmentNodeDef['cls'] {
  if (type === 'planar') return 'mount_face'
  if (type === 'point') return 'generic'
  // Cylindrical without an authored cls — the catalog CI forbids this, so it
  // only happens for runtime-injected specs. 'generic' mates with anything.
  return 'generic'
}

function _humanizeConnectorId(id: string): string {
  return id.replace(/_/g, ' ').replace(/\b\w/g, ch => ch.toUpperCase())
}

/** WS4: attachment "ports" derive from the merged connector list — one
 * vocabulary for geometry (mates) AND compatibility (port classes). Replaces
 * the deleted `resolveComponentPortsForBounds` string-prefix heuristics:
 * a servo's shaft is a shaft because its `shaft_out`/`top` connector is
 * AUTHORED `cls: 'shaft'`, not because its id starts with `actuator_servo`.
 * Connector origins are mm; port origins stay meters (legacy contract). */
export function portsFromConnectors(connectors: MateConnector[]): AttachmentNodeDef[] {
  return connectors.map(c => {
    const cls = c.cls ?? _defaultClsForConnectorType(c.type)
    return {
      nodeId: c.id,
      label: _humanizeConnectorId(c.id),
      cls,
      origin_xyz: [
        c.origin_xyz_mm[0] / 1000,
        c.origin_xyz_mm[1] / 1000,
        c.origin_xyz_mm[2] / 1000,
      ] as [number, number, number],
      origin_rpy: [0, 0, 0] as [number, number, number],
      kinematic: { joint_type: 'fixed' as const, axis_xyz: [...c.axis_xyz] as [number, number, number] },
      single: c.single ?? (cls === 'shaft' || cls === 'bore'),
    }
  })
}
