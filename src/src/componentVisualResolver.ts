import * as THREE from 'three'
import type { AttachmentNodeDef } from './componentSpec'
import {
  generateVisuals,
  SERVO_HORN_ORIGIN_Z_RATIO,
  servoBodyShape,
  servoHornShape,
  servoSideYokeShape,
  servoHornBeamAdapterShape,
} from './componentMeshes'
import type { UrdfVisualDesc } from './componentMeshes'
import { resolveComponent, resolveComponentBboxMm } from './componentResolver.ts'
import { getMeasuredCollisionExtentMm, getMeasuredVisualExtentMm, getMeasuredMeshEntry } from './meshExtents.ts'
import type { MateConnector } from './mateConnectors'
import { getMeshVisualMetadata, PROCEDURAL_VISUAL_ONLY } from './richVisuals/meshOverrides'
import { findRichGenerator } from './richVisuals/generators'
import { getComponentColor } from './richVisuals/materials'
import { getCachedMeshGroup, isMeshLoadInProgress } from './richVisuals/meshCache'
import { prepareMeshVisualGroup } from './richVisuals/meshVisual'

export type ComponentVisualSource = 'mesh' | 'rich' | 'urdf_primitives'
export type ComponentVisualStatus = 'ready' | 'loading' | 'fallback' | 'missing'
export type ComponentVisualScalePolicy = 'none' | 'uniform' | 'per-axis'
export type ComponentCollisionSource = 'authored_mesh' | 'urdf_primitives' | 'preset_bbox'
/** Coordinate convention the resolver's `previewGroup` is authored in.
 * Carry and render paths use this to compute the per-target world rotation
 * through `componentVisualWorldQuat` — making both paths identical by
 * construction. Rich generators emit Y-up; meshes (after rotation overrides)
 * and URDF primitives are already URDF Z-up. */
export type ComponentVisualAuthoredFrame = 'y_up' | 'z_up'

export interface ComponentVisualPresetLike {
  id: string
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
  mechanical_electrical: Record<string, unknown>
  mounting_logic?: Record<string, unknown>
  connectors?: MateConnector[]
}

export interface ComponentVisualInstanceLike {
  length_mm?: number
}

export interface ComponentVisualBounds {
  hx: number
  hy: number
  hz: number
  cx: number
  cy: number
  cz: number
  shape: 'box' | 'cylinder'
}

export interface ResolvedComponentVisual {
  componentId: string
  source: ComponentVisualSource
  boundsSource: 'mesh_target_bbox' | 'urdf_primitives' | 'preset_bbox'
  status: ComponentVisualStatus
  frame: 'urdf-z-up'
  authoredFrame: ComponentVisualAuthoredFrame
  scalePolicy: ComponentVisualScalePolicy
  previewGroup?: THREE.Group
  visuals: UrdfVisualDesc[]
  bounds: ComponentVisualBounds
  visualBounds: ComponentVisualBounds | null
  collision: {
    source: ComponentCollisionSource
    bounds: ComponentVisualBounds
    meshFile?: string
  }
  connectors: MateConnector[]
  ports: AttachmentNodeDef[]
  warnings: string[]
  renderedBodySize?: THREE.Vector3 | null
  fallbackReason?: string
}

export interface ResolvedSplitServoVisual {
  componentId: string
  frame: 'urdf-z-up'
  hornOriginZ: number
  bodyVisuals: UrdfVisualDesc[]
  hornVisuals: UrdfVisualDesc[]
  bodyCollision: ResolvedComponentVisual['collision']
  hornCollision: ResolvedComponentVisual['collision']
}

export interface ResolveComponentVisualArgs {
  preset: ComponentVisualPresetLike
  category: string
  instance?: ComponentVisualInstanceLike
  linkName?: string
  materialCache?: Map<string, THREE.MeshStandardMaterial>
  castShadow?: boolean
  receiveShadow?: boolean
}

export function resolveComponentVisual(args: ResolveComponentVisualArgs): ResolvedComponentVisual {
  const preset = buildVisualPreset(args.preset, args.instance)
  const visuals = generateVisuals(preset as unknown as Parameters<typeof generateVisuals>[0], args.category)
  const visualBounds = visualBoundsFromDescriptors(visuals)
  const boundsResult = resolveCurrentBounds(preset, visuals)
  const meshMetadata = getMeshVisualMetadata(preset.id)
  const collision = resolveCollisionEnvelope(preset, visuals, boundsResult.bounds, visualBounds)
  const resolvedLogical = resolveComponent({ spec: preset, instance: args.instance, category: args.category })
  const proceduralOnly = PROCEDURAL_VISUAL_ONLY.has(preset.id)
  const meshOverride = !!meshMetadata && !proceduralOnly
  const meshUsable = !!meshMetadata && !meshMetadata.blacklisted && !proceduralOnly
  const meshPreview = meshUsable ? buildCachedMeshPreviewGroup(preset, {
    linkName: args.linkName,
    materialCache: args.materialCache,
    castShadow: args.castShadow,
    receiveShadow: args.receiveShadow,
  }) : undefined
  const previewGroup = meshPreview?.group ?? buildRichPreviewGroup(preset, {
    linkName: args.linkName,
    castShadow: args.castShadow,
    receiveShadow: args.receiveShadow,
  })
  const status: ComponentVisualStatus = meshPreview
    ? 'ready'
    : (meshUsable && isMeshLoadInProgress(preset.id) ? 'loading' : (meshOverride ? 'fallback' : 'ready'))

  const source: ComponentVisualSource = meshPreview ? 'mesh' : (previewGroup ? 'rich' : 'urdf_primitives')
  return {
    componentId: preset.id,
    source,
    boundsSource: boundsResult.boundsSource,
    status,
    frame: 'urdf-z-up',
    // Authored-frame unification: previewGroups are now Z-up regardless of
    // source — rich output is wrapped at construction time in
    // buildRichPreviewGroup, GLBs are Z-up natively. The dual-frame branch
    // that used to live here was the root of the carry-vs-placed rotation
    // mismatch when the live render swapped sources mid-flight.
    authoredFrame: 'z_up',
    scalePolicy: boundsResult.scalePolicy,
    previewGroup,
    visuals,
    bounds: boundsResult.bounds,
    visualBounds,
    collision,
    connectors: resolvedLogical.connectors,
    ports: resolvedLogical.ports,
    warnings: [
      ...resolvedLogical.warnings,
      ...buildResolverWarnings(meshOverride, meshUsable, meshPreview !== undefined),
      ...buildMeshDivergenceWarnings(preset.id, boundsResult.scalePolicy),
    ],
    renderedBodySize: meshPreview?.renderedBodySize ?? null,
    fallbackReason: meshOverride && !meshPreview
      ? (meshUsable ? 'mesh override is not cached yet' : 'mesh override is blacklisted')
      : undefined,
  }
}

export function resolveSplitServoVisual(args: {
  preset: ComponentVisualPresetLike
  category: string
  includeSideYoke?: boolean
  instance?: ComponentVisualInstanceLike
}): ResolvedSplitServoVisual {
  const preset = buildVisualPreset(args.preset, args.instance)
  const bb = presetBboxMm(preset)
  const w = bb[0] / 1000
  const d = bb[1] / 1000
  const h = bb[2] / 1000
  const bodyBase = servoBodyShape(w, h, d, args.category)
  // Body box uses the canonical servoBodyShape dimensions (h*0.76 along Z),
  // which sits under the horn at h*SERVO_HORN_ORIGIN_Z_RATIO. Earlier we
  // replaced this with the measured collision OBJ extent, but that coupled
  // the visual to the OBJ's authored axes — and after measure-mesh-extents
  // started applying the rotation override, the collision OBJ's longest axis
  // (e.g. 46.5mm for high_torque) flipped into the URDF Z slot, producing a
  // body box taller than the horn origin and swallowing the horn. The
  // collision envelope is now the spec bbox (componentResolver bbox-as-truth),
  // so the canonical servoBodyShape already matches collision by construction.
  const hornBase = servoHornShape(w, h, d, args.category)
  const bodyVisuals = args.includeSideYoke
    ? [...bodyBase, ...servoSideYokeShape(w, h, d, args.category)]
    : bodyBase
  const hornVisuals = args.includeSideYoke
    ? [...hornBase, ...servoHornBeamAdapterShape(w, h, d, args.category)]
    : hornBase

  const bodyBounds = visualBoundsFromDescriptors(bodyVisuals) ?? boundsFromBboxMm(bb)
  const hornBounds = visualBoundsFromDescriptors(hornVisuals) ?? boundsFromBboxMm(bb)
  const bodyCollision: ResolvedComponentVisual['collision'] = preset.physical.collision_mesh
    ? {
      source: 'authored_mesh',
      bounds: bodyBounds,
      meshFile: preset.physical.collision_mesh,
    }
    : {
      source: 'urdf_primitives',
      bounds: bodyBounds,
    }

  return {
    componentId: preset.id,
    frame: 'urdf-z-up',
    hornOriginZ: h * SERVO_HORN_ORIGIN_Z_RATIO,
    bodyVisuals,
    hornVisuals,
    bodyCollision,
    hornCollision: {
      source: 'urdf_primitives',
      bounds: hornBounds,
    },
  }
}

function resolveCollisionEnvelope(
  preset: ComponentVisualPresetLike,
  visuals: UrdfVisualDesc[],
  bounds: ComponentVisualBounds,
  visualBounds: ComponentVisualBounds | null,
): ResolvedComponentVisual['collision'] {
  if (preset.physical.collision_mesh) {
    // Bbox-as-source-of-truth: collision bounds use the spec bbox (centered at
    // origin), not the OBJ-measured AABB. The previous behavior trusted the
    // OBJ extent + center — but collision OBJs ship at fixed sizes (ignoring
    // per-instance length_mm), and several authors placed the OBJ origin at
    // a corner/face rather than the centroid (e.g. extrusion_2020.obj has
    // center_mm=[0,0,250] because the part runs z=0..500). Both errors caused
    // children to attach at the wrong distance and the collision shape to
    // float relative to the visual. Anchoring collision to the bbox keeps
    // placement, visual, and connector positions consistent by construction.
    // The OBJ stays as a build-time measurement for divergence reporting.
    const collisionShape: 'box' | 'cylinder' =
      preset.physical.inertia_primitive === 'cylinder' ? 'cylinder' : 'box'
    return {
      source: 'authored_mesh',
      bounds: { ...bounds, shape: collisionShape },
      meshFile: preset.physical.collision_mesh,
    }
  }
  if (visuals.length > 0) {
    return {
      source: 'urdf_primitives',
      bounds: visualBounds ?? bounds,
    }
  }
  return {
    source: 'preset_bbox',
    bounds,
  }
}

// Phase 5 step 4: visual divergence is meaningful only when the GLB renders at
// its native size (scalePolicy === 'none'). 'per-axis' and 'uniform' scale the
// mesh into the bbox by design, so divergence there is expected, not a bug.
// The collision side has no such scaling, so it gates unconditionally.
const COLLISION_DIVERGENCE_LIMIT = 0.15
const VISUAL_DIVERGENCE_LIMIT = 0.05

export function computeMeshDivergence(
  componentId: string,
  scalePolicy: ComponentVisualScalePolicy,
): { visualWorst: number; collisionWorst: number; errors: string[] } {
  const entry = getMeasuredMeshEntry(componentId)
  if (!entry) return { visualWorst: 0, collisionWorst: 0, errors: [] }
  const declared = entry.declared_bbox_mm
  const errors: string[] = []
  let visualWorst = 0
  let collisionWorst = 0
  if (declared) {
    if (scalePolicy === 'none') {
      const visualMeasured = getMeasuredVisualExtentMm(componentId)
      if (visualMeasured) {
        for (let i = 0; i < 3; i++) {
          if (declared[i] === 0) continue
          const d = Math.abs(visualMeasured[i] - declared[i]) / declared[i]
          if (d > visualWorst) visualWorst = d
        }
        if (visualWorst > VISUAL_DIVERGENCE_LIMIT) {
          errors.push(
            `${componentId}: visual mesh extent diverges from declared bbox by ${(visualWorst * 100).toFixed(1)}% (limit ${VISUAL_DIVERGENCE_LIMIT * 100}%, scalePolicy=none)`,
          )
        }
      }
    }
    const collisionMeasured = getMeasuredCollisionExtentMm(componentId)
    if (collisionMeasured) {
      for (let i = 0; i < 3; i++) {
        if (declared[i] === 0) continue
        const d = Math.abs(collisionMeasured[i] - declared[i]) / declared[i]
        if (d > collisionWorst) collisionWorst = d
      }
      if (collisionWorst > COLLISION_DIVERGENCE_LIMIT) {
        errors.push(
          `${componentId}: collision mesh extent diverges from declared bbox by ${(collisionWorst * 100).toFixed(1)}% (limit ${COLLISION_DIVERGENCE_LIMIT * 100}%)`,
        )
      }
    }
  }
  return { visualWorst, collisionWorst, errors }
}

function buildMeshDivergenceWarnings(
  componentId: string,
  scalePolicy: ComponentVisualScalePolicy,
): string[] {
  return computeMeshDivergence(componentId, scalePolicy).errors
}

function buildResolverWarnings(
  hasMeshOverride: boolean,
  meshUsable: boolean,
  meshReady: boolean,
): string[] {
  const warnings: string[] = []
  if (hasMeshOverride && !meshUsable) {
    warnings.push('mesh override is blacklisted; using fallback visual source')
  } else if (hasMeshOverride && !meshReady) {
    warnings.push('mesh override is not cached yet; placement-relevant fields remain deterministic')
  }
  return warnings
}

export function buildVisualPreset<T extends ComponentVisualPresetLike>(
  preset: T,
  instance?: ComponentVisualInstanceLike,
): T {
  const phys = preset.physical
  const bb = resolveComponentBboxMm(preset, instance)
  if (
    phys.bounding_box_mm?.[0] === bb[0]
    && phys.bounding_box_mm?.[1] === bb[1]
    && phys.bounding_box_mm?.[2] === bb[2]
  ) {
    return preset
  }
  return {
    ...preset,
    physical: {
      ...phys,
      bounding_box_mm: bb,
    },
  }
}

function presetBboxMm(preset: ComponentVisualPresetLike): [number, number, number] {
  return resolveComponentBboxMm(preset)
}

function boundsFromBboxMm(bb: [number, number, number]): ComponentVisualBounds {
  return {
    hx: bb[0] / 2000,
    hy: bb[1] / 2000,
    hz: bb[2] / 2000,
    cx: 0,
    cy: 0,
    cz: 0,
    shape: 'box',
  }
}

function buildCachedMeshPreviewGroup(
  preset: ComponentVisualPresetLike,
  opts: {
    linkName?: string
    materialCache?: Map<string, THREE.MeshStandardMaterial>
    castShadow?: boolean
    receiveShadow?: boolean
  } = {},
): { group: THREE.Group; renderedBodySize: THREE.Vector3 | null } | undefined {
  const cached = getCachedMeshGroup(preset.id)
  if (!cached) return undefined
  try {
    const bb = presetBboxMm(preset)
    const dims = { x: (bb[0] ?? 40) / 1000, y: (bb[1] ?? 40) / 1000, z: (bb[2] ?? 40) / 1000 }
    const clone = cached.clone(true)
    const prepared = prepareMeshVisualGroup(clone, dims, preset.id, {
      linkName: opts.linkName,
      materialCache: opts.materialCache,
      includeShaftOverlay: true,
      castShadow: opts.castShadow ?? false,
      receiveShadow: opts.receiveShadow ?? false,
    })
    const group = new THREE.Group()
    group.add(prepared.group)
    if (prepared.shaftOverlayMesh) group.add(prepared.shaftOverlayMesh)
    return { group, renderedBodySize: prepared.renderedBodySize }
  } catch (e) {
    console.warn(`[componentVisualResolver] Cached mesh preview failed for ${preset.id}:`, e)
    return undefined
  }
}

function buildRichPreviewGroup(
  preset: ComponentVisualPresetLike,
  opts: {
    linkName?: string
    castShadow?: boolean
    receiveShadow?: boolean
  } = {},
): THREE.Group | undefined {
  const generator = findRichGenerator(preset.id)
  if (!generator) return undefined
  try {
    const bb = presetBboxMm(preset)
    const dims = { x: (bb[0] ?? 40) / 1000, y: (bb[1] ?? 40) / 1000, z: (bb[2] ?? 40) / 1000 }
    const compColor = getComponentColor(preset.id)
    const group = generator(preset.id, dims, compColor.tint)
    const castShadow = opts.castShadow ?? false
    const receiveShadow = opts.receiveShadow ?? false
    group.traverse(child => {
      if (child instanceof THREE.Mesh) {
        child.castShadow = castShadow
        child.receiveShadow = receiveShadow
        if (opts.linkName) (child.userData as Record<string, unknown>).urdfLinkName = opts.linkName
      }
    })
    // Authored-frame unification: rich generators are written Y-up internally,
    // GLBs are authored Z-up. Wrap the rich output in +90° X here so the
    // returned previewGroup is Z-up authored regardless of source. Single
    // outer group lets the carry/render adapter set its quaternion without
    // disturbing the wrap. See authored-frame
    // section.
    const wrapper = new THREE.Group()
    group.quaternion.setFromEuler(new THREE.Euler(Math.PI / 2, 0, 0, 'XYZ'))
    wrapper.add(group)
    return wrapper
  } catch (e) {
    console.warn(`[componentVisualResolver] Rich preview failed for ${preset.id}:`, e)
    return undefined
  }
}

export function visualBoundsFromDescriptors(visuals: UrdfVisualDesc[]): ComponentVisualBounds | null {
  const box = new THREE.Box3()
  const tempBox = new THREE.Box3()
  const tempMatrix = new THREE.Matrix4()
  let hasGeom = false
  let allCylinders = visuals.length > 0

  for (const vis of visuals) {
    const g = vis.geometry
    if (g.type === 'box') {
      tempBox.set(
        new THREE.Vector3(-g.size[0] / 2, -g.size[1] / 2, -g.size[2] / 2),
        new THREE.Vector3( g.size[0] / 2,  g.size[1] / 2,  g.size[2] / 2),
      )
      allCylinders = false
    } else if (g.type === 'cylinder') {
      tempBox.set(
        new THREE.Vector3(-g.radius, -g.radius, -g.length / 2),
        new THREE.Vector3( g.radius,  g.radius,  g.length / 2),
      )
    } else {
      tempBox.set(
        new THREE.Vector3(-g.radius, -g.radius, -g.radius),
        new THREE.Vector3( g.radius,  g.radius,  g.radius),
      )
      allCylinders = false
    }

    const [ox, oy, oz] = vis.origin_xyz
    tempMatrix.makeRotationFromEuler(new THREE.Euler(...vis.origin_rpy, 'XYZ'))
    tempMatrix.setPosition(ox, oy, oz)
    box.union(tempBox.clone().applyMatrix4(tempMatrix))
    hasGeom = true
  }

  if (!hasGeom || box.isEmpty()) return null
  const center = box.getCenter(new THREE.Vector3())
  const size = box.getSize(new THREE.Vector3())
  return {
    hx: size.x / 2,
    hy: size.y / 2,
    hz: size.z / 2,
    cx: center.x,
    cy: center.y,
    cz: center.z,
    shape: allCylinders ? 'cylinder' : 'box',
  }
}

function resolveCurrentBounds(
  preset: ComponentVisualPresetLike,
  visuals: UrdfVisualDesc[],
): {
  bounds: ComponentVisualBounds
  boundsSource: ResolvedComponentVisual['boundsSource']
  scalePolicy: ComponentVisualScalePolicy
} {
  const meshMetadata = getMeshVisualMetadata(preset.id)
  // Both 'per-axis' and 'uniform' scale the GLB to fit the bbox envelope —
  // 'uniform' may leave a small gap on non-binding axes, but the bbox still
  // describes the carry footprint and joint geometry, so bounds derive from it.
  const glbScalesToBbox = !!meshMetadata
    && !meshMetadata.blacklisted
    && (meshMetadata.scalePolicy === 'per-axis' || meshMetadata.scalePolicy === 'uniform')
  if (glbScalesToBbox) {
    const bb = presetBboxMm(preset)
    const shaftOverlay = meshMetadata.shaftOverlay
    const zMm = shaftOverlay ? Math.max(1, (bb[2] ?? 40) - shaftOverlay.shaft_length_mm) : (bb[2] ?? 40)
    return {
      bounds: {
        hx: (bb[0] ?? 40) / 2000,
        hy: (bb[1] ?? 40) / 2000,
        hz: zMm / 2000,
        cx: 0, cy: 0, cz: 0,
        shape: 'box',
      },
      boundsSource: 'mesh_target_bbox',
      scalePolicy: meshMetadata.scalePolicy,
    }
  }

  const primitive = visualBoundsFromDescriptors(visuals)
  if (primitive) {
    return { bounds: primitive, boundsSource: 'urdf_primitives', scalePolicy: 'none' }
  }

  const bb = presetBboxMm(preset)
  return {
    bounds: boundsFromBboxMm(bb),
    boundsSource: 'preset_bbox',
    scalePolicy: 'none',
  }
}
