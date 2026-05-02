import * as THREE from 'three'
import {
  generateVisuals,
  SERVO_HORN_ORIGIN_Z_RATIO,
  servoBodyShape,
  servoHornShape,
  servoSideYokeShape,
  servoHornBeamAdapterShape,
} from './componentMeshes'
import type { UrdfVisualDesc } from './componentMeshes'
import { getMeshVisualMetadata } from './richVisuals/meshOverrides'
import { findRichGenerator } from './richVisuals/generators'
import { getComponentColor } from './richVisuals/materials'
import { getCachedMeshGroup, isMeshLoadInProgress } from './richVisuals/meshCache'
import { prepareMeshVisualGroup } from './richVisuals/meshVisual'

export type ComponentVisualSource = 'mesh' | 'rich' | 'urdf_primitives'
export type ComponentVisualStatus = 'ready' | 'loading' | 'fallback' | 'missing'
export type ComponentVisualScalePolicy = 'none' | 'uniform' | 'per-axis'
export type ComponentCollisionSource = 'authored_mesh' | 'urdf_primitives' | 'preset_bbox'
/** Coordinate convention the resolver's `previewGroup` is authored in.
 *  Carry and render paths use this to compute the per-target world rotation
 *  through `componentVisualWorldQuat` — making both paths identical by
 *  construction. Rich generators emit Y-up; meshes (after rotation overrides)
 *  and URDF primitives are already URDF Z-up. */
export type ComponentVisualAuthoredFrame = 'y_up' | 'z_up'

export interface ComponentVisualPresetLike {
  id: string
  physical: {
    mass_kg?: number
    mass_kg_per_100mm?: number
    bounding_box_mm?: number[]
    cross_section_mm?: number[]
    inertia_primitive?: string
    outer_diameter_mm?: number
    inner_diameter_mm?: number
    wall_thickness_mm?: number
    collision_mesh?: string
  }
  mechanical_electrical: Record<string, unknown>
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
  mode?: 'carry' | 'render' | 'collision' | 'diagnostic'
  linkName?: string
  materialCache?: Map<string, THREE.MeshStandardMaterial>
  castShadow?: boolean
  receiveShadow?: boolean
}

export function resolveComponentVisual(args: ResolveComponentVisualArgs): ResolvedComponentVisual {
  const preset = buildVisualPreset(args.preset, args.instance)
  const visuals = generateVisuals(preset as Parameters<typeof generateVisuals>[0], args.category)
  const visualBounds = visualBoundsFromDescriptors(visuals)
  const boundsResult = resolveCurrentBounds(preset, visuals)
  const meshMetadata = getMeshVisualMetadata(preset.id)
  const collision = resolveCollisionEnvelope(preset, visuals, boundsResult.bounds, visualBounds)
  const meshOverride = !!meshMetadata
  const meshUsable = !!meshMetadata && !meshMetadata.blacklisted
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
    authoredFrame: source === 'rich' ? 'y_up' : 'z_up',
    scalePolicy: boundsResult.scalePolicy,
    previewGroup,
    visuals,
    bounds: boundsResult.bounds,
    visualBounds,
    collision,
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
    return {
      source: 'authored_mesh',
      bounds,
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

export function buildVisualPreset<T extends ComponentVisualPresetLike>(
  preset: T,
  instance?: ComponentVisualInstanceLike,
): T {
  const phys = preset.physical
  const bb = phys.bounding_box_mm ?? phys.cross_section_mm ?? [40, 40, 40]
  if (instance?.length_mm && phys.cross_section_mm) {
    return {
      ...preset,
      physical: {
        ...phys,
        bounding_box_mm: [bb[0] ?? 40, bb[1] ?? 40, instance.length_mm],
      },
    }
  }
  return preset
}

function presetBboxMm(preset: ComponentVisualPresetLike): [number, number, number] {
  const phys = preset.physical
  const bb = phys.bounding_box_mm ?? phys.cross_section_mm ?? [40, 40, 40]
  return [bb[0] ?? 40, bb[1] ?? 40, bb[2] ?? 40]
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
    return group
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
  const glbScalesToBbox = !!meshMetadata
    && !meshMetadata.blacklisted
    && meshMetadata.scalePolicy === 'per-axis'
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
      scalePolicy: 'per-axis',
    }
  }

  const primitive = legacyPrimitiveBounds(visuals)
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

function legacyPrimitiveBounds(visuals: UrdfVisualDesc[]): ComponentVisualBounds | null {
  let minX = Infinity, maxX = -Infinity
  let minY = Infinity, maxY = -Infinity
  let minZ = Infinity, maxZ = -Infinity
  let allCylinders = visuals.length > 0

  for (const vis of visuals) {
    const [ox, oy, oz] = vis.origin_xyz
    const g = vis.geometry
    let ex = 0, ey = 0, ez = 0
    if (g.type === 'box') {
      ex = g.size[0] / 2
      ey = g.size[1] / 2
      ez = g.size[2] / 2
      allCylinders = false
    } else if (g.type === 'cylinder') {
      ex = g.radius
      ey = g.radius
      ez = g.length / 2
    } else {
      ex = g.radius
      ey = g.radius
      ez = g.radius
      allCylinders = false
    }
    if (vis.origin_rpy[0] !== 0 || vis.origin_rpy[1] !== 0 || vis.origin_rpy[2] !== 0) {
      allCylinders = false
    }
    minX = Math.min(minX, ox - ex); maxX = Math.max(maxX, ox + ex)
    minY = Math.min(minY, oy - ey); maxY = Math.max(maxY, oy + ey)
    minZ = Math.min(minZ, oz - ez); maxZ = Math.max(maxZ, oz + ez)
  }

  if (!isFinite(minX)) return null
  return {
    hx: (maxX - minX) / 2,
    hy: (maxY - minY) / 2,
    hz: (maxZ - minZ) / 2,
    cx: (maxX + minX) / 2,
    cy: (maxY + minY) / 2,
    cz: (maxZ + minZ) / 2,
    shape: allCylinders ? 'cylinder' : 'box',
  }
}
