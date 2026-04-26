import * as THREE from 'three'
import { generateVisuals } from './componentMeshes'
import type { UrdfVisualDesc } from './componentMeshes'
import { getRenderedMeshDims } from './meshDimsCache'
import { getShaftOverlay, hasMeshOverride, SLOW_MESH_BLACKLIST } from './richVisuals/meshOverrides'
import { getOrComputeBbox } from './componentDims'
import { findRichGenerator } from './richVisuals/generators'
import { getComponentColor } from './richVisuals/materials'
import { getCachedMeshGroup, isMeshLoadInProgress } from './richVisuals/meshCache'
import { prepareMeshVisualGroup } from './richVisuals/meshVisual'

export type ComponentVisualSource = 'mesh' | 'rich' | 'urdf_primitives'
export type ComponentVisualStatus = 'ready' | 'loading' | 'fallback' | 'missing'
export type ComponentVisualScalePolicy = 'none' | 'uniform' | 'per-axis'

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
  boundsSource: 'rendered_mesh_cache' | 'mesh_target_bbox' | 'urdf_primitives' | 'preset_bbox'
  status: ComponentVisualStatus
  frame: 'urdf-z-up'
  scalePolicy: ComponentVisualScalePolicy
  previewGroup?: THREE.Group
  visuals: UrdfVisualDesc[]
  bounds: ComponentVisualBounds
  visualBounds: ComponentVisualBounds | null
  fallbackReason?: string
}

export interface ResolveComponentVisualArgs {
  preset: ComponentVisualPresetLike
  category: string
  instance?: ComponentVisualInstanceLike
  mode?: 'carry' | 'render' | 'collision' | 'diagnostic'
}

export function resolveComponentVisual(args: ResolveComponentVisualArgs): ResolvedComponentVisual {
  const preset = buildVisualPreset(args.preset, args.instance)
  const visuals = generateVisuals(preset as Parameters<typeof generateVisuals>[0], args.category)
  const visualBounds = visualBoundsFromDescriptors(visuals)
  const boundsResult = resolveCurrentBounds(preset, visuals)
  const meshOverride = hasMeshOverride(preset.id)
  const meshUsable = meshOverride && !SLOW_MESH_BLACKLIST.has(preset.id)
  const meshPreviewGroup = meshUsable ? buildCachedMeshPreviewGroup(preset) : undefined
  const previewGroup = meshPreviewGroup ?? buildRichPreviewGroup(preset)
  const status: ComponentVisualStatus = meshPreviewGroup
    ? 'ready'
    : (meshUsable && isMeshLoadInProgress(preset.id) ? 'loading' : (meshOverride ? 'fallback' : 'ready'))

  return {
    componentId: preset.id,
    source: meshPreviewGroup ? 'mesh' : (previewGroup ? 'rich' : 'urdf_primitives'),
    boundsSource: boundsResult.boundsSource,
    status,
    frame: 'urdf-z-up',
    scalePolicy: boundsResult.scalePolicy,
    previewGroup,
    visuals,
    bounds: boundsResult.bounds,
    visualBounds,
    fallbackReason: meshOverride && !meshPreviewGroup
      ? (meshUsable ? 'mesh override is not cached yet' : 'mesh override is blacklisted')
      : undefined,
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

function buildCachedMeshPreviewGroup(preset: ComponentVisualPresetLike): THREE.Group | undefined {
  const cached = getCachedMeshGroup(preset.id)
  if (!cached) return undefined
  try {
    const bb = getOrComputeBbox(preset.id, preset)
    const dims = { x: (bb[0] ?? 40) / 1000, y: (bb[1] ?? 40) / 1000, z: (bb[2] ?? 40) / 1000 }
    const clone = cached.clone(true)
    const prepared = prepareMeshVisualGroup(clone, dims, preset.id, {
      includeShaftOverlay: true,
      castShadow: false,
      receiveShadow: false,
    })
    const group = new THREE.Group()
    group.add(prepared.group)
    if (prepared.shaftOverlayMesh) group.add(prepared.shaftOverlayMesh)
    return group
  } catch (e) {
    console.warn(`[componentVisualResolver] Cached mesh preview failed for ${preset.id}:`, e)
    return undefined
  }
}

function buildRichPreviewGroup(preset: ComponentVisualPresetLike): THREE.Group | undefined {
  const generator = findRichGenerator(preset.id)
  if (!generator) return undefined
  try {
    const bb = getOrComputeBbox(preset.id, preset)
    const dims = { x: (bb[0] ?? 40) / 1000, y: (bb[1] ?? 40) / 1000, z: (bb[2] ?? 40) / 1000 }
    const compColor = getComponentColor(preset.id)
    return generator(preset.id, dims, compColor.tint)
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
  const renderedDims = getRenderedMeshDims(preset.id)
  if (renderedDims && renderedDims.x > 0.001) {
    return {
      bounds: {
        hx: renderedDims.x / 2,
        hy: renderedDims.y / 2,
        hz: renderedDims.z / 2,
        cx: 0, cy: 0, cz: 0,
        shape: 'box',
      },
      boundsSource: 'rendered_mesh_cache',
      scalePolicy: 'per-axis',
    }
  }

  const perAxisBlacklist = ['gripper', 'effector', 'claw', 'suction']
  const glbScalesToBbox = hasMeshOverride(preset.id)
    && !SLOW_MESH_BLACKLIST.has(preset.id)
    && !perAxisBlacklist.some(k => preset.id.includes(k))
  if (glbScalesToBbox) {
    const bb = getOrComputeBbox(preset.id, preset)
    const shaftOverlay = getShaftOverlay(preset.id)
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

  const bb = getOrComputeBbox(preset.id, preset)
  return {
    bounds: { hx: bb[0] / 2000, hy: bb[1] / 2000, hz: bb[2] / 2000, cx: 0, cy: 0, cz: 0, shape: 'box' },
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
