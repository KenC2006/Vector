/**
 * Component visual resolver: the one place that decides what a catalog part
 * looks like. Returns a preview group sized to the part's bbox, authored in
 * the catalog (URDF Z-up) frame: the part's GLB when one is mapped and
 * cached, else its procedural rich generator. Used by the renderer
 * (richVisuals.applyRichVisuals) and the assembly editor's placement ghost.
 */
import * as THREE from 'three'
import { hasMeshOverride } from './richVisuals/meshOverrides'
import { findRichGenerator } from './richVisuals/generators'
import { getComponentColor } from './richVisuals/materials'
import { getCachedMeshGroup, isMeshLoadInProgress } from './richVisuals/meshCache'
import { prepareMeshVisualGroup } from './richVisuals/meshVisual'

/** 'urdf_primitives': no preview; the caller keeps its own box/primitives. */
export type ComponentVisualSource = 'mesh' | 'rich' | 'urdf_primitives'
/** 'loading' / 'fallback': the part has a GLB that is loading / not cached;
 *  the rich generator stands in meanwhile. */
export type ComponentVisualStatus = 'ready' | 'loading' | 'fallback'
/** Frame the previewGroup is authored in. Every source is Z-up authored;
 *  componentVisualWorldQuat maps it into the caller's target frame. */
export type ComponentVisualAuthoredFrame = 'y_up' | 'z_up'

export interface ComponentVisualPresetLike {
  id: string
  physical: {
    bbox_mm?: number[]
    bounding_box_mm?: number[]
  }
  mechanical_electrical?: Record<string, unknown>
}

export interface ResolvedComponentVisual {
  componentId: string
  source: ComponentVisualSource
  status: ComponentVisualStatus
  frame: 'urdf-z-up'
  authoredFrame: ComponentVisualAuthoredFrame
  previewGroup?: THREE.Group
}

export interface ResolveComponentVisualArgs {
  preset: ComponentVisualPresetLike
  /** Catalog category; kept for callers, the visual is chosen by id. */
  category?: string
  linkName?: string
  materialCache?: Map<string, THREE.MeshStandardMaterial>
  castShadow?: boolean
  receiveShadow?: boolean
}

export function resolveComponentVisual(args: ResolveComponentVisualArgs): ResolvedComponentVisual {
  const { preset } = args
  const meshOverride = hasMeshOverride(preset.id)
  const meshPreview = meshOverride ? buildCachedMeshPreviewGroup(preset, args) : undefined
  const previewGroup = meshPreview ?? buildRichPreviewGroup(preset, args)
  const status: ComponentVisualStatus = meshPreview || !meshOverride
    ? 'ready'
    : (isMeshLoadInProgress(preset.id) ? 'loading' : 'fallback')
  return {
    componentId: preset.id,
    source: meshPreview ? 'mesh' : (previewGroup ? 'rich' : 'urdf_primitives'),
    status,
    frame: 'urdf-z-up',
    authoredFrame: 'z_up',
    previewGroup,
  }
}

/**
 * True when a catalog part renders as something better than a plain box: an
 * authored mesh or a procedural rich generator. Every catalog category has a
 * generator, so this holds for the whole catalog; ids outside it fall back to
 * a box.
 */
export function hasVisual(id: string): boolean {
  return hasMeshOverride(id) || findRichGenerator(id) !== null
}

/** Part envelope in meters: the preset's bbox (callers pass the instance
 *  size for cut-to-length parts). */
function bboxMeters(preset: ComponentVisualPresetLike): { x: number; y: number; z: number } {
  const bb = [preset.physical.bbox_mm, preset.physical.bounding_box_mm]
    .find(v => Array.isArray(v) && v.length === 3 && v.every(n => Number.isFinite(n) && n > 0))
    ?? [40, 40, 40]
  return { x: bb[0] / 1000, y: bb[1] / 1000, z: bb[2] / 1000 }
}

function buildCachedMeshPreviewGroup(
  preset: ComponentVisualPresetLike,
  opts: {
    linkName?: string
    materialCache?: Map<string, THREE.MeshStandardMaterial>
    castShadow?: boolean
    receiveShadow?: boolean
  } = {},
): THREE.Group | undefined {
  const cached = getCachedMeshGroup(preset.id)
  if (!cached) return undefined
  try {
    const dims = bboxMeters(preset)
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
    return group
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
    const dims = bboxMeters(preset)
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
    // Rich generators are written Y-up internally, GLBs are authored Z-up.
    // Wrap the rich output in +90° X so every previewGroup is Z-up authored;
    // the outer group lets callers set their own quaternion on top.
    const wrapper = new THREE.Group()
    group.quaternion.setFromEuler(new THREE.Euler(Math.PI / 2, 0, 0, 'XYZ'))
    wrapper.add(group)
    return alignAxesToBbox(wrapper, [dims.x, dims.y, dims.z])
  } catch (e) {
    console.warn(`[componentVisualResolver] Rich preview failed for ${preset.id}:`, e)
    return undefined
  }
}

/**
 * Generators disagree on whether their long axis runs along Y or Z, and some
 * read the wrong dimension as height, so parts came out lying along the wrong
 * axis (beams, tubes) or 2x too tall (motors). The catalog bbox is what
 * placement and connectors use, so conform the built visual to it:
 * first rotate when its extents are an axis permutation of the bbox, then
 * rescale any axis still more than 25% off.
 */
function alignAxesToBbox(visual: THREE.Group, target: [number, number, number]): THREE.Group {
  let box = new THREE.Box3().setFromObject(visual)
  if (box.isEmpty()) return visual
  let s = box.getSize(new THREE.Vector3()).toArray()
  const cost = (perm: number[]) =>
    perm.reduce((acc, src, dst) => acc + Math.abs(Math.log(Math.max(s[src], 1e-6) / Math.max(target[dst], 1e-6))), 0)
  const perms = [[0, 1, 2], [0, 2, 1], [1, 0, 2], [1, 2, 0], [2, 0, 1], [2, 1, 0]]
  let best = perms[0]
  for (const p of perms) if (cost(p) < cost(best)) best = p
  let out = visual
  // Only re-orient on a clear misorientation; near-cubic parts stay put.
  if (best !== perms[0] && cost(perms[0]) - cost(best) >= 0.5) {
    // Rotation taking rendered axis best[i] onto catalog axis i (det +1).
    const cols = [new THREE.Vector3(), new THREE.Vector3(), new THREE.Vector3()]
    for (let dst = 0; dst < 3; dst++) cols[best[dst]].setComponent(dst, 1)
    const m = new THREE.Matrix4().makeBasis(cols[0], cols[1], cols[2])
    if (m.determinant() < 0) {
      cols[best[0]].negate()
      m.makeBasis(cols[0], cols[1], cols[2])
    }
    visual.quaternion.setFromRotationMatrix(m)
    out = new THREE.Group()
    out.add(visual)
    box = new THREE.Box3().setFromObject(out)
    s = box.getSize(new THREE.Vector3()).toArray()
  }
  const ratio = target.map((t, i) => t / Math.max(s[i], 1e-6))
  if (ratio.some(r => r < 0.8 || r > 1.25)) {
    const scaled = new THREE.Group()
    const c = box.getCenter(new THREE.Vector3())
    const inner = new THREE.Group()
    inner.position.copy(c).negate()
    inner.add(out)
    scaled.scale.set(ratio[0], ratio[1], ratio[2])
    scaled.add(inner)
    out = new THREE.Group()
    out.add(scaled)
  }
  return out
}
