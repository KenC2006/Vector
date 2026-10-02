/**
 * Rich Visual Override System
 *
 * After parseURDFToScene() builds the scene from URDF primitives, this module
 * scans all links. For any link whose name matches a preset component ID, it
 * replaces the primitive meshes with rich Three.js geometry.
 *
 * URDF stays clean with simple collision primitives. Only the viewport gets
 * rich visuals. Falls back gracefully for non-preset links.
 */
import * as THREE from 'three'
import type { GeneratorDims } from './generators'
import { getMeshOverrideUrl, MESH_OVERRIDES } from './meshOverrides'
import { componentIdForLink } from '../design/identity'
import { resolveComponentVisual } from '../componentVisualResolver'
import type {
  ComponentVisualAuthoredFrame,
  ComponentVisualPresetLike,
  ResolvedComponentVisual,
} from '../componentVisualResolver'
import {
  clearMeshLoadInProgress,
  hasCachedMeshGroup,
  isMeshLoadInProgress,
  markMeshLoadInProgress,
  setCachedMeshGroup,
} from './meshCache'
interface ParsedRobotLike {
  group: THREE.Group
  linkGroups: Map<string, THREE.Group>
}

/**
 * Measure the bounding box of existing geometry in a link group.
 * Used to determine dimensions for the rich generator.
 */
function measureLinkDims(linkGroup: THREE.Group): GeneratorDims {
  // Read dimensions directly from geometry parameters — no world-matrix dependency.
  // This avoids all first-render timing issues with stale matrixWorld.
  // Returns dimensions in URDF space (Z-up): x=width, y=depth, z=height.
  let maxX = 0, maxY = 0, maxZ = 0

  const geometryGroup = linkGroup.children[0]
  if (geometryGroup) {
    geometryGroup.traverse(child => {
      if (child instanceof THREE.Mesh) {
        const geom = child.geometry
        const params = (geom as any).parameters

        if (geom instanceof THREE.BoxGeometry && params) {
          // BoxGeometry(width, height, depth) → URDF: width=X, height=Y, depth=Z
          maxX = Math.max(maxX, params.width || 0)
          maxY = Math.max(maxY, params.height || 0)
          maxZ = Math.max(maxZ, params.depth || 0)
        } else if (geom instanceof THREE.CylinderGeometry && params) {
          // CylinderGeometry: radius along XY, length along Y (Three.js)
          // In URDF parser, cylinders are rotated PI/2 around X to align with Z
          const r = params.radiusTop || params.radiusBottom || 0
          const h = params.height || 0
          maxX = Math.max(maxX, r * 2)
          maxY = Math.max(maxY, r * 2)
          maxZ = Math.max(maxZ, h)
        } else if (geom instanceof THREE.SphereGeometry && params) {
          const r = params.radius || 0
          maxX = Math.max(maxX, r * 2)
          maxY = Math.max(maxY, r * 2)
          maxZ = Math.max(maxZ, r * 2)
        } else {
          // Fallback: compute bounding box from geometry vertices (local space)
          geom.computeBoundingBox()
          if (geom.boundingBox) {
            const s = new THREE.Vector3()
            geom.boundingBox.getSize(s)
            maxX = Math.max(maxX, s.x)
            maxY = Math.max(maxY, s.y)
            maxZ = Math.max(maxZ, s.z)
          }
        }
      }
    })
  }

  if (maxX < 0.001 && maxY < 0.001 && maxZ < 0.001) return { x: 0.04, y: 0.04, z: 0.04 }

  return { x: Math.max(maxX, 0.005), y: Math.max(maxY, 0.005), z: Math.max(maxZ, 0.005) }
}

// Cache tinted GLB materials per (compId, sourceMaterialUUID) to avoid
// re-cloning identical materials for repeated instances (e.g., 8 servos).
// Cleared on each applyRichVisuals call to prevent stale material leaks.
const _tintedMatCache = new Map<string, THREE.MeshStandardMaterial>()

export type ComponentVisualTargetFrame = 'urdf_z_up'

/**
 * Local quaternion that orients a resolved `previewGroup` in the target frame.
 * Every previewGroup is authored Z-up (rich output is wrapped at construction,
 * GLBs are Z-up natively) and both the renderer and the editor ghost parent
 * it into a URDF Z-up link frame, so this is the identity; it stays the single
 * place to change if a Y-up consumer comes back.
 */
export function componentVisualWorldQuat(
  _authoredFrame: ComponentVisualAuthoredFrame,
  _target: ComponentVisualTargetFrame,
): THREE.Quaternion {
  void _authoredFrame; void _target
  return new THREE.Quaternion()
}

/**
 * Apply rich visuals to all preset-derived links in the parsed robot.
 * Call this after parseURDFToScene() and after adding the group to the scene.
 */
export function applyRichVisuals(
  parsedRobot: ParsedRobotLike,
  onMeshLoaded?: (linkName: string) => void,
  /** Look up a component's authoritative bounding box (mm) from the preset catalog.
   *  When provided, overrides measureLinkDims for multi-primitive components like
   *  servos whose parametric placeholders extend beyond the preset bbox (e.g. mounting
   *  ears at +15% X) and would otherwise distort GLB scaling. Returning null keeps the
   *  current measureLinkDims behavior (right for single-primitive components like
   *  extrusions, where per-instance length_mm is already in the URDF box). */
  getPresetBoundingBoxMm?: (compId: string) => [number, number, number] | null,
  /** Links whose URDF primitives ARE the authored design (`link_geometry`
   *  body shells). The rich pass must NOT replace their geometry — swapping
   *  in the donor preset's stock visual silently hides the sculpted body
   *  while bounds/collision/placement keep using the real shell (the
   *  "baseplate always looks the same" defect). */
  hasCustomGeometry?: (linkName: string) => boolean,
): void {
  // Dispose and clear previous tinted materials
  for (const mat of _tintedMatCache.values()) mat.dispose()
  _tintedMatCache.clear()
  for (const [linkName, linkGroup] of parsedRobot.linkGroups) {
    // Authored body shells render their URDF primitives verbatim.
    if (hasCustomGeometry?.(linkName)) continue
    const compId = componentIdForLink(linkName)
    if (!compId) continue

    // Prefer the preset's authoritative bbox over measured placeholder geometry
    // (which over-counts ears/horns for multi-primitive components like servos).
    // Falls back to measureLinkDims for components without a 3-tuple bbox preset.
    const presetBbox = getPresetBoundingBoxMm?.(compId) ?? null
    const dims: GeneratorDims = presetBbox
      ? { x: presetBbox[0] / 1000, y: presetBbox[1] / 1000, z: presetBbox[2] / 1000 }
      : measureLinkDims(linkGroup)

    const resolved = resolveComponentVisual({
      preset: makeResolverPreset(compId, dims),
      linkName,
      materialCache: _tintedMatCache,
      castShadow: true,
      receiveShadow: true,
    })

    // Check for a real mesh override. Cached meshes are applied through the
    // resolver; uncached meshes keep the resolver fallback while loading.
    const meshUrl = getMeshOverrideUrl(compId)
    if (meshUrl) {
      // Check cache first — reuse previously loaded GLB mesh
      if (resolved.source === 'mesh' && resolved.previewGroup) {
        applyResolvedVisualToLink(resolved, linkGroup)
        onMeshLoaded?.(linkName)
        continue
      }
      // Async load — use parametric until GLB is ready
      if (!isMeshLoadInProgress(compId)) {
        markMeshLoadInProgress(compId)
        loadMeshOverride(meshUrl, linkName, linkGroup, dims, compId, onMeshLoaded)
      }
      // Fall through to parametric generation as placeholder
    }

    if (!resolved.previewGroup) continue
    const richGroup = resolved.previewGroup

    // Find the geometryGroup (first child of linkGroup that contains any mesh descendants)
    const geometryChild = linkGroup.children.find(c => {
      if (!(c instanceof THREE.Group)) return false
      let hasMesh = false
      c.traverse(gc => { if (gc instanceof THREE.Mesh) hasMesh = true })
      return hasMesh
    }) as THREE.Group | undefined

    if (geometryChild) {
      // Dispose old primitive geometry to prevent GPU memory leaks
      geometryChild.traverse(child => {
        if (child instanceof THREE.Mesh) {
          child.geometry?.dispose()
          // Don't dispose shared defaultMat — only dispose per-visual materials
          if (child.material && 'dispose' in child.material) {
            const mat = child.material as THREE.Material
            if (mat.uuid) {
              // Only dispose if it's not the shared default material
              // (materials from the preset system always have a color set)
              const stdMat = mat as THREE.MeshStandardMaterial
              if (stdMat.color && stdMat.color.getHex() !== 0x888888) {
                mat.dispose()
              }
            }
          }
        } else if (child instanceof THREE.LineSegments) {
          // Feature-edge LineSegments (added by addEdgeLines) own their
          // EdgesGeometry — dispose it so async mesh replacement doesn't leak.
          child.geometry?.dispose()
        }
      })

      const visualOrigin = urdfVisualOrigin(geometryChild)
      while (geometryChild.children.length > 0) {
        geometryChild.remove(geometryChild.children[0])
      }

      // The resolver's previewGroup is authored in the catalog (URDF Z-up)
      // frame; place it at the link's <visual><origin>.
      richGroup.position.copy(visualOrigin.position)
      richGroup.quaternion.copy(visualOrigin.quaternion)
        .multiply(componentVisualWorldQuat(resolved.authoredFrame, 'urdf_z_up'))
      richGroup.scale.set(1, 1, 1)
      geometryChild.add(richGroup)
    }
  }
}

function makeResolverPreset(compId: string, dims: GeneratorDims): ComponentVisualPresetLike {
  return {
    id: compId,
    physical: {
      bounding_box_mm: [
        Math.max(1, dims.x * 1000),
        Math.max(1, dims.y * 1000),
        Math.max(1, dims.z * 1000),
      ],
    },
    mechanical_electrical: {},
  }
}

function resolveAndApplyLoadedMesh(
  linkName: string,
  linkGroup: THREE.Group,
  dims: GeneratorDims,
  compId: string,
): boolean {
  const resolved = resolveComponentVisual({
    preset: makeResolverPreset(compId, dims),
    linkName,
    materialCache: _tintedMatCache,
    castShadow: true,
    receiveShadow: true,
  })
  if (resolved.source !== 'mesh' || !resolved.previewGroup) return false
  applyResolvedVisualToLink(resolved, linkGroup)
  return true
}

/**
 * The link's URDF `<visual><origin>` (urdfParser puts xyz/rpy on the visual
 * group), so a catalog part can sit off its link frame, e.g. a limb segment
 * whose link frame is its joint pivot. The designer compiler emits one visual
 * per catalog link and no rpy, so the rotation is normally identity.
 *
 * The rotation is honoured only when the link has a single visual: older
 * Vector URDFs decompose a part into several primitive <visual>s, each with
 * the pose of that primitive (a wheel's first cylinder carries rpy="pi/2 0 0"),
 * not of the part; applying it to the whole replacement visual lays wheels
 * flat.
 */
function urdfVisualOrigin(geometryChild: THREE.Group): { position: THREE.Vector3; quaternion: THREE.Quaternion } {
  const first = geometryChild.children[0]
  if (!first) return { position: new THREE.Vector3(), quaternion: new THREE.Quaternion() }
  return {
    position: first.position.clone(),
    quaternion: geometryChild.children.length === 1 ? first.quaternion.clone() : new THREE.Quaternion(),
  }
}

function applyResolvedVisualToLink(
  resolved: ResolvedComponentVisual,
  linkGroup: THREE.Group,
): void {
  const resolvedGroup = resolved.previewGroup
  if (!resolvedGroup) return

  const geometryChild = linkGroup.children.find(c => {
    if (!(c instanceof THREE.Group)) return false
    let hasMesh = false
    c.traverse(gc => { if (gc instanceof THREE.Mesh) hasMesh = true })
    return hasMesh
  }) as THREE.Group | undefined
  if (!geometryChild) return

  const visualOrigin = urdfVisualOrigin(geometryChild)
  while (geometryChild.children.length > 0) {
    geometryChild.remove(geometryChild.children[0])
  }
  resolvedGroup.position.copy(visualOrigin.position)
  resolvedGroup.quaternion.copy(visualOrigin.quaternion)
    .multiply(componentVisualWorldQuat(resolved.authoredFrame, 'urdf_z_up'))
  resolvedGroup.scale.set(1, 1, 1)
  geometryChild.add(resolvedGroup)
}

/**
 * Async load a component's GLB, cache it, and apply it to the link.
 * Calls onMeshLoaded(linkName) after the mesh is applied so callers can re-run
 * node placement that depends on actual rendered geometry (e.g. rebuildMountNodes).
 */
async function loadMeshOverride(
  meshUrl: string,
  linkName: string,
  linkGroup: THREE.Group,
  dims: GeneratorDims,
  compId: string,
  onMeshLoaded?: (linkName: string) => void,
): Promise<void> {
  try {
    const meshGroup = await loadGLB(meshUrl)

    // Cache the raw parsed mesh (before material/scaling)
    setCachedMeshGroup(compId, meshGroup)
    clearMeshLoadInProgress(compId)

    // Apply to the current link through the same resolver used by cached meshes.
    resolveAndApplyLoadedMesh(linkName, linkGroup, dims, compId)
    onMeshLoaded?.(linkName)

    console.log(`[richVisuals] Mesh loaded and cached: ${compId} (${meshUrl})`)
  } catch (e) {
    console.warn(`[richVisuals] Mesh failed for ${compId}, parametric fallback:`, e)
    clearMeshLoadInProgress(compId)
  }
}

/** Warm the mesh cache for a single component without applying it to any link.
 *  Call when the user enters carry mode for a new part — by the time they click
 *  to commit, the GLB is in cache so the carry ghost and the placed model use
 *  the same geometry. Without this, first-of-type carry shows the parametric
 *  fallback while the placed component renders the (later-loaded) GLB, and the
 *  parametric's larger envelope causes liftAboveFloor to over-lift on commit.
 *
 *  Does NOT touch markMeshLoadInProgress — that flag is owned by loadMeshOverride
 *  so applyRichVisuals can correctly decide whether to start its own load. If the
 *  preload and a live load race, both write to the same cache; the redundant
 *  network round-trip is the price of not blocking the live applyRichVisuals path
 *  (which needs to drive the link-application + onMeshLoaded callback chain).
 *
 *  `onReady` fires once the cache is populated (preload success OR a concurrent
 *  load completing first). Lets the carry ghost rebuild itself with the GLB
 *  geometry mid-carry — without it, the ghost stays as parametric while the
 *  eventual placed model uses the GLB, producing a visible rotation/shape
 *  mismatch between ghost and placed. */
export function preloadComponentMesh(compId: string, onReady?: () => void): void {
  if (hasCachedMeshGroup(compId)) { onReady?.(); return }
  const meshUrl = getMeshOverrideUrl(compId)
  if (!meshUrl) { onReady?.(); return }
  loadGLB(meshUrl)
    .then(group => {
      // Don't clobber if a concurrent live load already cached it — the live
      // load's applied mesh is already wired into a real link, ours isn't.
      if (!hasCachedMeshGroup(compId)) setCachedMeshGroup(compId, group)
      onReady?.()
    })
    .catch(e => {
      console.warn(`[richVisuals] preload failed for ${compId}:`, e)
      onReady?.()
    })
}

/** Load a GLB file and return a Three.js Group. */
async function loadGLB(url: string): Promise<THREE.Group> {
  const { GLTFLoader } = await import('three/addons/loaders/GLTFLoader.js')
  const loader = new GLTFLoader()
  return new Promise((resolve, reject) => {
    loader.load(
      url,
      (gltf) => {
        const group = new THREE.Group()
        // Move all children from the scene into our group
        while (gltf.scene.children.length > 0) {
          group.add(gltf.scene.children[0])
        }
        resolve(group)
      },
      undefined,
      (err) => reject(err),
    )
  })
}

/**
 * Pre-warm the GLB mesh cache at app startup so the first applyRichVisuals call
 * can use real meshes instead of parametric fallback.
 *
 * Deduplicates by GLB URL — each unique file is fetched once, then stored in
 * meshCache for every component ID that maps to it.
 */
export async function preloadMeshCache(): Promise<void> {
  // Build URL → [compId, ...] map. Only mesh-rendered parts are listed in
  // MESH_OVERRIDES, so every GLB fetched here is actually used.
  const urlToCompIds = new Map<string, string[]>()
  for (const compId of Object.keys(MESH_OVERRIDES)) {
    const url = getMeshOverrideUrl(compId)
    if (!url) continue
    const list = urlToCompIds.get(url)
    if (list) list.push(compId)
    else urlToCompIds.set(url, [compId])
  }

  const results = await Promise.allSettled(
    Array.from(urlToCompIds.entries()).map(async ([url, compIds]) => {
      const meshGroup = await loadGLB(url)
      // Store the same parsed mesh for every component ID sharing this GLB;
      // the resolver scales a clone to each part's bbox.
      for (const compId of compIds) {
        setCachedMeshGroup(compId, meshGroup)
        clearMeshLoadInProgress(compId)
      }
      return compIds.length
    }),
  )

  let loaded = 0, failed = 0
  for (const r of results) {
    if (r.status === 'fulfilled') loaded += r.value
    else failed++
  }
  console.log(`[richVisuals] GLB cache preloaded: ${loaded} components from ${results.length - failed} files (${failed} failed)`)
}
