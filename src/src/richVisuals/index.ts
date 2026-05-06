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
import { getMeshOverrideUrl, getStepFallbackUrl, MESH_OVERRIDES, SLOW_MESH_BLACKLIST } from './meshOverrides'
import { setRenderedMeshDims } from '../meshDimsCache'
import { resolveComponentVisual } from '../componentVisualResolver'
import type {
  ComponentVisualAuthoredFrame,
  ComponentVisualPresetLike,
  ComponentVisualSource,
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
 * Extract the component ID from a link name.
 * Link names use the pattern: "{component_id}_{instance_number}"
 * e.g., "actuator_servo_high_torque_4" → "actuator_servo_high_torque"
 *
 * Returns null if the link name doesn't match a preset pattern.
 */
function extractComponentId(linkName: string): string | null {
  // Strip the trailing _N instance number
  const match = linkName.match(/^(.+)_(\d+)$/)
  if (!match) return null
  return match[1]
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

/**
 * Apply rich visuals to all preset-derived links in the parsed robot.
 * Call this after parseURDFToScene() and after adding the group to the scene.
 */
/** Return the raw parsed GLB/STEP mesh group for a component, or null if
 *  the cache hasn't been populated yet. Caller should treat the returned
 *  group as READ-ONLY (shared across every instance of that component).
 *  Used by the runtime ICP nudge in urdfAssembly.ts to raycast against the
 *  child's actual mesh surface when the child isn't yet in the scene
 *  (new-component adds). The group is in its raw authored units (GLBs from
 *  our STEP converter are mm; scale before raycasting against meter-space
 *  origins). */
export { getCachedMeshGroup } from './meshCache'
// Cache tinted GLB materials per (compId, sourceMaterialUUID) to avoid
// re-cloning identical materials for repeated instances (e.g., 8 servos).
// Cleared on each applyRichVisuals call to prevent stale material leaks.
const _tintedMatCache = new Map<string, THREE.MeshStandardMaterial>()

const URDF_Z_UP_TO_SCENE_Y_UP = new THREE.Quaternion().setFromEuler(
  new THREE.Euler(-Math.PI / 2, 0, 0, 'XYZ'),
)

export type ComponentVisualTargetFrame = 'urdf_z_up' | 'scene_y_up'

/**
 * Compute the local-quaternion to apply to a resolved component's
 * `previewGroup` so it lands oriented in the requested target frame.
 *
 * Authored-frame unification: every previewGroup is Z-up authored — rich
 * outputs are wrapped at construction time, GLBs are Z-up natively. So this
 * adapter is a function of `target` only; `authoredFrame` is accepted for
 * API stability but no longer branches behavior. Carry parents into the
 * scene Y-up world (apply -90° X), render parents into the URDF Z-up link
 * group (identity).
 */
export function componentVisualWorldQuat(
  _authoredFrame: ComponentVisualAuthoredFrame,
  target: ComponentVisualTargetFrame,
): THREE.Quaternion {
  void _authoredFrame
  if (target === 'urdf_z_up') return new THREE.Quaternion()
  return URDF_Z_UP_TO_SCENE_Y_UP.clone()
}

/**
 * Backwards-compatible alias used by the parity corpus and any caller that
 * still thinks in source-rather-than-frame terms. After unification both
 * sources are Z-up authored, so this always returns identity (the URDF
 * link-frame target).
 */
export function renderVisualQuaternionForSource(
  _source: ComponentVisualSource,
  _visualQuat?: THREE.Quaternion,
): THREE.Quaternion {
  void _source; void _visualQuat
  return new THREE.Quaternion()
}

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
): void {
  // Dispose and clear previous tinted materials
  for (const mat of _tintedMatCache.values()) mat.dispose()
  _tintedMatCache.clear()
  for (const [linkName, linkGroup] of parsedRobot.linkGroups) {
    // Split-servo body/horn links (`<id>_<N>_body` / `_horn`) intentionally
    // skip the rich-visual replacement: the URDF primitives emitted by
    // `resolveSplitServoVisual` (servoBodyShape + servoHornShape) are the
    // single source of truth for split servos, and the carry ghost is forced
    // down the same primitive path in `computeCarryGhostPreview`. Replacing
    // them here would re-create the carry-vs-placed mismatch.
    const compId = extractComponentId(linkName)
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
      category: inferComponentCategory(compId),
      linkName,
      materialCache: _tintedMatCache,
      castShadow: true,
      receiveShadow: true,
    })

    // Check for a real mesh override. Cached meshes are applied through the
    // resolver; uncached meshes keep the resolver fallback while loading.
    const meshUrl = getMeshOverrideUrl(compId)
    if (meshUrl && !SLOW_MESH_BLACKLIST.has(compId)) {
      // Check cache first — reuse previously loaded STEP mesh
      if (resolved.source === 'mesh' && resolved.previewGroup) {
        applyResolvedRenderVisual(resolved, linkGroup)
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

      while (geometryChild.children.length > 0) {
        geometryChild.remove(geometryChild.children[0])
      }

      // Single shared adapter: take the resolver's previewGroup (authored
      // either Y-up for rich or Z-up for mesh) and rotate it into the URDF
      // Z-up link frame. The URDF primitive's `<visual rpy>` is intentionally
      // ignored here — that rpy was tuned to orient the primitive cylinder,
      // not the rich group, and re-applying it stacked an extra Rx(π/2) on
      // wheels (the load-bearing piece of the post-commit "flat tire" bug).
      richGroup.position.set(0, 0, 0)
      richGroup.quaternion.copy(componentVisualWorldQuat(resolved.authoredFrame, 'urdf_z_up'))
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

function inferComponentCategory(compId: string): string {
  if (compId.startsWith('actuator_')) return 'actuators'
  if (compId.startsWith('motor_')) return 'motors'
  if (compId.startsWith('sensor_')) return 'sensors'
  if (compId.startsWith('compute_')) return 'compute'
  if (compId.startsWith('power_')) return 'power'
  if (compId.startsWith('transmission_')) return 'transmission'
  if (compId.startsWith('effector_')) return 'end_effectors'
  if (compId.startsWith('mobility_')) return 'mobility'
  return 'structural'
}

function resolveAndApplyLoadedMesh(
  linkName: string,
  linkGroup: THREE.Group,
  dims: GeneratorDims,
  compId: string,
): boolean {
  const resolved = resolveComponentVisual({
    preset: makeResolverPreset(compId, dims),
    category: inferComponentCategory(compId),
    linkName,
    materialCache: _tintedMatCache,
    castShadow: true,
    receiveShadow: true,
  })
  if (resolved.source !== 'mesh' || !resolved.previewGroup) return false
  applyResolvedRenderVisual(resolved, linkGroup)
  return true
}

function applyResolvedRenderVisual(
  resolved: ResolvedComponentVisual,
  linkGroup: THREE.Group,
): void {
  applyResolvedVisualToLink(resolved, linkGroup)
  if (
    resolved.renderedBodySize &&
    (resolved.renderedBodySize.x > 0.001 || resolved.renderedBodySize.y > 0.001 || resolved.renderedBodySize.z > 0.001)
  ) {
    setRenderedMeshDims(resolved.componentId, resolved.renderedBodySize)
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

  while (geometryChild.children.length > 0) {
    geometryChild.remove(geometryChild.children[0])
  }
  resolvedGroup.position.set(0, 0, 0)
  resolvedGroup.quaternion.copy(componentVisualWorldQuat(resolved.authoredFrame, 'urdf_z_up'))
  resolvedGroup.scale.set(1, 1, 1)
  geometryChild.add(resolvedGroup)
}

/**
 * Async load a mesh (GLB preferred, STEP fallback), cache it, and apply to the link.
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
  const ext = meshUrl.split('.').pop()?.toLowerCase() || ''
  try {
    let meshGroup: THREE.Group

    if (ext === 'glb' || ext === 'gltf') {
      // Load GLB — fast path (~100ms)
      meshGroup = await loadGLB(meshUrl)
    } else {
      // STEP/STP fallback — slow path (5-15s)
      meshGroup = await loadSTEP(meshUrl)
    }

    // Cache the raw parsed mesh (before material/scaling)
    setCachedMeshGroup(compId, meshGroup)
    clearMeshLoadInProgress(compId)

    // Apply to the current link through the same resolver used by cached meshes.
    resolveAndApplyLoadedMesh(linkName, linkGroup, dims, compId)
    onMeshLoaded?.(linkName)

    console.log(`[richVisuals] Mesh loaded and cached: ${compId} (${meshUrl})`)
  } catch (e) {
    // If GLB failed, try STEP fallback
    if (ext === 'glb' || ext === 'gltf') {
      const stepUrl = getStepFallbackUrl(compId)
      if (stepUrl) {
        console.warn(`[richVisuals] GLB failed for ${compId}, trying STEP fallback...`)
        try {
          const meshGroup = await loadSTEP(stepUrl)
          setCachedMeshGroup(compId, meshGroup)
          clearMeshLoadInProgress(compId)
          resolveAndApplyLoadedMesh(linkName, linkGroup, dims, compId)
          onMeshLoaded?.(linkName)
          console.log(`[richVisuals] STEP fallback loaded: ${compId}`)
          return
        } catch (e2) {
          console.warn(`[richVisuals] STEP fallback also failed for ${compId}:`, e2)
        }
      }
    }
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
  if (SLOW_MESH_BLACKLIST.has(compId)) { onReady?.(); return }
  if (hasCachedMeshGroup(compId)) { onReady?.(); return }
  const meshUrl = getMeshOverrideUrl(compId)
  if (!meshUrl) { onReady?.(); return }
  const ext = meshUrl.split('.').pop()?.toLowerCase() || ''
  const loader = ext === 'glb' || ext === 'gltf' ? loadGLB(meshUrl) : loadSTEP(meshUrl)
  loader
    .then(group => {
      // Don't clobber if a concurrent live load already cached it — the live
      // load's applied mesh is already wired into a real link, ours isn't.
      if (!hasCachedMeshGroup(compId)) setCachedMeshGroup(compId, group)
      onReady?.()
    })
    .catch(async e => {
      if (ext === 'glb' || ext === 'gltf') {
        const stepUrl = getStepFallbackUrl(compId)
        if (stepUrl) {
          try {
            const group = await loadSTEP(stepUrl)
            if (!hasCachedMeshGroup(compId)) setCachedMeshGroup(compId, group)
            onReady?.()
            return
          } catch {/* fall through */}
        }
      }
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

/** Load a STEP file via OpenCascade WASM and return a Three.js Group. */
async function loadSTEP(url: string): Promise<THREE.Group> {
  const response = await fetch(url)
  if (!response.ok) throw new Error(`HTTP ${response.status}`)
  const buffer = await response.arrayBuffer()

  if (buffer.byteLength > 20 * 1024 * 1024) {
    throw new Error(`STEP file too large (${(buffer.byteLength / 1024 / 1024).toFixed(1)}MB)`)
  }

  const { parseSTEP } = await import('../stepLoader')
  return parseSTEP(buffer)
}

/**
 * Pre-warm the GLB mesh cache at app startup so the first applyRichVisuals call
 * can use real meshes instead of parametric fallback.
 *
 * Deduplicates by GLB URL — each unique file is fetched once, then stored in
 * meshCache for every component ID that maps to it.
 */
export async function preloadMeshCache(): Promise<void> {
  // Build URL → [compId, ...] map, skipping blacklisted components
  const urlToCompIds = new Map<string, string[]>()
  for (const compId of Object.keys(MESH_OVERRIDES)) {
    if (SLOW_MESH_BLACKLIST.has(compId)) continue
    const url = getMeshOverrideUrl(compId)
    if (!url) continue
    const list = urlToCompIds.get(url)
    if (list) list.push(compId)
    else urlToCompIds.set(url, [compId])
  }

  const results = await Promise.allSettled(
    Array.from(urlToCompIds.entries()).map(async ([url, compIds]) => {
      const meshGroup = await loadGLB(url)
      // Store the same parsed mesh for every component ID sharing this GLB.
      // meshDimsCache is NOT pre-populated here — the raw GLB size is meaningless for
      // shared-GLB components (all variants would get the same dims). The resolver
      // records diagnostic meshDimsCache data after per-axis scaling on first placement.
      // Placement bounds do not depend on this cache.
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
