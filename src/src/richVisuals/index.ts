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
import { findRichGenerator } from './generators'
import type { GeneratorDims } from './generators'
import { getMeshOverrideUrl, getRotationOverride, getShaftOverlay, getStepFallbackUrl, MESH_OVERRIDES } from './meshOverrides'
import { getComponentColor, getMaterial, getTintedMaterial } from './materials'
import { getRenderedMeshDims as _getRenderedMeshDims, setRenderedMeshDims } from '../meshDimsCache'


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
// Cache loaded STEP meshes so they survive reparse cycles
const meshCache = new Map<string, THREE.Group>()  // compId → cloneable mesh group
const loadingInProgress = new Set<string>()  // prevent duplicate loads
// Rendered mesh AABB cache is owned by ../meshDimsCache so pure/node-runnable
// modules can read it through componentDims without pulling in this module's
// directory-import dependency tree. Re-exported here for source-compat with
// existing urdfAssembly imports.
export const getRenderedMeshDims = _getRenderedMeshDims

/** Return the raw parsed GLB/STEP mesh group for a component, or null if
 *  the cache hasn't been populated yet. Caller should treat the returned
 *  group as READ-ONLY (shared across every instance of that component).
 *  Used by the runtime ICP nudge in urdfAssembly.ts to raycast against the
 *  child's actual mesh surface when the child isn't yet in the scene
 *  (new-component adds). The group is in its raw authored units (GLBs from
 *  our STEP converter are mm; scale before raycasting against meter-space
 *  origins). */
export function getCachedMeshGroup(compId: string): THREE.Group | null {
  return meshCache.get(compId) ?? null
}
// Component IDs whose meshes are too large/slow to load at runtime — use parametric instead.
// Includes: no GLB available (STEP >25MB skipped), or GLB >10MB.
export const SLOW_MESH_BLACKLIST = new Set([
  // No GLB (STEP files blacklisted from conversion: >25MB)
  'compute_sbc_gpu',                   // sbc_gpu.stp — 71MB STEP
  'mobility_track_tread_system',       // mobility_track.step — 50MB STEP
  // GLB still >10MB (too slow to fetch+parse at runtime)
  'actuator_bldc_small',               // bldc_outrunner.glb — 15MB
  'actuator_bldc_large',               // bldc_outrunner.glb — 15MB
  'motor_hub_80mm',                    // motor_hub.glb — 11MB
  'motor_hub_120mm',                   // motor_hub.glb — 11MB
  'compute_foc_controller',            // compute_foc_controller.glb — 11MB
  // Wrong STEP file or broken geometry
  'transmission_rack_pinion_set',      // STEP is industrial-scale (2.4m), not robotics
  'motor_harmonic_drive_compact',      // STEP is a disc servo, not a harmonic drive
  'motor_harmonic_drive_large',        // same mislabeled STEP
])

// Cache tinted GLB materials per (compId, sourceMaterialUUID) to avoid
// re-cloning identical materials for repeated instances (e.g., 8 servos).
// Cleared on each applyRichVisuals call to prevent stale material leaks.
const _tintedMatCache = new Map<string, THREE.MeshStandardMaterial>()

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
    const compId = extractComponentId(linkName)
    if (!compId) continue

    const generator = findRichGenerator(compId)
    if (!generator) continue

    // Prefer the preset's authoritative bbox over measured placeholder geometry
    // (which over-counts ears/horns for multi-primitive components like servos).
    // Falls back to measureLinkDims for components without a 3-tuple bbox preset.
    const presetBbox = getPresetBoundingBoxMm?.(compId) ?? null
    const dims: GeneratorDims = presetBbox
      ? { x: presetBbox[0] / 1000, y: presetBbox[1] / 1000, z: presetBbox[2] / 1000 }
      : measureLinkDims(linkGroup)

    // Check for real mesh override (STEP file from manufacturer)
    const meshUrl = getMeshOverrideUrl(compId)
    if (meshUrl && !SLOW_MESH_BLACKLIST.has(compId)) {
      // Check cache first — reuse previously loaded STEP mesh
      if (meshCache.has(compId)) {
        const cached = meshCache.get(compId)!
        const clone = cached.clone(true)
        applyMeshToLink(clone, linkName, linkGroup, dims, compId)
        onMeshLoaded?.(linkName)
        continue
      }
      // Async load — use parametric until GLB is ready
      if (!loadingInProgress.has(compId)) {
        loadingInProgress.add(compId)
        loadMeshOverride(meshUrl, linkName, linkGroup, dims, compId, onMeshLoaded)
      }
      // Fall through to parametric generation as placeholder
    }

    // Generate rich visual group (parametric fallback)
    let richGroup: THREE.Group
    try {
      const compColor = getComponentColor(compId)
      richGroup = generator(compId, dims, compColor.tint)
    } catch (e) {
      console.warn(`[richVisuals] Generator failed for ${compId}:`, e)
      continue  // keep primitive visuals
    }

    // Tag all meshes for raycasting (preserve existing contract)
    richGroup.traverse(child => {
      if (child instanceof THREE.Mesh) {
        child.castShadow = true
        child.receiveShadow = true
        ;(child.userData as Record<string, unknown>).urdfLinkName = linkName
      }
    })

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

      // Preserve the first visual's rotation (URDF <visual rpy>). Generators
      // produce content in the component's natural frame — e.g. wheels with
      // axles along Z — and rely on the URDF visual rpy (typically π/2 about X
      // for wheels) to rotate that into the link's URDF frame. Without this,
      // wheels land flat on their sides and motor shafts point the wrong way.
      const firstVisual = geometryChild.children.find(c => c instanceof THREE.Group) as THREE.Group | undefined
      const visualQuat = firstVisual ? firstVisual.quaternion.clone() : new THREE.Quaternion()

      while (geometryChild.children.length > 0) {
        geometryChild.remove(geometryChild.children[0])
      }

      richGroup.position.set(0, 0, 0)
      richGroup.quaternion.copy(visualQuat)
      richGroup.scale.set(1, 1, 1)
      geometryChild.add(richGroup)
    }
  }
}

/** Apply a cached mesh group to a link, handling scaling and raycasting tags. */
function applyMeshToLink(
  meshGroup: THREE.Group,
  linkName: string,
  linkGroup: THREE.Group,
  dims: GeneratorDims,
  compId: string,
) {
  // Apply per-component color tint to GLB meshes.
  // Default-gray meshes get full material replacement; others get a cached tint blend.
  const compColor = getComponentColor(compId)
  const catMat = getTintedMaterial(compColor.material, ...compColor.tint, compColor.strength ?? 0.4)
  const tintColor = new THREE.Color(compColor.tint[0], compColor.tint[1], compColor.tint[2])
  const tintStrength = compColor.strength ?? 0.4
  // Materials where the real-world color dominates the PBR look and should
  // never be softened by a source-material blend. Rubber parts are physically
  // black regardless of whatever generic color the STEP converter emitted.
  const forceReplaceMaterials = new Set(['rubber_black'])
  const forceReplace = forceReplaceMaterials.has(compColor.material)
  meshGroup.traverse(child => {
    if (child instanceof THREE.Mesh) {
      const mat = child.material as THREE.MeshStandardMaterial
      const isDefaultGray = mat?.color &&
        Math.abs(mat.color.r - 0.533) < 0.05 &&
        Math.abs(mat.color.g - 0.533) < 0.05 &&
        Math.abs(mat.color.b - 0.533) < 0.05
      if (forceReplace || isDefaultGray) {
        child.material = catMat
      } else if (mat?.color) {
        // Cache tinted materials per (compId, sourceMaterial) to share across instances
        const cacheKey = `${compId}_${mat.uuid}`
        let tinted = _tintedMatCache.get(cacheKey)
        if (!tinted) {
          tinted = mat.clone()
          tinted.color.lerp(tintColor, tintStrength)
          _tintedMatCache.set(cacheKey, tinted)
        }
        child.material = tinted
      }
      child.castShadow = true
      child.receiveShadow = true
      ;(child.userData as Record<string, unknown>).urdfLinkName = linkName
    }
  })

  // Normalize units: GLB files from our STEP converter are in mm.
  // Detect by comparing raw mesh size to expected size (in meters).
  // If mesh is >10x larger than expected, assume mm → convert to m.
  const meshBox = new THREE.Box3().setFromObject(meshGroup)
  const meshSize = new THREE.Vector3()
  meshBox.getSize(meshSize)
  const maxMeshDim = Math.max(meshSize.x, meshSize.y, meshSize.z)
  const maxExpectedDim = Math.max(dims.x, dims.y, dims.z)

  if (maxMeshDim > 0.0001) {
    if (maxMeshDim > maxExpectedDim * 10) {
      meshGroup.scale.setScalar(0.001) // mm → m
    }

    // Apply per-component rotation override by baking it into the geometry
    // vertices BEFORE per-axis scaling. Setting meshGroup.rotation alone
    // would not work: Three.js composes T*R*S, so a non-uniform scale.[xyz]
    // applied after rotation acts on the original local axes — the per-axis
    // scaling below would modify the wrong axis. Baking the rotation into
    // the geometry realigns local axes with the desired world axes, so
    // scale.x correctly controls the world X extent, etc.
    // We clone the geometry first so the shared cached mesh isn't mutated.
    const rotation = getRotationOverride(compId)
    if (rotation) {
      const rotMatrix = new THREE.Matrix4().makeRotationFromEuler(
        new THREE.Euler(rotation[0], rotation[1], rotation[2], 'XYZ'),
      )
      meshGroup.traverse(child => {
        if (child instanceof THREE.Mesh && child.geometry) {
          child.geometry = child.geometry.clone()
          child.geometry.applyMatrix4(rotMatrix)
        }
      })
      meshGroup.updateMatrixWorld(true)
    }

    // Per-axis scaling: scale GLB to match the component's declared bounding_box_mm.
    // Skip odd-shaped components (grippers, end effectors) whose GLB meshes don't
    // scale cleanly along independent axes.
    const PERAXIS_BLACKLIST = ['gripper', 'effector', 'claw', 'suction']
    const skipPerAxis = PERAXIS_BLACKLIST.some(k => compId.includes(k))
    // Shaft overlay: when this component has an entry in SHAFT_OVERLAYS, the GLB
    // body is scaled to (bbox.z - shaft_length) and a procedural cylinder fills
    // the remaining shaft_length region above. This lets the placement engine
    // align the body face to a coupler bottom (Layer 4 child connector path)
    // while keeping the shaft visible inside the coupler bore.
    const shaftOverlay = getShaftOverlay(compId)
    const shaftLenM = shaftOverlay ? shaftOverlay.shaft_length_mm / 1000 : 0
    const targetZ = shaftOverlay ? Math.max(0.001, dims.z - shaftLenM) : dims.z
    if (!skipPerAxis) {
      meshBox.setFromObject(meshGroup)
      meshBox.getSize(meshSize)
      if (meshSize.x > 0.0001 && meshSize.y > 0.0001 && meshSize.z > 0.0001) {
        const scaleX = dims.x / meshSize.x
        const scaleY = dims.y / meshSize.y
        const scaleZ = targetZ / meshSize.z
        meshGroup.scale.x *= scaleX
        meshGroup.scale.y *= scaleY
        meshGroup.scale.z *= scaleZ
      }
    }

    // Center the mesh on origin so it sits properly in the link frame
    meshBox.setFromObject(meshGroup)
    const center = new THREE.Vector3()
    meshBox.getCenter(center)
    meshGroup.position.sub(center)

    // Shaft overlay: shift the now-centered body DOWN by shaft_length/2 so its
    // top face sits at z = (bbox.z/2 - shaft_length) = body_top, and the upper
    // shaft_length region is empty for the procedural cylinder added below.
    if (shaftOverlay) {
      meshGroup.position.z -= shaftLenM / 2
    }

    // Cache the actual rendered size (full extents in meters) as the authoritative
    // dimension source for ghost bounds and node placement. For shaft-overlay
    // components, measure the BODY mesh only (the procedural shaft cylinder is
    // added to `geometryChild` below, AFTER this measurement; it doesn't appear
    // in `finalBox`). Previously we overwrote `finalSize.z = dims.z` here to
    // report the full body+shaft envelope, but that meant getParentBounds →
    // getRenderedMeshDims returned 34mm for high_torque when the actual mating
    // face sits at ±14.5mm; placement stacked children 2.5-4mm beyond the real
    // body surface, and the post-reconcile ICP saw a `p90 ≈ 4mm` gap that
    // exceeded the 3mm cap (clamp → visible under-engage). Reporting body-only
    // dims lets placement math land the child against the real body extent
    // instead of the cosmetic envelope. Ghost bounds lose a few mm of "shaft"
    // visualization but gain correctness — acceptable tradeoff.
    const finalBox = new THREE.Box3().setFromObject(meshGroup)
    const finalSize = new THREE.Vector3()
    finalBox.getSize(finalSize)
    if (finalSize.x > 0.001 || finalSize.y > 0.001 || finalSize.z > 0.001) {
      setRenderedMeshDims(compId, finalSize)
    }
  }

  // Replace geometry in link group
  const geometryChild = linkGroup.children.find(c => {
    if (!(c instanceof THREE.Group)) return false
    let hasMesh = false
    c.traverse(gc => { if (gc instanceof THREE.Mesh) hasMesh = true })
    return hasMesh
  }) as THREE.Group | undefined

  if (geometryChild) {
    while (geometryChild.children.length > 0) {
      geometryChild.remove(geometryChild.children[0])
    }
    geometryChild.add(meshGroup)

    // Shaft overlay: add a procedural shaft cylinder above the body. Sibling to
    // meshGroup (not its child) so meshGroup's per-axis scale doesn't deform it.
    // Cylinder is along URDF +Z, centered in the upper shaft_length region of
    // the bbox, with brushed-steel material to match catalog shaft features
    // (shaft_collar, hex_standoff stubs, etc.).
    const overlay = getShaftOverlay(compId)
    if (overlay) {
      const shaftLength = overlay.shaft_length_mm / 1000
      const shaftRadius = overlay.shaft_radius_mm / 1000
      const shaftMat = getMaterial('brushed_steel')
      const shaftGeo = new THREE.CylinderGeometry(shaftRadius, shaftRadius, shaftLength, 24)
      const shaftMesh = new THREE.Mesh(shaftGeo, shaftMat)
      shaftMesh.rotation.x = Math.PI / 2 // align cylinder Y axis with URDF +Z
      shaftMesh.position.z = dims.z / 2 - shaftLength / 2
      shaftMesh.castShadow = true
      shaftMesh.receiveShadow = true
      ;(shaftMesh.userData as Record<string, unknown>).urdfLinkName = linkName
      geometryChild.add(shaftMesh)
    }
  }
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
    meshCache.set(compId, meshGroup)
    loadingInProgress.delete(compId)

    // Apply to the current link
    const clone = meshGroup.clone(true)
    applyMeshToLink(clone, linkName, linkGroup, dims, compId)
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
          meshCache.set(compId, meshGroup)
          loadingInProgress.delete(compId)
          const clone = meshGroup.clone(true)
          applyMeshToLink(clone, linkName, linkGroup, dims, compId)
          onMeshLoaded?.(linkName)
          console.log(`[richVisuals] STEP fallback loaded: ${compId}`)
          return
        } catch (e2) {
          console.warn(`[richVisuals] STEP fallback also failed for ${compId}:`, e2)
        }
      }
    }
    console.warn(`[richVisuals] Mesh failed for ${compId}, parametric fallback:`, e)
    loadingInProgress.delete(compId)
  }
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
      // shared-GLB components (all variants would get the same dims). applyMeshToLink
      // sets meshDimsCache after per-axis scaling to bounding_box_mm on first placement.
      // computeCarryGhostBounds falls back to bounding_box_mm directly when no dims cached.
      for (const compId of compIds) {
        meshCache.set(compId, meshGroup)
        loadingInProgress.delete(compId)
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
