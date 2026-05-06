// Phase 1 smoke test: bake a single preset STEP via Replicad and drop it into
// the scene next to the existing per-preset GLB for visual comparison.
//
// Invoked from the devtools console during `cargo tauri dev`:
//
//     __bake('actuator_servo_standard')     // bake + side-by-side GLB
//     __bake('structural_bracket_l', 0.5)   // custom X offset (m)
//     __bakeClear()                         // remove all smoke-test meshes
//
// This is dev-only scaffolding and stays out of the normal render path. The
// Phase 2+ integration lives in the accept-click path inside viewportChat.ts
// once fuse+fillet is working.

import * as THREE from 'three'
import { bakeSinglePreset, bakeFuseTwo, bakeFuseParametric } from './bakeClient.ts'
import { buildBakedMesh } from './bakedThree.ts'
import { getMeshOverrideUrl, getStepFallbackUrl, hasMeshOverride } from '../richVisuals/meshOverrides.ts'
import type { FuseSpec, FuseParamSpec, ParametricDiscChild } from './types.ts'

interface SmokeTestDeps {
  scene: THREE.Scene
}

const SMOKE_GROUP_NAME = '__bake_smoke'
const GAP_M = 0.12  // side-by-side spacing between baked + GLB

function ensureSmokeGroup(scene: THREE.Scene): THREE.Group {
  let group = scene.getObjectByName(SMOKE_GROUP_NAME) as THREE.Group | undefined
  if (!group) {
    group = new THREE.Group()
    group.name = SMOKE_GROUP_NAME
    scene.add(group)
  }
  return group
}

async function loadSmokeGLB(url: string): Promise<THREE.Group> {
  const { GLTFLoader } = await import('three/addons/loaders/GLTFLoader.js')
  const loader = new GLTFLoader()
  return await new Promise((resolve, reject) => {
    loader.load(
      url,
      gltf => {
        const g = new THREE.Group()
        while (gltf.scene.children.length > 0) g.add(gltf.scene.children[0])
        resolve(g)
      },
      undefined,
      err => reject(err),
    )
  })
}

function bboxInfo(g: THREE.Object3D): { size: THREE.Vector3; min: THREE.Vector3; max: THREE.Vector3 } {
  const box = new THREE.Box3().setFromObject(g)
  const size = new THREE.Vector3()
  box.getSize(size)
  return { size, min: box.min.clone(), max: box.max.clone() }
}

/** Bake one preset via the worker and add it to the scene next to its GLB. */
export async function bakeAndShow(
  deps: SmokeTestDeps,
  presetId: string,
  offsetX = 0.4,
): Promise<void> {
  if (!hasMeshOverride(presetId)) {
    console.warn(`[bake-smoke] no mesh override for preset ${presetId}`)
    return
  }
  const stepUrl = getStepFallbackUrl(presetId)
  const glbUrl = getMeshOverrideUrl(presetId)
  if (!stepUrl || !glbUrl) {
    console.warn(`[bake-smoke] missing STEP/GLB URLs for ${presetId}`)
    return
  }

  const smokeGroup = ensureSmokeGroup(deps.scene)

  console.log(`[bake-smoke] baking ${presetId} from ${stepUrl}`)
  const t0 = performance.now()
  const outcome = await bakeSinglePreset(stepUrl)
  const dtMs = performance.now() - t0
  if (!outcome.ok) {
    console.error(`[bake-smoke] bake FAILED (${outcome.phase}): ${outcome.message}`)
    return
  }

  const bakedGroup = buildBakedMesh(outcome.mesh, outcome.edges, {
    material: new THREE.MeshStandardMaterial({ color: 0x8fb3ff, roughness: 0.4, metalness: 0.3 }),
    includeEdges: false,
  })
  bakedGroup.name = `baked-${presetId}`
  bakedGroup.position.set(offsetX, 0, 0)
  smokeGroup.add(bakedGroup)

  const bakedBBox = bboxInfo(bakedGroup)
  console.log(`[bake-smoke] ${presetId}: bake=${dtMs.toFixed(0)}ms ` +
    `tris=${outcome.diagnostics.triangleCount} verts=${outcome.diagnostics.vertexCount} ` +
    `import=${outcome.diagnostics.importMs.toFixed(0)}ms mesh=${outcome.diagnostics.meshMs.toFixed(0)}ms ` +
    `size=(${bakedBBox.size.x.toFixed(3)}, ${bakedBBox.size.y.toFixed(3)}, ${bakedBBox.size.z.toFixed(3)})m`)

  try {
    const glbGroup = await loadSmokeGLB(glbUrl)
    // GLB files are mm; existing richVisuals path scales them to meters via
    // per-component logic. For the smoke test we just apply the same 0.001
    // factor so the two meshes sit at comparable scale for visual diff.
    const glbBBoxRaw = bboxInfo(glbGroup)
    const maxRaw = Math.max(glbBBoxRaw.size.x, glbBBoxRaw.size.y, glbBBoxRaw.size.z)
    const maxBaked = Math.max(bakedBBox.size.x, bakedBBox.size.y, bakedBBox.size.z)
    if (maxRaw > maxBaked * 10) glbGroup.scale.setScalar(0.001)
    glbGroup.name = `glb-${presetId}`
    glbGroup.position.set(offsetX + (bakedBBox.size.x + GAP_M), 0, 0)
    smokeGroup.add(glbGroup)
    const glbBBox = bboxInfo(glbGroup)
    console.log(`[bake-smoke] GLB counterpart size=(${glbBBox.size.x.toFixed(3)}, ${glbBBox.size.y.toFixed(3)}, ${glbBBox.size.z.toFixed(3)})m`)
    const dx = Math.abs(bakedBBox.size.x - glbBBox.size.x)
    const dy = Math.abs(bakedBBox.size.y - glbBBox.size.y)
    const dz = Math.abs(bakedBBox.size.z - glbBBox.size.z)
    console.log(`[bake-smoke] bbox delta vs GLB: dx=${(dx*1000).toFixed(2)}mm dy=${(dy*1000).toFixed(2)}mm dz=${(dz*1000).toFixed(2)}mm`)
  } catch (e) {
    console.warn(`[bake-smoke] GLB load failed (non-fatal):`, e)
  }
}

// ── Phase 2: fuse + fillet smoke test ──

interface PresetConnectorLite {
  id: string
  origin_xyz_mm: [number, number, number]
  axis_xyz: [number, number, number]
}

let _presetCache: Map<string, { connectors: PresetConnectorLite[] }> | null = null

async function loadPresetConnectors(): Promise<Map<string, { connectors: PresetConnectorLite[] }>> {
  if (_presetCache) return _presetCache
  const res = await fetch('/generic_presets.json')
  if (!res.ok) throw new Error(`failed to fetch presets: HTTP ${res.status}`)
  // Shape: { categories: { <catName>: { components: Component[] } } } — matches
  // connectorInspector.ts's parser.
  const json = await res.json() as {
    categories: Record<string, { components: { id: string; connectors?: PresetConnectorLite[] }[] }>
  }
  const map = new Map<string, { connectors: PresetConnectorLite[] }>()
  for (const cat of Object.values(json.categories)) {
    for (const comp of cat.components) {
      map.set(comp.id, { connectors: comp.connectors ?? [] })
    }
  }
  _presetCache = map
  return map
}

async function resolveConnector(presetId: string, connectorId: string): Promise<PresetConnectorLite> {
  const presets = await loadPresetConnectors()
  const preset = presets.get(presetId)
  if (!preset) throw new Error(`preset ${presetId} not found`)
  const conn = preset.connectors.find(c => c.id === connectorId)
  if (!conn) throw new Error(`preset ${presetId} has no connector "${connectorId}" (available: ${preset.connectors.map(c => c.id).join(', ')})`)
  return conn
}

export interface FuseSmokeArgs {
  parent: { presetId: string; connectorId: string }
  child: { presetId: string; connectorId: string }
  filletRadiusMm?: number
  offsetX?: number
  /** Skip the fuse+fillet — just translate and mesh both parts. Use to
   *  diagnose whether placement/mesh work independently of the boolean. */
  skipFuse?: boolean
  fuseOptimisation?: 'none' | 'commonFace' | 'sameFace'
  simplifyImports?: boolean
  /** Override the default 30s timeout. Pass a larger number when testing a
   *  known-expensive STEP pair. */
  timeoutMs?: number
}

/** Bake a fuse+fillet of two preset parts, drop it into the scene, and side
 *  by side drop the un-fused parent+child for comparison. */
export async function fuseAndShow(deps: SmokeTestDeps, args: FuseSmokeArgs): Promise<void> {
  const { parent, child } = args
  const parentStepUrl = getStepFallbackUrl(parent.presetId)
  const childStepUrl = getStepFallbackUrl(child.presetId)
  if (!parentStepUrl || !childStepUrl) {
    console.warn(`[bake-smoke] missing STEP URLs: parent=${parentStepUrl} child=${childStepUrl}`)
    return
  }
  const [parentConn, childConn] = await Promise.all([
    resolveConnector(parent.presetId, parent.connectorId),
    resolveConnector(child.presetId, child.connectorId),
  ])

  const spec: FuseSpec = {
    parent: {
      stepUrl: parentStepUrl,
      connectorOriginMm: parentConn.origin_xyz_mm,
      connectorAxisXyz: parentConn.axis_xyz,
    },
    child: {
      stepUrl: childStepUrl,
      connectorOriginMm: childConn.origin_xyz_mm,
      connectorAxisXyz: childConn.axis_xyz,
    },
    filletRadiusMm: args.filletRadiusMm ?? 1.0,
    fuseOptimisation: args.fuseOptimisation,
    skipFuse: args.skipFuse,
    simplifyImports: args.simplifyImports,
  }

  const smokeGroup = ensureSmokeGroup(deps.scene)
  const offsetX = args.offsetX ?? 0.4
  const GAP = 0.15

  console.log(`[bake-smoke/fuse] ${parent.presetId}:${parent.connectorId} ↔ ${child.presetId}:${child.connectorId}`)
  console.log(`[bake-smoke/fuse] parent conn: origin=${parentConn.origin_xyz_mm.join(',')}mm axis=${parentConn.axis_xyz.join(',')}`)
  console.log(`[bake-smoke/fuse] child conn:  origin=${childConn.origin_xyz_mm.join(',')}mm axis=${childConn.axis_xyz.join(',')}`)

  const t0 = performance.now()
  const outcome = await bakeFuseTwo(spec, { timeoutMs: args.timeoutMs })
  const dtMs = performance.now() - t0

  if (!outcome.ok) {
    console.error(`[bake-smoke/fuse] FAILED (${outcome.phase}): ${outcome.message}`)
    return
  }

  const d = outcome.diagnostics
  console.log(`[bake-smoke/fuse] done in ${dtMs.toFixed(0)}ms — ` +
    `import=${d.importMs.toFixed(0)} fuse=${(d.fuseMs ?? 0).toFixed(0)} ` +
    `fillet=${(d.filletMs ?? 0).toFixed(0)} mesh=${d.meshMs.toFixed(0)}ms ` +
    `tris=${d.triangleCount}${d.filletFellBackToFuseOnly ? ' (fillet failed → fuse-only)' : ''}`)

  const fusedGroup = buildBakedMesh(outcome.mesh, outcome.edges, {
    material: new THREE.MeshStandardMaterial({ color: 0x8fb3ff, roughness: 0.4, metalness: 0.3 }),
    includeEdges: true,
    edgeMaterial: new THREE.LineBasicMaterial({ color: 0x1c2436 }),
  })
  fusedGroup.name = `fused-${parent.presetId}-${child.presetId}`
  fusedGroup.position.set(offsetX, 0, 0)
  smokeGroup.add(fusedGroup)
  const fusedBox = new THREE.Box3().setFromObject(fusedGroup)
  const fusedSize = new THREE.Vector3(); fusedBox.getSize(fusedSize)

  // Side-by-side unfused pair: bake each separately and hand-compose them with
  // the same translate the worker applied.
  try {
    const [parentOutcome, childOutcome] = await Promise.all([
      bakeSinglePreset(parentStepUrl),
      bakeSinglePreset(childStepUrl),
    ])
    if (parentOutcome.ok && childOutcome.ok) {
      const unfused = new THREE.Group()
      const parentMesh = buildBakedMesh(parentOutcome.mesh, undefined, {
        material: new THREE.MeshStandardMaterial({ color: 0xcccccc, roughness: 0.4, metalness: 0.3 }),
      })
      unfused.add(parentMesh)
      const childMesh = buildBakedMesh(childOutcome.mesh, undefined, {
        material: new THREE.MeshStandardMaterial({ color: 0xcccccc, roughness: 0.4, metalness: 0.3 }),
      })
      // Translate child by (parent_conn_origin - child_conn_origin), in meters.
      const dx = (parentConn.origin_xyz_mm[0] - childConn.origin_xyz_mm[0]) * 0.001
      const dy = (parentConn.origin_xyz_mm[1] - childConn.origin_xyz_mm[1]) * 0.001
      const dz = (parentConn.origin_xyz_mm[2] - childConn.origin_xyz_mm[2]) * 0.001
      childMesh.position.set(dx, dy, dz)
      unfused.add(childMesh)
      unfused.position.set(offsetX + fusedSize.x + GAP, 0, 0)
      unfused.name = `unfused-${parent.presetId}-${child.presetId}`
      smokeGroup.add(unfused)
    }
  } catch (e) {
    console.warn(`[bake-smoke/fuse] unfused counterpart render failed:`, e)
  }
}

// ── Phase 2: fuse + fillet with parametric child ──
//
// Used when the child preset's STEP is too complex for OCCT booleans (the
// coupler's 25-tooth splined bore gave 974 edges; fuse and simplify both
// timed out). Builds a simple disc-with-bore parametrically inside the worker
// as the child. Keeps the parent STEP authentic so the seam we see is real.

export interface FuseParamSmokeArgs {
  parent: { presetId: string; connectorId: string }
  child: {
    odMm: number
    idMm: number
    thicknessMm: number
    connectorLocalZMm: number  // signed z-offset of child's mate in local frame
    axisSign: 1 | -1            // child connector axis: -1 for "bottom face outward"
  }
  filletRadiusMm?: number
  offsetX?: number
  fuseOptimisation?: 'none' | 'commonFace' | 'sameFace'
  skipFuse?: boolean
  simplifyImports?: boolean
  interpenetrationMm?: number
  timeoutMs?: number
}

export async function fuseParamAndShow(deps: SmokeTestDeps, args: FuseParamSmokeArgs): Promise<void> {
  const parentStepUrl = getStepFallbackUrl(args.parent.presetId)
  if (!parentStepUrl) {
    console.warn(`[bake-smoke/fuseParam] no STEP URL for ${args.parent.presetId}`)
    return
  }
  const parentConn = await resolveConnector(args.parent.presetId, args.parent.connectorId)

  const childSpec: ParametricDiscChild = {
    kind: 'disc',
    odMm: args.child.odMm,
    idMm: args.child.idMm,
    thicknessMm: args.child.thicknessMm,
    connectorOriginMm: [0, 0, args.child.connectorLocalZMm],
    connectorAxisXyz: [0, 0, args.child.axisSign],
  }

  const spec: FuseParamSpec = {
    parent: {
      stepUrl: parentStepUrl,
      connectorOriginMm: parentConn.origin_xyz_mm,
      connectorAxisXyz: parentConn.axis_xyz,
    },
    child: childSpec,
    filletRadiusMm: args.filletRadiusMm ?? 0.8,
    fuseOptimisation: args.fuseOptimisation,
    skipFuse: args.skipFuse,
    simplifyImports: args.simplifyImports,
    interpenetrationMm: args.interpenetrationMm,
  }

  const smokeGroup = ensureSmokeGroup(deps.scene)
  const offsetX = args.offsetX ?? 0.4

  console.log(`[bake-smoke/fuseParam] ${args.parent.presetId}:${args.parent.connectorId} ↔ disc(od=${args.child.odMm}, id=${args.child.idMm}, t=${args.child.thicknessMm})`)
  const t0 = performance.now()
  const outcome = await bakeFuseParametric(spec, { timeoutMs: args.timeoutMs })
  const dtMs = performance.now() - t0

  if (!outcome.ok) {
    console.error(`[bake-smoke/fuseParam] FAILED (${outcome.phase}): ${outcome.message}`)
    return
  }

  const d = outcome.diagnostics
  console.log(`[bake-smoke/fuseParam] done in ${dtMs.toFixed(0)}ms — ` +
    `import=${d.importMs.toFixed(0)} fuse=${(d.fuseMs ?? 0).toFixed(0)} ` +
    `fillet=${(d.filletMs ?? 0).toFixed(0)} mesh=${d.meshMs.toFixed(0)}ms ` +
    `tris=${d.triangleCount}${d.filletFellBackToFuseOnly ? ' (fillet failed → fuse-only)' : ''}`)

  const group = buildBakedMesh(outcome.mesh, outcome.edges, {
    material: new THREE.MeshStandardMaterial({ color: 0x8fb3ff, roughness: 0.4, metalness: 0.3 }),
    includeEdges: true,
    edgeMaterial: new THREE.LineBasicMaterial({ color: 0x1c2436 }),
  })
  group.name = `fused-param-${args.parent.presetId}`
  group.position.set(offsetX, 0, 0)
  smokeGroup.add(group)
  const fusedBox = new THREE.Box3().setFromObject(group)
  const fusedSize = new THREE.Vector3(); fusedBox.getSize(fusedSize)

  // Also render an UNFUSED counterpart (skipFuse=true) right next to it so
  // the fillet improvement is visible at a glance. Same geometry, but as
  // two separate solids placed flush.
  if (!args.skipFuse) {
    const unfusedSpec: FuseParamSpec = { ...spec, skipFuse: true, filletRadiusMm: 0 }
    const unfusedOutcome = await bakeFuseParametric(unfusedSpec, { timeoutMs: args.timeoutMs })
    if (unfusedOutcome.ok) {
      const unfusedGroup = buildBakedMesh(unfusedOutcome.mesh, unfusedOutcome.edges, {
        material: new THREE.MeshStandardMaterial({ color: 0xcccccc, roughness: 0.4, metalness: 0.3 }),
        includeEdges: true,
        edgeMaterial: new THREE.LineBasicMaterial({ color: 0x444444 }),
      })
      unfusedGroup.name = `unfused-param-${args.parent.presetId}`
      unfusedGroup.position.set(offsetX + fusedSize.x + 0.08, 0, 0)
      smokeGroup.add(unfusedGroup)
      console.log(`[bake-smoke/fuseParam] rendered unfused counterpart for comparison (gray, on the right)`)
    }
  }
}

export function clearBakeSmoke(deps: SmokeTestDeps): void {
  const g = deps.scene.getObjectByName(SMOKE_GROUP_NAME)
  if (g) {
    deps.scene.remove(g)
    g.traverse(o => {
      const m = o as THREE.Mesh
      if (m.isMesh) {
        m.geometry?.dispose()
        const mat = m.material
        if (Array.isArray(mat)) for (const mm of mat) mm.dispose()
        else mat?.dispose()
      }
    })
  }
}
