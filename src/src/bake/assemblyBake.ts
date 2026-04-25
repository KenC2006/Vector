// Assembly-level bake orchestration.
//
// Given the current AssemblyGraph (from urdfAssemblyApi.getLastAssemblyGraph())
// plus the Three.js scene's link groups, this module:
//
//   1. Clusters components by fixed joints (each revolute/prismatic joint
//      starts a new cluster). Revolute joints articulate — we can't fuse
//      across them or the robot stops moving.
//   2. Per cluster, reads each member's world transform from the scene and
//      converts it to cluster-root-local mm + rpy.
//   3. Applies per-preset bake-source overrides (e.g. swap coupler STEP for
//      a parametric disc — see presetBakeOverrides.ts).
//   4. Sends one `bakeCluster` job per cluster to the worker.
//   5. Caches results by cluster hash (content-addressed).
//   6. Swaps per-preset meshes for baked meshes in the scene, or falls back
//      to per-preset render on failure.
//
// Progress is reported via an optional callback so the caller (UI) can show
// a status string. Failure of one cluster doesn't abort the rest.
//
// Not wired into the accept flow yet — Phase 5 does the user-facing wire-up.
// For now `window.__bakeScene()` invokes this manually.

import * as THREE from 'three'
import type { AssemblyGraph, AssemblyComponent } from '../urdfGraphEquivalence.ts'
import { bakeClusterRequest, type BakeOutcome } from './bakeClient.ts'
import { buildBakedMesh } from './bakedThree.ts'
import { getStepFallbackUrl, hasMeshOverride, getRotationOverride } from '../richVisuals/meshOverrides.ts'
import { getBakeSourceOverride } from './presetBakeOverrides.ts'
import { resolveFilletPolicy } from './filletConfig.ts'
import { getComponentColor, getTintedMaterial } from '../richVisuals/materials.ts'
import type { BakeClusterSpec, ClusterPart, ClusterJoint } from './types.ts'

// ── Connector lookup (shared with smokeTest.ts pattern) ────────────────────

interface PresetConnectorLite {
  id: string
  origin_xyz_mm: [number, number, number]
  axis_xyz: [number, number, number]
  type?: string
  diameter_mm?: number
}

interface PresetLite {
  connectors: Map<string, PresetConnectorLite>
  bboxMm: [number, number, number] | null
}

let _presetCache: Map<string, PresetLite> | null = null
async function loadPresetLites(): Promise<Map<string, PresetLite>> {
  if (_presetCache) return _presetCache
  const res = await fetch('/generic_presets.json')
  if (!res.ok) throw new Error(`presets HTTP ${res.status}`)
  const json = await res.json() as {
    categories: Record<string, {
      components: {
        id: string
        connectors?: PresetConnectorLite[]
        physical?: { bounding_box_mm?: [number, number, number] }
      }[]
    }>
  }
  const map = new Map<string, PresetLite>()
  for (const cat of Object.values(json.categories)) {
    for (const comp of cat.components) {
      const inner = new Map<string, PresetConnectorLite>()
      for (const c of comp.connectors ?? []) inner.set(c.id, c)
      map.set(comp.id, {
        connectors: inner,
        bboxMm: comp.physical?.bounding_box_mm ?? null,
      })
    }
  }
  _presetCache = map
  return map
}

// ── Cluster planning ───────────────────────────────────────────────────────

export interface ClusterPlan {
  rootLinkName: string
  members: AssemblyComponent[]       // index 0 = root, then descendants
  /** Which non-root member connects to which index in `members`. Parallel to
   *  members[1..], so attachIndex[i] points to an earlier entry. */
  attachIndex: number[]
}

/** Walk the assembly tree, grouping links into clusters separated by
 *  non-fixed joints. Each cluster's root is either the URDF base or the
 *  child of a non-fixed joint. Fixed-joint subtrees merge into the parent's
 *  cluster.
 *
 *  Isolated components (no attach_to, not base) are skipped — they'd need
 *  their own cluster of size 1, which doesn't benefit from fuse anyway.
 */
export function planClusters(graph: AssemblyGraph): ClusterPlan[] {
  const byLink = new Map<string, AssemblyComponent>()
  for (const c of graph.components) byLink.set(c.link_name, c)

  // Build children map
  const childrenOf = new Map<string, AssemblyComponent[]>()
  for (const c of graph.components) {
    if (!c.attach_to) continue
    const arr = childrenOf.get(c.attach_to)
    if (arr) arr.push(c)
    else childrenOf.set(c.attach_to, [c])
  }

  const clusters: ClusterPlan[] = []
  const visited = new Set<string>()

  // A link is a cluster root if (a) it's the graph's base_link, OR (b) its
  // parent joint is non-fixed. For this MVP we only bake components with
  // STEP-backed meshes; a synthetic base_link (no preset) is a legal root
  // only if the cluster has >=2 real parts.
  const isFixed = (c: AssemblyComponent): boolean => (c.joint_type ?? 'fixed') === 'fixed'

  const walkClusterFrom = (rootName: string): ClusterPlan | null => {
    const members: AssemblyComponent[] = []
    const attachIndex: number[] = []
    // BFS from root, stopping at non-fixed joints.
    const rootComp = byLink.get(rootName)
    const stack: { comp: AssemblyComponent; parentIdx: number }[] = []
    if (rootComp) {
      members.push(rootComp)
      visited.add(rootComp.link_name)
    }
    for (const child of childrenOf.get(rootName) ?? []) {
      if (!isFixed(child)) continue
      stack.push({ comp: child, parentIdx: rootComp ? 0 : -1 })
    }
    while (stack.length > 0) {
      const { comp, parentIdx } = stack.shift()!
      visited.add(comp.link_name)
      const memberIdx = members.length
      members.push(comp)
      attachIndex.push(parentIdx)
      for (const gchild of childrenOf.get(comp.link_name) ?? []) {
        if (!isFixed(gchild)) continue
        stack.push({ comp: gchild, parentIdx: memberIdx })
      }
    }
    // Filter: need at least 2 bakeable STEP parts to be useful.
    const bakeableCount = members.filter(m => hasMeshOverride(m.component_id) || getBakeSourceOverride(m.component_id)).length
    if (bakeableCount < 2) return null
    if (members.length === 0) return null
    return { rootLinkName: rootName, members, attachIndex }
  }

  // Start a cluster at base_link, plus at every non-fixed-child root.
  const baseCluster = walkClusterFrom(graph.base_link)
  if (baseCluster) clusters.push(baseCluster)

  for (const c of graph.components) {
    if (visited.has(c.link_name)) continue
    // Start of a non-fixed subtree (c's parent joint is revolute etc).
    if (c.attach_to === null) continue  // floating — skip
    if (isFixed(c)) {
      // Should be visited via its parent's cluster walk. If not, parent is
      // outside graph (orphan) — skip.
      visited.add(c.link_name)
      continue
    }
    const cl = walkClusterFrom(c.link_name)
    if (cl) clusters.push(cl)
    else visited.add(c.link_name)
  }

  return clusters
}

// ── Transforms from scene ─────────────────────────────────────────────────

/** Read each cluster member's world matrix from the scene, compute
 *  root-local pose in mm + radians. */
export function readClusterTransforms(
  cluster: ClusterPlan,
  linkGroups: Map<string, THREE.Group>,
): { translateMm: [number, number, number]; rotateRadXyz: [number, number, number] }[] | null {
  const rootGroup = linkGroups.get(cluster.rootLinkName)
  if (!rootGroup) return null
  rootGroup.updateMatrixWorld(true)
  const rootInv = rootGroup.matrixWorld.clone().invert()
  const rootWorldPos = new THREE.Vector3().setFromMatrixPosition(rootGroup.matrixWorld)
  console.log(`[bake/transforms] ${cluster.rootLinkName} world=(${(rootWorldPos.x*1000).toFixed(1)}, ${(rootWorldPos.y*1000).toFixed(1)}, ${(rootWorldPos.z*1000).toFixed(1)})mm`)

  const out: { translateMm: [number, number, number]; rotateRadXyz: [number, number, number] }[] = []
  for (let i = 0; i < cluster.members.length; i++) {
    const m = cluster.members[i]
    const g = linkGroups.get(m.link_name)
    if (!g) return null
    g.updateMatrixWorld(true)
    if (i === 0) {
      out.push({ translateMm: [0, 0, 0], rotateRadXyz: [0, 0, 0] })
      continue
    }
    const local = new THREE.Matrix4().multiplyMatrices(rootInv, g.matrixWorld)
    const pos = new THREE.Vector3()
    const quat = new THREE.Quaternion()
    const scale = new THREE.Vector3()
    local.decompose(pos, quat, scale)
    const euler = new THREE.Euler().setFromQuaternion(quat, 'XYZ')
    const memberWorld = new THREE.Vector3().setFromMatrixPosition(g.matrixWorld)
    console.log(`[bake/transforms]   member[${i}] ${m.link_name} world=(${(memberWorld.x*1000).toFixed(1)}, ${(memberWorld.y*1000).toFixed(1)}, ${(memberWorld.z*1000).toFixed(1)})mm rootLocal=(${(pos.x*1000).toFixed(1)}, ${(pos.y*1000).toFixed(1)}, ${(pos.z*1000).toFixed(1)})mm`)
    out.push({
      translateMm: [pos.x * 1000, pos.y * 1000, pos.z * 1000],
      rotateRadXyz: [euler.x, euler.y, euler.z],
    })
  }
  return out
}

// ── Build BakeClusterSpec ─────────────────────────────────────────────────

async function buildClusterSpec(
  cluster: ClusterPlan,
  transforms: { translateMm: [number, number, number]; rotateRadXyz: [number, number, number] }[],
): Promise<{ spec: BakeClusterSpec; skippedCount: number } | { error: string }> {
  const presetMap = await loadPresetLites()

  const parts: ClusterPart[] = []
  const joints: ClusterJoint[] = []
  let skipped = 0

  // Build parts[] by mapping each member to a ClusterPart, honoring overrides.
  for (let i = 0; i < cluster.members.length; i++) {
    const m = cluster.members[i]
    const t = transforms[i]
    const override = getBakeSourceOverride(m.component_id)
    if (override) {
      if (override.kind === 'disc') {
        parts.push({
          kind: 'disc',
          odMm: override.odMm, idMm: override.idMm, thicknessMm: override.thicknessMm,
          translateMm: t.translateMm, rotateRadXyz: t.rotateRadXyz,
        })
        continue
      }
      if (override.kind === 'extrusion') {
        const len = m.length_mm ?? 100
        parts.push({
          kind: 'extrusion',
          crossSectionMm: override.crossSectionMm,
          lengthMm: len,
          slotWidthMm: override.slotWidthMm,
          slotDepthMm: override.slotDepthMm,
          ridgeWidthMm: override.ridgeWidthMm,
          ridgeHeightMm: override.ridgeHeightMm,
          translateMm: t.translateMm, rotateRadXyz: t.rotateRadXyz,
        })
        continue
      }
      if (override.kind === 'box') {
        // Assemble the three dimensions from the override. For parametric
        // presets the variable axis is filled from the AssemblyComponent's
        // length_mm, and the other two come from crossSectionMm.
        let size: [number, number, number]
        if (override.sizeMm) {
          size = [...override.sizeMm]
        } else if (override.crossSectionMm && override.lengthAxis !== undefined) {
          const len = override.lengthField === 'length_mm' ? (m.length_mm ?? 100) : 100
          const cs = override.crossSectionMm
          const out: [number, number, number] = [cs[0], cs[1], cs[0]]
          // Put length on the designated axis; other two get crossSectionMm in order.
          const axisIdx = override.lengthAxis
          const others = [0, 1, 2].filter(i => i !== axisIdx)
          out[axisIdx] = len
          out[others[0]] = cs[0]
          out[others[1]] = cs[1]
          size = out
        } else {
          // Misconfigured override — fall through to STEP.
          const stepUrl = getStepFallbackUrl(m.component_id)
          if (!stepUrl) return { error: `parametric box override for ${m.component_id} missing size AND no STEP` }
          parts.push({
            kind: 'step', stepUrl,
            translateMm: t.translateMm, rotateRadXyz: t.rotateRadXyz,
            simplify: true, centerOnBbox: true,
          })
          continue
        }
        parts.push({
          kind: 'box',
          sizeMm: size,
          translateMm: t.translateMm, rotateRadXyz: t.rotateRadXyz,
        })
        continue
      }
    }
    const stepUrl = getStepFallbackUrl(m.component_id)
    if (!stepUrl) {
      // Member has no STEP source — can't bake it. Skip the whole cluster:
      // fusing a subset and leaving per-preset for the rest produces ugly
      // overlapping renders.
      return { error: `member ${m.link_name} (${m.component_id}) has no STEP source` }
    }
    const rotOverride = getRotationOverride(m.component_id)
    parts.push({
      kind: 'step', stepUrl,
      translateMm: t.translateMm, rotateRadXyz: t.rotateRadXyz,
      simplify: true,
      rotationOverrideRadXyz: rotOverride ?? undefined,
      centerOnBbox: true,
    })
  }

  // Build joints[]. One per non-root member.
  for (let j = 0; j < cluster.attachIndex.length; j++) {
    const childMemberIdx = j + 1
    const parentIdx = cluster.attachIndex[j]
    const child = cluster.members[childMemberIdx]
    const parent = parentIdx >= 0 ? cluster.members[parentIdx] : null
    const childT = transforms[childMemberIdx]

    // Resolve which connector on parent the child mates to. Prefer explicit
    // attach_connector, fall back to attach_face, then default "top".
    const parentConnId = child.attach_connector ?? child.attach_face ?? 'top'
    const parentPreset = parent ? presetMap.get(parent.component_id) ?? null : null
    const parentConn = parentPreset?.connectors.get(parentConnId)
    const childPreset = presetMap.get(child.component_id) ?? null

    // Mate plane: parent connector origin in root frame. Root is parts[0];
    // parent may be deeper in the cluster, so we need parent's own transform.
    const parentT = parentIdx >= 0 ? transforms[parentIdx] : { translateMm: [0,0,0] as [number,number,number], rotateRadXyz: [0,0,0] as [number,number,number] }
    let planeCenter: [number, number, number]
    let axis: [number, number, number] = [0, 0, 1]
    if (parentConn) {
      // Rotate connector origin by parent's rotation, then add parent's translation.
      const o = new THREE.Vector3(parentConn.origin_xyz_mm[0], parentConn.origin_xyz_mm[1], parentConn.origin_xyz_mm[2])
      const rotM = new THREE.Matrix4().makeRotationFromEuler(new THREE.Euler(parentT.rotateRadXyz[0], parentT.rotateRadXyz[1], parentT.rotateRadXyz[2], 'XYZ'))
      o.applyMatrix4(rotM)
      o.x += parentT.translateMm[0]
      o.y += parentT.translateMm[1]
      o.z += parentT.translateMm[2]
      planeCenter = [o.x, o.y, o.z]

      const a = new THREE.Vector3(parentConn.axis_xyz[0], parentConn.axis_xyz[1], parentConn.axis_xyz[2])
      a.applyMatrix4(rotM)
      axis = [a.x, a.y, a.z]
    } else {
      // Fallback: use child's translation as the mate plane center. Better
      // than nothing for default-connector links.
      planeCenter = [...childT.translateMm]
      // Axis heuristic: the direction from parent to child.
      const dx = childT.translateMm[0] - parentT.translateMm[0]
      const dy = childT.translateMm[1] - parentT.translateMm[1]
      const dz = childT.translateMm[2] - parentT.translateMm[2]
      const len = Math.hypot(dx, dy, dz) || 1
      axis = [dx / len, dy / len, dz / len]
    }

    // Resolve fillet policy by mate type, with per-preset-pair overrides.
    const policy = resolveFilletPolicy(
      child.mate_type,
      parent?.component_id ?? '',
      child.component_id,
    )

    // Lateral filter extent: half of the child's longest bbox perpendicular
    // to the mate axis, plus a small margin. Falls back to policy.lateralHalf
    // or a generous 40mm default when we can't measure. Tight enough to
    // exclude unrelated parent housing edges while covering the full mate
    // ring on typical disc/plate children.
    let lat = policy.lateralHalfMm ?? 40
    if (childPreset?.bboxMm) {
      const [bx, by, bz] = childPreset.bboxMm
      const nX = Math.abs(axis[0]), nY = Math.abs(axis[1]), nZ = Math.abs(axis[2])
      // Pick the two dimensions perpendicular to the dominant axis component.
      const perps: number[] = []
      if (nX < 0.99) perps.push(bx)
      if (nY < 0.99) perps.push(by)
      if (nZ < 0.99) perps.push(bz)
      if (perps.length > 0) lat = Math.max(...perps) / 2 + 3
    }

    joints.push({
      filletBoxHalfSideMm: lat,
      planeCenterMm: planeCenter,
      axisXyz: axis,
      filletRadiusMm: policy.skipFillet ? 0 : policy.radiusMm,
      slabMm: policy.slabMm,
      fuseOptimisation: 'sameFace',
      debugLabel: `${parent?.link_name ?? '<null>'}→${child.link_name}`,
    })
  }

  return { spec: { parts, joints }, skippedCount: skipped }
}

// ── Cache ──────────────────────────────────────────────────────────────────

export interface CachedBaked {
  spec: BakeClusterSpec
  outcome: BakeOutcome
}

const _clusterCache = new Map<string, CachedBaked>()

function hashClusterSpec(spec: BakeClusterSpec): string {
  // Canonical-ish JSON; order-stable since we iterate arrays in index order.
  // Round mm fields to avoid trivial float noise busting the cache on reparse.
  const rounded = (n: number) => Math.round(n * 1000) / 1000  // µm precision
  const parts = spec.parts.map(p => {
    const t = p.translateMm.map(rounded)
    const r = p.rotateRadXyz.map(n => Math.round(n * 1e6) / 1e6)
    if (p.kind === 'step') {
      const ovr = p.rotationOverrideRadXyz?.map(n => Math.round(n * 1e6) / 1e6) ?? null
      return ['step', p.stepUrl, t, r, ovr, p.centerOnBbox === false ? 0 : 1]
    }
    if (p.kind === 'box') {
      return ['box', p.sizeMm.map(rounded), t, r]
    }
    if (p.kind === 'extrusion') {
      return ['extrusion', p.crossSectionMm.map(rounded), p.lengthMm, p.slotWidthMm, p.slotDepthMm, p.ridgeWidthMm, p.ridgeHeightMm, t, r]
    }
    return ['disc', p.odMm, p.idMm, p.thicknessMm, t, r]
  })
  const joints = spec.joints.map(j => [
    j.filletBoxHalfSideMm,
    j.planeCenterMm.map(rounded),
    j.axisXyz.map(n => Math.round(n * 1e6) / 1e6),
    j.slabMm ?? 0,
    j.filletRadiusMm,
    j.fuseOptimisation ?? '',
  ])
  return JSON.stringify([parts, joints, spec.perPartMeshes ? 1 : 0])
}

export function clearBakeCache(): void {
  _clusterCache.clear()
}

// ── Public API ─────────────────────────────────────────────────────────────

export interface BakeSceneProgress {
  (phase: 'planning' | 'cluster-start' | 'cluster-done' | 'cluster-fail' | 'done', data?: {
    clusterIdx?: number
    totalClusters?: number
    clusterLabel?: string
    outcome?: BakeOutcome
  }): void
}

export interface BakeSceneResult {
  clusters: Array<{
    plan: ClusterPlan
    outcome: BakeOutcome
    bakedGroup?: THREE.Group
  }>
  // Links whose per-preset meshes were HIDDEN in favor of a baked mesh.
  hiddenLinks: Set<string>
}

export interface BakeSceneInputs {
  graph: AssemblyGraph
  linkGroups: Map<string, THREE.Group>
  onProgress?: BakeSceneProgress
  /** Disable swap-in — just plan and bake, don't mutate the scene. Useful
   *  for before/after debugging. */
  dryRun?: boolean
  /** Skip the cache (always re-bake). Useful for debugging. */
  bypassCache?: boolean
  /** Keep per-component colors by tessellating each cluster part separately
   *  (skip fuse). Default true — matches authored color palette. Disable to
   *  use the fuse path which produces a single merged solid in one color. */
  preserveColors?: boolean
}

/** Plan, bake, and swap meshes in the scene. Returns per-cluster outcomes. */
export async function bakeScene(inputs: BakeSceneInputs): Promise<BakeSceneResult> {
  const { graph, linkGroups, onProgress, dryRun, bypassCache } = inputs
  const preserveColors = inputs.preserveColors !== false
  const progress = onProgress ?? (() => {})

  // Guarantee fresh world matrices on every link. updateMatrixWorld only
  // walks DOWN, so calling it on a link's group assumes ancestors are
  // current. Find the scene root by walking up from any link, then update
  // the whole tree from there — one sweep refreshes every link's matrixWorld.
  const anyLink = linkGroups.values().next().value
  if (anyLink) {
    let sceneRoot: THREE.Object3D = anyLink
    while (sceneRoot.parent) sceneRoot = sceneRoot.parent
    sceneRoot.updateMatrixWorld(true)
  }

  // Remove any baked groups left over from a prior run. Without this, a
  // second __bakeScene() call after HMR or a topology change stacks the new
  // meshes on top of the old ones — producing the visible "duplicate legs"
  // disaster. We also un-hide prior per-preset meshes so transient renders
  // between unbake and rebake aren't empty.
  for (const [, g] of linkGroups) {
    const staleBaked: THREE.Object3D[] = []
    for (const c of g.children) if (c.name.startsWith('baked-cluster-')) staleBaked.push(c)
    for (const c of staleBaked) {
      g.remove(c)
      c.traverse(obj => {
        const m = obj as THREE.Mesh
        if (m.isMesh) {
          m.geometry?.dispose()
          const mat = m.material
          if (Array.isArray(mat)) for (const mm of mat) mm.dispose()
          else mat?.dispose()
        }
      })
    }
  }

  progress('planning')
  const plans = planClusters(graph)
  const result: BakeSceneResult = { clusters: [], hiddenLinks: new Set() }

  for (let i = 0; i < plans.length; i++) {
    const plan = plans[i]
    const label = `cluster[${i}] root=${plan.rootLinkName} n=${plan.members.length}`
    progress('cluster-start', { clusterIdx: i, totalClusters: plans.length, clusterLabel: label })

    const transforms = readClusterTransforms(plan, linkGroups)
    if (!transforms) {
      console.warn(`[bake/scene] ${label} missing linkGroups — skipping`)
      result.clusters.push({ plan, outcome: { ok: false, phase: 'transforms', message: 'missing linkGroups' } })
      progress('cluster-fail', { clusterIdx: i, totalClusters: plans.length, clusterLabel: label })
      continue
    }

    const built = await buildClusterSpec(plan, transforms)
    if ('error' in built) {
      console.warn(`[bake/scene] ${label} spec error: ${built.error}`)
      result.clusters.push({ plan, outcome: { ok: false, phase: 'spec', message: built.error } })
      progress('cluster-fail', { clusterIdx: i, totalClusters: plans.length, clusterLabel: label })
      continue
    }
    const spec: BakeClusterSpec = { ...built.spec, perPartMeshes: preserveColors }

    let outcome: BakeOutcome
    const cacheKey = hashClusterSpec(spec)
    if (!bypassCache) {
      const cached = _clusterCache.get(cacheKey)
      if (cached) {
        console.log(`[bake/scene] ${label} cache hit`)
        outcome = cached.outcome
      } else {
        outcome = await bakeClusterRequest(spec)
        _clusterCache.set(cacheKey, { spec, outcome })
      }
    } else {
      outcome = await bakeClusterRequest(spec)
      _clusterCache.set(cacheKey, { spec, outcome })
    }

    if (!outcome.ok) {
      console.warn(`[bake/scene] ${label} bake FAILED (${outcome.phase}): ${outcome.message}`)
      result.clusters.push({ plan, outcome })
      progress('cluster-fail', { clusterIdx: i, totalClusters: plans.length, clusterLabel: label, outcome })
      continue
    }

    console.log(`[bake/scene] ${label} OK tris=${outcome.diagnostics.triangleCount}`)

    if (!dryRun) {
      const rootGroup = linkGroups.get(plan.rootLinkName)
      if (rootGroup) {
        // Hide per-preset meshes on all cluster members.
        for (const m of plan.members) {
          const g = linkGroups.get(m.link_name)
          if (!g) continue
          hideRichVisualMeshes(g)
          result.hiddenLinks.add(m.link_name)
        }
        const bakedGroup = new THREE.Group()
        bakedGroup.name = `baked-cluster-${plan.rootLinkName}`

        if ('parts' in outcome) {
          // Multi-part path: each cluster member renders as its own Mesh
          // with its own preset-derived material. Preserves authored colors
          // at the cost of not fusing flush-tangent contacts (minor z-fight
          // possible where parts touch; the 0.3mm embed that fuse relied on
          // is NOT applied here since we're placing at link origins).
          for (const pm of outcome.parts) {
            const member = plan.members[pm.partIdx]
            const compId = member?.component_id ?? ''
            const color = getComponentColor(compId)
            const mat = getTintedMaterial(color.material, ...color.tint, color.strength ?? 0.4)
            const partGroup = buildBakedMesh(pm.mesh, pm.edges, {
              material: mat,
              includeEdges: true,
              edgeMaterial: new THREE.LineBasicMaterial({ color: 0x1c2436, transparent: true, opacity: 0.5 }),
            })
            partGroup.name = `baked-part-${member?.link_name ?? pm.partIdx}`
            bakedGroup.add(partGroup)
          }
        } else {
          // Single-solid path: one material for the whole fused cluster,
          // tinted from the cluster root's color.
          const rootCompId = plan.members[0]?.component_id ?? ''
          const rootColor = getComponentColor(rootCompId)
          const bakedMat = getTintedMaterial(rootColor.material, ...rootColor.tint, rootColor.strength ?? 0.4)
          const baked = buildBakedMesh(outcome.mesh, outcome.edges, {
            material: bakedMat,
            includeEdges: true,
            edgeMaterial: new THREE.LineBasicMaterial({ color: 0x1c2436, transparent: true, opacity: 0.55 }),
          })
          bakedGroup.add(baked)
        }
        rootGroup.add(bakedGroup)
        // Diagnostic: actual rendered world bbox of the cluster's baked
        // geometry. Compare against the per-preset's expected world position.
        rootGroup.updateMatrixWorld(true)
        const renderBox = new THREE.Box3().setFromObject(bakedGroup)
        const renderCenter = new THREE.Vector3(); renderBox.getCenter(renderCenter)
        const renderSize = new THREE.Vector3(); renderBox.getSize(renderSize)
        console.log(`[bake/render] ${plan.rootLinkName} bakedGroup world center=(${(renderCenter.x*1000).toFixed(1)}, ${(renderCenter.y*1000).toFixed(1)}, ${(renderCenter.z*1000).toFixed(1)})mm size=(${(renderSize.x*1000).toFixed(1)}, ${(renderSize.y*1000).toFixed(1)}, ${(renderSize.z*1000).toFixed(1)})mm`)
        // Per-part world bbox — pinpoints which member is misplaced.
        bakedGroup.children.forEach((partGrp, idx) => {
          const pBox = new THREE.Box3().setFromObject(partGrp)
          if (pBox.isEmpty()) return
          const pCenter = new THREE.Vector3(); pBox.getCenter(pCenter)
          const pSize = new THREE.Vector3(); pBox.getSize(pSize)
          const memberName = plan.members[idx]?.link_name ?? '?'
          console.log(`[bake/render]   part[${idx}] ${memberName} world center=(${(pCenter.x*1000).toFixed(1)}, ${(pCenter.y*1000).toFixed(1)}, ${(pCenter.z*1000).toFixed(1)})mm size=(${(pSize.x*1000).toFixed(1)}, ${(pSize.y*1000).toFixed(1)}, ${(pSize.z*1000).toFixed(1)})mm`)
        })
        result.clusters.push({ plan, outcome, bakedGroup })
      } else {
        result.clusters.push({ plan, outcome })
      }
    } else {
      result.clusters.push({ plan, outcome })
    }

    progress('cluster-done', { clusterIdx: i, totalClusters: plans.length, clusterLabel: label, outcome })
  }

  progress('done', { totalClusters: plans.length })
  return result
}

// ── Mesh swap ──────────────────────────────────────────────────────────────

/** Hide meshes that were added by the rich-visuals pipeline. Preserves
 *  helpers (axes, rings, attachment node overlays) and debug groups so the
 *  user can still see connectors, joint frames, etc. */
function hideRichVisualMeshes(linkGroup: THREE.Group): void {
  linkGroup.traverse(obj => {
    if (!(obj instanceof THREE.Mesh)) return
    // Helpers: boxgeometry 12mm (attachment nodes), torus (axis rings), etc.
    const geom = obj.geometry
    if (!geom) return
    const isHelper =
      obj.name === 'attachment_nodes' ||
      obj.name === 'attachment_node_rings' ||
      obj.name.startsWith('baked-cluster-') ||
      (geom as { parameters?: { width?: number } }).parameters?.width === 0.012 ||
      geom instanceof THREE.TorusGeometry
    if (!isHelper) obj.visible = false
  })
}

/** Undo a previous bakeScene — restore per-preset mesh visibility + remove
 *  baked groups. Used by `__bakeClear()` on the smoke entry point. */
export function clearBakedScene(inputs: {
  linkGroups: Map<string, THREE.Group>
  hiddenLinks: Set<string>
}): void {
  for (const linkName of inputs.hiddenLinks) {
    const g = inputs.linkGroups.get(linkName)
    if (!g) continue
    g.traverse(obj => {
      if (!(obj instanceof THREE.Mesh)) return
      obj.visible = true
    })
    // Remove baked groups attached to this link
    const toRemove: THREE.Object3D[] = []
    g.children.forEach(c => {
      if (c.name.startsWith('baked-cluster-')) toRemove.push(c)
    })
    for (const c of toRemove) {
      g.remove(c)
      c.traverse(obj => {
        const m = obj as THREE.Mesh
        if (m.isMesh) {
          m.geometry?.dispose()
          const mat = m.material
          if (Array.isArray(mat)) for (const mm of mat) mm.dispose()
          else mat?.dispose()
        }
      })
    }
  }
}
