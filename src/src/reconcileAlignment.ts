// Render-time alignment pass (Option C from docs/ENGINE_ARCHITECTURE.md).
//
// Kept in its own module so node --experimental-strip-types can load it
// without pulling in the DOM/tauri/richVisuals graph that urdfAssembly.ts
// drags along. This mirrors the urdfGraphEquivalence / topologyValidation
// split: pure modules tested from Node, impure glue in urdfAssembly.ts.
//
// The pass keeps the placement engine pure. After URDF parse + rich-visuals
// load, walk each parent→child pair; if the rendered mesh AABB of the parent
// extends past where the placement engine assumed, shift the joint pivot so
// the child's contact face meets the parent's attach face.
//
// Works at the SCENE level. Measures whatever actually rendered — robust to
// preset-vs-GLB axis swaps, raised lips, asymmetric meshes, baked rotation
// overrides. Shifts cascade to grandchildren automatically (Three.js scene
// graph is hierarchical), so a single top-down pass is sufficient.

import * as THREE from 'three'
import type { AssemblyComponent, AssemblyGraph } from './urdfGraphEquivalence.ts'

export interface ReconcileShift {
  linkName: string
  dxMm: number
  dyMm: number
  dzMm: number
}

export interface ReconcileResult {
  adjustedCount: number
  residualMaxMm: number
  shifts: ReconcileShift[]
}

export interface ReconcileInputs {
  /** The last resolved assembly graph — tells the pass which attach_face was intended. */
  graph: AssemblyGraph
  /** Map of link name → Three.js Group (as produced by parseURDFToScene). */
  linkGroups: Map<string, THREE.Group>
  /** Map of joint name → pivot group. Each childLinkGroup.parent must appear here. */
  joints: Map<string, { group: THREE.Group; axis: THREE.Vector3; type: string }>
  /** Ignore deltas below this (meters). Default 0.0005 m = 0.5 mm. */
  epsMeters?: number
  /** Cap per-link shift (meters). Any larger delta is logged and skipped — protects
   *  against absurd shifts from measurement failures. Default 0.5 m (500 mm). */
  maxShiftMeters?: number
  /** Suppress per-link console.log (still logs skips/warnings). */
  silent?: boolean
}

const FACE_AXIS_MAP: Record<string, { axis: 0 | 1 | 2; sign: 1 | -1 }> = {
  top:    { axis: 2, sign:  1 },
  bottom: { axis: 2, sign: -1 },
  front:  { axis: 0, sign:  1 },
  back:   { axis: 0, sign: -1 },
  right:  { axis: 1, sign:  1 },
  left:   { axis: 1, sign: -1 },
}

const OPPOSITE_FACE: Record<string, string> = {
  top: 'bottom', bottom: 'top',
  front: 'back', back: 'front',
  left: 'right', right: 'left',
}

/** Unit normal for the named face, in the owning AABB's frame. */
function faceNormal(face: string): THREE.Vector3 | null {
  const fa = FACE_AXIS_MAP[face]
  if (!fa) return null
  return new THREE.Vector3(
    fa.axis === 0 ? fa.sign : 0,
    fa.axis === 1 ? fa.sign : 0,
    fa.axis === 2 ? fa.sign : 0,
  )
}

/** AABB of a link's own geometry in its link-local frame, excluding any
 *  descendant pivot subtrees (their meshes belong to other links). */
function linkLocalAABBExcludingPivots(
  linkGroup: THREE.Group,
  pivotGroups: Set<THREE.Object3D>,
): THREE.Box3 | null {
  linkGroup.updateMatrixWorld(true)
  const linkWorldInv = linkGroup.matrixWorld.clone().invert()
  const box = new THREE.Box3()
  let hasGeom = false
  const tmpBox = new THREE.Box3()
  const tmpMat = new THREE.Matrix4()
  const walk = (obj: THREE.Object3D): void => {
    if (obj !== linkGroup && pivotGroups.has(obj)) return
    const mesh = obj as THREE.Mesh
    if (mesh.isMesh && mesh.geometry) {
      if (!mesh.geometry.boundingBox) mesh.geometry.computeBoundingBox()
      const bb = mesh.geometry.boundingBox
      if (bb && !bb.isEmpty()) {
        mesh.updateMatrixWorld(true)
        tmpMat.multiplyMatrices(linkWorldInv, mesh.matrixWorld)
        tmpBox.copy(bb).applyMatrix4(tmpMat)
        box.union(tmpBox)
        hasGeom = true
      }
    }
    for (const child of obj.children) walk(child)
  }
  walk(linkGroup)
  return hasGeom ? box : null
}

/** Pure render-time alignment pass. Mutates pivot positions in-place inside
 *  the provided scene. Idempotent: re-running with the same scene state is a
 *  no-op (the EPS guard stops already-aligned pairs from re-shifting). */
export function reconcileNodePlacement(inputs: ReconcileInputs): ReconcileResult {
  const eps = inputs.epsMeters ?? 0.0005
  const maxShift = inputs.maxShiftMeters ?? 0.5
  const silent = inputs.silent === true
  const shifts: ReconcileShift[] = []
  let adjustedCount = 0
  let residualMaxMm = 0

  const pivotGroups = new Set<THREE.Object3D>()
  for (const [, j] of inputs.joints) pivotGroups.add(j.group)

  const childrenByParent = new Map<string, AssemblyComponent[]>()
  for (const c of inputs.graph.components) {
    if (!c.attach_to) continue
    const list = childrenByParent.get(c.attach_to)
    if (list) list.push(c)
    else childrenByParent.set(c.attach_to, [c])
  }

  // BFS from the root. Parents are always processed before their children so
  // grandchildren read updated parent world matrices.
  const queue: string[] = [inputs.graph.base_link]
  const seen = new Set<string>([inputs.graph.base_link])
  while (queue.length > 0) {
    const parentName = queue.shift()!
    const children = childrenByParent.get(parentName)
    if (!children) continue

    const parentGroup = inputs.linkGroups.get(parentName)
    const parentAABB = parentGroup
      ? linkLocalAABBExcludingPivots(parentGroup, pivotGroups)
      : null

    for (const child of children) {
      if (!seen.has(child.link_name)) {
        seen.add(child.link_name)
        queue.push(child.link_name)
      }

      const childGroup = inputs.linkGroups.get(child.link_name)
      if (!childGroup || !parentGroup || !parentAABB) continue

      const pivot = childGroup.parent as THREE.Group | null
      if (!pivot || !pivotGroups.has(pivot)) continue

      const face = child.attach_face
      if (!face || !FACE_AXIS_MAP[face]) continue
      const oppositeFace = OPPOSITE_FACE[face]
      if (!oppositeFace) continue

      const childAABB = linkLocalAABBExcludingPivots(childGroup, pivotGroups)
      if (!childAABB) continue

      const parentFA = FACE_AXIS_MAP[face]
      const normal = faceNormal(face)
      if (!normal) continue

      // Build the contact points using AABB.min/max along the face axis only;
      // the other two components can be anything — we project onto the normal
      // and discard them anyway.
      const parentFacePoint = new THREE.Vector3()
      const pSrc = parentFA.sign > 0 ? parentAABB.max : parentAABB.min
      if (parentFA.axis === 0) parentFacePoint.x = pSrc.x
      else if (parentFA.axis === 1) parentFacePoint.y = pSrc.y
      else parentFacePoint.z = pSrc.z

      const childFA = FACE_AXIS_MAP[oppositeFace]
      const childContactLocal = new THREE.Vector3()
      const cSrc = childFA.sign > 0 ? childAABB.max : childAABB.min
      if (childFA.axis === 0) childContactLocal.x = cSrc.x
      else if (childFA.axis === 1) childContactLocal.y = cSrc.y
      else childContactLocal.z = cSrc.z

      // Project the gap onto the parent's face normal so the placement engine's
      // tangential choices (4 legs at corners, 4 electronics at corners)
      // survive the pass. We only correct depth along the face normal —
      // that's what eliminates submerged / floating children.
      const rotatedChildContact = childContactLocal.clone().applyQuaternion(pivot.quaternion)
      const currentOnNormal = pivot.position.clone().add(rotatedChildContact).dot(normal)
      const targetOnNormal = parentFacePoint.dot(normal)
      const deltaScalar = targetOnNormal - currentOnNormal
      const delta = normal.clone().multiplyScalar(deltaScalar)

      const deltaLen = Math.abs(deltaScalar)
      if (deltaLen <= eps) continue

      if (deltaLen > maxShift) {
        console.warn(
          `[reconcile] skip ${child.link_name}: delta ${(deltaLen * 1000).toFixed(1)}mm ` +
          `exceeds ${(maxShift * 1000).toFixed(0)}mm safety cap — likely an AABB measurement failure`,
        )
        continue
      }

      pivot.position.add(delta)
      pivot.updateMatrixWorld(true)

      const dxMm = delta.x * 1000
      const dyMm = delta.y * 1000
      const dzMm = delta.z * 1000
      shifts.push({ linkName: child.link_name, dxMm, dyMm, dzMm })
      adjustedCount++
      residualMaxMm = Math.max(residualMaxMm, deltaLen * 1000)
      if (!silent) {
        console.log(
          `[reconcile] ${child.link_name} shifted by ` +
          `(${dxMm.toFixed(2)}, ${dyMm.toFixed(2)}, ${dzMm.toFixed(2)})mm ` +
          `(face=${face}, parent=${parentName})`,
        )
      }
    }
  }

  if (!silent) {
    console.log(
      `[reconcile] Done: ${adjustedCount} link(s) adjusted, ` +
      `max shift = ${residualMaxMm.toFixed(2)}mm`,
    )
  }
  return { adjustedCount, residualMaxMm, shifts }
}
