// Single coordinate-system seam between the URDF/placement world (mm, Z-up,
// X-Y-Z fixed-axis RPY) and the Three.js scene (m, Y-up, quaternion).
//
// Every Z-up ↔ Y-up basis swap in this codebase must go through one of the
// two functions below. PLACEMENT_REWRITE_PLAN.md Phase 5 adds a CI grep gate
// that fails the build if any other file performs the swap inline.
//
// Convention chosen for this project:
//   urdf.x → scene.x
//   urdf.y → scene.-z      (URDF +Y points "left", scene +Z points "out of screen")
//   urdf.z → scene.+y
// Equivalently: rotate the URDF frame by -90° about world X to enter scene
// space. The inverse (+90° about X) takes scene back to URDF.
//
// Rotations expressed as URDF RPY are converted via the canonical rotationIO
// helpers; we then conjugate by the basis-swap quaternion. This keeps RPY
// composition correct under arbitrary tilts, not just axis-aligned spins.

import * as THREE from 'three'

import { quatToRpy, rpyToQuat } from './rotationIO.ts'

export type Vec3Mm = readonly [number, number, number]
export type RpyXyz = readonly [number, number, number]

const MM_PER_M = 1000

// Quaternion that takes a URDF-frame vector into scene-frame: -90° about X.
// Exported so non-vector callers (matrix-side basis conjugation in commitCarry,
// for instance) don't reach for `setFromAxisAngle` inline and re-introduce the
// inline basis-swap pattern the Phase 5 grep gate was meant to forbid.
export const URDF_TO_SCENE_Q: THREE.Quaternion = new THREE.Quaternion()
  .setFromAxisAngle(new THREE.Vector3(1, 0, 0), -Math.PI / 2)
export const SCENE_TO_URDF_Q: THREE.Quaternion = URDF_TO_SCENE_Q.clone().invert()

export function urdfFrameToScene(
  xyzMm: Vec3Mm,
  rpy: RpyXyz,
): { position: THREE.Vector3; quaternion: THREE.Quaternion } {
  // Translate: swap Z-up → Y-up, scale mm → m.
  const position = new THREE.Vector3(
    xyzMm[0] / MM_PER_M,
    xyzMm[2] / MM_PER_M,
    -xyzMm[1] / MM_PER_M,
  )
  // Rotate: conjugate the URDF rotation by the basis-swap quaternion so the
  // resulting scene rotation produces the same physical orientation.
  const qUrdf = rpyToQuat([rpy[0], rpy[1], rpy[2]])
  const quaternion = URDF_TO_SCENE_Q.clone()
    .multiply(qUrdf)
    .multiply(SCENE_TO_URDF_Q)
  return { position, quaternion }
}

/**
 * Pure axis swap for vectors that are already in the destination unit
 * (meters) — useful for carry-ghost local-frame math where bounds and port
 * origins are already in m. No mm scaling, no rotation conjugation. The
 * canonical convention from `urdfFrameToScene` is preserved:
 *   urdf.x → scene.x ;  urdf.y → scene.-z ;  urdf.z → scene.+y
 *
 * Accepts either a tuple or a Vector3 to match the legacy
 * `carrySnapMath.urdfVectorToSceneLocal` shape (which now re-exports this).
 */
export function urdfVecToSceneVec(
  v: readonly [number, number, number] | THREE.Vector3,
): THREE.Vector3 {
  const x = v instanceof THREE.Vector3 ? v.x : v[0]
  const y = v instanceof THREE.Vector3 ? v.y : v[1]
  const z = v instanceof THREE.Vector3 ? v.z : v[2]
  return new THREE.Vector3(x, z, -y)
}

export function sceneVecToUrdfVec(
  v: readonly [number, number, number] | THREE.Vector3,
): THREE.Vector3 {
  const x = v instanceof THREE.Vector3 ? v.x : v[0]
  const y = v instanceof THREE.Vector3 ? v.y : v[1]
  const z = v instanceof THREE.Vector3 ? v.z : v[2]
  // Inverse of urdfVecToSceneVec: scene.x → urdf.x ; scene.y → urdf.z ; scene.z → urdf.-y
  return new THREE.Vector3(x, -z, y)
}

export function sceneFrameToUrdf(
  position: THREE.Vector3,
  quaternion: THREE.Quaternion,
): { xyzMm: [number, number, number]; rpy: [number, number, number] } {
  const xyzMm: [number, number, number] = [
    position.x * MM_PER_M,
    -position.z * MM_PER_M,
    position.y * MM_PER_M,
  ]
  const qUrdf = SCENE_TO_URDF_Q.clone().multiply(quaternion).multiply(URDF_TO_SCENE_Q)
  const rpy = quatToRpy(qUrdf)
  return { xyzMm, rpy }
}
