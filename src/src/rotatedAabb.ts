// Post-rotation AABB half-extents.
//
// The placement compiler currently treats `bounds.half` as fixed regardless
// of `attach_rpy`. For axis-aligned rotations (90° multiples) this happens
// to be correct because the half-extent triple is just permuted; for any
// other rotation it is silently wrong (the bounding box grows).
//
// Phase 2 routes every face/connector calculation
// in placementCompiler/{face,mate}.ts through this helper before reading
// `child.bounds.half`. RPY interpretation matches `rotationIO.rpyToQuat`
// (URDF X-Y-Z fixed axes).

import * as THREE from 'three'

import { rpyToQuat } from './rotationIO.ts'

export type Vec3 = readonly [number, number, number]
export type RpyXyz = readonly [number, number, number]

/**
 * Tight AABB half-extents of a box with half-extents `half` after rotation
 * by URDF rpy. Returned vector is itself axis-aligned in the parent frame.
 *
 * Implementation note: |R| applied to the absolute half-extent vector gives
 * the projection of the rotated box onto each parent axis. This is the
 * standard Erickson trick (Real-Time Collision Detection §4.2.6).
 */
export function rotatedHalfExtents(half: Vec3, rpy: RpyXyz): [number, number, number] {
  const q = rpyToQuat([rpy[0], rpy[1], rpy[2]])
  const m = new THREE.Matrix4().makeRotationFromQuaternion(q).elements
  // Three.js stores column-major. m[ row + col*4 ].
  const absR = [
    [Math.abs(m[0]), Math.abs(m[4]), Math.abs(m[8])],
    [Math.abs(m[1]), Math.abs(m[5]), Math.abs(m[9])],
    [Math.abs(m[2]), Math.abs(m[6]), Math.abs(m[10])],
  ]
  const hx = absR[0][0] * half[0] + absR[0][1] * half[1] + absR[0][2] * half[2]
  const hy = absR[1][0] * half[0] + absR[1][1] * half[1] + absR[1][2] * half[2]
  const hz = absR[2][0] * half[0] + absR[2][1] * half[1] + absR[2][2] * half[2]
  return [hx, hy, hz]
}
