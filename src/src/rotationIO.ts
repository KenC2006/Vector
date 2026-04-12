import * as THREE from 'three'

// URDF rpy convention: roll-pitch-yaw, fixed-axis XYZ = extrinsic ZYX.
// Three.js Euler order 'ZYX' matches this when Euler(roll, pitch, yaw, 'ZYX').
// All quaternion↔RPY conversions in this app must go through these two helpers
// so the Euler order is enforced in exactly one place.

/** Convert a quaternion to a URDF RPY triple [roll, pitch, yaw] (extrinsic ZYX). */
export function quatToRpy(q: THREE.Quaternion): [number, number, number] {
  const e = new THREE.Euler().setFromQuaternion(q, 'ZYX')
  return [e.x, e.y, e.z]
}

/** Convert a URDF RPY triple [roll, pitch, yaw] to a quaternion (extrinsic ZYX). */
export function rpyToQuat(rpy: [number, number, number] | number[]): THREE.Quaternion {
  const e = new THREE.Euler(rpy[0] ?? 0, rpy[1] ?? 0, rpy[2] ?? 0, 'ZYX')
  return new THREE.Quaternion().setFromEuler(e)
}
