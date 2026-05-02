import * as THREE from 'three'

// URDF rpy convention: roll-pitch-yaw about fixed X, then Y, then Z axes.
// Three.js Euler order 'XYZ' produces the same matrix convention used by URDF:
// R = Rz(yaw) * Ry(pitch) * Rx(roll).
// All quaternion/RPY conversions in this app must go through these two helpers
// so the Euler order is enforced in exactly one place.

/** Convert a quaternion to a URDF RPY triple [roll, pitch, yaw]. */
export function quatToRpy(q: THREE.Quaternion): [number, number, number] {
  const e = new THREE.Euler().setFromQuaternion(q, 'XYZ')
  return [e.x, e.y, e.z]
}

/** Convert a URDF RPY triple [roll, pitch, yaw] to a quaternion. */
export function rpyToQuat(rpy: [number, number, number] | number[]): THREE.Quaternion {
  const e = new THREE.Euler(rpy[0] ?? 0, rpy[1] ?? 0, rpy[2] ?? 0, 'XYZ')
  return new THREE.Quaternion().setFromEuler(e)
}
