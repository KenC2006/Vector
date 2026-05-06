import * as THREE from 'three'

// Single basis-swap seam lives in coordinates.ts. This module re-exports the
// helper for backward compatibility — existing imports of
// `urdfVectorToSceneLocal` keep working while the live owner is coordinates.
export { urdfVecToSceneVec as urdfVectorToSceneLocal } from './coordinates.ts'

export interface ConnectorSnapFrame {
  position: THREE.Vector3
  axis?: THREE.Vector3 | null
  quaternion?: THREE.Quaternion | null
}

function normalizedAxis(axis?: THREE.Vector3 | null): THREE.Vector3 | null {
  if (!axis) return null
  const len = axis.length()
  if (!(len > 1e-9)) return null
  return axis.clone().multiplyScalar(1 / len)
}

export function composeGhostWorldForConnectorSnap(
  sourceLocal: ConnectorSnapFrame,
  targetWorld: ConnectorSnapFrame,
): THREE.Matrix4 {
  const sourceAxis = normalizedAxis(sourceLocal.axis)
  const targetAxis = normalizedAxis(targetWorld.axis)
  const quat = new THREE.Quaternion()

  if (sourceAxis && targetAxis) {
    quat.setFromUnitVectors(sourceAxis, targetAxis.clone().negate())
  } else if (targetWorld.quaternion) {
    quat.copy(targetWorld.quaternion)
  }

  const sourceOffsetWorld = sourceLocal.position.clone().applyQuaternion(quat)
  const pos = targetWorld.position.clone().sub(sourceOffsetWorld)
  return new THREE.Matrix4().compose(pos, quat, new THREE.Vector3(1, 1, 1))
}
