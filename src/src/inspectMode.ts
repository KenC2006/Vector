import * as THREE from 'three'
import type { OrbitControls } from 'three/addons/controls/OrbitControls.js'

const DIM_OPACITY = 0.18

export interface CameraFocusTween {
  startMs: number
  durationMs: number
  fromPos: THREE.Vector3
  toPos: THREE.Vector3
  fromTarget: THREE.Vector3
  toTarget: THREE.Vector3
}

/** Union bounding box in world space for all meshes under `root` tagged with `urdfLinkName === linkName`. */
export function computeLinkWorldBox(
  root: THREE.Object3D,
  linkName: string,
): THREE.Box3 | null {
  const box = new THREE.Box3()
  let any = false
  root.updateMatrixWorld(true)
  root.traverse(child => {
    if (!(child instanceof THREE.Mesh) || !child.geometry) return
    const n = (child.userData as Record<string, unknown>).urdfLinkName
    if (n !== linkName) return
    if (!child.geometry.boundingBox) child.geometry.computeBoundingBox()
    const bb = child.geometry.boundingBox
    if (!bb) return
    child.updateWorldMatrix(true, false)
    const temp = bb.clone().applyMatrix4(child.matrixWorld)
    box.union(temp)
    any = true
  })
  return any ? box : null
}

export function cameraPoseForBox(
  box: THREE.Box3,
  camera: THREE.PerspectiveCamera,
  controls: OrbitControls,
): { position: THREE.Vector3; target: THREE.Vector3 } {
  const center = box.getCenter(new THREE.Vector3())
  const size = box.getSize(new THREE.Vector3())
  const maxDim = Math.max(size.x, size.y, size.z, 0.05)
  const fov = camera.fov * (Math.PI / 180)
  const dist = Math.min(
    Math.max(maxDim / (2 * Math.tan(fov / 2)) * 1.35, 0.15),
    controls.maxDistance * 0.95,
  )
  const offset = new THREE.Vector3(dist * 0.65, dist * 0.45, dist * 0.65)
  return {
    target: center.clone(),
    position: center.clone().add(offset),
  }
}

export function stepCameraFocusTween(
  tween: CameraFocusTween,
  camera: THREE.PerspectiveCamera,
  controls: OrbitControls,
  nowMs: number,
): CameraFocusTween | null {
  const t = Math.min(1, (nowMs - tween.startMs) / tween.durationMs)
  const k = 1 - (1 - t) ** 3
  camera.position.lerpVectors(tween.fromPos, tween.toPos, k)
  controls.target.lerpVectors(tween.fromTarget, tween.toTarget, k)
  return t >= 1 ? null : tween
}

function disposeMaterial(m: THREE.Material) {
  m.dispose()
}

/** Restore original materials on all meshes under `robot` that were cloned for inspect dimming. */
export function restoreInspectMaterials(robot: THREE.Object3D) {
  robot.traverse(obj => {
    if (!(obj instanceof THREE.Mesh)) return
    const backup = (obj.userData as Record<string, unknown>).inspectMatBackup as
      | THREE.Material
      | THREE.Material[]
      | undefined
    if (backup === undefined) return
    const cur = obj.material
    if (Array.isArray(cur)) cur.forEach(disposeMaterial)
    else disposeMaterial(cur as THREE.Material)
    obj.material = backup
    delete (obj.userData as Record<string, unknown>).inspectMatBackup
  })
}

function ensureClonedMaterials(mesh: THREE.Mesh) {
  if ((mesh.userData as Record<string, unknown>).inspectMatBackup !== undefined) return
  const orig = mesh.material
  if (Array.isArray(orig)) {
    ;(mesh.userData as Record<string, unknown>).inspectMatBackup = orig.slice()
    mesh.material = orig.map(m => m.clone())
  } else {
    ;(mesh.userData as Record<string, unknown>).inspectMatBackup = orig
    mesh.material = orig.clone()
  }
}

function setMaterialsOpacity(mesh: THREE.Mesh, opacity: number, forceTransparent: boolean) {
  const mats = Array.isArray(mesh.material) ? mesh.material : [mesh.material]
  for (const m of mats) {
    if ('opacity' in m && 'transparent' in m) {
      const mm = m as THREE.MeshStandardMaterial | THREE.MeshBasicMaterial
      mm.transparent = forceTransparent || opacity < 0.999
      mm.opacity = opacity
      mm.depthWrite = opacity > 0.5
    }
  }
}

/**
 * Dim every URDF mesh except `focusedLink`. Call `restoreInspectMaterials` before changing robots or clearing focus.
 */
export function applyInspectDimming(robot: THREE.Object3D, focusedLink: string) {
  robot.updateMatrixWorld(true)
  robot.traverse(obj => {
    if (!(obj instanceof THREE.Mesh) || !obj.geometry) return
    const link = (obj.userData as Record<string, unknown>).urdfLinkName
    if (typeof link !== 'string' || !link) return
    ensureClonedMaterials(obj)
    const dim = link !== focusedLink
    setMaterialsOpacity(obj, dim ? DIM_OPACITY : 1, dim || focusedLink !== '')
  })
}
