import * as THREE from 'three'
import type { GeneratorDims } from './generators'
import { getComponentColor, getMaterial, getTintedMaterial } from './materials'
import { getRotationOverride, getShaftOverlay } from './meshOverrides'

export interface PreparedMeshVisual {
  group: THREE.Group
  renderedBodySize: THREE.Vector3 | null
  shaftOverlayMesh?: THREE.Mesh
}

export function prepareMeshVisualGroup(
  meshGroup: THREE.Group,
  dims: GeneratorDims,
  compId: string,
  opts: {
    linkName?: string
    materialCache?: Map<string, THREE.MeshStandardMaterial>
    includeShaftOverlay?: boolean
    castShadow?: boolean
    receiveShadow?: boolean
  } = {},
): PreparedMeshVisual {
  const castShadow = opts.castShadow ?? true
  const receiveShadow = opts.receiveShadow ?? true
  applyMeshMaterials(meshGroup, compId, opts.materialCache, opts.linkName, castShadow, receiveShadow)

  const meshBox = new THREE.Box3().setFromObject(meshGroup)
  const meshSize = new THREE.Vector3()
  meshBox.getSize(meshSize)
  const maxMeshDim = Math.max(meshSize.x, meshSize.y, meshSize.z)
  const maxExpectedDim = Math.max(dims.x, dims.y, dims.z)
  let renderedBodySize: THREE.Vector3 | null = null

  if (maxMeshDim > 0.0001) {
    if (maxMeshDim > maxExpectedDim * 10) {
      meshGroup.scale.setScalar(0.001)
    }

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

    const perAxisBlacklist = ['gripper', 'effector', 'claw', 'suction']
    const skipPerAxis = perAxisBlacklist.some(k => compId.includes(k))
    const shaftOverlay = getShaftOverlay(compId)
    const shaftLenM = shaftOverlay ? shaftOverlay.shaft_length_mm / 1000 : 0
    const targetZ = shaftOverlay ? Math.max(0.001, dims.z - shaftLenM) : dims.z
    if (!skipPerAxis) {
      meshBox.setFromObject(meshGroup)
      meshBox.getSize(meshSize)
      if (meshSize.x > 0.0001 && meshSize.y > 0.0001 && meshSize.z > 0.0001) {
        meshGroup.scale.x *= dims.x / meshSize.x
        meshGroup.scale.y *= dims.y / meshSize.y
        meshGroup.scale.z *= targetZ / meshSize.z
      }
    }

    meshBox.setFromObject(meshGroup)
    const center = new THREE.Vector3()
    meshBox.getCenter(center)
    meshGroup.position.sub(center)

    if (shaftOverlay) {
      meshGroup.position.z -= shaftLenM / 2
    }

    const finalBox = new THREE.Box3().setFromObject(meshGroup)
    const finalSize = new THREE.Vector3()
    finalBox.getSize(finalSize)
    if (finalSize.x > 0.001 || finalSize.y > 0.001 || finalSize.z > 0.001) {
      renderedBodySize = finalSize
    }
  }

  const shaftOverlayMesh = opts.includeShaftOverlay ? makeShaftOverlayMesh(compId, dims, opts.linkName, castShadow, receiveShadow) : undefined
  return { group: meshGroup, renderedBodySize, shaftOverlayMesh }
}

function applyMeshMaterials(
  meshGroup: THREE.Group,
  compId: string,
  materialCache: Map<string, THREE.MeshStandardMaterial> | undefined,
  linkName: string | undefined,
  castShadow: boolean,
  receiveShadow: boolean,
): void {
  const compColor = getComponentColor(compId)
  const catMat = getTintedMaterial(compColor.material, ...compColor.tint, compColor.strength ?? 0.4)
  const tintColor = new THREE.Color(compColor.tint[0], compColor.tint[1], compColor.tint[2])
  const tintStrength = compColor.strength ?? 0.4
  const forceReplaceMaterials = new Set(['rubber_black'])
  const forceReplace = forceReplaceMaterials.has(compColor.material)

  meshGroup.traverse(child => {
    if (!(child instanceof THREE.Mesh)) return
    const mat = child.material as THREE.MeshStandardMaterial
    const isDefaultGray = mat?.color &&
      Math.abs(mat.color.r - 0.533) < 0.05 &&
      Math.abs(mat.color.g - 0.533) < 0.05 &&
      Math.abs(mat.color.b - 0.533) < 0.05
    if (forceReplace || isDefaultGray) {
      child.material = catMat
    } else if (mat?.color) {
      const cacheKey = `${compId}_${mat.uuid}`
      let tinted = materialCache?.get(cacheKey)
      if (!tinted) {
        tinted = mat.clone()
        tinted.color.lerp(tintColor, tintStrength)
        materialCache?.set(cacheKey, tinted)
      }
      child.material = tinted
    }
    child.castShadow = castShadow
    child.receiveShadow = receiveShadow
    if (linkName) (child.userData as Record<string, unknown>).urdfLinkName = linkName
  })
}

function makeShaftOverlayMesh(
  compId: string,
  dims: GeneratorDims,
  linkName: string | undefined,
  castShadow: boolean,
  receiveShadow: boolean,
): THREE.Mesh | undefined {
  const overlay = getShaftOverlay(compId)
  if (!overlay) return undefined
  const shaftLength = overlay.shaft_length_mm / 1000
  const shaftRadius = overlay.shaft_radius_mm / 1000
  const shaftGeo = new THREE.CylinderGeometry(shaftRadius, shaftRadius, shaftLength, 24)
  const shaftMesh = new THREE.Mesh(shaftGeo, getMaterial('brushed_steel'))
  shaftMesh.rotation.x = Math.PI / 2
  shaftMesh.position.z = dims.z / 2 - shaftLength / 2
  shaftMesh.castShadow = castShadow
  shaftMesh.receiveShadow = receiveShadow
  if (linkName) (shaftMesh.userData as Record<string, unknown>).urdfLinkName = linkName
  return shaftMesh
}
