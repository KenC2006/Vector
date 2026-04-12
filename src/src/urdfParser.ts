import * as THREE from 'three'
import { STLLoader } from 'three/addons/loaders/STLLoader.js'
import { OBJLoader } from 'three/addons/loaders/OBJLoader.js'
import { ColladaLoader } from 'three/addons/loaders/ColladaLoader.js'
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js'
import { invoke } from '@tauri-apps/api/core'
import { rpyToQuat } from './rotationIO'

// ── Loaders ──────────────────────────────────────────────────────────────────

const stlLoader = new STLLoader()
const objLoader = new OBJLoader()
const colladaLoader = new ColladaLoader()
const gltfLoader = new GLTFLoader()

// ── Materials ────────────────────────────────────────────────────────────────

export const defaultMat = new THREE.MeshStandardMaterial({
  color: 0xbbbbbb, roughness: 0.4, metalness: 0.3,
})

// ── Interfaces ───────────────────────────────────────────────────────────────

export interface ParsedRobot {
  group: THREE.Group
  joints: Map<string, { group: THREE.Group; axis: THREE.Vector3; type: string }>
  linkGroups: Map<string, THREE.Group>
  vertexCount: number
  faceCount: number
  linkCount: number
  jointCount: number
}

interface URDFLink {
  name: string
  mass: number
  comPos: THREE.Vector3
  geometry: THREE.Group
}

interface URDFJoint {
  name: string
  type: string
  parentLink: string
  childLink: string
  axis: THREE.Vector3
  origin: { pos: THREE.Vector3; rot: THREE.Quaternion }
}

export interface KinematicLink {
  name: string
  mass: number
  parent?: string
  children: string[]
}

export interface KinematicJoint {
  name: string
  type: string
  axis: string
  parentLink: string
  childLink: string
}

// ── Path resolver callback type ──────────────────────────────────────────────

/**
 * Callback that returns (activeFileDiskPath, openedFolderPath) so the mesh
 * loader can resolve relative paths without depending on main.ts globals.
 */
export type PathResolver = () => {
  activeFileDiskPath: string | null
  openedFolderPath: string | null
}

let _pathResolver: PathResolver = () => ({ activeFileDiskPath: null, openedFolderPath: null })

/** Call once from main.ts to wire up path resolution. */
export function setPathResolver(resolver: PathResolver): void {
  _pathResolver = resolver
}

// ── Mesh file loading (async) ────────────────────────────────────────────────

async function loadMeshFile(
  filename: string,
  parent: THREE.Group,
  placeholder: THREE.Mesh,
  material: THREE.MeshStandardMaterial,
  scaleAttr: string | null,
) {
  try {
    // Resolve path: strip package:// prefix, handle relative paths
    let resolvedPath = filename
    if (resolvedPath.startsWith('package://')) {
      resolvedPath = resolvedPath.replace('package://', '')
    }
    // If relative, resolve from the active file's directory or opened folder
    const { activeFileDiskPath, openedFolderPath } = _pathResolver()
    if (!resolvedPath.match(/^[A-Z]:/i) && !resolvedPath.startsWith('/')) {
      if (activeFileDiskPath) {
        const dir = activeFileDiskPath.replace(/[\\/][^\\/]+$/, '')
        resolvedPath = `${dir}/${resolvedPath}`
      } else if (openedFolderPath) {
        resolvedPath = `${openedFolderPath}/${resolvedPath}`
      }
    }
    console.log(`[mesh] Loading: ${filename} → ${resolvedPath}`)

    let buffer: ArrayBuffer
    if (/^https?:\/\//i.test(resolvedPath) || resolvedPath.startsWith('/')) {
      const res = await fetch(resolvedPath)
      if (!res.ok) throw new Error(`HTTP ${res.status}`)
      buffer = await res.arrayBuffer()
    } else {
      const bytes = await invoke<number[]>('read_binary_file', { path: resolvedPath })
      buffer = new Uint8Array(bytes).buffer
    }

    // Detect format and parse
    const ext = filename.split('.').pop()?.toLowerCase() || ''
    let loadedObject: THREE.Object3D | null = null

    if (ext === 'stl') {
      const geometry = stlLoader.parse(buffer)
      loadedObject = new THREE.Mesh(geometry, material)
    } else if (ext === 'obj') {
      const text = new TextDecoder().decode(buffer)
      const group = objLoader.parse(text)
      // Apply material to all meshes in the OBJ group
      group.traverse(child => {
        if (child instanceof THREE.Mesh && !child.material) child.material = material
      })
      loadedObject = group
    } else if (ext === 'dae') {
      const text = new TextDecoder().decode(buffer)
      const result = colladaLoader.parse(text, resolvedPath)
      loadedObject = result?.scene ?? null
    } else if (ext === 'glb' || ext === 'gltf') {
      // GLTF needs async parsing
      const result = await new Promise<{ scene: THREE.Group }>((resolve, reject) => {
        gltfLoader.parse(buffer, '', resolve, reject)
      })
      loadedObject = result.scene
    } else if (ext === 'step' || ext === 'stp') {
      // STEP files — use OpenCascade WASM
      const { parseSTEP } = await import('./stepLoader')
      loadedObject = await parseSTEP(buffer, material)
    } else if (ext === 'iges' || ext === 'igs') {
      const { parseIGES } = await import('./stepLoader')
      loadedObject = await parseIGES(buffer, material)
    }

    if (!loadedObject) {
      console.warn(`[mesh] Unsupported mesh format: ${ext} (${filename})`)
      return
    }

    // Apply shadow and userData to all meshes
    loadedObject.traverse(child => {
      if (child instanceof THREE.Mesh) {
        child.castShadow = true
        child.receiveShadow = true
      }
    })

    // Apply scale if specified
    if (scaleAttr) {
      const s = scaleAttr.split(/\s+/).map(parseFloat)
      if (s.length >= 3) loadedObject.scale.set(s[0], s[1], s[2])
      else if (s.length === 1 && s[0]) loadedObject.scale.setScalar(s[0])
    }

    // Copy userData from placeholder
    Object.assign(loadedObject.userData, placeholder.userData)

    // Replace placeholder with loaded object
    parent.remove(placeholder)
    placeholder.geometry.dispose()
    parent.add(loadedObject)

  } catch (err) {
    console.warn(`[mesh] Failed to load ${filename}:`, err)
    // Keep placeholder — don't crash
  }
}

// ── URDF Parser ─────────────────────────────────────────────────────────────

export function parseURDFToScene(urdfXml: string): ParsedRobot {
  const parser = new DOMParser()
  const doc = parser.parseFromString(urdfXml, 'application/xml')

  if (doc.documentElement.nodeName === 'parsererror') {
    console.error('URDF parse error')
    throw new Error('Invalid URDF XML')
  }

  const robot = new THREE.Group()
  const linkGroups = new Map<string, THREE.Group>()
  const joints = new Map<string, { group: THREE.Group; axis: THREE.Vector3; type: string }>()
  const linkData = new Map<string, URDFLink>()
  const jointData: URDFJoint[] = []
  const childLinkSet = new Set<string>()

  // Collect named materials defined at robot level (e.g., <material name="Grey"><color rgba="0.7 0.7 0.7 1"/></material>)
  const namedMaterials = new Map<string, THREE.MeshStandardMaterial>()
  const robotEl = doc.querySelector('robot')
  if (robotEl) {
    for (const matEl of robotEl.querySelectorAll(':scope > material')) {
      const matName = matEl.getAttribute('name')
      const colorEl = matEl.querySelector('color')
      if (matName && colorEl) {
        const rgba = (colorEl.getAttribute('rgba') || '0.5 0.5 0.5 1').split(/\s+/).map(parseFloat)
        namedMaterials.set(matName, new THREE.MeshStandardMaterial({
          color: new THREE.Color(rgba[0], rgba[1], rgba[2]),
          roughness: 0.4,
          metalness: 0.3,
        }))
      }
    }
  }

  // Parse all links
  const linkElements = doc.querySelectorAll('link')
  for (const linkEl of linkElements) {
    const linkName = linkEl.getAttribute('name') || ''
    const geometryGroup = new THREE.Group()
    let mass = 0
    let comPos = new THREE.Vector3()

    // Parse mass
    const inertialEl = linkEl.querySelector('inertial')
    if (inertialEl) {
      const massEl = inertialEl.querySelector('mass')
      if (massEl) {
        mass = parseFloat(massEl.getAttribute('value') || '0')
      }
      const originEl = inertialEl.querySelector('origin')
      if (originEl) {
        const xyz = (originEl.getAttribute('xyz') || '0 0 0').split(/\s+/).map(parseFloat)
        comPos = new THREE.Vector3(xyz[0], xyz[1], xyz[2])
      }
    }

    // Parse all visual geometry elements (multiple <visual> per link supported)
    const visualEls = linkEl.querySelectorAll('visual')
    for (const visualEl of visualEls) {
      const geomEl = visualEl.querySelector('geometry')
      if (!geomEl) continue

      let mat: THREE.Material = defaultMat

      // Get material color if specified (inline or by name reference)
      const matEl = visualEl.querySelector('material')
      if (matEl) {
        const colorEl = matEl.querySelector('color')
        if (colorEl) {
          const rgba = (colorEl.getAttribute('rgba') || '0.5 0.5 0.5 1').split(/\s+/).map(parseFloat)
          const color = new THREE.Color(rgba[0], rgba[1], rgba[2])
          mat = new THREE.MeshStandardMaterial({
            color,
            roughness: 0.4,
            metalness: 0.3,
          })
        } else {
          // Try named material lookup
          const matName = matEl.getAttribute('name')
          if (matName && namedMaterials.has(matName)) {
            mat = namedMaterials.get(matName)!
          }
        }
      }

      // Create a sub-group for this visual (each has its own origin transform)
      const visualGroup = new THREE.Group()

      // Parse geometry type
      const cylinderEl = geomEl.querySelector('cylinder')
      if (cylinderEl) {
        const r = parseFloat(cylinderEl.getAttribute('radius') || '0.05')
        const l = parseFloat(cylinderEl.getAttribute('length') || '0.1')
        const geom = new THREE.CylinderGeometry(r, r, l, 32)
        const mesh = new THREE.Mesh(geom, mat)
        mesh.rotation.x = Math.PI / 2
        visualGroup.add(mesh)
      } else {
        const boxEl = geomEl.querySelector('box')
        if (boxEl) {
          const size = (boxEl.getAttribute('size') || '0.1 0.1 0.1').split(/\s+/).map(parseFloat)
          const geom = new THREE.BoxGeometry(size[0], size[1], size[2])
          const mesh = new THREE.Mesh(geom, mat)
          visualGroup.add(mesh)
        } else {
          const sphereEl = geomEl.querySelector('sphere')
          if (sphereEl) {
            const r = parseFloat(sphereEl.getAttribute('radius') || '0.05')
            const geom = new THREE.SphereGeometry(r, 24, 24)
            const mesh = new THREE.Mesh(geom, mat)
            visualGroup.add(mesh)
          } else {
            // Check for mesh file reference
            const meshEl = geomEl.querySelector('mesh')
            if (meshEl) {
              const filename = meshEl.getAttribute('filename') || ''
              const scaleAttr = meshEl.getAttribute('scale')
              // Show placeholder immediately, load mesh async
              const placeholder = new THREE.Mesh(
                new THREE.SphereGeometry(0.02, 16, 16),
                (mat as THREE.Material).clone(),
              )
              placeholder.userData._meshFile = filename
              visualGroup.add(placeholder)
              // Async mesh loading
              loadMeshFile(filename, visualGroup, placeholder, mat as THREE.MeshStandardMaterial, scaleAttr)
            } else {
              const placeholderGeom = new THREE.SphereGeometry(0.02, 16, 16)
              const mesh = new THREE.Mesh(placeholderGeom, mat)
              visualGroup.add(mesh)
            }
          }
        }
      }

      // Apply this visual's origin transform
      const visOriginEl = visualEl.querySelector('origin')
      if (visOriginEl) {
        const xyz = (visOriginEl.getAttribute('xyz') || '0 0 0').split(/\s+/).map(parseFloat)
        const rpy = (visOriginEl.getAttribute('rpy') || '0 0 0').split(/\s+/).map(parseFloat)
        visualGroup.position.set(xyz[0], xyz[1], xyz[2])
        visualGroup.quaternion.copy(rpyToQuat(rpy))
      }

      geometryGroup.add(visualGroup)
    }

    // Add shadow properties and tag with link name for raycasting
    geometryGroup.traverse((child) => {
      if (child instanceof THREE.Mesh) {
        child.castShadow = true
        child.receiveShadow = true
        child.userData.urdfLinkName = linkName
      }
    })

    const linkGroup = new THREE.Group()
    linkGroup.add(geometryGroup)
    linkGroups.set(linkName, linkGroup)
    linkData.set(linkName, { name: linkName, mass, comPos, geometry: linkGroup })
  }

  // Parse all joints
  const jointElements = doc.querySelectorAll('joint')
  for (const jointEl of jointElements) {
    const jointName = jointEl.getAttribute('name') || ''
    const jointType = jointEl.getAttribute('type') || ''

    const parentEl = jointEl.querySelector('parent')
    const childEl = jointEl.querySelector('child')
    const parentLink = parentEl?.getAttribute('link') || ''
    const childLink = childEl?.getAttribute('link') || ''

    if (parentLink && childLink) {
      childLinkSet.add(childLink)

      let axisVec = new THREE.Vector3(0, 0, 1)
      const axisEl = jointEl.querySelector('axis')
      if (axisEl) {
        const xyz = (axisEl.getAttribute('xyz') || '0 0 1').split(/\s+/).map(parseFloat)
        axisVec = new THREE.Vector3(xyz[0], xyz[1], xyz[2]).normalize()
      }

      let pos = new THREE.Vector3()
      let rot = new THREE.Quaternion()
      const originEl = jointEl.querySelector('origin')
      if (originEl) {
        const xyz = (originEl.getAttribute('xyz') || '0 0 0').split(/\s+/).map(parseFloat)
        const rpy = (originEl.getAttribute('rpy') || '0 0 0').split(/\s+/).map(parseFloat)
        pos = new THREE.Vector3(xyz[0], xyz[1], xyz[2])
        rot = rpyToQuat(rpy)
      }

      jointData.push({
        name: jointName,
        type: jointType,
        parentLink,
        childLink,
        axis: axisVec,
        origin: { pos, rot },
      })
    }
  }

  // Build hierarchy
  const rootLinkName = Array.from(linkData.keys()).find((name) => !childLinkSet.has(name)) || 'base_link'
  const rootLinkGroup = linkGroups.get(rootLinkName)
  if (rootLinkGroup) {
    robot.add(rootLinkGroup)
  }

  function attachChildren(parentLinkName: string, parentGroup: THREE.Group) {
    for (const joint of jointData) {
      if (joint.parentLink === parentLinkName) {
        const childLinkName = joint.childLink
        const childLinkGroup = linkGroups.get(childLinkName)
        if (childLinkGroup) {
          const pivotGroup = new THREE.Group()
          pivotGroup.position.copy(joint.origin.pos)
          pivotGroup.quaternion.copy(joint.origin.rot)
          pivotGroup.add(childLinkGroup)
          parentGroup.add(pivotGroup)

          joints.set(joint.name, { group: pivotGroup, axis: joint.axis, type: joint.type })

          attachChildren(childLinkName, childLinkGroup)
        }
      }
    }
  }

  attachChildren(rootLinkName, rootLinkGroup || robot)

  // Count meshes
  let vertexCount = 0
  let faceCount = 0
  robot.traverse((child) => {
    if (child instanceof THREE.Mesh && child.geometry) {
      const posAttr = child.geometry.getAttribute('position')
      if (posAttr) {
        vertexCount += posAttr.count
      }
      if (child.geometry.getIndex()) {
        faceCount += child.geometry.getIndex()!.count / 3
      }
    }
  })

  return {
    group: robot,
    joints,
    linkGroups,
    vertexCount,
    faceCount,
    linkCount: linkData.size,
    jointCount: jointData.length,
  }
}

// ── Kinematic Graph Builder ─────────────────────────────────────────────────

export function buildKinematicGraphFromURDF(urdfXml: string): {
  kinematicGraph: Record<string, KinematicLink>
  kinematicJoints: Record<string, KinematicJoint>
} {
  const parser = new DOMParser()
  const doc = parser.parseFromString(urdfXml, 'application/xml')

  const kinematicGraph: Record<string, KinematicLink> = {}
  const kinematicJoints: Record<string, KinematicJoint> = {}
  const childLinkSet = new Set<string>()

  // Parse all links
  const linkElements = doc.querySelectorAll('link')
  for (const linkEl of linkElements) {
    const linkName = linkEl.getAttribute('name') || ''
    let mass = 0

    const inertialEl = linkEl.querySelector('inertial')
    if (inertialEl) {
      const massEl = inertialEl.querySelector('mass')
      if (massEl) {
        mass = parseFloat(massEl.getAttribute('value') || '0')
      }
    }

    kinematicGraph[linkName] = {
      name: linkName,
      mass,
      children: [],
    }
  }

  // Parse all joints
  const jointElements = doc.querySelectorAll('joint')
  for (const jointEl of jointElements) {
    const jointName = jointEl.getAttribute('name') || ''
    const jointType = jointEl.getAttribute('type') || ''

    const parentEl = jointEl.querySelector('parent')
    const childEl = jointEl.querySelector('child')
    const parentLink = parentEl?.getAttribute('link') || ''
    const childLink = childEl?.getAttribute('link') || ''

    if (parentLink && childLink) {
      childLinkSet.add(childLink)

      let axis = '--'
      const axisEl = jointEl.querySelector('axis')
      if (axisEl) {
        const xyz = (axisEl.getAttribute('xyz') || '0 0 1').split(/\s+/).map(parseFloat)
        if (Math.abs(xyz[0]) > 0.5) axis = 'X'
        else if (Math.abs(xyz[1]) > 0.5) axis = 'Y'
        else if (Math.abs(xyz[2]) > 0.5) axis = 'Z'
      }

      kinematicJoints[jointName] = {
        name: jointName,
        type: jointType,
        axis,
        parentLink,
        childLink,
      }

      // Add child to parent's children list
      if (kinematicGraph[parentLink]) {
        kinematicGraph[parentLink].children.push(childLink)
      }
    }
  }

  // Set parent links and identify root
  for (const [linkName, link] of Object.entries(kinematicGraph)) {
    if (childLinkSet.has(linkName)) {
      // Find parent
      for (const joint of Object.values(kinematicJoints)) {
        if (joint.childLink === linkName) {
          link.parent = joint.parentLink
          break
        }
      }
    }
  }

  return { kinematicGraph, kinematicJoints }
}
