/**
 * STEP file loader using occt-import-js (OpenCascade WebAssembly).
 *
 * Converts STEP/STP/IGES files to Three.js BufferGeometry at runtime
 * in the browser — no server-side conversion needed.
 */
import * as THREE from 'three'

// occt-import-js is loaded dynamically to avoid blocking initial load
let occtModule: any = null
let occtLoading: Promise<any> | null = null

async function getOcct(): Promise<any> {
  if (occtModule) return occtModule
  if (occtLoading) return occtLoading

  occtLoading = (async () => {
    try {
      // Dynamic import of occt-import-js
      const occtInit = (await import('occt-import-js')).default
      // Configure WASM location — serve from public/ to avoid Vite MIME issues
      occtModule = await occtInit({
        locateFile: (filename: string) => {
          if (filename.endsWith('.wasm')) return '/occt-import-js.wasm'
          return filename
        },
      })
      console.log('[STEP] OpenCascade WASM loaded')
      return occtModule
    } catch (e) {
      console.error('[STEP] Failed to load OpenCascade:', e)
      occtLoading = null
      throw e
    }
  })()

  return occtLoading
}

/**
 * Parse a STEP/STP file buffer into a Three.js Group.
 *
 * @param buffer - ArrayBuffer of the STEP file
 * @param material - Optional material to apply (defaults to gray standard)
 * @returns THREE.Group containing all meshes from the STEP file
 */
export async function parseSTEP(
  buffer: ArrayBuffer,
  material?: THREE.Material,
): Promise<THREE.Group> {
  const occt = await getOcct()

  const fileBuffer = new Uint8Array(buffer)
  const result = occt.ReadStepFile(fileBuffer, null)

  if (!result.success) {
    throw new Error('Failed to parse STEP file')
  }

  const group = new THREE.Group()
  const defaultMat = material || new THREE.MeshStandardMaterial({
    color: 0x888888,
    roughness: 0.4,
    metalness: 0.3,
  })

  // Process each mesh in the result
  for (let i = 0; i < result.meshes.length; i++) {
    const mesh = result.meshes[i]

    // Create BufferGeometry from the mesh data
    const geometry = new THREE.BufferGeometry()

    // Vertices (flat array of x, y, z)
    const vertices = new Float32Array(mesh.attributes.position.array)
    geometry.setAttribute('position', new THREE.BufferAttribute(vertices, 3))

    // Normals
    if (mesh.attributes.normal) {
      const normals = new Float32Array(mesh.attributes.normal.array)
      geometry.setAttribute('normal', new THREE.BufferAttribute(normals, 3))
    }

    // Indices
    if (mesh.index) {
      const indices = new Uint32Array(mesh.index.array)
      geometry.setIndex(new THREE.BufferAttribute(indices, 1))
    }

    // Compute normals if not provided
    if (!mesh.attributes.normal) {
      geometry.computeVertexNormals()
    }

    // Use explicitly passed material, or STEP embedded color, or default
    let meshMat = defaultMat
    if (!material && mesh.color) {
      // Only use STEP colors if no explicit material was passed
      const [r, g, b] = mesh.color
      meshMat = new THREE.MeshStandardMaterial({
        color: new THREE.Color(r / 255, g / 255, b / 255),
        roughness: 0.4,
        metalness: 0.3,
      })
    }

    const threeMesh = new THREE.Mesh(geometry, meshMat)
    threeMesh.castShadow = true
    threeMesh.receiveShadow = true
    group.add(threeMesh)
  }

  // Process node hierarchy if present
  if (result.root && result.root.children) {
    applyNodeTransforms(result.root, group, result.meshes)
  }

  return group
}

/**
 * Parse an IGES file buffer into a Three.js Group.
 */
export async function parseIGES(
  buffer: ArrayBuffer,
  material?: THREE.Material,
): Promise<THREE.Group> {
  const occt = await getOcct()
  const fileBuffer = new Uint8Array(buffer)
  const result = occt.ReadIgesFile(fileBuffer, null)

  if (!result.success) {
    throw new Error('Failed to parse IGES file')
  }

  // Same mesh processing as STEP
  return buildGroupFromResult(result, material)
}

function buildGroupFromResult(result: any, material?: THREE.Material): THREE.Group {
  const group = new THREE.Group()
  const defaultMat = material || new THREE.MeshStandardMaterial({
    color: 0x888888,
    roughness: 0.4,
    metalness: 0.3,
  })

  for (let i = 0; i < result.meshes.length; i++) {
    const mesh = result.meshes[i]
    const geometry = new THREE.BufferGeometry()

    const vertices = new Float32Array(mesh.attributes.position.array)
    geometry.setAttribute('position', new THREE.BufferAttribute(vertices, 3))

    if (mesh.attributes.normal) {
      const normals = new Float32Array(mesh.attributes.normal.array)
      geometry.setAttribute('normal', new THREE.BufferAttribute(normals, 3))
    }

    if (mesh.index) {
      const indices = new Uint32Array(mesh.index.array)
      geometry.setIndex(new THREE.BufferAttribute(indices, 1))
    }

    if (!mesh.attributes.normal) {
      geometry.computeVertexNormals()
    }

    let meshMat = defaultMat
    if (mesh.color) {
      const [r, g, b] = mesh.color
      meshMat = new THREE.MeshStandardMaterial({
        color: new THREE.Color(r / 255, g / 255, b / 255),
        roughness: 0.4,
        metalness: 0.3,
      })
    }

    const threeMesh = new THREE.Mesh(geometry, meshMat)
    threeMesh.castShadow = true
    threeMesh.receiveShadow = true
    group.add(threeMesh)
  }

  return group
}

function applyNodeTransforms(node: any, group: THREE.Group, _meshes: any[]) {
  // The node hierarchy provides names and transforms
  // For now, we just use the flat mesh list
  if (node.name) {
    group.name = node.name
  }
}
