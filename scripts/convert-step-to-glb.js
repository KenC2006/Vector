/**
 * STEP → GLB Build Script
 *
 * Converts all STEP/STP files in src/public/meshes/components/ to GLB format
 * for fast runtime loading (~100ms vs 5-15s for STEP parsing in-browser).
 *
 * Uses occt-import-js (OpenCascade WASM) for STEP parsing, then writes
 * GLB (Binary glTF 2.0) directly — no Three.js renderer or browser APIs needed.
 *
 * Usage:
 *   cd src && node ../scripts/convert-step-to-glb.js
 *   node ../scripts/convert-step-to-glb.js --force   # re-convert all
 *   node ../scripts/convert-step-to-glb.js servo_small.step  # convert one file
 */

const fs = require('fs')
const path = require('path')

// Resolve modules from src/node_modules since that's where they're installed
const SRC_DIR = path.join(__dirname, '..', 'src')
const occtInit = require(path.join(SRC_DIR, 'node_modules', 'occt-import-js'))

const COMPONENTS_DIR = path.join(SRC_DIR, 'public', 'meshes', 'components')
const GLB_DIR = path.join(__dirname, '..', 'src', 'public', 'meshes', 'glb')

// Files too large / complex — skip entirely (use parametric at runtime)
const SKIP_FILES = new Set([
  'sbc_gpu.stp',           // 71MB — would produce huge GLB
  'mobility_track.step',   // 50MB
])

// Size threshold: skip STEP files larger than this (bytes)
const MAX_STEP_SIZE = 25 * 1024 * 1024 // 25 MB

async function main() {
  const args = process.argv.slice(2)
  const forceAll = args.includes('--force')
  const singleFile = args.find(a => a.endsWith('.step') || a.endsWith('.stp'))

  // Ensure output dir exists
  fs.mkdirSync(GLB_DIR, { recursive: true })

  // Initialize OpenCascade WASM
  console.log('[convert] Loading OpenCascade WASM...')
  const occt = await occtInit()
  console.log('[convert] Ready.\n')

  // Gather files to convert
  let files = fs.readdirSync(COMPONENTS_DIR)
    .filter(f => f.endsWith('.step') || f.endsWith('.stp'))
    .sort()

  if (singleFile) {
    files = files.filter(f => f === singleFile)
    if (files.length === 0) {
      console.error(`File not found: ${singleFile}`)
      process.exit(1)
    }
  }

  console.log(`[convert] ${files.length} STEP files to process\n`)

  let converted = 0, skipped = 0, failed = 0

  for (const file of files) {
    const stepPath = path.join(COMPONENTS_DIR, file)
    const baseName = file.replace(/\.(step|stp)$/i, '')
    const glbPath = path.join(GLB_DIR, baseName + '.glb')

    // Skip blacklisted
    if (SKIP_FILES.has(file)) {
      console.log(`  SKIP  ${file} (blacklisted)`)
      skipped++
      continue
    }

    // Check file size
    const stat = fs.statSync(stepPath)
    if (stat.size > MAX_STEP_SIZE) {
      console.log(`  SKIP  ${file} (${(stat.size / 1024 / 1024).toFixed(1)}MB > ${MAX_STEP_SIZE / 1024 / 1024}MB limit)`)
      skipped++
      continue
    }

    // Skip if GLB already exists and is newer than STEP (unless --force)
    if (!forceAll && fs.existsSync(glbPath)) {
      const glbStat = fs.statSync(glbPath)
      if (glbStat.mtimeMs > stat.mtimeMs) {
        console.log(`  SKIP  ${file} (GLB up to date)`)
        skipped++
        continue
      }
    }

    // Parse STEP
    const t0 = Date.now()
    try {
      const stepBuf = fs.readFileSync(stepPath)
      const result = occt.ReadStepFile(new Uint8Array(stepBuf), null)

      if (!result.success || result.meshes.length === 0) {
        console.log(`  FAIL  ${file} (parse failed or no meshes)`)
        failed++
        continue
      }

      // Build GLB
      const glbBuffer = buildGLB(result.meshes)
      fs.writeFileSync(glbPath, glbBuffer)

      const elapsed = ((Date.now() - t0) / 1000).toFixed(1)
      const ratio = ((glbBuffer.length / stepBuf.length) * 100).toFixed(0)
      console.log(`  OK    ${file} → ${baseName}.glb  (${formatSize(stepBuf.length)} → ${formatSize(glbBuffer.length)}, ${ratio}%, ${elapsed}s)`)
      converted++
    } catch (e) {
      const elapsed = ((Date.now() - t0) / 1000).toFixed(1)
      console.log(`  FAIL  ${file} (${elapsed}s): ${e.message}`)
      failed++
    }
  }

  console.log(`\n[convert] Done: ${converted} converted, ${skipped} skipped, ${failed} failed`)
}

function formatSize(bytes) {
  if (bytes < 1024) return bytes + 'B'
  if (bytes < 1024 * 1024) return (bytes / 1024).toFixed(0) + 'KB'
  return (bytes / (1024 * 1024)).toFixed(1) + 'MB'
}

// ── GLB Writer ───────────────────────────────────────────────────────────────
// GLB = 12-byte header + JSON chunk + BIN chunk
// We write a minimal glTF 2.0 with one scene containing N meshes.

function buildGLB(meshes) {
  // Collect all geometry into a single binary buffer
  const accessors = []
  const bufferViews = []
  const gltfMeshes = []
  const nodes = []
  const bufferParts = []
  let byteOffset = 0

  for (let i = 0; i < meshes.length; i++) {
    const mesh = meshes[i]
    const primitive = {}
    const attributes = {}

    // Positions (required)
    const posArray = new Float32Array(mesh.attributes.position.array)
    const posBytes = Buffer.from(posArray.buffer, posArray.byteOffset, posArray.byteLength)

    // Compute position bounds for glTF spec (required for POSITION)
    const vertexCount = posArray.length / 3
    const posMin = [Infinity, Infinity, Infinity]
    const posMax = [-Infinity, -Infinity, -Infinity]
    for (let v = 0; v < vertexCount; v++) {
      for (let c = 0; c < 3; c++) {
        const val = posArray[v * 3 + c]
        if (val < posMin[c]) posMin[c] = val
        if (val > posMax[c]) posMax[c] = val
      }
    }

    // Pad to 4-byte alignment
    const posPadded = padTo4(posBytes)
    bufferViews.push({
      buffer: 0,
      byteOffset,
      byteLength: posBytes.length,
    })
    accessors.push({
      bufferView: bufferViews.length - 1,
      componentType: 5126, // FLOAT
      count: vertexCount,
      type: 'VEC3',
      min: posMin,
      max: posMax,
    })
    attributes.POSITION = accessors.length - 1
    byteOffset += posPadded.length
    bufferParts.push(posPadded)

    // Normals (optional)
    if (mesh.attributes.normal) {
      const normArray = new Float32Array(mesh.attributes.normal.array)
      const normBytes = Buffer.from(normArray.buffer, normArray.byteOffset, normArray.byteLength)
      const normPadded = padTo4(normBytes)
      bufferViews.push({
        buffer: 0,
        byteOffset,
        byteLength: normBytes.length,
      })
      accessors.push({
        bufferView: bufferViews.length - 1,
        componentType: 5126,
        count: normArray.length / 3,
        type: 'VEC3',
      })
      attributes.NORMAL = accessors.length - 1
      byteOffset += normPadded.length
      bufferParts.push(normPadded)
    }

    // Indices (optional)
    if (mesh.index) {
      const idxArray = new Uint32Array(mesh.index.array)
      const idxBytes = Buffer.from(idxArray.buffer, idxArray.byteOffset, idxArray.byteLength)
      const idxPadded = padTo4(idxBytes)
      bufferViews.push({
        buffer: 0,
        byteOffset,
        byteLength: idxBytes.length,
      })
      accessors.push({
        bufferView: bufferViews.length - 1,
        componentType: 5125, // UNSIGNED_INT
        count: idxArray.length,
        type: 'SCALAR',
      })
      primitive.indices = accessors.length - 1
      byteOffset += idxPadded.length
      bufferParts.push(idxPadded)
    }

    // Material — store STEP color if present
    primitive.attributes = attributes
    if (mesh.color) {
      primitive.material = i // we'll create per-mesh materials below
    }

    gltfMeshes.push({ primitives: [primitive] })
    nodes.push({ mesh: i })
  }

  // Materials (one per mesh with color, or shared default)
  const materials = []
  for (let i = 0; i < meshes.length; i++) {
    if (meshes[i].color) {
      const [r, g, b] = meshes[i].color
      materials.push({
        pbrMetallicRoughness: {
          baseColorFactor: [r / 255, g / 255, b / 255, 1],
          metallicFactor: 0.3,
          roughnessFactor: 0.4,
        },
      })
    } else {
      materials.push({
        pbrMetallicRoughness: {
          baseColorFactor: [0.533, 0.533, 0.533, 1],
          metallicFactor: 0.3,
          roughnessFactor: 0.4,
        },
      })
    }
  }

  // Assemble binary buffer
  const binBuffer = Buffer.concat(bufferParts)

  // Build glTF JSON
  const gltf = {
    asset: { version: '2.0', generator: 'vector-step-converter' },
    scene: 0,
    scenes: [{ nodes: nodes.map((_, i) => i) }],
    nodes,
    meshes: gltfMeshes,
    accessors,
    bufferViews,
    buffers: [{ byteLength: binBuffer.length }],
    materials,
  }

  const jsonStr = JSON.stringify(gltf)
  const jsonBuf = Buffer.from(jsonStr, 'utf8')
  const jsonPadded = padTo4(jsonBuf, 0x20) // pad JSON with spaces

  // GLB structure:
  // Header: magic(4) + version(4) + length(4) = 12 bytes
  // JSON chunk: chunkLength(4) + chunkType(4) + data
  // BIN chunk: chunkLength(4) + chunkType(4) + data
  const totalLength = 12 + 8 + jsonPadded.length + 8 + padTo4(binBuffer).length
  const binPadded = padTo4(binBuffer, 0x00)

  const out = Buffer.alloc(12 + 8 + jsonPadded.length + 8 + binPadded.length)
  let offset = 0

  // Header
  out.writeUInt32LE(0x46546C67, offset); offset += 4  // 'glTF'
  out.writeUInt32LE(2, offset); offset += 4            // version 2
  out.writeUInt32LE(out.length, offset); offset += 4   // total length

  // JSON chunk
  out.writeUInt32LE(jsonPadded.length, offset); offset += 4
  out.writeUInt32LE(0x4E4F534A, offset); offset += 4  // 'JSON'
  jsonPadded.copy(out, offset); offset += jsonPadded.length

  // BIN chunk
  out.writeUInt32LE(binPadded.length, offset); offset += 4
  out.writeUInt32LE(0x004E4942, offset); offset += 4  // 'BIN\0'
  binPadded.copy(out, offset)

  return out
}

/**
 * Pad a buffer to 4-byte alignment.
 * @param {Buffer} buf
 * @param {number} padByte - byte value for padding (0x00 for BIN, 0x20 for JSON)
 */
function padTo4(buf, padByte = 0x00) {
  const remainder = buf.length % 4
  if (remainder === 0) return buf
  const padding = Buffer.alloc(4 - remainder, padByte)
  return Buffer.concat([buf, padding])
}

main().catch(e => {
  console.error('Fatal:', e)
  process.exit(1)
})
