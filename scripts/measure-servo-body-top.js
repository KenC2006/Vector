/**
 * Measure body_top (= max Z of the largest mesh component) for each servo GLB.
 * The body extends along ±Z; the shaft is a thin protrusion at +Z. The
 * authored `top` connector for face-mounting children should land on body_top
 * (NOT bbox_top = shaft tip), so children sit flush against the servo body.
 *
 * Usage: node scripts/measure-servo-body-top.js
 *
 * Reuses the GLB walker from audit-preset-bboxes.js (zero-dep). The "body" is
 * inferred as the largest primitive by bounding-box volume; everything else
 * is treated as a protrusion (shaft, horn, mounting tab).
 */

'use strict'

const fs = require('fs')
const path = require('path')

const REPO_ROOT = path.resolve(__dirname, '..')
const GLB_DIR = path.join(REPO_ROOT, 'src', 'public', 'meshes', 'glb')

const SERVO_PRESETS = [
  // [preset_id, glb_filename, bbox_mm, rotation_xyz_rad]
  ['actuator_servo_micro',       'servo_small.glb',         [23,    12.2, 29], [0, 0, 0]],
  ['actuator_servo_standard',    'servo_standard.glb',      [40,    20,   37], [0, 0, 0]],
  ['actuator_servo_high_torque', 'servo_high_torque.glb',   [46.5,  36,   34], [0, 0, 0]],
  // heavy_duty shares the high_torque GLB (per meshOverrides.ts)
  ['actuator_servo_heavy_duty',  'servo_high_torque.glb',   [54,    42,   54], [0, 0, 0]],
]

function loadGltfJson(filePath) {
  const buf = fs.readFileSync(filePath)
  const dv = new DataView(buf.buffer, buf.byteOffset, buf.byteLength)
  const magic = dv.getUint32(0, true)
  if (magic !== 0x46546c67) throw new Error(`not a GLB: ${filePath}`)
  const jsonChunkLength = dv.getUint32(12, true)
  const jsonChunkType = dv.getUint32(16, true)
  if (jsonChunkType !== 0x4e4f534a) throw new Error(`expected JSON chunk: ${filePath}`)
  const jsonBytes = buf.subarray(20, 20 + jsonChunkLength)
  return JSON.parse(new TextDecoder().decode(jsonBytes))
}

function nodeLocalMatrix(node) {
  if (node.matrix) return node.matrix.slice()
  const t = node.translation || [0, 0, 0]
  const r = node.rotation || [0, 0, 0, 1]
  const s = node.scale || [1, 1, 1]
  const [x, y, z, w] = r
  const xx = x * x, yy = y * y, zz = z * z
  const xy = x * y, xz = x * z, yz = y * z
  const wx = w * x, wy = w * y, wz = w * z
  return [
    (1 - 2 * (yy + zz)) * s[0], 2 * (xy + wz) * s[0], 2 * (xz - wy) * s[0], 0,
    2 * (xy - wz) * s[1], (1 - 2 * (xx + zz)) * s[1], 2 * (yz + wx) * s[1], 0,
    2 * (xz + wy) * s[2], 2 * (yz - wx) * s[2], (1 - 2 * (xx + yy)) * s[2], 0,
    t[0], t[1], t[2], 1,
  ]
}

function mat4Multiply(a, b) {
  const out = new Array(16).fill(0)
  for (let c = 0; c < 4; c++) {
    for (let r = 0; r < 4; r++) {
      let v = 0
      for (let k = 0; k < 4; k++) v += a[k * 4 + r] * b[c * 4 + k]
      out[c * 4 + r] = v
    }
  }
  return out
}

function transformPoint(m, p) {
  return [
    m[0] * p[0] + m[4] * p[1] + m[8]  * p[2] + m[12],
    m[1] * p[0] + m[5] * p[1] + m[9]  * p[2] + m[13],
    m[2] * p[0] + m[6] * p[1] + m[10] * p[2] + m[14],
  ]
}

function transformAabb(m, min, max) {
  const corners = [
    [min[0], min[1], min[2]], [max[0], min[1], min[2]],
    [min[0], max[1], min[2]], [max[0], max[1], min[2]],
    [min[0], min[1], max[2]], [max[0], min[1], max[2]],
    [min[0], max[1], max[2]], [max[0], max[1], max[2]],
  ]
  const newMin = [Infinity, Infinity, Infinity]
  const newMax = [-Infinity, -Infinity, -Infinity]
  for (const c0 of corners) {
    const c = transformPoint(m, c0)
    for (let i = 0; i < 3; i++) {
      if (c[i] < newMin[i]) newMin[i] = c[i]
      if (c[i] > newMax[i]) newMax[i] = c[i]
    }
  }
  return { min: newMin, max: newMax }
}

function gatherPrimitives(json, nodeIdx, parentMatrix, out) {
  const node = json.nodes[nodeIdx]
  const worldMatrix = mat4Multiply(parentMatrix, nodeLocalMatrix(node))
  if (typeof node.mesh === 'number') {
    const mesh = json.meshes[node.mesh]
    for (const prim of mesh.primitives) {
      const posIdx = prim.attributes && prim.attributes.POSITION
      if (typeof posIdx !== 'number') continue
      const acc = json.accessors[posIdx]
      if (!acc || !acc.min || !acc.max) continue
      const aabb = transformAabb(worldMatrix, acc.min, acc.max)
      out.push({
        nodeName: node.name || `node_${nodeIdx}`,
        meshName: mesh.name || `mesh_${node.mesh}`,
        min: aabb.min,
        max: aabb.max,
        size: [
          aabb.max[0] - aabb.min[0],
          aabb.max[1] - aabb.min[1],
          aabb.max[2] - aabb.min[2],
        ],
      })
    }
  }
  for (const child of node.children || []) gatherPrimitives(json, child, worldMatrix, out)
}

function measureGlb(filePath) {
  const json = loadGltfJson(filePath)
  const scene = json.scenes[json.scene != null ? json.scene : 0]
  const prims = []
  const identity = [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]
  for (const root of scene.nodes) gatherPrimitives(json, root, identity, prims)
  return prims
}

function aabbVolume(p) {
  return p.size[0] * p.size[1] * p.size[2]
}

function main() {
  for (const [presetId, glbName, bbox, rot] of SERVO_PRESETS) {
    const glbPath = path.join(GLB_DIR, glbName)
    if (!fs.existsSync(glbPath)) {
      console.log(`SKIP ${presetId}: GLB missing at ${glbPath}`)
      continue
    }
    const prims = measureGlb(glbPath)
    if (prims.length === 0) {
      console.log(`SKIP ${presetId}: no primitives`)
      continue
    }

    // Total mesh AABB (used to compute the per-axis scale that will be
    // applied at runtime in applyMeshToLink).
    const totalMin = [Infinity, Infinity, Infinity]
    const totalMax = [-Infinity, -Infinity, -Infinity]
    for (const p of prims) {
      for (let i = 0; i < 3; i++) {
        if (p.min[i] < totalMin[i]) totalMin[i] = p.min[i]
        if (p.max[i] > totalMax[i]) totalMax[i] = p.max[i]
      }
    }
    const totalSize = [
      totalMax[0] - totalMin[0],
      totalMax[1] - totalMin[1],
      totalMax[2] - totalMin[2],
    ]

    // After runtime per-axis scaling, every axis is stretched/squished to fit
    // bbox_mm. Then the mesh is recentered on origin. So a primitive's
    // post-runtime max.z = (prim.max.z - totalCenter.z) * (bbox.z / totalSize.z).
    const totalCenter = [
      (totalMin[0] + totalMax[0]) / 2,
      (totalMin[1] + totalMax[1]) / 2,
      (totalMin[2] + totalMax[2]) / 2,
    ]
    const scale = [bbox[0] / totalSize[0], bbox[1] / totalSize[1], bbox[2] / totalSize[2]]

    // Sort primitives by volume descending — the largest is the "body".
    const ranked = prims.slice().sort((a, b) => aabbVolume(b) - aabbVolume(a))
    const body = ranked[0]
    const bodyMaxZScaled = (body.max[2] - totalCenter[2]) * scale[2]
    const bodyMinZScaled = (body.min[2] - totalCenter[2]) * scale[2]
    const bbox_hz = bbox[2] / 2
    const shaftLength = bbox_hz - bodyMaxZScaled

    console.log(`\n=== ${presetId} (GLB: ${glbName}) ===`)
    console.log(`  raw AABB: [${totalSize.map(v => v.toFixed(2)).join(', ')}] mm`)
    console.log(`  bbox_mm:  [${bbox.join(', ')}] (scale per-axis: [${scale.map(v => v.toFixed(3)).join(', ')}])`)
    console.log(`  ${prims.length} primitives:`)
    for (const p of ranked.slice(0, 5)) {
      const sz = p.size.map(v => v.toFixed(2)).join('×')
      const cz = ((p.min[2] + p.max[2]) / 2).toFixed(2)
      console.log(`    [${sz}] center.z=${cz} vol=${aabbVolume(p).toFixed(0)}  ${p.nodeName}/${p.meshName}`)
    }
    console.log(`  → BODY (largest): max.z (post-scale, recentered) = ${bodyMaxZScaled.toFixed(2)} mm`)
    console.log(`  → BODY: min.z (post-scale, recentered) = ${bodyMinZScaled.toFixed(2)} mm`)
    console.log(`  → SHAFT length (bbox top - body top) = ${shaftLength.toFixed(2)} mm`)
    console.log(`  → RECOMMENDED top connector z = ${bodyMaxZScaled.toFixed(2)} mm`)
    console.log(`  → RECOMMENDED bottom connector z = ${bodyMinZScaled.toFixed(2)} mm`)
  }
}

main()
