// Replicate the exact ICP runtime for the quadruped's thigh_extrusion on
// hip_pitch_servo.bottom mate, using real GLB geometry for the servo and
// parametric box geometry for the extrusion. Dumps per-sample axials.

import * as THREE from '../src/node_modules/three/build/three.module.js'
import fs from 'node:fs/promises'
import path from 'node:path'
import { fileURLToPath } from 'node:url'
import { nudgeAlongNormal } from '../src/src/contactCleanup.ts'

const here = path.dirname(fileURLToPath(import.meta.url))
const repoRoot = path.resolve(here, '..')

// ── Load the servo_high_torque GLB and build a Three.js BufferGeometry ──
function loadGlbDoc(filePath) {
  return fs.readFile(filePath).then(buf => {
    const dv = new DataView(buf.buffer, buf.byteOffset, buf.byteLength)
    const jsonChunkLength = dv.getUint32(12, true)
    const jsonBytes = buf.subarray(20, 20 + jsonChunkLength)
    const json = JSON.parse(new TextDecoder().decode(jsonBytes))
    const binOffset = 20 + jsonChunkLength
    const binLength = dv.getUint32(binOffset, true)
    const bin = buf.subarray(binOffset + 8, binOffset + 8 + binLength)
    return { json, bin }
  })
}
function accReader(json, bin, accIdx) {
  const acc = json.accessors[accIdx]
  const bv = json.bufferViews[acc.bufferView]
  const offset = (bv.byteOffset || 0) + (acc.byteOffset || 0)
  const compSz = (acc.type === 'VEC3' ? 3 : 1) * (acc.componentType === 5126 ? 4 : acc.componentType === 5123 ? 2 : 4)
  const stride = bv.byteStride || compSz
  const view = new DataView(bin.buffer, bin.byteOffset + offset, acc.count * stride)
  return { acc, view, stride }
}
async function loadServoGeometry(bboxMm, shaftLenMm) {
  const glb = path.join(repoRoot, 'src/public/meshes/glb/servo_high_torque.glb')
  const { json, bin } = await loadGlbDoc(glb)
  const scene = json.scenes[json.scene ?? 0]
  const positions = []
  const indices = []
  let vertOffset = 0
  const walk = (nodeIdx) => {
    const node = json.nodes[nodeIdx]
    if (typeof node.mesh === 'number') {
      const mesh = json.meshes[node.mesh]
      for (const prim of mesh.primitives) {
        const posIdx = prim.attributes.POSITION
        const { acc: posAcc, view: posView, stride: posStride } = accReader(json, bin, posIdx)
        const startVert = vertOffset
        for (let i = 0; i < posAcc.count; i++) {
          positions.push(
            posView.getFloat32(i * posStride, true),
            posView.getFloat32(i * posStride + 4, true),
            posView.getFloat32(i * posStride + 8, true),
          )
        }
        vertOffset += posAcc.count
        if (typeof prim.indices === 'number') {
          const { acc: idxAcc, view: idxView } = accReader(json, bin, prim.indices)
          const getIdx = idxAcc.componentType === 5123 ? i => idxView.getUint16(i * 2, true) :
                         idxAcc.componentType === 5125 ? i => idxView.getUint32(i * 4, true) :
                         i => idxView.getUint8(i)
          for (let i = 0; i < idxAcc.count; i++) indices.push(startVert + getIdx(i))
        } else {
          for (let i = 0; i < posAcc.count; i++) indices.push(startVert + i)
        }
      }
    }
    for (const c of node.children || []) walk(c)
  }
  for (const r of scene.nodes) walk(r)

  // Normalize: AABB → per-axis scale to (bbox - shaftLen on Z) + recenter + shaftLen shift.
  const posArr = new Float32Array(positions)
  let mn = [Infinity, Infinity, Infinity], mx = [-Infinity, -Infinity, -Infinity]
  for (let i = 0; i < posArr.length; i += 3) {
    for (let j = 0; j < 3; j++) {
      if (posArr[i + j] < mn[j]) mn[j] = posArr[i + j]
      if (posArr[i + j] > mx[j]) mx[j] = posArr[i + j]
    }
  }
  const size = [mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2]]
  const ctr = [(mn[0] + mx[0]) / 2, (mn[1] + mx[1]) / 2, (mn[2] + mx[2]) / 2]
  const targetZ = bboxMm[2] - shaftLenMm
  const s = [bboxMm[0] / size[0], bboxMm[1] / size[1], targetZ / size[2]]
  const zOff = -shaftLenMm / 2
  const transformed = new Float32Array(positions.length)
  for (let i = 0; i < posArr.length; i += 3) {
    transformed[i]     = (posArr[i]     - ctr[0]) * s[0] / 1000 // mm → m
    transformed[i + 1] = (posArr[i + 1] - ctr[1]) * s[1] / 1000
    transformed[i + 2] = ((posArr[i + 2] - ctr[2]) * s[2] + zOff) / 1000
  }

  const geom = new THREE.BufferGeometry()
  geom.setAttribute('position', new THREE.BufferAttribute(transformed, 3))
  geom.setIndex(indices)
  geom.computeBoundingBox()
  return geom
}

async function main() {
  // Parent (servo) link group with body GLB + procedural shaft cylinder.
  const parentLg = new THREE.Group()
  parentLg.name = 'servo_link'
  {
    const geomChild = new THREE.Group() // matches URDF parser structure
    const servoGeom = await loadServoGeometry([46.5, 36, 34], 5)
    const mat = new THREE.MeshBasicMaterial()
    const bodyMesh = new THREE.Mesh(servoGeom, mat)
    geomChild.add(bodyMesh)
    // Shaft overlay cylinder (matches applyMeshToLink's procedural shaft):
    const shaftGeo = new THREE.CylinderGeometry(0.004, 0.004, 0.005, 24)
    const shaftMesh = new THREE.Mesh(shaftGeo, mat)
    shaftMesh.rotation.x = Math.PI / 2
    shaftMesh.position.z = 0.034 / 2 - 0.005 / 2 // 0.017 - 0.0025 = 0.0145
    geomChild.add(shaftMesh)
    parentLg.add(geomChild)
  }

  // Child (extrusion) link group with a parametric 20x20x100 box.
  const extrusionLen = 0.100
  const childLg = new THREE.Group()
  childLg.name = 'extrusion_link'
  {
    const geomChild = new THREE.Group()
    const extrusionGeom = new THREE.BoxGeometry(0.020, 0.020, extrusionLen)
    const mat = new THREE.MeshBasicMaterial()
    geomChild.add(new THREE.Mesh(extrusionGeom, mat))
    childLg.add(geomChild)
  }

  // Mate: pivot places extrusion center at (0, 0, -67mm) in parent local
  // (parent.bottom at -17, child.top at +50, fastened → pivot -17-50 = -67).
  const pivot = new THREE.Group()
  pivot.position.set(0, 0, -0.067)
  pivot.add(childLg)
  parentLg.add(pivot)
  parentLg.updateMatrixWorld(true)

  // Connector geometry: parent.bottom at (0,0,-17mm), axis (0,0,-1).
  //                    child.top at (0,0,+50mm), axis (0,0,+1).
  const pOriginWorld = new THREE.Vector3(0, 0, -0.017).applyMatrix4(parentLg.matrixWorld)
  const pAxisWorld = new THREE.Vector3(0, 0, -1).transformDirection(parentLg.matrixWorld).normalize()
  const cOriginWorldCorrect = new THREE.Vector3(0, 0, 0.050).applyMatrix4(childLg.matrixWorld)
  // BUGGY child origin — what would happen if we used bbox fallback (40mm)
  // instead of length_mm=100: child.top default = (0,0,+20mm).
  const cOriginWorldBuggy = new THREE.Vector3(0, 0, 0.020).applyMatrix4(childLg.matrixWorld)

  console.log(`pOriginWorld = (${pOriginWorld.x.toFixed(4)}, ${pOriginWorld.y.toFixed(4)}, ${pOriginWorld.z.toFixed(4)})`)
  console.log(`cOriginWorld CORRECT  (at +50mm in child) = (${cOriginWorldCorrect.x.toFixed(4)}, ${cOriginWorldCorrect.y.toFixed(4)}, ${cOriginWorldCorrect.z.toFixed(4)})`)
  console.log(`cOriginWorld BUGGY    (at +20mm in child) = (${cOriginWorldBuggy.x.toFixed(4)}, ${cOriginWorldBuggy.y.toFixed(4)}, ${cOriginWorldBuggy.z.toFixed(4)})`)
  console.log(`pAxisWorld = (${pAxisWorld.x.toFixed(3)}, ${pAxisWorld.y.toFixed(3)}, ${pAxisWorld.z.toFixed(3)})`)

  const pivotGroups = new Set([pivot])

  // CORRECT case — child origin at (0,0,+50)
  {
    const diag = { sampleCount: 0, parentHits: 0, childHits: 0, pairedCount: 0, faceRadiusM: 0, nudgeMm: 0, reason: '' }
    const nudgeM = nudgeAlongNormal(
      parentLg, childLg,
      [pOriginWorld.x, pOriginWorld.y, pOriginWorld.z],
      [pAxisWorld.x, pAxisWorld.y, pAxisWorld.z],
      [cOriginWorldCorrect.x, cOriginWorldCorrect.y, cOriginWorldCorrect.z],
      { excludeParent: pivotGroups, excludeChild: pivotGroups, diagnostics: diag },
    )
    console.log(`\nCORRECT cOrigin +50mm in child:`)
    console.log(`  nudge=${(nudgeM * 1000).toFixed(3)}mm  paired=${diag.pairedCount}/${diag.sampleCount}  pHit=${diag.parentHits}  cHit=${diag.childHits}`)
    console.log(`  gap(mm) min=${(diag.gapMinMm ?? 0).toFixed(2)} p50=${(diag.gapP50Mm ?? 0).toFixed(2)} p90=${(diag.gapP90Mm ?? 0).toFixed(2)} max=${(diag.gapMaxMm ?? 0).toFixed(2)}`)
    console.log(`  reason: ${diag.reason}`)
  }

  // BUGGY case — child origin at (0,0,+20) (fallback bbox)
  {
    const diag = { sampleCount: 0, parentHits: 0, childHits: 0, pairedCount: 0, faceRadiusM: 0, nudgeMm: 0, reason: '' }
    const nudgeM = nudgeAlongNormal(
      parentLg, childLg,
      [pOriginWorld.x, pOriginWorld.y, pOriginWorld.z],
      [pAxisWorld.x, pAxisWorld.y, pAxisWorld.z],
      [cOriginWorldBuggy.x, cOriginWorldBuggy.y, cOriginWorldBuggy.z],
      { excludeParent: pivotGroups, excludeChild: pivotGroups, diagnostics: diag },
    )
    console.log(`\nBUGGY cOrigin +20mm in child:`)
    console.log(`  nudge=${(nudgeM * 1000).toFixed(3)}mm  paired=${diag.pairedCount}/${diag.sampleCount}  pHit=${diag.parentHits}  cHit=${diag.childHits}`)
    console.log(`  gap(mm) min=${(diag.gapMinMm ?? 0).toFixed(2)} p50=${(diag.gapP50Mm ?? 0).toFixed(2)} p90=${(diag.gapP90Mm ?? 0).toFixed(2)} max=${(diag.gapMaxMm ?? 0).toFixed(2)}`)
    console.log(`  reason: ${diag.reason}`)
  }
}

main().catch(e => { console.error(e); process.exit(1) })
