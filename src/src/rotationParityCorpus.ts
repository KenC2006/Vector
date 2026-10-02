// Phase 5b — frontend half of the rotation parity contract.
//
// Loads the frozen corpus at `scripts/rotation-corpus.json` and confirms that
// `rpyToQuat` (the only TS rpy→quaternion path) matches the corpus values
// exactly. The Python `_rpy_to_quat` in `core/sim/urdf_to_mjcf.py` is verified
// against the same corpus, so any drift between the two converters fails this
// test on at least one side.

import { readFileSync } from 'node:fs'
import { resolve as resolvePath, dirname } from 'node:path'
import { fileURLToPath } from 'node:url'
import * as THREE from 'three'

import { rpyToQuat } from './rotationIO.ts'
import { rpyToMatrix } from './design/math.ts'
import { urdfFrameToScene, sceneFrameToUrdf } from './coordinates.ts'

type Case = {
  name: string
  rpy: [number, number, number]
  quat_wxyz: [number, number, number, number]
  matrix_row_major: number[][]
}
type Corpus = { tolerance: number; cases: Case[] }

const __dirname = dirname(fileURLToPath(import.meta.url))
const corpusPath = resolvePath(__dirname, '..', '..', 'scripts', 'rotation-corpus.json')
const corpus = JSON.parse(readFileSync(corpusPath, 'utf8')) as Corpus

let failed = 0
for (const c of corpus.cases) {
  const q = rpyToQuat(c.rpy)
  // three.js stores [x,y,z,w]; corpus stores [w,x,y,z]. Compare with sign
  // ambiguity (q and -q are the same rotation).
  const got: [number, number, number, number] = [q.w, q.x, q.y, q.z]
  const exp = c.quat_wxyz
  const sign = (got[0] * exp[0] + got[1] * exp[1] + got[2] * exp[2] + got[3] * exp[3]) < 0 ? -1 : 1
  let qErr = 0
  for (let i = 0; i < 4; i++) qErr = Math.max(qErr, Math.abs(sign * got[i] - exp[i]))

  // Matrix check: build from quaternion and compare element-wise.
  const m = new THREE.Matrix4().makeRotationFromQuaternion(q).elements
  // three.js Matrix4.elements is column-major.
  const got3 = [
    [m[0], m[4], m[8]],
    [m[1], m[5], m[9]],
    [m[2], m[6], m[10]],
  ]
  let mErr = 0
  for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) {
    mErr = Math.max(mErr, Math.abs(got3[i][j] - c.matrix_row_major[i][j]))
  }

  if (qErr > corpus.tolerance || mErr > corpus.tolerance) {
    failed++
    console.error(`  FAIL  ${c.name}: qErr=${qErr.toExponential(2)} mErr=${mErr.toExponential(2)}`)
  } else {
    console.log(`  PASS  ${c.name}`)
  }
}

console.log(`\nrotation-parity (TS): ${corpus.cases.length - failed}/${corpus.cases.length} passed`)

// ── Phase 0 helper checks ────────────────────────────────────────────────
//
// The design editor's rotation math against the frozen corpus, and the
// `coordinates.ts` Z-up/Y-up adapter against closed-form expectations.

let helperFailed = 0
const TOL = 1e-9

function approxVec(name: string, got: readonly number[], exp: readonly number[]): void {
  let err = 0
  for (let i = 0; i < got.length; i++) err = Math.max(err, Math.abs(got[i] - exp[i]))
  if (err > 1e-6) {
    helperFailed++
    console.error(`  FAIL  ${name}: got=${JSON.stringify(got)} exp=${JSON.stringify(exp)} err=${err.toExponential(2)}`)
  } else {
    console.log(`  PASS  ${name}`)
  }
}

// design/math.ts rpyToMatrix (the placement editor's port of the Python
// compiler) must reproduce the frozen corpus matrices exactly.
for (const c of corpus.cases) {
  const R = rpyToMatrix(c.rpy[0], c.rpy[1], c.rpy[2])
  approxVec(`design rpyToMatrix ${c.name}`, R.flat(), c.matrix_row_major.flat())
}

// coordinates round-trip — a representative pose must survive
// urdf → scene → urdf with bit-near identity.
{
  const xyzMm: [number, number, number] = [123, -45, 678]
  const rpy: [number, number, number] = [0.3, -0.7, 1.2]
  const { position, quaternion } = urdfFrameToScene(xyzMm, rpy)
  const back = sceneFrameToUrdf(position, quaternion)
  approxVec('coordinates round-trip xyz', back.xyzMm, xyzMm)
  // RPY can land on an equivalent representation; compare via quaternion.
  const qBack = rpyToQuat(back.rpy)
  const qOrig = rpyToQuat(rpy)
  const dot = qBack.x * qOrig.x + qBack.y * qOrig.y + qBack.z * qOrig.z + qBack.w * qOrig.w
  if (Math.abs(Math.abs(dot) - 1) > 1e-6) {
    helperFailed++
    console.error(`  FAIL  coordinates round-trip rpy: dot=${dot}`)
  } else {
    console.log('  PASS  coordinates round-trip rpy')
  }
}

// urdf Z-up → scene Y-up: the URDF +Z vector must land on scene +Y.
{
  const { position } = urdfFrameToScene([0, 0, 1000], [0, 0, 0])
  approxVec('coordinates URDF +Z → scene +Y', [position.x, position.y, position.z], [0, 1, 0])
}

console.log(`\nrotation-parity helpers: ${helperFailed === 0 ? 'all passed' : helperFailed + ' failed'}`)
if (failed > 0 || helperFailed > 0) process.exit(1)
void TOL
