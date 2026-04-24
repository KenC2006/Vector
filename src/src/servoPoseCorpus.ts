// Unit tests for the servoPose solver. Matches the canonical cases in
// docs/SERVO_SPLIT_IMPLEMENTATION.md §Step 2.2.
//
// Run: cd src && node --experimental-strip-types src/servoPoseCorpus.ts

import * as THREE from 'three'
import { solveServoBodyPose, pickBracket, type MountFaceLocal, type SolveInput } from './servoPose.ts'
import { rpyToQuat } from './rotationIO.ts'

interface Case {
  name: string
  parentFaceNormal: [number, number, number]
  desiredHornAxis: [number, number, number]
  expectedTiltDeg: number
  expectedMountFace: MountFaceLocal
  expectedFeasible: boolean
}

const CASES: Case[] = [
  {
    name: 'stacked up (top, z)',
    parentFaceNormal: [0, 0, 1],
    desiredHornAxis: [0, 0, 1],
    expectedTiltDeg: 0,
    expectedMountFace: 'bottom',
    expectedFeasible: true,
  },
  {
    name: 'stacked up, Y-axis (top, y)',
    parentFaceNormal: [0, 0, 1],
    desiredHornAxis: [0, 1, 0],
    expectedTiltDeg: 90,
    expectedMountFace: 'bottom',
    expectedFeasible: true,
  },
  {
    name: 'hip abduction (bottom, x)',
    parentFaceNormal: [0, 0, -1],
    desiredHornAxis: [1, 0, 0],
    expectedTiltDeg: 90,
    expectedMountFace: 'bottom',
    expectedFeasible: true,
  },
  {
    name: 'dangling straight (bottom, z)',
    parentFaceNormal: [0, 0, -1],
    desiredHornAxis: [0, 0, -1],
    expectedTiltDeg: 0,
    expectedMountFace: 'bottom',
    expectedFeasible: true,
  },
  {
    name: 'horn into parent (impossible direct)',
    parentFaceNormal: [0, 0, 1],
    desiredHornAxis: [0, 0, -1],
    expectedTiltDeg: 180,
    expectedMountFace: 'x_plus',
    expectedFeasible: false,
  },
]

const TOL = 1e-4
const fakeSplit = {
  split_link: {
    body_mass_frac: 0.9,
    output_origin_xyz_mm: [0, 0, 15] as [number, number, number],
    output_axis_xyz: [0, 0, 1] as [number, number, number],
    output_half_extents_mm: [6, 6, 2.6] as [number, number, number],
    bracket_tilt_threshold_deg: 45,
  },
}

function vec(v: [number, number, number]): THREE.Vector3 {
  return new THREE.Vector3(v[0], v[1], v[2])
}

function approxEq(a: number, b: number, tol = TOL): boolean {
  return Math.abs(a - b) < tol
}

function vecApproxEq(a: THREE.Vector3, b: THREE.Vector3, tol = TOL): boolean {
  return a.clone().sub(b).length() < tol
}

let fails = 0
let passed = 0

for (const c of CASES) {
  const input: SolveInput = {
    parentFaceNormalWorld: vec(c.parentFaceNormal),
    desiredHornAxisWorld: vec(c.desiredHornAxis),
    parentWorldQuat: new THREE.Quaternion(),
    preset: fakeSplit,
  }
  const out = solveServoBodyPose(input)

  const errs: string[] = []

  if (!approxEq(out.bodyTiltDeg, c.expectedTiltDeg, 1e-3)) {
    errs.push(`tilt ${out.bodyTiltDeg.toFixed(3)} ≠ ${c.expectedTiltDeg}`)
  }
  if (out.mountFaceLocal !== c.expectedMountFace) {
    errs.push(`mountFace ${out.mountFaceLocal} ≠ ${c.expectedMountFace}`)
  }
  if (out.feasible !== c.expectedFeasible) {
    errs.push(`feasible ${out.feasible} ≠ ${c.expectedFeasible}`)
  }

  // Apply the returned rpy (as a world quat since parent is identity here) to
  // +Z and verify it lands on the desired horn axis.
  const bodyQ = rpyToQuat(out.bodyLocalRpy)
  const hornApplied = new THREE.Vector3(0, 0, 1).applyQuaternion(bodyQ)
  if (!vecApproxEq(hornApplied, vec(c.desiredHornAxis))) {
    errs.push(
      `horn applied = (${hornApplied.x.toFixed(4)}, ${hornApplied.y.toFixed(4)}, ${hornApplied.z.toFixed(4)})` +
      ` ≠ expected (${c.desiredHornAxis.join(', ')})`,
    )
  }

  if (errs.length === 0) {
    passed += 1
    console.log(`  ✓ ${c.name}`)
  } else {
    fails += 1
    console.log(`  ✗ ${c.name}`)
    for (const e of errs) console.log(`      ${e}`)
  }
}

// ── Bracket picker spot checks ──────────────────────────────────────────────
interface BracketCase {
  tilt: number
  threshold: number
  expected: 'none' | 'structural_bracket_l' | 'structural_bracket_u'
}
const BRACKET_CASES: BracketCase[] = [
  { tilt: 0,   threshold: 45, expected: 'none' },
  { tilt: 30,  threshold: 45, expected: 'none' },
  { tilt: 45,  threshold: 45, expected: 'none' },
  { tilt: 46,  threshold: 45, expected: 'structural_bracket_l' },
  { tilt: 90,  threshold: 45, expected: 'structural_bracket_l' },
  { tilt: 134, threshold: 45, expected: 'structural_bracket_l' },
  { tilt: 135, threshold: 45, expected: 'structural_bracket_u' },
  { tilt: 180, threshold: 45, expected: 'structural_bracket_u' },
]

for (const b of BRACKET_CASES) {
  const got = pickBracket(b.tilt, b.threshold)
  if (got === b.expected) {
    passed += 1
    console.log(`  ✓ pickBracket(tilt=${b.tilt}, thr=${b.threshold}) = ${got}`)
  } else {
    fails += 1
    console.log(`  ✗ pickBracket(tilt=${b.tilt}, thr=${b.threshold}) = ${got}, expected ${b.expected}`)
  }
}

console.log(`\n${passed} passed, ${fails} failed`)
if (fails > 0) process.exit(1)
