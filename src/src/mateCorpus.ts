// Mate-connector fixture harness: exercises the pure resolver in
// mateConnectors.ts against hand-derived expected poses, including the
// "default-connector fastened mate == legacy bbox math" parity that
// makes the Phase 2 feature flag safe to ship on by default.
//
// Run: cd src && npm run test:mate-corpus
// Direct: node --experimental-strip-types src/src/mateCorpus.ts
//
// Why this file exists: Phase 1 of docs/MATE_CONNECTOR_MIGRATION.md swaps
// in a closed-form matrix composition for assembly placement. The whole
// migration is load-bearing on `resolveMate(default top, default bottom, fastened)`
// producing bit-identical output to computeFacePlacement(face='top'). If
// that parity ever breaks, every quadruped/rover/arm render drifts silently.
// This corpus fails loudly when it does.

import * as THREE from 'three'
import {
  generateDefaultConnectors,
  resolveMate,
  mergeConnectors,
  findConnector,
  buildInPlaneBasis,
  childConnectorIdForAttachFace,
  type MateConnector,
  type MateType,
  type MateParams,
  type ConnectorBoundingBoxMm,
} from './mateConnectors.ts'

// ── Helpers ────────────────────────────────────────────────────────────────

interface DecomposedPose {
  pos: THREE.Vector3
  quat: THREE.Quaternion
}

function decompose(m: THREE.Matrix4): DecomposedPose {
  const pos = new THREE.Vector3()
  const quat = new THREE.Quaternion()
  const scale = new THREE.Vector3()
  m.decompose(pos, quat, scale)
  return { pos, quat }
}

function approxEqualVec(a: THREE.Vector3, b: THREE.Vector3, tol = 1e-6): boolean {
  return Math.abs(a.x - b.x) <= tol && Math.abs(a.y - b.y) <= tol && Math.abs(a.z - b.z) <= tol
}

function approxEqualQuat(a: THREE.Quaternion, b: THREE.Quaternion, tol = 1e-6): boolean {
  // Quaternions q and -q represent the same rotation; check both.
  const pos = Math.abs(a.x - b.x) <= tol && Math.abs(a.y - b.y) <= tol && Math.abs(a.z - b.z) <= tol && Math.abs(a.w - b.w) <= tol
  const neg = Math.abs(a.x + b.x) <= tol && Math.abs(a.y + b.y) <= tol && Math.abs(a.z + b.z) <= tol && Math.abs(a.w + b.w) <= tol
  return pos || neg
}

function fmtVec(v: THREE.Vector3): string {
  return `(${v.x.toFixed(5)}, ${v.y.toFixed(5)}, ${v.z.toFixed(5)})`
}

function fmtQuat(q: THREE.Quaternion): string {
  return `[${q.x.toFixed(4)}, ${q.y.toFixed(4)}, ${q.z.toFixed(4)}, ${q.w.toFixed(4)}]`
}

// ── Legacy bbox reference (matches computeFacePlacement in urdfAssembly.ts) ─
// Mirror the closed-form pivot math the current engine emits for symmetric
// bbox components on a given attach_face. No rotation; pure translation.
// Parent origin offset (cx,cy,cz) is zero here — this is the symmetric case
// the default connectors must match exactly.

const ATTACH_FACE_OFFSET = {
  top:    (hpz: number, hcz: number) => new THREE.Vector3(0,     0,     hpz + hcz),
  bottom: (hpz: number, hcz: number) => new THREE.Vector3(0,     0,   -(hpz + hcz)),
  front:  (hpx: number, hcx: number) => new THREE.Vector3( hpx + hcx,  0, 0),
  back:   (hpx: number, hcx: number) => new THREE.Vector3(-(hpx + hcx), 0, 0),
  right:  (hpy: number, hcy: number) => new THREE.Vector3(0,     hpy + hcy,  0),
  left:   (hpy: number, hcy: number) => new THREE.Vector3(0,  -(hpy + hcy),  0),
} as const

function legacyBboxPose(
  parentBbox: ConnectorBoundingBoxMm,
  childBbox: ConnectorBoundingBoxMm,
  face: keyof typeof ATTACH_FACE_OFFSET,
): { pos: THREE.Vector3; quat: THREE.Quaternion } {
  const hp = face === 'top' || face === 'bottom' ? parentBbox.hzMm
           : face === 'front' || face === 'back' ? parentBbox.hxMm
           : parentBbox.hyMm
  const hc = face === 'top' || face === 'bottom' ? childBbox.hzMm
           : face === 'front' || face === 'back' ? childBbox.hxMm
           : childBbox.hyMm
  const offsetMm = ATTACH_FACE_OFFSET[face](hp, hc)
  return {
    pos: offsetMm.multiplyScalar(1 / 1000),
    quat: new THREE.Quaternion(), // identity
  }
}

// ── Fixtures ───────────────────────────────────────────────────────────────

interface Fixture {
  name: string
  setup: () => {
    parentWorld: THREE.Matrix4
    parentConn: MateConnector
    childConn: MateConnector
    mateType: MateType
    params?: MateParams
  }
  /** Pose we expect the resolver to produce (child-link-world, decomposed). */
  expected: { pos: THREE.Vector3; quat: THREE.Quaternion }
  tolerance?: number
}

const parentBbox: ConnectorBoundingBoxMm = { hxMm: 100, hyMm: 75, hzMm: 20 }
const childBbox:  ConnectorBoundingBoxMm = { hxMm: 15,  hyMm: 15, hzMm: 15 }
const pDefs = generateDefaultConnectors(parentBbox)
const cDefs = generateDefaultConnectors(childBbox)

function pickConn(list: MateConnector[], id: string): MateConnector {
  const c = findConnector(list, id)
  if (!c) throw new Error(`fixture setup: connector "${id}" missing`)
  return c
}

const fixtures: Fixture[] = [
  // ── 1-5: legacy-parity across all six face pairs (top/bottom/front/back/left/right) ──
  ...(['top', 'bottom', 'front', 'back', 'left', 'right'] as const).map((face): Fixture => ({
    name: `parity: fastened / default ${face}↔${childConnectorIdForAttachFace(face)} == legacy bbox math`,
    setup: () => ({
      parentWorld: new THREE.Matrix4(),
      parentConn: pickConn(pDefs, face),
      childConn:  pickConn(cDefs, childConnectorIdForAttachFace(face)!),
      mateType:   'fastened',
    }),
    expected: legacyBboxPose(parentBbox, childBbox, face),
  })),

  // ── 6: planar with in-plane offset — child translates on the face, no spin
  {
    name: 'planar: top↔bottom with offset_uv=[10,-5]mm translates child in plane, no rotation',
    setup: () => ({
      parentWorld: new THREE.Matrix4(),
      parentConn: pickConn(pDefs, 'top'),
      childConn:  pickConn(cDefs, 'bottom'),
      mateType:   'planar',
      params:     { offset_uv_mm: [10, -5] },
    }),
    expected: (() => {
      // For normal = +Z, buildInPlaneBasis picks helper = +X (or another
      // least-parallel axis), so we use the same function to derive the
      // expected displacement — mirroring the resolver's canonical basis
      // choice rather than hard-coding the X/Y assumption.
      const { u, v } = buildInPlaneBasis(new THREE.Vector3(0, 0, 1))
      const t = new THREE.Vector3()
        .addScaledVector(u, 0.010)
        .addScaledVector(v, -0.005)
      t.z += (parentBbox.hzMm + childBbox.hzMm) / 1000
      return { pos: t, quat: new THREE.Quaternion() }
    })(),
  },

  // ── 7: concentric on default top-bottom, offset=0 matches fastened ──
  {
    name: 'concentric: top↔bottom, zero offset, zero spin == fastened (parity)',
    setup: () => ({
      parentWorld: new THREE.Matrix4(),
      parentConn: pickConn(pDefs, 'top'),
      childConn:  pickConn(cDefs, 'bottom'),
      mateType:   'concentric',
      params:     { offset_mm: 0, rotation_rad: 0 },
    }),
    expected: legacyBboxPose(parentBbox, childBbox, 'top'),
  },

  // ── 8: concentric with axial offset (shaft-in-hole slide) ──
  {
    name: 'concentric: top↔bottom with offset_mm=5 shifts child 5mm along +Z (deeper out along parent axis)',
    setup: () => ({
      parentWorld: new THREE.Matrix4(),
      parentConn: pickConn(pDefs, 'top'),
      childConn:  pickConn(cDefs, 'bottom'),
      mateType:   'concentric',
      params:     { offset_mm: 5 },
    }),
    expected: {
      pos: new THREE.Vector3(0, 0, (parentBbox.hzMm + childBbox.hzMm + 5) / 1000),
      quat: new THREE.Quaternion(),
    },
  },

  // ── 9: concentric with axial spin (π/2 about +Z) ──
  {
    name: 'concentric: top↔bottom with rotation_rad=π/2 spins child about +Z (Z translation unchanged)',
    setup: () => ({
      parentWorld: new THREE.Matrix4(),
      parentConn: pickConn(pDefs, 'top'),
      childConn:  pickConn(cDefs, 'bottom'),
      mateType:   'concentric',
      params:     { rotation_rad: Math.PI / 2 },
    }),
    expected: {
      pos: new THREE.Vector3(0, 0, (parentBbox.hzMm + childBbox.hzMm) / 1000),
      quat: new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 0, 1), Math.PI / 2),
    },
  },

  // ── 10: servo shaft_out → coupler shaft_hole, concentric ──
  // Authored connectors from the worked example in docs/MATE_CONNECTOR_MIGRATION.md.
  {
    name: 'concentric: servo shaft_out → coupler shaft_hole (authored connectors)',
    setup: () => {
      const servoConn: MateConnector = {
        id: 'shaft_out', origin_xyz_mm: [0, 0, 13], axis_xyz: [0, 0, 1], type: 'cylindrical', diameter_mm: 6,
      }
      const couplerConn: MateConnector = {
        id: 'shaft_hole', origin_xyz_mm: [0, 0, 4], axis_xyz: [0, 0, 1], type: 'cylindrical', diameter_mm: 6,
      }
      return {
        parentWorld: new THREE.Matrix4(),
        parentConn: servoConn,
        childConn:  couplerConn,
        mateType:   'concentric',
        params:     {},
      }
    },
    // Servo axis = +Z, coupler axis = +Z → antiparallel mating requires a
    // 180° swing. Three.js's setFromUnitVectors picks the +Y axis for the
    // antiparallel-Z case (arbitrary but deterministic — see three/src/math/Quaternion.js).
    // Child's connector origin (0,0,4) rotated by Ry(π) lands at (0,0,-4).
    // Translation = (0,0,13) + 0 - (0,0,-4) = (0,0,17). The axial spin is a
    // free DOF of the concentric mate; either Rx(π) or Ry(π) would be a valid
    // pose (differing only by Rz — the free spin). We assert the one the
    // resolver actually produces so the fixture stays deterministic.
    expected: {
      pos: new THREE.Vector3(0, 0, 17 / 1000),
      quat: new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 1, 0), Math.PI),
    },
  },

  // ── 11: face-ambiguity on L-bracket mock ──
  // An L-bracket has TWO planar faces (plate top and wall inner) that the legacy
  // `attach_face: "top"` can't distinguish — both would resolve to "bbox top
  // face center." Named connectors resolve the ambiguity.
  {
    name: 'L-bracket mock: plate_top and wall_inner resolve to DIFFERENT poses',
    setup: () => ({
      parentWorld: new THREE.Matrix4(),
      // plate_top — at top of horizontal plate (z = +4), axis +Z
      parentConn: { id: 'plate_top',  origin_xyz_mm: [0, 0, 4], axis_xyz: [0, 0, 1], type: 'planar' },
      childConn:  pickConn(cDefs, 'bottom'),
      mateType:   'fastened',
    }),
    expected: {
      // child sits on z=+0.004 with its bottom face at -0.015 → pivot.z = 0.019
      pos: new THREE.Vector3(0, 0, (4 + childBbox.hzMm) / 1000),
      quat: new THREE.Quaternion(),
    },
  },
  {
    name: 'L-bracket mock: wall_inner (perpendicular face) sits at x=20, axes along +X',
    setup: () => ({
      parentWorld: new THREE.Matrix4(),
      // wall_inner — at the inward face of the L's vertical wall (x = -20), axis -X
      parentConn: { id: 'wall_inner', origin_xyz_mm: [-20, 0, 15], axis_xyz: [-1, 0, 0], type: 'planar' },
      childConn:  pickConn(cDefs, 'front'),
      mateType:   'fastened',
    }),
    expected: {
      // Parent axis -X and child axis +X are ALREADY antiparallel, so qAlign
      // is identity — no flip needed. Parent's wall_inner face sits at
      // x=-0.020 in parent-local; the child's "front" face in child-local is
      // at +x=0.015, so placing the child at x=-0.035 puts its +X face flush
      // against the wall_inner face. (Key insight: wall_inner encodes an
      // INWARD-pointing axis; without the axis disambiguation, an attach_face:"back"
      // on a bbox'd L-bracket would land the child at the wrong plane.)
      pos: new THREE.Vector3((-20 - childBbox.hxMm) / 1000, 0, 15 / 1000),
      quat: new THREE.Quaternion(),
    },
  },

  // ── 12: backward-compat full-sweep parity with a non-identity parentWorld ──
  // Ensures the resolver composes T_p correctly (not just accidentally
  // ignoring it). We place the parent at (0.05, 0.10, 0.25) and confirm the
  // child world pose is parent world + legacy offset on each of 6 faces.
  ...(['top', 'bottom', 'front', 'back', 'left', 'right'] as const).map((face): Fixture => {
    const parentT = new THREE.Matrix4().makeTranslation(0.05, 0.10, 0.25)
    const legacy = legacyBboxPose(parentBbox, childBbox, face)
    return {
      name: `parity (translated parent): fastened / ${face}↔${childConnectorIdForAttachFace(face)}`,
      setup: () => ({
        parentWorld: parentT.clone(),
        parentConn: pickConn(pDefs, face),
        childConn:  pickConn(cDefs, childConnectorIdForAttachFace(face)!),
        mateType:   'fastened',
      }),
      expected: {
        pos: legacy.pos.clone().add(new THREE.Vector3(0.05, 0.10, 0.25)),
        quat: legacy.quat.clone(),
      },
    }
  }),

  // ── 13a: servo shaft_out (real preset dims) → coupler shaft_hole, concentric ──
  // Mirrors the connector dims authored on actuator_servo_high_torque
  // (origin_xyz_mm=[0,0,17], d=8) mated to a shaft_hole connector at z=4
  // (coupler bbox hz=4). Protects against silent drift between preset data
  // and the resolver's output as the catalog grows.
  {
    name: 'concentric: actuator_servo_high_torque.shaft_out (z=17, d=8) → coupler shaft_hole (z=4, d=8)',
    setup: () => ({
      parentWorld: new THREE.Matrix4(),
      parentConn: { id: 'shaft_out',  origin_xyz_mm: [0, 0, 17], axis_xyz: [0, 0, 1], type: 'cylindrical', diameter_mm: 8 } as MateConnector,
      childConn:  { id: 'shaft_hole', origin_xyz_mm: [0, 0,  4], axis_xyz: [0, 0, 1], type: 'cylindrical', diameter_mm: 8 } as MateConnector,
      mateType:   'concentric',
      params:     {},
    }),
    // Both axes +Z → antiparallel mating picks Ry(π). Child origin (0,0,4)
    // under Ry(π) lands at (0,0,-4). t_local = (0,0,17) + 0 - (0,0,-4) = (0,0,21).
    expected: {
      pos: new THREE.Vector3(0, 0, 21 / 1000),
      quat: new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 1, 0), Math.PI),
    },
  },

  // ── 13b: servo shaft_out (micro) → coupler shaft_hole, smaller scale ──
  {
    name: 'concentric: actuator_servo_micro.shaft_out (z=14.5, d=4.6) → coupler shaft_hole (z=4, d=4.6)',
    setup: () => ({
      parentWorld: new THREE.Matrix4(),
      parentConn: { id: 'shaft_out',  origin_xyz_mm: [0, 0, 14.5], axis_xyz: [0, 0, 1], type: 'cylindrical', diameter_mm: 4.6 } as MateConnector,
      childConn:  { id: 'shaft_hole', origin_xyz_mm: [0, 0,  4],   axis_xyz: [0, 0, 1], type: 'cylindrical', diameter_mm: 4.6 } as MateConnector,
      mateType:   'concentric',
      params:     {},
    }),
    // t_local = (0,0,14.5) + 0 - (0,0,-4) = (0,0,18.5)
    expected: {
      pos: new THREE.Vector3(0, 0, 18.5 / 1000),
      quat: new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 1, 0), Math.PI),
    },
  },

  // ── 13c: servo.shaft_out → coupler.shaft_hole with axial offset ──
  // Proves offset_mm is honored in the concentric path with realistic
  // coupler.shaft_hole dims. Simulates the coupler pushed 2mm further
  // along the servo shaft axis (occasionally needed when a user wants
  // clearance between the servo can and the coupler face).
  {
    name: 'concentric: servo_high_torque.shaft_out → coupler.shaft_hole with offset_mm=2',
    setup: () => ({
      parentWorld: new THREE.Matrix4(),
      parentConn: { id: 'shaft_out',  origin_xyz_mm: [0, 0, 17], axis_xyz: [0, 0, 1], type: 'cylindrical', diameter_mm: 8 } as MateConnector,
      childConn:  { id: 'shaft_hole', origin_xyz_mm: [0, 0,  4], axis_xyz: [0, 0, 1], type: 'cylindrical', diameter_mm: 8 } as MateConnector,
      mateType:   'concentric',
      params:     { offset_mm: 2 },
    }),
    // t_local = (0,0,17) + (0,0,2) - (0,0,-4) = (0,0,23mm)
    expected: {
      pos: new THREE.Vector3(0, 0, 23 / 1000),
      quat: new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 1, 0), Math.PI),
    },
  },

  // ── 13d: coupler.bottom_face as a parent connector ──
  // Coupler hosts a child bracket/plate on its bolt-ring side. bottom_face is
  // a semantic alias for the default bottom at the same pose; this fixture
  // confirms the authored connector lookup resolves it to the identical pose
  // so readable mate code stays bit-identical to legacy attach_face: "bottom".
  {
    name: 'fastened: coupler.bottom_face (authored) → child.top default resolves to (0, 0, -(4+hcz))',
    setup: () => ({
      parentWorld: new THREE.Matrix4(),
      parentConn: { id: 'bottom_face', origin_xyz_mm: [0, 0, -4], axis_xyz: [0, 0, -1], type: 'planar' } as MateConnector,
      childConn:  pickConn(cDefs, 'top'),
      mateType:   'fastened',
    }),
    // Parent axis -Z, child axis +Z — antiparallel already, qAlign=identity.
    // c_origin (0,0,hcz) under identity: unchanged.
    // t_local = (0,0,-4) + 0 - (0,0,hcz) = (0, 0, -(4+hcz))
    expected: {
      pos: new THREE.Vector3(0, 0, -(4 + childBbox.hzMm) / 1000),
      quat: new THREE.Quaternion(),
    },
  },

  // ── 13e: camera mount_back → bracket.top (fastened) — M2 fix case ──
  // Validates that authoring mount_back at (-45,0,0)/-X on the camera,
  // then fastening to a bracket top face, places the camera so its body
  // +X axis (lens / optical_front) points along world +Z — outward from
  // the bracket face. This is the M2-known-issues fix: previously the
  // camera's lens faced sideways (+Y) due to rpy preset rotation baked
  // into visuals; with a mount_back connector carrying the right axis,
  // resolveMate derives the correct 90° rotation (Qy(-π/2)) instead.
  {
    name: 'fastened: camera.mount_back (authored, -X) → bracket.top default (+Z) — lens points +Z',
    setup: () => ({
      parentWorld: new THREE.Matrix4(),
      parentConn: pickConn(pDefs, 'top'),
      childConn:  { id: 'mount_back', origin_xyz_mm: [-45, 0, 0], axis_xyz: [-1, 0, 0], type: 'planar' } as MateConnector,
      mateType:   'fastened',
    }),
    // qAlign(-X → -Z) = Qy(-π/2). Under that, c_origin (-45,0,0) → (0,0,-45).
    // t_local = (0,0,hpz) + 0 - (0,0,-45) = (0, 0, hpz + 45).
    // parentBbox.hzMm = 20 → t=(0,0,65mm).
    expected: {
      pos: new THREE.Vector3(0, 0, (parentBbox.hzMm + 45) / 1000),
      quat: new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 1, 0), -Math.PI / 2),
    },
  },

  // ── 13f: camera mount_back → bracket.front (fastened) — no rotation path ──
  // Parent axis +X and child axis -X already antiparallel → identity
  // rotation. Validates the authored connector works in the face-aligned
  // case without accidentally inducing a spin. Lens ends up pointing +X
  // world (out from bracket's front face).
  {
    name: 'fastened: camera.mount_back → bracket.front default (+X) — identity rotation',
    setup: () => ({
      parentWorld: new THREE.Matrix4(),
      parentConn: pickConn(pDefs, 'front'),
      childConn:  { id: 'mount_back', origin_xyz_mm: [-45, 0, 0], axis_xyz: [-1, 0, 0], type: 'planar' } as MateConnector,
      mateType:   'fastened',
    }),
    // Axes -X and +X already antiparallel → qAlign = identity.
    // c_origin unchanged: (-45, 0, 0).
    // t_local = (hpx, 0, 0) + 0 - (-45, 0, 0) = (hpx + 45, 0, 0).
    // parentBbox.hxMm = 100 → t=(145mm, 0, 0).
    expected: {
      pos: new THREE.Vector3((parentBbox.hxMm + 45) / 1000, 0, 0),
      quat: new THREE.Quaternion(),
    },
  },

  // ── 14: authored connectors override defaults of the same name ──
  {
    name: 'mergeConnectors: authored "top" overrides default "top"',
    setup: () => {
      const authoredTop: MateConnector = {
        id: 'top', origin_xyz_mm: [0, 0, 30], axis_xyz: [0, 0, 1], type: 'planar',
      }
      const merged = mergeConnectors(pDefs, [authoredTop])
      const pTop = findConnector(merged, 'top')!
      return {
        parentWorld: new THREE.Matrix4(),
        parentConn: pTop,
        childConn:  pickConn(cDefs, 'bottom'),
        mateType:   'fastened',
      }
    },
    // Authored top sits 30mm above parent origin (vs default 20mm), so child
    // bottom at z=-hcz → pivot.z = 30 + hcz.
    expected: {
      pos: new THREE.Vector3(0, 0, (30 + childBbox.hzMm) / 1000),
      quat: new THREE.Quaternion(),
    },
  },
]

// ── Runner ─────────────────────────────────────────────────────────────────

interface Outcome { name: string; ok: boolean; reason?: string }

function runFixture(f: Fixture): Outcome {
  const s = f.setup()
  const world = resolveMate(s.parentWorld, s.parentConn, s.childConn, s.mateType, s.params)
  const got = decompose(world)
  const tol = f.tolerance ?? 1e-6
  if (!approxEqualVec(got.pos, f.expected.pos, tol)) {
    return { name: f.name, ok: false, reason: `pos mismatch: expected ${fmtVec(f.expected.pos)}, got ${fmtVec(got.pos)}` }
  }
  if (!approxEqualQuat(got.quat, f.expected.quat, tol)) {
    return { name: f.name, ok: false, reason: `quat mismatch: expected ${fmtQuat(f.expected.quat)}, got ${fmtQuat(got.quat)}` }
  }
  return { name: f.name, ok: true }
}

function main(): void {
  const outcomes = fixtures.map(runFixture)
  let passed = 0, failed = 0
  for (const o of outcomes) {
    if (o.ok) { console.log(`  ✓ ${o.name}`); passed++ }
    else       { console.log(`  ✗ ${o.name}`); console.log(`      ${o.reason}`); failed++ }
  }
  console.log(`\n[mate-corpus] ${passed}/${outcomes.length} passed`)
  if (failed > 0) process.exit(1)
}

main()
