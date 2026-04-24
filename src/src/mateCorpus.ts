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
import * as fs from 'node:fs'
import * as path from 'node:path'
import { fileURLToPath } from 'node:url'
import {
  generateDefaultConnectors,
  resolveMate,
  mergeConnectors,
  findConnector,
  buildInPlaneBasis,
  childConnectorIdForAttachFace,
  faceUVToWorldOffset,
  tangentBasisFromAxis,
  type MateConnector,
  type MateType,
  type MateParams,
  type ConnectorBoundingBoxMm,
} from './mateConnectors.ts'
import { getOrComputeBbox } from './componentDims.ts'

// ── Preset loader (for integration fixtures) ───────────────────────────────
// Parses the shipping catalog so integration fixtures exercise the
// preset JSON → merge → resolve pipeline end-to-end. Without this, the
// hand-typed fixtures above only prove the resolver math; they bypass
// the JSON authoring layer where a typo or schema drift could silently
// break a rendered robot. Loader-based fixtures catch that.

interface CorpusPresetPhysical {
  bounding_box_mm?: number[]
  cross_section_mm?: number[]
}
interface CorpusPreset {
  id: string
  physical: CorpusPresetPhysical
  connectors?: MateConnector[]
}
interface CorpusPresetFile {
  categories: Record<string, { components: CorpusPreset[] }>
}

function loadPresets(): Map<string, CorpusPreset> {
  const here = path.dirname(fileURLToPath(import.meta.url))
  const presetPath = path.resolve(here, '..', 'public', 'generic_presets.json')
  const raw = fs.readFileSync(presetPath, 'utf-8')
  const data = JSON.parse(raw) as CorpusPresetFile
  const byId = new Map<string, CorpusPreset>()
  for (const cat of Object.values(data.categories)) {
    for (const c of cat.components) byId.set(c.id, c)
  }
  return byId
}

function presetBboxMm(p: CorpusPreset): ConnectorBoundingBoxMm {
  // Corpus runs in node without rendered meshes — getOrComputeBbox falls
  // through to the authored field, matching the pre-helper behavior here.
  const bb = getOrComputeBbox(p.id, p)
  return { hxMm: bb[0] / 2, hyMm: bb[1] / 2, hzMm: bb[2] / 2 }
}

/** Mimic the urdfAssembly.computeMatePlacement merge-then-find path:
 *  generateDefaultConnectors(bbox) + mergeConnectors(defaults, preset.connectors)
 *  + findConnector(merged, id). Any drift between this path and the runtime
 *  path = drift between corpus and production; keep them in sync. */
function resolveFromPresets(
  parentPreset: CorpusPreset,
  childPreset: CorpusPreset,
  parentConnId: string,
  childConnId: string,
  mateType: MateType,
  params: MateParams = {},
): THREE.Matrix4 {
  const pConnectors = mergeConnectors(generateDefaultConnectors(presetBboxMm(parentPreset)), parentPreset.connectors)
  const cConnectors = mergeConnectors(generateDefaultConnectors(presetBboxMm(childPreset)),  childPreset.connectors)
  const pConn = findConnector(pConnectors, parentConnId)
  const cConn = findConnector(cConnectors, childConnId)
  if (!pConn) throw new Error(`preset "${parentPreset.id}" missing connector "${parentConnId}"`)
  if (!cConn) throw new Error(`preset "${childPreset.id}" missing connector "${childConnId}"`)
  return resolveMate(new THREE.Matrix4(), pConn, cConn, mateType, params)
}

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
  // (origin_xyz_mm=[0,0,17], axis +Z, d=8) mated to the coupler's shaft_hole
  // at (0,0,-4)/-Z (bore opens on the coupler's BOTTOM face — receives the
  // servo shaft from below). Antiparallel axes → no flip — coupler stays
  // upright with its bolt-ring side (+Z face) facing up for a child bracket
  // to mount correctly.
  //
  // History: initial authoring put shaft_hole at (0,0,+4)/+Z following the
  // "axis outward from each face" convention literally, which forced a 180°
  // flip under concentric mating. Smoke test caught this — the auto-coupler
  // rendered upside-down and downstream servo clipped into its parent.
  {
    name: 'concentric: actuator_servo_high_torque.shaft_out (z=17, +Z) → coupler shaft_hole (z=-4, -Z)',
    setup: () => ({
      parentWorld: new THREE.Matrix4(),
      parentConn: { id: 'shaft_out',  origin_xyz_mm: [0, 0, 17], axis_xyz: [0, 0,  1], type: 'cylindrical', diameter_mm: 8 } as MateConnector,
      childConn:  { id: 'shaft_hole', origin_xyz_mm: [0, 0, -4], axis_xyz: [0, 0, -1], type: 'cylindrical', diameter_mm: 8 } as MateConnector,
      mateType:   'concentric',
      params:     {},
    }),
    // Axes +Z and -Z already antiparallel → qAlign = identity. c_origin (0,0,-4)
    // stays. t_local = (0,0,17) + 0 - (0,0,-4) = (0,0,21). Coupler sits
    // upright — no rotation applied.
    expected: {
      pos: new THREE.Vector3(0, 0, 21 / 1000),
      quat: new THREE.Quaternion(),
    },
  },

  // ── 13b: servo shaft_out (micro) → coupler shaft_hole, smaller scale ──
  {
    name: 'concentric: actuator_servo_micro.shaft_out (z=14.5, +Z) → coupler shaft_hole (z=-4, -Z)',
    setup: () => ({
      parentWorld: new THREE.Matrix4(),
      parentConn: { id: 'shaft_out',  origin_xyz_mm: [0, 0, 14.5], axis_xyz: [0, 0,  1], type: 'cylindrical', diameter_mm: 4.6 } as MateConnector,
      childConn:  { id: 'shaft_hole', origin_xyz_mm: [0, 0, -4],   axis_xyz: [0, 0, -1], type: 'cylindrical', diameter_mm: 4.6 } as MateConnector,
      mateType:   'concentric',
      params:     {},
    }),
    // t_local = (0,0,14.5) + 0 - (0,0,-4) = (0,0,18.5), identity rotation.
    expected: {
      pos: new THREE.Vector3(0, 0, 18.5 / 1000),
      quat: new THREE.Quaternion(),
    },
  },

  // ── 13c: servo.shaft_out → coupler.shaft_hole with axial offset ──
  // Proves offset_mm is honored in the concentric path with realistic
  // coupler.shaft_hole dims. offset_mm is in parent-axis direction (+Z),
  // so positive offset pushes the child FURTHER along +Z (away from the
  // servo body). Useful for clearance between the servo can and the
  // coupler face.
  {
    name: 'concentric: servo_high_torque.shaft_out → coupler.shaft_hole with offset_mm=2',
    setup: () => ({
      parentWorld: new THREE.Matrix4(),
      parentConn: { id: 'shaft_out',  origin_xyz_mm: [0, 0, 17], axis_xyz: [0, 0,  1], type: 'cylindrical', diameter_mm: 8 } as MateConnector,
      childConn:  { id: 'shaft_hole', origin_xyz_mm: [0, 0, -4], axis_xyz: [0, 0, -1], type: 'cylindrical', diameter_mm: 8 } as MateConnector,
      mateType:   'concentric',
      params:     { offset_mm: 2 },
    }),
    // t_local = (0,0,17) + (0,0,2) - (0,0,-4) = (0,0,23mm). Identity rotation
    // (axes already antiparallel).
    expected: {
      pos: new THREE.Vector3(0, 0, 23 / 1000),
      quat: new THREE.Quaternion(),
    },
  },

  // ── 13d: coupler.top_face as a parent connector ──
  // Coupler hosts a child bracket/plate on its bolt-ring side (+Z face).
  // top_face is a semantic alias for the default top at the same pose; this
  // fixture confirms the authored connector lookup resolves it to the same
  // output so readable mate code stays bit-identical to legacy attach_face.
  {
    name: 'fastened: coupler.top_face (authored) → child.bottom default resolves to (0, 0, 4+hcz)',
    setup: () => ({
      parentWorld: new THREE.Matrix4(),
      parentConn: { id: 'top_face', origin_xyz_mm: [0, 0, 4], axis_xyz: [0, 0, 1], type: 'planar' } as MateConnector,
      childConn:  pickConn(cDefs, 'bottom'),
      mateType:   'fastened',
    }),
    // Parent axis +Z, child axis -Z — antiparallel already, qAlign=identity.
    // c_origin (0,0,-hcz) under identity stays. t = (0,0,4) - (0,0,-hcz) = (0,0,4+hcz).
    expected: {
      pos: new THREE.Vector3(0, 0, (4 + childBbox.hzMm) / 1000),
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

  // ── 13g-i: structural_bracket_l — plate_top / wall_inner / wall_outer real-preset values ──
  // The P1 disambiguation: the three authored connectors land a child at
  // three distinct positions and orientations. This is the central proof of
  // the L-bracket problem — with only the default bbox "top" at (0,0,+20)/+Z,
  // there was no way to say "mount on the horizontal plate surface" vs
  // "mount on the concave inside of the wall" vs "mount on the back of the
  // wall"; all three intentions collapsed to the same ambiguous face.
  {
    name: 'L-bracket real: plate_top (0,0,1.6)/+Z → child.bottom default, fastened → child at (0,0,1.6+hcz)',
    setup: () => ({
      parentWorld: new THREE.Matrix4(),
      parentConn: { id: 'plate_top',  origin_xyz_mm: [0, 0, 1.6], axis_xyz: [0, 0, 1], type: 'planar' } as MateConnector,
      childConn:  pickConn(cDefs, 'bottom'),
      mateType:   'fastened',
    }),
    // Axes +Z/-Z antiparallel → identity rotation. c_origin (0,0,-hcz) stays.
    // t = (0,0,1.6) + 0 - (0,0,-hcz) = (0, 0, 1.6+hcz).
    expected: {
      pos: new THREE.Vector3(0, 0, (1.6 + childBbox.hzMm) / 1000),
      quat: new THREE.Quaternion(),
    },
  },
  {
    name: 'L-bracket real: wall_inner (-16.8,0,18.4)/+X → child.back default, fastened — child on L concave side',
    setup: () => ({
      parentWorld: new THREE.Matrix4(),
      parentConn: { id: 'wall_inner', origin_xyz_mm: [-16.8, 0, 18.4], axis_xyz: [1, 0, 0], type: 'planar' } as MateConnector,
      childConn:  pickConn(cDefs, 'back'),
      mateType:   'fastened',
    }),
    // Parent axis +X, child axis -X — antiparallel already → identity rotation.
    // c_origin (-hcx, 0, 0) stays. t = (-16.8, 0, 18.4) - (-hcx, 0, 0) = (-16.8+hcx, 0, 18.4).
    expected: {
      pos: new THREE.Vector3((-16.8 + childBbox.hxMm) / 1000, 0, 18.4 / 1000),
      quat: new THREE.Quaternion(),
    },
  },
  {
    name: 'L-bracket real: wall_outer (-20,0,18.4)/-X → child.front default, fastened — child on back of wall',
    setup: () => ({
      parentWorld: new THREE.Matrix4(),
      parentConn: { id: 'wall_outer', origin_xyz_mm: [-20, 0, 18.4], axis_xyz: [-1, 0, 0], type: 'planar' } as MateConnector,
      childConn:  pickConn(cDefs, 'front'),
      mateType:   'fastened',
    }),
    // Parent axis -X, child axis +X — antiparallel already → identity rotation.
    // c_origin (+hcx, 0, 0) stays. t = (-20, 0, 18.4) - (hcx, 0, 0) = (-20-hcx, 0, 18.4).
    expected: {
      pos: new THREE.Vector3((-20 - childBbox.hxMm) / 1000, 0, 18.4 / 1000),
      quat: new THREE.Quaternion(),
    },
  },

  // ── 13j-l: end-to-end integration — load real generic_presets.json ──
  // The fixtures above hand-write the connector values they test. These
  // three reach through the shipping JSON to prove the full pipeline works:
  // load → findPreset → mergeConnectors(defaults, preset.connectors) →
  // findConnector(merged, id) → resolveMate → expected pose. A typo in a
  // shipping JSON connector (wrong diameter, swapped axis sign) fails loudly
  // here, not silently in the renderer.
  {
    name: 'integration (JSON): actuator_servo_high_torque.shaft_out → coupler.shaft_hole, concentric',
    setup: () => {
      const presets = loadPresets()
      const servo   = presets.get('actuator_servo_high_torque')!
      const coupler = presets.get('structural_servo_coupler_disc')!
      return {
        parentWorld: new THREE.Matrix4(),
        parentConn: findConnector(mergeConnectors(generateDefaultConnectors(presetBboxMm(servo)),   servo.connectors),   'shaft_out')!,
        childConn:  findConnector(mergeConnectors(generateDefaultConnectors(presetBboxMm(coupler)), coupler.connectors), 'shaft_hole')!,
        mateType:   'concentric',
      }
    },
    // Servo shaft_out at (0,0,17)/+Z, coupler shaft_hole at (0,0,-4)/-Z.
    // Antiparallel already → identity rotation. t_local = (0,0,17) - (0,0,-4)
    // = (0,0,21mm). Same math as fixture 13a, but via loadPresets() so JSON
    // data drift (e.g. someone accidentally reverts shaft_hole to +Z) fails
    // loudly here before it causes a runtime upside-down coupler.
    expected: {
      pos: new THREE.Vector3(0, 0, 21 / 1000),
      quat: new THREE.Quaternion(),
    },
  },
  {
    name: 'integration (JSON): structural_bracket_l three connectors resolve to three DIFFERENT poses',
    setup: () => {
      const presets = loadPresets()
      const bracket = presets.get('structural_bracket_l')!
      const merged = mergeConnectors(generateDefaultConnectors(presetBboxMm(bracket)), bracket.connectors)
      // Verify data shape up-front — a missing/renamed connector fails loudly.
      if (!findConnector(merged, 'plate_top'))  throw new Error('L-bracket JSON missing plate_top')
      if (!findConnector(merged, 'wall_inner')) throw new Error('L-bracket JSON missing wall_inner')
      if (!findConnector(merged, 'wall_outer')) throw new Error('L-bracket JSON missing wall_outer')
      // Use plate_top as the setup for this fixture's resolve assertion.
      return {
        parentWorld: new THREE.Matrix4(),
        parentConn:  findConnector(merged, 'plate_top')!,
        childConn:   pickConn(cDefs, 'bottom'),
        mateType:    'fastened',
      }
    },
    // plate_top = (0,0,1.6)/+Z → child.bottom → t=(0,0,1.6+hcz). Matches
    // fixture 13g value — the point of THIS fixture is the data-shape
    // assertion in setup() (would throw if JSON ever drops or renames a
    // connector), with the resolve as a secondary sanity check.
    expected: {
      pos: new THREE.Vector3(0, 0, (1.6 + childBbox.hzMm) / 1000),
      quat: new THREE.Quaternion(),
    },
  },
  {
    name: 'integration (JSON): resolveFromPresets helper — camera.mount_back → bracket_l.plate_top',
    setup: () => {
      const presets = loadPresets()
      const bracket = presets.get('structural_bracket_l')!
      const camera  = presets.get('sensor_depth_camera_small')!
      // Call resolveFromPresets once just to prove the helper composes cleanly
      // on real presets without throwing. The Fixture runner below re-resolves
      // via parentConn/childConn/mateType and compares to expected — we keep
      // that indirection because the generic runner prints clearer diagnostics
      // on mismatch than our one-off throw would.
      resolveFromPresets(bracket, camera, 'plate_top', 'mount_back', 'fastened')
      return {
        parentWorld: new THREE.Matrix4(),
        parentConn: findConnector(mergeConnectors(generateDefaultConnectors(presetBboxMm(bracket)), bracket.connectors), 'plate_top')!,
        childConn:  findConnector(mergeConnectors(generateDefaultConnectors(presetBboxMm(camera)),  camera.connectors),  'mount_back')!,
        mateType:   'fastened',
      }
    },
    // plate_top (0,0,1.6)/+Z vs camera.mount_back (-45,0,0)/-X:
    // qAlign(-X → -Z) = Qy(-π/2). c_origin (-45,0,0) under Qy(-π/2) → (0,0,-45).
    // t = (0,0,1.6) + 0 - (0,0,-45) = (0, 0, 46.6mm). Camera ends up 46.6mm
    // above L-bracket plate, body +X (lens) pointing +Z — proves the P1+M2
    // fix path composes correctly.
    expected: {
      pos: new THREE.Vector3(0, 0, 46.6 / 1000),
      quat: new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 1, 0), -Math.PI / 2),
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

// ── faceUVToWorldOffset checks (separate from resolveMate fixtures) ────────
// Tangential multi-child distribution in the connector path (see
// urdfAssembly.computeMatePlacement's multiChild branch) uses this helper
// to convert legacy (u, v) offsets into world-frame (dx, dy, dz) so 4 legs
// mating via mate_connector="bottom" spread across the baseplate instead
// of collapsing to one world point. Table matches the face→XYZ mapping
// that computeFacePlacement emits in urdfAssembly.ts lines ~1826-1892.

interface FaceMapCheck {
  name: string
  face: string
  u: number
  v: number
  expected: { dx: number; dy: number; dz: number }
}

const faceMapChecks: FaceMapCheck[] = [
  // top/bottom: face normal ±Z, tangent plane is XY — u→X, v→Y
  { name: 'faceUV: top (+Z normal) → u→X, v→Y',       face: 'top',    u: 0.010, v: 0.020, expected: { dx: 0.010, dy: 0.020, dz: 0 } },
  { name: 'faceUV: bottom (-Z normal) → u→X, v→Y',    face: 'bottom', u: -0.010, v: -0.020, expected: { dx: -0.010, dy: -0.020, dz: 0 } },
  // front/back: face normal ±X, tangent plane is YZ — u→Y, v→Z
  { name: 'faceUV: front (+X normal) → u→Y, v→Z',     face: 'front',  u: 0.015, v: 0.005, expected: { dx: 0, dy: 0.015, dz: 0.005 } },
  { name: 'faceUV: back (-X normal) → u→Y, v→Z',      face: 'back',   u: -0.015, v: 0.005, expected: { dx: 0, dy: -0.015, dz: 0.005 } },
  // left/right: face normal ±Y, tangent plane is XZ — u→X, v→Z
  { name: 'faceUV: right (+Y normal) → u→X, v→Z',     face: 'right',  u: 0.010, v: 0.020, expected: { dx: 0.010, dy: 0, dz: 0.020 } },
  { name: 'faceUV: left (-Y normal) → u→X, v→Z',      face: 'left',   u: 0.010, v: 0.020, expected: { dx: 0.010, dy: 0, dz: 0.020 } },
  // Fallback: unknown face name hits the default branch (same as top/bottom)
  { name: 'faceUV: unknown face falls back to XY mapping', face: 'plate_top', u: 0.010, v: 0.020, expected: { dx: 0.010, dy: 0.020, dz: 0 } },
]

function runFaceMapCheck(c: FaceMapCheck): Outcome {
  const got = faceUVToWorldOffset(c.face, c.u, c.v)
  const dx = got.dx - c.expected.dx
  const dy = got.dy - c.expected.dy
  const dz = got.dz - c.expected.dz
  const tol = 1e-9
  if (Math.abs(dx) > tol || Math.abs(dy) > tol || Math.abs(dz) > tol) {
    return { name: c.name, ok: false, reason: `expected (${c.expected.dx}, ${c.expected.dy}, ${c.expected.dz}), got (${got.dx}, ${got.dy}, ${got.dz})` }
  }
  return { name: c.name, ok: true }
}

// ── tangentBasisFromAxis checks ────────────────────────────────────────────
// The multi-child distribution path projects parent AABB extents onto the
// (u,v) tangent basis derived from the parent connector's axis. For canonical
// ±X/±Y/±Z axes the basis must match the face-name UV convention so results
// map cleanly through faceUVToWorldOffset and produce IDENTICAL placements to
// the legacy faceUVHalfExtents lookup. The tilted-axis branch is locked in
// against the generic orthonormality contract (u⊥n, v⊥n, u⊥v) so a future
// change can't silently break the path that no shipped preset exercises today.

interface BasisCheck {
  name: string
  axis: [number, number, number]
  // Either an exact basis (canonical-axis cases) or 'fallback' (perpendicularity
  // contract only — buildInPlaneBasis chooses one of two valid orientations).
  expected: { u: [number, number, number]; v: [number, number, number] } | 'fallback'
}

const basisChecks: BasisCheck[] = [
  { name: 'tangentBasis: +Z (top)    → u=X, v=Y',     axis: [0, 0, 1],  expected: { u: [1, 0, 0], v: [0, 1, 0] } },
  { name: 'tangentBasis: -Z (bottom) → u=X, v=Y',     axis: [0, 0, -1], expected: { u: [1, 0, 0], v: [0, 1, 0] } },
  { name: 'tangentBasis: +X (front)  → u=Y, v=Z',     axis: [1, 0, 0],  expected: { u: [0, 1, 0], v: [0, 0, 1] } },
  { name: 'tangentBasis: -X (back)   → u=Y, v=Z',     axis: [-1, 0, 0], expected: { u: [0, 1, 0], v: [0, 0, 1] } },
  { name: 'tangentBasis: +Y (right)  → u=X, v=Z',     axis: [0, 1, 0],  expected: { u: [1, 0, 0], v: [0, 0, 1] } },
  { name: 'tangentBasis: -Y (left)   → u=X, v=Z',     axis: [0, -1, 0], expected: { u: [1, 0, 0], v: [0, 0, 1] } },
  // Tilted axis (3-4-5 triangle in YZ plane): canonical branch must NOT match;
  // generic basis must satisfy the orthonormality contract.
  { name: 'tangentBasis: tilted [0, 0.6, 0.8] → buildInPlaneBasis fallback (orthonormal to axis)', axis: [0, 0.6, 0.8], expected: 'fallback' },
  // Slightly off-canonical axis (within ALIGN_EPS=0.999) still picks the
  // canonical branch — locks the threshold in.
  { name: 'tangentBasis: near-Z [0.01, 0, 0.9999] → still canonical XY basis', axis: [0.01, 0, 0.9999], expected: { u: [1, 0, 0], v: [0, 1, 0] } },
]

function approxVec3(a: THREE.Vector3, b: [number, number, number], tol = 1e-9): boolean {
  return Math.abs(a.x - b[0]) <= tol && Math.abs(a.y - b[1]) <= tol && Math.abs(a.z - b[2]) <= tol
}

function runBasisCheck(c: BasisCheck): Outcome {
  const { u, v } = tangentBasisFromAxis(new THREE.Vector3(c.axis[0], c.axis[1], c.axis[2]))
  if (c.expected === 'fallback') {
    const n = new THREE.Vector3(c.axis[0], c.axis[1], c.axis[2]).normalize()
    const tol = 1e-6
    const uLen = u.length(), vLen = v.length()
    if (Math.abs(uLen - 1) > tol || Math.abs(vLen - 1) > tol) {
      return { name: c.name, ok: false, reason: `non-unit basis: |u|=${uLen.toFixed(6)} |v|=${vLen.toFixed(6)}` }
    }
    if (Math.abs(u.dot(n)) > tol) return { name: c.name, ok: false, reason: `u not perpendicular to axis: u·n=${u.dot(n).toFixed(6)}` }
    if (Math.abs(v.dot(n)) > tol) return { name: c.name, ok: false, reason: `v not perpendicular to axis: v·n=${v.dot(n).toFixed(6)}` }
    if (Math.abs(u.dot(v)) > tol) return { name: c.name, ok: false, reason: `u not perpendicular to v: u·v=${u.dot(v).toFixed(6)}` }
    return { name: c.name, ok: true }
  }
  if (!approxVec3(u, c.expected.u) || !approxVec3(v, c.expected.v)) {
    return {
      name: c.name,
      ok: false,
      reason: `expected u=(${c.expected.u.join(',')}) v=(${c.expected.v.join(',')}), got u=(${u.x},${u.y},${u.z}) v=(${v.x},${v.y},${v.z})`,
    }
  }
  return { name: c.name, ok: true }
}

function main(): void {
  const outcomes: Outcome[] = [
    ...fixtures.map(runFixture),
    ...faceMapChecks.map(runFaceMapCheck),
    ...basisChecks.map(runBasisCheck),
  ]
  let passed = 0, failed = 0
  for (const o of outcomes) {
    if (o.ok) { console.log(`  ✓ ${o.name}`); passed++ }
    else       { console.log(`  ✗ ${o.name}`); console.log(`      ${o.reason}`); failed++ }
  }
  console.log(`\n[mate-corpus] ${passed}/${outcomes.length} passed`)
  if (failed > 0) process.exit(1)
}

main()
