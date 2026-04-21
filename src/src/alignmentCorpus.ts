// Render-time alignment corpus: runs synthetic parent↔child scenes through
// the pure reconcileNodePlacement pass and asserts the delta applied to the
// child's joint origin matches the mesh/bbox mismatch it was supposed to fix.
//
// Run: cd src && npm run test:alignment-corpus
// Direct: node --experimental-strip-types src/src/alignmentCorpus.ts
//
// Each fixture is a parent mesh + child mesh with *intentional* bbox/mesh
// mismatch (mesh extends past the preset box on one side, or is off-center).
// The placement engine would have placed the child assuming the preset box;
// reconcile is expected to observe the real AABBs and shift the pivot so
// the child's contact face meets the parent's attach face.
//
// Why this file exists: placement engine math stays pure (no THREE import in
// graphPreservationCorpus / topologyCorpus). Reconcile *does* need THREE —
// Three.js Box3/Object3D is the right tool for measuring rendered AABBs.
// This corpus keeps the THREE-dependent path behind its own harness so the
// rest of the test suite stays DOM-free.

import * as THREE from 'three'
import { reconcileNodePlacement } from './reconcileAlignment.ts'
import { resolveMate, type MateConnector } from './mateConnectors.ts'
import type { AssemblyGraph } from './urdfGraphEquivalence.ts'

// ── Scene builders ─────────────────────────────────────────────────────────

interface SyntheticLink {
  linkName: string
  /** Mesh size in local frame (meters). */
  size: [number, number, number]
  /** Mesh center offset from link origin (meters). Simulates off-center GLBs. */
  meshCenter?: [number, number, number]
}

interface SyntheticPair {
  parent: SyntheticLink
  child: SyntheticLink
  /** Assumed joint-origin placement (meters, parent-local). What the placement
   *  engine would have emitted based on preset boxes. The reconcile pass
   *  should correct this so the child face actually meets the parent face. */
  assumedPivotXyz: [number, number, number]
  attachFace: 'top' | 'bottom' | 'front' | 'back' | 'left' | 'right'
  pivotQuat?: THREE.Quaternion
}

function buildLinkGroup(link: SyntheticLink): THREE.Group {
  const lg = new THREE.Group()
  lg.userData.urdfLinkName = link.linkName
  const geomGroup = new THREE.Group()
  lg.add(geomGroup)
  const [sx, sy, sz] = link.size
  const geom = new THREE.BoxGeometry(sx, sy, sz)
  const mesh = new THREE.Mesh(geom)
  const [cx, cy, cz] = link.meshCenter ?? [0, 0, 0]
  mesh.position.set(cx, cy, cz)
  geomGroup.add(mesh)
  return lg
}

interface BuiltScene {
  linkGroups: Map<string, THREE.Group>
  joints: Map<string, { group: THREE.Group; axis: THREE.Vector3; type: string }>
  pivot: THREE.Group
  childLinkGroup: THREE.Group
  parentLinkGroup: THREE.Group
  graph: AssemblyGraph
}

function buildScene(pair: SyntheticPair): BuiltScene {
  const parentLg = buildLinkGroup(pair.parent)
  const childLg = buildLinkGroup(pair.child)

  // Same hierarchy the URDF parser produces: pivot between parent and child.
  const pivot = new THREE.Group()
  pivot.position.set(...pair.assumedPivotXyz)
  if (pair.pivotQuat) pivot.quaternion.copy(pair.pivotQuat)
  pivot.add(childLg)
  parentLg.add(pivot)

  const linkGroups = new Map<string, THREE.Group>([
    [pair.parent.linkName, parentLg],
    [pair.child.linkName, childLg],
  ])
  const joints = new Map<string, { group: THREE.Group; axis: THREE.Vector3; type: string }>([
    [`joint_${pair.child.linkName}`, { group: pivot, axis: new THREE.Vector3(0, 0, 1), type: 'fixed' }],
  ])
  const graph: AssemblyGraph = {
    base_link: pair.parent.linkName,
    components: [
      { link_name: pair.parent.linkName, component_id: pair.parent.linkName, attach_to: null, attach_face: null, joint_type: 'fixed', joint_axis: 'z' },
      { link_name: pair.child.linkName, component_id: pair.child.linkName, attach_to: pair.parent.linkName, attach_face: pair.attachFace, joint_type: 'fixed', joint_axis: 'z' },
    ],
  }
  parentLg.updateMatrixWorld(true)
  return { linkGroups, joints, pivot, childLinkGroup: childLg, parentLinkGroup: parentLg, graph }
}

// ── Fixtures ───────────────────────────────────────────────────────────────
// Scenarios are built around the real mismatch classes from
// docs/ENGINE_ALIGNMENT_PLAN.md: asymmetric mesh centroids, mesh extents that
// differ from the assumed preset bbox, and axis-swapped child meshes.

interface Fixture {
  name: string
  pair: SyntheticPair
  /** Expected pivot position AFTER reconcile, parent-local meters. */
  expectedPivotXyz: [number, number, number]
  /** Tolerance (meters). */
  tolerance?: number
}

const fixtures: Fixture[] = [
  {
    // Parent mesh is CENTERED, but the mesh is 40 mm *taller* than the box the
    // placement engine assumed. Reconcile should lift the child up by 20 mm
    // (half the extra height) so its bottom touches the real top face.
    name: 'parent mesh taller than placement assumed → child shifts UP',
    pair: {
      parent: { linkName: 'plate', size: [0.2, 0.15, 0.04] },     // 40 mm tall
      child:  { linkName: 'cube',  size: [0.03, 0.03, 0.03] },    // 30 mm cube
      assumedPivotXyz: [0, 0, 0.015], // what placement engine WOULD emit for a 0-mm-thick plate
      attachFace: 'top',
    },
    // Parent top face is at +0.02 m (half of 0.04). Child bottom in child-local
    // is -0.015 m. Target pivot.z = 0.02 - (-0.015) = 0.035 m.
    expectedPivotXyz: [0, 0, 0.035],
  },
  {
    // Parent mesh is THINNER than placement assumed (preset over-reports the
    // thickness by 20 mm total). Reconcile should pull the child DOWN by 10 mm
    // so it meets the real (lower) top face.
    name: 'parent mesh thinner than placement assumed → child shifts DOWN',
    pair: {
      parent: { linkName: 'plate', size: [0.2, 0.15, 0.008] },    // real 8 mm
      child:  { linkName: 'cube',  size: [0.03, 0.03, 0.03] },
      assumedPivotXyz: [0, 0, 0.025], // assumed 28 mm plate → child sits 25 mm above origin
      attachFace: 'top',
    },
    // Parent top at +0.004. Child bottom in child-local is -0.015. Target pivot.z = 0.004 + 0.015 = 0.019.
    expectedPivotXyz: [0, 0, 0.019],
  },
  {
    // Asymmetric mesh: centroid offset from link origin by (0, +0.04, +0.04).
    // Mirrors the structural_torso_panel M1 case. For a TOP face mount, the
    // placement engine would put the child at pivot.z = hz (half thickness),
    // but the real top face is higher by the centroid offset in Z. Reconcile
    // projects the delta onto the face normal (+Z), so X/Y are preserved — we
    // don't re-centre the child on the off-centre mesh.
    name: 'parent mesh centroid off-center on Z → child lifts to match true top (XY preserved)',
    pair: {
      parent: {
        linkName: 'torso',
        size: [0.094, 0.154, 0.105],
        meshCenter: [0, 0.04, 0.04],      // off-center GLB, mirrors M1 report
      },
      child: { linkName: 'electronics', size: [0.05, 0.05, 0.02] },
      // Placement engine assumed a centered mesh: parent top at +0.0525,
      // child bottom at -0.01 → pivot.z = 0.0525 + 0.01 = 0.0625.
      assumedPivotXyz: [0, 0, 0.0625],
      attachFace: 'top',
    },
    // Real parent top face (aabb max z) = 0.04 + 0.105/2 = 0.0925.
    // Target pivot.z = 0.0925 + 0.01 = 0.1025. X/Y stay at their placement-
    // engine values (here, 0) — normal-only projection discards tangential shift.
    expectedPivotXyz: [0, 0, 0.1025],
  },
  {
    // Axis-swapped child mesh (like structural_servo_coupler_disc whose GLB
    // has extents [31.96, 8, 31.98] vs preset [32, 32, 8]). Reconcile should
    // use the REAL child AABB (8 mm thick on Z) instead of the assumed 32 mm
    // thick preset, so the child doesn't float above the parent.
    name: 'axis-swapped child (thin on Z) sits flush — not floating',
    pair: {
      parent: { linkName: 'plate', size: [0.2, 0.15, 0.02] },     // 20 mm
      child:  { linkName: 'disc',  size: [0.032, 0.032, 0.008] }, // real coupler shape: thin Z
      // Placement engine assumed a 32 mm thick disc → pivot.z = 0.02/2 + 0.032/2 = 0.026.
      assumedPivotXyz: [0, 0, 0.026],
      attachFace: 'top',
    },
    // Real: parent top at +0.01, child bottom at -0.004 → target pivot.z = 0.014.
    expectedPivotXyz: [0, 0, 0.014],
  },
  {
    // Bottom-face mount: child hangs off the underside of the parent. Regression
    // check that the face-normal math is symmetric and 'bottom' resolves correctly.
    name: 'bottom-face child shifts DOWN when parent mesh thicker than assumed',
    pair: {
      parent: { linkName: 'chassis', size: [0.2, 0.15, 0.06] },
      child:  { linkName: 'pad',     size: [0.04, 0.04, 0.01] },
      // Assumed parent 20 mm thick → pivot.z = -(0.02/2 + 0.01/2) = -0.015.
      assumedPivotXyz: [0, 0, -0.015],
      attachFace: 'bottom',
    },
    // Real: parent bottom at -0.03, child top at +0.005 → target pivot.z = -0.035.
    expectedPivotXyz: [0, 0, -0.035],
  },
  {
    // Regression: when the placement engine distributed N children across a face
    // (e.g. 4 legs on baseplate corners), reconcile must NOT re-centre them on
    // the face. The pre-fix bug collapsed all 4 electronics to (0,0) — this
    // fixture would have caught it immediately.
    name: 'child at corner XY offset — reconcile preserves tangential placement',
    pair: {
      parent: { linkName: 'plate', size: [0.35, 0.25, 0.008] },
      child:  { linkName: 'battery', size: [0.105, 0.034, 0.024] },
      // Engine placed the battery at the +X,+Y corner of the top face with its
      // bottom at the plate surface.
      assumedPivotXyz: [0.1225, 0.0875, 0.016],
      attachFace: 'top',
    },
    // Only Z needs adjustment here (mesh matches bbox, so delta≈0). X/Y stay
    // at their corner values — the pass must NOT drag them to (0,0).
    expectedPivotXyz: [0.1225, 0.0875, 0.016],
    tolerance: 1e-9,
  },
]

// ── Bug 1 probe: connector-placed servo+coupler through reconcile ─────────
// Mimics actuator_servo_standard (shaft_out at [0,0,18.5] mm) + structural_
// servo_coupler_disc (shaft_hole at [0,0,-4] mm), the canonical auto-repair
// Case 1 pair. computeMatePlacement puts the coupler via concentric mate
// (resolveMate closed-form). The child retains attach_face="top" — the
// auto-repair emission pattern — so reconcile enters the attach_face branch.
// If delta > ~0.5 mm, Bug 1 is real (connector placement + reconcile disagree)
// and the `placed_via_connector` skip flag has to land. Delta ≈ 0 means the
// authored connector origins happen to sit on the bbox faces that reconcile
// measures, so the flag is dead code — ENGINE_EXECUTION_PLAN Bug 1 outcome.

function deriveConcentricPivotMeters(
  parentConn: MateConnector,
  childConn: MateConnector,
): [number, number, number] {
  const m = resolveMate(new THREE.Matrix4(), parentConn, childConn, 'concentric', {})
  const pos = new THREE.Vector3()
  const q = new THREE.Quaternion()
  const s = new THREE.Vector3()
  m.decompose(pos, q, s)
  return [pos.x, pos.y, pos.z]
}

const bug1_servoShaftOut: MateConnector = {
  id: 'shaft_out',
  origin_xyz_mm: [0, 0, 18.5],
  axis_xyz: [0, 0, 1],
  type: 'cylindrical',
  diameter_mm: 5.9,
}
const bug1_couplerShaftHole: MateConnector = {
  id: 'shaft_hole',
  origin_xyz_mm: [0, 0, -4],
  axis_xyz: [0, 0, -1],
  type: 'cylindrical',
  diameter_mm: 8,
}
const bug1_pivotXyz = deriveConcentricPivotMeters(bug1_servoShaftOut, bug1_couplerShaftHole)

fixtures.push({
  // Canonical servo+coupler via mate connectors. Parent servo mesh matches the
  // bbox [40,20,37] exactly (shaft_out sits flush with bbox +Z top). Coupler
  // mesh matches bbox [32,32,8] exactly (shaft_hole sits flush with bbox -Z
  // bottom). With attach_face="top" set on the child (auto-repair emission),
  // reconcile measures parent_top ↔ child_bottom and — if connectors and bbox
  // faces coincide — finds zero delta. Expectation: pivot stays where the
  // connector resolver placed it (ex/ey/ez = the resolveMate output).
  name: 'Bug 1 probe: connector-placed servo→coupler, attach_face="top" set, reconcile leaves pivot alone',
  pair: {
    parent: { linkName: 'servo', size: [0.040, 0.020, 0.037] },
    child:  { linkName: 'coupler', size: [0.032, 0.032, 0.008] },
    assumedPivotXyz: bug1_pivotXyz,
    attachFace: 'top',
  },
  expectedPivotXyz: bug1_pivotXyz,
  tolerance: 0.0005, // 0.5 mm — Bug 1 real if reconcile shifts more than this.
})

// ── Runner ─────────────────────────────────────────────────────────────────

interface Outcome {
  name: string
  ok: boolean
  reason?: string
}

function runFixture(f: Fixture): Outcome {
  const built = buildScene(f.pair)
  const res = reconcileNodePlacement({
    graph: built.graph,
    linkGroups: built.linkGroups,
    joints: built.joints,
    silent: true,
  })

  const tol = f.tolerance ?? 1e-6
  const [ex, ey, ez] = f.expectedPivotXyz
  const { x, y, z } = built.pivot.position
  const dx = Math.abs(x - ex)
  const dy = Math.abs(y - ey)
  const dz = Math.abs(z - ez)

  if (dx > tol || dy > tol || dz > tol) {
    return {
      name: f.name,
      ok: false,
      reason: `pivot mismatch: expected (${ex}, ${ey}, ${ez}), got (${x.toFixed(5)}, ${y.toFixed(5)}, ${z.toFixed(5)}); shifts=${JSON.stringify(res.shifts)}`,
    }
  }
  return { name: f.name, ok: true }
}

function main(): void {
  const outcomes = fixtures.map(runFixture)
  let passed = 0
  let failed = 0
  for (const o of outcomes) {
    if (o.ok) {
      console.log(`  ✓ ${o.name}`)
      passed++
    } else {
      console.log(`  ✗ ${o.name}`)
      console.log(`      ${o.reason}`)
      failed++
    }
  }
  console.log(`\n[alignment-corpus] ${passed}/${outcomes.length} passed`)
  if (failed > 0) process.exit(1)
}

main()
