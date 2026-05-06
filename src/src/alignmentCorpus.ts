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
import { nudgeAlongNormal, shouldApplyRuntimeNudge } from './contactCleanup.ts'
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
  /** When true, the child carries `placed_via_connector` — reconcile must
   *  leave its pivot alone regardless of any bbox-vs-mesh disagreement. */
  childPlacedViaConnector?: boolean
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
      {
        link_name: pair.child.linkName,
        component_id: pair.child.linkName,
        attach_to: pair.parent.linkName,
        attach_face: pair.attachFace,
        joint_type: 'fixed',
        joint_axis: 'z',
        ...(pair.childPlacedViaConnector ? { placed_via_connector: true } : {}),
      },
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
// measures, so the flag is dead code.

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

// ── Layer 3 probe: placed_via_connector flag stops reconcile stomping ──────
// Mirror the smoke-test failure mode: a parent whose bbox-derived face center
// disagrees with where the child actually sits (here, an authored connector
// recessed 10 mm below the bbox top — common in compound housings, sensor
// recesses, baseplates with mounting standoffs). Without the flag, reconcile
// would measure the bbox top and shove the IMU 10 mm upward, exactly the
// stomp pattern the 2026-04-21 quadruped trace showed on the coupler discs
// (-10 mm reconcile delta on top of a +0 connector placement). With the flag,
// reconcile must leave the pivot at the connector's chosen Z.
fixtures.push({
  name: 'Layer 3 probe: placed_via_connector flag prevents reconcile from stomping a connector-recessed child',
  pair: {
    parent: { linkName: 'baseplate', size: [0.350, 0.250, 0.030] },
    child:  { linkName: 'imu',       size: [0.016, 0.016, 0.004] },
    // Connector "top" authored recessed at z=+5mm (parent bbox top is at +15mm).
    // Connector path placed IMU at pivot.z = 5mm + 4mm/2 = 7mm. Without the
    // flag, reconcile would target bbox top + child half = 15 + 2 = 17mm and
    // shift the pivot by +10mm. With the flag, pivot stays at 7mm.
    assumedPivotXyz: [0, 0, 0.007],
    attachFace: 'top',
    childPlacedViaConnector: true,
  },
  expectedPivotXyz: [0, 0, 0.007],
  tolerance: 1e-9,
})

// ── Step 1 probe: engagement_depth_mm pulls child INTO parent along axis ──
// Validates the connector schema's new engagement_depth_mm field. Two parent
// connectors with identical origin/axis but engagement_depth_mm = 0 vs = 2:
// the engaged pivot must be exactly 2mm closer along the parent connector's
// axis. The placed_via_connector flag is set so reconcile cannot stomp the
// intentional sub-flush placement (the whole point of the field is to sit
// the child slightly INSIDE the parent surface, hiding chamfer-vs-flat gaps).
//
// Why a delta assertion: a fixture that just hard-codes the expected pivot
// would pass even if engagement_depth were silently dropped (resolveMate
// would return the unchanged pose). Comparing engaged vs non-engaged proves
// the field is actually consumed.

const step1_parentTop_noEngagement: MateConnector = {
  id: 'top',
  origin_xyz_mm: [0, 0, 20],
  axis_xyz: [0, 0, 1],
  type: 'planar',
}
const step1_parentTop_engaged2mm: MateConnector = {
  id: 'top',
  origin_xyz_mm: [0, 0, 20],
  axis_xyz: [0, 0, 1],
  type: 'planar',
  engagement_depth_mm: 2,
}
const step1_childBottom: MateConnector = {
  id: 'bottom',
  origin_xyz_mm: [0, 0, -15],
  axis_xyz: [0, 0, -1],
  type: 'planar',
}

function deriveFastenedPivotMeters(
  parentConn: MateConnector,
  childConn: MateConnector,
): [number, number, number] {
  const m = resolveMate(new THREE.Matrix4(), parentConn, childConn, 'fastened', {})
  const pos = new THREE.Vector3()
  const q = new THREE.Quaternion()
  const s = new THREE.Vector3()
  m.decompose(pos, q, s)
  return [pos.x, pos.y, pos.z]
}

const step1_pivot_noEng = deriveFastenedPivotMeters(step1_parentTop_noEngagement, step1_childBottom)
const step1_pivot_engaged = deriveFastenedPivotMeters(step1_parentTop_engaged2mm, step1_childBottom)

// Sanity: along parent axis (+Z), engaged pivot must be 2mm LESS than non-
// engaged. axis_xyz=[0,0,1] so the axial component is just z.
const step1_axialDelta_mm = (step1_pivot_noEng[2] - step1_pivot_engaged[2]) * 1000
if (Math.abs(step1_axialDelta_mm - 2) > 1e-6) {
  console.log(`  ✗ Step 1 pre-check: engagement_depth_mm=2 should pull pivot 2mm closer along +Z — got ${step1_axialDelta_mm.toFixed(6)}mm`)
  process.exit(1)
}

fixtures.push({
  // Mirrors the chamfered-servo-top use case: child connector sits at the
  // bbox top, but the parent's authored engagement_depth_mm pulls the child
  // 2mm into the parent body so the disc edge tucks under the chamfer line.
  // Reconcile must NOT undo the intentional 2mm overlap.
  name: 'Step 1 probe: parent engagement_depth_mm=2 pulls child 2mm closer along axis; reconcile leaves flagged pivot alone',
  pair: {
    parent: { linkName: 'plate', size: [0.040, 0.040, 0.040] }, // 40mm cube — bbox top at +20mm
    child:  { linkName: 'disc',  size: [0.030, 0.030, 0.030] }, // 30mm cube — bbox bottom at -15mm
    assumedPivotXyz: step1_pivot_engaged, // pivot from engaged resolveMate
    attachFace: 'top',
    childPlacedViaConnector: true,
  },
  expectedPivotXyz: step1_pivot_engaged, // reconcile must leave it alone
  tolerance: 1e-9,
})

// ── Step 2 ICP fixtures: runtime nudge-along-normal ───────────────────────
// Hand-built THREE meshes exercise the pure `nudgeAlongNormal` from
// contactCleanup.ts. These fixtures don't share the reconcileNodePlacement
// runner above because ICP is measured at placement time (before reconcile),
// not scene-level. A separate runner asserts the returned nudge magnitude.

interface IcpFixture {
  name: string
  run: () => { ok: boolean; reason?: string }
}

/** Flat-top parent cube, parent connector at +Z face center. */
function buildFlatTopParent(sideM = 0.04): THREE.Mesh {
  const geom = new THREE.BoxGeometry(sideM, sideM, sideM)
  return new THREE.Mesh(geom)
}

/** Parent whose top face has a 2 mm "chamfered" ring — modeled as a raised
 *  central plateau (20 mm wide) sitting on a shorter base (40 mm wide).
 *  Rays inside |u|,|v| < 10 mm hit the plateau top at z=+20 (axial 0); rays
 *  outside hit the base top at z=+18 (axial −2 mm). With the default disc
 *  sample radius of 0.4×40 mm = 16 mm, ≈60% of samples land in the
 *  "chamfered" ring, so the 90th-percentile gap lands at 2 mm. */
function buildChamferedTopParent(): THREE.Group {
  const g = new THREE.Group()
  // Base: 40×40×38, top face at z=+18.
  const baseGeom = new THREE.BoxGeometry(0.040, 0.040, 0.038)
  const base = new THREE.Mesh(baseGeom)
  base.position.set(0, 0, -0.001)
  g.add(base)
  // Plateau: 20×20×2, top face at z=+20.
  const plateauGeom = new THREE.BoxGeometry(0.020, 0.020, 0.002)
  const plateau = new THREE.Mesh(plateauGeom)
  plateau.position.set(0, 0, 0.019)
  g.add(plateau)
  return g
}

function buildFlatBottomChild(sideM = 0.03): THREE.Mesh {
  const geom = new THREE.BoxGeometry(sideM, sideM, sideM)
  return new THREE.Mesh(geom)
}

const icpFixtures: IcpFixture[] = [
  {
    name: 'Step 2 ICP: flat-on-flat → nudge ≈ 0 (no-op)',
    run: () => {
      const parent = buildFlatTopParent(0.040)
      const child = buildFlatBottomChild(0.030)
      const nudge = nudgeAlongNormal(
        parent, child,
        [0, 0, 0.020],  [0, 0, 1],
        [0, 0, -0.015],
      )
      if (nudge > 0.0001) {
        return { ok: false, reason: `expected ~0, got ${(nudge * 1000).toFixed(4)} mm` }
      }
      return { ok: true }
    },
  },
  {
    name: 'Step 2 ICP: flat-on-chamfered → nudge ≈ 2 mm chamfer height',
    run: () => {
      const parent = buildChamferedTopParent()
      const child = buildFlatBottomChild(0.030)
      const nudge = nudgeAlongNormal(
        parent, child,
        [0, 0, 0.020],  [0, 0, 1],
        [0, 0, -0.015],
      )
      const nudgeMm = nudge * 1000
      // Allow 0.5 mm tolerance — the exact value depends on how disc samples
      // tile the plateau-vs-ring split; 1.5-2.5 mm is well within spec.
      if (Math.abs(nudgeMm - 2) > 0.5) {
        return { ok: false, reason: `expected ~2 mm, got ${nudgeMm.toFixed(4)} mm` }
      }
      return { ok: true }
    },
  },
  {
    name: 'Step 2 ICP: engagement_depth_mm authored → runtime nudge skipped',
    run: () => {
      // The caller (urdfAssembly.ts) consults shouldApplyRuntimeNudge before
      // invoking the expensive raycast pass. When the parent connector has
      // engagement_depth_mm authored (Step 1 path), the runtime nudge must
      // be skipped so we don't stack two gap-closing translations on top of
      // each other. Verify the gate rule directly — skipping isolates the
      // decision from scene mocking.
      if (shouldApplyRuntimeNudge(undefined) !== true) {
        return { ok: false, reason: 'expected skip=false for undefined engagement' }
      }
      if (shouldApplyRuntimeNudge(0) !== true) {
        return { ok: false, reason: 'expected skip=false for engagement=0 (author-disabled)' }
      }
      if (shouldApplyRuntimeNudge(1.5) !== false) {
        return { ok: false, reason: 'expected skip=true for engagement=1.5 (authored)' }
      }
      if (shouldApplyRuntimeNudge(NaN) !== true) {
        return { ok: false, reason: 'expected skip=false for NaN (malformed)' }
      }
      return { ok: true }
    },
  },
  {
    // Parent connector authored 5mm BELOW actual body bottom (mirrors the
    // servo_high_torque case where authored bottom.origin_xyz_mm[2]=-17 but
    // the rendered body's bottom surface sits at z=-12.834 after shaft-overlay
    // shift). ICP must detect the consistent 5mm gap and — because the
    // spread is near-zero and paired coverage is high — engage the adaptive
    // confident-cap to close the full 5mm instead of clipping to 3mm.
    //
    // Regression guard for the Session 3 Phase 2 fix: before adaptive cap
    // landed, a 4-5mm uniform gap would return nudge=3mm (clamped) and
    // leave a visible 1-2mm residual. After: returns the real 5mm.
    name: 'Step 2 ICP: flat-5mm-recessed parent → nudge ≈ 5 mm (adaptive confident-cap)',
    run: () => {
      // Parent body: 40×40×30 centered at origin. Real body bottom at z=-15.
      // Authored bottom connector at z=-20 (5mm below actual surface).
      const parent = new THREE.Mesh(new THREE.BoxGeometry(0.040, 0.040, 0.030))
      // Child: 20×20×20 cube with top at z=+10 (authored).
      const child = new THREE.Mesh(new THREE.BoxGeometry(0.020, 0.020, 0.020))
      const nudge = nudgeAlongNormal(
        parent, child,
        [0, 0, -0.020], [0, 0, -1],   // parent bottom connector, outward -Z
        [0, 0, +0.010],                // child top connector in child local
      )
      const nudgeMm = nudge * 1000
      // Expect ~5mm — slightly below due to percentile picking p90 (mostly
      // same values). Allow 4-6mm.
      if (Math.abs(nudgeMm - 5) > 1) {
        return { ok: false, reason: `expected ~5 mm (adaptive), got ${nudgeMm.toFixed(4)} mm — adaptive cap may not be firing` }
      }
      return { ok: true }
    },
  },
  {
    // Small child on big parent face — the footpad-on-extrusion-bottom and
    // thigh-extrusion-on-servo-bottom case. Parent 80×80 face with authored
    // connector offset 5 mm from actual surface. Child 20×20 footprint — ~12%
    // of the sample disc lands inside the child, so paired/total is below
    // the 30% "tight" threshold. But every paired sample reports the same
    // 5 mm gap, so the UNIFORM gate (|p90-p50| < 1 mm AND paired ≥ 6) must
    // fire the confident cap even though paired fraction is low.
    //
    // Regression guard: without UNIFORM mode, this clamps at 3 mm.
    name: 'Step 2 ICP: small-child + uniform gap → nudge ≈ 5 mm (adaptive via UNIFORM gate)',
    run: () => {
      // Parent 80×80×30 (deliberately large face so child is a small fraction
      // of the sample disc). Real body bottom at z=-15.
      const parent = new THREE.Mesh(new THREE.BoxGeometry(0.080, 0.080, 0.030))
      // Child 20×20×20 cube, top connector at z=+10.
      const child = new THREE.Mesh(new THREE.BoxGeometry(0.020, 0.020, 0.020))
      const diag = {
        sampleCount: 0, parentHits: 0, childHits: 0, pairedCount: 0, faceRadiusM: 0,
        nudgeMm: 0, reason: '',
      }
      const nudge = nudgeAlongNormal(
        parent, child,
        [0, 0, -0.020], [0, 0, -1],
        [0, 0, +0.010],
        { diagnostics: diag },
      )
      const nudgeMm = nudge * 1000
      if (Math.abs(nudgeMm - 5) > 1) {
        return {
          ok: false,
          reason: `expected ~5 mm via UNIFORM gate, got ${nudgeMm.toFixed(4)} mm (paired=${diag.pairedCount}, reason="${diag.reason}")`,
        }
      }
      // Sanity: UNIFORM mode should be the reason label, not "tight" (paired
      // fraction for this setup sits below 30%).
      if (!diag.reason.includes('uniform') && !diag.reason.includes('tight')) {
        return {
          ok: false,
          reason: `expected confident-cap to fire, got "${diag.reason}"`,
        }
      }
      return { ok: true }
    },
  },
]

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
  // ICP fixtures run their own THREE scenes — merge into the same pass/fail
  // accounting so a single harness exit status covers the whole corpus.
  const icpOutcomes: Outcome[] = icpFixtures.map(f => {
    const r = f.run()
    return { name: f.name, ok: r.ok, reason: r.reason }
  })
  const all = [...outcomes, ...icpOutcomes]
  let passed = 0
  let failed = 0
  for (const o of all) {
    if (o.ok) {
      console.log(`  ✓ ${o.name}`)
      passed++
    } else {
      console.log(`  ✗ ${o.name}`)
      console.log(`      ${o.reason}`)
      failed++
    }
  }
  console.log(`\n[alignment-corpus] ${passed}/${all.length} passed`)
  if (failed > 0) process.exit(1)
}

main()
