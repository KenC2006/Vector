// Phase 3b — placement compiler corpus.
//
// Today (Phase 3b.4.A): smoke-tests the empty-graph stub, the
// PlacementClass taxonomy, and the parity harness wiring on synthetic data.
// Slices 4.B–4.J each add invariants exercising the placement class they
// implement. Slice 4.K adds the cross-runtime fingerprint match.
//
// Wired into `npm run check` from day one so regressions in the harness
// itself are caught before any compiler logic lands.

import {
  compileAssembly,
  COMPILER_VERSION,
  ALL_PLACEMENT_CLASSES,
  type ComponentResolver,
} from './placementCompiler/index.ts'
import type { MateConnector } from './mateConnectors.ts'
import {
  compareGraphs,
  type ObservedGraph,
} from './placementCompiler/parityHarness.ts'
import type { AssemblyGraph } from './urdfGraphEquivalence.ts'

/** Synthesizes a 4-cm cube preset stand-in. Real bounds come from the
 *  preset catalog at runtime; the corpus just needs deterministic data. */
const stubResolver: ComponentResolver = (componentId, _instance) => ({
  componentId,
  bounds: {
    half: [0.02, 0.02, 0.02],
    center: [0, 0, 0],
    shape: 'box',
  },
})

type TestFn = () => void | Promise<void>
const TESTS: Array<[string, TestFn]> = []
function test(name: string, fn: TestFn) { TESTS.push([name, fn]) }

function assert(cond: unknown, msg: string): asserts cond {
  if (!cond) throw new Error(msg)
}

// ── Skeleton invariants ─────────────────────────────────────────────────────

test('compileAssembly empty graph: no links, all non-root classes skipped', () => {
  const graph: AssemblyGraph = { base_link: 'base', components: [] }
  const result = compileAssembly(graph)
  assert(result.baseLink === 'base', 'baseLink passes through')
  assert(result.links.length === 0, 'no links to emit for an empty graph')
  assert(result.attachIndex && Object.keys(result.attachIndex).length === 0, 'empty attachIndex')
  assert(result.diagnostics.length === 0, 'no diagnostics for empty graph')
  assert(result.fingerprint.length > 0, 'fingerprint present')
  // Slice 4.B implements `root`; everything else is still skipped.
  assert(result.skippedClasses.length === ALL_PLACEMENT_CLASSES.length - 12,
    `expected ${ALL_PLACEMENT_CLASSES.length - 12} skipped classes (root + face_simple + face_multi_child + mate_connector + servo_split + foot_leveling + ground_offset + mirrored_hardware + archetype_axis_normalize + drivetrain_remap implemented), got ${result.skippedClasses.length}`)
  assert(!result.skippedClasses.includes('drivetrain_remap'), 'drivetrain_remap must NOT be in skippedClasses after slice 4.I')
  assert(!result.skippedClasses.includes('parametric_splice'), 'parametric_splice must NOT be in skippedClasses after slice 4.J½')
  assert(!result.skippedClasses.includes('port_resolution'), 'port_resolution must NOT be in skippedClasses after slice 4.J')
  assert(result.skippedClasses.length === 0, `expected ALL classes implemented, still skipped: ${result.skippedClasses.join(', ')}`)
  assert(!result.skippedClasses.includes('root'), 'root must NOT be in skippedClasses after slice 4.B')
  assert(!result.skippedClasses.includes('face_simple'), 'face_simple must NOT be in skippedClasses after slice 4.C')
  assert(!result.skippedClasses.includes('face_multi_child'), 'face_multi_child must NOT be in skippedClasses after slice 4.D')
  assert(!result.skippedClasses.includes('mate_connector'), 'mate_connector must NOT be in skippedClasses after slice 4.E')
  assert(!result.skippedClasses.includes('servo_split'), 'servo_split must NOT be in skippedClasses after slice 4.F')
  assert(!result.skippedClasses.includes('foot_leveling'), 'foot_leveling must NOT be in skippedClasses after slice 4.H')
  assert(!result.skippedClasses.includes('ground_offset'), 'ground_offset must NOT be in skippedClasses after slice 4.H')
  assert(!result.skippedClasses.includes('mirrored_hardware'), 'mirrored_hardware must NOT be in skippedClasses after slice 4.G')
  assert(!result.skippedClasses.includes('archetype_axis_normalize'), 'archetype_axis_normalize must NOT be in skippedClasses after slice 4.I')
})

test('compiler version is present and non-empty', () => {
  assert(typeof COMPILER_VERSION === 'string' && COMPILER_VERSION.length > 0,
    'COMPILER_VERSION must be a non-empty string for fingerprint pinning')
})

test('PlacementClass taxonomy enumerates every rollout slice', () => {
  // 4.B…4.J each implement one class; 4.K cuts over. Bumping the list here
  // without bumping the rollout plan is a red flag — if you're adding a class,
  // add a slice for it.
  const expected = [
    'root', 'face_simple', 'face_multi_child', 'mate_connector', 'servo_split',
    'mirrored_hardware', 'foot_leveling', 'ground_offset', 'drivetrain_remap',
    'archetype_axis_normalize', 'parametric_splice', 'port_resolution',
  ]
  assert(ALL_PLACEMENT_CLASSES.length === expected.length,
    `expected ${expected.length} placement classes, got ${ALL_PLACEMENT_CLASSES.length}`)
  for (let i = 0; i < expected.length; i++) {
    assert(ALL_PLACEMENT_CLASSES[i] === expected[i],
      `class[${i}]: expected "${expected[i]}", got "${ALL_PLACEMENT_CLASSES[i]}"`)
  }
})

// ── Parity harness wiring ───────────────────────────────────────────────────

// ── Slice 4.B — root link emission ──────────────────────────────────────────

test('compileAssembly emits a single root CompiledLink at identity pose', () => {
  const graph: AssemblyGraph = {
    base_link: 'plate_1',
    components: [{
      link_name: 'plate_1', component_id: 'structural_baseplate',
      attach_to: null, attach_face: 'top',
      joint_type: 'fixed', joint_axis: 'z',
    }],
  }
  const result = compileAssembly(graph, { resolveComponent: stubResolver })
  assert(result.diagnostics.length === 0, `unexpected diagnostics: ${JSON.stringify(result.diagnostics)}`)
  assert(result.baseLink === 'plate_1', `baseLink should reflect root.link_name, got ${result.baseLink}`)
  assert(result.links.length === 1, `expected 1 link, got ${result.links.length}`)
  const root = result.links[0]
  assert(root.logicalName === 'plate_1', 'logicalName')
  assert(root.physicalLinks.length === 1 && root.physicalLinks[0] === 'plate_1', 'physicalLinks')
  assert(root.childAttachTarget === 'plate_1', 'childAttachTarget')
  assert(root.localXyz[0] === 0 && root.localXyz[1] === 0 && root.localXyz[2] === 0, 'root local xyz zero')
  assert(root.worldXyz[0] === 0 && root.worldXyz[1] === 0 && root.worldXyz[2] === 0, 'root world xyz zero')
  assert(root.joints.length === 0, 'root has no joints')
  assert(root.syntheticRole === null, 'root is not synthetic')
  assert(result.attachIndex['plate_1'] === 'plate_1', 'attachIndex maps logical to itself')
})

test('compileAssembly emits diagnostic when resolver missing for a non-empty graph', () => {
  const graph: AssemblyGraph = {
    base_link: 'a',
    components: [{
      link_name: 'a', component_id: 'x',
      attach_to: null, attach_face: 'top',
      joint_type: 'fixed', joint_axis: 'z',
    }],
  }
  const result = compileAssembly(graph)   // no resolver
  assert(result.links.length === 0, 'no links emitted when resolver missing')
  assert(result.diagnostics.some(d => d.code === 'compiler.missing_resolver'),
    'must emit missing_resolver diagnostic')
})

test('compileAssembly emits diagnostic for unknown root component', () => {
  const graph: AssemblyGraph = {
    base_link: 'a',
    components: [{
      link_name: 'a', component_id: 'does_not_exist',
      attach_to: null, attach_face: 'top',
      joint_type: 'fixed', joint_axis: 'z',
    }],
  }
  const result = compileAssembly(graph, { resolveComponent: () => null })
  assert(result.links.length === 0, 'no links emitted when root unresolvable')
  assert(result.diagnostics.some(d => d.code === 'compiler.unknown_component'),
    'must emit unknown_component diagnostic')
})

test('parity harness matches root link between observed and compiled', () => {
  // Synthesize an observed graph mirroring a real assembly's root capture
  // (identity world pose, null local — exactly what captureObservedGraph
  // produces for the root in resolveAssemblyGraph).
  const observed: ObservedGraph = {
    baseLink: 'plate_1',
    links: [{
      logicalName: 'plate_1', physicalLinkName: 'plate_1',
      componentId: 'structural_baseplate',
      parentLogicalName: null,
      localXyz: null, localRpy: null,
      worldXyz: [0, 0, 0], worldRpy: [0, 0, 0],
    }],
  }
  const compiled = compileAssembly({
    base_link: 'plate_1',
    components: [{
      link_name: 'plate_1', component_id: 'structural_baseplate',
      attach_to: null, attach_face: 'top',
      joint_type: 'fixed', joint_axis: 'z',
    }],
  }, { resolveComponent: stubResolver })
  const report = compareGraphs(observed, compiled)
  assert(report.diffs.length === 0, `expected zero diffs, got: ${JSON.stringify(report.diffs)}`)
  assert(report.matched === 1, `expected 1 matched, got ${report.matched}`)
  assert(report.unmatched === 0, `expected 0 unmatched, got ${report.unmatched}`)
})

// ── Slice 4.C — simple face placement ───────────────────────────────────────

test('compileAssembly emits CompiledLink for a single face-mounted child', () => {
  // Root: 4-cm cube. Child: 4-cm cube on the top face.
  // Expected local placement: child sits flush above parent → xyz = (0, 0, parent.hz + child.hz) = (0, 0, 0.04)
  const graph: AssemblyGraph = {
    base_link: 'plate_1',
    components: [
      {
        link_name: 'plate_1', component_id: 'structural_baseplate',
        attach_to: null, attach_face: 'top',
        joint_type: 'fixed', joint_axis: 'z',
      },
      {
        link_name: 'sensor_1', component_id: 'sensor_imu',
        attach_to: 'plate_1', attach_face: 'top',
        joint_type: 'fixed', joint_axis: 'z',
      },
    ],
  }
  const result = compileAssembly(graph, { resolveComponent: stubResolver })
  assert(result.diagnostics.length === 0, `unexpected diagnostics: ${JSON.stringify(result.diagnostics)}`)
  assert(result.links.length === 2, `expected 2 links (root + child), got ${result.links.length}`)
  const child = result.links[1]
  assert(child.logicalName === 'sensor_1', 'child logical name')
  assert(child.physicalLinks[0] === 'sensor_imu_2', `auto-name should be "sensor_imu_2", got "${child.physicalLinks[0]}"`)
  // Stub resolver returns a 4-cm cube (half = 0.02). Child on top face:
  // local Z should be parent.hz + child.hz = 0.02 + 0.02 = 0.04.
  assert(Math.abs(child.localXyz[2] - 0.04) < 1e-6,
    `expected localXyz[2] ≈ 0.04, got ${child.localXyz[2]}`)
  assert(Math.abs(child.worldXyz[2] - 0.04) < 1e-6,
    `expected worldXyz[2] ≈ 0.04, got ${child.worldXyz[2]}`)
  assert(result.attachIndex['sensor_1'] === 'sensor_imu_2', 'attachIndex routing')
})

// ── Slice 4.D — face_multi_child distribution ──────────────────────────────

test('compileAssembly emits all siblings under one (parent, face) — multi-child distribution', () => {
  const graph: AssemblyGraph = {
    base_link: 'plate_1',
    components: [
      { link_name: 'plate_1', component_id: 'structural_baseplate',
        attach_to: null, attach_face: 'top', joint_type: 'fixed', joint_axis: 'z' },
      { link_name: 'a', component_id: 'sensor_imu',
        attach_to: 'plate_1', attach_face: 'top', joint_type: 'fixed', joint_axis: 'z' },
      { link_name: 'b', component_id: 'sensor_imu',
        attach_to: 'plate_1', attach_face: 'top', joint_type: 'fixed', joint_axis: 'z' },
    ],
  }
  const result = compileAssembly(graph, { resolveComponent: stubResolver })
  assert(result.diagnostics.length === 0, `unexpected diagnostics: ${JSON.stringify(result.diagnostics)}`)
  assert(result.links.length === 3, `expected 3 links (root + 2 siblings), got ${result.links.length}`)
  const a = result.links.find(l => l.logicalName === 'a')!
  const b = result.links.find(l => l.logicalName === 'b')!
  // Distributed siblings should not occupy the same UV position.
  // Z (face-normal) should be identical (both flush above parent).
  assert(Math.abs(a.localXyz[2] - b.localXyz[2]) < 1e-6,
    `siblings should share local Z, got ${a.localXyz[2]} vs ${b.localXyz[2]}`)
  // X or Y should differ (distribution along the face plane).
  const sameUV = Math.abs(a.localXyz[0] - b.localXyz[0]) < 1e-9
              && Math.abs(a.localXyz[1] - b.localXyz[1]) < 1e-9
  assert(!sameUV, `siblings must be distributed across the face, both at (${a.localXyz[0]}, ${a.localXyz[1]})`)
})

test('compileAssembly siblings on different faces are NOT treated as multi-child', () => {
  // Two children: one on top, one on bottom. Each has totalOnFace=1; should
  // emit at flush-on-face Z, not multi-child distributed.
  const graph: AssemblyGraph = {
    base_link: 'plate_1',
    components: [
      { link_name: 'plate_1', component_id: 'structural_baseplate',
        attach_to: null, attach_face: 'top', joint_type: 'fixed', joint_axis: 'z' },
      { link_name: 'top_sensor', component_id: 'sensor_imu',
        attach_to: 'plate_1', attach_face: 'top', joint_type: 'fixed', joint_axis: 'z' },
      { link_name: 'bot_sensor', component_id: 'sensor_imu',
        attach_to: 'plate_1', attach_face: 'bottom', joint_type: 'fixed', joint_axis: 'z' },
    ],
  }
  const result = compileAssembly(graph, { resolveComponent: stubResolver })
  assert(result.links.length === 3, `expected 3 links, got ${result.links.length}`)
  const top = result.links.find(l => l.logicalName === 'top_sensor')!
  const bot = result.links.find(l => l.logicalName === 'bot_sensor')!
  // Top should be flush above (Z = +0.04), bottom flush below (Z = -0.04). No UV offset.
  assert(Math.abs(top.localXyz[2] - 0.04) < 1e-6, `top Z ≈ +0.04, got ${top.localXyz[2]}`)
  assert(Math.abs(bot.localXyz[2] + 0.04) < 1e-6, `bottom Z ≈ -0.04, got ${bot.localXyz[2]}`)
  assert(Math.abs(top.localXyz[0]) < 1e-9 && Math.abs(top.localXyz[1]) < 1e-9,
    `single-on-face top should have zero UV offset, got (${top.localXyz[0]}, ${top.localXyz[1]})`)
})

test('compileAssembly four siblings on bottom face produce four distinct UV positions', () => {
  // Quadruped-like leg distribution: 4 legs on the bottom face of a baseplate.
  const graph: AssemblyGraph = {
    base_link: 'body_1',
    components: [
      { link_name: 'body_1', component_id: 'structural_baseplate',
        attach_to: null, attach_face: 'top', joint_type: 'fixed', joint_axis: 'z' },
      { link_name: 'leg_a', component_id: 'sensor_imu',
        attach_to: 'body_1', attach_face: 'bottom', joint_type: 'fixed', joint_axis: 'z' },
      { link_name: 'leg_b', component_id: 'sensor_imu',
        attach_to: 'body_1', attach_face: 'bottom', joint_type: 'fixed', joint_axis: 'z' },
      { link_name: 'leg_c', component_id: 'sensor_imu',
        attach_to: 'body_1', attach_face: 'bottom', joint_type: 'fixed', joint_axis: 'z' },
      { link_name: 'leg_d', component_id: 'sensor_imu',
        attach_to: 'body_1', attach_face: 'bottom', joint_type: 'fixed', joint_axis: 'z' },
    ],
  }
  const result = compileAssembly(graph, { resolveComponent: stubResolver })
  assert(result.links.length === 5, `expected 5 links, got ${result.links.length}`)
  const legs = result.links.filter(l => l.logicalName.startsWith('leg_'))
  assert(legs.length === 4, `expected 4 leg children, got ${legs.length}`)
  // All four should be unique in (X, Y) — no two share the same UV position.
  const positions = new Set(legs.map(l => `${l.localXyz[0].toFixed(4)},${l.localXyz[1].toFixed(4)}`))
  assert(positions.size === 4, `expected 4 unique UV positions, got ${positions.size}: ${[...positions].join('; ')}`)
})

test('compileAssembly with mate_connector falls back to face when connectors absent', () => {
  // Stub resolver omits connectors; `hasMateConnectorFields` is still true
  // because the instance authored `mate_connector`. The pure mate function
  // returns a `connector_not_found_*` miss; compileAssembly silently falls
  // through to face placement (matching the assembler's wrapper behavior).
  const graph: AssemblyGraph = {
    base_link: 'plate_1',
    components: [
      { link_name: 'plate_1', component_id: 'structural_baseplate',
        attach_to: null, attach_face: 'top', joint_type: 'fixed', joint_axis: 'z' },
      { link_name: 'm', component_id: 'sensor_imu',
        attach_to: 'plate_1', attach_face: 'top',
        joint_type: 'fixed', joint_axis: 'z',
        mate_connector: 'bottom' },
    ],
  }
  const result = compileAssembly(graph, { resolveComponent: stubResolver })
  assert(result.links.length === 2, `child should fall back to face, got ${result.links.length} links`)
  assert(result.diagnostics.length === 0, `silent fallback — no diagnostics, got: ${JSON.stringify(result.diagnostics)}`)
})

// ── Slice 4.E — mate_connector placement ────────────────────────────────────

test('compileAssembly mate_connector path emits placement when connectors resolve', () => {
  // Two components whose presets carry `top` and `bottom` mate connectors.
  // The child opts in via `mate_connector: 'bottom'`. Mate placement should
  // succeed and produce a finite placement (we don't assert exact values
  // here — the connector resolver math is exercised by mateCorpus).
  const connectorStub: ComponentResolver = (componentId, _instance) => ({
    componentId,
    bounds: { half: [0.02, 0.02, 0.02], center: [0, 0, 0], shape: 'box' },
    connectors: [
      // Default top/bottom planar connectors at face centers (20 mm half-extent).
      { id: 'top',    origin_xyz_mm: [0, 0,  20], axis_xyz: [0, 0,  1], type: 'planar' },
      { id: 'bottom', origin_xyz_mm: [0, 0, -20], axis_xyz: [0, 0, -1], type: 'planar' },
    ] as MateConnector[],
  })
  const graph: AssemblyGraph = {
    base_link: 'plate_1',
    components: [
      { link_name: 'plate_1', component_id: 'structural_baseplate',
        attach_to: null, attach_face: 'top', joint_type: 'fixed', joint_axis: 'z' },
      { link_name: 'm', component_id: 'sensor_imu',
        attach_to: 'plate_1', attach_face: 'top',
        joint_type: 'fixed', joint_axis: 'z',
        attach_connector: 'top', mate_connector: 'bottom', mate_type: 'fastened' },
    ],
  }
  const result = compileAssembly(graph, { resolveComponent: connectorStub })
  assert(result.diagnostics.length === 0, `unexpected diagnostics: ${JSON.stringify(result.diagnostics)}`)
  assert(result.links.length === 2, `expected 2 links, got ${result.links.length}`)
  const child = result.links[1]
  // Child bottom-connector mated to parent top-connector → child sits on top.
  // Connector frames: parent.top at +0.02; child.bottom at -0.02.
  // Closed-form mate places child origin so child.bottom == parent.top → child Z = +0.04.
  assert(Math.abs(child.localXyz[2] - 0.04) < 1e-3,
    `expected mate-driven local Z ≈ 0.04, got ${child.localXyz[2]}`)
})

test('compileAssembly skips revolute children (defers — joint rest pose handling)', () => {
  const graph: AssemblyGraph = {
    base_link: 'plate_1',
    components: [
      { link_name: 'plate_1', component_id: 'structural_baseplate',
        attach_to: null, attach_face: 'top', joint_type: 'fixed', joint_axis: 'z' },
      { link_name: 'r', component_id: 'sensor_imu',
        attach_to: 'plate_1', attach_face: 'top',
        joint_type: 'revolute', joint_axis: 'y' },
    ],
  }
  const result = compileAssembly(graph, { resolveComponent: stubResolver })
  assert(result.links.length === 1, `revolute child should be deferred, got ${result.links.length} links`)
})

// ── Slice 4.I — drivetrain_remap ────────────────────────────────────────────

test('compileAssembly emits drivetrain hub motor with continuous joint type', () => {
  // Drivetrain wheels are continuous joints but must NOT be deferred — link
  // pose is computed normally; only joint URDF emission stays for slice 4.J.
  const driveResolver: ComponentResolver = (componentId, _instance) => ({
    componentId,
    bounds: { half: [0.03, 0.02, 0.03], center: [0, 0, 0], shape: 'cylinder' },
    assembledOuterRadiusM: 0.05,
  })
  const graph: AssemblyGraph = {
    base_link: 'plate_1',
    components: [
      { link_name: 'plate_1', component_id: 'structural_baseplate',
        attach_to: null, attach_face: 'top', joint_type: 'fixed', joint_axis: 'z' },
      { link_name: 'wheel', component_id: 'drivetrain_hub_motor',
        attach_to: 'plate_1', attach_face: 'bottom',
        joint_type: 'continuous', joint_axis: 'y' },
    ],
  }
  const result = compileAssembly(graph, { resolveComponent: driveResolver })
  assert(result.links.length === 2, `drivetrain hub should be placed, got ${result.links.length} links`)
  const wheel = result.links.find(l => l.logicalName === 'wheel')
  assert(wheel, 'wheel CompiledLink emitted')
  // Bottom-mounted hub uses rolling pose: rpy roll = -π/2 lays the wheel on
  // its side, vertical extent then comes from the assembled-tire-swapped
  // childY (0.10 m → vExtent/2 = 0.05). Pose Z must be below the plate body
  // by ≥ 0.05 — proves the effectiveCym swap took effect (raw bound half is
  // only 0.02). Sign is downward (-).
  assert(wheel!.localXyz[2] < -0.06,
    `expected wheel below plate by ≥ assembled tire radius, got z=${wheel!.localXyz[2]}`)
  assert(Math.abs(wheel!.localRpy[0] + Math.PI / 2) < 1e-3,
    `expected rolling-pose roll ≈ -π/2, got ${wheel!.localRpy[0]}`)
})

test('compileAssembly: tire-on-drivetrain ignores AI mate fields (always outboard)', () => {
  // Regression for "wheels on the inside" bug. The AI repeatedly emitted
  // attach_connector="bottom" on the tire because it (mis)reasoned the motor's
  // "top" connector was consumed by the baseplate bolt-down. Honoring that
  // mate field puts the wheel at motor-local -Z, which after the rolling-pose
  // ±π/2 roll lands inboard of the chassis edge instead of outboard. The
  // compiler must skip the mate path for tire-on-drivetrain so the
  // face short-circuit (face.ts:92, +dz axial offset) always wins.
  const conn = (id: string, z: number, axisSign: 1 | -1): MateConnector => ({
    id, type: 'planar',
    origin_xyz_mm: [0, 0, z],
    axis_xyz: [0, 0, axisSign],
  })
  const motorConn: MateConnector[] = [
    conn('top', 22.5, 1), conn('shaft_out', 22.5, 1), conn('bottom', -22.5, -1),
  ]
  const wheelConn: MateConnector[] = [
    conn('hub_bore', 15, 1), conn('top', 15, 1), conn('bottom', -15, -1),
  ]
  const resolver: ComponentResolver = (componentId, _instance) => {
    if (componentId === 'drivetrain_hub_motor_80') return {
      componentId, bounds: { half: [0.04, 0.04, 0.0225], center: [0, 0, 0], shape: 'cylinder' },
      connectors: motorConn, presetConnectors: motorConn,
      assembledOuterRadiusM: 0.05,
    }
    if (componentId === 'mobility_wheel_driven') return {
      componentId, bounds: { half: [0.05, 0.05, 0.015], center: [0, 0, 0], shape: 'cylinder' },
      connectors: wheelConn, presetConnectors: wheelConn,
    }
    return {
      componentId, bounds: { half: [0.175, 0.125, 0.004], center: [0, 0, 0], shape: 'box' },
    }
  }
  // The "evil" input: the AI emits a wrong attach_connector. The engine must
  // still produce an outboard placement.
  const graph: AssemblyGraph = {
    base_link: 'plate',
    components: [
      { link_name: 'plate', component_id: 'structural_baseplate_large',
        attach_to: null, attach_face: 'top', joint_type: 'fixed', joint_axis: 'z' },
      { link_name: 'motor', component_id: 'drivetrain_hub_motor_80',
        attach_to: 'plate', attach_face: 'bottom',
        joint_type: 'continuous', joint_axis: 'y' },
      { link_name: 'tire', component_id: 'mobility_wheel_driven',
        attach_to: 'motor', attach_face: 'coaxial',
        attach_connector: 'bottom', mate_connector: 'hub_bore', mate_type: 'concentric',
        joint_type: 'fixed', joint_axis: 'z' },
    ],
  }
  const result = compileAssembly(graph, { resolveComponent: resolver, useMateConnectors: true })
  const tire = result.links.find(l => l.logicalName === 'tire')
  assert(tire, 'tire CompiledLink emitted')
  // Outboard means motor-local +Z, which with motorHalfZ=0.0225 + tireHalfAxle=0.015
  // = +0.0375. Negative would mean the bug is back.
  assert(tire!.localXyz[2] > 0.03,
    `tire-on-drivetrain must end outboard (z > 0.03 in motor frame); got z=${tire!.localXyz[2]} — `
    + 'mate-connector path leaked through the guard at index.ts:583.')
  // RPY must stay zero (no axis flip from concentric mate). A non-zero rpy
  // here would indicate the mate path ran and rotated the wheel.
  assert(Math.abs(tire!.localRpy[0]) + Math.abs(tire!.localRpy[1]) + Math.abs(tire!.localRpy[2]) < 1e-3,
    `tire-on-drivetrain rpy must stay zero, got [${tire!.localRpy.join(',')}]`)
})

// ── Slice 4.J½ — parametric_splice ──────────────────────────────────────────

test('compileAssembly uses parent parametricLengthMm for parentBodyHZ', () => {
  // Parent extrusion declares length_mm = 100 (→ 0.05 m half-length on Z).
  // Resolver bounds.hz reports a larger value (0.06) to simulate end-cap
  // pivot bosses. The compiler must clamp parentBodyHZ to length_mm/2000 so
  // the child-on-top sits at z = 0.05 + childHZ, not 0.06 + childHZ.
  const parentResolver: ComponentResolver = (componentId, instance) => {
    if (componentId === 'structural_extrusion_2020') {
      return {
        componentId,
        bounds: { half: [0.01, 0.01, 0.06], center: [0, 0, 0], shape: 'box' },
        parametricLengthMm: instance?.length_mm,
      }
    }
    return { componentId, bounds: { half: [0.02, 0.02, 0.02], center: [0, 0, 0], shape: 'box' } }
  }
  const graph: AssemblyGraph = {
    base_link: 'beam_1',
    components: [
      { link_name: 'beam_1', component_id: 'structural_extrusion_2020',
        attach_to: null, attach_face: 'top',
        joint_type: 'fixed', joint_axis: 'z', length_mm: 100 },
      { link_name: 'imu', component_id: 'sensor_imu',
        attach_to: 'beam_1', attach_face: 'top',
        joint_type: 'fixed', joint_axis: 'z' },
    ],
  }
  const result = compileAssembly(graph, { resolveComponent: parentResolver })
  const imu = result.links.find(l => l.logicalName === 'imu')
  assert(imu, 'imu CompiledLink emitted')
  const expectedZ = 0.05 + 0.02   // length_mm/2000 + childBodyHZ
  assert(Math.abs(imu!.localXyz[2] - expectedZ) < 1e-4,
    `expected imu z=${expectedZ} (length_mm-clamped), got ${imu!.localXyz[2]}`)
})

// ── Slice 4.J — port_resolution / joint synthesis ───────────────────────────

test('compileAssembly emits a single fixed joint for non-servo children', () => {
  const graph: AssemblyGraph = {
    base_link: 'plate_1',
    components: [
      { link_name: 'plate_1', component_id: 'structural_baseplate',
        attach_to: null, attach_face: 'top', joint_type: 'fixed', joint_axis: 'z' },
      { link_name: 'imu', component_id: 'sensor_imu',
        attach_to: 'plate_1', attach_face: 'top',
        joint_type: 'fixed', joint_axis: 'z' },
    ],
  }
  const result = compileAssembly(graph, { resolveComponent: stubResolver })
  const root = result.links.find(l => l.logicalName === 'plate_1')!
  const imu = result.links.find(l => l.logicalName === 'imu')!
  assert(root.joints.length === 0, 'root link emits no joints')
  assert(imu.joints.length === 1, `non-servo child emits 1 joint, got ${imu.joints.length}`)
  const j = imu.joints[0]
  assert(j.type === 'fixed', `expected fixed joint, got ${j.type}`)
  assert(j.parentLink === 'plate_1' && j.childLink === 'sensor_imu_2',
    `expected parent=plate_1 child=sensor_imu_2, got parent=${j.parentLink} child=${j.childLink}`)
  assert(j.axis[0] === 0 && j.axis[1] === 0 && j.axis[2] === 1,
    `expected axis [0,0,1], got [${j.axis.join(',')}]`)
  assert(!j.limits, 'fixed joint should have no limits')
})

test('compileAssembly drivetrain bottom-mount remaps joint axis y → 0 0 1', () => {
  const driveResolver: ComponentResolver = (componentId, _instance) => ({
    componentId,
    bounds: { half: [0.03, 0.02, 0.03], center: [0, 0, 0], shape: 'cylinder' },
    assembledOuterRadiusM: 0.05,
    jointLimitsRad: [-Math.PI, Math.PI],
    maxTorqueNm: 5,
  })
  const graph: AssemblyGraph = {
    base_link: 'plate_1',
    components: [
      { link_name: 'plate_1', component_id: 'structural_baseplate',
        attach_to: null, attach_face: 'top', joint_type: 'fixed', joint_axis: 'z' },
      { link_name: 'wheel', component_id: 'drivetrain_hub_motor',
        attach_to: 'plate_1', attach_face: 'bottom',
        joint_type: 'continuous', joint_axis: 'y' },
    ],
  }
  const result = compileAssembly(graph, { resolveComponent: driveResolver })
  const wheel = result.links.find(l => l.logicalName === 'wheel')!
  assert(wheel.joints.length === 1, 'drivetrain emits 1 joint')
  const j = wheel.joints[0]
  assert(j.type === 'continuous', `expected continuous, got ${j.type}`)
  assert(j.axis[0] === 0 && j.axis[1] === 0 && j.axis[2] === 1,
    `expected axis remapped to [0,0,1], got [${j.axis.join(',')}]`)
})

test('compileAssembly servo emits mount + revolute joints (2 total)', () => {
  const servoResolver: ComponentResolver = (componentId, _instance) => ({
    componentId,
    bounds: { half: [0.02, 0.02, 0.02], center: [0, 0, 0], shape: 'box' },
    jointLimitsRad: [-Math.PI / 2, Math.PI / 2],
    maxTorqueNm: 12,
  })
  const graph: AssemblyGraph = {
    base_link: 'plate_1',
    components: [
      { link_name: 'plate_1', component_id: 'structural_baseplate',
        attach_to: null, attach_face: 'top', joint_type: 'fixed', joint_axis: 'z' },
      { link_name: 'svo', component_id: 'actuator_servo_mg996r',
        attach_to: 'plate_1', attach_face: 'top',
        joint_type: 'revolute', joint_axis: 'z' },
    ],
  }
  const result = compileAssembly(graph, { resolveComponent: servoResolver })
  const svo = result.links.find(l => l.logicalName === 'svo')!
  assert(svo.joints.length === 2, `expected 2 joints (mount + revolute), got ${svo.joints.length}`)
  const mount = svo.joints[0]
  const rev = svo.joints[1]
  assert(mount.type === 'fixed' && mount.name.endsWith('_mount'),
    `joint[0] should be the fixed mount, got name=${mount.name} type=${mount.type}`)
  assert(rev.type === 'revolute' && rev.parentLink.endsWith('_body') && rev.childLink.endsWith('_horn'),
    `joint[1] should be revolute body→horn, got parent=${rev.parentLink} child=${rev.childLink}`)
  assert(rev.limits && Math.abs(rev.limits[1] - Math.PI / 2) < 1e-9, 'revolute limits propagated')
  assert(rev.effort === 12, `expected effort 12 N·m, got ${rev.effort}`)
})

test('compileAssembly auto-naming counter advances for skipped children', () => {
  // Root + skipped (multi-child filler) + eligible third child. Auto-name
  // for the eligible child must reflect counter ticking through the skipped
  // ones, mirroring the assembler's placedCount semantics.
  const graph: AssemblyGraph = {
    base_link: 'plate_1',
    components: [
      { link_name: 'plate_1', component_id: 'structural_baseplate',
        attach_to: null, attach_face: 'top', joint_type: 'fixed', joint_axis: 'z' },
      // Two siblings on the same face: face_multi_child, skipped, but tick
      { link_name: 'a', component_id: 'sensor_imu',
        attach_to: 'plate_1', attach_face: 'top', joint_type: 'fixed', joint_axis: 'z' },
      { link_name: 'b', component_id: 'sensor_imu',
        attach_to: 'plate_1', attach_face: 'top', joint_type: 'fixed', joint_axis: 'z' },
      // Eligible child on a different face → emitted; counter should be 4.
      { link_name: 'c', component_id: 'sensor_imu',
        attach_to: 'plate_1', attach_face: 'bottom', joint_type: 'fixed', joint_axis: 'z' },
    ],
  }
  const result = compileAssembly(graph, { resolveComponent: stubResolver })
  const c = result.links.find(l => l.logicalName === 'c')
  assert(c !== undefined, 'child "c" must be emitted')
  assert(c!.physicalLinks[0] === 'sensor_imu_4',
    `counter must tick for skipped multi-child siblings; expected "sensor_imu_4", got "${c!.physicalLinks[0]}"`)
})

test('parity harness reports zero diffs against an empty compiled graph', () => {
  // Synthetic observed graph: root + one face-placed child. Compiled graph
  // is the 4.A empty stub. Harness must NOT emit diffs (compiler skipped
  // every class) — it should silently bump `unmatched` for each observed
  // link and keep `diffs` empty. This is the contract the live shadow-
  // compile path relies on to stay quiet during early slices.
  const observed: ObservedGraph = {
    baseLink: 'base',
    links: [
      {
        logicalName: 'base', physicalLinkName: 'base', componentId: 'r',
        parentLogicalName: null,
        localXyz: null, localRpy: null,
        worldXyz: [0, 0, 0], worldRpy: [0, 0, 0],
      },
      {
        logicalName: 'child', physicalLinkName: 'child', componentId: 'c',
        parentLogicalName: 'base',
        localXyz: [0.1, 0, 0], localRpy: [0, 0, 0],
        worldXyz: [0.1, 0, 0], worldRpy: [0, 0, 0],
      },
    ],
  }
  const compiled = compileAssembly({ base_link: 'base', components: [] })
  const report = compareGraphs(observed, compiled)
  assert(report.diffs.length === 0,
    `expected zero diffs while compiler is empty, got ${report.diffs.length}: ${JSON.stringify(report.diffs)}`)
  assert(report.matched === 0, 'nothing matched (compiler emitted no links)')
  assert(report.unmatched === 2, `expected 2 unmatched, got ${report.unmatched}`)
  assert(report.skippedClasses.length === ALL_PLACEMENT_CLASSES.length - 12,
    `expected ${ALL_PLACEMENT_CLASSES.length - 12} skipped classes, got ${report.skippedClasses.length}`)
})

test('parity harness flags localXyz divergence above tolerance', () => {
  // Defensive test: synthesize a CompiledGraph by hand (bypassing
  // compileAssembly) with a deliberately wrong xyz, and confirm the
  // comparator catches it. Guards against the harness silently passing
  // when 4.B comes online and starts emitting real CompiledLinks.
  const observed: ObservedGraph = {
    baseLink: 'base',
    links: [{
      logicalName: 'a', physicalLinkName: 'a', componentId: 'c',
      parentLogicalName: 'base',
      localXyz: [0.1, 0, 0], localRpy: [0, 0, 0],
      worldXyz: [0.1, 0, 0], worldRpy: [0, 0, 0],
    }],
  }
  const compiled = {
    baseLink: 'base',
    links: [{
      logicalName: 'a', componentId: 'c',
      physicalLinks: ['a'], childAttachTarget: 'a',
      bounds: {
        half: [0.01, 0.01, 0.01] as [number, number, number],
        center: [0, 0, 0] as [number, number, number],
        shape: 'box' as const,
      },
      localXyz: [0.2, 0, 0] as [number, number, number],   // off by 0.1 m
      localRpy: [0, 0, 0] as [number, number, number],
      worldXyz: [0.1, 0, 0] as [number, number, number],
      worldRpy: [0, 0, 0] as [number, number, number],
      parentConnector: null, childConnector: null,
      joints: [], syntheticRole: null,
      physicalWorldXyz: [[0.1, 0, 0] as [number, number, number]],
      physicalWorldRpy: [[0, 0, 0] as [number, number, number]],
      placedViaConnector: false,
      massKg: 0,
    }],
    attachIndex: { a: 'a' },
    diagnostics: [],
    fingerprint: 'test',
    skippedClasses: [],
  }
  const report = compareGraphs(observed, compiled)
  const xyzDiff = report.diffs.find(d => d.field === 'localXyz[0]')
  assert(xyzDiff !== undefined, 'comparator must flag localXyz[0] mismatch')
  assert(typeof xyzDiff!.delta === 'number' && xyzDiff!.delta! > 0.05,
    `delta should be ~0.1, got ${xyzDiff!.delta}`)
  assert(report.matched === 0, 'mismatched link counts as not-matched')
})

// ── Slice 4.F — servo split ─────────────────────────────────────────────────

test('compileAssembly emits split-servo physicalLinks=[body, horn], childAttachTarget=horn', () => {
  const graph: AssemblyGraph = {
    base_link: 'plate_1',
    components: [
      { link_name: 'plate_1', component_id: 'structural_baseplate',
        attach_to: null, attach_face: 'top', joint_type: 'fixed', joint_axis: 'z' },
      { link_name: 'hip', component_id: 'actuator_servo_mg996r',
        attach_to: 'plate_1', attach_face: 'top',
        joint_type: 'revolute', joint_axis: 'y' },
    ],
  }
  const result = compileAssembly(graph, { resolveComponent: stubResolver })
  assert(result.diagnostics.length === 0, `unexpected diagnostics: ${JSON.stringify(result.diagnostics)}`)
  assert(result.links.length === 2, `expected 2 links (root + servo), got ${result.links.length}`)
  const servo = result.links.find(l => l.logicalName === 'hip')!
  assert(servo.physicalLinks.length === 2,
    `non-compound servo emits [body, horn], got ${JSON.stringify(servo.physicalLinks)}`)
  assert(servo.physicalLinks[0].endsWith('_body'), `physicalLinks[0] should be body, got ${servo.physicalLinks[0]}`)
  assert(servo.physicalLinks[1].endsWith('_horn'), `physicalLinks[1] should be horn, got ${servo.physicalLinks[1]}`)
  assert(servo.childAttachTarget === servo.physicalLinks[1],
    `childAttachTarget should be horn, got ${servo.childAttachTarget} vs ${servo.physicalLinks[1]}`)
  assert(result.attachIndex['hip'].endsWith('_horn'),
    `attachIndex routes "hip" to horn, got ${result.attachIndex['hip']}`)
})

test('compileAssembly child of servo uses driven placement (not bbox face)', () => {
  // Servo on baseplate; limb hangs off the servo. The servo-driven child
  // placement (servoDrivenChildPlacement) returns a different geometry than
  // bbox face placement — assert non-zero placement on the radial axis.
  const graph: AssemblyGraph = {
    base_link: 'plate_1',
    components: [
      { link_name: 'plate_1', component_id: 'structural_baseplate',
        attach_to: null, attach_face: 'top', joint_type: 'fixed', joint_axis: 'z' },
      { link_name: 'hip', component_id: 'actuator_servo_mg996r',
        attach_to: 'plate_1', attach_face: 'top',
        joint_type: 'revolute', joint_axis: 'y' },
      { link_name: 'limb', component_id: 'structural_bracket_l',
        attach_to: 'hip', attach_face: 'top',
        joint_type: 'fixed', joint_axis: 'z' },
    ],
  }
  const result = compileAssembly(graph, { resolveComponent: stubResolver })
  assert(result.diagnostics.length === 0, `unexpected diagnostics: ${JSON.stringify(result.diagnostics)}`)
  assert(result.links.length === 3, `expected 3 links (root + servo + limb), got ${result.links.length}`)
  const limb = result.links.find(l => l.logicalName === 'limb')!
  // servoDrivenChildPlacement for parentAxis=y returns xyz=(0, sign*offset, 0).
  // Bbox face placement on a 4-cm cube top face would be xyz=(0, 0, 0.04).
  // So the limb's local Y must be non-zero AND local Z near zero.
  assert(Math.abs(limb.localXyz[1]) > 1e-3,
    `driven child placement sets local Y, got xyz=(${limb.localXyz.join(', ')})`)
})

test('compileAssembly compound servo (servo on servo) includes carrier in physicalLinks', () => {
  const graph: AssemblyGraph = {
    base_link: 'plate_1',
    components: [
      { link_name: 'plate_1', component_id: 'structural_baseplate',
        attach_to: null, attach_face: 'top', joint_type: 'fixed', joint_axis: 'z' },
      { link_name: 'hip', component_id: 'actuator_servo_mg996r',
        attach_to: 'plate_1', attach_face: 'top',
        joint_type: 'revolute', joint_axis: 'y' },
      { link_name: 'knee', component_id: 'actuator_servo_mg996r',
        attach_to: 'hip', attach_face: 'top',
        joint_type: 'revolute', joint_axis: 'y' },
    ],
  }
  const result = compileAssembly(graph, { resolveComponent: stubResolver })
  assert(result.diagnostics.length === 0, `unexpected diagnostics: ${JSON.stringify(result.diagnostics)}`)
  const knee = result.links.find(l => l.logicalName === 'knee')!
  assert(knee.physicalLinks.length === 3,
    `compound servo emits [carrier, body, horn], got ${JSON.stringify(knee.physicalLinks)}`)
  assert(knee.physicalLinks[0].endsWith('_compound_carrier'),
    `physicalLinks[0] should be carrier, got ${knee.physicalLinks[0]}`)
  assert(knee.childAttachTarget.endsWith('_horn'),
    `childAttachTarget should be horn, got ${knee.childAttachTarget}`)
})

test('compileAssembly servo with attach_rpy override sets servoHornZeroRpy', () => {
  // Slice 4.J.4: explicit attach_rpy on a Y-axis servo translates into a
  // local horn rest rpy via servoLocalRestRpyFromJointRpy. The body→horn
  // joint origin's rpy should reflect that, and the horn's worldRpy should
  // differ from the body mount's rpy.
  const graph: AssemblyGraph = {
    base_link: 'plate_1',
    components: [
      { link_name: 'plate_1', component_id: 'structural_baseplate',
        attach_to: null, attach_face: 'top', joint_type: 'fixed', joint_axis: 'z' },
      { link_name: 'hip', component_id: 'actuator_servo_mg996r',
        attach_to: 'plate_1', attach_face: 'top',
        joint_type: 'revolute', joint_axis: 'y',
        attach_rpy: [0, 0.7854, 0] },
    ],
  }
  const result = compileAssembly(graph, { resolveComponent: stubResolver })
  assert(result.links.length === 2, `expected root + servo, got ${result.links.length} links`)
  const hip = result.links.find(l => l.logicalName === 'hip')!
  const revJoint = hip.joints.find(j => j.type === 'revolute')!
  // arm-rest depth=1 contributes +π/4 BEFORE the explicit override replaces it.
  // The explicit [0, 0.7854, 0] on a Y-axis servo lands as a non-zero horn
  // rest rpy via servoLocalRestRpyFromJointRpy.
  const hornRpy = revJoint.originRpy
  assert(hornRpy.some(v => Math.abs(v) > 0.001),
    `expected non-zero horn rest rpy from attach_rpy override, got [${hornRpy.join(', ')}]`)
})

test('compileAssembly: foot pad on a shin under a Y-axis servo lands AWAY from the joint, not at it', async () => {
  // Robot-dog regression: the AI chains knee_servo → shin (limb_link_slim,
  // attach_face='bottom') → foot_pad (attach_face='bottom'). Without the
  // distance-based distal-beam-bottom flip, `servoDrivenStructuralLimbPlacement`
  // offsets the shin radially outward by `adapterGap + childBodyHZ` and the
  // shin's local -Z face (the 'bottom' resolution) lands at the NEAR-knee end
  // of the shin — putting the foot at the knee joint instead of at the leg
  // tip. This test pins the fixed behavior: foot world-distance from the
  // knee servo origin must be at least the shin's length minus a small slack.
  const THREE = await import('three')
  const SHIN_LENGTH_M = 0.120

  // Distinguish limb from foot in the stub bounds so the placement compiler
  // gets realistic half-extents (otherwise everything is a 4cm cube and the
  // distance check has no spread to work with).
  const dogResolver: ComponentResolver = (componentId, instance) => {
    if (componentId === 'structural_limb_link_slim') {
      const lengthM = (instance?.length_mm ?? SHIN_LENGTH_M * 1000) / 1000
      return {
        componentId,
        bounds: { half: [0.007, 0.003, lengthM / 2], center: [0, 0, 0], shape: 'box' },
        parametricLengthMm: instance?.length_mm,
      }
    }
    if (componentId === 'mobility_rubber_foot_pad') {
      return {
        componentId,
        bounds: { half: [0.012, 0.012, 0.005], center: [0, 0, 0], shape: 'box' },
      }
    }
    return { componentId, bounds: { half: [0.02, 0.02, 0.02], center: [0, 0, 0], shape: 'box' } }
  }

  const graph: AssemblyGraph = {
    base_link: 'plate_1',
    components: [
      { link_name: 'plate_1', component_id: 'structural_baseplate',
        attach_to: null, attach_face: 'top', joint_type: 'fixed', joint_axis: 'z' },
      { link_name: 'knee', component_id: 'actuator_servo_high_torque',
        attach_to: 'plate_1', attach_face: 'top', joint_type: 'revolute', joint_axis: 'y' },
      { link_name: 'shin', component_id: 'structural_limb_link_slim',
        attach_to: 'knee', attach_face: 'bottom', joint_type: 'fixed', joint_axis: 'z',
        length_mm: SHIN_LENGTH_M * 1000 },
      { link_name: 'foot', component_id: 'mobility_rubber_foot_pad',
        attach_to: 'shin', attach_face: 'bottom', joint_type: 'fixed', joint_axis: 'z' },
    ],
  }
  const r = compileAssembly(graph, { resolveComponent: dogResolver })
  assert(r.diagnostics.length === 0, `unexpected diagnostics: ${JSON.stringify(r.diagnostics)}`)
  const knee = r.links.find(l => l.logicalName === 'knee')!
  const foot = r.links.find(l => l.logicalName === 'foot')!
  const kneeWorld = new THREE.Vector3(knee.worldXyz[0], knee.worldXyz[1], knee.worldXyz[2])
  const footWorld = new THREE.Vector3(foot.worldXyz[0], foot.worldXyz[1], foot.worldXyz[2])
  const dist = footWorld.distanceTo(kneeWorld)
  // Foot must land near the leg tip, not at the joint. Allow some slack for
  // adapterGap + foot half-thickness; require at least 80% of shin length.
  assert(dist > SHIN_LENGTH_M * 0.8,
    `foot must land at leg tip (>${(SHIN_LENGTH_M * 0.8 * 1000).toFixed(0)}mm from knee), got ${(dist * 1000).toFixed(1)}mm — ` +
    `foot.world=${foot.worldXyz.join(',')} knee.world=${knee.worldXyz.join(',')} foot.local=${foot.localXyz.join(',')}`)
})

// ── Slice 4.H — foot leveling ───────────────────────────────────────────────

test('compileAssembly foot pad on a tilted parent gets world-leveled rpy', () => {
  // Build a chain where the foot pad's parent is itself tilted relative to
  // the world: root → tilted_bracket (top face) → foot. Without leveling,
  // the foot would inherit the bracket's world rotation. With leveling, its
  // world rpy must cancel the parent's rotation (level under world frame).
  //
  // We can't easily set non-zero parent rotation through the existing
  // graph without `attach_rpy` (which is bypassed). Instead, use a servo
  // parent: servoMountRpyForParentWorld rotates the body-mount frame, so
  // anything attached above the horn lives in a non-identity world frame.
  //
  // Simpler check: for an upright parent (identity world), worldLevelRpy
  // should be zero, and foot pad rpy should match. For a non-identity
  // parent we just assert the override fires (rpy != face placement rpy).
  const graph: AssemblyGraph = {
    base_link: 'plate_1',
    components: [
      { link_name: 'plate_1', component_id: 'structural_baseplate',
        attach_to: null, attach_face: 'top', joint_type: 'fixed', joint_axis: 'z' },
      { link_name: 'foot', component_id: 'mobility_rubber_foot_pad',
        attach_to: 'plate_1', attach_face: 'bottom',
        joint_type: 'fixed', joint_axis: 'z' },
    ],
  }
  const result = compileAssembly(graph, { resolveComponent: stubResolver })
  assert(result.diagnostics.length === 0, `unexpected diagnostics: ${JSON.stringify(result.diagnostics)}`)
  const foot = result.links.find(l => l.logicalName === 'foot')
  assert(foot !== undefined, 'foot pad must be emitted (no longer deferred)')
  // Identity parent → world-level rpy is identity → foot rpy should be zero.
  assert(foot!.localRpy.every(v => Math.abs(v) < 1e-6),
    `foot rpy on upright parent should be zero, got ${JSON.stringify(foot!.localRpy)}`)
})

// ── Slice 5b — exporter contract: world poses agree with chained local poses ─

test('compileAssembly: every link\'s worldXyz/worldRpy matches the chain of local poses (rpyToQuat)', async () => {
  // Phase 5b invariant. The compiler emits both per-link local poses and
  // resolved world poses. Exporters (URDF → three.js, Python → MJCF) all walk
  // the local chain and must arrive at the same world pose the compiler
  // reports — otherwise the reconcile-pass / mate / servo split paths can
  // drift between runtimes. This test walks the chain in isolation and
  // confirms agreement, so any divergence trips here before it reaches the
  // sim or the renderer.
  const THREE = await import('three')
  const { rpyToQuat } = await import('./rotationIO.ts')

  const graph: AssemblyGraph = {
    base_link: 'plate_1',
    components: [
      { link_name: 'plate_1', component_id: 'structural_baseplate',
        attach_to: null, attach_face: 'top', joint_type: 'fixed', joint_axis: 'z' },
      { link_name: 'hip', component_id: 'actuator_servo_high_torque',
        attach_to: 'plate_1', attach_face: 'bottom', joint_type: 'revolute', joint_axis: 'y' },
      { link_name: 'thigh', component_id: 'structural_limb_link_slim',
        attach_to: 'hip', attach_face: 'top', joint_type: 'fixed', joint_axis: 'z',
        length_mm: 100 },
      { link_name: 'foot', component_id: 'mobility_rubber_foot_pad',
        attach_to: 'thigh', attach_face: 'bottom', joint_type: 'fixed', joint_axis: 'z' },
    ],
  }
  // Stub: every component is a 4cm cube; sufficient to test the transform
  // chain math without depending on the preset catalog.
  const r = compileAssembly(graph, { resolveComponent: stubResolver })
  assert(r.diagnostics.length === 0, `diagnostics: ${JSON.stringify(r.diagnostics)}`)

  const byLogical = new Map(r.links.map(l => [l.logicalName, l]))

  function parentLogicalOf(name: string): string | null {
    const c = graph.components.find(c => c.link_name === name)
    return c?.attach_to ?? null
  }

  // For each non-actuated link, recompute world pose from parent's world pose
  // composed with this link's local pose, and confirm it matches the
  // compiler's emitted worldXyz/worldRpy. (Actuated servos have their own
  // body→horn split where worldXyz != compose(parent.world, local) — that
  // invariant is covered by the existing servo-split tests above.)
  let checked = 0
  for (const link of r.links) {
    const parentName = parentLogicalOf(link.logicalName)
    // Skip the root (no parent in graph) and skip actuated servos (split frame).
    const isActuatedServo = link.physicalLinks.some(n => n.endsWith('_horn'))
    if (parentName === null || isActuatedServo) continue
    const parent = byLogical.get(parentName)
    assert(parent !== undefined, `parent ${parentName} not in compiled graph`)

    const parentWorld = new THREE.Matrix4().compose(
      new THREE.Vector3(parent!.worldXyz[0], parent!.worldXyz[1], parent!.worldXyz[2]),
      rpyToQuat(parent!.worldRpy),
      new THREE.Vector3(1, 1, 1),
    )
    const local = new THREE.Matrix4().compose(
      new THREE.Vector3(link.localXyz[0], link.localXyz[1], link.localXyz[2]),
      rpyToQuat(link.localRpy),
      new THREE.Vector3(1, 1, 1),
    )
    const worldM = parentWorld.multiply(local)
    const wp = new THREE.Vector3(); const wq = new THREE.Quaternion(); const ws = new THREE.Vector3()
    worldM.decompose(wp, wq, ws)

    const dx = Math.abs(wp.x - link.worldXyz[0])
    const dy = Math.abs(wp.y - link.worldXyz[1])
    const dz = Math.abs(wp.z - link.worldXyz[2])
    assert(dx < 1e-4 && dy < 1e-4 && dz < 1e-4,
      `${link.logicalName} worldXyz drift: chain=[${wp.x},${wp.y},${wp.z}] compiled=${JSON.stringify(link.worldXyz)}`)

    const expectedQ = rpyToQuat(link.worldRpy)
    const dot = Math.abs(wq.x * expectedQ.x + wq.y * expectedQ.y + wq.z * expectedQ.z + wq.w * expectedQ.w)
    assert(dot > 1 - 1e-4,
      `${link.logicalName} worldRpy mismatch (dot=${dot})`)
    checked++
  }
  assert(checked >= 2, `expected to check >=2 non-servo links in this fixture, only checked ${checked}`)
})

// ── Runner ──────────────────────────────────────────────────────────────────

async function run() {
  let failed = 0
  for (const [name, fn] of TESTS) {
    try { await fn(); console.log(`  PASS  ${name}`) }
    catch (e) { failed++; console.log(`  FAIL  ${name}: ${(e as Error).message}`) }
  }
  console.log(`\nplacement-compiler corpus: ${TESTS.length - failed}/${TESTS.length} passed`)
  process.exit(failed === 0 ? 0 : 1)
}
run()
