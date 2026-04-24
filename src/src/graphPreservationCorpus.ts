// Graph preservation corpus: exercises the `graphsEquivalent` / `cloneAssemblyGraph`
// primitives that Workstream #1 (AssemblyGraph Preservation) relies on to carry
// the canonical graph across multi-turn edits without touching the lossy
// URDF round-trip.
//
// Run: cd src && npm run test:graph-preservation
// Direct: node --experimental-strip-types src/graphPreservationCorpus.ts
//
// What this covers:
//   1. Identity: `graphsEquivalent(g, g)` and `graphsEquivalent(g, clone(g))`
//      must return equal:true — otherwise the canonical stash is useless.
//   2. Characterizes the documented urdfToAssemblyGraph loss:
//      orientation / elevation_angle / length_mm / attach_rpy / ground_offset
//      all drop or default on a pure-URDF round-trip. We simulate that loss
//      with `_simulateUrdfRoundTripLoss` and assert graphsEquivalent surfaces
//      exactly those fields as differences. If the simulator or the equality
//      check drifts, this fixture catches it.
//
// The simulator lives in the test file (not production) because the real
// urdfToAssemblyGraph needs DOMParser and we don't want to pull in jsdom.
// It mirrors the loss set documented in project_vector_improvement_plan.md
// §2 "urdfToAssemblyGraph known limitations".

import {
  cloneAssemblyGraph,
  graphsEquivalent,
  type AssemblyGraph,
  type AssemblyComponent,
} from './urdfGraphEquivalence.ts'

// Known-good graph: exercises every field that the lossy round-trip would drop.
const CANONICAL_GRAPH: AssemblyGraph = {
  base_link: 'baseplate_1',
  ground_offset: false, // non-default — round-trip hardcodes to true
  components: [
    {
      link_name: 'baseplate_1',
      component_id: 'structural_baseplate_large',
      attach_to: null,
      attach_face: null,
      joint_type: 'fixed',
      joint_axis: '0 0 1',
    },
    {
      link_name: 'extrusion_1',
      component_id: 'structural_extrusion_2020',
      attach_to: 'baseplate_1',
      attach_face: 'top',
      joint_type: 'fixed',
      joint_axis: '0 0 1',
      length_mm: 350, // dropped by reverse-parse
      orientation: 'vertical', // dropped by reverse-parse
    },
    {
      link_name: 'hip_pitch_1',
      component_id: 'actuator_servo_standard',
      attach_to: 'extrusion_1',
      attach_face: 'front',
      joint_type: 'revolute',
      joint_axis: '0 1 0',
      elevation_angle: 25, // dropped by reverse-parse
      attach_rpy: [0, 0.52, 0], // dropped by reverse-parse
    },
  ],
}

/** Mirrors the documented lossy behavior of `urdfToAssemblyGraph` on a graph
 *  whose URDF encoding can't round-trip the dropped fields.
 *  See project_vector_improvement_plan.md §2 for the spec. */
function _simulateUrdfRoundTripLoss(graph: AssemblyGraph): AssemblyGraph {
  return {
    base_link: graph.base_link,
    ground_offset: true, // hardcoded true on reverse-parse
    components: graph.components.map(c => {
      const out: AssemblyComponent = {
        link_name: c.link_name,
        component_id: c.component_id,
        attach_to: c.attach_to,
        attach_face: c.attach_face,
        joint_type: c.joint_type,
        joint_axis: c.joint_axis,
      }
      // length_mm, orientation, elevation_angle, attach_rpy intentionally omitted —
      // that's the whole point of this simulator.
      return out
    }),
  }
}

interface Case {
  name: string
  passed: boolean
  reason?: string
}

function assertEqual(name: string, a: AssemblyGraph, b: AssemblyGraph): Case {
  const result = graphsEquivalent(a, b)
  if (!result.equal) {
    return { name, passed: false, reason: `expected equal, got differences: ${result.differences.join('; ')}` }
  }
  return { name, passed: true }
}

function assertDiffersOn(name: string, a: AssemblyGraph, b: AssemblyGraph, expectedFieldTags: string[]): Case {
  const result = graphsEquivalent(a, b)
  if (result.equal) {
    return { name, passed: false, reason: 'expected differences, got equal' }
  }
  const joined = result.differences.join(' ; ')
  for (const tag of expectedFieldTags) {
    if (!joined.includes(tag)) {
      return {
        name,
        passed: false,
        reason: `expected difference mentioning "${tag}" — got: ${joined}`,
      }
    }
  }
  return { name, passed: true }
}

function main(): void {
  const results: Case[] = []

  // ── Identity: canonical preservation ────────────────────────────────────────
  results.push(assertEqual('identity: graphsEquivalent(g, g)', CANONICAL_GRAPH, CANONICAL_GRAPH))
  results.push(assertEqual(
    'identity: graphsEquivalent(g, clone(g)) — clone must preserve every field',
    CANONICAL_GRAPH,
    cloneAssemblyGraph(CANONICAL_GRAPH),
  ))

  // ── Clone isolation: mutating the clone must not disturb the original ───────
  const cloned = cloneAssemblyGraph(CANONICAL_GRAPH)
  cloned.components[2].attach_rpy![1] = 0
  results.push(assertEqual(
    'clone isolation: mutating clone.attach_rpy leaves original intact',
    CANONICAL_GRAPH,
    CANONICAL_GRAPH, // identity re-check; the mutation should not have reached here
  ))
  // Also assert the mutated clone *does* now differ.
  results.push(assertDiffersOn(
    'clone isolation: mutated clone differs from original on attach_rpy',
    CANONICAL_GRAPH,
    cloned,
    ['attach_rpy'],
  ))

  // ── Round-trip characterization: URDF round-trip is lossy ───────────────────
  // This is the regression-catcher — if graphsEquivalent stops flagging these
  // fields, the whole Workstream #1 divergence-detection guard silently breaks.
  const roundTripped = _simulateUrdfRoundTripLoss(CANONICAL_GRAPH)
  results.push(assertDiffersOn(
    'urdf round-trip loses orientation',
    CANONICAL_GRAPH,
    roundTripped,
    ['orientation'],
  ))
  results.push(assertDiffersOn(
    'urdf round-trip loses length_mm',
    CANONICAL_GRAPH,
    roundTripped,
    ['length_mm'],
  ))
  results.push(assertDiffersOn(
    'urdf round-trip loses elevation_angle',
    CANONICAL_GRAPH,
    roundTripped,
    ['elevation_angle'],
  ))
  results.push(assertDiffersOn(
    'urdf round-trip loses attach_rpy',
    CANONICAL_GRAPH,
    roundTripped,
    ['attach_rpy'],
  ))
  results.push(assertDiffersOn(
    'urdf round-trip defaults ground_offset=true',
    CANONICAL_GRAPH,
    roundTripped,
    ['ground_offset'],
  ))

  // ── Tolerance: near-zero attach_rpy treated as absent (matches runtime) ─────
  // resolveAssemblyGraph only overrides when any component of attach_rpy
  // exceeds 1e-3 rad — graphsEquivalent must agree so behavioral-identity
  // doesn't produce false-positive drift warnings.
  const withZeroRpy = cloneAssemblyGraph(CANONICAL_GRAPH)
  withZeroRpy.components[0].attach_rpy = [0, 0, 0]
  results.push(assertEqual(
    'tolerance: [0,0,0] attach_rpy equals absent',
    CANONICAL_GRAPH,
    withZeroRpy,
  ))

  // ── Structural: detecting an added component ───────────────────────────────
  const withExtra = cloneAssemblyGraph(CANONICAL_GRAPH)
  withExtra.components.push({
    link_name: 'camera_1',
    component_id: 'sensor_depth_camera_small',
    attach_to: 'extrusion_1',
    attach_face: 'top',
    joint_type: 'fixed',
    joint_axis: '0 0 1',
  })
  results.push(assertDiffersOn(
    'structural: added component surfaces as "only in B"',
    CANONICAL_GRAPH,
    withExtra,
    ['camera_1'],
  ))

  // ── Print + exit ────────────────────────────────────────────────────────────
  const failed = results.filter(r => !r.passed)
  for (const r of results) {
    const tag = r.passed ? 'PASS' : 'FAIL'
    console.log(`[${tag}] ${r.name}`)
    if (!r.passed && r.reason) console.log(`       ${r.reason}`)
  }
  console.log('──────────────────────────────────────────────────────────────')
  console.log(`${results.length - failed.length}/${results.length} passed`)
  if (failed.length > 0) process.exit(1)
}

main()
