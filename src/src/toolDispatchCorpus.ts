// Tool-call dispatch corpus (Workstream #2). Each fixture drives a single
// GraphMutation through the pure `applyMutation` entrypoint and asserts the
// structured tool-result — this is the "dispatch layer" counterpart to the
// pure-validator corpus in topologyCorpus.ts.
//
// Run: cd src && npm run test:tool-dispatch
// Direct: node --experimental-strip-types src/toolDispatchCorpus.ts
//
// Every new tool or structured-error code needs a pair here:
//   - a `bad` case that produces the error code (and no mutation)
//   - a `fix` case where the mutation applies cleanly
// The harness is the load-bearing regression net for the edit surface; add
// before you ship new tools or rule enforcement.

import * as fs from 'node:fs'
import * as path from 'node:path'
import { fileURLToPath } from 'node:url'

import { applyMutation } from './graphMutations.ts'
import type { GraphMutation } from './graphMutations.ts'
import type { ValidationContext, ValidationPreset } from './topologyValidation.ts'
import type { AssemblyComponent, AssemblyGraph } from './urdfGraphEquivalence.ts'

interface PresetFile {
  categories: Record<string, { components: ValidationPreset[] }>
}

function loadPresets(): ValidationContext {
  const here = path.dirname(fileURLToPath(import.meta.url))
  const presetPath = path.resolve(here, '..', 'public', 'generic_presets.json')
  const raw = fs.readFileSync(presetPath, 'utf-8')
  const data = JSON.parse(raw) as PresetFile
  const byId = new Map<string, ValidationPreset>()
  for (const cat of Object.values(data.categories)) {
    for (const c of cat.components) byId.set(c.id, c)
  }
  return { findPreset: (id: string) => byId.get(id) ?? null }
}

type ComponentInput = Partial<AssemblyComponent> & {
  link_name: string
  component_id: string
  attach_to: string | null
}

function completeComponent(c: ComponentInput): AssemblyComponent {
  return {
    link_name: c.link_name,
    component_id: c.component_id,
    attach_to: c.attach_to,
    attach_face: c.attach_face ?? null,
    joint_type: c.joint_type ?? 'fixed',
    joint_axis: c.joint_axis ?? '0 0 1',
    length_mm: c.length_mm,
    orientation: c.orientation,
    elevation_angle: c.elevation_angle,
    attach_rpy: c.attach_rpy,
  }
}

function toGraph(input: { base_link: string; components: ComponentInput[] }): AssemblyGraph {
  return { base_link: input.base_link, components: input.components.map(completeComponent) }
}

interface Fixture {
  name: string
  graph: { base_link: string; components: ComponentInput[] }
  mutation: GraphMutation
  expect:
    | { ok: true;  summaryIncludes?: string; graphContainsLink?: string }
    | { ok: false; code: string }
}

// ── Shared base graphs used across fixtures ─────────────────────────────────
const GRAPH_WITH_EXTRUSION_AND_SERVO = {
  base_link: 'base_link',
  components: [
    { link_name: 'plate',  component_id: 'structural_baseplate',         attach_to: null },
    { link_name: 'ext1',   component_id: 'structural_extrusion_2020',    attach_to: 'plate',  attach_face: 'top', length_mm: 200 },
    { link_name: 'servo1', component_id: 'actuator_servo_standard',      attach_to: 'ext1',   attach_face: 'top' },
  ] as ComponentInput[],
}

const fixtures: Fixture[] = [
  // ── add_link: valid + structural attach_face mismatch ───────────────────
  {
    name: 'add_link: structural bracket on servo top — bracket preset accepted',
    graph: GRAPH_WITH_EXTRUSION_AND_SERVO,
    mutation: {
      kind: 'add_link',
      args: {
        link_name: 'brac1', parent_link: 'servo1',
        component_id: 'structural_bracket_u', attach_face: 'top',
      },
    },
    expect: { ok: true, graphContainsLink: 'brac1' },
  },
  {
    name: 'add_link: unknown parent — UNKNOWN_PARENT',
    graph: GRAPH_WITH_EXTRUSION_AND_SERVO,
    mutation: {
      kind: 'add_link',
      args: {
        link_name: 'x', parent_link: 'does_not_exist',
        component_id: 'structural_bracket_u', attach_face: 'top',
      },
    },
    expect: { ok: false, code: 'UNKNOWN_PARENT' },
  },
  {
    name: 'add_link: unknown preset — UNKNOWN_COMPONENT',
    graph: GRAPH_WITH_EXTRUSION_AND_SERVO,
    mutation: {
      kind: 'add_link',
      args: {
        link_name: 'x', parent_link: 'ext1',
        component_id: 'not_a_real_preset', attach_face: 'top',
      },
    },
    expect: { ok: false, code: 'UNKNOWN_COMPONENT' },
  },
  {
    name: 'add_link: duplicate link_name — DUPLICATE_LINK',
    graph: GRAPH_WITH_EXTRUSION_AND_SERVO,
    mutation: {
      kind: 'add_link',
      args: {
        link_name: 'servo1', parent_link: 'ext1',
        component_id: 'structural_bracket_u', attach_face: 'top',
      },
    },
    expect: { ok: false, code: 'DUPLICATE_LINK' },
  },
  {
    name: 'add_link: servo-on-split-servo-top stack — accepted (horn routes to _output sub-link)',
    // docs/SERVO_SPLIT_PLAN.md §retire-coupler — the parent servo is a
    // split-link preset, so its `top` port is tagged subLink='output' and
    // the URDF emitter routes the child's mate onto the horn sub-link
    // directly. No coupler disc is needed; PORT_MISMATCH is skipped for
    // split-link parents.
    graph: GRAPH_WITH_EXTRUSION_AND_SERVO,
    mutation: {
      kind: 'add_link',
      args: {
        link_name: 'servo2', parent_link: 'servo1',
        component_id: 'actuator_servo_standard', attach_face: 'top',
      },
    },
    expect: { ok: true },
  },
  {
    name: 'add_link: servo on bracket (the PORT_MISMATCH escape hatch) — accepted',
    // Same servo-on-shaft intent, but with the bracket in between. The
    // helper's early-return for `structural_bracket_*` parents keeps the
    // mutation valid so Claude can fix PORT_MISMATCH via add_link(bracket)
    // before retrying the actuator attach.
    graph: {
      base_link: 'base_link',
      components: [
        { link_name: 'plate',  component_id: 'structural_baseplate',       attach_to: null },
        { link_name: 'servo1', component_id: 'actuator_servo_standard',    attach_to: 'plate',  attach_face: 'top' },
        { link_name: 'brac1',  component_id: 'structural_bracket_u',       attach_to: 'servo1', attach_face: 'top' },
      ],
    },
    mutation: {
      kind: 'add_link',
      args: {
        link_name: 'servo2', parent_link: 'brac1',
        component_id: 'actuator_servo_standard', attach_face: 'top',
      },
    },
    expect: { ok: true },
  },
  {
    name: 'add_link: normal structural chain — no PORT_MISMATCH false-fire',
    // Regression guard: when both parent and child expose mount_face on the
    // shared pair of opposite faces, the helper must pass through.
    graph: {
      base_link: 'base_link',
      components: [
        { link_name: 'plate', component_id: 'structural_baseplate', attach_to: null },
      ],
    },
    mutation: {
      kind: 'add_link',
      args: {
        link_name: 'servo1', parent_link: 'plate',
        component_id: 'actuator_servo_standard', attach_face: 'top',
      },
    },
    expect: { ok: true },
  },

  // ── attach_sensor ────────────────────────────────────────────────────────
  {
    name: 'attach_sensor: camera on extrusion side — valid',
    graph: GRAPH_WITH_EXTRUSION_AND_SERVO,
    mutation: {
      kind: 'attach_sensor',
      args: {
        link_name: 'cam1', parent_link: 'ext1',
        component_id: 'sensor_depth_camera_small', mount_face: 'front',
      },
    },
    expect: { ok: true, graphContainsLink: 'cam1' },
  },
  {
    name: 'attach_sensor: camera on servo shaft — SENSOR_ON_ACTUATOR rejects',
    graph: GRAPH_WITH_EXTRUSION_AND_SERVO,
    mutation: {
      kind: 'attach_sensor',
      args: {
        link_name: 'cam1', parent_link: 'servo1',
        component_id: 'sensor_depth_camera_small', mount_face: 'top',
      },
    },
    expect: { ok: false, code: 'SENSOR_ON_ACTUATOR' },
  },
  {
    name: 'attach_sensor: non-sensor preset — NOT_A_SENSOR',
    graph: GRAPH_WITH_EXTRUSION_AND_SERVO,
    mutation: {
      kind: 'attach_sensor',
      args: {
        link_name: 'x', parent_link: 'ext1',
        component_id: 'structural_bracket_u', mount_face: 'top',
      },
    },
    expect: { ok: false, code: 'NOT_A_SENSOR' },
  },

  // ── replace_component ───────────────────────────────────────────────────
  {
    name: 'replace_component: extrusion size swap — valid',
    graph: GRAPH_WITH_EXTRUSION_AND_SERVO,
    mutation: {
      kind: 'replace_component',
      args: { link_name: 'ext1', new_component_id: 'structural_extrusion_4040' },
    },
    expect: { ok: true, summaryIncludes: 'structural_extrusion_2020' },
  },
  {
    name: 'replace_component: unknown link — UNKNOWN_LINK',
    graph: GRAPH_WITH_EXTRUSION_AND_SERVO,
    mutation: {
      kind: 'replace_component',
      args: { link_name: 'missing', new_component_id: 'structural_extrusion_4040' },
    },
    expect: { ok: false, code: 'UNKNOWN_LINK' },
  },

  // ── set_joint ───────────────────────────────────────────────────────────
  {
    name: 'set_joint: flip to revolute with rest rpy — valid',
    graph: GRAPH_WITH_EXTRUSION_AND_SERVO,
    mutation: {
      kind: 'set_joint',
      args: {
        link_name: 'servo1', joint_type: 'revolute',
        joint_axis: 'y', attach_rpy: [0, 0.52, 0],
      },
    },
    expect: { ok: true },
  },
  {
    name: 'set_joint: invalid joint_type — BAD_JOINT_TYPE',
    graph: GRAPH_WITH_EXTRUSION_AND_SERVO,
    mutation: {
      kind: 'set_joint',
      args: { link_name: 'servo1', joint_type: 'twist' },
    },
    expect: { ok: false, code: 'BAD_JOINT_TYPE' },
  },

  // ── remove_link ─────────────────────────────────────────────────────────
  {
    name: 'remove_link: cascade delete subtree — valid',
    graph: GRAPH_WITH_EXTRUSION_AND_SERVO,
    mutation: {
      kind: 'remove_link',
      args: { link_name: 'ext1' },
    },
    expect: { ok: true, summaryIncludes: 'descendant' },
  },
  {
    name: 'remove_link: refuse to remove root — ROOT_REMOVAL',
    graph: GRAPH_WITH_EXTRUSION_AND_SERVO,
    mutation: {
      kind: 'remove_link',
      args: { link_name: 'plate' },
    },
    expect: { ok: false, code: 'ROOT_REMOVAL' },
  },
  {
    name: 'remove_link: reparent_children grafts descendants up one level',
    graph: GRAPH_WITH_EXTRUSION_AND_SERVO,
    mutation: {
      kind: 'remove_link',
      args: { link_name: 'ext1', reparent_children: true },
    },
    expect: { ok: true, graphContainsLink: 'servo1' },
  },
]

// ── Runner ──────────────────────────────────────────────────────────────────
interface CaseResult { name: string; passed: boolean; reason?: string }

function runFixture(f: Fixture, ctx: ValidationContext): CaseResult {
  const graph = toGraph(f.graph)
  const result = applyMutation(graph, f.mutation, ctx)

  if (f.expect.ok) {
    if (!result.ok) {
      return { name: f.name, passed: false, reason: `expected ok:true but got code=${result.code} msg=${result.message}` }
    }
    if (f.expect.summaryIncludes && !result.summary.includes(f.expect.summaryIncludes)) {
      return { name: f.name, passed: false, reason: `summary "${result.summary}" did not include "${f.expect.summaryIncludes}"` }
    }
    if (f.expect.graphContainsLink) {
      const found = result.graph.components.map(c => c.link_name)
      if (!found.includes(f.expect.graphContainsLink)) {
        return { name: f.name, passed: false, reason: `graph did not contain link "${f.expect.graphContainsLink}". links: [${found.join(', ')}]` }
      }
    }
    // Guard against accidental base_link-field mutation — the dispatcher should
    // never touch it, and we don't want a regression that silently renames the root.
    if (result.graph.base_link !== graph.base_link) {
      return { name: f.name, passed: false, reason: `base_link changed: "${graph.base_link}" → "${result.graph.base_link}"` }
    }
    return { name: f.name, passed: true }
  }

  if (result.ok) {
    return { name: f.name, passed: false, reason: `expected code=${f.expect.code} but got ok:true summary="${result.summary}"` }
  }
  if (result.code !== f.expect.code) {
    return { name: f.name, passed: false, reason: `expected code=${f.expect.code} but got code=${result.code} msg=${result.message}` }
  }
  return { name: f.name, passed: true }
}

function main(): void {
  const ctx = loadPresets()
  const results: CaseResult[] = []
  console.log('── Tool-dispatch fixtures ────────────────────────────────────')
  for (const f of fixtures) results.push(runFixture(f, ctx))

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
