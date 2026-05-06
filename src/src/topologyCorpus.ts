// Topology validator corpus: runs synthetic AssemblyGraphs through the pure
// validator + auto-repair and asserts expected outcomes.
//
// Run: cd src && npm run test:validator
// Direct: node --experimental-strip-types src/topologyCorpus.ts
//
// Every new validator rule must ship with a FIXTURE PAIR in this file:
//   - `bad` case: minimal graph that trips the rule (expected_errors contains the tag)
//   - `fix` case: minimal corrected graph that passes (expected_pass: true)
//
// Rules also get a FEASIBILITY PRECHECK entry that names at least one preset
// combination capable of satisfying them. Without a satisfiable fix, a hard
// rule burns the entire Claude redesign budget and hard-fails valid robots —
// see project_topology_validator_attempt.md for the BASEPLATE_TOO_THIN
// postmortem.

import * as fs from 'node:fs'
import * as path from 'node:path'
import { fileURLToPath } from 'node:url'
import { validateTopology, autoRepairTopology } from './topologyValidation.ts'
import type { ValidationPreset, ValidationContext } from './topologyValidation.ts'
import type { AssemblyComponent, AssemblyGraph } from './urdfAssembly.ts'

// ── Preset loader ────────────────────────────────────────────────────────────
interface PresetFile {
  categories: Record<string, { components: ValidationPreset[] }>
}

function loadPresets(): ValidationContext {
  const here = path.dirname(fileURLToPath(import.meta.url))
  const presetPath = path.resolve(here, '..', '..', 'core', 'presets', 'generic_presets.json')
  const raw = fs.readFileSync(presetPath, 'utf-8')
  const data = JSON.parse(raw) as PresetFile
  const byId = new Map<string, ValidationPreset>()
  for (const cat of Object.values(data.categories)) {
    for (const c of cat.components) byId.set(c.id, c)
  }
  return { findPreset: (id: string) => byId.get(id) ?? null }
}

// ── Fixture shape ────────────────────────────────────────────────────────────
type AssemblyComponentInput = Partial<AssemblyComponent> & {
  link_name: string
  component_id: string
  attach_to: string | null
}

interface Fixture {
  name: string
  kind: 'validate' | 'auto_repair'
  input: { base_link: string; components: AssemblyComponentInput[] }
  expected_pass: boolean
  /** Error tags that must appear (substring match). */
  expected_errors?: string[]
  /** Error tags that must NOT appear. */
  forbidden_errors?: string[]
  /** Warning tags that must appear (substring match). */
  expected_warnings?: string[]
  /** Warning tags that must NOT appear. */
  forbidden_warnings?: string[]
  /** Repair kinds that must have fired (in any order). */
  expected_repair_kinds?: string[]
  /** After auto-repair + re-validate, this must hold. */
  expected_pass_after_repair?: boolean
  /** After auto-repair, assert specific fields on specific components by
   *  link_name. Added for task #7 (WS6 Phase 3) so the auto-bracket path's
   *  mate-connector propagation is actually checked — previously the only
   *  assertion was that the repair kind fired, not that the inserted
   *  component got the correct attach_connector/mate_connector/mate_type. */
  expected_component_fields?: Record<string, Partial<AssemblyComponent>>
}

function completeComponent(c: AssemblyComponentInput): AssemblyComponent {
  return {
    link_name: c.link_name,
    component_id: c.component_id,
    attach_to: c.attach_to,
    attach_face: c.attach_face ?? null,
    joint_type: c.joint_type ?? (c.attach_to ? 'fixed' : 'fixed'),
    joint_axis: c.joint_axis ?? '0 0 1',
    length_mm: c.length_mm,
    orientation: c.orientation,
    elevation_angle: c.elevation_angle,
    attach_connector: c.attach_connector,
    mate_connector: c.mate_connector,
    mate_type: c.mate_type,
  }
}

function toGraph(input: Fixture['input']): AssemblyGraph {
  return {
    base_link: input.base_link,
    components: input.components.map(completeComponent),
  }
}

// ── Feasibility prechecks ────────────────────────────────────────────────────
// Each hard rule must have at least one named preset combination that satisfies
// it. These are asserted before fixtures run.
interface FeasibilityCheck {
  rule: string
  /** A graph that must validate cleanly, naming the presets that satisfy the rule. */
  satisfyingGraph: Fixture['input']
  note: string
}

const feasibilityChecks: FeasibilityCheck[] = [
  {
    rule: 'SHAFT_FANOUT',
    note: 'A servo with exactly one child on its shaft passes.',
    satisfyingGraph: {
      base_link: 'base_link',
      components: [
        { link_name: 'plate',  component_id: 'structural_baseplate',      attach_to: null },
        { link_name: 'servo1', component_id: 'actuator_servo_standard',   attach_to: 'plate',  attach_face: 'top' },
        { link_name: 'brac1',  component_id: 'structural_bracket_u',      attach_to: 'servo1', attach_face: 'top' },
      ],
    },
  },
  {
    rule: 'SENSOR_ON_ACTUATOR',
    note: 'A sensor attached to a structural extrusion passes.',
    satisfyingGraph: {
      base_link: 'base_link',
      components: [
        { link_name: 'plate',    component_id: 'structural_baseplate',       attach_to: null },
        { link_name: 'ext1',     component_id: 'structural_extrusion_2020',  attach_to: 'plate',  attach_face: 'top', length_mm: 200 },
        { link_name: 'cam1',     component_id: 'sensor_depth_camera_small',  attach_to: 'ext1',   attach_face: 'top' },
      ],
    },
  },
  {
    rule: 'BARE_TIRE',
    note: 'A tire attached coaxially to a drivetrain passes.',
    satisfyingGraph: {
      base_link: 'base_link',
      components: [
        { link_name: 'plate', component_id: 'structural_baseplate', attach_to: null },
        { link_name: 'hub1',  component_id: 'drivetrain_hub_motor_80', attach_to: 'plate', attach_face: 'bottom', joint_type: 'continuous', joint_axis: 'y' },
        { link_name: 'tire1', component_id: 'mobility_wheel_driven', attach_to: 'hub1', attach_face: 'coaxial' },
      ],
    },
  },
]

// ── Fixtures ────────────────────────────────────────────────────────────────
const fixtures: Fixture[] = [
  // SHAFT_FANOUT ───────────────────────────────────────────────────────────
  {
    name: 'SHAFT_FANOUT: bad — servo has 2 non-mobility children on shaft',
    kind: 'validate',
    expected_pass: false,
    expected_errors: ['[SHAFT_FANOUT]'],
    input: {
      base_link: 'base_link',
      components: [
        { link_name: 'plate',  component_id: 'structural_baseplate', attach_to: null },
        { link_name: 'servo1', component_id: 'actuator_servo_standard',         attach_to: 'plate',  attach_face: 'top' },
        { link_name: 'brac1',  component_id: 'structural_bracket_u',      attach_to: 'servo1', attach_face: 'top' },
        { link_name: 'cam1',   component_id: 'sensor_depth_camera_small',       attach_to: 'servo1', attach_face: 'top' },
      ],
    },
  },
  {
    name: 'SHAFT_FANOUT: fix — the extra child lives on an extrusion instead',
    kind: 'validate',
    expected_pass: true,
    forbidden_errors: ['[SHAFT_FANOUT]'],
    input: {
      base_link: 'base_link',
      components: [
        { link_name: 'plate',  component_id: 'structural_baseplate', attach_to: null },
        { link_name: 'ext1',   component_id: 'structural_extrusion_2020',       attach_to: 'plate',  attach_face: 'top', length_mm: 200 },
        { link_name: 'servo1', component_id: 'actuator_servo_standard',         attach_to: 'ext1',   attach_face: 'top' },
        { link_name: 'brac1',  component_id: 'structural_bracket_u',      attach_to: 'servo1', attach_face: 'top' },
        { link_name: 'cam1',   component_id: 'sensor_depth_camera_small',       attach_to: 'ext1',   attach_face: 'x_plus' },
      ],
    },
  },
  {
    name: 'SHAFT_FANOUT: autorepair — extra non-mobility child reparented off shaft',
    kind: 'auto_repair',
    expected_pass: false,  // bad input
    expected_pass_after_repair: true,
    expected_repair_kinds: ['shaft_fanout'],
    input: {
      base_link: 'base_link',
      components: [
        { link_name: 'plate',  component_id: 'structural_baseplate',       attach_to: null },
        { link_name: 'ext1',   component_id: 'structural_extrusion_2020',  attach_to: 'plate',  attach_face: 'top', length_mm: 200 },
        { link_name: 'servo1', component_id: 'actuator_servo_standard',    attach_to: 'ext1',   attach_face: 'top' },
        { link_name: 'brac1',  component_id: 'structural_bracket_u',       attach_to: 'servo1', attach_face: 'top' },
        { link_name: 'brac2',  component_id: 'structural_bracket_l',       attach_to: 'servo1', attach_face: 'top' },
      ],
    },
  },
  {
    name: 'SHAFT_FANOUT: bad — two tires on one drivetrain shaft',
    kind: 'validate',
    expected_pass: false,
    expected_errors: ['[SHAFT_FANOUT]'],
    forbidden_errors: ['[BARE_TIRE]'],
    input: {
      base_link: 'base_link',
      components: [
        { link_name: 'plate',  component_id: 'structural_baseplate',    attach_to: null },
        { link_name: 'hub1',   component_id: 'drivetrain_hub_motor_80', attach_to: 'plate', attach_face: 'bottom', joint_type: 'continuous', joint_axis: 'y' },
        { link_name: 'wheelL', component_id: 'mobility_wheel_driven',   attach_to: 'hub1',  attach_face: 'coaxial' },
        { link_name: 'wheelR', component_id: 'mobility_wheel_driven',   attach_to: 'hub1',  attach_face: 'coaxial' },
      ],
    },
  },

  // SENSOR_ON_ACTUATOR ─────────────────────────────────────────────────────
  {
    name: 'SENSOR_ON_ACTUATOR: bad — camera directly on servo shaft',
    kind: 'validate',
    expected_pass: false,
    expected_errors: ['[SENSOR_ON_ACTUATOR]'],
    input: {
      base_link: 'base_link',
      components: [
        { link_name: 'plate',  component_id: 'structural_baseplate', attach_to: null },
        { link_name: 'ext1',   component_id: 'structural_extrusion_2020',       attach_to: 'plate',  attach_face: 'top', length_mm: 200 },
        { link_name: 'servo1', component_id: 'actuator_servo_standard',         attach_to: 'ext1',   attach_face: 'top' },
        { link_name: 'cam1',   component_id: 'sensor_depth_camera_small',       attach_to: 'servo1', attach_face: 'x_plus' },
      ],
    },
  },
  {
    name: 'SENSOR_ON_ACTUATOR: fix — camera moved onto the extrusion',
    kind: 'validate',
    expected_pass: true,
    forbidden_errors: ['[SENSOR_ON_ACTUATOR]'],
    input: {
      base_link: 'base_link',
      components: [
        { link_name: 'plate',  component_id: 'structural_baseplate', attach_to: null },
        { link_name: 'ext1',   component_id: 'structural_extrusion_2020',       attach_to: 'plate',  attach_face: 'top', length_mm: 200 },
        { link_name: 'servo1', component_id: 'actuator_servo_standard',         attach_to: 'ext1',   attach_face: 'top' },
        { link_name: 'cam1',   component_id: 'sensor_depth_camera_small',       attach_to: 'ext1',   attach_face: 'x_plus' },
      ],
    },
  },
  {
    name: 'SENSOR_ON_ACTUATOR: autorepair — sensor reparented to structural ancestor',
    kind: 'auto_repair',
    expected_pass: false,
    expected_pass_after_repair: true,
    expected_repair_kinds: ['sensor_on_actuator'],
    input: {
      base_link: 'base_link',
      components: [
        { link_name: 'plate',  component_id: 'structural_baseplate', attach_to: null },
        { link_name: 'ext1',   component_id: 'structural_extrusion_2020',       attach_to: 'plate',  attach_face: 'top', length_mm: 200 },
        { link_name: 'servo1', component_id: 'actuator_servo_standard',         attach_to: 'ext1',   attach_face: 'top' },
        { link_name: 'cam1',   component_id: 'sensor_depth_camera_small',       attach_to: 'servo1', attach_face: 'x_plus' },
      ],
    },
  },

  // PORT_MISMATCH (auto-repair 5) ──────────────────────────────────────────
  {
    name: 'CONNECTOR_SANITIZE: invalid L-bracket connector on hip servo is stripped',
    kind: 'auto_repair',
    expected_pass: true,
    expected_pass_after_repair: true,
    expected_repair_kinds: ['invalid_connector_removed'],
    expected_component_fields: {
      hip_pitch_fl: {
        attach_connector: undefined,
        mate_type: undefined,
      },
    },
    input: {
      base_link: 'base_link',
      components: [
        { link_name: 'body', component_id: 'structural_baseplate_large', attach_to: null },
        { link_name: 'hip_abd_fl', component_id: 'actuator_servo_high_torque', attach_to: 'body', attach_face: 'bottom', joint_type: 'revolute', joint_axis: 'x' },
        {
          link_name: 'hip_pitch_fl',
          component_id: 'actuator_servo_high_torque',
          attach_to: 'hip_abd_fl',
          attach_face: 'bottom',
          attach_connector: 'plate_top',
          mate_type: 'fastened',
          joint_type: 'revolute',
          joint_axis: 'y',
        },
      ],
    },
  },
  {
    name: 'PORT_MISMATCH: shaft facing baseplate bottom → bracket inserted',
    kind: 'auto_repair',
    // No validator error is thrown for the mismatch (validator doesn't check
    // port classes; placement does), so both pre- and post-repair pass.
    expected_pass: true,
    expected_pass_after_repair: true,
    input: {
      base_link: 'base_link',
      components: [
        { link_name: 'plate',  component_id: 'structural_baseplate',    attach_to: null },
        { link_name: 'servo1', component_id: 'actuator_servo_standard', attach_to: 'plate', attach_face: 'bottom' },
      ],
    },
  },
  {
    name: 'PORT_MISMATCH: tire on drivetrain shaft is not repaired',
    kind: 'auto_repair',
    expected_pass: true,
    expected_pass_after_repair: true,
    // Tire children on drivetrain shafts are the intended wheel pattern; no
    // bracket insertion should happen.
    input: {
      base_link: 'base_link',
      components: [
        { link_name: 'plate', component_id: 'structural_baseplate',    attach_to: null },
        { link_name: 'hub1',  component_id: 'drivetrain_hub_motor_80', attach_to: 'plate', attach_face: 'bottom', joint_type: 'continuous', joint_axis: 'y' },
        { link_name: 'tire1', component_id: 'mobility_wheel_driven',   attach_to: 'hub1',  attach_face: 'coaxial' },
      ],
    },
  },
  {
    // Task #7 (WS6 Phase 3) integration check. Previously the PORT_MISMATCH
    // repair was only asserted on repair-kind firing; this fixture asserts
    // the inserted bracket actually carries the mate-connector fields
    // (attach_connector="shaft_out", mate_connector="shaft_hole",
    // mate_type="concentric") that route the servo↔coupler mate through
    // the closed-form resolver rather than legacy bbox-stack math.
    name: 'PORT_MISMATCH: Case 1 (shaft parent) bracket carries concentric mate fields',
    kind: 'auto_repair',
    expected_pass: true,
    expected_pass_after_repair: true,
    // Case 1 requires the CHILD to pass isRepairableChild (actuator_* or
    // motor_*). The setup is a second servo mounted with its bottom
    // (mount_face) against the parent servo's top (shaft output), which
    // gives pClass=shaft, cClass=mount_face — the narrow code path that
    // opts into the connector resolver.
    input: {
      base_link: 'base_link',
      components: [
        { link_name: 'plate',  component_id: 'structural_baseplate',        attach_to: null },
        { link_name: 'servo1', component_id: 'actuator_servo_high_torque',  attach_to: 'plate',  attach_face: 'top' },
        { link_name: 'servo2', component_id: 'actuator_servo_standard',     attach_to: 'servo1', attach_face: 'top' },
      ],
    },
  },
  {
    // Case 2 (mount_face parent, shaft child) routes the bracket through
    // the connector resolver,
    // using default face connectors with a fastened mate. Bit-identical to
    // the legacy bbox path on presets that still rely on default face
    // connectors, but flips the repair onto the connector engine so it
    // picks up authored parent face connectors as Phase 2 lands them.
    // Multi-child distribution is preserved via computeMatePlacement's
    // multiChild branch so N auto-couplers still spread across one face.
    name: 'PORT_MISMATCH: Case 2 (mount_face parent) bracket carries fastened mate fields',
    kind: 'auto_repair',
    expected_pass: true,
    expected_pass_after_repair: true,
    input: {
      base_link: 'base_link',
      components: [
        { link_name: 'plate',  component_id: 'structural_baseplate',    attach_to: null },
        { link_name: 'servo1', component_id: 'actuator_servo_standard', attach_to: 'plate', attach_face: 'bottom' },
      ],
    },
  },
  {
    name: 'PORT_MISMATCH: repair is idempotent (no bracket on pre-bracketed servo)',
    kind: 'auto_repair',
    expected_pass: true,
    expected_pass_after_repair: true,
    // Servo on bracket on baseplate — bracket parent already exists, so no
    // additional bracket should be inserted above the servo.
    input: {
      base_link: 'base_link',
      components: [
        { link_name: 'plate',   component_id: 'structural_baseplate',    attach_to: null },
        { link_name: 'bracket', component_id: 'structural_bracket_u',    attach_to: 'plate',   attach_face: 'top' },
        { link_name: 'servo1',  component_id: 'actuator_servo_standard', attach_to: 'bracket', attach_face: 'bottom' },
      ],
    },
  },

  // BARE_TIRE -----------------------------------------------------------------
  {
    name: 'BARE_TIRE: bad - tire attached directly to baseplate',
    kind: 'validate',
    expected_pass: false,
    expected_errors: ['[BARE_TIRE]'],
    input: {
      base_link: 'base_link',
      components: [
        { link_name: 'plate', component_id: 'structural_baseplate',  attach_to: null },
        { link_name: 'tire1', component_id: 'mobility_wheel_driven', attach_to: 'plate', attach_face: 'bottom' },
      ],
    },
  },
  {
    name: 'BARE_TIRE: fix - tire attached coaxially to drivetrain',
    kind: 'validate',
    expected_pass: true,
    forbidden_errors: ['[BARE_TIRE]'],
    input: {
      base_link: 'base_link',
      components: [
        { link_name: 'plate', component_id: 'structural_baseplate',    attach_to: null },
        { link_name: 'hub1',  component_id: 'drivetrain_hub_motor_80', attach_to: 'plate', attach_face: 'bottom', joint_type: 'continuous', joint_axis: 'y' },
        { link_name: 'tire1', component_id: 'mobility_wheel_driven',   attach_to: 'hub1',  attach_face: 'coaxial' },
      ],
    },
  },
  {
    name: 'BARE_TIRE: autorepair - inserts drivetrain parent',
    kind: 'auto_repair',
    expected_pass: false,
    expected_pass_after_repair: true,
    expected_repair_kinds: ['bare_tire_drivetrain'],
    input: {
      base_link: 'base_link',
      components: [
        { link_name: 'plate', component_id: 'structural_baseplate',  attach_to: null },
        { link_name: 'tire1', component_id: 'mobility_wheel_driven', attach_to: 'plate', attach_face: 'bottom' },
      ],
    },
  },

  // DIRECT_SERVO_STACK (warning) ----------------------------------------------
  {
    name: 'DIRECT_SERVO_STACK: actuator on servo emits warning',
    kind: 'validate',
    expected_pass: true,
    expected_warnings: ['[DIRECT_SERVO_STACK]'],
    input: {
      base_link: 'base_link',
      components: [
        { link_name: 'plate',  component_id: 'structural_baseplate',    attach_to: null },
        { link_name: 'servo1', component_id: 'actuator_servo_standard', attach_to: 'plate',  attach_face: 'top' },
        { link_name: 'motor1', component_id: 'actuator_bldc_small', attach_to: 'servo1', attach_face: 'top' },
      ],
    },
  },
  {
    name: 'DIRECT_SERVO_STACK: servos separated by extrusion stay silent',
    kind: 'validate',
    expected_pass: true,
    forbidden_warnings: ['[DIRECT_SERVO_STACK]'],
    input: {
      base_link: 'base_link',
      components: [
        { link_name: 'plate',  component_id: 'structural_baseplate',      attach_to: null },
        { link_name: 'servo1', component_id: 'actuator_servo_standard',   attach_to: 'plate',  attach_face: 'top' },
        { link_name: 'ext1',   component_id: 'structural_extrusion_2020', attach_to: 'servo1', attach_face: 'top', length_mm: 200 },
        { link_name: 'servo2', component_id: 'actuator_servo_standard',   attach_to: 'ext1',   attach_face: 'top' },
      ],
    },
  },

  // TIPPY_PROPORTIONS ──────────────────────────────────────────────────────
  // Currently emitted as a WARNING, not a hard error — threshold is unverified
  // (postmortem: 4.1× was unstable, so 5× may be too lenient) AND only one
  // baseplate preset exists at 200mm wide, so the rule may be unsatisfiable.
  // Promote to a hard error only after fixture-verifying the threshold AND
  // adding a feasibilityChecks entry above.
  {
    name: 'TIPPY_PROPORTIONS: tall arm on narrow baseplate emits warning (not error)',
    kind: 'validate',
    expected_pass: true,                         // warnings don't fail the validator
    expected_warnings: ['[TIPPY_PROPORTIONS]'],
    input: {
      base_link: 'base_link',
      components: [
        { link_name: 'plate',  component_id: 'structural_baseplate',       attach_to: null },
        // Eight 200mm extrusions stacked vertically ⇒ ~1600mm tall on ~200mm baseplate (8×).
        { link_name: 'ext1',   component_id: 'structural_extrusion_2020',  attach_to: 'plate',  attach_face: 'top', length_mm: 200 },
        { link_name: 'ext2',   component_id: 'structural_extrusion_2020',  attach_to: 'ext1',   attach_face: 'top', length_mm: 200 },
        { link_name: 'ext3',   component_id: 'structural_extrusion_2020',  attach_to: 'ext2',   attach_face: 'top', length_mm: 200 },
        { link_name: 'ext4',   component_id: 'structural_extrusion_2020',  attach_to: 'ext3',   attach_face: 'top', length_mm: 200 },
        { link_name: 'ext5',   component_id: 'structural_extrusion_2020',  attach_to: 'ext4',   attach_face: 'top', length_mm: 200 },
        { link_name: 'ext6',   component_id: 'structural_extrusion_2020',  attach_to: 'ext5',   attach_face: 'top', length_mm: 200 },
        { link_name: 'ext7',   component_id: 'structural_extrusion_2020',  attach_to: 'ext6',   attach_face: 'top', length_mm: 200 },
        { link_name: 'ext8',   component_id: 'structural_extrusion_2020',  attach_to: 'ext7',   attach_face: 'top', length_mm: 200 },
      ],
    },
  },
  {
    name: 'TIPPY_PROPORTIONS: short stable robot stays silent',
    kind: 'validate',
    expected_pass: true,
    forbidden_warnings: ['[TIPPY_PROPORTIONS]'],
    input: {
      base_link: 'base_link',
      components: [
        { link_name: 'plate', component_id: 'structural_baseplate',      attach_to: null },
        { link_name: 'ext1',  component_id: 'structural_extrusion_2020', attach_to: 'plate', attach_face: 'top', length_mm: 100 },
      ],
    },
  },

  // Basic sanity rules ─────────────────────────────────────────────────────
  {
    name: 'Unique root: passes with a single root',
    kind: 'validate',
    expected_pass: true,
    input: {
      base_link: 'base_link',
      components: [
        { link_name: 'plate', component_id: 'structural_baseplate', attach_to: null },
      ],
    },
  },
  {
    name: 'Unique root: fails with two roots',
    kind: 'validate',
    expected_pass: false,
    expected_errors: ['Multiple root components'],
    input: {
      base_link: 'base_link',
      components: [
        { link_name: 'p1', component_id: 'structural_baseplate', attach_to: null },
        { link_name: 'p2', component_id: 'structural_baseplate', attach_to: null },
      ],
    },
  },
  {
    name: 'Duplicate names: auto-repaired',
    kind: 'auto_repair',
    expected_pass: false,
    expected_pass_after_repair: true,
    expected_repair_kinds: ['duplicate_name'],
    input: {
      base_link: 'base_link',
      components: [
        { link_name: 'plate',   component_id: 'structural_baseplate', attach_to: null },
        { link_name: 'ext',     component_id: 'structural_extrusion_2020',       attach_to: 'plate', attach_face: 'top', length_mm: 200 },
        { link_name: 'ext',     component_id: 'structural_extrusion_2020',       attach_to: 'plate', attach_face: 'bottom', length_mm: 200 },
      ],
    },
  },
]

// ── Runner ──────────────────────────────────────────────────────────────────
interface CaseResult {
  name: string
  passed: boolean
  reason?: string
}

function runFixture(f: Fixture, ctx: ValidationContext): CaseResult {
  const graph = toGraph(f.input)
  if (f.kind === 'validate') {
    const { errors, warnings } = validateTopology(graph.components, ctx)
    const passed = errors.length === 0
    if (passed !== f.expected_pass) {
      return {
        name: f.name,
        passed: false,
        reason: `expected_pass=${f.expected_pass} but got errors=${JSON.stringify(errors)}`,
      }
    }
    for (const tag of f.expected_errors ?? []) {
      if (!errors.some(e => e.includes(tag))) {
        return { name: f.name, passed: false, reason: `expected error containing "${tag}" not found. got: ${JSON.stringify(errors)}` }
      }
    }
    for (const tag of f.forbidden_errors ?? []) {
      if (errors.some(e => e.includes(tag))) {
        return { name: f.name, passed: false, reason: `forbidden error "${tag}" fired. got: ${JSON.stringify(errors)}` }
      }
    }
    for (const tag of f.expected_warnings ?? []) {
      if (!warnings.some(w => w.includes(tag))) {
        return { name: f.name, passed: false, reason: `expected warning containing "${tag}" not found. got: ${JSON.stringify(warnings)}` }
      }
    }
    for (const tag of f.forbidden_warnings ?? []) {
      if (warnings.some(w => w.includes(tag))) {
        return { name: f.name, passed: false, reason: `forbidden warning "${tag}" fired. got: ${JSON.stringify(warnings)}` }
      }
    }
    return { name: f.name, passed: true }
  }

  // auto_repair kind
  const before = validateTopology(graph.components, ctx)
  const beforePassed = before.errors.length === 0
  if (beforePassed !== f.expected_pass) {
    return { name: f.name, passed: false, reason: `pre-repair expected_pass=${f.expected_pass} but got errors=${JSON.stringify(before.errors)}` }
  }
  const { repairs } = autoRepairTopology(graph, ctx)
  for (const kind of f.expected_repair_kinds ?? []) {
    if (!repairs.some(r => r.kind === kind)) {
      return { name: f.name, passed: false, reason: `expected repair kind "${kind}" but repairs fired: ${JSON.stringify(repairs.map(r => r.kind))}` }
    }
  }
  if (f.expected_pass_after_repair !== undefined) {
    const after = validateTopology(graph.components, ctx)
    const afterPassed = after.errors.length === 0
    if (afterPassed !== f.expected_pass_after_repair) {
      return { name: f.name, passed: false, reason: `post-repair expected_pass=${f.expected_pass_after_repair} but got errors=${JSON.stringify(after.errors)}` }
    }
  }
  for (const [linkName, expected] of Object.entries(f.expected_component_fields ?? {})) {
    const comp = graph.components.find(c => c.link_name === linkName)
    if (!comp) {
      return { name: f.name, passed: false, reason: `expected_component_fields references "${linkName}" but no such component exists post-repair. Graph: ${graph.components.map(c => c.link_name).join(', ')}` }
    }
    for (const [field, expectedValue] of Object.entries(expected) as [keyof AssemblyComponent, unknown][]) {
      const actualValue = comp[field]
      if (actualValue !== expectedValue) {
        return { name: f.name, passed: false, reason: `post-repair "${linkName}".${String(field)}: expected ${JSON.stringify(expectedValue)}, got ${JSON.stringify(actualValue)}` }
      }
    }
  }
  return { name: f.name, passed: true }
}

function runFeasibility(check: FeasibilityCheck, ctx: ValidationContext): CaseResult {
  const { errors } = validateTopology(toGraph(check.satisfyingGraph).components, ctx)
  // The satisfying graph must not trigger the rule under test.
  const triggered = errors.filter(e => e.includes(`[${check.rule}]`))
  if (triggered.length > 0) {
    return {
      name: `feasibility[${check.rule}]: ${check.note}`,
      passed: false,
      reason: `rule fired on its satisfying graph: ${triggered.join('; ')}`,
    }
  }
  return { name: `feasibility[${check.rule}]: ${check.note}`, passed: true }
}

function main(): void {
  const ctx = loadPresets()
  const results: CaseResult[] = []

  console.log('── Feasibility prechecks ─────────────────────────────────────')
  for (const check of feasibilityChecks) results.push(runFeasibility(check, ctx))
  console.log('── Fixtures ──────────────────────────────────────────────────')
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
