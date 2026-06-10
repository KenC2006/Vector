// Topology validator corpus: runs synthetic AssemblyGraphs through the pure
// structured validator and asserts expected outcomes.
//
// Run: cd src && npm run test:validator
// Direct: node --experimental-strip-types src/topologyCorpus.ts
//
// WS2: the auto-repair pass is gone — scenarios that used to assert a silent
// rewrite now assert the corresponding WARNING (with its suggested_repair
// content). Hard errors are limited to structural impossibilities:
// UNKNOWN_PARENT / UNKNOWN_COMPONENT / DUPLICATE_LINK_NAME / MULTIPLE_ROOTS /
// CYCLE. Every warning rule keeps a bad/fix fixture pair.
//
// Rules also get a FEASIBILITY PRECHECK entry that names at least one preset
// combination capable of satisfying them — a rule with no satisfiable fix
// burns the Claude redesign budget (see project_topology_validator_attempt.md
// for the BASEPLATE_TOO_THIN postmortem).

import * as fs from 'node:fs'
import * as path from 'node:path'
import { fileURLToPath } from 'node:url'
import { validateTopology, validateTopologyStructured } from './topologyValidation.ts'
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
  input: { base_link: string; components: AssemblyComponentInput[] }
  /** errors.length === 0 expected? */
  expected_pass: boolean
  /** Error tags that must appear (substring match against formatted strings). */
  expected_errors?: string[]
  /** Error tags that must NOT appear. */
  forbidden_errors?: string[]
  /** Warning tags that must appear (substring match). */
  expected_warnings?: string[]
  /** Warning tags that must NOT appear. */
  forbidden_warnings?: string[]
  /** Substrings that must appear in some warning's suggested_repair. */
  expected_repair_hints?: string[]
}

function completeComponent(c: AssemblyComponentInput): AssemblyComponent {
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
    attach_connector: c.attach_connector,
    mate_connector: c.mate_connector,
    mate_type: c.mate_type,
    link_geometry: c.link_geometry,
    attach_primitive: c.attach_primitive,
    attach_anchor: c.attach_anchor,
  }
}

function toGraph(input: Fixture['input']): AssemblyGraph {
  return {
    base_link: input.base_link,
    components: input.components.map(completeComponent),
  }
}

// ── Feasibility prechecks ────────────────────────────────────────────────────
// Each rule must have at least one named preset combination that satisfies it
// (i.e. the rule stays silent). Asserted before fixtures run.
interface FeasibilityCheck {
  rule: string
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
  {
    rule: 'PORT_MISMATCH',
    note: 'A BLDC motor on a coupler disc passes (coupler exists to bridge shaft/mount_face).',
    satisfyingGraph: {
      base_link: 'base_link',
      components: [
        { link_name: 'plate',   component_id: 'structural_baseplate',           attach_to: null },
        { link_name: 'coupler', component_id: 'structural_servo_coupler_disc',  attach_to: 'plate',   attach_face: 'top' },
        { link_name: 'motor1',  component_id: 'actuator_bldc_small',            attach_to: 'coupler', attach_face: 'top' },
      ],
    },
  },
]

// ── Fixtures ────────────────────────────────────────────────────────────────
const fixtures: Fixture[] = [
  // SHAFT_FANOUT (warning) ─────────────────────────────────────────────────
  {
    name: 'SHAFT_FANOUT: bad — servo has 2 non-mobility children on shaft → warning',
    expected_pass: true,
    expected_warnings: ['[SHAFT_FANOUT]'],
    expected_repair_hints: ['reparent'],
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
    expected_pass: true,
    forbidden_warnings: ['[SHAFT_FANOUT]'],
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
    name: 'SHAFT_FANOUT: two tires on one drivetrain shaft → warning (not BARE_TIRE)',
    expected_pass: true,
    expected_warnings: ['[SHAFT_FANOUT]'],
    forbidden_warnings: ['[BARE_TIRE]'],
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

  // SENSOR_ON_ACTUATOR (warning) ───────────────────────────────────────────
  {
    name: 'SENSOR_ON_ACTUATOR: camera directly on servo → warning naming the structural ancestor',
    expected_pass: true,
    expected_warnings: ['[SENSOR_ON_ACTUATOR]'],
    expected_repair_hints: ['ext1'],
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
    expected_pass: true,
    forbidden_warnings: ['[SENSOR_ON_ACTUATOR]'],
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

  // PORT_MISMATCH (warning, shared with graphMutations) ────────────────────
  {
    name: 'PORT_MISMATCH: BLDC motor shaft against baseplate face → warning with coupler hint',
    expected_pass: true,
    expected_warnings: ['[PORT_MISMATCH]'],
    expected_repair_hints: ['structural_servo_coupler_disc'],
    input: {
      base_link: 'base_link',
      components: [
        { link_name: 'plate',  component_id: 'structural_baseplate',  attach_to: null },
        { link_name: 'motor1', component_id: 'actuator_bldc_small',   attach_to: 'plate', attach_face: 'bottom' },
      ],
    },
  },
  {
    name: 'PORT_MISMATCH: tire on drivetrain shaft stays silent (intended wheel pattern)',
    expected_pass: true,
    forbidden_warnings: ['[PORT_MISMATCH]'],
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
    name: 'PORT_MISMATCH: split servo on servo stays silent (compound joint owns its carrier)',
    expected_pass: true,
    forbidden_warnings: ['[PORT_MISMATCH]'],
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
    name: 'PORT_MISMATCH: motor on existing bracket stays silent (idempotent escape hatch)',
    expected_pass: true,
    forbidden_warnings: ['[PORT_MISMATCH]'],
    input: {
      base_link: 'base_link',
      components: [
        { link_name: 'plate',   component_id: 'structural_baseplate',    attach_to: null },
        { link_name: 'bracket', component_id: 'structural_bracket_u',    attach_to: 'plate',   attach_face: 'top' },
        { link_name: 'motor1',  component_id: 'actuator_bldc_small',     attach_to: 'bracket', attach_face: 'bottom' },
      ],
    },
  },

  // BARE_TIRE (warning) ────────────────────────────────────────────────────
  {
    name: 'BARE_TIRE: tire attached directly to baseplate → warning with drivetrain hint',
    expected_pass: true,
    expected_warnings: ['[BARE_TIRE]'],
    expected_repair_hints: ['drivetrain_hub_motor_80'],
    input: {
      base_link: 'base_link',
      components: [
        { link_name: 'plate', component_id: 'structural_baseplate',  attach_to: null },
        { link_name: 'tire1', component_id: 'mobility_wheel_driven', attach_to: 'plate', attach_face: 'bottom' },
      ],
    },
  },
  {
    name: 'BARE_TIRE: fix — tire attached coaxially to drivetrain',
    expected_pass: true,
    forbidden_warnings: ['[BARE_TIRE]'],
    input: {
      base_link: 'base_link',
      components: [
        { link_name: 'plate', component_id: 'structural_baseplate',    attach_to: null },
        { link_name: 'hub1',  component_id: 'drivetrain_hub_motor_80', attach_to: 'plate', attach_face: 'bottom', joint_type: 'continuous', joint_axis: 'y' },
        { link_name: 'tire1', component_id: 'mobility_wheel_driven',   attach_to: 'hub1',  attach_face: 'coaxial' },
      ],
    },
  },

  // EFFECTOR_HAS_CHILDREN / FOOT_PAD_HAS_CHILDREN (warnings) ───────────────
  {
    name: 'EFFECTOR_HAS_CHILDREN: sensor past a gripper → warning, not rewrite',
    expected_pass: true,
    expected_warnings: ['[EFFECTOR_HAS_CHILDREN]'],
    input: {
      base_link: 'base_link',
      components: [
        { link_name: 'plate',   component_id: 'structural_baseplate',             attach_to: null },
        { link_name: 'grip1',   component_id: 'effector_parallel_gripper_small',  attach_to: 'plate', attach_face: 'top' },
        { link_name: 'feeler1', component_id: 'sensor_tof',                       attach_to: 'grip1', attach_face: 'top' },
      ],
    },
  },
  {
    name: 'FOOT_PAD_HAS_CHILDREN: antenna on a foot pad → warning, not rewrite',
    expected_pass: true,
    expected_warnings: ['[FOOT_PAD_HAS_CHILDREN]'],
    input: {
      base_link: 'base_link',
      components: [
        { link_name: 'plate', component_id: 'structural_baseplate',      attach_to: null },
        { link_name: 'foot1', component_id: 'mobility_rubber_foot_pad',  attach_to: 'plate', attach_face: 'bottom' },
        { link_name: 'ant1',  component_id: 'sensor_ultrasonic',         attach_to: 'foot1', attach_face: 'top' },
      ],
    },
  },

  // SERVO_SPACER (warning) ─────────────────────────────────────────────────
  {
    name: 'SERVO_SPACER: coupler disc between servos → warning, preserved in graph',
    expected_pass: true,
    expected_warnings: ['[SERVO_SPACER]'],
    input: {
      base_link: 'base_link',
      components: [
        { link_name: 'plate',   component_id: 'structural_baseplate',           attach_to: null },
        { link_name: 'servo1',  component_id: 'actuator_servo_standard',        attach_to: 'plate',   attach_face: 'top' },
        { link_name: 'disc1',   component_id: 'structural_servo_coupler_disc',  attach_to: 'servo1',  attach_face: 'top' },
        { link_name: 'servo2',  component_id: 'actuator_servo_standard',        attach_to: 'disc1',   attach_face: 'top' },
      ],
    },
  },

  // DIRECT_SERVO_STACK (warning) ───────────────────────────────────────────
  {
    name: 'DIRECT_SERVO_STACK: actuator on servo emits warning',
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

  // SENSOR_ON_SENSOR (warning) ─────────────────────────────────────────────
  {
    name: 'SENSOR_ON_SENSOR: camera on lidar → warning',
    expected_pass: true,
    expected_warnings: ['[SENSOR_ON_SENSOR]'],
    input: {
      base_link: 'base_link',
      components: [
        { link_name: 'plate',  component_id: 'structural_baseplate',       attach_to: null },
        { link_name: 'lidar1', component_id: 'sensor_lidar_2d',            attach_to: 'plate',  attach_face: 'top' },
        { link_name: 'cam1',   component_id: 'sensor_depth_camera_small',  attach_to: 'lidar1', attach_face: 'top' },
      ],
    },
  },

  // TIPPY_PROPORTIONS (warning) ────────────────────────────────────────────
  {
    name: 'TIPPY_PROPORTIONS: tall arm on narrow baseplate emits warning (not error)',
    expected_pass: true,
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

  // Primitive anchors (WS5) ────────────────────────────────────────────────
  {
    name: 'PRIMITIVE_ANCHOR: valid anchor on a named primitive passes',
    expected_pass: true,
    forbidden_errors: ['[BAD_PRIMITIVE_REF]', '[BAD_ANCHOR]', '[PRIMITIVE_ON_NON_CAD_BODY]'],
    input: {
      base_link: 'base_link',
      components: [
        { link_name: 'body', component_id: 'structural_baseplate', attach_to: null,
          link_geometry: [
            { name: 'shoulder_l', shape: 'cylinder', radius_mm: 25, length_mm: 60, xyz_mm: [0, -115, 230], rpy: [1.5708, 0, 0] },
          ] },
        { link_name: 'servo1', component_id: 'actuator_servo_high_torque', attach_to: 'body',
          attach_primitive: 'shoulder_l', attach_anchor: '+axis_end',
          attach_face: 'left', joint_type: 'revolute', joint_axis: 'y' },
      ],
    },
  },
  {
    name: 'BAD_PRIMITIVE_REF: typo in primitive name → hard error with closest-match hint',
    expected_pass: false,
    // formatStructuredDiagnostic folds suggested_repair into the error string,
    // so the closest-match hint is assertable as a plain substring.
    expected_errors: ['[BAD_PRIMITIVE_REF]', 'shoulder_l'],
    input: {
      base_link: 'base_link',
      components: [
        { link_name: 'body', component_id: 'structural_baseplate', attach_to: null,
          link_geometry: [
            { name: 'shoulder_l', shape: 'cylinder', radius_mm: 25, length_mm: 60 },
          ] },
        { link_name: 'servo1', component_id: 'actuator_servo_high_torque', attach_to: 'body',
          attach_primitive: 'sholder_l', attach_anchor: '+axis_end',
          attach_face: 'left', joint_type: 'revolute', joint_axis: 'y' },
      ],
    },
  },
  {
    name: 'BAD_ANCHOR: pole anchor on a box → hard error listing valid anchors',
    expected_pass: false,
    expected_errors: ['[BAD_ANCHOR]'],
    input: {
      base_link: 'base_link',
      components: [
        { link_name: 'body', component_id: 'structural_baseplate', attach_to: null,
          link_geometry: [{ name: 'chest', shape: 'box', size_mm: [180, 100, 260] }] },
        { link_name: 'servo1', component_id: 'actuator_servo_high_torque', attach_to: 'body',
          attach_primitive: 'chest', attach_anchor: '+z_pole',
          attach_face: 'top', joint_type: 'revolute', joint_axis: 'y' },
      ],
    },
  },
  {
    name: 'PRIMITIVE_ON_NON_CAD_BODY: attach_primitive on a plain preset → hard error',
    expected_pass: false,
    expected_errors: ['[PRIMITIVE_ON_NON_CAD_BODY]'],
    input: {
      base_link: 'base_link',
      components: [
        { link_name: 'plate', component_id: 'structural_baseplate', attach_to: null },
        { link_name: 'servo1', component_id: 'actuator_servo_high_torque', attach_to: 'plate',
          attach_primitive: 'chest', attach_anchor: '+z_face',
          attach_face: 'top', joint_type: 'revolute', joint_axis: 'y' },
      ],
    },
  },

  // Hard errors ────────────────────────────────────────────────────────────
  {
    name: 'Unique root: passes with a single root',
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
    expected_pass: false,
    expected_errors: ['[MULTIPLE_ROOTS]'],
    input: {
      base_link: 'base_link',
      components: [
        { link_name: 'p1', component_id: 'structural_baseplate', attach_to: null },
        { link_name: 'p2', component_id: 'structural_baseplate', attach_to: null },
      ],
    },
  },
  {
    name: 'Duplicate names: hard error with rename hint',
    expected_pass: false,
    expected_errors: ['[DUPLICATE_LINK_NAME]'],
    input: {
      base_link: 'base_link',
      components: [
        { link_name: 'plate',   component_id: 'structural_baseplate', attach_to: null },
        { link_name: 'ext',     component_id: 'structural_extrusion_2020',       attach_to: 'plate', attach_face: 'top', length_mm: 200 },
        { link_name: 'ext',     component_id: 'structural_extrusion_2020',       attach_to: 'plate', attach_face: 'bottom', length_mm: 200 },
      ],
    },
  },
  {
    name: 'Unknown parent: hard error',
    expected_pass: false,
    expected_errors: ['[UNKNOWN_PARENT]'],
    input: {
      base_link: 'base_link',
      components: [
        { link_name: 'plate', component_id: 'structural_baseplate',      attach_to: null },
        { link_name: 'ext1',  component_id: 'structural_extrusion_2020', attach_to: 'ghost', attach_face: 'top' },
      ],
    },
  },
  {
    name: 'Unknown component: hard error',
    expected_pass: false,
    expected_errors: ['[UNKNOWN_COMPONENT]'],
    input: {
      base_link: 'base_link',
      components: [
        { link_name: 'plate', component_id: 'structural_baseplate', attach_to: null },
        { link_name: 'x1',    component_id: 'made_up_component',    attach_to: 'plate', attach_face: 'top' },
      ],
    },
  },
  {
    name: 'Cycle: hard error',
    expected_pass: false,
    expected_errors: ['[CYCLE]'],
    input: {
      base_link: 'base_link',
      components: [
        { link_name: 'plate', component_id: 'structural_baseplate',      attach_to: null },
        { link_name: 'a',     component_id: 'structural_extrusion_2020', attach_to: 'b', attach_face: 'top' },
        { link_name: 'b',     component_id: 'structural_extrusion_2020', attach_to: 'a', attach_face: 'top' },
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
  if (f.expected_repair_hints?.length) {
    const structured = validateTopologyStructured(graph.components, ctx)
    for (const hint of f.expected_repair_hints) {
      if (!structured.some(d => d.suggested_repair?.includes(hint))) {
        return {
          name: f.name, passed: false,
          reason: `expected a suggested_repair containing "${hint}". got: ${JSON.stringify(structured.map(d => d.suggested_repair))}`,
        }
      }
    }
  }
  return { name: f.name, passed: true }
}

function runFeasibility(check: FeasibilityCheck, ctx: ValidationContext): CaseResult {
  const { errors, warnings } = validateTopology(toGraph(check.satisfyingGraph).components, ctx)
  // The satisfying graph must not trigger the rule under test (on either channel).
  const triggered = [...errors, ...warnings].filter(e => e.includes(`[${check.rule}]`))
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
