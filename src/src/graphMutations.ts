// Pure AssemblyGraph mutation entrypoints for the WS2 tool-call edit surface.
//
// Each exported mutator takes a graph + typed args, applies the change to a
// deep clone (never mutates the caller's copy), re-runs `validateTopology` on
// the result, and returns a structured pass/fail response. The response shape
// is the same one the backend marshals back to Claude as a `tool_result`:
//
//   { ok: true,  graph: <mutated>, warnings: string[] }
//   { ok: false, code: <rule>,     message: string, suggested_repair?: string }
//
// Staying pure (no THREE / DOM / presets.json I/O) is load-bearing: the
// fixture runner drives this from Node, and the Python edit-turn loop calls
// through the same API via urdfAssembly.ts. Every rule the TS validator
// knows about (sensor-on-actuator, shaft-fanout, port class mismatch,
// direct-servo-stack) is enforced here automatically — this module calls the
// same `validateTopology` that the placement engine runs.
//
// Port-mismatch enforcement: validateTopology doesn't check shaft/mount_face
// port classes (placement currently logs a warning and proceeds — the
// "detect→enforce gap" in `project_vector_improvement_plan.md` anti-patterns).
// This module closes that gap: `_checkPortCompatibility` rejects any mutation
// that would stack an actuator directly on another actuator's shaft face or
// land a shaft on a structural mount_face without a bracket in between.
// Auto-repair (Repair 5) fires for those cases in resolveAssemblyGraph, but
// per-call enforcement lets Claude fix it itself within the same turn instead
// of relying on a downstream repair that may not satisfy the user intent.

import { validateTopology } from './topologyValidation.ts'
import type { ValidationContext } from './topologyValidation.ts'
import { componentPortsForPreset, resolveFaceToPort } from './attachmentNodes.ts'
import type { AttachmentNodeClass } from './attachmentNodes.ts'
import { getOrComputeBbox } from './componentDims.ts'
import { cloneAssemblyGraph } from './urdfGraphEquivalence.ts'
import type { AssemblyComponent, AssemblyGraph } from './urdfGraphEquivalence.ts'

// ── Tool-call argument types ────────────────────────────────────────────────

export interface AddLinkArgs {
  link_name: string
  parent_link: string
  component_id: string
  attach_face: string
  joint_type?: string
  joint_axis?: string
  length_mm?: number
  orientation?: string
  elevation_angle?: number
  attach_rpy?: number[]
  // Phase 4 of docs/MATE_CONNECTOR_MIGRATION.md — optional named-connector
  // overrides (see AssemblyComponent for semantics). Forwarded verbatim to
  // the new child component.
  attach_connector?: string
  mate_connector?: string
  mate_type?: string
}

export interface AttachSensorArgs {
  link_name: string
  parent_link: string
  component_id: string
  mount_face: string
  elevation_angle?: number
}

export interface ReplaceComponentArgs {
  link_name: string
  new_component_id: string
}

export interface SetJointArgs {
  link_name: string
  joint_type: string
  joint_axis?: string
  attach_rpy?: number[]
}

export interface RemoveLinkArgs {
  link_name: string
  reparent_children?: boolean
}

export type GraphMutation =
  | { kind: 'add_link';           args: AddLinkArgs }
  | { kind: 'attach_sensor';      args: AttachSensorArgs }
  | { kind: 'replace_component';  args: ReplaceComponentArgs }
  | { kind: 'set_joint';          args: SetJointArgs }
  | { kind: 'remove_link';        args: RemoveLinkArgs }

// ── Response shape ──────────────────────────────────────────────────────────

export interface MutationOk {
  ok: true
  graph: AssemblyGraph
  warnings: string[]
  /** Short human-readable summary (e.g. "added camera_1 on extrusion_1.top"). */
  summary: string
}

export interface MutationError {
  ok: false
  /** Stable rule code Claude can pattern-match on. */
  code: string
  message: string
  /** Short actionable hint (e.g. "attach to a structural extrusion instead"). */
  suggested_repair?: string
}

export type MutationResult = MutationOk | MutationError

// ── Port-compatibility helper (inline enforcement of the shaft/mount_face gap) ──

const OPPOSITE_FACE: Record<string, string> = {
  top: 'bottom', bottom: 'top',
  front: 'back', back: 'front',
  left: 'right', right: 'left',
}

function _portClassAtFace(
  ctx: ValidationContext,
  componentId: string,
  face: string,
): AttachmentNodeClass | undefined {
  const preset = ctx.findPreset(componentId)
  if (!preset) return undefined
  const bb = getOrComputeBbox(preset.id, preset)
  const ports = componentPortsForPreset(
    preset.id,
    bb[0] / 2000,
    bb[1] / 2000,
    bb[2] / 2000,
    preset.mounting_logic as { primary?: string; output?: string; shaft_diameter_mm?: number } | undefined,
  )
  return resolveFaceToPort(face, ports)?.cls
}

/** Reject mutations whose parent-face class can't mechanically host the child's
 *  contact-face class. Scoped to actuator/motor children because those are the
 *  cases the bracket-insertion auto-repair (Repair 5) covers in resolveAssembly-
 *  Graph. Structural→structural chains skip this check by design:
 *    - mount_face↔mount_face is mechanically valid (brackets bolt on extrusions).
 *    - validateTopology doesn't look at port classes, so a genuinely broken
 *      structural port pairing is still caught only at placement-time via
 *      `nodesCompatible` warnings. Widening this enforcement to structurals
 *      would require naming an auto-repair that covers the widened cases. */
function _checkPortCompatibility(
  ctx: ValidationContext,
  parent: AssemblyComponent | undefined,
  child: AssemblyComponent,
): MutationError | null {
  if (!parent) return null
  const isActuator = (id: string) => id.startsWith('actuator_') || id.startsWith('motor_')
  if (!isActuator(child.component_id)) return null
  // Brackets and coupler discs exist precisely to mate shaft↔mount_face — skip.
  if (
    parent.component_id.startsWith('structural_bracket_') ||
    parent.component_id === 'structural_servo_coupler_disc'
  ) return null

  const parentFace = child.attach_face || 'top'
  const childFace = OPPOSITE_FACE[parentFace] || 'bottom'
  const pClass = _portClassAtFace(ctx, parent.component_id, parentFace)
  const cClass = _portClassAtFace(ctx, child.component_id, childFace)
  if (!pClass || !cClass) return null

  const mismatched =
    (pClass === 'shaft' && cClass === 'mount_face') ||
    (pClass === 'mount_face' && cClass === 'shaft')
  if (!mismatched) return null

  return {
    ok: false,
    code: 'PORT_MISMATCH',
    message:
      `${child.link_name} (${child.component_id}) cannot mount on ` +
      `${parent.link_name}.${parentFace} (${parent.component_id}): ` +
      `${pClass} ↔ ${cClass} incompatible. Insert a structural bracket ` +
      `(structural_bracket_u / structural_servo_coupler_disc) between them, ` +
      `or attach the child to a structural extrusion near ${parent.link_name}.`,
    suggested_repair:
      `add_link(parent_link=${parent.link_name}, preset_id=structural_servo_coupler_disc, ` +
      `attach_face=${parentFace}) first, then attach ${child.component_id} to the bracket.`,
  }
}

// ── Mutation dispatchers ────────────────────────────────────────────────────

function _validateAndRespond(
  graph: AssemblyGraph,
  ctx: ValidationContext,
  summary: string,
): MutationResult {
  const { errors, warnings } = validateTopology(graph.components, ctx)
  if (errors.length > 0) {
    // Prefer the bracketed rule code if present; fall back to the first error.
    const first = errors[0]
    const codeMatch = first.match(/^\[([A-Z_]+)\]/)
    return {
      ok: false,
      code: codeMatch ? codeMatch[1] : 'VALIDATION_FAILED',
      message: errors.join(' | '),
      suggested_repair:
        codeMatch?.[1] === 'SHAFT_FANOUT' ? 'reparent the extra children to a structural extrusion'
        : codeMatch?.[1] === 'SENSOR_ON_ACTUATOR' ? 'attach the sensor to a structural extrusion near the actuator instead'
        : undefined,
    }
  }
  return { ok: true, graph, warnings, summary }
}

export function addLink(
  graph: AssemblyGraph,
  args: AddLinkArgs,
  ctx: ValidationContext,
): MutationResult {
  if (!ctx.findPreset(args.component_id)) {
    return {
      ok: false,
      code: 'UNKNOWN_COMPONENT',
      message: `component_id "${args.component_id}" is not in the preset library`,
      suggested_repair: 'pick a component_id from the catalog',
    }
  }

  const next = cloneAssemblyGraph(graph)
  const parent = next.components.find(c => c.link_name === args.parent_link)
  if (!parent) {
    return {
      ok: false,
      code: 'UNKNOWN_PARENT',
      message: `parent_link "${args.parent_link}" does not exist in the current graph`,
      suggested_repair: 'pass parent_link = one of the existing link_names',
    }
  }
  if (next.components.some(c => c.link_name === args.link_name)) {
    return {
      ok: false,
      code: 'DUPLICATE_LINK',
      message: `link_name "${args.link_name}" already exists`,
      suggested_repair: 'pick a unique link_name (convention: <component_id>_<N>)',
    }
  }

  const child: AssemblyComponent = {
    link_name: args.link_name,
    component_id: args.component_id,
    attach_to: args.parent_link,
    attach_face: args.attach_face || 'top',
    joint_type: args.joint_type ?? 'fixed',
    joint_axis: args.joint_axis ?? 'z',
    length_mm: args.length_mm,
    orientation: args.orientation,
    elevation_angle: args.elevation_angle,
    attach_rpy: args.attach_rpy,
    attach_connector: args.attach_connector,
    mate_connector: args.mate_connector,
    mate_type: args.mate_type,
  }

  const portErr = _checkPortCompatibility(ctx, parent, child)
  if (portErr) return portErr

  next.components.push(child)
  return _validateAndRespond(
    next, ctx,
    `added ${child.link_name} (${child.component_id}) on ${args.parent_link}.${child.attach_face}`,
  )
}

export function attachSensor(
  graph: AssemblyGraph,
  args: AttachSensorArgs,
  ctx: ValidationContext,
): MutationResult {
  if (!args.component_id.startsWith('sensor_')) {
    return {
      ok: false,
      code: 'NOT_A_SENSOR',
      message: `attach_sensor expects a sensor_* preset; got "${args.component_id}"`,
      suggested_repair: 'use add_link for non-sensor components',
    }
  }
  // Sensors are always fixed — rotating joints under a sensor mean the
  // camera/imu drifts with joint state, which is almost never the intent.
  return addLink(graph, {
    link_name: args.link_name,
    parent_link: args.parent_link,
    component_id: args.component_id,
    attach_face: args.mount_face,
    joint_type: 'fixed',
    joint_axis: 'z',
    elevation_angle: args.elevation_angle,
  }, ctx)
}

export function replaceComponent(
  graph: AssemblyGraph,
  args: ReplaceComponentArgs,
  ctx: ValidationContext,
): MutationResult {
  if (!ctx.findPreset(args.new_component_id)) {
    return {
      ok: false,
      code: 'UNKNOWN_COMPONENT',
      message: `component_id "${args.new_component_id}" is not in the preset library`,
    }
  }
  const next = cloneAssemblyGraph(graph)
  const target = next.components.find(c => c.link_name === args.link_name)
  if (!target) {
    return {
      ok: false,
      code: 'UNKNOWN_LINK',
      message: `link_name "${args.link_name}" does not exist`,
    }
  }
  const prevId = target.component_id
  target.component_id = args.new_component_id

  const parent = next.components.find(c => c.link_name === target.attach_to)
  const portErr = _checkPortCompatibility(ctx, parent, target)
  if (portErr) return portErr

  return _validateAndRespond(
    next, ctx,
    `replaced ${args.link_name}: ${prevId} → ${args.new_component_id}`,
  )
}

export function setJoint(
  graph: AssemblyGraph,
  args: SetJointArgs,
  ctx: ValidationContext,
): MutationResult {
  const validTypes = new Set(['fixed', 'revolute', 'prismatic', 'continuous'])
  if (!validTypes.has(args.joint_type)) {
    return {
      ok: false,
      code: 'BAD_JOINT_TYPE',
      message: `joint_type must be one of ${[...validTypes].join(', ')}; got "${args.joint_type}"`,
    }
  }
  const next = cloneAssemblyGraph(graph)
  const target = next.components.find(c => c.link_name === args.link_name)
  if (!target) {
    return {
      ok: false,
      code: 'UNKNOWN_LINK',
      message: `link_name "${args.link_name}" does not exist`,
    }
  }
  target.joint_type = args.joint_type
  if (args.joint_axis !== undefined) target.joint_axis = args.joint_axis
  if (args.attach_rpy !== undefined) target.attach_rpy = args.attach_rpy

  return _validateAndRespond(
    next, ctx,
    `joint on ${args.link_name} set to ${args.joint_type}` +
      (args.joint_axis ? ` axis=${args.joint_axis}` : ''),
  )
}

export function removeLink(
  graph: AssemblyGraph,
  args: RemoveLinkArgs,
  ctx: ValidationContext,
): MutationResult {
  const next = cloneAssemblyGraph(graph)
  const target = next.components.find(c => c.link_name === args.link_name)
  if (!target) {
    return {
      ok: false,
      code: 'UNKNOWN_LINK',
      message: `link_name "${args.link_name}" does not exist`,
    }
  }
  if (target.attach_to === null) {
    return {
      ok: false,
      code: 'ROOT_REMOVAL',
      message: `cannot remove root link "${args.link_name}" — every robot needs a base`,
      suggested_repair: 'use replace_component to swap the base, not remove_link',
    }
  }

  const directChildren = next.components.filter(c => c.attach_to === args.link_name)
  if (args.reparent_children && directChildren.length > 0) {
    // Graft the children onto the removed link's parent (preserves attach_face —
    // may produce overlaps; the placement engine handles spacing).
    for (const child of directChildren) child.attach_to = target.attach_to
    next.components = next.components.filter(c => c.link_name !== args.link_name)
    return _validateAndRespond(
      next, ctx,
      `removed ${args.link_name}, reparented ${directChildren.length} child(ren) to ${target.attach_to}`,
    )
  }

  // Default: cascade-delete the whole subtree (matches applyTopologyOps' remove op).
  const toRemove = new Set<string>([args.link_name])
  let changed = true
  while (changed) {
    changed = false
    for (const c of next.components) {
      if (c.attach_to && toRemove.has(c.attach_to) && !toRemove.has(c.link_name)) {
        toRemove.add(c.link_name)
        changed = true
      }
    }
  }
  const removedCount = toRemove.size
  next.components = next.components.filter(c => !toRemove.has(c.link_name))
  return _validateAndRespond(
    next, ctx,
    `removed ${args.link_name} and ${removedCount - 1} descendant(s)`,
  )
}

/** Thin dispatcher — routes a discriminated `GraphMutation` to the right mutator.
 *  Python-side tool dispatch goes through here so there's a single table for
 *  (tool_name ↔ mutator) instead of per-call if/else chains. */
export function applyMutation(
  graph: AssemblyGraph,
  mutation: GraphMutation,
  ctx: ValidationContext,
): MutationResult {
  switch (mutation.kind) {
    case 'add_link':          return addLink(graph, mutation.args, ctx)
    case 'attach_sensor':     return attachSensor(graph, mutation.args, ctx)
    case 'replace_component': return replaceComponent(graph, mutation.args, ctx)
    case 'set_joint':         return setJoint(graph, mutation.args, ctx)
    case 'remove_link':       return removeLink(graph, mutation.args, ctx)
  }
}
