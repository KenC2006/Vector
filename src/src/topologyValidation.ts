// Pure topology validation for AssemblyGraphs — structured diagnostics, no
// graph mutation.
//
// WS2 of the assembler refactor replaced the silent auto-repair pass
// (autoRepairTopology) with structured warnings: the validator DESCRIBES what
// looks wrong and suggests the concrete edit, and Claude applies (or argues
// with) the suggestion on its next turn. Only structural impossibilities are
// hard errors that block compilation:
//
//   UNKNOWN_PARENT / UNKNOWN_COMPONENT / DUPLICATE_LINK_NAME /
//   MULTIPLE_ROOTS / CYCLE
//
// Everything that used to be silently rewritten (effector children, foot-pad
// children, sensors on actuators, shaft fan-out, bare tires, servo spacers,
// port mismatches) is now a warning with a `suggested_repair`.
//
// No DOM, no THREE runtime deps at this module boundary — so the logic can be
// driven from a node-based corpus runner as well as from the browser-side
// urdfAssembly pipeline.

import { resolveFaceToPort } from './attachmentNodes.ts'
import type { AssemblyComponent } from './urdfAssembly.ts'
import {
  isDrivetrainComponentId,
  isFootPadComponentId,
  isTireComponentId,
  resolveComponent,
  resolveComponentHalfBoundsMm,
} from './componentResolver.ts'
import type { MateConnector } from './mateConnectors.ts'
import {
  type Diagnostic,
  DiagnosticOwner,
  routeDiagnostics,
} from './compilerDiagnostics.ts'

// Minimal shape of a preset that the validator needs. The real PresetComponent
// in urdfAssembly.ts is a superset of this; pass anything structurally compatible.
export interface ValidationPreset {
  id: string
  physical: {
    bounding_box_mm?: number[]
    cross_section_mm?: number[]
  }
  mechanical_electrical: Record<string, unknown>
  mounting_logic: Record<string, unknown>
  /** Authored mate connectors. Merged over the 6 default face connectors
   *  (top/bottom/front/back/left/right) by id. */
  connectors?: MateConnector[]
}

export interface ValidationContext {
  findPreset: (componentId: string) => ValidationPreset | null
}

/** One validation finding. Same field vocabulary as graphMutations.MutationError
 * so the tool-call loop and the batch design path speak one language. */
export interface StructuredDiagnostic {
  severity: 'error' | 'warning'
  /** Stable SCREAMING_SNAKE rule code Claude can pattern-match on. */
  code: string
  message: string
  link_name?: string
  /** Concrete edit that would resolve the finding. */
  suggested_repair?: string
}

export interface ValidationResult {
  errors: string[]
  warnings: string[]
}

/** Render a diagnostic in the legacy string format consumed by
 * resolveAssemblyGraph / viewportChat / the redesign-retry prompt. */
export function formatStructuredDiagnostic(d: StructuredDiagnostic): string {
  const fix = d.suggested_repair ? ` Fix: ${d.suggested_repair}` : ''
  return `[${d.code}] ${d.message}${fix}`
}

/** Owner-tagged view for the diagnostics router. Topology findings are
 * AI-fixable by definition (they describe wrong parents/children/placements
 * that the next AI turn should address). */
export function validateTopologyRouted(
  components: AssemblyComponent[],
  ctx: ValidationContext,
): { diagnostics: Diagnostic[]; routed: Record<DiagnosticOwner, Diagnostic[]> } {
  const structured = validateTopologyStructured(components, ctx)
  const diagnostics: Diagnostic[] = structured.map(d => ({
    code: d.code.toLowerCase(),
    severity: d.severity,
    owner: DiagnosticOwner.AiTopology,
    message: formatStructuredDiagnostic(d),
  }))
  return { diagnostics, routed: routeDiagnostics(diagnostics) }
}

function isSplitServoComponentId(componentId: string): boolean {
  return (
    componentId.startsWith('actuator_servo') ||
    componentId.startsWith('actuator_continuous_rotation_servo') ||
    componentId.startsWith('actuator_high_speed')
  )
}

// Walk up from `start` to the nearest component whose id begins with `structural_`.
// Cycle-guarded because callers may run before cycle validation.
export function findStructuralAncestor(
  components: AssemblyComponent[],
  start: AssemblyComponent | undefined,
): AssemblyComponent | undefined {
  let anc = start
  const visited = new Set<string>()
  while (anc) {
    if (visited.has(anc.link_name)) return undefined
    visited.add(anc.link_name)
    if (anc.component_id.startsWith('structural_')) return anc
    if (!anc.attach_to) return undefined
    const nextName: string = anc.attach_to
    anc = components.find(c => c.link_name === nextName)
  }
  return undefined
}

function portsForComponent(preset: ValidationPreset) {
  return resolveComponent({ spec: preset }).ports
}

function resolvePresetBoundsMm(
  preset: ValidationPreset,
  instance?: { length_mm?: number },
): { hxMm: number; hyMm: number; hzMm: number } {
  return resolveComponentHalfBoundsMm(preset, instance)
}

// Mirrors the oppositeFace table in urdfAssembly.ts.
const OPPOSITE_FACE: Record<string, string> = {
  top: 'bottom', bottom: 'top',
  front: 'back', back: 'front',
  left: 'right', right: 'left',
  coaxial: 'coaxial',
}

function portClassAtFace(preset: ValidationPreset, face: string): string | undefined {
  return resolveFaceToPort(face, portsForComponent(preset))?.cls
}

// ── Shared port-compatibility check ─────────────────────────────────────────
// Single source of truth for shaft↔mount_face pairing, used by BOTH edit
// surfaces: validateTopologyStructured (design_robot batch path) and
// graphMutations (tool-call loop). A mismatch is a WARNING — sometimes a
// "wrong" pairing is a deliberate creative choice — but the suggested_repair
// names the coupler-disc pattern a physical assembly would use.

export function checkPortCompatibility(
  ctx: ValidationContext,
  parent: AssemblyComponent | undefined,
  child: AssemblyComponent,
): StructuredDiagnostic | null {
  if (!parent) return null
  const isActuator = (id: string) => id.startsWith('actuator_') || id.startsWith('motor_')
  if (!isActuator(child.component_id)) return null
  // Split servos carry their own body holder + horn adapter internally.
  if (isSplitServoComponentId(child.component_id)) return null
  // Brackets and coupler discs exist precisely to mate shaft↔mount_face — skip.
  if (
    parent.component_id.startsWith('structural_bracket_') ||
    parent.component_id === 'structural_servo_coupler_disc'
  ) return null

  const parentPreset = ctx.findPreset(parent.component_id)
  const childPreset = ctx.findPreset(child.component_id)
  if (!parentPreset || !childPreset) return null

  const parentFace = child.attach_face || 'top'
  const childFace = OPPOSITE_FACE[parentFace] || 'bottom'
  const pClass = portClassAtFace(parentPreset, parentFace)
  const cClass = portClassAtFace(childPreset, childFace)
  if (!pClass || !cClass) return null

  const mismatched =
    (pClass === 'shaft' && cClass === 'mount_face') ||
    (pClass === 'mount_face' && cClass === 'shaft')
  if (!mismatched) return null

  return {
    severity: 'warning',
    code: 'PORT_MISMATCH',
    link_name: child.link_name,
    message:
      `${child.link_name} (${child.component_id}) mounts on ` +
      `${parent.link_name}.${parentFace} (${parent.component_id}) with ` +
      `incompatible port classes (${pClass} ↔ ${cClass}).`,
    suggested_repair:
      `insert a structural_servo_coupler_disc between ${parent.link_name} and ` +
      `${child.link_name} (attach the coupler to ${parent.link_name}.${parentFace}, ` +
      `then attach ${child.link_name} to the coupler), or attach ${child.link_name} ` +
      `to a structural extrusion near ${parent.link_name}.`,
  }
}

// ── Structured validation ────────────────────────────────────────────────────

export function validateTopologyStructured(
  components: AssemblyComponent[],
  ctx: ValidationContext,
): StructuredDiagnostic[] {
  const out: StructuredDiagnostic[] = []
  const linkNames = new Set(components.map(c => c.link_name))
  const byName = new Map<string, AssemblyComponent>()
  for (const c of components) byName.set(c.link_name, c)

  // ── Hard errors ────────────────────────────────────────────────────────────

  const seenNames = new Set<string>()
  for (const comp of components) {
    if (comp.attach_to && !linkNames.has(comp.attach_to)) {
      out.push({
        severity: 'error', code: 'UNKNOWN_PARENT', link_name: comp.link_name,
        message: `${comp.link_name} references unknown parent "${comp.attach_to}".`,
        suggested_repair: 'set attach_to to an existing link_name',
      })
    }
    if (!ctx.findPreset(comp.component_id)) {
      out.push({
        severity: 'error', code: 'UNKNOWN_COMPONENT', link_name: comp.link_name,
        message: `Unknown component_id "${comp.component_id}" on ${comp.link_name}.`,
        suggested_repair: 'pick a component_id from the catalog',
      })
    }
    if (seenNames.has(comp.link_name)) {
      out.push({
        severity: 'error', code: 'DUPLICATE_LINK_NAME', link_name: comp.link_name,
        message: `Duplicate link_name "${comp.link_name}".`,
        suggested_repair: `rename one occurrence (convention: <component_id>_<N>)`,
      })
    }
    seenNames.add(comp.link_name)
  }

  const roots = components.filter(c => !c.attach_to)
  if (roots.length > 1) {
    out.push({
      severity: 'error', code: 'MULTIPLE_ROOTS',
      message: `Multiple root components: ${roots.map(r => r.link_name).join(', ')}.`,
      suggested_repair: 'exactly one component may have attach_to: null — attach the others to it',
    })
  }

  // Cycles / disconnected components (topological sort must complete).
  {
    const visited = new Set<string>()
    const remaining = components.filter(c => c.attach_to)
    let maxIter = remaining.length * 2
    const toProcess = [...remaining]
    for (const r of roots) visited.add(r.link_name)
    while (toProcess.length > 0 && maxIter-- > 0) {
      const idx = toProcess.findIndex(c => visited.has(c.attach_to!))
      if (idx === -1) break
      visited.add(toProcess.splice(idx, 1)[0].link_name)
    }
    if (toProcess.length > 0) {
      out.push({
        severity: 'error', code: 'CYCLE',
        message: `Cycle or disconnected components: ${toProcess.map(c => c.link_name).join(', ')}.`,
        suggested_repair: 'break the attach_to cycle so the topology forms a tree rooted at the base link',
      })
    }
  }

  // ── Warnings (the former auto-repairs, as feedback) ───────────────────────

  for (const comp of components) {
    // Sensor parented to sensor — nothing rigid to mount to.
    if (comp.attach_to && comp.component_id.startsWith('sensor_')) {
      const parentComp = byName.get(comp.attach_to)
      if (parentComp?.component_id.startsWith('sensor_')) {
        out.push({
          severity: 'warning', code: 'SENSOR_ON_SENSOR', link_name: comp.link_name,
          message: `Sensor ${comp.link_name} is attached to sensor ${comp.attach_to}.`,
          suggested_repair: `attach ${comp.link_name} to a structural or actuator link instead`,
        })
      }
    }

    // Effectors are conventionally terminal. Creative reuse (a feeler past a
    // pincer) is allowed — hence warning, not error, and no rewrite.
    if (comp.component_id.startsWith('effector_')) {
      const kids = components.filter(c => c.attach_to === comp.link_name)
      if (kids.length > 0) {
        out.push({
          severity: 'warning', code: 'EFFECTOR_HAS_CHILDREN', link_name: comp.link_name,
          message:
            `End effector ${comp.link_name} has children (${kids.map(k => k.link_name).join(', ')}). ` +
            `Children of an effector move with its jaws/tool.`,
          suggested_repair:
            `if unintended, reparent them to ${comp.attach_to ?? 'the effector\'s parent'}; ` +
            `keep them only for deliberate designs (e.g. a sensor feeler on a pincer)`,
        })
      }
    }

    // Foot pads are conventionally terminal leaf nodes.
    if (isFootPadComponentId(comp.component_id)) {
      const kids = components.filter(c => c.attach_to === comp.link_name)
      if (kids.length > 0) {
        out.push({
          severity: 'warning', code: 'FOOT_PAD_HAS_CHILDREN', link_name: comp.link_name,
          message:
            `${comp.link_name} (mobility_rubber_foot_pad) has children ` +
            `(${kids.map(k => k.link_name).join(', ')}). Foot pads are auto-leveled ` +
            `ground contacts; children inherit that leveling.`,
          suggested_repair:
            `if these are limb segments, reparent them to the shin/extrusion above the foot pad`,
        })
      }
    }
  }

  // SHAFT_FANOUT: >1 child on a single-use shaft port.
  {
    const shaftGroups = new Map<string, { children: string[] }>()
    for (const comp of components) {
      if (!comp.attach_to) continue
      const parentDef = byName.get(comp.attach_to)
      if (!parentDef) continue
      const pp = ctx.findPreset(parentDef.component_id)
      if (!pp) continue
      const face = comp.attach_face || 'top'
      const port = resolveFaceToPort(face, portsForComponent(pp))
      if (port?.cls === 'shaft' && port.single) {
        const key = `${comp.attach_to}::${face}`
        const entry = shaftGroups.get(key) || { children: [] }
        entry.children.push(comp.link_name)
        shaftGroups.set(key, entry)
      }
    }
    for (const [key, entry] of shaftGroups) {
      if (entry.children.length <= 1) continue
      const [parentName] = key.split('::')
      const extras = entry.children.slice(1).join(', ')
      out.push({
        severity: 'warning', code: 'SHAFT_FANOUT', link_name: parentName,
        message:
          `${parentName}: shaft has ${entry.children.length} children ` +
          `(${entry.children.join(', ')}). A shaft drives exactly one load.`,
        suggested_repair:
          `keep one child on the shaft and reparent the others (${extras}) to the nearest structural extrusion`,
      })
    }
  }

  // SENSOR_ON_ACTUATOR: sensor directly parented to an actuator/motor.
  {
    const isActuatorId = (id: string) => id.startsWith('actuator_') || id.startsWith('motor_')
    for (const comp of components) {
      if (!comp.attach_to || !comp.component_id.startsWith('sensor_')) continue
      const parentComp = byName.get(comp.attach_to)
      if (!parentComp || !isActuatorId(parentComp.component_id)) continue
      const structural = findStructuralAncestor(components, parentComp)
      out.push({
        severity: 'warning', code: 'SENSOR_ON_ACTUATOR', link_name: comp.link_name,
        message:
          `${comp.link_name}: sensor attached to ${parentComp.link_name} ` +
          `(${parentComp.component_id}). Sensors on actuators rotate/vibrate with ` +
          `the joint and have no rigid mounting face.`,
        suggested_repair:
          `attach ${comp.link_name} to ${structural ? structural.link_name : 'a structural extrusion near the actuator'} instead`,
      })
    }
  }

  // DIRECT_SERVO_STACK: actuator directly on actuator with no structural link.
  {
    const isActuatorId = (id: string) => id.startsWith('actuator_') || id.startsWith('motor_')
    for (const comp of components) {
      if (!comp.attach_to || !isActuatorId(comp.component_id)) continue
      const parentComp = byName.get(comp.attach_to)
      if (!parentComp || !isActuatorId(parentComp.component_id)) continue
      // Split servos stacking on split servos form the compound 2-DOF joint —
      // the compiler inserts the carrier bracket itself.
      if (
        isSplitServoComponentId(comp.component_id) &&
        isSplitServoComponentId(parentComp.component_id)
      ) continue
      out.push({
        severity: 'warning', code: 'DIRECT_SERVO_STACK', link_name: comp.link_name,
        message:
          `${comp.link_name} (${comp.component_id}) mounts directly on ` +
          `${parentComp.link_name} (${parentComp.component_id}).`,
        suggested_repair: 'insert a bracket or extrusion between them for a realistic assembly',
      })
    }
  }

  // SERVO_SPACER: explicit coupler/bracket spacer inside a servo chain. The
  // compiler's split servos own their yoke + horn-link adapter internally, so
  // an authored spacer usually doubles the offset. Deliberate adapters for
  // creative articulation are legitimate — warning only.
  for (const comp of components) {
    const parent = comp.attach_to ? byName.get(comp.attach_to) : undefined
    const kids = components.filter(c => c.attach_to === comp.link_name)
    const isServoCoupler = comp.component_id === 'structural_servo_coupler_disc'
    const isServoToServoBracket = comp.component_id.startsWith('structural_bracket_')
      && !!parent
      && isSplitServoComponentId(parent.component_id)
      && kids.some(c => isSplitServoComponentId(c.component_id))
    if (!isServoCoupler && !isServoToServoBracket) continue
    const touchesServo = !!(
      (parent && isSplitServoComponentId(parent.component_id)) ||
      kids.some(c => isSplitServoComponentId(c.component_id))
    )
    if (!touchesServo) continue
    out.push({
      severity: 'warning', code: 'SERVO_SPACER', link_name: comp.link_name,
      message:
        `${comp.link_name} (${comp.component_id}) is a spacer inside a servo chain. ` +
        `Split servos already include their own yoke and horn adapter, so an extra ` +
        `spacer offsets the next joint twice.`,
      suggested_repair:
        `remove ${comp.link_name} and attach its children directly to ` +
        `${comp.attach_to ?? 'the servo'} — keep it only if the extra offset is deliberate`,
    })
  }

  // BARE_TIRE: tire attached to a non-drivetrain parent. No spin axis.
  for (const comp of components) {
    if (!isTireComponentId(comp.component_id) || !comp.attach_to) continue
    const parentComp = byName.get(comp.attach_to)
    if (!parentComp || isDrivetrainComponentId(parentComp.component_id)) continue
    out.push({
      severity: 'warning', code: 'BARE_TIRE', link_name: comp.link_name,
      message:
        `${comp.link_name} (${comp.component_id}) is attached directly to ` +
        `${parentComp.link_name} (${parentComp.component_id}) with no drivetrain — ` +
        `it has no spin axis and will not roll.`,
      suggested_repair:
        `insert a drivetrain_hub_motor_80 (attach_face=${comp.attach_face || 'bottom'}, ` +
        `joint_type=continuous, joint_axis=y) between ${parentComp.link_name} and the tire, ` +
        `then attach the tire to it with attach_face=coaxial — unless the wheel is deliberately decorative`,
    })
  }

  // PORT_MISMATCH: shaft↔mount_face pairing without a bracket.
  for (const comp of components) {
    if (!comp.attach_to) continue
    const parentComp = byName.get(comp.attach_to)
    const diag = checkPortCompatibility(ctx, parentComp, comp)
    if (diag) out.push(diag)
  }

  // TIPPY_PROPORTIONS (only for baseplate-rooted robots).
  const rootComp = components.find(c => !c.attach_to)
  if (rootComp && rootComp.component_id.startsWith('structural_baseplate')) {
    const rootPreset = ctx.findPreset(rootComp.component_id)
    if (rootPreset) {
      const rootBounds = resolvePresetBoundsMm(rootPreset, rootComp)
      const baseW = Math.min(rootBounds.hxMm * 2, rootBounds.hyMm * 2)

      const heightOf = (comp: AssemblyComponent): number => {
        const p = ctx.findPreset(comp.component_id)
        if (!p) return 0
        return resolvePresetBoundsMm(p, comp).hzMm * 2
      }
      const stacksVertically = (child: AssemblyComponent): boolean => {
        const f = child.attach_face ?? 'top'
        return f === 'top' || f === 'bottom'
      }
      const chainHeight = (comp: AssemblyComponent, seen: Set<string>): number => {
        if (seen.has(comp.link_name)) return 0
        seen.add(comp.link_name)
        const stacking = components.filter(c => c.attach_to === comp.link_name && stacksVertically(c))
        const own = heightOf(comp)
        if (stacking.length === 0) return own
        return own + Math.max(...stacking.map(c => chainHeight(c, seen)))
      }
      const totalHeightMm = chainHeight(rootComp, new Set<string>())

      if (baseW > 0 && totalHeightMm / baseW > 5) {
        const ratio = (totalHeightMm / baseW).toFixed(1)
        out.push({
          severity: 'warning', code: 'TIPPY_PROPORTIONS', link_name: rootComp.link_name,
          message:
            `Robot is ~${Math.round(totalHeightMm)}mm tall but the baseplate is only ` +
            `${Math.round(baseW)}mm wide (${ratio}x ratio — unstable).`,
          suggested_repair:
            `use a wider baseplate (at least ${Math.round(totalHeightMm / 3)}mm across) or reduce the height`,
        })
      }
    }
  }

  return out
}

/** Legacy string view — errors block compilation, warnings flow to the
 * redesign loop. Thin adapter over validateTopologyStructured. */
export function validateTopology(
  components: AssemblyComponent[],
  ctx: ValidationContext,
): ValidationResult {
  const structured = validateTopologyStructured(components, ctx)
  return {
    errors: structured.filter(d => d.severity === 'error').map(formatStructuredDiagnostic),
    warnings: structured.filter(d => d.severity === 'warning').map(formatStructuredDiagnostic),
  }
}
