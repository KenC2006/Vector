// Pure topology validation + auto-repair for AssemblyGraphs.
//
// No DOM, no THREE runtime deps at this module boundary — so the logic can be
// driven from a node-based corpus runner as well as from the browser-side
// urdfAssembly pipeline.

import { componentPortsForPreset, resolveFaceToPort } from './attachmentNodes.ts'
import type { AssemblyComponent, AssemblyGraph } from './urdfAssembly.ts'
import { getOrComputeBbox } from './componentDims.ts'

// Minimal shape of a preset that the validator needs. The real PresetComponent
// in urdfAssembly.ts is a superset of this; pass anything structurally compatible.
export interface ValidationPreset {
  id: string
  physical: {
    bounding_box_mm?: number[]
    cross_section_mm?: number[]
  }
  mounting_logic: Record<string, unknown>
}

export interface ValidationContext {
  findPreset: (componentId: string) => ValidationPreset | null
}

export interface ValidationResult {
  errors: string[]
  warnings: string[]
}

export type RepairKind =
  | 'duplicate_name'
  | 'effector_children'
  | 'sensor_on_actuator'
  | 'shaft_fanout'
  | 'port_mismatch_bracket'

export interface RepairLogEntry {
  kind: RepairKind
  message: string
}

export interface RepairResult {
  graph: AssemblyGraph
  repairs: RepairLogEntry[]
}

// Walk up from `start` to the nearest component whose id begins with `structural_`.
// Cycle-guarded because auto-repairs run before cycle validation.
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

function portsForComponent(
  preset: ValidationPreset,
) {
  const bb = getOrComputeBbox(preset.id, preset)
  return componentPortsForPreset(
    preset.id,
    bb[0] / 2000,
    bb[1] / 2000,
    bb[2] / 2000,
    preset.mounting_logic as { primary?: string; output?: string; shaft_diameter_mm?: number } | undefined,
  )
}

// Mirrors the oppositeFace table in urdfAssembly.ts — the child's contact face
// is the opposite of the parent's attach_face. Kept in sync manually; tiny
// enough that a shared constant isn't worth the cross-module coupling.
const OPPOSITE_FACE: Record<string, string> = {
  top: 'bottom', bottom: 'top',
  front: 'back', back: 'front',
  left: 'right', right: 'left',
}

function portClassAtFace(preset: ValidationPreset, face: string): string | undefined {
  return resolveFaceToPort(face, portsForComponent(preset))?.cls
}

export function validateTopology(
  components: AssemblyComponent[],
  ctx: ValidationContext,
): ValidationResult {
  const errors: string[] = []
  const warnings: string[] = []
  const linkNames = new Set(components.map(c => c.link_name))

  for (const comp of components) {
    // Rule 1: every non-root references a valid parent.
    if (comp.attach_to && !linkNames.has(comp.attach_to)) {
      errors.push(`${comp.link_name} references unknown parent "${comp.attach_to}"`)
    }
    // Rule 2: component_id must exist in the preset library.
    if (!ctx.findPreset(comp.component_id)) {
      errors.push(`Unknown component_id "${comp.component_id}" on ${comp.link_name}`)
    }
    // Rule 3: sensor parented to sensor (nothing to mount it rigidly to).
    if (comp.attach_to) {
      const parentComp = components.find(c => c.link_name === comp.attach_to)
      if (parentComp?.component_id.startsWith('sensor_') && comp.component_id.startsWith('sensor_')) {
        errors.push(`Sensor ${comp.link_name} attached to sensor ${comp.attach_to} — sensors should attach to structural/actuator links`)
      }
    }
    // Rule 4: effectors must be terminal.
    if (comp.component_id.startsWith('effector_')) {
      const hasChildren = components.some(c => c.attach_to === comp.link_name)
      if (hasChildren) {
        errors.push(`End effector ${comp.link_name} has children — effectors should be terminal nodes`)
      }
    }
    // Rule 5: link names must be unique.
    const dupes = components.filter(c => c.link_name === comp.link_name)
    if (dupes.length > 1) {
      errors.push(`Duplicate link_name "${comp.link_name}"`)
    }
  }

  // Rule 8 — SHAFT_FANOUT: >1 non-mobility child on a single-use shaft port.
  // Exception: all-mobility groups (differential drive) skipped — plausible intent.
  const shaftGroups = new Map<string, { children: string[]; componentIds: string[] }>()
  for (const comp of components) {
    if (!comp.attach_to) continue
    const parentDef = components.find(c => c.link_name === comp.attach_to)
    if (!parentDef) continue
    const pp = ctx.findPreset(parentDef.component_id)
    if (!pp) continue
    const face = comp.attach_face || 'top'
    const port = resolveFaceToPort(face, portsForComponent(pp))
    if (port?.cls === 'shaft' && port.single) {
      const key = `${comp.attach_to}::${face}`
      const entry = shaftGroups.get(key) || { children: [], componentIds: [] }
      entry.children.push(comp.link_name)
      entry.componentIds.push(comp.component_id)
      shaftGroups.set(key, entry)
    }
  }
  for (const [key, entry] of shaftGroups) {
    if (entry.children.length <= 1) continue
    const allMobility = entry.componentIds.every(id => id.startsWith('mobility_'))
    if (allMobility) continue
    const [parentName] = key.split('::')
    const extras = entry.children.slice(1).join(', ')
    errors.push(
      `[SHAFT_FANOUT] ${parentName}: shaft has ${entry.children.length} children (${entry.children.join(', ')}). A servo shaft drives exactly one load; extra children on the same shaft are mechanically invalid. Fix: keep one child on the shaft and reparent the others (${extras}) to the nearest structural extrusion.`,
    )
  }

  // Rule 9 — SENSOR_ON_ACTUATOR: sensor directly parented to an actuator/motor.
  const isActuatorId = (id: string) => id.startsWith('actuator_') || id.startsWith('motor_')
  const isSensorId = (id: string) => id.startsWith('sensor_')
  for (const comp of components) {
    if (!comp.attach_to) continue
    if (!isSensorId(comp.component_id)) continue
    const parentComp = components.find(c => c.link_name === comp.attach_to)
    if (!parentComp) continue
    if (isActuatorId(parentComp.component_id)) {
      errors.push(
        `[SENSOR_ON_ACTUATOR] ${comp.link_name}: sensor attached to ${parentComp.link_name} (${parentComp.component_id}). Sensors on actuators rotate/vibrate with the joint and have no rigid mounting face. Fix: attach ${comp.link_name} to a structural extrusion near the actuator instead.`,
      )
    }
  }

  // Rule 12 — DIRECT_SERVO_STACK (warning): actuator/motor directly parented to
  // another actuator/motor with no structural link between them. Emitted as a
  // warning because (a) the port-mismatch auto-repair already inserts a bracket
  // for most cases, and (b) a real physical assembly uses an extrusion for this,
  // not a bracket — the AI should learn to emit that pattern.
  //
  // Delivery: warnings flow through `resolveAssemblyGraph`'s `topologyWarnings`
  // return field into `viewportChat.ts`, where they surface inline in the chat
  // panel and append to the redesign-retry prompt so subsequent Claude turns
  // see them as feedback.
  for (const comp of components) {
    if (!comp.attach_to) continue
    if (!isActuatorId(comp.component_id)) continue
    const parentComp = components.find(c => c.link_name === comp.attach_to)
    if (!parentComp) continue
    if (isActuatorId(parentComp.component_id)) {
      warnings.push(
        `[DIRECT_SERVO_STACK] ${comp.link_name} (${comp.component_id}) mounts directly on ${parentComp.link_name} (${parentComp.component_id}). Insert a bracket or extrusion between them for a realistic assembly.`,
      )
    }
  }

  // Rule 6: exactly one root.
  const roots = components.filter(c => !c.attach_to)
  if (roots.length > 1) {
    errors.push(`Multiple root components: ${roots.map(r => r.link_name).join(', ')}`)
  }

  // Rule 7: no cycles (topological sort completes).
  const visited = new Set<string>()
  const remaining = components.filter(c => c.attach_to)
  let maxIter = remaining.length * 2
  const toProcess = [...remaining]
  if (roots.length > 0) visited.add(roots[0].link_name)
  while (toProcess.length > 0 && maxIter-- > 0) {
    const idx = toProcess.findIndex(c => visited.has(c.attach_to!))
    if (idx === -1) break
    visited.add(toProcess.splice(idx, 1)[0].link_name)
  }
  if (toProcess.length > 0) {
    errors.push(`Cycle or disconnected components: ${toProcess.map(c => c.link_name).join(', ')}`)
  }

  // Rule 10 — TIPPY_PROPORTIONS (only for baseplate-rooted robots).
  //
  // Emitted as a WARNING, not a hard error: the 5× threshold is unverified
  // (postmortem: test robot at 4.1× was likely unstable, so 5× may be too
  // lenient) AND only one baseplate preset exists at 200mm wide, so a tall
  // arm may be unsatisfiable by any preset combo — the same failure mode that
  // forced BASEPLATE_TOO_THIN's revert. Keep as warning until a fixture pair
  // + feasibility precheck are added; only then promote to a hard error.
  //
  // BASEPLATE_TOO_THIN was removed entirely for the same reason —
  // see project_topology_validator_attempt.md for the postmortem.
  const rootComp = components.find(c => !c.attach_to)
  if (rootComp && rootComp.component_id.startsWith('structural_baseplate')) {
    const rootPreset = ctx.findPreset(rootComp.component_id)
    if (rootPreset) {
      const rootBb = getOrComputeBbox(rootPreset.id, rootPreset)
      const baseW = Math.min(rootBb[0], rootBb[1])

      const heightOf = (comp: AssemblyComponent): number => {
        const p = ctx.findPreset(comp.component_id)
        if (!p) return 0
        const bb = getOrComputeBbox(p.id, p)
        let h = bb[2]
        if (comp.length_mm && p.physical.cross_section_mm) h = comp.length_mm
        return h
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
        warnings.push(
          `[TIPPY_PROPORTIONS] Robot is ~${Math.round(totalHeightMm)}mm tall but baseplate is only ${Math.round(baseW)}mm wide (${ratio}x ratio — unstable). Fix: use a wider baseplate (at least ${Math.round(totalHeightMm / 3)}mm across) or reduce arm height.`,
        )
      }
    }
  }

  return { errors, warnings }
}

// Auto-repairs mutate `graph.components` in place and return a repair log.
// Order: duplicate names → effector children → sensor-on-actuator → shaft fan-out.
export function autoRepairTopology(
  graph: AssemblyGraph,
  ctx: ValidationContext,
): RepairResult {
  const repairs: RepairLogEntry[] = []

  // Repair 1: duplicate link_names get incrementing suffix; later children
  // in the array that referenced the old name are rewritten to the new name.
  const seen = new Set<string>()
  for (let i = 0; i < graph.components.length; i++) {
    const comp = graph.components[i]
    if (!seen.has(comp.link_name)) {
      seen.add(comp.link_name)
      continue
    }
    const oldName = comp.link_name
    const baseName = oldName.replace(/_\d+$/, '')
    let suffix = 2
    while (seen.has(`${baseName}_${suffix}`)) suffix++
    const newName = `${baseName}_${suffix}`
    for (let j = i + 1; j < graph.components.length; j++) {
      if (graph.components[j].attach_to === oldName) {
        graph.components[j].attach_to = newName
      }
    }
    comp.link_name = newName
    seen.add(newName)
    repairs.push({ kind: 'duplicate_name', message: `"${oldName}" → "${newName}"` })
  }

  // Repair 2: an effector with children → reparent those children to the
  // effector's own parent (effectors must be terminal).
  for (const comp of graph.components) {
    if (!comp.component_id.startsWith('effector_')) continue
    const effectorChildren = graph.components.filter(c => c.attach_to === comp.link_name)
    if (effectorChildren.length === 0) continue
    for (const child of effectorChildren) {
      const oldParent = child.attach_to
      child.attach_to = comp.attach_to
      repairs.push({
        kind: 'effector_children',
        message: `"${child.link_name}" reparented from effector "${oldParent}" to "${child.attach_to}"`,
      })
    }
  }

  // Repair 3: sensor directly on actuator → nearest structural ancestor.
  for (const comp of graph.components) {
    if (!comp.component_id.startsWith('sensor_')) continue
    if (!comp.attach_to) continue
    const parent = graph.components.find(c => c.link_name === comp.attach_to)
    if (!parent) continue
    const parentIsActuator =
      parent.component_id.startsWith('actuator_') || parent.component_id.startsWith('motor_')
    if (!parentIsActuator) continue
    const newParent =
      findStructuralAncestor(graph.components, parent) ??
      graph.components.find(c => c.link_name === parent.attach_to)
    if (!newParent) continue
    comp.attach_to = newParent.link_name
    repairs.push({
      kind: 'sensor_on_actuator',
      message: `sensor "${comp.link_name}" moved off actuator "${parent.link_name}" → "${newParent.link_name}"`,
    })
  }

  // Repair 4: multiple non-mobility children on a single-use shaft → keep the
  // first, reparent the rest. All-mobility groups (diff-drive) are skipped.
  const shaftRepairGroups = new Map<string, AssemblyComponent[]>()
  for (const comp of graph.components) {
    if (!comp.attach_to) continue
    const parent = graph.components.find(c => c.link_name === comp.attach_to)
    if (!parent) continue
    const pp = ctx.findPreset(parent.component_id)
    if (!pp) continue
    const face = comp.attach_face || 'top'
    const port = resolveFaceToPort(face, portsForComponent(pp))
    if (port?.cls === 'shaft' && port.single) {
      const key = `${comp.attach_to}::${face}`
      const list = shaftRepairGroups.get(key) || []
      list.push(comp)
      shaftRepairGroups.set(key, list)
    }
  }
  for (const [key, children] of shaftRepairGroups) {
    if (children.length < 2) continue
    if (children.every(c => c.component_id.startsWith('mobility_'))) continue
    const parent = graph.components.find(c => c.link_name === children[0].attach_to)
    if (!parent) continue
    const newParent =
      findStructuralAncestor(graph.components, parent) ??
      graph.components.find(c => c.link_name === parent.attach_to)
    if (!newParent) continue
    for (let i = 1; i < children.length; i++) {
      const oldParent = children[i].attach_to
      children[i].attach_to = newParent.link_name
      repairs.push({
        kind: 'shaft_fanout',
        message: `shaft "${key}" extra "${children[i].link_name}" moved from "${oldParent}" to "${newParent.link_name}"`,
      })
    }
  }

  // Repair 5: shaft ↔ mount_face port mismatch → insert a structural bracket
  // between parent and child. Fires when an actuator/motor child connects to a
  // parent via a class mismatch (the quadruped "hip servo shaft facing the
  // baseplate" pattern, or a direct servo→servo stack). Skipped for mobility
  // children on motor shafts — wheels on shafts are a legitimate connection.
  // Skipped if the parent is already an auto-inserted bracket (idempotent).
  //
  // Limitation: inserting a bracket cleans the parent↔bracket connection
  // (mount_face ↔ mount_face) but leaves the bracket↔child mismatch intact
  // (brackets have only mount_face ports). The placement engine still emits
  // the compatibility warning on the child side, but the assembly graph now
  // reflects the structural intermediate a physical robot would have.
  const existingNames = new Set(graph.components.map(c => c.link_name))
  const isRepairableChild = (id: string) =>
    id.startsWith('actuator_') || id.startsWith('motor_')
  const insertions: Array<{ bracket: AssemblyComponent; beforeLinkName: string }> = []
  let bracketSerial = 0

  for (const comp of graph.components) {
    if (!comp.attach_to) continue
    if (!isRepairableChild(comp.component_id)) continue
    const parent = graph.components.find(c => c.link_name === comp.attach_to)
    if (!parent) continue
    // Idempotency: skip when parent is already a coupler-type structural.
    // Covers user-emitted brackets (structural_bracket_u / structural_bracket_l)
    // AND the auto-inserted servo coupler disc. Without this, a second run
    // of autoRepair (e.g., via modify_topology reverse-parse) would insert
    // another coupler between the existing coupler and the servo.
    if (
      parent.component_id.startsWith('structural_bracket_') ||
      parent.component_id === 'structural_servo_coupler_disc'
    ) continue
    const parentPreset = ctx.findPreset(parent.component_id)
    const childPreset = ctx.findPreset(comp.component_id)
    if (!parentPreset || !childPreset) continue
    const parentFace = comp.attach_face || 'top'
    const childFace = OPPOSITE_FACE[parentFace] || 'bottom'
    const pClass = portClassAtFace(parentPreset, parentFace)
    const cClass = portClassAtFace(childPreset, childFace)
    if (!pClass || !cClass) continue
    const mismatched =
      (pClass === 'shaft' && cClass === 'mount_face') ||
      (pClass === 'mount_face' && cClass === 'shaft')
    if (!mismatched) continue

    bracketSerial++
    let bracketName = `structural_bracket_auto_${bracketSerial}`
    while (existingNames.has(bracketName)) {
      bracketSerial++
      bracketName = `structural_bracket_auto_${bracketSerial}`
    }
    existingNames.add(bracketName)

    const bracket: AssemblyComponent = {
      link_name: bracketName,
      // Use the thin 25T servo coupler disc instead of the bulky U-bracket —
      // keeps shaft↔mount_face mechanically correct while collapsing the
      // visual gap at each junction from ~40mm to ~6mm.
      component_id: 'structural_servo_coupler_disc',
      attach_to: parent.link_name,
      attach_face: parentFace,
      joint_type: 'fixed',
      // 'z' resolves through urdfAssembly's axisMap to '0 0 1' intentionally,
      // instead of landing in the fallback branch via an unrecognized literal.
      joint_axis: 'z',
    }
    // Phase 3 task #7 / C4 (docs/ENGINE_EXECUTION_PLAN.md): route both
    // mismatch cases through the mate-connector resolver.
    //
    // Case 1 (shaft parent, mount_face child): coupler's shaft_hole
    // mates concentrically onto the parent's shaft_out. Without this,
    // a shaft-in-hole would be encoded as a flat stack in URDF and the
    // visual seam at the shaft tip wouldn't close.
    //
    // Case 2 (mount_face parent, shaft child): coupler mounts flat on
    // the parent face via default face connectors (fastened). Bit-
    // identical to the legacy bbox path while every parent still carries
    // default connectors, but flips the auto-repair path onto the
    // connector engine so it picks up authored parent connectors
    // automatically as Phase 2 lands them. Multi-child distribution is
    // preserved — computeMatePlacement threads totalOnFace/childIdx
    // through the connector resolver so N coupler discs still spread
    // across one face.
    if (pClass === 'shaft' && cClass === 'mount_face') {
      bracket.attach_connector = 'shaft_out'
      bracket.mate_connector = 'shaft_hole'
      bracket.mate_type = 'concentric'
    } else if (pClass === 'mount_face' && cClass === 'shaft') {
      bracket.attach_connector = parentFace
      bracket.mate_connector = childFace
      bracket.mate_type = 'fastened'
    }
    insertions.push({ bracket, beforeLinkName: comp.link_name })
    comp.attach_to = bracketName
    repairs.push({
      kind: 'port_mismatch_bracket',
      message: `inserted "${bracketName}" between "${parent.link_name}" (${pClass}) and "${comp.link_name}" (${cClass})`,
    })
  }

  // Splice brackets into the components array just before their paired child so
  // any order-sensitive consumer (e.g. placement's topo sort) sees parents first.
  for (const ins of insertions) {
    const childIdx = graph.components.findIndex(c => c.link_name === ins.beforeLinkName)
    if (childIdx >= 0) graph.components.splice(childIdx, 0, ins.bracket)
    else graph.components.push(ins.bracket)
  }

  return { graph, repairs }
}
