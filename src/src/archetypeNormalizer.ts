// Archetype normalizer (Phase 3.
//
// Deterministic post-processing for assembly graphs. Runs after the AI emits
// a graph and before topology auto-repair / placement, enforcing invariants
// the LLM should not have to remember on every turn.
//
// TS port of core/ai/archetype_normalizer.py — both must agree on the same
// rules (the plan calls out one shared compiler boundary; until that lands,
// keeping TS and Python in lockstep here is the substitute).
//
// Pure: no DOM, no THREE, no resolver — operates only on link_name /
// component_id / attach_to / attach_face. Diagnostics are structured records
// the caller can route per the plan (ai_topology / component_spec /
// placement_compiler / exporter owners).
//
// Note: topologyValidation.ts already enforces the foot_pad-has-children rule
// and auto-repairs it. The normalizer surfaces the same invariant as a
// structured diagnostic so the AI sees feedback labelled by owner.

import type { AssemblyComponent } from './urdfAssembly.ts'

export type ArchetypeOwner =
  | 'ai_topology'
  | 'component_spec'
  | 'placement_compiler'
  | 'exporter'

export type ArchetypeSeverity = 'info' | 'warning' | 'repaired'

export interface ArchetypeDiagnostic {
  code: string
  owner: ArchetypeOwner
  severity: ArchetypeSeverity
  message: string
  component?: string
  components?: string[]
  repair?: string
}

export interface RequestedFeatures {
  tail?: boolean
  articulated_head?: boolean
}

export interface NormalizeResult {
  components: AssemblyComponent[]
  diagnostics: ArchetypeDiagnostic[]
  archetype: string | null
}

function childrenOf(components: AssemblyComponent[], parent: string): AssemblyComponent[] {
  return components.filter(c => c.attach_to === parent)
}

function subtreeLinkNames(components: AssemblyComponent[], root: string): Set<string> {
  const out = new Set<string>()
  const pending = [root]
  while (pending.length) {
    const name = pending.pop()!
    if (out.has(name)) continue
    out.add(name)
    for (const child of childrenOf(components, name)) {
      if (child.link_name) pending.push(child.link_name)
    }
  }
  return out
}

function subtreeComponentIds(components: AssemblyComponent[], root: string): Set<string> {
  const names = subtreeLinkNames(components, root)
  const ids = new Set<string>()
  for (const c of components) if (names.has(c.link_name)) ids.add(c.component_id || '')
  return ids
}

export function detectArchetype(components: AssemblyComponent[]): string | null {
  const footCount = components.filter(c => c.component_id === 'mobility_rubber_foot_pad').length
  if (footCount >= 4) return 'quadruped'
  return null
}

function normalizeQuadruped(
  components: AssemblyComponent[],
  features: RequestedFeatures,
): NormalizeResult {
  const diagnostics: ArchetypeDiagnostic[] = []
  const allowTail = !!features.tail

  const baseNames = new Set(
    components.filter(c => c.component_id?.startsWith('structural_baseplate')).map(c => c.link_name),
  )

  // Cosmetic-tail removal: subtree off baseplate's back face (or named *tail*)
  // with no functional terminal (foot pad / sensor / compute / power).
  const toRemove = new Set<string>()
  for (const comp of components) {
    if (!baseNames.has(comp.attach_to ?? '')) continue
    const linkName = comp.link_name || ''
    const cid = comp.component_id || ''
    if (comp.attach_face !== 'back' && !linkName.toLowerCase().includes('tail')) continue

    const subtreeIds = subtreeComponentIds(components, linkName)
    const hasFunctionalTerminal = Array.from(subtreeIds).some(
      sid =>
        sid === 'mobility_rubber_foot_pad' ||
        sid.startsWith('sensor_') ||
        sid.startsWith('compute_') ||
        sid.startsWith('power_'),
    )
    const tailLike =
      cid.startsWith('actuator_servo') ||
      cid.startsWith('actuator_high_speed') ||
      cid === 'structural_limb_link_slim' ||
      linkName.toLowerCase().includes('tail')

    if (!(tailLike && !hasFunctionalTerminal)) continue

    if (allowTail) {
      diagnostics.push({
        code: 'quadruped_tail_present',
        owner: 'ai_topology',
        severity: 'info',
        component: linkName,
        message: `tail subtree '${linkName}' kept (requested_features.tail=true)`,
      })
      continue
    }

    for (const n of subtreeLinkNames(components, linkName)) toRemove.add(n)
    diagnostics.push({
      code: 'quadruped_tail_forbidden',
      owner: 'ai_topology',
      severity: 'repaired',
      component: linkName,
      repair: 'remove_subtree',
      message: `removed default tail chain '${linkName}' (no foot/sensor/compute/power terminal)`,
    })
  }

  const newComponents = components.filter(c => !toRemove.has(c.link_name))

  // Foot pad terminal-leaf invariant (warn-only; topologyValidation auto-repairs).
  for (const comp of newComponents) {
    if (comp.component_id !== 'mobility_rubber_foot_pad') continue
    const kids = childrenOf(newComponents, comp.link_name)
    if (kids.length === 0) continue
    diagnostics.push({
      code: 'foot_pad_has_children',
      owner: 'ai_topology',
      severity: 'warning',
      component: comp.link_name,
      components: kids.map(k => k.link_name),
      repair: 'reparent_children_to_shin',
      message: `rubber foot pad '${comp.link_name}' has ${kids.length} child(ren); foot pads are terminal leaves`,
    })
  }

  // Single-root invariant (warn-only).
  const roots = newComponents.filter(c => !c.attach_to)
  if (roots.length > 1) {
    diagnostics.push({
      code: 'multiple_roots',
      owner: 'ai_topology',
      severity: 'warning',
      components: roots.map(r => r.link_name),
      message: `${roots.length} root components; quadruped expects exactly one base`,
    })
  }

  return { components: newComponents, diagnostics, archetype: 'quadruped' }
}

const NORMALIZERS: Record<string, (c: AssemblyComponent[], f: RequestedFeatures) => NormalizeResult> = {
  quadruped: normalizeQuadruped,
}

export function normalizeAssembly(
  components: AssemblyComponent[],
  features: RequestedFeatures = {},
): NormalizeResult {
  const archetype = detectArchetype(components)
  if (archetype === null) return { components, diagnostics: [], archetype: null }
  const fn = NORMALIZERS[archetype]
  if (!fn) {
    return {
      components,
      diagnostics: [{
        code: 'archetype_unknown',
        owner: 'ai_topology',
        severity: 'info',
        message: `detected archetype '${archetype}' has no normalizer registered`,
      }],
      archetype,
    }
  }
  return fn(components, features)
}

/** Render a diagnostic as a human-readable line for the redesign-retry prompt.
 * Owner tag lets the AI distinguish "you (the LLM) did this" from "the spec
 * is wrong" or "the placement compiler is wrong" — per plan §3.8 routing. */
export function formatDiagnosticForPrompt(d: ArchetypeDiagnostic): string {
  return `[${d.owner}/${d.severity}] ${d.code}: ${d.message}`
}
