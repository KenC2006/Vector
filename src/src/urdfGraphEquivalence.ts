// Pure AssemblyGraph utilities — deliberately free of THREE / DOM imports so
// Workstream #1 (AssemblyGraph Preservation) can test and reuse these from
// Node without pulling in the whole rendering stack.
//
// `cloneAssemblyGraph` is used to stash the canonical graph into localStorage
// and to hand snapshots to Claude / the chat layer without exposing the
// in-memory source-of-truth to mutation.
//
// `graphsEquivalent` is the structural equality check. The Plan's Workstream #1
// item 5 requires divergence to be *detected and logged*, not silently
// accepted — so anywhere we reverse-parse URDF as a fallback, we have a
// diagnostic to compare against the canonical.

export interface AssemblyComponent {
  link_name: string
  component_id: string
  attach_to: string | null
  attach_face: string | null
  joint_type: string
  joint_axis: string
  length_mm?: number
  /** 'horizontal' | 'vertical' | 'auto' or a numeric string in degrees (e.g. '45') for yaw rotation around face normal */
  orientation?: string
  /** Degrees of upward/downward tilt for side-face (front/back/left/right) attachments. Positive = upward. */
  elevation_angle?: number
  /** Explicit rest-pose [roll, pitch, yaw] in radians. When any component is non-zero, overrides
   *  the auto-computed joint rpy (placement + arm rest-pose). Used for Z-crouch quadruped poses etc. */
  attach_rpy?: number[]
}

export interface AssemblyGraph {
  base_link: string
  ground_offset?: boolean
  components: AssemblyComponent[]
}

/** Deep clone an AssemblyGraph — use when you don't want callers to mutate the canonical copy. */
export function cloneAssemblyGraph(graph: AssemblyGraph): AssemblyGraph {
  return {
    base_link: graph.base_link,
    ground_offset: graph.ground_offset,
    components: graph.components.map(c => ({
      ...c,
      attach_rpy: c.attach_rpy ? [...c.attach_rpy] : undefined,
    })),
  }
}

/** Result of a structural graph comparison. When `equal` is false, `differences` is
 *  a human-readable list of fields/components that disagreed — not a machine-diff. */
export interface GraphEquivalenceResult {
  equal: boolean
  differences: string[]
}

const _EPS_NUM = 1e-6
// Matches the override threshold in resolveAssemblyGraph (~0.057°). Treating
// "absent" and "all-zero within EPS" as equal keeps graphsEquivalent aligned
// with runtime behavior — otherwise a perfectly equivalent graph would
// spuriously report drift.
const _EPS_RPY = 1e-3

function _numsEqual(a: number | undefined, b: number | undefined, eps: number): boolean {
  if (a === undefined && b === undefined) return true
  if (a === undefined || b === undefined) return false
  return Math.abs(a - b) <= eps
}

function _rpyEqual(a: number[] | undefined, b: number[] | undefined): boolean {
  const aActive = !!a && a.some(v => Math.abs(v) > _EPS_RPY)
  const bActive = !!b && b.some(v => Math.abs(v) > _EPS_RPY)
  if (!aActive && !bActive) return true
  if (aActive !== bActive) return false
  for (let i = 0; i < 3; i++) {
    if (!_numsEqual(a![i], b![i], _EPS_RPY)) return false
  }
  return true
}

/** Structural + parametric equality for two AssemblyGraphs.
 *
 *  Components are matched by `link_name` (order-independent). A mismatch on
 *  base_link, ground_offset, component set, or any per-component field
 *  (component_id, attach_to, attach_face, joint_*, length_mm, orientation,
 *  elevation_angle, attach_rpy) is reported as a difference.
 *
 *  Numeric fields use small tolerances; `attach_rpy` uses the same EPS the
 *  runtime uses to decide whether to apply the override (so "absent" and
 *  "all-zero" are considered equal — they produce identical placement). */
export function graphsEquivalent(a: AssemblyGraph, b: AssemblyGraph): GraphEquivalenceResult {
  const diffs: string[] = []
  if (a.base_link !== b.base_link) {
    diffs.push(`base_link: "${a.base_link}" vs "${b.base_link}"`)
  }
  if (!!a.ground_offset !== !!b.ground_offset) {
    diffs.push(`ground_offset: ${!!a.ground_offset} vs ${!!b.ground_offset}`)
  }

  const byName = (g: AssemblyGraph) => {
    const m = new Map<string, AssemblyComponent>()
    for (const c of g.components) m.set(c.link_name, c)
    return m
  }
  const aMap = byName(a)
  const bMap = byName(b)

  for (const name of aMap.keys()) {
    if (!bMap.has(name)) diffs.push(`component only in A: ${name}`)
  }
  for (const name of bMap.keys()) {
    if (!aMap.has(name)) diffs.push(`component only in B: ${name}`)
  }

  for (const [name, ac] of aMap) {
    const bc = bMap.get(name)
    if (!bc) continue
    const fieldDiffs: string[] = []
    if (ac.component_id !== bc.component_id) fieldDiffs.push(`component_id ${ac.component_id}≠${bc.component_id}`)
    if ((ac.attach_to ?? null) !== (bc.attach_to ?? null)) fieldDiffs.push(`attach_to ${ac.attach_to}≠${bc.attach_to}`)
    if ((ac.attach_face ?? null) !== (bc.attach_face ?? null)) fieldDiffs.push(`attach_face ${ac.attach_face}≠${bc.attach_face}`)
    if ((ac.joint_type ?? '') !== (bc.joint_type ?? '')) fieldDiffs.push(`joint_type ${ac.joint_type}≠${bc.joint_type}`)
    if ((ac.joint_axis ?? '') !== (bc.joint_axis ?? '')) fieldDiffs.push(`joint_axis ${ac.joint_axis}≠${bc.joint_axis}`)
    if (!_numsEqual(ac.length_mm, bc.length_mm, _EPS_NUM)) fieldDiffs.push(`length_mm ${ac.length_mm}≠${bc.length_mm}`)
    if ((ac.orientation ?? '') !== (bc.orientation ?? '')) fieldDiffs.push(`orientation ${ac.orientation}≠${bc.orientation}`)
    if (!_numsEqual(ac.elevation_angle, bc.elevation_angle, _EPS_NUM)) fieldDiffs.push(`elevation_angle ${ac.elevation_angle}≠${bc.elevation_angle}`)
    if (!_rpyEqual(ac.attach_rpy, bc.attach_rpy)) {
      const fmtRpy = (r?: number[]) => r ? `[${r.map(v => v.toFixed(3)).join(',')}]` : 'undefined'
      fieldDiffs.push(`attach_rpy ${fmtRpy(ac.attach_rpy)}≠${fmtRpy(bc.attach_rpy)}`)
    }
    if (fieldDiffs.length > 0) diffs.push(`${name}: ${fieldDiffs.join('; ')}`)
  }

  return { equal: diffs.length === 0, differences: diffs }
}
