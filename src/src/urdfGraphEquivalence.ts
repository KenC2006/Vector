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
   * the auto-computed joint rpy (placement + arm rest-pose). Used for Z-crouch quadruped poses etc. */
  attach_rpy?: number[]
  // ── Phase 1/2 mate-connector fields ──
  // Optional; when any are set, the engine routes through the connector
  // resolver (mateConnectors.ts) instead of the bbox half-extent path.
  // Omitted fields fall back to the legacy attach_face path — bit-identical
  // to the WS5 output so USE_MATE_CONNECTORS=on is safe to ship by default.
  /** Parent-side connector id. Defaults to the same face name as `attach_face`
   * when omitted (parent "top" face == parent "top" default connector). */
  attach_connector?: string
  /** Child-side connector id. When omitted but `attach_face` is set, the
   * opposite-face default is inferred (top ↔ bottom, etc.). */
  mate_connector?: string
  /** Mate type: 'fastened' / 'planar' / 'concentric'. Defaults to 'fastened'
   * when a connector is named but the type is omitted (matches attach_face
   * semantics — treat as a weld unless told otherwise). */
  mate_type?: string
  /** Set by the assembly engine when this child's placement was resolved via
   * the authored mate-connector path (computeMatePlacement OR the Layer-2
   * parentConnectors override in computeFacePlacement). reconcile uses this
   * to skip bbox-based correction — connector positions are authoritative,
   * bbox-derived deltas would stomp them. Runtime state, not authored. */
  placed_via_connector?: boolean
  // ── Tier-A novel-mode authoring fields ──
  // These are the small set of creative-authority levers that novel-mode
  // designs can use to break the deterministic placement template (radial
  // grids, computed splay angles). Stripped by `strip_forbidden_fields` in
  // standard mode so dog/arm/wheeled stay byte-identical.
  /** Position offset in MILLIMETERS [dx, dy, dz] applied AFTER face placement.
   * Clamped to ±50mm per axis. Use for asymmetric layouts (front pincer arms
   * longer than back walking legs, off-center electronics). Novel mode only. */
  placement_offset_mm?: number[]
  /** Override the auto-computed splay angle (degrees) for this child on a
   * bottom-face attach. Default splay is from `splayAngleForLegCount`.
   * Range -75 to +75 degrees. Novel mode only. */
  splay_angle_deg?: number
  /** Tier-B "complete control" — raw [x, y, z] joint-origin position relative
   * to parent, in METERS. When present, the placement compiler bypasses face/
   * mate placement and uses this verbatim as the URDF joint origin's xyz.
   * Stripped in standard mode (Phase-3 contract preserved for dog/arm/wheeled).
   * Novel-mode only. */
  xyz?: number[]
  /** Tier-B "complete control" — raw [roll, pitch, yaw] joint origin rotation
   * in RADIANS. When present, bypasses the auto-computed face/splay/servo-flip
   * orientation. Different from `attach_rpy` (which is the joint rest-pose
   * within a fixed mounting); `rpy` is the mounting orientation itself.
   * Stripped in standard mode. Novel-mode only. */
  rpy?: number[]
  /** Novel-mode primitive composition: an array of free-form
   * box/cylinder/sphere primitives (millimetres) that replace the preset's
   * rendered visuals. Lets the AI compose body shells (humanoid torso,
   * drone frame, tank hull, snake segment, sculpture, ...) without expanding
   * the preset catalog. Bounds/collision/connectors auto-derive from the
   * AABB of the primitives. Stripped in standard mode. See linkGeometry.ts
   * for the wire format. */
  link_geometry?: unknown[]
}

export interface AssemblyGraph {
  base_link: string
  ground_offset?: boolean
  components: AssemblyComponent[]
  /** Set by the Python AI pipeline when Claude declares the design archetype.
   * 'standard' = quadruped/arm/wheeled/biped/humanoid — full archetype scaffolding
   * applies. 'novel' = non-standard creature/topology; the placement compiler
   * relaxes archetype-shaped layouts (4-corner symmetric grid → radial), and the
   * archetype normalizer skips quadruped-template enforcement. Optional;
   * unset behaves like 'standard'. */
  _archetype_mode?: 'standard' | 'novel'
}

/** Deep clone an AssemblyGraph — use when you don't want callers to mutate the canonical copy. */
export function cloneAssemblyGraph(graph: AssemblyGraph): AssemblyGraph {
  return {
    base_link: graph.base_link,
    ground_offset: graph.ground_offset,
    _archetype_mode: graph._archetype_mode,
    components: graph.components.map(c => ({
      ...c,
      attach_rpy: c.attach_rpy ? [...c.attach_rpy] : undefined,
      placement_offset_mm: c.placement_offset_mm ? [...c.placement_offset_mm] : undefined,
      xyz: c.xyz ? [...c.xyz] : undefined,
      rpy: c.rpy ? [...c.rpy] : undefined,
      link_geometry: c.link_geometry ? c.link_geometry.map(p => ({ ...(p as object) })) : undefined,
    })),
  }
}

/** Result of a structural graph comparison. When `equal` is false, `differences` is
 * a human-readable list of fields/components that disagreed — not a machine-diff. */
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
 * Components are matched by `link_name` (order-independent). A mismatch on
 * base_link, ground_offset, component set, or any per-component field
 * (component_id, attach_to, attach_face, joint_*, length_mm, orientation,
 * elevation_angle, attach_rpy) is reported as a difference.
 *
 * Numeric fields use small tolerances; `attach_rpy` uses the same EPS the
 * runtime uses to decide whether to apply the override (so "absent" and
 * "all-zero" are considered equal — they produce identical placement). */
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
    if ((ac.attach_connector ?? null) !== (bc.attach_connector ?? null)) fieldDiffs.push(`attach_connector ${ac.attach_connector}≠${bc.attach_connector}`)
    if ((ac.mate_connector ?? null)   !== (bc.mate_connector ?? null))   fieldDiffs.push(`mate_connector ${ac.mate_connector}≠${bc.mate_connector}`)
    if ((ac.mate_type ?? null)        !== (bc.mate_type ?? null))        fieldDiffs.push(`mate_type ${ac.mate_type}≠${bc.mate_type}`)
    if (fieldDiffs.length > 0) diffs.push(`${name}: ${fieldDiffs.join('; ')}`)
  }

  return { equal: diffs.length === 0, differences: diffs }
}
