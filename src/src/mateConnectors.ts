// Mate connectors — named local frames on presets + closed-form frame
// composition for "weld" / planar / concentric mates. Pure module: no DOM,
// no tauri, no richVisuals imports. Runs from Node via --experimental-strip-types
// for the mateCorpus fixture harness (mirrors the urdfGraphEquivalence /
// reconcileAlignment split the rest of the codebase uses).
//
// Architecture: Phase 1 of docs/MATE_CONNECTOR_MIGRATION.md — WS5's render-time
// reconciliation pass (reconcileAlignment.ts) closes mesh-vs-bbox depth gaps
// at the scene level but can't disambiguate face-identity or concentric shafts.
// Connectors move that ambiguity into preset data (named frames) and resolve
// pose by matrix composition, exactly the way Onshape/Drake/AIDL do it.
//
// Backward compatibility (load-bearing): every preset gets 6 auto-generated
// default connectors (top/bottom/front/back/left/right) whose math is
// mathematically equivalent to today's bbox half-extent path. Mating default
// connectors with `fastened` produces bit-identical child poses to
// computeFacePlacement() in urdfAssembly.ts — that's what makes the Phase 2
// feature flag safe to ship on by default.

import * as THREE from 'three'

export type ConnectorType = 'planar' | 'cylindrical' | 'point'
export type MateType = 'fastened' | 'planar' | 'concentric'

/** A named local frame on a preset. origin/axis are in the preset link's
 *  local coordinate frame, units mm (matches preset JSON throughout). */
export interface MateConnector {
  id: string
  origin_xyz_mm: [number, number, number]
  /** Primary axis direction (unit-length by convention; not re-normalised on
   *  read so authored values stay stable round-trip). */
  axis_xyz: [number, number, number]
  type: ConnectorType
  /** Only meaningful for cylindrical; used for port-compat diagnostics. */
  diameter_mm?: number
}

/** Preset-local bbox half-extents, in mm. Matches the `bounding_box_mm` field
 *  on presets divided by 2. */
export interface ConnectorBoundingBoxMm {
  hxMm: number
  hyMm: number
  hzMm: number
}

/** Runtime params for a mate. All fields optional — their defaults produce the
 *  "flush, no slide, no spin" result, which equals fastened. */
export interface MateParams {
  /** Axial slide along the mate axis (concentric). +mm moves the child
   *  connector along the parent connector's +axis direction. */
  offset_mm?: number
  /** Axial spin about the mate axis (planar, concentric), radians. */
  rotation_rad?: number
  /** In-plane translation for a planar mate. Basis vectors are picked
   *  canonically from the face normal so the same (u,v) yields the same
   *  displacement across runs (see `buildInPlaneBasis`). */
  offset_uv_mm?: [number, number]
}

/** A single authored mate — references two connectors by their globally
 *  qualified id (link_name.connector_id) and a type. */
export interface MateConstraint {
  parent_connector: string
  child_connector: string
  type: MateType
  offset_mm?: number
  rotation_rad?: number
  offset_uv_mm?: [number, number]
}

// ── Default connectors ─────────────────────────────────────────────────────

/** 6 face-center connectors derived from bbox half-extents. Ids match the
 *  legacy `attach_face` names in urdfAssembly.ts exactly, so on the backward-
 *  compat path `attach_face: "top"` is equivalent to
 *  `attach_connector: "top"` + `mate_connector: "bottom"` + `mate_type: "fastened"`.
 *  Axes point OUTWARD from the face (away from the link body). */
export function generateDefaultConnectors(bbox: ConnectorBoundingBoxMm): MateConnector[] {
  const { hxMm: hx, hyMm: hy, hzMm: hz } = bbox
  return [
    { id: 'top',    origin_xyz_mm: [0,  0,  hz], axis_xyz: [0,  0,  1], type: 'planar' },
    { id: 'bottom', origin_xyz_mm: [0,  0, -hz], axis_xyz: [0,  0, -1], type: 'planar' },
    { id: 'front',  origin_xyz_mm: [ hx, 0,  0], axis_xyz: [ 1, 0,  0], type: 'planar' },
    { id: 'back',   origin_xyz_mm: [-hx, 0,  0], axis_xyz: [-1, 0,  0], type: 'planar' },
    { id: 'right',  origin_xyz_mm: [0,  hy,  0], axis_xyz: [0,  1,  0], type: 'planar' },
    { id: 'left',   origin_xyz_mm: [0, -hy,  0], axis_xyz: [0, -1,  0], type: 'planar' },
  ]
}

/** Opposite-face convention. Used to infer the child connector id when the
 *  legacy `attach_face` field is the only thing authored — parent face "top"
 *  pairs with child face "bottom", etc. */
export const DEFAULT_OPPOSITE_FACE: Record<string, string> = {
  top: 'bottom', bottom: 'top',
  front: 'back', back: 'front',
  left: 'right', right: 'left',
}

/**
 * Merge authored connectors over defaults by id. Authored entries WIN —
 * that's the "override same-name default" contract from the migration doc.
 * Ids not present among defaults are appended.
 */
export function mergeConnectors(
  defaults: MateConnector[],
  authored: MateConnector[] | undefined,
): MateConnector[] {
  if (!authored || authored.length === 0) return defaults
  const byId = new Map<string, MateConnector>()
  for (const d of defaults) byId.set(d.id, d)
  for (const a of authored) byId.set(a.id, a)
  return Array.from(byId.values())
}

// ── Closed-form resolver ───────────────────────────────────────────────────

const _tmpVec = new THREE.Vector3()

function vec3(v: readonly [number, number, number]): THREE.Vector3 {
  return new THREE.Vector3(v[0], v[1], v[2])
}

/** Rotation that takes unit vector `a` onto unit vector `b`. Uses three.js's
 *  setFromUnitVectors, which already handles the antiparallel degenerate
 *  case (picks an arbitrary perpendicular axis for the 180° swing). */
function quatFromTo(a: THREE.Vector3, b: THREE.Vector3): THREE.Quaternion {
  const q = new THREE.Quaternion()
  q.setFromUnitVectors(_tmpVec.copy(a).normalize(), new THREE.Vector3().copy(b).normalize())
  return q
}

/**
 * Canonical orthonormal basis (u, v) spanning the plane perpendicular to
 * `normal`. Deterministic choice — important for fixtures and cache keys:
 *   - picks the world axis LEAST parallel to `normal` as the helper;
 *   - u = normalize(normal × helper);
 *   - v = normalize(normal × u).
 *
 * For axis-aligned normals this produces a stable {u,v} per axis.
 */
export function buildInPlaneBasis(normal: THREE.Vector3): { u: THREE.Vector3; v: THREE.Vector3 } {
  const n = new THREE.Vector3().copy(normal).normalize()
  const ax = Math.abs(n.x), ay = Math.abs(n.y), az = Math.abs(n.z)
  let helper: THREE.Vector3
  if (ax <= ay && ax <= az) helper = new THREE.Vector3(1, 0, 0)
  else if (ay <= az)        helper = new THREE.Vector3(0, 1, 0)
  else                      helper = new THREE.Vector3(0, 0, 1)
  const u = new THREE.Vector3().crossVectors(n, helper).normalize()
  const v = new THREE.Vector3().crossVectors(n, u).normalize()
  return { u, v }
}

/**
 * Closed-form frame composition. Given the parent link's world transform, the
 * two authored connectors, a mate type, and its params, return the child
 * link's world transform as a Matrix4.
 *
 * Convention (URDF-native, matches legacy bbox path):
 *   - axis_xyz is the OUTWARD direction. Two connectors mated with `fastened`
 *     have antiparallel axes in world space: parent's axis points +Z out of
 *     the parent face, child's axis points -Z out of the child face. Placing
 *     the child with identity rotation atop the parent makes those two axes
 *     antiparallel in world (parent +Z vs child -Z == world +Z vs -Z). ✓
 *   - For default face connectors, `resolveMate(fastened)` reproduces the
 *     current bbox math bit-for-bit. The parity is verified in mateCorpus.ts.
 *
 * Math:
 *   R_align  = quatFromTo(child.axis, -parent.axis)   // so child.axis ends up antiparallel to parent.axis in parent-local
 *   R_spin   = rotate(parent.axis, params.rotation_rad)
 *   R_local  = R_spin · R_align                        // child link orientation in parent-local
 *   t_local  = parent.origin + disp(mate, params) - R_local · child.origin
 *   childLocal = compose(t_local, R_local)
 *   childWorld = parentWorld · childLocal
 */
export function resolveMate(
  parentWorld: THREE.Matrix4,
  parentConnector: MateConnector,
  childConnector: MateConnector,
  mateType: MateType,
  params: MateParams = {},
): THREE.Matrix4 {
  const pOriginMm = vec3(parentConnector.origin_xyz_mm)
  const cOriginMm = vec3(childConnector.origin_xyz_mm)
  const pAxis = vec3(parentConnector.axis_xyz).normalize()
  const cAxis = vec3(childConnector.axis_xyz).normalize()

  // Preset JSON is mm; scene is m. Convert once, up-front.
  const pOrigin = pOriginMm.multiplyScalar(1 / 1000)
  const cOrigin = cOriginMm.multiplyScalar(1 / 1000)

  // Rotation: place child so its axis is antiparallel to parent's axis (in
  // parent-local frame — parentWorld is applied once at the end).
  const qAlign = quatFromTo(cAxis, new THREE.Vector3().copy(pAxis).negate())

  // Axial spin about the shared mate axis (parent's axis in parent-local).
  const rotRad = params.rotation_rad ?? 0
  const qSpin = new THREE.Quaternion().setFromAxisAngle(pAxis, rotRad)
  const qChildLocal = qSpin.multiply(qAlign)

  // Mate-type-specific displacement, in parent-local frame.
  // fastened  → 0 DOF, no displacement
  // concentric → axial slide along parent's axis (+mm = deeper along +axis)
  // planar     → in-plane (u,v) translation spanning parent face
  const displacement = new THREE.Vector3(0, 0, 0)
  if (mateType === 'concentric') {
    const offsetM = (params.offset_mm ?? 0) / 1000
    displacement.addScaledVector(pAxis, offsetM)
  } else if (mateType === 'planar') {
    const [uMm, vMm] = params.offset_uv_mm ?? [0, 0]
    const { u, v } = buildInPlaneBasis(pAxis)
    displacement.addScaledVector(u, uMm / 1000)
    displacement.addScaledVector(v, vMm / 1000)
  }

  const rChildLocal = new THREE.Matrix4().makeRotationFromQuaternion(qChildLocal)
  const cOriginRotated = cOrigin.clone().applyMatrix4(rChildLocal)
  const tChildLocal = pOrigin.clone().add(displacement).sub(cOriginRotated)

  const childLocal = new THREE.Matrix4().compose(
    tChildLocal,
    qChildLocal,
    new THREE.Vector3(1, 1, 1),
  )
  return new THREE.Matrix4().multiplyMatrices(parentWorld, childLocal)
}

/** Convenience: default-connector opposite-face pair. Callers that only know
 *  `attach_face` can compute the child's connector id without synthesising
 *  the full connector table. Returns null for non-default face names
 *  (e.g. "plate_top") so the caller can fail loudly rather than guess. */
export function childConnectorIdForAttachFace(attachFace: string): string | null {
  return DEFAULT_OPPOSITE_FACE[attachFace] ?? null
}

/** Find a connector by id in a flat list. Returns null if missing — callers
 *  MUST fail loudly (the migration doc explicitly rejects silent fallbacks). */
export function findConnector(
  connectors: MateConnector[],
  id: string,
): MateConnector | null {
  for (const c of connectors) if (c.id === id) return c
  return null
}
