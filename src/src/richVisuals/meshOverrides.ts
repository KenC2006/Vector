/**
 * Per-component mesh overrides for catalog parts.
 *
 * The data lives in visualOverrides.json (shared with the build scripts in
 * scripts/, which must never parse this file): which pre-converted GLB a part
 * renders with, the rotation that maps the CAD axes onto the catalog frame,
 * the scale policy, and optional shaft overlays. Every entry carries a "why".
 * Parts without a mesh override render through their procedural rich
 * generator (./generators).
 */
import visualOverrides from './visualOverrides.json' with { type: 'json' }

export type MeshVisualScalePolicy = 'none' | 'uniform' | 'per-axis'
export type MeshVisualUnits = 'm' | 'mm' | 'auto'

export interface MeshVisualMetadata {
  file: string
  rotation?: [number, number, number]
  units: MeshVisualUnits
  scalePolicy: MeshVisualScalePolicy
  targetFrame: 'urdf-z-up'
  shaftOverlay?: { shaft_length_mm: number; shaft_radius_mm: number }
}

interface VisualOverridesFile {
  meshOverrides: Record<string, string>
  proceduralVisualOnly: Record<string, { mesh: string; why: string }>
  rotationOverrides: Record<string, { rpy: number[]; why: string }>
  scalePolicy: Record<string, { policy: MeshVisualScalePolicy; why: string }>
  shaftOverlays: Record<string, { shaft_length_mm: number; shaft_radius_mm: number; why: string }>
}

const DATA = visualOverrides as unknown as VisualOverridesFile

/** Component id -> source CAD file name (the GLB shares its basename). */
export const MESH_OVERRIDES: Record<string, string> = DATA.meshOverrides

/**
 * Parts whose authored mesh was rejected (wrong part / wrong proportions) and
 * that render procedurally instead. They have no MESH_OVERRIDES entry, so
 * nothing is ever downloaded for them; the set exists for tooling and docs.
 */
export const PROCEDURAL_VISUAL_ONLY: Set<string> = new Set(Object.keys(DATA.proceduralVisualOnly))

/**
 * Per-component Euler rotations (XYZ order, radians) applied to the loaded
 * mesh BEFORE scaling, so the GLB's axes line up with the preset's
 * bounding_box_mm / connectors.
 */
export const ROTATION_OVERRIDES: Record<string, [number, number, number]> = Object.fromEntries(
  Object.entries(DATA.rotationOverrides).map(([id, e]) => [id, [e.rpy[0], e.rpy[1], e.rpy[2]] as [number, number, number]]),
)

/**
 * GLBs whose shape does not split body and output shaft: the body is scaled
 * to (bbox.z - shaft_length_mm) and a procedural shaft fills the top of the
 * bbox (see prepareMeshVisualGroup).
 */
export const SHAFT_OVERLAYS: Record<string, { shaft_length_mm: number; shaft_radius_mm: number }> = Object.fromEntries(
  Object.entries(DATA.shaftOverlays).map(([id, e]) => [id, { shaft_length_mm: e.shaft_length_mm, shaft_radius_mm: e.shaft_radius_mm }]),
)

/**
 * Meshes whose proportions already match the preset bbox (after rotation):
 * uniform scaling keeps the model honest at the cost of a small gap.
 * Everything else is fitted per axis.
 */
export const EXPLICIT_SCALE_POLICY: Record<string, MeshVisualScalePolicy> = Object.fromEntries(
  Object.entries(DATA.scalePolicy).map(([id, e]) => [id, e.policy]),
)

const MESH_VISUAL_METADATA: Record<string, MeshVisualMetadata> = Object.fromEntries(
  Object.entries(MESH_OVERRIDES).map(([componentId, file]) => [
    componentId,
    {
      file,
      rotation: ROTATION_OVERRIDES[componentId],
      units: 'auto',
      scalePolicy: EXPLICIT_SCALE_POLICY[componentId] ?? 'per-axis',
      targetFrame: 'urdf-z-up',
      shaftOverlay: SHAFT_OVERLAYS[componentId],
    } satisfies MeshVisualMetadata,
  ]),
)

export function getMeshVisualMetadata(componentId: string): MeshVisualMetadata | null {
  return MESH_VISUAL_METADATA[componentId] ?? null
}

/** URL of the pre-converted GLB for a component, or null if it has none. */
export function getMeshOverrideUrl(componentId: string): string | null {
  const metadata = getMeshVisualMetadata(componentId)
  if (!metadata) return null
  const baseName = metadata.file.replace(/\.(step|stp)$/i, '')
  return `/meshes/glb/${baseName}.glb`
}

/** Check if a component ID renders with a real mesh. */
export function hasMeshOverride(componentId: string): boolean {
  return !!getMeshVisualMetadata(componentId)
}

export function getShaftOverlay(componentId: string): { shaft_length_mm: number; shaft_radius_mm: number } | null {
  return getMeshVisualMetadata(componentId)?.shaftOverlay ?? null
}

/** Get the per-component rotation override (XYZ Euler radians), or null if none. */
export function getRotationOverride(componentId: string): [number, number, number] | null {
  return getMeshVisualMetadata(componentId)?.rotation ?? null
}

// ── Shadow-cast policy ──────────────────────────────────────────────────────
// End-effectors ship intricate geometry (gears, finger plates, pinion teeth,
// suction cups, finger linkages). Hard PCFShadowMap projects every detail onto
// the parallel face of the parent extrusion/bracket below them — the result
// reads as a duplicate mesh, not a shadow. Catch all `effector_*` presets so
// the policy stays consistent across catalog additions; effectors still
// RECEIVE shadows from the rest of the scene.
const NO_SHADOW_CAST_PREFIXES: readonly string[] = [
  'effector_',
]

/** True when a mesh of this catalog component should cast shadows. False for
 *  detail-rich end-effectors whose hard shadow on adjacent surfaces reads as
 *  a duplicate mesh. Takes a component id (resolve link names with
 *  componentIdForLink first); null (custom body) casts. */
export function shouldCastShadow(componentId: string | null): boolean {
  if (!componentId) return true
  return !NO_SHADOW_CAST_PREFIXES.some(prefix => componentId.startsWith(prefix))
}
