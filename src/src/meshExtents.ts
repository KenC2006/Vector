// Runtime accessor for build-time measured mesh extents (Phase 1 of
//).
//
// scripts/measure-mesh-extents.mjs writes meshExtents.generated.json under
// src/public/. This module loads it on demand and exposes synchronous lookups
// so the resolver can populate collision.bounds and bbox-divergence warnings
// without touching the live scene.
//
// In environments without fetch (Node test runners, parity corpus), call
// setMeshExtentsCatalog() with a parsed catalog or leave it empty — every
// accessor degrades to "no measurement available".

export interface MeasuredMeshVisual {
  file: string
  rotation_rpy: [number, number, number]
  raw_extent_mm: [number, number, number]
  post_rotation_extent_mm: [number, number, number]
  vertex_count: number
  divergence_vs_bbox?: number
  warning?: string
  error?: string
}

export interface MeasuredMeshCollision {
  file: string
  extent_mm: [number, number, number]
  center_mm: [number, number, number]
  vertex_count: number
  divergence_vs_bbox?: number
  error?: string
}

export interface MeasuredMeshEntry {
  declared_bbox_mm?: [number, number, number]
  visual?: MeasuredMeshVisual
  collision?: MeasuredMeshCollision
}

export interface MeasuredMeshCatalog {
  generated_at?: string
  notes?: string
  components: Record<string, MeasuredMeshEntry>
}

let catalog: MeasuredMeshCatalog | null = null
let fetchPromise: Promise<MeasuredMeshCatalog | null> | null = null

export function setMeshExtentsCatalog(next: MeasuredMeshCatalog | null): void {
  catalog = next
}

export function getMeshExtentsCatalog(): MeasuredMeshCatalog | null {
  return catalog
}

export function getMeasuredMeshEntry(componentId: string): MeasuredMeshEntry | undefined {
  return catalog?.components?.[componentId]
}

export function getMeasuredCollisionExtentMm(componentId: string): [number, number, number] | undefined {
  const e = catalog?.components?.[componentId]?.collision?.extent_mm
  return e && e.length === 3 ? [e[0], e[1], e[2]] : undefined
}

export function getMeasuredCollisionCenterMm(componentId: string): [number, number, number] | undefined {
  const c = catalog?.components?.[componentId]?.collision?.center_mm
  return c && c.length === 3 ? [c[0], c[1], c[2]] : undefined
}

export function getMeasuredVisualExtentMm(componentId: string): [number, number, number] | undefined {
  const v = catalog?.components?.[componentId]?.visual?.post_rotation_extent_mm
  return v && v.length === 3 ? [v[0], v[1], v[2]] : undefined
}

/** Fire-and-forget loader for the browser path. The first caller triggers the
 * fetch; subsequent calls reuse the in-flight promise or the cached catalog.
 * Resolves to null when running in non-browser environments. */
export function ensureMeshExtentsLoaded(url = '/meshExtents.generated.json'): Promise<MeasuredMeshCatalog | null> {
  if (catalog) return Promise.resolve(catalog)
  if (fetchPromise) return fetchPromise
  if (typeof fetch !== 'function') return Promise.resolve(null)
  fetchPromise = fetch(url)
    .then(r => (r.ok ? r.json() : null))
    .then((data: MeasuredMeshCatalog | null) => {
      if (data && data.components) catalog = data
      return catalog
    })
    .catch(() => null)
    .finally(() => { fetchPromise = null })
  return fetchPromise
}
