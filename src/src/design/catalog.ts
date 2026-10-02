/**
 * The component catalog (public/generic_presets.json), loaded once.
 */
import { setCatalogIds } from './identity.ts'
import type { Vec3 } from './types'

export interface CatalogConnector {
  id: string
  origin_xyz_mm: Vec3
  axis_xyz: Vec3
  type?: string
  cls?: string
  diameter_mm?: number
  single?: boolean
}

export interface CatalogComponent {
  id: string
  name: string
  description: string
  physical: {
    mass_kg?: number
    mass_kg_per_100mm?: number
    bounding_box_mm?: number[]
    bbox_mm?: number[]
    cross_section_mm?: number[]
    parametric?: { axis?: 'x' | 'y' | 'z'; cross_section_mm?: number[] }
    length_mm?: number
    inertia_primitive?: string
    [k: string]: unknown
  }
  mechanical_electrical: Record<string, unknown>
  mounting_logic?: Record<string, unknown>
  sim_metadata?: Record<string, unknown>
  connectors?: CatalogConnector[]
  [k: string]: unknown
}

export interface CatalogData {
  categories: Record<string, { description: string; components: CatalogComponent[] }>
}

let data: CatalogData | null = null
const byId = new Map<string, CatalogComponent>()
const categoryById = new Map<string, string>()
let loading: Promise<CatalogData> | null = null

export function loadCatalog(): Promise<CatalogData> {
  if (!loading) {
    loading = fetch('/generic_presets.json')
      .then(r => (r.ok ? r.json() : Promise.reject(new Error(`catalog HTTP ${r.status}`))))
      .then((d: CatalogData) => {
        data = d
        for (const [cat, c] of Object.entries(d.categories)) {
          for (const comp of c.components) {
            byId.set(comp.id, comp)
            categoryById.set(comp.id, cat)
          }
        }
        setCatalogIds(byId.keys())
        return d
      })
  }
  return loading
}

export function catalog(): CatalogData | null {
  return data
}

export function getComponent(id: string | null | undefined): CatalogComponent | null {
  return id ? byId.get(id) ?? null : null
}

export function categoryOf(id: string): string {
  return categoryById.get(id) ?? 'structural'
}

function tuple(v: unknown, n: number): v is number[] {
  return Array.isArray(v) && v.length === n && v.every(x => typeof x === 'number' && Number.isFinite(x))
}

/** Cut-to-length part (needs `length_mm`). Mirrors core/presets is_parametric_spec. */
export function isParametric(comp: CatalogComponent): boolean {
  const ph = comp.physical ?? {}
  const par = ph.parametric
  if (par && (par.axis === 'x' || par.axis === 'y' || par.axis === 'z') && tuple(par.cross_section_mm, 2)) return true
  return tuple(ph.cross_section_mm, 2)
}

/** Full outer envelope in mm. Mirrors core/presets resolve_component_bounds_mm. */
export function componentSizeMm(comp: CatalogComponent, lengthMm?: number): Vec3 {
  const ph = comp.physical ?? {}
  if (tuple(ph.bbox_mm, 3)) return [...ph.bbox_mm] as Vec3
  if (tuple(ph.bounding_box_mm, 3)) return [...ph.bounding_box_mm] as Vec3
  const len = lengthMm && lengthMm > 0 ? lengthMm : undefined
  const par = ph.parametric
  if (par && (par.axis === 'x' || par.axis === 'y' || par.axis === 'z') && tuple(par.cross_section_mm, 2)) {
    const c = par.cross_section_mm
    const L = len ?? ph.length_mm ?? 100
    if (par.axis === 'x') return [L, c[0], c[1]]
    if (par.axis === 'y') return [c[0], L, c[1]]
    return [c[0], c[1], L]
  }
  if (tuple(ph.cross_section_mm, 3)) return [...ph.cross_section_mm] as Vec3
  if (tuple(ph.cross_section_mm, 2)) return [ph.cross_section_mm[0], ph.cross_section_mm[1], len ?? 40]
  return [40, 40, 40]
}

/** Preset bbox for renderers; null for cut-to-length parts (their size comes
 *  from the instance's URDF geometry). */
export function presetBoundingBoxMm(id: string): Vec3 | null {
  const c = getComponent(id)
  if (!c || isParametric(c)) return null
  return componentSizeMm(c)
}
