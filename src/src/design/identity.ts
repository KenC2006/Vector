/**
 * Which catalog part (if any) a URDF link is.
 *
 * Designer-built URDFs carry an explicit map in a `<!-- vector:parts {...} -->`
 * comment: link name -> { part, component, mirror_of?, joint? }. That map is
 * the identity. URDFs without it (hand-written, or from older Vector builds)
 * fall back to the `<component_id>_<N>` link-name convention, accepted only
 * when the id is a real catalog part.
 *
 * Every consumer (rich visuals, collision view, inspector, connector overlay)
 * must go through here instead of parsing link names itself.
 */

export interface PartMapEntry {
  part: string
  component: string | null
  mirror_of?: string
  joint?: string
}

export type PartMap = Record<string, PartMapEntry>

const PARTS_RE = /<!-- vector:parts (.*?) -->/s
const INSTANCE_RE = /^(.+?)_(\d+)$/

let currentMap: PartMap | null = null
let catalogIds: Set<string> | null = null

/** Parse the explicit map from URDF text (null when absent or malformed). */
export function parsePartMap(urdf: string): PartMap | null {
  const m = PARTS_RE.exec(urdf || '')
  if (!m) return null
  try {
    const v = JSON.parse(m[1])
    return v && typeof v === 'object' ? (v as PartMap) : null
  } catch {
    return null
  }
}

/** Call whenever the URDF being displayed changes. */
export function setIdentitySource(urdf: string): void {
  currentMap = parsePartMap(urdf)
}

/** Register catalog ids so the name-convention fallback only matches real parts. */
export function setCatalogIds(ids: Iterable<string>): void {
  catalogIds = new Set(ids)
}

export function getPartMap(): PartMap | null {
  return currentMap
}

/** Catalog component id of a link, or null for custom bodies / unknown links. */
export function componentIdForLink(link: string): string | null {
  if (currentMap) return currentMap[link]?.component ?? null
  const m = INSTANCE_RE.exec(link)
  if (!m) return null
  // Without the catalog loaded yet, trust the convention (the renderer
  // re-runs once presets arrive).
  if (catalogIds && !catalogIds.has(m[1])) return null
  return m[1]
}

/** Design part name for a link (the link name itself when there is no map). */
export function partForLink(link: string): string {
  return currentMap?.[link]?.part ?? link
}

/** Link name of a design part (inverse of partForLink). */
export function linkForPart(part: string): string | null {
  if (!currentMap) return part
  for (const [link, e] of Object.entries(currentMap)) if (e.part === part) return link
  return null
}
