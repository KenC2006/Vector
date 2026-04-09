/**
 * PBR Material cache for rich component visuals.
 *
 * 8 canonical material types with lazy caching and category-tint support.
 * All materials are MeshStandardMaterial so the existing highlight system
 * (emissive.setHex) continues to work.
 */
import * as THREE from 'three'

// ── Material definitions ─────────────────────────────────────────────────────

export interface MaterialDef {
  roughness: number
  metalness: number
  baseColor: number  // default color (can be overridden by tint)
}

export const MATERIAL_DEFS: Record<string, MaterialDef> = {
  anodized_aluminum: { roughness: 0.35, metalness: 0.85, baseColor: 0x8899aa },
  brushed_steel:     { roughness: 0.45, metalness: 0.90, baseColor: 0x888899 },
  matte_plastic:     { roughness: 0.70, metalness: 0.05, baseColor: 0x222222 },
  glossy_plastic:    { roughness: 0.25, metalness: 0.08, baseColor: 0x444444 },
  pcb_green:         { roughness: 0.60, metalness: 0.10, baseColor: 0x1a5c1a },
  rubber_black:      { roughness: 0.95, metalness: 0.02, baseColor: 0x111111 },
  copper_trace:      { roughness: 0.40, metalness: 0.75, baseColor: 0xb87333 },
  dark_chrome:       { roughness: 0.20, metalness: 0.95, baseColor: 0x333344 },
}

// ── Cache ────────────────────────────────────────────────────────────────────

const _cache = new Map<string, THREE.MeshStandardMaterial>()

function cacheKey(materialId: string, color?: number): string {
  return color != null ? `${materialId}_${color.toString(16)}` : materialId
}

/**
 * Get or create a PBR material.
 * @param id - one of the MATERIAL_DEFS keys
 * @param tintColor - optional color override (hex number like 0xff0000)
 */
export function getMaterial(id: string, tintColor?: number): THREE.MeshStandardMaterial {
  const key = cacheKey(id, tintColor)
  let mat = _cache.get(key)
  if (mat) return mat

  const def = MATERIAL_DEFS[id]
  if (!def) {
    // fallback
    mat = new THREE.MeshStandardMaterial({ color: tintColor ?? 0x888888, roughness: 0.5, metalness: 0.3 })
    _cache.set(key, mat)
    return mat
  }

  mat = new THREE.MeshStandardMaterial({
    color: tintColor ?? def.baseColor,
    roughness: def.roughness,
    metalness: def.metalness,
  })
  _cache.set(key, mat)
  return mat
}

/**
 * Get a tinted variant of a material — blends the tint with the base color.
 * Useful for category-coloring metal parts (e.g., orange-anodized servo body).
 */
export function getTintedMaterial(id: string, tintR: number, tintG: number, tintB: number, strength = 0.4): THREE.MeshStandardMaterial {
  const def = MATERIAL_DEFS[id]
  if (!def) return getMaterial(id)

  const base = new THREE.Color(def.baseColor)
  const tint = new THREE.Color(tintR, tintG, tintB)
  const blended = base.lerp(tint, strength)
  const hex = blended.getHex()
  return getMaterial(id, hex)
}

/**
 * Dispose all cached materials (call on app shutdown or full scene reset).
 */
export function disposeAllMaterials(): void {
  for (const mat of _cache.values()) {
    mat.dispose()
  }
  _cache.clear()
}
