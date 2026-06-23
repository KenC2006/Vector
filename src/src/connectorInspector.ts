// Viewport debug overlay — renders every authored mate connector from
// generic_presets.json on every spawned link. Toggle via Shift+C.
//
// Exists to accelerate preset authoring: load a robot, press Shift+C, see
// whether the sphere lands on the real mesh surface. Mismatches are evidence
// of missing or wrong connector coords.
//
// Coordinate convention: authored connector `origin_xyz_mm` is in the
// URDF link frame — the same frame `resolveMate` in mateConnectors.ts
// divides by 1000 before composing with parent-local transforms. Parenting
// the marker to the linkGroup and dividing by 1000 here mirrors that
// exactly, so what the overlay shows is what the resolver would consume.

import * as THREE from 'three'
import type { MateConnector, ConnectorType } from './mateConnectors'

interface PresetComponentLite {
  id: string
  connectors?: MateConnector[]
}
interface PresetCategoryLite { components: PresetComponentLite[] }
interface PresetDataLite { categories: Record<string, PresetCategoryLite> }

interface ParsedRobotLike {
  linkGroups: Map<string, THREE.Group>
}

// Link names follow "{component_id}_{instance}" — same rule richVisuals
// uses. Anything that doesn't match (e.g. base_link) is skipped silently.
function extractComponentId(linkName: string): string | null {
  const m = linkName.match(/^(.+)_(\d+)$/)
  return m ? m[1] : null
}

let _presetCache: Map<string, PresetComponentLite> | null = null
let _presetsFetching: Promise<void> | null = null

function fetchPresets(): Promise<void> {
  if (_presetCache) return Promise.resolve()
  if (_presetsFetching) return _presetsFetching
  _presetsFetching = fetch('/generic_presets.json')
    .then(r => r.json() as Promise<PresetDataLite>)
    .then(data => {
      const map = new Map<string, PresetComponentLite>()
      for (const cat of Object.values(data.categories)) {
        for (const comp of cat.components) map.set(comp.id, comp)
      }
      _presetCache = map
    })
    .catch(err => {
      console.warn('[connectorInspector] failed to load presets:', err)
      _presetsFetching = null
    })
  return _presetsFetching
}

// Color scheme: id prefix wins over raw type so shaft_out / shaft_hole /
// mount_* / wall_* / plate_* get their documented color even though
// several share the same `type`. Falls through to type-based defaults.
const OVERLAY_COLOR_RULES: Array<{ match: (id: string, type: ConnectorType) => boolean; color: number }> = [
  { match: (id) => id === 'shaft_out', color: 0xff8800 },                          // orange
  { match: (id) => id === 'shaft_hole', color: 0x33cc66 },                         // green
  { match: (id) => id.startsWith('mount_'), color: 0x3388ff },                     // blue
  { match: (id) => id.startsWith('wall_'), color: 0xaa55ff },                      // purple
  { match: (id) => id.startsWith('plate_') || id === 'top_face', color: 0x22ccdd },// cyan
  { match: (id) => id === 'optical_front', color: 0xff4466 },                      // red
  { match: (_id, t) => t === 'cylindrical', color: 0xff8800 },
  { match: (_id, t) => t === 'planar', color: 0x3388ff },
  { match: (_id, t) => t === 'point', color: 0xff4466 },
]
const FALLBACK_COLOR = 0xcccccc

function colorFor(connector: MateConnector): number {
  for (const r of OVERLAY_COLOR_RULES) {
    if (r.match(connector.id, connector.type)) return r.color
  }
  return FALLBACK_COLOR
}

// Visual scale — small enough to read alongside 20-40mm servos without
// obscuring the mesh, large enough to see at typical framing distance.
const SPHERE_RADIUS_M = 0.002
const ARROW_LENGTH_M = 0.020
const ARROW_HEAD_LEN = 0.005
const ARROW_HEAD_WIDTH = 0.0035
const OVERLAY_TAG = '__connectorOverlay'

let _overlayEnabled = false

/** Flip overlay state. Returns the new state. Spawns the overlay for every
 *  link in `parsed` (or tears it down if toggling off). */
export function toggleConnectorOverlay(parsed: ParsedRobotLike): boolean {
  _overlayEnabled = !_overlayEnabled
  if (_overlayEnabled) rebuildConnectorOverlay(parsed)
  else clearConnectorOverlay(parsed)
  return _overlayEnabled
}

/** Rebuild the overlay after a URDF reparse (linkGroups were just replaced).
 *  No-op when the overlay is off — safe to call from every reparse path. */
export function refreshConnectorOverlay(parsed: ParsedRobotLike): void {
  if (!_overlayEnabled) return
  rebuildConnectorOverlay(parsed)
}

function disposeOverlayNode(node: THREE.Object3D): void {
  node.traverse(o => {
    const anyObj = o as any
    if (anyObj.geometry?.dispose) anyObj.geometry.dispose()
    const mat = anyObj.material
    if (Array.isArray(mat)) mat.forEach((m: THREE.Material) => m.dispose?.())
    else mat?.dispose?.()
  })
}

function clearConnectorOverlay(parsed: ParsedRobotLike): void {
  for (const linkGroup of parsed.linkGroups.values()) {
    const toRemove: THREE.Object3D[] = []
    for (const child of linkGroup.children) {
      if ((child.userData as Record<string, unknown>)[OVERLAY_TAG]) toRemove.push(child)
    }
    for (const c of toRemove) {
      disposeOverlayNode(c)
      linkGroup.remove(c)
    }
  }
}

function rebuildConnectorOverlay(parsed: ParsedRobotLike): void {
  clearConnectorOverlay(parsed)
  if (!_presetCache) {
    // First toggle: presets haven't loaded yet. Fetch then re-run; if the
    // user toggled off in the meantime, refreshConnectorOverlay short-circuits.
    fetchPresets().then(() => {
      if (_overlayEnabled) rebuildConnectorOverlay(parsed)
    })
    return
  }
  let authoredLinkCount = 0
  let authoredConnectorCount = 0
  for (const [linkName, linkGroup] of parsed.linkGroups) {
    const compId = extractComponentId(linkName)
    if (!compId) continue
    const preset = _presetCache.get(compId)
    if (!preset?.connectors?.length) continue
    const group = new THREE.Group()
    group.name = `__connector_overlay_${linkName}`
    ;(group.userData as Record<string, unknown>)[OVERLAY_TAG] = true
    for (const conn of preset.connectors) {
      group.add(buildConnectorMarker(conn))
    }
    linkGroup.add(group)
    authoredLinkCount++
    authoredConnectorCount += preset.connectors.length
  }
  console.log(
    `[connectorInspector] overlay ON — ${authoredConnectorCount} authored connectors on ${authoredLinkCount} links`,
  )
}

function buildConnectorMarker(conn: MateConnector): THREE.Object3D {
  const color = colorFor(conn)
  const group = new THREE.Group()
  group.name = `__connector_${conn.id}`
  const [ox, oy, oz] = conn.origin_xyz_mm
  group.position.set(ox / 1000, oy / 1000, oz / 1000)

  const sphereMat = new THREE.MeshBasicMaterial({
    color,
    depthTest: false,
    transparent: true,
    opacity: 0.95,
  })
  const sphere = new THREE.Mesh(new THREE.SphereGeometry(SPHERE_RADIUS_M, 12, 8), sphereMat)
  sphere.renderOrder = 999
  sphere.raycast = () => {}
  group.add(sphere)

  const axis = new THREE.Vector3(conn.axis_xyz[0], conn.axis_xyz[1], conn.axis_xyz[2])
  if (axis.lengthSq() > 1e-9) {
    axis.normalize()
    const arrow = new THREE.ArrowHelper(
      axis,
      new THREE.Vector3(0, 0, 0),
      ARROW_LENGTH_M,
      color,
      ARROW_HEAD_LEN,
      ARROW_HEAD_WIDTH,
    )
    // Render through-geometry so markers buried inside the mesh are still visible.
    arrow.line.material = new THREE.LineBasicMaterial({
      color,
      depthTest: false,
      transparent: true,
      opacity: 0.95,
    })
    const coneMat = arrow.cone.material as THREE.MeshBasicMaterial
    coneMat.depthTest = false
    coneMat.transparent = true
    coneMat.opacity = 0.95
    arrow.renderOrder = 999
    arrow.line.renderOrder = 999
    arrow.cone.renderOrder = 999
    arrow.line.raycast = () => {}
    arrow.cone.raycast = () => {}
    group.add(arrow)
  }

  return group
}
