/**
 * Manual robot editing on the design model.
 *
 * Everything the user does here — placing a catalog part, moving or rotating
 * one with the gizmo, editing a field in the inspector, deleting — is an edit
 * of the design (core/designer), recompiled to URDF by the same compiler the
 * AI designer uses. There is no second placement engine: the placement
 * preview runs the compiler's own mate math (design/math.ts), so a part lands
 * exactly where its ghost showed it, and the URDF always says what the design
 * says.
 *
 * Coordinates: design parts live in the design world (mm, Z up). The viewer's
 * `urdf_world` group is the URDF world = the root link frame, in metres. All
 * overlay objects (placement ghost, anchor markers, gizmo proxy) are children
 * of `urdf_world`, so they share the robot's grounding and view transform.
 */
import * as THREE from 'three'
import { TransformControls } from 'three/examples/jsm/controls/TransformControls.js'
import type { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js'
import { STLExporter } from 'three/examples/jsm/exporters/STLExporter.js'
import { invoke } from '@tauri-apps/api/core'
import { resolveComponentVisual } from './componentVisualResolver'
import { componentVisualWorldQuat, preloadComponentMesh } from './richVisuals'
import { CATEGORY_COLORS } from './componentMeshes'
import {
  catalog, categoryOf, componentSizeMm, getComponent, isParametric, loadCatalog,
  type CatalogComponent,
} from './design/catalog'
import {
  anchorLocal, axesOf, axisAngle, col, cross, dot, isFlushMate, mulM, mulV, norm, partAnchors,
  poseForSpec, resolveAt, scale, sub, transpose, worldPoint,
  type Anchor, type PartGeom,
} from './design/math'
import {
  addPart, anchorOccupancy, atParts, dependents, findPart, jointOf, mirrorName, orderParts,
  removePart, renamePart, replacePart, sourcePartName, translateAt, uniqueName,
} from './design/edits'
import {
  adoptCompiled, compileDesign, designToViewerDir, designToViewerPoint, invalidateSession,
  openSession, viewerToDesignDir, viewerToDesignPoint, type DesignSession,
} from './design/session'
import type { Design, DesignPart, JointSpec, JointType, Mat3, Vec3 } from './design/types'

export interface ParsedRobotLike {
  group: THREE.Group
  linkGroups: Map<string, THREE.Group>
  joints: Map<string, { group: THREE.Group }>
}

export interface AssemblyEditorContext {
  scene: THREE.Scene
  camera: THREE.PerspectiveCamera
  canvas: HTMLCanvasElement
  controls: OrbitControls
  showToast: (message: string, type?: 'success' | 'warning' | 'error' | 'info') => void
  switchPanel: (name: string) => void
  getUrdfText: () => string
  setUrdfText: (content: string) => void
  reparseUrdf: (xmlOverride?: string, opts?: { skipGround?: boolean; ground?: boolean }) => void
  getParsedRobot: () => ParsedRobotLike
  getKinematicGraph: () => Record<string, { name: string; mass: number; parent?: string; children: string[] }>
  isViewport3D: () => boolean
  getInteractionMode: () => 'build' | 'inspect'
  isSimActive?: () => boolean
  onInspectLinkFocused: (linkName: string | null) => void
  onAfterModelUpdated?: () => void
  zeroAssemblyWorldPosition?: () => void
  groundAssembly?: () => void
}

export interface AssemblyEditorApi {
  onModelUpdated(): void
  recordUndoExternal(content: string): void
  exitCarryMode(): void
  onInteractionModeChanged(mode: 'build' | 'inspect'): void
  setSelectedLink(linkName: string | null): void
  getUndoState(): { undo: string[]; redo: string[] }
  restoreUndoState(state: { undo: string[]; redo: string[] }): void
  /** Re-place overlay objects after the robot group moved (grounding). */
  refreshOverlay(): void
}

const CATEGORY_ICONS: Record<string, string> = {
  actuators: `<svg viewBox="0 0 14 14" fill="none" stroke="currentColor" stroke-width="1.2"><circle cx="7" cy="7" r="2.2"/><path d="M7 1v1.5M7 11.5V13M1 7h1.5M11.5 7H13M2.93 2.93l1.06 1.06M10.01 10.01l1.06 1.06M2.93 11.07l1.06-1.06M10.01 3.99l1.06-1.06" stroke-linecap="round"/></svg>`,
  motors: `<svg viewBox="0 0 14 14" fill="none" stroke="currentColor" stroke-width="1.2"><circle cx="7" cy="7" r="4.5"/><circle cx="7" cy="7" r="1.5"/><path d="M7 2.5v1.8M7 9.7v1.8M2.5 7h1.8M9.7 7h1.8" stroke-linecap="round"/></svg>`,
  sensors: `<svg viewBox="0 0 14 14" fill="none" stroke="currentColor" stroke-width="1.2"><ellipse cx="7" cy="7" rx="5" ry="3.5"/><circle cx="7" cy="7" r="1.5"/><path d="M2.5 5C3.5 2.5 10.5 2.5 11.5 5" stroke-linecap="round"/></svg>`,
  compute: `<svg viewBox="0 0 14 14" fill="none" stroke="currentColor" stroke-width="1.2"><rect x="3" y="3" width="8" height="8" rx="1"/><path d="M5 1v2M9 1v2M5 11v2M9 11v2M1 5h2M1 9h2M11 5h2M11 9h2" stroke-linecap="round"/></svg>`,
  power: `<svg viewBox="0 0 14 14" fill="none" stroke="currentColor" stroke-width="1.2"><path d="M8.5 1.5L5 7.5h4L5 12.5" stroke-linecap="round" stroke-linejoin="round"/></svg>`,
  structural: `<svg viewBox="0 0 14 14" fill="none" stroke="currentColor" stroke-width="1.2"><rect x="1.5" y="4" width="11" height="6" rx="0.5"/><line x1="1.5" y1="6.5" x2="12.5" y2="6.5"/><line x1="1.5" y1="7.5" x2="12.5" y2="7.5"/></svg>`,
  transmission: `<svg viewBox="0 0 14 14" fill="none" stroke="currentColor" stroke-width="1.2"><circle cx="4.5" cy="7" r="2.5"/><circle cx="9.5" cy="7" r="2"/><line x1="7" y1="7" x2="7.5" y2="7" stroke-width="1.5"/></svg>`,
  end_effectors: `<svg viewBox="0 0 14 14" fill="none" stroke="currentColor" stroke-width="1.2"><path d="M4 13V7.5L2 5V3h1.5l1.5 2.5L7 7M10 13V7.5l2-2.5V3H10.5L9 5.5 7 7" stroke-linecap="round" stroke-linejoin="round"/><line x1="7" y1="7" x2="7" y2="13" stroke-linecap="round"/></svg>`,
  mobility: `<svg viewBox="0 0 14 14" fill="none" stroke="currentColor" stroke-width="1.2"><circle cx="7" cy="7" r="5"/><circle cx="7" cy="7" r="1.5"/><path d="M7 2v2M7 10v2M2 7h2M10 7h2" stroke-linecap="round"/></svg>`,
  drivetrain: `<svg viewBox="0 0 14 14" fill="none" stroke="currentColor" stroke-width="1.2"><circle cx="4" cy="9" r="2.5"/><circle cx="10" cy="9" r="2.5"/><path d="M4 9h6M7 3v6" stroke-linecap="round"/></svg>`,
}

const DIRECTIONS = ['+x', '-x', '+y', '-y', '+z', '-z'] as const
const SHAFT_LIKE = (a: Anchor | undefined) => !!a && (a.cls === 'shaft' || /^(shaft_out|rod_out)$/.test(a.key))
const BORE_LIKE = /^(hub_bore|inner_bore|shaft_hole|shaft_in|horn|mount_hub)/
const SNAP_PX = 22

const esc = (s: string) => s.replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;').replace(/"/g, '&quot;')
const fmtNum = (v: number, d = 1) => String(Math.round(v * 10 ** d) / 10 ** d)
const round3 = (v: Vec3): Vec3 => v.map(x => Math.round(x * 1000) / 1000 || 0) as Vec3
/** Degrees normalised to (-180, 180]. */
const normDeg = (d: number): number => { const r = ((d % 360) + 360) % 360; return r > 180 ? r - 360 : r }

function matrixFrom(R: Mat3, pMetres: Vec3): THREE.Matrix4 {
  return new THREE.Matrix4().set(
    R[0][0], R[0][1], R[0][2], pMetres[0],
    R[1][0], R[1][1], R[1][2], pMetres[1],
    R[2][0], R[2][1], R[2][2], pMetres[2],
    0, 0, 0, 1,
  )
}

function mat3FromQuat(q: THREE.Quaternion): Mat3 {
  const m = new THREE.Matrix4().makeRotationFromQuaternion(q).elements
  return [[m[0], m[4], m[8]], [m[1], m[5], m[9]], [m[2], m[6], m[10]]]
}

function shortName(componentId: string): string {
  return componentId.replace(/^(actuator|motor|sensor|compute|power|structural|transmission|effector|mobility|drivetrain)_/, '')
}

function defaultJoint(target: Anchor | undefined, comp: CatalogComponent): JointType {
  if (!SHAFT_LIKE(target)) return 'fixed'
  if (target!.key === 'rod_out') return 'prismatic'
  const contact = String((comp.sim_metadata ?? {}).contact_class ?? '')
  return contact === 'wheel' || contact === 'track' || /wheel|tire|tread/.test(comp.id) ? 'continuous' : 'revolute'
}

/** Anchors of the part being placed, most likely mating face first. */
function alignCandidates(comp: CatalogComponent, targetAnchor: Anchor | undefined): string[] {
  const size = componentSizeMm(comp)
  const anchors = partAnchors({ spec: { name: 'x', component: comp.id }, component: comp, size, centerLocal: [0, 0, 0] })
  const keys = anchors.map(a => a.key)
  const ordered: string[] = []
  const push = (k: string | undefined) => { if (k && keys.includes(k) && !ordered.includes(k)) ordered.push(k) }
  if (SHAFT_LIKE(targetAnchor)) for (const k of keys) if (BORE_LIKE.test(k)) push(k)
  for (const k of ['mount_back', 'mount_top', 'mount_bottom', 'bottom']) push(k)
  for (const a of anchors) if (a.kind === 'connector') push(a.key)
  for (const k of ['-z', '+z', '-x', '+x', '-y', '+y']) push(k)
  return ordered
}

export function initAssemblyEditor(ctx: AssemblyEditorContext): AssemblyEditorApi {
  let session: DesignSession | null = null
  let selected: string | null = null          // compiled part name (may be a mirrored copy)
  let mode: 'build' | 'inspect' = 'build'
  let undo: string[] = []
  let redo: string[] = []
  let lastIssues: string[] = []
  let convertedNoticeShown = false

  const overlay = new THREE.Group()
  overlay.name = 'design_overlay'
  function attachOverlay(): THREE.Object3D | null {
    const world = ctx.scene.getObjectByName('urdf_world')
    if (world && overlay.parent !== world) world.add(overlay)
    return world ?? null
  }

  const gizmo = new TransformControls(ctx.camera, ctx.canvas)
  gizmo.setSpace('world')
  gizmo.setSize(0.8)
  ctx.scene.add(gizmo.getHelper())
  const proxy = new THREE.Object3D()
  proxy.name = 'design_gizmo_proxy'
  overlay.add(proxy)

  const markers = new THREE.Group()
  markers.name = 'design_anchor_markers'
  overlay.add(markers)

  // ── DOM ─────────────────────────────────────────────────────────────────
  const $ = <T extends HTMLElement>(id: string) => document.getElementById(id) as T | null
  const buildTree = $<HTMLDivElement>('build-tree')
  const buildEmpty = $<HTMLDivElement>('build-empty')
  const bsParts = $<HTMLSpanElement>('bs-parts')
  const bsMass = $<HTMLSpanElement>('bs-mass')
  const bsJoints = $<HTMLSpanElement>('bs-joints')
  const inspTitle = $<HTMLSpanElement>('insp-title')
  const inspBody = document.querySelector('#panel-inspector .insp-body') as HTMLDivElement | null
  const compItems = $<HTMLDivElement>('comp-items')
  const compDetail = $<HTMLDivElement>('comp-detail')
  const toolboxSearch = $<HTMLInputElement>('toolbox-search')
  const parentName = $<HTMLSpanElement>('tb-parent-name')
  const parentIndicator = $<HTMLDivElement>('tb-parent-indicator')
  const hud = document.createElement('div')
  hud.id = 'carry-hud'
  hud.style.cssText = `position:absolute; bottom:48px; left:50%; transform:translateX(-50%);
    background:rgba(0,0,0,0.72); color:#d4d4d4; font-size:12px; padding:6px 14px; border-radius:6px;
    pointer-events:none; display:none; white-space:nowrap; z-index:100; border:1px solid rgba(255,255,255,0.12);`
  ctx.canvas.parentElement?.appendChild(hud)
  const setHud = (msg: string) => { hud.style.display = msg ? 'block' : 'none'; hud.textContent = msg }

  // ── session ─────────────────────────────────────────────────────────────
  async function ensureSession(): Promise<DesignSession | null> {
    const text = ctx.getUrdfText()
    if (session && session.urdf === text) return session
    try {
      session = await openSession(text)
      return session.urdf === ctx.getUrdfText() ? session : null
    } catch (e) {
      session = null
      console.warn('[editor] design unavailable:', e)
      return null
    }
  }

  function current(): DesignSession | null {
    return session && session.urdf === ctx.getUrdfText() ? session : null
  }

  function recordUndo(text: string) {
    undo.push(text)
    if (undo.length > 80) undo.shift()
    redo = []
  }

  /** Compile a design and make it the robot. */
  async function commit(design: Design, message?: string): Promise<boolean> {
    let ordered: Design
    try {
      ordered = orderParts(design)
    } catch (e) {
      ctx.showToast(String((e as Error).message ?? e), 'error')
      return false
    }
    const res = await compileDesign(ordered, true)
    if ('error' in res) {
      ctx.showToast(res.error.split('\n')[0], 'error')
      return false
    }
    const wasImported = current()?.imported ?? false
    recordUndo(ctx.getUrdfText())
    session = adoptCompiled(ordered, res)
    lastIssues = res.issues
    ctx.setUrdfText(res.urdf)
    ctx.reparseUrdf(res.urdf)
    if (wasImported && !convertedNoticeShown) {
      convertedNoticeShown = true
      ctx.showToast('This URDF is now tracked as an editable design (poses kept exactly).', 'info')
    }
    if (res.issues.length) ctx.showToast(`${message ? message + ' — ' : ''}${res.issues.length} geometry note(s): ${res.issues[0]}`, 'warning')
    else if (message) ctx.showToast(message, 'success')
    return true
  }

  // ── viewer helpers ──────────────────────────────────────────────────────
  function linkOfObject(o: THREE.Object3D | null): string | null {
    for (let cur = o; cur; cur = cur.parent) {
      const n = (cur.userData as Record<string, unknown>).urdfLinkName
      if (typeof n === 'string' && n) return n
    }
    return null
  }

  function pickTargets(): THREE.Object3D[] {
    const out: THREE.Object3D[] = []
    ctx.getParsedRobot().group.traverse(o => {
      if ((o as THREE.Mesh).isMesh && linkOfObject(o) && !(o.userData as Record<string, unknown>).isCollision) out.push(o)
    })
    return out
  }

  function raycaster(e: { clientX: number; clientY: number }): THREE.Raycaster {
    const r = ctx.canvas.getBoundingClientRect()
    const ray = new THREE.Raycaster()
    ray.setFromCamera(new THREE.Vector2(((e.clientX - r.left) / r.width) * 2 - 1, -((e.clientY - r.top) / r.height) * 2 + 1), ctx.camera)
    return ray
  }

  /** Design-world point -> screen px. */
  function toScreen(s: DesignSession, p: Vec3): THREE.Vector2 {
    const world = attachOverlay()
    const v = new THREE.Vector3(...designToViewerPoint(s, p))
    if (world) v.applyMatrix4(world.matrixWorld)
    v.project(ctx.camera)
    const r = ctx.canvas.getBoundingClientRect()
    return new THREE.Vector2((v.x + 1) / 2 * r.width, (1 - v.y) / 2 * r.height)
  }

  function viewerPointFromWorld(p: THREE.Vector3): Vec3 {
    const world = attachOverlay()
    const v = world ? world.worldToLocal(p.clone()) : p.clone()
    return [v.x, v.y, v.z]
  }

  function viewerDirFromWorld(d: THREE.Vector3): Vec3 {
    const world = attachOverlay()
    const q = new THREE.Quaternion()
    world?.getWorldQuaternion(q)
    const v = d.clone().applyQuaternion(q.invert()).normalize()
    return [v.x, v.y, v.z]
  }

  // ── part visuals for the placement ghost ────────────────────────────────
  const ghostMat = new THREE.MeshStandardMaterial({ color: 0x33ff99, transparent: true, opacity: 0.35, depthWrite: false })
  const ghostEdge = new THREE.LineBasicMaterial({ color: 0x33ff99, transparent: true, opacity: 0.8 })

  function partVisual(comp: CatalogComponent, sizeMm: Vec3): THREE.Group {
    const resolved = resolveComponentVisual({
      preset: { id: comp.id, physical: { bounding_box_mm: [...sizeMm] }, mechanical_electrical: {} },
      category: categoryOf(comp.id),
    })
    const g = new THREE.Group()
    let body: THREE.Object3D
    if (resolved.previewGroup) {
      body = resolved.previewGroup.clone(true)
      body.quaternion.copy(componentVisualWorldQuat(resolved.authoredFrame, 'urdf_z_up'))
    } else {
      body = new THREE.Mesh(new THREE.BoxGeometry(sizeMm[0] / 1000, sizeMm[1] / 1000, sizeMm[2] / 1000))
    }
    const meshes: THREE.Mesh[] = []
    body.traverse(o => { if ((o as THREE.Mesh).isMesh) meshes.push(o as THREE.Mesh) })
    for (const m of meshes) {
      m.material = ghostMat
      m.castShadow = false
      const edges = new THREE.LineSegments(new THREE.EdgesGeometry(m.geometry, 30), ghostEdge)
      m.add(edges)
    }
    g.add(body)
    return g
  }

  // ── anchor markers ──────────────────────────────────────────────────────
  const markerGeo = new THREE.SphereGeometry(0.0045, 12, 8)
  const coneGeo = new THREE.ConeGeometry(0.003, 0.012, 10)
  coneGeo.translate(0, 0.006 + 0.0045, 0)
  const MARK = {
    free: new THREE.MeshBasicMaterial({ color: 0x3fa9ff, depthTest: false, transparent: true, opacity: 0.95 }),
    used: new THREE.MeshBasicMaterial({ color: 0xff7a45, depthTest: false, transparent: true, opacity: 0.95 }),
    face: new THREE.MeshBasicMaterial({ color: 0x8a8f98, depthTest: false, transparent: true, opacity: 0.7 }),
    hot: new THREE.MeshBasicMaterial({ color: 0x2dff8a, depthTest: false }),
  }

  function clearMarkers() {
    while (markers.children.length) markers.remove(markers.children[0])
  }

  function addMarker(s: DesignSession, g: PartGeom, a: Anchor, mat: THREE.Material) {
    const p = designToViewerPoint(s, worldPoint(g, a.p))
    const m = new THREE.Group()
    m.position.set(...p)
    const dot = new THREE.Mesh(markerGeo, mat)
    dot.renderOrder = 999
    m.add(dot)
    if (a.n) {
      const n = designToViewerDir(s, mulV(g.R, a.n))
      const cone = new THREE.Mesh(coneGeo, mat)
      cone.renderOrder = 999
      cone.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), new THREE.Vector3(...n))
      m.add(cone)
    }
    markers.add(m)
  }

  /** Markers for one part's anchors: connectors always, faces optionally. */
  function showAnchors(s: DesignSession, part: string, opts: { faces: boolean; hot?: string } = { faces: false }) {
    const g = s.geoms.get(part)
    if (!g) return
    const usedSet = usedAnchors(s)
    for (const a of partAnchors(g)) {
      if (a.kind === 'primitive') continue
      if (a.kind === 'face' && !opts.faces && a.key !== opts.hot) continue
      const used = usedSet.has(`${part}.${a.key}`)
      const mat = a.key === opts.hot ? MARK.hot : a.kind === 'face' ? MARK.face : used ? MARK.used : MARK.free
      addMarker(s, g, a, mat)
    }
  }

  // ── selection / gizmo ───────────────────────────────────────────────────
  interface DragState {
    part: string
    pivot: THREE.Object3D
    pivotWorld0: THREE.Matrix4
    proxyWorld0: THREE.Matrix4
    proxyPos0: THREE.Vector3
    proxyQuat0: THREE.Quaternion
    /** Flush-mated parts slide along the face they're mounted on (viewer-frame normal). */
    slideNormal: THREE.Vector3 | null
  }
  let drag: DragState | null = null

  function pivotOfLink(link: string): THREE.Object3D | null {
    const lg = ctx.getParsedRobot().linkGroups.get(link)
    const pivot = lg?.parent
    return pivot && pivot !== ctx.getParsedRobot().group ? pivot : null
  }

  /** Where the gizmo sits for a part: the point it is attached at. */
  function placeProxy(s: DesignSession, part: string): boolean {
    const g = s.geoms.get(part)
    const cp = s.compiled.parts[part]
    if (!g || !cp) return false
    let local: Vec3 = [0, 0, 0]
    try { local = anchorLocal(g, g.spec.align ?? 'center').p } catch { /* center */ }
    const pt = worldPoint(g, local)
    proxy.position.set(...designToViewerPoint(s, pt))
    const Rv = mulM(transpose(s.root.R), g.R)
    proxy.quaternion.setFromRotationMatrix(matrixFrom(Rv, [0, 0, 0]))
    proxy.updateMatrixWorld(true)
    return true
  }

  function refreshSelectionVisuals() {
    clearMarkers()
    const s = current()
    gizmo.detach()
    if (!s || !selected || mode !== 'build' || !s.geoms.has(selected)) return
    attachOverlay()
    showAnchors(s, selected, { faces: false })
    const isRoot = selected === s.compiled.root
    if (!isRoot && placeProxy(s, selected) && pivotOfLink(s.compiled.parts[selected].link)) {
      gizmo.attach(proxy)
    }
  }

  function select(part: string | null) {
    selected = part
    refreshSelectionVisuals()
    refreshBuildTree()
    renderInspector()
    updateParentIndicator()
  }

  function selectByLink(link: string | null) {
    if (!link) { select(null); return }
    const s = current()
    const part = s?.linkToPart.get(link) ?? null
    if (part) { select(part); return }
    // Session not ready yet: load it, then select.
    void ensureSession().then(ss => select(ss?.linkToPart.get(link) ?? null))
  }

  gizmo.addEventListener('dragging-changed', ev => {
    const on = (ev as unknown as { value: boolean }).value
    ctx.controls.enabled = !on
    const s = current()
    if (!s || !selected) return
    if (on) {
      const pivot = pivotOfLink(s.compiled.parts[selected].link)
      if (!pivot) return
      pivot.updateMatrixWorld(true)
      proxy.updateMatrixWorld(true)
      const spec = findPart(s.design, sourcePartName(s.design, selected))
      const g = s.geoms.get(selected)
      let slideNormal: THREE.Vector3 | null = null
      if (spec && g && isFlushMate(g.spec, s.geoms)) {
        const n = resolveAt(g.spec.at, s.geoms).n
        if (n) slideNormal = new THREE.Vector3(...designToViewerDir(s, n))
      }
      drag = {
        part: selected, pivot,
        pivotWorld0: pivot.matrixWorld.clone(),
        proxyWorld0: proxy.matrixWorld.clone(),
        proxyPos0: proxy.position.clone(),
        proxyQuat0: proxy.quaternion.clone(),
        slideNormal,
      }
      return
    }
    const d = drag
    drag = null
    if (d) void finishDrag(s, d)
  })

  gizmo.addEventListener('change', () => {
    if (!drag || !drag.pivot.parent) return
    if (drag.slideNormal && gizmo.mode === 'translate') {
      // Keep it flush: drop the motion along the mate normal.
      const dv = proxy.position.clone().sub(drag.proxyPos0)
      dv.addScaledVector(drag.slideNormal, -dv.dot(drag.slideNormal))
      proxy.position.copy(drag.proxyPos0).add(dv)
    }
    // Live preview: move the part's subtree by the same rigid motion as the gizmo.
    proxy.updateMatrixWorld(true)
    const D = proxy.matrixWorld.clone().multiply(drag.proxyWorld0.clone().invert())
    const parentInv = drag.pivot.parent.matrixWorld.clone().invert()
    const local = parentInv.multiply(D.multiply(drag.pivotWorld0))
    local.decompose(drag.pivot.position, drag.pivot.quaternion, drag.pivot.scale)
    drag.pivot.updateMatrixWorld(true)
  })

  async function finishDrag(s: DesignSession, d: DragState) {
    const design = s.design
    const srcName = sourcePartName(design, d.part)
    const isMirror = srcName !== d.part
    const spec = findPart(design, srcName)
    if (!spec) return
    // Mirror copies are driven by their source: reflect the motion across XZ.
    const S = (v: Vec3): Vec3 => (isMirror ? [v[0], -v[1], v[2]] : v)
    const SRS = (R: Mat3): Mat3 => (isMirror ? mulM(mulM([[1, 0, 0], [0, -1, 0], [0, 0, 1]], R), [[1, 0, 0], [0, -1, 0], [0, 0, 1]]) : R)
    let next: DesignPart = JSON.parse(JSON.stringify(spec))
    let what = ''
    if (gizmo.mode === 'translate') {
      const dv = proxy.position.clone().sub(d.proxyPos0)
      if (dv.length() < 1e-6) { refreshSelectionVisuals(); return }
      const delta = S(viewerToDesignDir(s, [dv.x * 1000, dv.y * 1000, dv.z * 1000]))
      next.at = translateAt(spec.at, delta)
      what = `Moved ${srcName}`
    } else {
      const dq = proxy.quaternion.clone().multiply(d.proxyQuat0.clone().invert())
      const Dv = mat3FromQuat(dq)
      const Dd = SRS(mulM(mulM(s.root.R, Dv), transpose(s.root.R)))
      const { axis, angle } = axisAngle(Dd)
      if (angle < 1e-4) { refreshSelectionVisuals(); return }
      const g = s.geoms.get(srcName)!
      const target = resolveAt(spec.at, s.geoms)
      if (isFlushMate(spec, s.geoms) && target.n && Math.abs(dot(axis, target.n)) > 0.999) {
        // Rotating about the mate normal is a spin of the mate.
        const sign = dot(axis, scale(target.n, -1)) > 0 ? 1 : -1
        next.spin_deg = Math.round(((spec.spin_deg ?? 0) + sign * angle * 180 / Math.PI) * 100) / 100
        if (!next.spin_deg) delete next.spin_deg
      } else {
        const { z, x } = axesOf(mulM(Dd, g.R))
        next.z_axis = z
        next.x_axis = x
        delete next.spin_deg
      }
      what = `Rotated ${srcName}`
    }
    const ok = await commit(replacePart(design, srcName, next), what)
    if (!ok) ctx.reparseUrdf(ctx.getUrdfText())   // undo the live preview
  }

  // ── placing a catalog part ──────────────────────────────────────────────
  interface Target {
    part: string
    anchor: string | null      // null = free placement (not mated)
    anchorInfo?: Anchor
    offset: Vec3               // design world mm
    point?: Vec3               // free placement point (design world)
  }
  interface Carry {
    comp: CatalogComponent
    group: THREE.Group
    lengthMm?: number
    alignKeys: string[]
    alignIdx: number
    alignLocked: boolean
    spin: number
    joint: JointType
    jointLocked: boolean
    target: Target | null
    spec: DesignPart | null
  }
  let carry: Carry | null = null
  let lastMouse = { clientX: 0, clientY: 0 }

  function carrySpec(s: DesignSession, c: Carry): DesignPart | null {
    const t = c.target
    if (!t) return null
    const base: DesignPart = {
      name: uniqueName(s.design, shortName(c.comp.id)),
      component: c.comp.id,
      parent: sourcePartName(s.design, t.part),
    }
    if (c.lengthMm) base.length_mm = c.lengthMm
    if (t.anchor === null) {
      // Free placement: stand the part upright with its bottom at the point.
      const g = s.geoms.get(t.part)!
      const center = worldPoint(g, g.centerLocal)
      base.at = { ref: t.part, offset: round3(sub(t.point!, center)) }
      base.align = '-z'
      base.z_axis = '+z'
      const a = c.spin * Math.PI / 180
      base.x_axis = round3([Math.cos(a), Math.sin(a), 0]).map(v => Math.round(v * 1e6) / 1e6) as Vec3
      return base
    }
    const ref = `${t.part}.${t.anchor}`
    base.at = norm(t.offset) > 1e-6 ? { ref, offset: round3(t.offset) } : ref
    base.align = c.alignKeys[c.alignIdx]
    const spin = normDeg(uprightSpin(base, s.geoms) + c.spin)
    if (spin) base.spin_deg = spin
    if (c.joint !== 'fixed') base.joint = { type: c.joint }
    return base
  }

  /** Twist (deg) about the mate normal that keeps a flush-mated part upright:
   *  on a side face its own up axis points up (a camera level, an ultrasonic's
   *  transducers side by side); on a top/bottom face it faces forward. */
  function uprightSpin(spec: DesignPart, placed: Map<string, PartGeom>): number {
    try {
      if (!isFlushMate(spec, placed)) return 0
      const { R } = poseForSpec(spec, placed)
      const n = resolveAt(spec.at, placed).n!
      const about = scale(n, -1)
      const vertical = Math.abs(n[2]) > 0.7
      const want: Vec3 = vertical ? [1, 0, 0] : [0, 0, 1]
      // The part's axis that should line up: +X on horizontal faces; on walls
      // its +Z unless that is the mate axis itself, then +Y.
      const local = vertical ? col(R, 0) : Math.abs(dot(col(R, 2), n)) < 0.7 ? col(R, 2) : col(R, 1)
      const a = sub(local, scale(about, dot(local, about)))
      const w = sub(want, scale(about, dot(want, about)))
      if (norm(a) < 1e-6 || norm(w) < 1e-6) return 0
      return Math.round(Math.atan2(dot(about, cross(a, w)), dot(a, w)) * 180 / Math.PI)
    } catch {
      return 0
    }
  }

  /** Anchors already mated: ones something is mounted on, and each part's
   *  own mounting anchor (compiled names, so mirrored copies count too). */
  function usedAnchors(s: DesignSession): Set<string> {
    const out = new Set<string>()
    for (const [name, g] of s.geoms) {
      if (g.spec.align && g.spec.align !== 'center') out.add(`${name}.${g.spec.align}`)
      const at = atParts(g.spec.at)
      if (at.ref && at.ref.includes('.')) {
        out.add(at.ref)
        const src = at.ref.split('.')[0]
        if (s.compiled.parts[name]?.mirror_of && s.geoms.has(mirrorName(src))) out.add(`${mirrorName(src)}.${at.ref.slice(src.length + 1)}`)
      }
    }
    return out
  }

  /** Where the mouse points: an anchor on a robot part, or the floor. */
  function findTarget(s: DesignSession, e: { clientX: number; clientY: number }): Target | null {
    const ray = raycaster(e)
    const hits = ray.intersectObjects(pickTargets(), false)
    for (const hit of hits) {
      const link = linkOfObject(hit.object)
      const part = link ? s.linkToPart.get(link) : undefined
      if (!part) continue
      if (s.compiled.parts[part]?.mirror_of) {
        setHud('Mirrored parts follow their original — attach to the left-side part instead')
        return null
      }
      const g = s.geoms.get(part)!
      const hitP = viewerToDesignPoint(s, viewerPointFromWorld(hit.point))
      const faceN = hit.face
        ? viewerToDesignDir(s, viewerDirFromWorld(hit.face.normal.clone().transformDirection(hit.object.matrixWorld)))
        : null
      const anchors = partAnchors(g)
      // 1. A free connector close to the cursor snaps exactly — only one you
      //    can see (facing the camera, near the surface point under the
      //    cursor), not one buried inside or already holding something.
      const mouse = new THREE.Vector2(e.clientX - ctx.canvas.getBoundingClientRect().left, e.clientY - ctx.canvas.getBoundingClientRect().top)
      const rayDir = viewerToDesignDir(s, viewerDirFromWorld(ray.ray.direction))
      const used = usedAnchors(s)
      const reach = Math.max(15, 0.25 * Math.max(...g.size))
      let best: { a: Anchor; d: number } | null = null
      for (const a of anchors) {
        if (a.kind !== 'connector' || !a.n) continue
        if (used.has(`${part}.${a.key}`)) continue
        const pw = worldPoint(g, a.p)
        if (dot(mulV(g.R, a.n), rayDir) > 0.2 || norm(sub(pw, hitP)) > reach) continue
        const d = toScreen(s, pw).distanceTo(mouse)
        if (d < SNAP_PX && (!best || d < best.d)) best = { a, d }
      }
      if (best) return { part, anchor: best.a.key, anchorInfo: best.a, offset: [0, 0, 0] }
      // 2. Otherwise the face under the cursor, keeping the exact spot as an offset.
      let face: { a: Anchor; score: number } | null = null
      for (const a of anchors) {
        if (a.kind === 'connector' || !a.n) continue
        const nW = mulV(g.R, a.n)
        const align = faceN ? dot(nW, faceN) : 1
        if (align < 0.7) continue
        const planeDist = Math.abs(dot(sub(hitP, worldPoint(g, a.p)), nW))
        const score = planeDist - (a.kind === 'primitive' ? 0.5 : 0) + (1 - align) * 20
        if (!face || score < face.score) face = { a, score }
      }
      if (!face) continue
      const nW = mulV(g.R, face.a.n!)
      const pW = worldPoint(g, face.a.p)
      const off = sub(hitP, pW)
      const inPlane = sub(off, scale(nW, dot(off, nW)))
      const snapped = round3(inPlane.map(v => Math.round(v * 2) / 2) as Vec3)
      return { part, anchor: face.a.key, anchorInfo: face.a, offset: snapped }
    }
    // 3. Nothing under the cursor: free placement on the floor, attached to
    //    the selected part (or the root body).
    // The visible floor is the scene's y = 0 plane (the robot is grounded on it).
    const plane = new THREE.Plane(new THREE.Vector3(0, 1, 0), 0)
    const pt = new THREE.Vector3()
    if (!ray.ray.intersectPlane(plane, pt)) return null
    const base = selected && s.geoms.has(selected) && !s.compiled.parts[selected]?.mirror_of ? selected : s.compiled.root
    const dp = viewerToDesignPoint(s, viewerPointFromWorld(pt))
    return { part: base, anchor: null, offset: [0, 0, 0], point: round3(dp.map(v => Math.round(v)) as Vec3) }
  }

  function updateCarry(e: { clientX: number; clientY: number }) {
    const s = current()
    if (!carry || !s) return
    lastMouse = { clientX: e.clientX, clientY: e.clientY }
    const t = findTarget(s, e)
    const prevAnchor = carry.target?.anchor
    carry.target = t
    if (t && t.anchor !== prevAnchor) {
      if (!carry.alignLocked) {
        carry.alignKeys = alignCandidates(carry.comp, t.anchorInfo)
        carry.alignIdx = 0
      }
      if (!carry.jointLocked) carry.joint = defaultJoint(t.anchorInfo, carry.comp)
    }
    clearMarkers()
    if (!t) { carry.group.visible = false; carry.spec = null; return }
    showAnchors(s, t.part, { faces: false, hot: t.anchor ?? undefined })
    const spec = carrySpec(s, carry)
    carry.spec = spec
    if (!spec) { carry.group.visible = false; return }
    try {
      const { R, p } = poseForSpec(spec, s.geoms)
      const Rv = mulM(transpose(s.root.R), R)
      carry.group.matrixAutoUpdate = false
      carry.group.matrix.copy(matrixFrom(Rv, designToViewerPoint(s, p)))
      carry.group.matrixWorldNeedsUpdate = true
      carry.group.visible = true
    } catch (err) {
      carry.group.visible = false
      console.warn('[editor] placement preview failed:', err)
    }
    updateCarryHud()
  }

  function updateCarryHud() {
    if (!carry) return
    const t = carry.target
    const where = !t ? 'point at the robot or the floor'
      : t.anchor === null ? `free on the floor, attached to ${t.part}`
      : `${t.part}.${t.anchor}${norm(t.offset) > 0 ? ` (+${t.offset.map(v => fmtNum(v, 0)).join(', ')} mm)` : ''}`
    const len = carry.lengthMm ? ` · length ${carry.lengthMm} mm ([ ])` : ''
    const al = t?.anchor !== null && t ? ` · mate: ${carry.alignKeys[carry.alignIdx]} (Tab)` : ''
    const jt = t?.anchor ? ` · joint: ${carry.joint} (J)` : ''
    setHud(`${carry.comp.name} → ${where}${al} · spin ${carry.spec?.spin_deg ?? 0}° (R)${jt}${len} · click to place · Esc cancel`)
  }

  async function enterCarry(comp: CatalogComponent) {
    exitCarry()
    if (mode !== 'build') { ctx.showToast('Switch to Build mode to place components', 'info'); return }
    if (ctx.isSimActive?.()) { ctx.showToast('Exit simulation before placing components', 'info'); return }
    const s = await ensureSession()
    if (!s) { ctx.showToast('This URDF cannot be edited as a design (see Validation)', 'error'); return }
    gizmo.detach()
    const lengthMm = isParametric(comp) ? 100 : undefined
    const group = partVisual(comp, componentSizeMm(comp, lengthMm))
    group.visible = false
    attachOverlay()
    overlay.add(group)
    carry = {
      comp, group, lengthMm, alignKeys: alignCandidates(comp, undefined), alignIdx: 0, alignLocked: false,
      spin: 0, joint: 'fixed', jointLocked: false, target: null, spec: null,
    }
    preloadComponentMesh(comp.id, () => {
      if (!carry || carry.comp.id !== comp.id) return
      rebuildCarryVisual()
    })
    const r = ctx.canvas.getBoundingClientRect()
    updateCarry(lastMouse.clientX ? lastMouse : { clientX: r.left + r.width / 2, clientY: r.top + r.height / 2 })
    updateCarryHud()
  }

  function rebuildCarryVisual() {
    if (!carry) return
    const m = carry.group.matrix.clone(), vis = carry.group.visible
    overlay.remove(carry.group)
    carry.group = partVisual(carry.comp, componentSizeMm(carry.comp, carry.lengthMm))
    carry.group.matrixAutoUpdate = false
    carry.group.matrix.copy(m)
    carry.group.visible = vis
    overlay.add(carry.group)
  }

  function exitCarry() {
    if (carry) overlay.remove(carry.group)
    carry = null
    setHud('')
    clearMarkers()
    compItems?.querySelectorAll('.tb-item.selected').forEach(i => i.classList.remove('selected'))
    refreshSelectionVisuals()
  }

  async function commitCarry() {
    const s = current()
    if (!carry || !s || !carry.spec) return
    const spec = carry.spec
    const name = carry.comp.name
    exitCarry()
    if (await commit(addPart(s.design, spec), `Placed ${name}`)) select(spec.name)
  }

  // ── inspector ───────────────────────────────────────────────────────────
  function anchorOptions(g: PartGeom | undefined, currentKey: string): string {
    const keys = ['center', ...(g ? partAnchors(g).map(a => a.key) : [])]
    if (!keys.includes(currentKey)) keys.push(currentKey)
    return keys.map(k => `<option value="${esc(k)}"${k === currentKey ? ' selected' : ''}>${esc(k)}</option>`).join('')
  }

  function dirOptions(v: unknown): string {
    const cur = typeof v === 'string' ? v : Array.isArray(v) ? 'custom' : ''
    const opts = ['', ...DIRECTIONS, ...(cur === 'custom' ? ['custom'] : [])]
    return opts.map(o => `<option value="${o}"${o === cur ? ' selected' : ''}>${o === '' ? '—' : o === 'custom' ? `[${(v as number[]).map(x => fmtNum(x, 3)).join(', ')}]` : o}</option>`).join('')
  }

  function num(id: string): number | undefined {
    const el = document.getElementById(id) as HTMLInputElement | null
    if (!el || el.value.trim() === '') return undefined
    const v = Number(el.value)
    return Number.isFinite(v) ? v : undefined
  }
  function val(id: string): string {
    return (document.getElementById(id) as HTMLInputElement | HTMLSelectElement | null)?.value ?? ''
  }

  function renderInspector() {
    if (!inspBody || !inspTitle) return
    const s = current()
    if (!selected || !s) {
      inspTitle.textContent = 'Properties'
      inspBody.innerHTML = `<div class="insp-empty">${selected ? 'Loading design…' : 'Select a part in the viewport or the structure tree'}</div>`
      return
    }
    const srcName = sourcePartName(s.design, selected)
    const spec = findPart(s.design, srcName)
    const cp = s.compiled.parts[selected]
    if (!spec || !cp) { inspBody.innerHTML = '<div class="insp-empty">Part not found</div>'; return }
    const isMirror = srcName !== selected
    const isRoot = selected === s.compiled.root
    const comp = getComponent(spec.component)
    const at = atParts(spec.at)
    const refPart = at.ref ? at.ref.split('.')[0] : ''
    const refAnchor = at.ref && at.ref.includes('.') ? at.ref.slice(at.ref.indexOf('.') + 1) : 'center'
    const flush = isFlushMate(spec, s.geoms)
    const j = jointOf(spec)
    const others = s.design.parts.filter(p => p.name !== srcName && !dependents(s.design, srcName).includes(p.name))
    const partOpts = (curName: string) => others.map(p => `<option value="${esc(p.name)}"${p.name === curName ? ' selected' : ''}>${esc(p.name)}</option>`).join('')
    const mass = ctx.getKinematicGraph()[cp.link]?.mass ?? 0
    const issues = lastIssues.filter(i => new RegExp(`\\b${selected}\\b`).test(i))
    const occ = anchorOccupancy(s.design)
    const mounted = [...occ.entries()].filter(([k]) => k.startsWith(`${srcName}.`))
    const worldC = worldPoint(s.geoms.get(selected)!, cp.center_local)
    const isRev = j.type === 'revolute', isPri = j.type === 'prismatic'

    inspTitle.textContent = selected
    inspBody.innerHTML = `
      <div class="bi-section">
        <div class="bi-section-title">Part</div>
        ${isMirror ? `<div class="insp-note">Mirror of <b>${esc(srcName)}</b> — edits apply to both sides.</div>` : ''}
        <div class="insp-row"><span class="insp-key">Name</span><input id="de-name" class="insp-input" value="${esc(srcName)}"${isMirror ? ' disabled' : ''}/></div>
        <div class="insp-row"><span class="insp-key">Part</span><span class="insp-val">${esc(comp ? comp.name : spec.shape?.length ? `Custom body (${spec.shape.length} shapes)` : 'Frame')}</span></div>
        <div class="insp-row"><span class="insp-key">URDF link</span><span class="insp-val">${esc(cp.link)}</span></div>
        <div class="insp-row"><span class="insp-key">Mass</span><span class="insp-val">${mass >= 1 ? mass.toFixed(2) + ' kg' : Math.round(mass * 1000) + ' g'}</span></div>
        ${comp && isParametric(comp) ? `<div class="insp-row"><span class="insp-key">Length mm</span><input id="de-length" class="insp-input" type="number" step="1" value="${spec.length_mm ?? ''}"/></div>` : ''}
      </div>
      ${isRoot ? `<div class="bi-section"><div class="bi-section-title">Placement</div><div class="insp-empty">Root body — the robot is built around it.</div></div>` : `
      <div class="bi-section">
        <div class="bi-section-title">Placement</div>
        <div class="insp-row"><span class="insp-key">Parent</span><select id="de-parent" class="insp-select">${partOpts(spec.parent ?? '')}</select></div>
        <div class="insp-row"><span class="insp-key">Attached to</span>
          <select id="de-atpart" class="insp-select">${at.point ? '<option value="" selected>world point</option>' : ''}${partOpts(refPart)}</select>
          ${at.point ? '' : `<select id="de-atanchor" class="insp-select">${anchorOptions(s.geoms.get(refPart), refAnchor)}</select>`}</div>
        <div class="joint-limits-row"><span class="joint-lim-label">${at.point ? 'Point mm' : 'Offset mm'}</span>
          ${[0, 1, 2].map(i => `<input id="de-off${i}" class="joint-lim-input" type="number" step="1" value="${fmtNum((at.point ?? at.offset)[i], 2)}"/>`).join('')}</div>
        <div class="insp-row"><span class="insp-key">Mates with</span><select id="de-align" class="insp-select">${anchorOptions(s.geoms.get(selected), spec.align ?? 'center')}</select></div>
        ${flush ? `
        <div class="insp-row"><span class="insp-key">Orientation</span><span class="insp-val">flush mate</span></div>
        <div class="insp-row"><span class="insp-key">Spin °</span><input id="de-spin" class="insp-input" type="number" step="15" value="${spec.spin_deg ?? 0}"/></div>
        <button type="button" class="bi-action-btn" id="de-explicit">Set axes explicitly</button>` : `
        <div class="insp-row"><span class="insp-key">Local +Z points</span><select id="de-zaxis" class="insp-select">${dirOptions(spec.z_axis)}</select></div>
        <div class="insp-row"><span class="insp-key">Local +X points</span><select id="de-xaxis" class="insp-select">${dirOptions(spec.x_axis)}</select></div>
        ${spec.z_axis !== undefined && refAnchor !== 'center' ? '<button type="button" class="bi-action-btn" id="de-flush">Mate flush instead</button>' : ''}`}
        ${spec.mirror !== undefined || !isMirror ? `<label class="insp-row"><span class="insp-key">Mirror L/R</span><input id="de-mirror" type="checkbox"${spec.mirror ? ' checked' : ''}/></label>` : ''}
      </div>
      <div class="bi-section">
        <div class="bi-section-title">Joint to parent</div>
        <div class="insp-row"><span class="insp-key">Type</span><select id="de-jtype" class="insp-select">
          ${(['fixed', 'revolute', 'continuous', 'prismatic'] as JointType[]).map(t => `<option${t === j.type ? ' selected' : ''}>${t}</option>`).join('')}</select></div>
        ${j.type !== 'fixed' ? `
        <div class="insp-row"><span class="insp-key">Axis</span><select id="de-jaxis" class="insp-select">
          <option value=""${j.axis === undefined ? ' selected' : ''}>from mount (${cp.joint?.axis ? cp.joint.axis.map(v => fmtNum(v, 2)).join(', ') : '—'})</option>
          ${DIRECTIONS.map(d => `<option${j.axis === d ? ' selected' : ''}>${d}</option>`).join('')}
          ${Array.isArray(j.axis) || (typeof j.axis === 'string' && !DIRECTIONS.includes(j.axis as typeof DIRECTIONS[number])) ? `<option value="__keep" selected>${esc(String(j.axis))}</option>` : ''}</select></div>
        ${isRev ? `<div class="insp-row"><span class="insp-key">Rest °</span><input id="de-rest" class="insp-input" type="number" step="5" value="${j.rest_deg ?? 0}"/></div>` : ''}
        ${isRev || isPri ? `<div class="joint-limits-row"><span class="joint-lim-label">Limits ${isPri ? 'mm' : '°'}</span>
          <input id="de-lo" class="joint-lim-input" type="number" value="${isPri ? j.lower_mm ?? 0 : j.lower_deg ?? -90}"/>
          <input id="de-hi" class="joint-lim-input" type="number" value="${isPri ? j.upper_mm ?? 50 : j.upper_deg ?? 90}"/></div>` : ''}
        <label class="insp-row"><span class="insp-key">Passive (no motor)</span><input id="de-passive" type="checkbox"${(j.passive ?? cp.joint?.passive) ? ' checked' : ''}/></label>
        ${j.passive === undefined && cp.joint?.passive ? '<div class="insp-note">Its parent is not an actuator, so this joint is a free pivot in the simulator.</div>' : ''}` : ''}
      </div>`}
      <div class="bi-section">
        <div class="bi-section-title">Mounted here</div>
        ${mounted.length ? mounted.map(([k, v]) => `<div class="insp-row"><span class="insp-key">${esc(k.slice(srcName.length + 1))}</span><span class="insp-val">${v.map(esc).join(', ')}</span></div>`).join('') : '<div class="insp-empty">Nothing mounted on this part\'s connectors</div>'}
      </div>
      <div class="bi-section">
        <div class="bi-section-title">Measured</div>
        <div class="insp-row"><span class="insp-key">Center (mm)</span><span class="insp-val">${worldC.map(v => fmtNum(v, 0)).join(', ')}</span></div>
        <div class="insp-row"><span class="insp-key">Size (mm)</span><span class="insp-val">${cp.size.map(v => fmtNum(v, 0)).join(' × ')}</span></div>
        ${cp.joint?.pivot ? `<div class="insp-row"><span class="insp-key">Pivot (mm)</span><span class="insp-val">${cp.joint.pivot.map(v => fmtNum(v, 0)).join(', ')}</span></div>` : ''}
        ${issues.map(i => `<div class="insp-note warn">${esc(i)}</div>`).join('')}
        ${s.imported && s.notes.length ? s.notes.map(n => `<div class="insp-note">${esc(n)}</div>`).join('') : ''}
      </div>
      <div class="insp-actions-group">
        ${!isRoot ? `<button type="button" class="bi-action-btn" id="de-dup" style="width:100%">Duplicate</button>` : ''}
        <button type="button" class="bi-action-btn" id="btn-export-link-stl" style="width:100%">Export Part STL</button>
        ${!isRoot ? `<button type="button" class="bi-action-btn danger-btn" id="de-delete">${isMirror ? 'Remove mirrored copy' : `Delete${dependents(s.design, srcName).length > 1 ? ` (+${dependents(s.design, srcName).length - 1} attached)` : ''}`}</button>` : ''}
      </div>`

    const edit = (fn: (p: DesignPart) => void, msg?: string) => {
      const next: DesignPart = JSON.parse(JSON.stringify(spec))
      fn(next)
      void commit(replacePart(s.design, srcName, next), msg)
    }
    const on = (id: string, ev: string, h: () => void) => document.getElementById(id)?.addEventListener(ev, h)

    on('de-name', 'change', () => {
      const to = val('de-name').trim()
      if (!to || to === srcName) return
      if (!/^[A-Za-z][A-Za-z0-9_]*$/.test(to)) { ctx.showToast('Names are letters, digits and _', 'warning'); renderInspector(); return }
      if (s.design.parts.some(p => p.name === to)) { ctx.showToast(`${to} already exists`, 'warning'); renderInspector(); return }
      selected = to
      void commit(renamePart(s.design, srcName, to), `Renamed to ${to}`)
    })
    on('de-length', 'change', () => edit(p => { p.length_mm = num('de-length') }))
    on('de-parent', 'change', () => edit(p => { p.parent = val('de-parent') }))
    on('de-atpart', 'change', () => edit(p => {
      // Re-reference without moving the part: keep the attach point where it
      // is and express it relative to the newly chosen part's center.
      const np = val('de-atpart')
      const here = resolveAt(spec.at, s.geoms).pt
      if (!np) { p.at = round3(here); return }
      const g = s.geoms.get(np)
      const off = g ? round3(sub(here, worldPoint(g, [0, 0, 0]))) : [0, 0, 0] as Vec3
      p.at = norm(off) > 0 ? { ref: np, offset: off } : np
      if (!p.parent) p.parent = np
    }))
    on('de-atanchor', 'change', () => edit(p => {
      const a = val('de-atanchor')
      const ref = a === 'center' ? refPart : `${refPart}.${a}`
      p.at = norm(at.offset) > 0 ? { ref, offset: at.offset } : ref
    }))
    const setOff = () => edit(p => {
      const v: Vec3 = [num('de-off0') ?? 0, num('de-off1') ?? 0, num('de-off2') ?? 0]
      p.at = at.point ? v : norm(v) > 0 ? { ref: at.ref!, offset: v } : at.ref!
    })
    for (const i of [0, 1, 2]) on(`de-off${i}`, 'change', setOff)
    on('de-align', 'change', () => edit(p => { p.align = val('de-align') }))
    on('de-spin', 'change', () => edit(p => { const v = num('de-spin') ?? 0; if (v) p.spin_deg = v; else delete p.spin_deg }))
    on('de-explicit', 'click', () => edit(p => {
      const { z, x } = axesOf(s.geoms.get(srcName)!.R)
      p.z_axis = z; p.x_axis = x; delete p.spin_deg
    }))
    on('de-flush', 'click', () => edit(p => { delete p.z_axis; delete p.x_axis }))
    const setAxes = () => edit(p => {
      const z = val('de-zaxis'), x = val('de-xaxis')
      if (z !== 'custom') { if (z) p.z_axis = z; else delete p.z_axis }
      if (x !== 'custom') { if (x) p.x_axis = x; else delete p.x_axis }
    })
    on('de-zaxis', 'change', setAxes)
    on('de-xaxis', 'change', setAxes)
    on('de-mirror', 'change', () => edit(p => {
      if ((document.getElementById('de-mirror') as HTMLInputElement).checked) p.mirror = true
      else delete p.mirror
    }))
    on('de-jtype', 'change', () => edit(p => {
      const t = val('de-jtype') as JointType
      p.joint = t === 'fixed' ? undefined : { ...jointOf(p), type: t }
      if (!p.joint) delete p.joint
    }))
    const setJoint = () => edit(p => {
      const jj: JointSpec = { ...jointOf(p) }
      const ax = val('de-jaxis')
      if (ax === '') delete jj.axis
      else if (ax !== '__keep') jj.axis = ax
      if (isRev) {
        const r = num('de-rest'); if (r) jj.rest_deg = r; else delete jj.rest_deg
        jj.lower_deg = num('de-lo'); jj.upper_deg = num('de-hi')
      } else if (isPri) {
        jj.lower_mm = num('de-lo'); jj.upper_mm = num('de-hi')
      }
      const passive = (document.getElementById('de-passive') as HTMLInputElement | null)?.checked
      // Only record it when it differs from what the compiler infers.
      if (passive === !!cp.joint?.passive) delete jj.passive
      else jj.passive = !!passive
      p.joint = jj
    })
    for (const id of ['de-jaxis', 'de-rest', 'de-lo', 'de-hi', 'de-passive']) on(id, 'change', setJoint)
    on('de-delete', 'click', () => void deleteSelected())
    on('de-dup', 'click', () => {
      const copy: DesignPart = JSON.parse(JSON.stringify(spec))
      copy.name = uniqueName(s.design, srcName.replace(/_\d+$/, ''))
      delete copy.link
      delete copy.mirror_link
      delete copy.mirror
      if (copy.joint && typeof copy.joint === 'object') delete copy.joint.name
      // Next to the original, one part-width along its own X.
      const g = s.geoms.get(srcName)!
      const shift: Vec3 = scale(col(g.R, 0), Math.max(20, g.size[0] * 1.2))
      copy.at = translateAt(spec.at, round3(shift))
      void commit(addPart(s.design, copy), `Duplicated ${srcName}`).then(ok => { if (ok) select(copy.name) })
    })
    on('btn-export-link-stl', 'click', () => void exportSelectedStl())
  }

  async function deleteSelected() {
    const s = current()
    if (!s || !selected) return
    if (selected === s.compiled.root) { ctx.showToast('The root body can\'t be deleted', 'warning'); return }
    const src = sourcePartName(s.design, selected)
    if (src !== selected) {
      const spec = findPart(s.design, src)!
      const next = { ...spec }
      delete next.mirror
      if (await commit(replacePart(s.design, src, next), `Removed mirrored ${selected}`)) select(null)
      return
    }
    const { design, removed } = removePart(s.design, src)
    if (await commit(design, `Deleted ${removed.length > 1 ? `${src} and ${removed.length - 1} attached part(s)` : src}`)) select(null)
  }

  // ── structure tree ──────────────────────────────────────────────────────
  function refreshBuildTree() {
    if (!buildTree) return
    const s = current()
    const graph = ctx.getKinematicGraph()
    const links = Object.values(graph)
    const total = links.reduce((a, l) => a + (l.mass || 0), 0)
    if (bsParts) bsParts.textContent = String(links.length)
    if (bsMass) bsMass.textContent = `${Math.round(total * 1000)} g`
    if (bsJoints) bsJoints.textContent = String(Math.max(0, links.length - 1))
    buildEmpty?.classList.toggle('hidden', links.length > 1)
    buildTree.innerHTML = ''
    if (!s) return
    // Design order (mirrored copies right after their source).
    const rank = new Map<string, number>()
    for (const p of s.design.parts) {
      rank.set(p.name, rank.size)
      if (p.mirror) rank.set(mirrorName(p.name), rank.size)
    }
    const kids = new Map<string, string[]>()
    for (const [n, cp] of Object.entries(s.compiled.parts)) {
      if (cp.parent) kids.set(cp.parent, [...(kids.get(cp.parent) ?? []), n])
    }
    for (const list of kids.values()) list.sort((a, b) => (rank.get(a) ?? 1e9) - (rank.get(b) ?? 1e9))
    const rows: Array<{ n: string; depth: number }> = []
    const walk = (n: string, depth: number) => {
      rows.push({ n, depth })
      for (const c of kids.get(n) ?? []) walk(c, depth + 1)
    }
    walk(s.compiled.root, 0)
    for (const { n, depth } of rows) {
      const cp = s.compiled.parts[n]
      const el = document.createElement('div')
      el.className = 'bt-row' + (selected === n ? ' selected' : '')
      el.style.paddingLeft = `${10 + depth * 14}px`
      const cat = cp.component ? categoryOf(cp.component) : null
      const cc = cat ? CATEGORY_COLORS[cat] ?? [0.6, 0.6, 0.6, 1] : [0.75, 0.75, 0.78, 1]
      const dotHtml = `<span class="bt-cat-dot" style="background:rgb(${cc.slice(0, 3).map(v => Math.round(v * 255)).join(',')})"></span>`
      const jt = cp.joint?.type ?? (cp.parent ? 'fixed' : 'root')
      el.innerHTML = `<span class="bt-joint-badge">${jt.slice(0, 3)}</span>${dotHtml}<span class="bt-name">${esc(n)}${cp.mirror_of ? ' <span class="bt-mirror">⇋</span>' : ''}</span><span class="bt-mass">${Math.round((graph[cp.link]?.mass || 0) * 1000)}g</span>`
      el.title = cp.component ? `${n} — ${getComponent(cp.component)?.name ?? cp.component}` : n
      el.addEventListener('click', () => select(n))
      buildTree.appendChild(el)
    }
  }

  function updateParentIndicator() {
    if (!parentName || !parentIndicator) return
    parentName.textContent = selected ?? 'root body'
    parentIndicator.classList.toggle('has-selection', !!selected)
  }

  // ── component picker ────────────────────────────────────────────────────
  function compactSpec(c: CatalogComponent): string {
    const me = c.mechanical_electrical as Record<string, unknown>
    for (const [k, unitS] of [['max_torque_nm', ' Nm'], ['stall_torque_nm', ' Nm'], ['holding_torque_nm', ' Nm'], ['max_force_n', ' N'], ['grip_force_n', ' N'], ['fov_h_deg', '° FOV'], ['range_m', ' m range'], ['capacity_mah', ' mAh']] as const) {
      if (me[k] != null) return `${me[k]}${unitS}`
    }
    const m = c.physical.mass_kg ?? c.physical.mass_kg_per_100mm
    if (m != null) return m >= 1 ? `${m.toFixed(1)} kg` : `${Math.round(m * 1000)} g`
    return ''
  }

  function renderDetail(c: CatalogComponent) {
    if (!compDetail) return
    const size = componentSizeMm(c)
    const specs = Object.entries(c.mechanical_electrical).slice(0, 6).map(([k, v]) =>
      `<div class="tb-kv"><span class="tb-kv-key">${esc(k.replace(/_/g, ' '))}</span><span class="tb-kv-val">${esc(Array.isArray(v) ? v.join(' – ') : String(v))}</span></div>`).join('')
    const conns = (c.connectors ?? []).map(k => `<span class="tb-chip">${esc(k.id)}</span>`).join('')
    compDetail.innerHTML = `
      <div class="tb-detail-name">${esc(c.name)}</div>
      <div class="tb-detail-desc">${esc(c.description ?? '')}</div>
      <div class="tb-detail-sec"><div class="tb-detail-sec-title">Size</div>
        <div class="tb-kv"><span class="tb-kv-key">${isParametric(c) ? 'Cross-section (cut to length)' : 'Bounding box'}</span><span class="tb-kv-val">${isParametric(c) ? `${size[0]}×${size[1]} mm` : size.map(v => Math.round(v)).join('×') + ' mm'}</span></div></div>
      ${conns ? `<div class="tb-detail-sec"><div class="tb-detail-sec-title">Connectors</div><div class="tb-detail-chiprow">${conns}</div></div>` : ''}
      <div class="tb-detail-sec"><div class="tb-detail-sec-title">Specs</div>${specs}</div>`
  }

  function renderComponents(filter: string) {
    const data = catalog()
    if (!compItems || !data) return
    const q = filter.trim().toLowerCase()
    compItems.innerHTML = ''
    for (const [catName, cat] of Object.entries(data.categories)) {
      const comps = cat.components.filter(c => !q || c.name.toLowerCase().includes(q) || c.id.includes(q) || (c.description ?? '').toLowerCase().includes(q))
      if (!comps.length) continue
      const icon = CATEGORY_ICONS[catName] ?? ''
      const cc = CATEGORY_COLORS[catName] ?? [0.6, 0.6, 0.6, 1]
      const color = `rgb(${cc.slice(0, 3).map(v => Math.round(v * 255)).join(',')})`
      const catEl = document.createElement('div')
      catEl.className = 'tb-cat'
      catEl.innerHTML = `<span class="tb-cat-icon">${icon}</span><span class="tb-cat-label">${esc(catName.replace(/_/g, ' ').replace(/\b\w/g, m => m.toUpperCase()))}</span><span class="tb-cat-count">${comps.length}</span><span class="tb-cat-arrow">▾</span>`
      const list = document.createElement('div')
      list.className = 'tb-list'
      catEl.addEventListener('click', () => {
        const collapsed = list.classList.toggle('collapsed')
        catEl.querySelector('.tb-cat-arrow')!.textContent = collapsed ? '▸' : '▾'
      })
      for (const c of comps) {
        const el = document.createElement('div')
        el.className = 'tb-item'
        el.innerHTML = `<span class="tb-cat-icon tb-item-icon" style="color:${color}">${icon}</span><div class="tb-item-info"><div class="tb-item-name">${esc(c.name)}</div><div class="tb-item-meta">${esc(compactSpec(c))}</div></div>`
        el.addEventListener('click', () => {
          compItems.querySelectorAll('.tb-item').forEach(i => i.classList.remove('selected'))
          el.classList.add('selected')
          renderDetail(c)
          void enterCarry(c)
        })
        list.appendChild(el)
      }
      compItems.append(catEl, list)
    }
    if (!compItems.children.length) compItems.innerHTML = '<div class="tb-empty">No matching components</div>'
  }

  void loadCatalog()
    .then(() => renderComponents(''))
    .catch(() => { if (compItems) compItems.innerHTML = '<div class="tb-empty">Failed to load the component catalog</div>' })
  toolboxSearch?.addEventListener('input', () => renderComponents(toolboxSearch.value))

  // ── export / reset ──────────────────────────────────────────────────────
  const stl = new STLExporter()
  async function saveStl(group: THREE.Object3D, defaultName: string) {
    try {
      const path = await invoke<string | null>('save_file_dialog', { default_name: defaultName, filters: [['STL Files', ['stl']]] })
      if (!path) return
      const hidden: THREE.Object3D[] = []
      group.traverse(o => { if (o.visible && (o.userData as Record<string, unknown>).isCollision) { o.visible = false; hidden.push(o) } })
      let text: string
      try { text = stl.parse(group, { binary: false }) as string } finally { hidden.forEach(o => { o.visible = true }) }
      await invoke('save_file', { path, content: text })
      ctx.showToast(`Exported ${path.split(/[\\/]/).pop()}`, 'success')
    } catch (err) {
      ctx.showToast(`Export failed: ${err}`, 'error')
    }
  }
  async function exportSelectedStl() {
    const s = current()
    const link = s && selected ? s.compiled.parts[selected]?.link : null
    const g = link ? ctx.getParsedRobot().linkGroups.get(link) : null
    if (!g) { ctx.showToast('Select a part to export', 'warning'); return }
    await saveStl(g, `${selected}.stl`)
  }
  document.getElementById('btn-export-stl')?.addEventListener('click', () => void saveStl(ctx.getParsedRobot().group, 'robot.stl'))
  document.getElementById('btn-export-urdf-pkg')?.addEventListener('click', async () => {
    const urdf = ctx.getUrdfText().trim()
    if (!urdf) { ctx.showToast('Nothing to export — URDF is empty', 'warning'); return }
    const folder = await invoke<string | null>('open_folder_dialog')
    if (!folder) return
    const name = (urdf.match(/<robot[^>]+name="([^"]+)"/)?.[1] ?? 'robot').replace(/[^A-Za-z0-9_]/g, '_')
    await invoke('save_file', { path: `${folder}/${name}.urdf`, content: urdf })
    await invoke('save_file', {
      path: `${folder}/package.xml`, content: `<?xml version="1.0"?>
<package format="3">
  <name>${name}</name>
  <version>0.0.1</version>
  <description>${name} robot description</description>
  <maintainer email="robot@example.com">Vector Builder</maintainer>
  <license>Apache-2.0</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <exec_depend>robot_state_publisher</exec_depend>
  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
`,
    })
    ctx.showToast(`Exported URDF package to ${folder}`, 'success')
  })
  document.getElementById('btn-focus-base')?.addEventListener('click', () => {
    const s = current()
    select(s ? s.compiled.root : null)
  })
  document.getElementById('btn-clear-assembly')?.addEventListener('click', async () => {
    const empty: Design = {
      name: 'robot', summary: '',
      parts: [{ name: 'base', shape: [{ name: 'plate', shape: 'box', size_mm: [200, 150, 6], color: [0.55, 0.58, 0.62] }] }],
    }
    ctx.zeroAssemblyWorldPosition?.()
    if (await commit(empty, 'Robot reset')) {
      try { ctx.groundAssembly?.() } catch { /* cosmetic */ }
      select('base')
    }
  })

  // ── input ───────────────────────────────────────────────────────────────
  const down = { x: 0, y: 0 }
  ctx.canvas.addEventListener('pointerdown', e => { down.x = e.clientX; down.y = e.clientY })
  ctx.canvas.addEventListener('click', e => {
    if (!ctx.isViewport3D()) return
    const moved = Math.hypot(e.clientX - down.x, e.clientY - down.y) > (carry ? 22 : 8)
    if (carry) { if (!moved) void commitCarry(); return }
    if (moved || gizmo.dragging) return
    const hits = raycaster(e).intersectObjects(pickTargets(), false)
    const link = hits.length ? linkOfObject(hits[0].object) : null
    if (ctx.getInteractionMode() === 'inspect') { ctx.onInspectLinkFocused(link); return }
    selectByLink(link)
  })
  ctx.canvas.addEventListener('mousemove', e => {
    if (!ctx.isViewport3D()) return
    lastMouse = { clientX: e.clientX, clientY: e.clientY }
    if (carry) { updateCarry(e); return }
  })

  const isTyping = (t: EventTarget | null) => {
    const el = t as HTMLElement | null
    return !!el && (el.tagName === 'INPUT' || el.tagName === 'TEXTAREA' || el.tagName === 'SELECT' || el.isContentEditable || !!el.closest?.('.monaco-editor'))
  }

  function restore(text: string, label: string) {
    invalidateSession()
    session = null
    ctx.setUrdfText(text)
    ctx.reparseUrdf(text)
    ctx.showToast(label, 'info')
  }

  document.addEventListener('keydown', e => {
    if (isTyping(e.target) || !ctx.isViewport3D()) return
    const k = e.key.toLowerCase()
    if ((e.ctrlKey || e.metaKey) && k === 'z' && !e.shiftKey) {
      e.preventDefault()
      const prev = undo.pop()
      if (prev === undefined) return
      redo.push(ctx.getUrdfText())
      restore(prev, 'Undo')
      return
    }
    if ((e.ctrlKey || e.metaKey) && (k === 'y' || (k === 'z' && e.shiftKey))) {
      e.preventDefault()
      const next = redo.pop()
      if (next === undefined) return
      undo.push(ctx.getUrdfText())
      restore(next, 'Redo')
      return
    }
    if (carry) {
      if (k === 'escape') { exitCarry(); return }
      if (k === 'enter') { e.preventDefault(); void commitCarry(); return }
      if (k === 'r') { e.preventDefault(); carry.spin = normDeg(carry.spin + (e.shiftKey ? -15 : 15)); updateCarry(lastMouse); return }
      if (k === 'tab') {
        e.preventDefault()
        const n = carry.alignKeys.length
        carry.alignIdx = (carry.alignIdx + (e.shiftKey ? n - 1 : 1)) % n
        carry.alignLocked = true
        updateCarry(lastMouse)
        return
      }
      if (k === 'j') {
        e.preventDefault()
        const order: JointType[] = ['fixed', 'revolute', 'continuous', 'prismatic']
        carry.joint = order[(order.indexOf(carry.joint) + 1) % order.length]
        carry.jointLocked = true
        updateCarry(lastMouse)
        return
      }
      if ((k === '[' || k === ']') && carry.lengthMm) {
        e.preventDefault()
        carry.lengthMm = Math.max(10, carry.lengthMm + (k === ']' ? 1 : -1) * (e.shiftKey ? 50 : 10))
        rebuildCarryVisual()
        updateCarry(lastMouse)
        return
      }
      return
    }
    if (k === 'i') { ctx.switchPanel('inspector'); return }
    if (k === 't') { ctx.switchPanel('toolbox'); return }
    if (k === 'r' && gizmo.object) {
      gizmo.setMode(gizmo.mode === 'translate' ? 'rotate' : 'translate')
      ctx.showToast(`Gizmo: ${gizmo.mode}`, 'info')
      return
    }
    if (k === 'escape' && selected) { select(null); return }
    if ((k === 'delete' || k === 'backspace') && selected) { e.preventDefault(); void deleteSelected() }
  })

  // ── model updates ───────────────────────────────────────────────────────
  let refreshTimer: ReturnType<typeof setTimeout> | null = null
  function onModelUpdated() {
    attachOverlay()
    if (mode === 'inspect') {
      selected = null
      gizmo.detach()
      clearMarkers()
    }
    // Fast path: the URDF we just compiled.
    refreshSelectionVisuals()
    refreshBuildTree()
    renderInspector()
    if (refreshTimer) clearTimeout(refreshTimer)
    refreshTimer = setTimeout(() => {
      void ensureSession().then(s => {
        if (!s) { refreshBuildTree(); renderInspector(); return }
        if (selected && !s.geoms.has(selected)) selected = null
        refreshSelectionVisuals()
        refreshBuildTree()
        renderInspector()
        if (carry) updateCarry(lastMouse)
      })
    }, 200)
    ctx.onAfterModelUpdated?.()
  }

  renderInspector()
  refreshBuildTree()

  // Test/automation hook (read-only view of the editor state).
  ;(window as unknown as { __vectorDesignEditor: unknown }).__vectorDesignEditor = {
    state: () => ({
      selected, mode, carrying: carry?.comp.id ?? null, target: carry?.target ?? null,
      spec: carry?.spec ?? null, hasSession: !!session, sessionCurrent: !!current(),
      undo: undo.length, redo: redo.length,
    }),
    session: () => current(),
  }

  return {
    onModelUpdated,
    recordUndoExternal: (content: string) => recordUndo(content),
    exitCarryMode: exitCarry,
    onInteractionModeChanged: (m: 'build' | 'inspect') => {
      mode = m
      if (m === 'inspect') { exitCarry(); select(null) }
      else refreshSelectionVisuals()
    },
    setSelectedLink: (link: string | null) => selectByLink(link),
    getUndoState: () => ({ undo: [...undo], redo: [...redo] }),
    restoreUndoState: st => { undo = [...st.undo]; redo = [...st.redo] },
    refreshOverlay: () => {
      const s = current()
      if (s && selected && !drag) placeProxy(s, selected)
    },
  }
}
