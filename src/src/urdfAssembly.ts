import * as THREE from 'three'
import { OrbitControls } from 'three/addons/controls/OrbitControls.js'
import { TransformControls } from 'three/addons/controls/TransformControls.js'
import { STLExporter } from 'three/addons/exporters/STLExporter.js'
import { invoke } from '@tauri-apps/api/core'
import { generateVisuals, CATEGORY_COLORS } from './componentMeshes'
import type { UrdfVisualDesc } from './componentMeshes'
import { isMountLinkName } from './attachmentNodes'
import { hasMeshOverride } from './richVisuals/meshOverrides'
import { SLOW_MESH_BLACKLIST } from './richVisuals/index'

export interface ParsedRobotLike {
  group: THREE.Group
  linkGroups: Map<string, THREE.Group>
  joints: Map<string, { group: THREE.Group; axis: THREE.Vector3; type: string }>
}

export interface UrdfAssemblyContext {
  scene: THREE.Scene
  camera: THREE.PerspectiveCamera
  canvas: HTMLCanvasElement
  controls: OrbitControls
  showToast: (message: string, type?: 'success' | 'warning' | 'error' | 'info') => void
  switchPanel: (name: string) => void
  getUrdfText: () => string
  setUrdfText: (content: string) => void
  reparseUrdf: () => void
  getParsedRobot: () => ParsedRobotLike
  getKinematicGraph: () => Record<string, { name: string; mass: number; parent?: string; children: string[] }>
  getKinematicJoints: () => Record<string, { name: string; type: string; axis: string; parentLink: string; childLink: string }>
  isViewport3D: () => boolean
}

// ── Preset types ──────────────────────────────────────────────────────────────

interface PresetPhysical {
  mass_kg?: number
  mass_kg_per_100mm?: number
  bounding_box_mm?: number[]
  cross_section_mm?: number[]
  inertia_primitive: string
}

interface PresetComponent {
  id: string
  name: string
  description: string
  physical: PresetPhysical
  mechanical_electrical: Record<string, unknown>
  mounting_logic: Record<string, unknown>
  sim_metadata: Record<string, unknown>
}

interface PresetCategory {
  description: string
  components: PresetComponent[]
}

interface PresetData {
  categories: Record<string, PresetCategory>
}

export interface UrdfAssemblyApi {
  onModelUpdated(): void
  recordUndoExternal(content: string): void
}

function parseNums(s: string, len = 3): number[] {
  const arr = (s || '').trim().split(/\s+/).map(v => Number(v))
  const out: number[] = []
  for (let i = 0; i < len; i++) out.push(Number.isFinite(arr[i]) ? arr[i] : 0)
  return out
}

function fmt(n: number): string {
  return Number.isFinite(n) ? n.toFixed(6).replace(/\.?0+$/, '') : '0'
}

function ensureOrigin(el: Element, doc: Document): Element {
  let origin = el.querySelector(':scope > origin')
  if (!origin) {
    origin = doc.createElement('origin')
    origin.setAttribute('xyz', '0 0 0')
    origin.setAttribute('rpy', '0 0 0')
    el.appendChild(origin)
  }
  if (!origin.getAttribute('xyz')) origin.setAttribute('xyz', '0 0 0')
  if (!origin.getAttribute('rpy')) origin.setAttribute('rpy', '0 0 0')
  return origin
}

function isTypingTarget(t: EventTarget | null): boolean {
  if (!(t instanceof HTMLElement)) return false
  const tag = t.tagName
  if (tag === 'INPUT' || tag === 'TEXTAREA' || tag === 'SELECT') return true
  if (t.closest('.monaco-editor')) return true
  return false
}

export function initUrdfAssembly(ctx: UrdfAssemblyContext): UrdfAssemblyApi {
  const gizmo = new TransformControls(ctx.camera, ctx.canvas)
  gizmo.setMode('translate')
  gizmo.setSpace('world')
  gizmo.setSize(0.8)
  ctx.scene.add(gizmo.getHelper())

  let selectedLink: string | null = null
  let gizmoBasePivotWorld = new THREE.Matrix4()
  let pointerDown = new THREE.Vector2()
  let urdfUndo: string[] = []
  let urdfRedo: string[] = []
  let rootDragWarned = false

  // ── Attachment nodes (mount-frame links) ────────────────────────────────────
  const nodesGroup = new THREE.Group()
  nodesGroup.name = 'attachment_nodes'
  ctx.scene.add(nodesGroup)
  const nodeRingsGroup = new THREE.Group()
  nodeRingsGroup.name = 'attachment_node_rings'
  ctx.scene.add(nodeRingsGroup)
  const nodeMeshByMount = new Map<string, THREE.Mesh>()
  const nodeRingsByMount = new Map<string, THREE.Group>()
  // Synthesized attachment face nodes. `mountLink` is a unique synthetic key
  // of the form `<parentLinkName>::<faceId>` and is NOT a real URDF link.
  // `localPos` is the face center in the parent link's local frame.
  let mountNodes: Array<{
    mountLink: string
    parentLink: string
    nodeId: string
    localPos: THREE.Vector3
    worldPos: THREE.Vector3
    worldQuat: THREE.Quaternion
  }> = []
  const occupiedNodeKeys = new Set<string>()

  const NODE_MAT_NEUTRAL = new THREE.MeshBasicMaterial({ color: 0x2f7bff, transparent: true, opacity: 0.95, depthTest: false })
  const NODE_MAT_COMPAT = new THREE.MeshBasicMaterial({ color: 0x44b3ff, transparent: true, opacity: 1, depthTest: false })
  const NODE_MAT_BEST = new THREE.MeshBasicMaterial({ color: 0x2dff8a, transparent: true, opacity: 1, depthTest: false })
  const NODE_MAT_OCCUPIED = new THREE.MeshBasicMaterial({ color: 0xff5533, transparent: true, opacity: 0.95, depthTest: false })
  const NODE_GEO = new THREE.BoxGeometry(0.012, 0.012, 0.012)
  const SNAP_RADIUS_M = 0.05
  const SNAP_ANGLE_RAD = Math.PI / 4

  const ghostGroup = new THREE.Group()
  ghostGroup.name = 'snap_ghost'
  ghostGroup.visible = false
  ctx.scene.add(ghostGroup)
  const ghostBox = new THREE.Box3()
  const ghostBoxHelper = new THREE.Box3Helper(ghostBox, 0x33ff99)
  ghostBoxHelper.renderOrder = 999
  ghostGroup.add(ghostBoxHelper)
  let bestMountCandidate: {
    mountLink: string
    targetParentLink: string
    sourceMountLink: string
    reason?: string
    desiredLinkWorld: THREE.Matrix4
  } | null = null

  function updateGhostBoxAtTransform(linkGroup: THREE.Group, desiredLinkWorld: THREE.Matrix4) {
    if (!selectedLink) return
    linkGroup.updateMatrixWorld(true)

    const box = new THREE.Box3().setFromObject(linkGroup)
    if (!isFinite(box.min.x) || !isFinite(box.max.x)) return
    const currentLinkWorld = linkGroup.matrixWorld.clone()
    const delta = desiredLinkWorld.clone().multiply(currentLinkWorld.invert())
    box.applyMatrix4(delta)

    ghostBox.copy(box)
    ghostBoxHelper.updateMatrixWorld(true)
  }

  function clearBestCandidateHighlight() {
    for (const [mount, mesh] of nodeMeshByMount.entries()) {
      mesh.material = isMountOccupied(mount) ? NODE_MAT_OCCUPIED : NODE_MAT_NEUTRAL
    }
  }

  function setNodeMeshState(mountLink: string, state: 'neutral' | 'compatible' | 'best' | 'occupied') {
    const mesh = nodeMeshByMount.get(mountLink)
    if (!mesh) return
    mesh.material = state === 'best' ? NODE_MAT_BEST
      : state === 'compatible' ? NODE_MAT_COMPAT
      : state === 'occupied' ? NODE_MAT_OCCUPIED
      : NODE_MAT_NEUTRAL
  }

  function angleBetweenNodes(a: THREE.Quaternion, b: THREE.Quaternion): number {
    return a.angleTo(b)
  }

  function nodeClassFromId(nodeId: string): 'mount_face' | 'generic' {
    if (nodeId === 'top' || nodeId === 'bottom' || nodeId === 'x_plus' || nodeId === 'x_minus' || nodeId === 'y_plus' || nodeId === 'y_minus') return 'mount_face'
    return 'generic'
  }

  function validateSnapTarget(
    movingLink: string,
    sourceNodeClass: 'mount_face' | 'generic',
    sourceWorldPos: THREE.Vector3,
    sourceWorldQuat: THREE.Quaternion,
    target: { mountLink: string; parentLink: string; nodeId: string; worldPos: THREE.Vector3; worldQuat: THREE.Quaternion },
  ): { ok: boolean; reason: string; dist: number } {
    if (target.parentLink === movingLink) return { ok: false, reason: 'same-component', dist: Infinity }
    if (isMountOccupied(target.mountLink)) return { ok: false, reason: 'occupied', dist: Infinity }
    const dist = target.worldPos.distanceTo(sourceWorldPos)
    if (dist > SNAP_RADIUS_M) return { ok: false, reason: 'too-far', dist }

    // Topology: do not attach into own subtree.
    const graph = ctx.getKinematicGraph()
    const stack = [movingLink]
    const seen = new Set<string>()
    while (stack.length) {
      const cur = stack.pop()!
      if (cur === target.parentLink) return { ok: false, reason: 'cycle', dist }
      if (seen.has(cur)) continue
      seen.add(cur)
      for (const ch of graph[cur]?.children ?? []) stack.push(ch)
    }

    // Type compatibility (v1): face nodes connect to face nodes; generic accepts generic.
    const tClass = nodeClassFromId(target.nodeId)
    const sClass = sourceNodeClass
    if (!(sClass === tClass || tClass === 'generic')) {
      return { ok: false, reason: 'type-mismatch', dist }
    }

    const ang = angleBetweenNodes(sourceWorldQuat, target.worldQuat)
    if (ang > SNAP_ANGLE_RAD) return { ok: false, reason: 'orientation', dist }

    return { ok: true, reason: 'ok', dist }
  }

  function getSourceNodesForSelected(): Array<{
    mountLink: string
    nodeId: string
    worldPos: THREE.Vector3
    worldQuat: THREE.Quaternion
    localToSelected: THREE.Matrix4
    cls: 'mount_face' | 'generic'
  }> {
    if (!selectedLink) return []
    const selectedGroup = ctx.getParsedRobot().linkGroups.get(selectedLink)
    if (!selectedGroup) return []
    selectedGroup.updateMatrixWorld(true)
    // Face nodes share the link's orientation and sit at localPos within the link frame,
    // so the local-to-selected transform is just a translation by localPos.
    const out: Array<{
      mountLink: string
      nodeId: string
      worldPos: THREE.Vector3
      worldQuat: THREE.Quaternion
      localToSelected: THREE.Matrix4
      cls: 'mount_face' | 'generic'
    }> = []
    for (const n of mountNodes) {
      if (n.parentLink !== selectedLink) continue
      const localToSelected = new THREE.Matrix4().makeTranslation(n.localPos.x, n.localPos.y, n.localPos.z)
      out.push({
        mountLink: n.mountLink,
        nodeId: n.nodeId,
        worldPos: n.worldPos.clone(),
        worldQuat: n.worldQuat.clone(),
        localToSelected,
        cls: nodeClassFromId(n.nodeId),
      })
    }
    return out
  }

  function updateBestCandidateDuringDrag(_pivot: THREE.Group) {
    refreshNodeWorldTransforms()
    bestMountCandidate = null
    if (mountNodes.length === 0) {
      clearBestCandidateHighlight()
      ghostGroup.visible = false
      return
    }
    if (!selectedLink) return
    const selectedGroup = ctx.getParsedRobot().linkGroups.get(selectedLink)
    if (!selectedGroup) return
    selectedGroup.updateMatrixWorld(true)
    const sourceNodes = getSourceNodesForSelected()
    if (sourceNodes.length === 0) {
      ghostGroup.visible = false
      clearBestCandidateHighlight()
      return
    }

    // Choose closest valid source-target pair.
    let best: {
      mountLink: string
      targetParentLink: string
      sourceMountLink: string
      dist: number
      reason: string
      desiredLinkWorld: THREE.Matrix4
    } | null = null
    let firstFailureReason: string | null = null

    for (const srcNode of sourceNodes) {
      for (const target of mountNodes) {
        if (target.parentLink === selectedLink) continue
        const verdict = validateSnapTarget(selectedLink, srcNode.cls, srcNode.worldPos, srcNode.worldQuat, target)
        if (!verdict.ok) {
          if (!firstFailureReason) firstFailureReason = verdict.reason
          continue
        }
        const sourceLocalInv = srcNode.localToSelected.clone().invert()
        const targetWorld = new THREE.Matrix4().compose(target.worldPos, target.worldQuat, new THREE.Vector3(1, 1, 1))
        const desiredLinkWorld = targetWorld.clone().multiply(sourceLocalInv)
        if (!best || verdict.dist < best.dist) {
          best = {
            mountLink: target.mountLink,
            targetParentLink: target.parentLink,
            sourceMountLink: srcNode.mountLink,
            dist: verdict.dist,
            reason: 'ok',
            desiredLinkWorld,
          }
        }
      }
    }

    clearBestCandidateHighlight()
    nodesGroup.visible = true
    for (const target of mountNodes) {
      setNodeMeshState(target.mountLink, isMountOccupied(target.mountLink) ? 'occupied' : 'neutral')
    }
    for (const target of mountNodes) {
      if (target.parentLink === selectedLink) continue
      setNodeMeshState(target.mountLink, isMountOccupied(target.mountLink) ? 'occupied' : 'compatible')
    }

    if (!best || !Number.isFinite(best.dist) || !best.desiredLinkWorld) {
      ghostGroup.visible = false
      bestMountCandidate = firstFailureReason
        ? { mountLink: '', targetParentLink: '', sourceMountLink: '', reason: firstFailureReason, desiredLinkWorld: new THREE.Matrix4() }
        : null
      return
    }
    bestMountCandidate = {
      mountLink: best.mountLink,
      targetParentLink: best.targetParentLink,
      sourceMountLink: best.sourceMountLink,
      reason: best.reason,
      desiredLinkWorld: best.desiredLinkWorld,
    }
    setNodeMeshState(best.mountLink, 'best')

    // Ghost preview: show subtree bounding box at solved snapped pose.
    ghostGroup.visible = true
    updateGhostBoxAtTransform(selectedGroup, best.desiredLinkWorld)
  }

  function isMountOccupied(mountKey: string): boolean {
    return occupiedNodeKeys.has(mountKey)
  }

  // Compute the bounding box of a link's own geometry in its local frame,
  // excluding child-link pivot subtrees so we don't include downstream components.
  function computeLinkLocalBoundingBox(linkGroup: THREE.Group): THREE.Box3 | null {
    const pivotGroups = new Set<THREE.Object3D>()
    for (const [, jointInfo] of ctx.getParsedRobot().joints) {
      pivotGroups.add(jointInfo.group)
    }

    linkGroup.updateMatrixWorld(true)
    const linkWorldInv = linkGroup.matrixWorld.clone().invert()

    const box = new THREE.Box3()
    let hasGeom = false

    const tempBox = new THREE.Box3()
    const tempMatrix = new THREE.Matrix4()
    function walk(obj: THREE.Object3D) {
      if (obj !== linkGroup && pivotGroups.has(obj)) return
      const mesh = obj as THREE.Mesh
      if (mesh.isMesh && mesh.geometry) {
        if (!mesh.geometry.boundingBox) mesh.geometry.computeBoundingBox()
        const bb = mesh.geometry.boundingBox
        if (bb) {
          mesh.updateMatrixWorld(true)
          tempMatrix.multiplyMatrices(linkWorldInv, mesh.matrixWorld)
          tempBox.copy(bb).applyMatrix4(tempMatrix)
          box.union(tempBox)
          hasGeom = true
        }
      }
      for (const child of obj.children) walk(child)
    }
    walk(linkGroup)

    return hasGeom ? box : null
  }

  function rebuildMountNodes() {
    nodesGroup.clear()
    nodeRingsGroup.clear()
    nodeMeshByMount.clear()
    nodeRingsByMount.clear()
    mountNodes = []
    occupiedNodeKeys.clear()

    const graph = ctx.getKinematicGraph()
    const kinJoints = ctx.getKinematicJoints()
    const parsed = ctx.getParsedRobot()

    const FACE_DIRS: Array<{ id: string; axis: [number, number, number] }> = [
      { id: 'top',     axis: [ 0,  0,  1] },
      { id: 'bottom',  axis: [ 0,  0, -1] },
      { id: 'x_plus',  axis: [ 1,  0,  0] },
      { id: 'x_minus', axis: [-1,  0,  0] },
      { id: 'y_plus',  axis: [ 0,  1,  0] },
      { id: 'y_minus', axis: [ 0, -1,  0] },
    ]
    const OCCUPIED_DIST_M = 0.025

    for (const linkName of Object.keys(graph)) {
      if (isMountLinkName(linkName)) continue
      const lg = parsed.linkGroups.get(linkName)
      if (!lg) continue
      const localBox = computeLinkLocalBoundingBox(lg)
      if (!localBox || localBox.isEmpty()) continue

      const center = localBox.getCenter(new THREE.Vector3())
      const size = localBox.getSize(new THREE.Vector3())
      const half = new THREE.Vector3(size.x / 2, size.y / 2, size.z / 2)

      // Gather child-joint origin xyz in parent-local frame, used to determine occupancy.
      const childOriginsLocal: THREE.Vector3[] = []
      for (const j of Object.values(kinJoints)) {
        if (j.parentLink !== linkName) continue
        if (isMountLinkName(j.childLink)) continue
        // The URDF parser retains joint origin in the pivot group transform of the child link.
        const childLg = parsed.linkGroups.get(j.childLink)
        if (!childLg) continue
        // Find the pivot group whose parent is lg
        let pivot: THREE.Object3D | null = childLg.parent
        if (!pivot) continue
        // pivot's position is joint origin in parent-local (lg) frame
        childOriginsLocal.push(new THREE.Vector3().copy(pivot.position))
      }

      lg.updateMatrixWorld(true)
      const linkWorldQuat = new THREE.Quaternion()
      lg.matrixWorld.decompose(new THREE.Vector3(), linkWorldQuat, new THREE.Vector3())

      for (const f of FACE_DIRS) {
        const localPos = new THREE.Vector3(
          center.x + f.axis[0] * half.x,
          center.y + f.axis[1] * half.y,
          center.z + f.axis[2] * half.z,
        )
        const nodeKey = `${linkName}::${f.id}`
        const worldPos = localPos.clone().applyMatrix4(lg.matrixWorld)
        const worldQuat = linkWorldQuat.clone()

        // Occupancy: is there a child joint whose origin sits near this face?
        let occupied = false
        for (const co of childOriginsLocal) {
          if (co.distanceTo(localPos) <= OCCUPIED_DIST_M) { occupied = true; break }
        }
        if (occupied) occupiedNodeKeys.add(nodeKey)

        mountNodes.push({
          mountLink: nodeKey,
          parentLink: linkName,
          nodeId: f.id,
          localPos,
          worldPos,
          worldQuat,
        })

        const mesh = new THREE.Mesh(NODE_GEO, occupied ? NODE_MAT_OCCUPIED : NODE_MAT_NEUTRAL)
        mesh.position.copy(worldPos)
        mesh.quaternion.copy(worldQuat)
        mesh.renderOrder = 999
        nodesGroup.add(mesh)
        nodeMeshByMount.set(nodeKey, mesh)

        const rings = makeNodeAxisRings()
        rings.position.copy(worldPos)
        rings.quaternion.copy(worldQuat)
        nodeRingsGroup.add(rings)
        nodeRingsByMount.set(nodeKey, rings)
      }
    }
    // Nodes are only shown while actively dragging a component; the drag
    // handler flips this on/off. Keep the group hidden by default.
    nodesGroup.visible = false
    applyNodeRingVisibility()
  }

  function refreshNodeWorldTransforms() {
    const parsed = ctx.getParsedRobot()
    const worldQuatTmp = new THREE.Quaternion()
    const worldPosTmp = new THREE.Vector3()
    const worldScaleTmp = new THREE.Vector3()
    for (const n of mountNodes) {
      const lg = parsed.linkGroups.get(n.parentLink)
      if (!lg) continue
      lg.updateMatrixWorld(true)
      lg.matrixWorld.decompose(worldPosTmp, worldQuatTmp, worldScaleTmp)
      n.worldQuat.copy(worldQuatTmp)
      n.worldPos.copy(n.localPos).applyMatrix4(lg.matrixWorld)

      const mesh = nodeMeshByMount.get(n.mountLink)
      if (mesh) {
        mesh.position.copy(n.worldPos)
        mesh.quaternion.copy(n.worldQuat)
      }
      const rings = nodeRingsByMount.get(n.mountLink)
      if (rings) {
        rings.position.copy(n.worldPos)
        rings.quaternion.copy(n.worldQuat)
      }
    }
  }

  const buildTree = document.getElementById('build-tree') as HTMLDivElement | null
  const buildEmpty = document.getElementById('build-empty') as HTMLDivElement | null
  const bsParts = document.getElementById('bs-parts') as HTMLSpanElement | null
  const bsMass = document.getElementById('bs-mass') as HTMLSpanElement | null
  const bsJoints = document.getElementById('bs-joints') as HTMLSpanElement | null
  const inspBody = document.querySelector('#panel-inspector .insp-body') as HTMLDivElement | null
  const inspTitle = document.getElementById('insp-title') as HTMLSpanElement | null
  const toolboxSearch = document.getElementById('toolbox-search') as HTMLInputElement | null
  const btnFocusBase = document.getElementById('btn-load-example') as HTMLButtonElement | null
  const btnResetRobot = document.getElementById('btn-clear-assembly') as HTMLButtonElement | null
  const btnToggleNodeRings = document.getElementById('toggle-node-rings') as HTMLButtonElement | null
  let showNodeRings = false

  function makeNodeAxisRings(): THREE.Group {
    const rings = new THREE.Group()
    const radius = 0.018
    const tube = 0.0015
    const ringGeo = new THREE.TorusGeometry(radius, tube, 8, 40)
    const matX = new THREE.MeshBasicMaterial({ color: 0xff6666, transparent: true, opacity: 0.9, depthTest: false })
    const matY = new THREE.MeshBasicMaterial({ color: 0x66ff66, transparent: true, opacity: 0.9, depthTest: false })
    const matZ = new THREE.MeshBasicMaterial({ color: 0x66aaff, transparent: true, opacity: 0.9, depthTest: false })
    const rx = new THREE.Mesh(ringGeo, matX)
    rx.rotation.y = Math.PI / 2
    const ry = new THREE.Mesh(ringGeo, matY)
    ry.rotation.x = Math.PI / 2
    const rz = new THREE.Mesh(ringGeo, matZ)
    rings.add(rx, ry, rz)
    rings.name = 'node-axis-rings'
    rings.visible = showNodeRings
    return rings
  }

  function applyNodeRingVisibility() {
    nodeRingsGroup.visible = showNodeRings
    btnToggleNodeRings?.classList.toggle('active', showNodeRings)
  }

  function recordUndo() {
    urdfUndo.push(ctx.getUrdfText())
    if (urdfUndo.length > 80) urdfUndo.shift()
    urdfRedo = []
  }

  function commitUrdf(mutator: (doc: Document) => boolean): boolean {
    const current = ctx.getUrdfText()
    const parser = new DOMParser()
    const doc = parser.parseFromString(current, 'application/xml')
    if (doc.documentElement.nodeName === 'parsererror') {
      ctx.showToast('Cannot edit invalid URDF', 'error')
      return false
    }
    recordUndo()
    const changed = mutator(doc)
    if (!changed) {
      urdfUndo.pop()
      return false
    }
    const xml = new XMLSerializer().serializeToString(doc)
    ctx.setUrdfText(xml)
    ctx.reparseUrdf()
    return true
  }

  function getPickTargets(): THREE.Mesh[] {
    const targets: THREE.Mesh[] = []
    ctx.getParsedRobot().group.traverse(o => {
      if (o instanceof THREE.Mesh) {
        const name = (o.userData as Record<string, unknown>).urdfLinkName
        if (typeof name === 'string' && name && !isMountLinkName(name)) targets.push(o)
      }
    })
    return targets
  }

  function makeRaycaster(e: { clientX: number; clientY: number }): THREE.Raycaster {
    const rect = ctx.canvas.getBoundingClientRect()
    const ndc = new THREE.Vector2(
      ((e.clientX - rect.left) / rect.width) * 2 - 1,
      -((e.clientY - rect.top) / rect.height) * 2 + 1,
    )
    const ray = new THREE.Raycaster()
    ray.setFromCamera(ndc, ctx.camera)
    return ray
  }

  function getParentJointForLink(linkName: string): { name: string; type: string; axis: string; parentLink: string; childLink: string } | null {
    const joints = ctx.getKinematicJoints()
    return Object.values(joints).find(j => j.childLink === linkName) ?? null
  }

  function getPivotGroupForLink(linkName: string): THREE.Group | null {
    const parentJoint = getParentJointForLink(linkName)
    if (!parentJoint) return null
    return ctx.getParsedRobot().joints.get(parentJoint.name)?.group ?? null
  }

  function refreshBuildPanel() {
    if (!buildTree || !buildEmpty || !bsParts || !bsMass || !bsJoints) return
    const graph = ctx.getKinematicGraph()
    const joints = ctx.getKinematicJoints()
    const links = Object.values(graph).filter(l => !isMountLinkName(l.name))
    const totalMass = links.reduce((sum, l) => sum + (l.mass || 0), 0)
    bsParts.textContent = String(links.length)
    bsMass.textContent = `${Math.round(totalMass * 1000)} g`
    bsJoints.textContent = String(Object.keys(joints).length)
    buildEmpty.classList.toggle('hidden', links.length > 0)

    const jointByChild = new Map<string, { name: string; type: string; parentLink: string }>()
    for (const j of Object.values(joints)) jointByChild.set(j.childLink, j)
    const roots = links.filter(l => !l.parent)
    const rows: Array<{ link: string; depth: number }> = []
    const walk = (name: string, depth: number) => {
      if (isMountLinkName(name)) return
      rows.push({ link: name, depth })
      const node = graph[name]
      if (!node) return
      for (const c of node.children) walk(c, depth + 1)
    }
    for (const r of roots) walk(r.name, 0)

    buildTree.innerHTML = ''
    for (const row of rows) {
      const meta = graph[row.link]
      const edge = jointByChild.get(row.link)
      const el = document.createElement('div')
      el.className = 'bt-row' + (selectedLink === row.link ? ' selected' : '')
      el.style.paddingLeft = `${10 + row.depth * 14}px`

      // Detect category from link name (preset links use ID prefixes)
      let catDot = ''
      if (presetData) {
        for (const [catName, cat] of Object.entries(presetData.categories)) {
          if (cat.components.some(c => row.link.startsWith(c.id))) {
            const cc = CATEGORY_COLORS[catName] ?? [0.6, 0.6, 0.6, 1]
            catDot = `<span class="bt-cat-dot" style="background:rgb(${Math.round(cc[0]*255)},${Math.round(cc[1]*255)},${Math.round(cc[2]*255)})"></span>`
            break
          }
        }
      }

      el.innerHTML = `
        <span class="bt-joint-badge">${edge ? edge.type.slice(0, 3) : 'root'}</span>
        ${catDot}
        <span class="bt-name">${row.link}</span>
        <span class="bt-mass">${Math.round((meta?.mass || 0) * 1000)}g</span>
      `
      el.addEventListener('click', () => selectLink(row.link))
      buildTree.appendChild(el)
    }
  }

  function renderInspector() {
    if (!inspBody || !inspTitle) return
    if (!selectedLink) {
      inspTitle.textContent = 'Properties'
      inspBody.innerHTML = '<div class="insp-empty">Select a URDF link in viewport or build tree</div>'
      return
    }
    const joints = ctx.getKinematicJoints()
    const parentJoint = Object.values(joints).find(j => j.childLink === selectedLink) || null
    // Find synthetic face nodes belonging to the currently selected link, plus any
    // child links whose joint origins match those faces (= occupying components).
    const nodesForLink = mountNodes.filter(n => n.parentLink === selectedLink)
    const childJointsByOrigin = Object.values(joints).filter(j => j.parentLink === selectedLink && !isMountLinkName(j.childLink))
    const mountRows = nodesForLink.map(n => {
      const occupied = isMountOccupied(n.mountLink)
      let occBy = ''
      if (occupied) {
        // Find the child joint whose pivot position is closest to this face
        const parsed = ctx.getParsedRobot()
        let bestDist = Infinity
        for (const j of childJointsByOrigin) {
          const childLg = parsed.linkGroups.get(j.childLink)
          const pivot = childLg?.parent
          if (!pivot) continue
          const d = pivot.position.distanceTo(n.localPos)
          if (d < bestDist) { bestDist = d; occBy = j.childLink }
        }
      }
      return `<div class="insp-row">
        <span class="insp-key">${n.nodeId}</span>
        <span class="insp-val">${occupied ? `occupied by ${occBy}` : 'free'}</span>
        ${occupied && occBy ? `<button type="button" class="bi-action-btn" data-detach-mount="${n.mountLink}" data-detach-child="${occBy}">Detach</button>` : ''}
      </div>`
    }).join('')
    inspTitle.textContent = selectedLink
    inspBody.innerHTML = `
      <div class="bi-section">
        <div class="bi-section-title">Link</div>
        <div class="insp-row"><span class="insp-key">Name</span><span class="insp-val">${selectedLink}</span></div>
      </div>
      <div class="bi-section">
        <div class="bi-section-title">Attachment Nodes</div>
        ${nodesForLink.length ? mountRows : '<div class="insp-empty">No mount nodes on this link</div>'}
      </div>
      <div class="bi-section">
        <div class="bi-section-title">Parent Joint</div>
        <div class="insp-row"><span class="insp-key">Name</span><span class="insp-val">${parentJoint?.name ?? 'root'}</span></div>
        <div class="insp-row"><span class="insp-key">Type</span><span class="insp-val">${parentJoint?.type ?? '--'}</span></div>
        ${parentJoint ? `
          <div class="joint-limits-row">
            <span class="joint-lim-label">Origin XYZ</span>
            <input id="urdf-origin-x" class="joint-lim-input" />
            <input id="urdf-origin-y" class="joint-lim-input" />
            <input id="urdf-origin-z" class="joint-lim-input" />
          </div>
          <div class="joint-limits-row">
            <span class="joint-lim-label">Origin RPY</span>
            <input id="urdf-origin-r" class="joint-lim-input" />
            <input id="urdf-origin-p" class="joint-lim-input" />
            <input id="urdf-origin-yaw" class="joint-lim-input" />
          </div>
        ` : '<div class="insp-empty">Root link has no parent joint origin</div>'}
      </div>
      <div class="insp-actions-group">
        ${parentJoint ? '<button type="button" class="bi-action-btn apply-btn" id="urdf-apply-origin">Apply Changes</button>' : ''}
        <button type="button" class="bi-action-btn" id="btn-export-link-stl" style="width:100%">Export Link STL</button>
        <button type="button" class="bi-action-btn danger-btn" id="urdf-delete-link">Delete Link</button>
      </div>
    `
    // Wire up delete button (available for all links)
    const deleteBtn = document.getElementById('urdf-delete-link')
    if (deleteBtn && selectedLink) {
      const linkToDelete = selectedLink
      deleteBtn.addEventListener('click', () => deleteLink(linkToDelete))
    }

    // Wire up detach buttons
    inspBody.querySelectorAll('button[data-detach-mount]').forEach(btn => {
      btn.addEventListener('click', () => {
        const mount = (btn as HTMLElement).getAttribute('data-detach-mount') || ''
        const child = (btn as HTMLElement).getAttribute('data-detach-child') || ''
        if (!mount || !child) return
        const childParentJoint = getParentJointForLink(child)
        if (!childParentJoint) return
        const ok = commitUrdf(documentXml => {
          const jointEl = documentXml.querySelector(`joint[name="${childParentJoint.name}"]`)
          if (!jointEl) return false
          const pEl = jointEl.querySelector('parent')
          if (!pEl) return false
          // Reattach the detached child to the currently selected link at origin.
          pEl.setAttribute('link', selectedLink!)
          const origin = ensureOrigin(jointEl, documentXml)
          origin.setAttribute('xyz', '0 0 0')
          origin.setAttribute('rpy', origin.getAttribute('rpy') || '0 0 0')
          return true
        })
        if (ok) ctx.showToast(`Detached ${child} from ${mount}`, 'success')
      })
    })
    // Wire up per-link STL export button
    const exportLinkBtn = document.getElementById('btn-export-link-stl')
    if (exportLinkBtn) {
      exportLinkBtn.addEventListener('click', exportSelectedLinkSTL)
    }

    if (!parentJoint) return

    const parser = new DOMParser()
    const doc = parser.parseFromString(ctx.getUrdfText(), 'application/xml')
    const jointEl = doc.querySelector(`joint[name="${parentJoint.name}"]`)
    const origin = jointEl ? ensureOrigin(jointEl, doc) : null
    const xyz = parseNums(origin?.getAttribute('xyz') || '0 0 0', 3)
    const rpy = parseNums(origin?.getAttribute('rpy') || '0 0 0', 3)

    const x = document.getElementById('urdf-origin-x') as HTMLInputElement | null
    const y = document.getElementById('urdf-origin-y') as HTMLInputElement | null
    const z = document.getElementById('urdf-origin-z') as HTMLInputElement | null
    const r = document.getElementById('urdf-origin-r') as HTMLInputElement | null
    const p = document.getElementById('urdf-origin-p') as HTMLInputElement | null
    const yw = document.getElementById('urdf-origin-yaw') as HTMLInputElement | null
    if (!x || !y || !z || !r || !p || !yw) return
    x.value = String(xyz[0]); y.value = String(xyz[1]); z.value = String(xyz[2])
    r.value = String(rpy[0]); p.value = String(rpy[1]); yw.value = String(rpy[2])

    const applyBtn = document.getElementById('urdf-apply-origin')
    applyBtn?.addEventListener('click', () => {
      commitUrdf(documentXml => {
        const j = documentXml.querySelector(`joint[name="${parentJoint.name}"]`)
        if (!j) return false
        const o = ensureOrigin(j, documentXml)
        o.setAttribute('xyz', `${fmt(Number(x.value))} ${fmt(Number(y.value))} ${fmt(Number(z.value))}`)
        o.setAttribute('rpy', `${fmt(Number(r.value))} ${fmt(Number(p.value))} ${fmt(Number(yw.value))}`)
        return true
      })
      ctx.showToast('Updated joint origin in URDF', 'success')
    })
  }

  function deleteLink(linkName: string) {
    if (!linkName) return
    const graph = ctx.getKinematicGraph()
    const node = graph[linkName]
    if (!node) return

    // Prevent deleting root link if it's the only one
    if (!node.parent && Object.keys(graph).length <= 1) {
      ctx.showToast('Cannot delete the only remaining link', 'warning')
      return
    }

    // Collect subtree: the link itself + all descendants
    const toRemove = new Set<string>()
    const walk = (name: string) => {
      toRemove.add(name)
      const n = graph[name]
      if (n) n.children.forEach(walk)
    }
    walk(linkName)

    const childCount = toRemove.size - 1
    const label = childCount > 0 ? `"${linkName}" and ${childCount} child link${childCount > 1 ? 's' : ''}` : `"${linkName}"`

    const changed = commitUrdf(doc => {
      const robot = doc.querySelector('robot')
      if (!robot) return false

      // Remove all links in subtree
      for (const name of toRemove) {
        const linkEl = doc.querySelector(`link[name="${name}"]`)
        if (linkEl) robot.removeChild(linkEl)
      }

      // Remove all joints whose parent or child is in the subtree
      const joints = doc.querySelectorAll('joint')
      joints.forEach(j => {
        const parentName = j.querySelector('parent')?.getAttribute('link')
        const childName = j.querySelector('child')?.getAttribute('link')
        if ((parentName && toRemove.has(parentName)) || (childName && toRemove.has(childName))) {
          robot.removeChild(j)
        }
      })

      return true
    })

    if (changed) {
      gizmo.detach()
      selectedLink = null
      ctx.showToast(`Deleted ${label}`, 'success')
      refreshBuildPanel()
      renderInspector()
    }
  }

  function selectLink(name: string | null) {
    if (name && isMountLinkName(name)) {
      selectedLink = null
      gizmo.detach()
      refreshBuildPanel()
      renderInspector()
      updateParentIndicator()
      return
    }
    selectedLink = name
    gizmo.detach()
    rootDragWarned = false
    if (name) {
      const pivot = getPivotGroupForLink(name)
      if (pivot) {
        gizmo.attach(pivot)
        pivot.updateMatrixWorld(true)
        gizmoBasePivotWorld.copy(pivot.matrixWorld)
      } else {
        ctx.showToast('Root movement disabled (no world joint yet)', 'warning')
        rootDragWarned = true
      }
    }
    if (name) {
      rebuildMountNodes()
    } else {
      ghostGroup.visible = false
      bestMountCandidate = null
      clearBestCandidateHighlight()
      rebuildMountNodes()
    }
    refreshBuildPanel()
    renderInspector()
    updateParentIndicator()
  }

  const parentNameEl = document.getElementById('tb-parent-name') as HTMLSpanElement | null
  const parentIndicator = document.getElementById('tb-parent-indicator') as HTMLDivElement | null

  function updateParentIndicator() {
    if (!parentNameEl || !parentIndicator) return
    if (selectedLink) {
      parentNameEl.textContent = selectedLink
      parentIndicator.classList.add('has-selection')
    } else {
      parentNameEl.textContent = 'none selected'
      parentIndicator.classList.remove('has-selection')
    }
  }

  // ── Component Presets (loaded from generic_presets.json) ────────────────────

  const compItems = document.getElementById('comp-items') as HTMLDivElement | null
  const compDetail = document.getElementById('comp-detail') as HTMLDivElement | null
  let presetData: PresetData | null = null

  function computeBoxInertia(mass: number, xm: number, ym: number, zm: number) {
    return {
      ixx: mass / 12 * (ym * ym + zm * zm),
      iyy: mass / 12 * (xm * xm + zm * zm),
      izz: mass / 12 * (xm * xm + ym * ym),
    }
  }

  function computeCylinderInertia(mass: number, rm: number, hm: number) {
    const ixx = mass / 12 * (3 * rm * rm + hm * hm)
    return { ixx, iyy: ixx, izz: mass / 2 * rm * rm }
  }

  function computeSphereInertia(mass: number, rm: number) {
    const i = 2 / 5 * mass * rm * rm
    return { ixx: i, iyy: i, izz: i }
  }

  function getParentBounds(doc: Document, parentLinkName: string): { hx: number; hy: number; hz: number } {
    // Extract parent link's bounding half-extents from its URDF geometry
    const linkEl = doc.querySelector(`link[name="${parentLinkName}"]`)
    if (!linkEl) return { hx: 0.05, hy: 0.05, hz: 0.05 }

    const vis = linkEl.querySelector('visual geometry')
    if (!vis) return { hx: 0.05, hy: 0.05, hz: 0.05 }

    const boxEl = vis.querySelector('box')
    if (boxEl) {
      const size = (boxEl.getAttribute('size') || '0.1 0.1 0.1').split(/\s+/).map(Number)
      return { hx: (size[0] || 0.1) / 2, hy: (size[1] || 0.1) / 2, hz: (size[2] || 0.1) / 2 }
    }
    const cylEl = vis.querySelector('cylinder')
    if (cylEl) {
      const r = Number(cylEl.getAttribute('radius')) || 0.05
      const h = Number(cylEl.getAttribute('length')) || 0.1
      return { hx: r, hy: r, hz: h / 2 }
    }
    const sphEl = vis.querySelector('sphere')
    if (sphEl) {
      const r = Number(sphEl.getAttribute('radius')) || 0.05
      return { hx: r, hy: r, hz: r }
    }
    return { hx: 0.05, hy: 0.05, hz: 0.05 }
  }

  function computePlacement(
    doc: Document, parentLinkName: string,
    comp: PresetComponent,
    childX: number, _childY: number, childZ: number,
  ): { xyz: string; rpy: string } {
    const parent = getParentBounds(doc, parentLinkName)
    const mount = (comp.mounting_logic?.primary as string) || 'face_mount'
    const gap = 0.005 // 5mm clearance

    // face_mount / pcb_solder / bracket_mount → stack on top (Z+) of parent
    if (mount === 'face_mount' || mount === 'pcb_solder' || mount === 'bracket_mount') {
      const oz = parent.hz + childZ / 2 + gap
      return { xyz: `0 0 ${oz.toFixed(4)}`, rpy: '0 0 0' }
    }

    // axial_shaft → coaxial along Z, placed at parent's top face
    if (mount === 'axial_shaft') {
      const oz = parent.hz + childZ / 2 + gap
      return { xyz: `0 0 ${oz.toFixed(4)}`, rpy: '0 0 0' }
    }

    // rail_slot / side_rail_mount → mount on the side (X+) of parent
    if (mount === 'rail_slot' || mount === 'side_rail_mount' || mount === 'clamp_mount') {
      const ox = parent.hx + childX / 2 + gap
      return { xyz: `${ox.toFixed(4)} 0 0`, rpy: '0 0 0' }
    }

    // hub_bore → coaxial, flush with parent face
    if (mount === 'hub_bore') {
      const oz = parent.hz + childZ / 2 + gap
      return { xyz: `0 0 ${oz.toFixed(4)}`, rpy: '0 0 0' }
    }

    // press_fit → inside parent bore, centered
    if (mount === 'press_fit') {
      return { xyz: '0 0 0', rpy: '0 0 0' }
    }

    // linear_rod → extend along Z from parent
    if (mount === 'linear_rod') {
      const oz = parent.hz + childZ / 2 + gap
      return { xyz: `0 0 ${oz.toFixed(4)}`, rpy: '0 0 0' }
    }

    // tool_changer_master/slave → stack on bottom (Z-) if slave
    if (mount === 'tool_changer_slave') {
      const oz = -(parent.hz + childZ / 2 + gap)
      return { xyz: `0 0 ${oz.toFixed(4)}`, rpy: '0 0 0' }
    }
    if (mount === 'tool_changer_master') {
      const oz = parent.hz + childZ / 2 + gap
      return { xyz: `0 0 ${oz.toFixed(4)}`, rpy: '0 0 0' }
    }

    // Default: stack on top
    const oz = parent.hz + childZ / 2 + gap
    return { xyz: `0 0 ${oz.toFixed(4)}`, rpy: '0 0 0' }
  }

  // Resolve which category a component belongs to
  function findCategory(comp: PresetComponent): string {
    if (!presetData) return 'structural'
    for (const [catName, cat] of Object.entries(presetData.categories)) {
      if (cat.components.some(c => c.id === comp.id)) return catName
    }
    return 'structural'
  }

  function addVisualElement(doc: Document, link: Element, vis: UrdfVisualDesc, matIdx: number) {
    const visual = doc.createElement('visual')
    const vo = doc.createElement('origin')
    vo.setAttribute('xyz', vis.origin_xyz.map(v => v.toFixed(6)).join(' '))
    vo.setAttribute('rpy', vis.origin_rpy.map(v => v.toFixed(6)).join(' '))
    const geometry = doc.createElement('geometry')
    const g = vis.geometry
    if (g.type === 'box') {
      const el = doc.createElement('box')
      el.setAttribute('size', g.size.map(v => v.toFixed(6)).join(' '))
      geometry.appendChild(el)
    } else if (g.type === 'cylinder') {
      const el = doc.createElement('cylinder')
      el.setAttribute('radius', g.radius.toFixed(6))
      el.setAttribute('length', g.length.toFixed(6))
      geometry.appendChild(el)
    } else {
      const el = doc.createElement('sphere')
      el.setAttribute('radius', g.radius.toFixed(6))
      geometry.appendChild(el)
    }
    visual.appendChild(vo)
    visual.appendChild(geometry)
    // Material with colour
    const mat = doc.createElement('material')
    mat.setAttribute('name', `comp_mat_${matIdx}`)
    const color = doc.createElement('color')
    color.setAttribute('rgba', vis.color_rgba.map(v => v.toFixed(3)).join(' '))
    mat.appendChild(color)
    visual.appendChild(mat)
    link.appendChild(visual)
  }

  function addComponent(comp: PresetComponent) {
    if (!selectedLink) {
      ctx.showToast('Select a parent link first', 'warning')
      return
    }
    const parentLink = selectedLink
    const graph = ctx.getKinematicGraph()
    const nextIdx = Object.keys(graph).length + 1
    const childName = `${comp.id}_${nextIdx}`
    const jointName = `joint_${comp.id}_${nextIdx}`

    const phys = comp.physical
    const mass = phys.mass_kg ?? phys.mass_kg_per_100mm ?? 0.1
    const bb = phys.bounding_box_mm ?? phys.cross_section_mm ?? [40, 40, 40]
    const shape = phys.inertia_primitive || 'box'

    // Convert mm to meters for URDF
    const xm = (bb[0] ?? 40) / 1000
    const ym = (bb[1] ?? 40) / 1000
    const zm = (bb[2] ?? 40) / 1000

    // Compute inertia (for physics — uses bounding primitive)
    let inertia: { ixx: number; iyy: number; izz: number }
    if (shape === 'cylinder') {
      inertia = computeCylinderInertia(mass, Math.max(xm, ym) / 2, zm)
    } else if (shape === 'sphere') {
      inertia = computeSphereInertia(mass, xm / 2)
    } else {
      inertia = computeBoxInertia(mass, xm, ym, zm)
    }

    // Generate parametric visuals
    const catName = findCategory(comp)
    const visuals = generateVisuals(comp as Parameters<typeof generateVisuals>[0], catName)

    const changed = commitUrdf(doc => {
      const robot = doc.querySelector('robot')
      if (!robot) return false

      // Create link
      const link = doc.createElement('link')
      link.setAttribute('name', childName)

      // Inertial
      const inertialEl = doc.createElement('inertial')
      const massEl = doc.createElement('mass')
      massEl.setAttribute('value', mass.toFixed(4))
      const inertiaEl = doc.createElement('inertia')
      inertiaEl.setAttribute('ixx', inertia.ixx.toFixed(6))
      inertiaEl.setAttribute('iyy', inertia.iyy.toFixed(6))
      inertiaEl.setAttribute('izz', inertia.izz.toFixed(6))
      inertiaEl.setAttribute('ixy', '0')
      inertiaEl.setAttribute('ixz', '0')
      inertiaEl.setAttribute('iyz', '0')
      inertialEl.appendChild(massEl)
      inertialEl.appendChild(inertiaEl)
      link.appendChild(inertialEl)

      // Multiple visual elements from parametric generator
      visuals.forEach((vis, i) => addVisualElement(doc, link, vis, i))

      // Single collision primitive (bounding shape for physics)
      const collision = doc.createElement('collision')
      const co = doc.createElement('origin')
      co.setAttribute('xyz', '0 0 0')
      co.setAttribute('rpy', '0 0 0')
      const collGeom = doc.createElement('geometry')
      if (shape === 'cylinder') {
        const el = doc.createElement('cylinder')
        el.setAttribute('radius', (Math.max(xm, ym) / 2).toFixed(6))
        el.setAttribute('length', zm.toFixed(6))
        collGeom.appendChild(el)
      } else if (shape === 'sphere') {
        const el = doc.createElement('sphere')
        el.setAttribute('radius', (Math.max(xm, ym, zm) / 2).toFixed(6))
        collGeom.appendChild(el)
      } else {
        const el = doc.createElement('box')
        el.setAttribute('size', `${xm.toFixed(6)} ${ym.toFixed(6)} ${zm.toFixed(6)}`)
        collGeom.appendChild(el)
      }
      collision.appendChild(co)
      collision.appendChild(collGeom)
      link.appendChild(collision)

      // Joint
      const joint = doc.createElement('joint')
      joint.setAttribute('name', jointName)
      const category = comp.id.split('_')[0]
      const isActuated = category === 'actuator' || category === 'motor'
      joint.setAttribute('type', isActuated ? 'revolute' : 'fixed')

      const parent = doc.createElement('parent')
      parent.setAttribute('link', parentLink)
      const child = doc.createElement('child')
      child.setAttribute('link', childName)
      // Smart placement based on parent geometry + mounting logic
      const origin = doc.createElement('origin')
      const placement = computePlacement(doc, parentLink, comp, xm, ym, zm)
      origin.setAttribute('xyz', placement.xyz)
      origin.setAttribute('rpy', placement.rpy)
      joint.appendChild(parent)
      joint.appendChild(child)
      joint.appendChild(origin)

      if (isActuated) {
        const axis = doc.createElement('axis')
        axis.setAttribute('xyz', '0 0 1')
        joint.appendChild(axis)
        const limit = doc.createElement('limit')
        limit.setAttribute('lower', '-3.14159')
        limit.setAttribute('upper', '3.14159')
        const maxTorque = (comp.mechanical_electrical.max_torque_nm as number) ??
                          (comp.mechanical_electrical.holding_torque_nm as number) ?? 10
        limit.setAttribute('effort', String(maxTorque))
        limit.setAttribute('velocity', '3.14')
        joint.appendChild(limit)
      }

      robot.appendChild(link)
      robot.appendChild(joint)

      // Attachment face nodes are synthesized on-the-fly from link bounding boxes
      // during rebuildMountNodes(); no persisted mount-frame links are needed here.
      return true
    })

    if (changed) {
      ctx.showToast(`Added ${comp.name} as "${childName}"`, 'success')
      selectLink(childName)
    }
  }

  function getCompactSpec(comp: PresetComponent): string {
    const me = comp.mechanical_electrical
    if (me.max_torque_nm) return `${me.max_torque_nm} Nm`
    if (me.holding_torque_nm) return `${me.holding_torque_nm} Nm`
    if (me.max_force_n) return `${me.max_force_n} N`
    if (me.grip_force_n) return `${me.grip_force_n} N`
    if (me.fov_h_deg) return `${me.fov_h_deg}° FOV`
    if (me.range_m) return `${me.range_m}m range`
    if (me.capacity_mah) return `${me.capacity_mah}mAh`
    if (me.gear_ratio_options) return 'gearbox'
    if (me.max_load_n) return `${me.max_load_n}N`
    const mass = comp.physical.mass_kg ?? comp.physical.mass_kg_per_100mm
    if (mass != null) return `${mass >= 1 ? mass.toFixed(1) : Math.round(mass * 1000)}${mass >= 1 ? 'kg' : 'g'}`
    return ''
  }

  function renderComponentDetail(comp: PresetComponent) {
    if (!compDetail) return
    const mass = comp.physical.mass_kg ?? comp.physical.mass_kg_per_100mm
    const massLabel = comp.physical.mass_kg_per_100mm ? `${(comp.physical.mass_kg_per_100mm * 1000).toFixed(0)}g/100mm` :
                      mass != null ? (mass >= 1 ? `${mass.toFixed(2)} kg` : `${Math.round(mass * 1000)} g`) : '—'
    const bb = comp.physical.bounding_box_mm
    const dims = bb ? `${bb[0]}×${bb[1]}×${bb[2]} mm` : '—'
    const shape = comp.physical.inertia_primitive || 'box'

    // Build mechanical specs
    const me = comp.mechanical_electrical
    const specs = Object.entries(me).slice(0, 6).map(([k, v]) => {
      const label = k.replace(/_/g, ' ').replace(/\b\w/g, c => c.toUpperCase())
      const val = Array.isArray(v) ? v.join(' – ') : String(v)
      return `<div class="tb-kv"><span class="tb-kv-key">${label}</span><span class="tb-kv-val">${val}</span></div>`
    }).join('')

    const mounting = comp.mounting_logic.primary ?? '—'

    compDetail.innerHTML = `
      <div class="tb-detail-name">${comp.name}</div>
      <div class="tb-detail-desc">${comp.description}</div>
      <div class="tb-detail-chiprow">
        <span class="tb-chip">${shape}</span>
        <span class="tb-chip">${massLabel}</span>
        <span class="tb-chip">${mounting}</span>
      </div>
      <div class="tb-detail-sec">
        <div class="tb-detail-sec-title">Dimensions</div>
        <div class="tb-kv"><span class="tb-kv-key">Bounding Box</span><span class="tb-kv-val">${dims}</span></div>
      </div>
      <div class="tb-detail-sec">
        <div class="tb-detail-sec-title">Specs</div>
        ${specs}
      </div>
    `
  }

  function renderComponents(filter: string) {
    if (!compItems || !presetData) return
    const q = filter.trim().toLowerCase()
    compItems.innerHTML = ''

    for (const [catName, cat] of Object.entries(presetData.categories)) {
      const comps = cat.components.filter(c => {
        // Only show components that have real mesh files
        if (!hasMeshOverride(c.id)) return false
        // Exclude components whose meshes are too large/slow to load
        if (SLOW_MESH_BLACKLIST.has(c.id)) return false
        // Apply search filter
        return !q || c.name.toLowerCase().includes(q) || c.id.toLowerCase().includes(q) || c.description.toLowerCase().includes(q)
      })
      if (comps.length === 0) continue

      // Category header
      const catLabel = catName.replace(/_/g, ' ').replace(/\b\w/g, c => c.toUpperCase())
      const catEl = document.createElement('div')
      catEl.className = 'tb-cat'
      catEl.innerHTML = `<span class="tb-cat-arrow">▾</span> ${catLabel} <span style="opacity:0.4;margin-left:auto;font-size:10px">${comps.length}</span>`
      let collapsed = false
      catEl.addEventListener('click', () => {
        collapsed = !collapsed
        listEl.classList.toggle('collapsed', collapsed)
        catEl.querySelector('.tb-cat-arrow')!.textContent = collapsed ? '▸' : '▾'
      })
      compItems.appendChild(catEl)

      const listEl = document.createElement('div')
      listEl.className = 'tb-list'
      const cc = CATEGORY_COLORS[catName] ?? [0.6, 0.6, 0.6, 1]
      const dotColor = `rgb(${Math.round(cc[0]*255)},${Math.round(cc[1]*255)},${Math.round(cc[2]*255)})`
      for (const comp of comps) {
        const el = document.createElement('div')
        el.className = 'tb-item'
        const spec = getCompactSpec(comp)
        el.innerHTML = `
          <span class="tb-cat-dot" style="background:${dotColor}"></span>
          <div class="tb-item-info">
            <div class="tb-item-name">${comp.name}</div>
            <div class="tb-item-meta">${spec}</div>
          </div>
        `
        el.addEventListener('click', () => {
          // Show detail
          renderComponentDetail(comp)
          // Highlight
          compItems!.querySelectorAll('.tb-item').forEach(i => i.classList.remove('selected'))
          el.classList.add('selected')
          // Insert into assembly
          addComponent(comp)
        })
        listEl.appendChild(el)
      }
      compItems.appendChild(listEl)
    }

    if (compItems.children.length === 0) {
      compItems.innerHTML = '<div class="tb-empty">No matching components</div>'
    }
  }

  // Load presets JSON from public directory
  fetch('/generic_presets.json')
    .then(r => r.ok ? r.json() : Promise.reject(r.status))
    .then((data: PresetData) => {
      presetData = data
      renderComponents('')
    })
    .catch(() => {
      if (compItems) compItems.innerHTML = '<div class="tb-empty">Failed to load component presets</div>'
    })

  btnFocusBase?.addEventListener('click', () => {
    const graph = ctx.getKinematicGraph()
    const base = Object.values(graph).find(l => !l.parent)?.name || 'base_link'
    selectLink(base)
  })
  btnResetRobot?.addEventListener('click', () => {
    if (!confirm('Reset robot to a minimal base_link URDF?')) return
    recordUndo()
    const empty = `<?xml version="1.0"?><robot name="robot"><link name="base_link"><inertial><mass value="0.1"/><inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/></inertial><visual><geometry><box size="0.1 0.1 0.05"/></geometry></visual></link></robot>`
    ctx.setUrdfText(empty)
    ctx.reparseUrdf()
    selectLink('base_link')
    ctx.showToast('Robot reset', 'info')
  })
  // ── STL Export ─────────────────────────────────────────────────────────────

  const stlExporter = new STLExporter()

  async function saveStlToFile(group: THREE.Object3D, defaultName: string) {
    try {
      const path = await invoke<string | null>('save_file_dialog', { default_name: defaultName })
      if (!path) return

      // Export as ASCII STL (text-based, works with save_file)
      const stlString = stlExporter.parse(group, { binary: false }) as string
      await invoke('save_file', { path, content: stlString })

      const filename = path.split(/[\\/]/).pop() || defaultName
      ctx.showToast(`Exported ${filename}`, 'success')
    } catch (err) {
      ctx.showToast(`Export failed: ${err}`, 'error')
    }
  }

  async function exportSelectedLinkSTL() {
    if (!selectedLink) {
      ctx.showToast('Select a link to export', 'warning')
      return
    }
    const linkGroup = ctx.getParsedRobot().linkGroups.get(selectedLink)
    if (!linkGroup) {
      ctx.showToast('Link geometry not found', 'error')
      return
    }
    await saveStlToFile(linkGroup, `${selectedLink}.stl`)
  }

  async function exportFullRobotSTL() {
    const robot = ctx.getParsedRobot()
    await saveStlToFile(robot.group, 'robot.stl')
  }

  // Wire export buttons
  const btnExportStl = document.getElementById('btn-export-stl')
  btnExportStl?.addEventListener('click', exportFullRobotSTL)
  toolboxSearch?.addEventListener('input', () => renderComponents(toolboxSearch.value))

  gizmo.addEventListener('dragging-changed', ev => {
    const on = Boolean((ev as unknown as { value: boolean }).value)
    ctx.controls.enabled = !on
    if (on && selectedLink) {
      const pivot = getPivotGroupForLink(selectedLink)
      if (pivot) {
        pivot.updateMatrixWorld(true)
        gizmoBasePivotWorld.copy(pivot.matrixWorld)
        rebuildMountNodes()
        nodesGroup.visible = true
        updateBestCandidateDuringDrag(pivot)
      }
      return
    }
    if (!on && selectedLink) {
      // Drag ended — hide all connection nodes regardless of snap outcome.
      nodesGroup.visible = false
      const parentJoint = getParentJointForLink(selectedLink)
      const pivot = getPivotGroupForLink(selectedLink)
      if (!parentJoint || !pivot) {
        if (!rootDragWarned) {
          ctx.showToast('Root movement disabled (no world joint yet)', 'warning')
          rootDragWarned = true
        }
        selectLink(selectedLink)
        return
      }

      // If we had a valid snap candidate, commit as a fixed joint reparent onto the target link.
      if (bestMountCandidate?.mountLink && bestMountCandidate.reason === 'ok') {
        const targetMount = bestMountCandidate.mountLink
        const targetParent = bestMountCandidate.targetParentLink
        const targetOccupied = isMountOccupied(targetMount)
        if (targetOccupied) {
          ctx.showToast('Snap target is occupied', 'warning')
          bestMountCandidate = null
          ghostGroup.visible = false
          clearBestCandidateHighlight()
          selectLink(selectedLink)
          return
        }
        const graph = ctx.getKinematicGraph()
        // topology / cycle check: don't attach into own subtree
        const stack = [selectedLink]
        const seen = new Set<string>()
        let cycle = false
        while (stack.length) {
          const cur = stack.pop()!
          if (cur === targetParent) { cycle = true; break }
          if (seen.has(cur)) continue
          seen.add(cur)
          for (const ch of graph[cur]?.children ?? []) stack.push(ch)
        }
        if (cycle) {
          ctx.showToast('Invalid snap (would create cycle)', 'warning')
          bestMountCandidate = null
          ghostGroup.visible = false
          clearBestCandidateHighlight()
          selectLink(selectedLink)
          return
        }

        pivot.updateMatrixWorld(true)
        const parentLinkGroup = ctx.getParsedRobot().linkGroups.get(targetParent)
        if (!parentLinkGroup) {
          ctx.showToast('Snap target not found in scene', 'warning')
          bestMountCandidate = null
          ghostGroup.visible = false
          clearBestCandidateHighlight()
          selectLink(selectedLink)
          return
        }
        parentLinkGroup.updateMatrixWorld(true)
        const parentWorldInv = parentLinkGroup.matrixWorld.clone().invert()
        // Compute selected-link frame pose in target parent frame from solved preview transform.
        const childLocalInParent = parentWorldInv.multiply(bestMountCandidate.desiredLinkWorld.clone())
        const newLocalPos = new THREE.Vector3().setFromMatrixPosition(childLocalInParent)
        const newLocalQuat = new THREE.Quaternion()
        childLocalInParent.decompose(new THREE.Vector3(), newLocalQuat, new THREE.Vector3())
        const newLocalEuler = new THREE.Euler().setFromQuaternion(newLocalQuat, 'XYZ')

        const ok = commitUrdf(documentXml => {
          const jointEl = documentXml.querySelector(`joint[name="${parentJoint.name}"]`)
          if (!jointEl) return false
          jointEl.setAttribute('type', 'fixed')
          const pEl = jointEl.querySelector('parent')
          if (!pEl) return false
          pEl.setAttribute('link', targetParent)
          const origin = ensureOrigin(jointEl, documentXml)
          origin.setAttribute('xyz', `${fmt(newLocalPos.x)} ${fmt(newLocalPos.y)} ${fmt(newLocalPos.z)}`)
          origin.setAttribute('rpy', `${fmt(newLocalEuler.x)} ${fmt(newLocalEuler.y)} ${fmt(newLocalEuler.z)}`)
          return true
        })
        bestMountCandidate = null
        ghostGroup.visible = false
        clearBestCandidateHighlight()
        if (ok) ctx.showToast(`Connected ${selectedLink} to ${targetParent}`, 'success')
        selectLink(selectedLink)
        return
      }
      if (bestMountCandidate?.reason && bestMountCandidate.reason !== 'ok') {
        const reasonLabel = bestMountCandidate.reason === 'occupied' ? 'target occupied'
          : bestMountCandidate.reason === 'cycle' ? 'would create cycle'
          : bestMountCandidate.reason === 'too-far' ? 'too far'
          : bestMountCandidate.reason === 'orientation' ? 'orientation mismatch'
          : bestMountCandidate.reason === 'type-mismatch' ? 'incompatible node types'
          : bestMountCandidate.reason === 'same-component' ? 'same component'
          : bestMountCandidate.reason
        ctx.showToast(`Snap rejected: ${reasonLabel}`, 'warning')
      }
      ghostGroup.visible = false
      clearBestCandidateHighlight()

      if (gizmo.mode !== 'translate') {
        ctx.showToast('Rotate persistence is not enabled yet', 'info')
        selectLink(selectedLink)
        return
      }

      pivot.updateMatrixWorld(true)
      const parentObj = pivot.parent
      if (!parentObj) {
        selectLink(selectedLink)
        return
      }
      parentObj.updateMatrixWorld(true)

      const parentWorldInv = parentObj.matrixWorld.clone().invert()
      const childLocal = parentWorldInv.multiply(pivot.matrixWorld.clone())

      const newLocalPos = new THREE.Vector3().setFromMatrixPosition(childLocal)

      const parser = new DOMParser()
      const doc = parser.parseFromString(ctx.getUrdfText(), 'application/xml')
      if (doc.documentElement.nodeName === 'parsererror') {
        selectLink(selectedLink)
        return
      }
      const j = doc.querySelector(`joint[name="${parentJoint.name}"]`)
      if (!j) {
        selectLink(selectedLink)
        return
      }
      const o = ensureOrigin(j, doc)
      const oldLocalPos = parseNums(o.getAttribute('xyz') || '0 0 0', 3)
      const oldPosVec = new THREE.Vector3(oldLocalPos[0], oldLocalPos[1], oldLocalPos[2])
      if (oldPosVec.distanceToSquared(newLocalPos) < 1e-12) {
        selectLink(selectedLink)
        return
      }

      const ok = commitUrdf(documentXml => {
        const jointEl = documentXml.querySelector(`joint[name="${parentJoint.name}"]`)
        if (!jointEl) return false
        const origin = ensureOrigin(jointEl, documentXml)
        origin.setAttribute('xyz', `${fmt(newLocalPos.x)} ${fmt(newLocalPos.y)} ${fmt(newLocalPos.z)}`)
        return true
      })
      if (ok) ctx.showToast(`Moved ${selectedLink} (joint origin updated)`, 'success')
      selectLink(selectedLink)
    }
  })

  gizmo.addEventListener('change', () => {
    if (!selectedLink) return
    if (!ctx.isViewport3D()) return
    const pivot = getPivotGroupForLink(selectedLink)
    if (!pivot) return
    updateBestCandidateDuringDrag(pivot)
  })

  ctx.canvas.addEventListener('pointerdown', e => {
    pointerDown.set(e.clientX, e.clientY)
  })

  ctx.canvas.addEventListener('click', e => {
    if (!ctx.isViewport3D()) return
    if (Math.abs(e.clientX - pointerDown.x) > 8 || Math.abs(e.clientY - pointerDown.y) > 8) return
    const ray = makeRaycaster(e)
    const hits = ray.intersectObjects(getPickTargets(), false)
    if (hits.length === 0) {
      selectLink(null)
      return
    }
    const first = hits[0].object
    const link = (first.userData as Record<string, unknown>).urdfLinkName
    if (typeof link === 'string' && link) {
      selectLink(link)
    }
  })

  ctx.canvas.addEventListener('mousemove', e => {
    if (!ctx.isViewport3D()) return
    const ray = makeRaycaster(e)
    const hits = ray.intersectObjects(getPickTargets(), false)
    ctx.canvas.style.cursor = hits.length > 0 ? 'pointer' : ''
  })

  document.addEventListener('keydown', e => {
    const inTyping = isTypingTarget(e.target)
    if (inTyping) return
    if (!ctx.isViewport3D()) return
    const k = e.key.toLowerCase()

    if ((e.ctrlKey || e.metaKey) && k === 'z' && !e.shiftKey) {
      e.preventDefault()
      const prev = urdfUndo.pop()
      if (!prev) return
      urdfRedo.push(ctx.getUrdfText())
      ctx.setUrdfText(prev)
      ctx.reparseUrdf()
      ctx.showToast('Undo', 'info')
      return
    }
    if ((e.ctrlKey || e.metaKey) && (k === 'y' || (k === 'z' && e.shiftKey))) {
      e.preventDefault()
      const next = urdfRedo.pop()
      if (!next) return
      urdfUndo.push(ctx.getUrdfText())
      ctx.setUrdfText(next)
      ctx.reparseUrdf()
      ctx.showToast('Redo', 'info')
      return
    }
    if (k === 'i') { ctx.switchPanel('inspector'); return }
    if (k === 't') { ctx.switchPanel('toolbox'); return }
    if (k === 'r' && selectedLink && gizmo.object) {
      gizmo.setMode(gizmo.mode === 'translate' ? 'rotate' : 'translate')
      ctx.showToast(`Gizmo: ${gizmo.mode}`, 'info')
      return
    }
    if ((k === 'delete' || k === 'backspace') && selectedLink) {
      e.preventDefault()
      deleteLink(selectedLink)
      return
    }
  })

  function onModelUpdated() {
    if (selectedLink) {
      if (!ctx.getParsedRobot().linkGroups.has(selectedLink)) {
        selectedLink = null
        gizmo.detach()
      } else {
        const pivot = getPivotGroupForLink(selectedLink)
        if (pivot) {
          gizmo.attach(pivot)
          pivot.updateMatrixWorld(true)
          gizmoBasePivotWorld.copy(pivot.matrixWorld)
        } else {
          gizmo.detach()
        }
      }
    }
    ghostGroup.visible = false
    bestMountCandidate = null
    rebuildMountNodes()
    refreshBuildPanel()
    renderInspector()
  }

  refreshBuildPanel()
  renderInspector()

  return {
    onModelUpdated,
    recordUndoExternal: (content: string) => {
      urdfUndo.push(content)
      if (urdfUndo.length > 80) urdfUndo.shift()
      urdfRedo = []
    },
  }
}

