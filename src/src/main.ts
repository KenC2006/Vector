import './style.css'
import * as THREE from 'three'
import { OrbitControls } from 'three/addons/controls/OrbitControls.js'
import { TransformControls } from 'three/addons/controls/TransformControls.js'
import { AssemblyGraph } from './assemblyGraph'
import { AssemblyRenderer } from './assemblyRenderer'
import { AssemblyHistory } from './history'
import { generateURDF, generateMJCF } from './urdfGenerator'
import { getPartDef, defaultParams, interfacesCompatible } from './partLibrary'
import { initToolbox, clearToolboxSelection } from './toolbox'
import { initBuildInspector, showBuildInspectorFor, hideBuildInspector } from './buildInspector'

// ── Scene ─────────────────────────────────────────────────────────────────────

const canvas        = document.getElementById('viewport')      as HTMLCanvasElement
const viewportPanel = document.getElementById('viewport-panel') as HTMLDivElement

const renderer = new THREE.WebGLRenderer({ canvas, antialias: true })
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2))
renderer.setClearColor(0x111827)
renderer.shadowMap.enabled   = true
renderer.shadowMap.type      = THREE.PCFSoftShadowMap
renderer.toneMapping         = THREE.ACESFilmicToneMapping
renderer.toneMappingExposure = 1.25

const scene = new THREE.Scene()
scene.fog   = new THREE.FogExp2(0x111827, 0.06)

const camera = new THREE.PerspectiveCamera(50, 1, 0.01, 100)
camera.position.set(1.2, 1.0, 1.6)

const controls = new OrbitControls(camera, canvas)
controls.enableDamping = true
controls.dampingFactor = 0.07
controls.target.set(0, 0.3, 0)
controls.minDistance = 0.2
controls.maxDistance = 8

// ── Transform gizmo (move tool) ───────────────────────────────────────────────

const gizmo = new TransformControls(camera, canvas)
gizmo.setMode('translate')
gizmo.setSpace('world')
gizmo.setSize(0.8)
// TransformControls is not an Object3D — add the helper root (see three.js docs).
scene.add(gizmo.getHelper())

// Disable orbit while dragging the gizmo, re-enable after
gizmo.addEventListener('dragging-changed', (event: any) => {
  controls.enabled = !event.value
  if (event.value) {
    // Drag started — show all interface nodes
    if (selectedId) aRenderer.beginDrag(selectedId)
  } else {
    // Drag ended — snap-connect or commit offset
    commitGizmoDrag()
    aRenderer.endDrag()
  }
})

// Live snap preview + collision feedback while gizmo is dragging
gizmo.addEventListener('objectChange', () => {
  if (!selectedId) return
  aRenderer.updateDragPreview(selectedId, 0.08)
  const collisions = aRenderer.checkCollisions(selectedId)
  aRenderer.highlightCollisions(collisions)
})

// ── Ground / grid / axes ──────────────────────────────────────────────────────
const grid = new THREE.GridHelper(8, 40, 0x2a3040, 0x1e2535)
scene.add(grid)

const ground = new THREE.Mesh(
  new THREE.PlaneGeometry(8, 8),
  new THREE.ShadowMaterial({ opacity: 0.2 }),
)
ground.rotation.x = -Math.PI / 2
ground.receiveShadow = true
scene.add(ground)

const originAxes = new THREE.AxesHelper(0.5)
scene.add(originAxes)

// Lights — generous ambient so parts are legible on a dark background
scene.add(new THREE.AmbientLight(0xccd4e8, 1.1))

const keyLight = new THREE.DirectionalLight(0xffffff, 1.6)
keyLight.position.set(3, 6, 4)
keyLight.castShadow = true
keyLight.shadow.mapSize.set(2048, 2048)
keyLight.shadow.camera.near   = 0.5
keyLight.shadow.camera.far    = 20
keyLight.shadow.camera.left   = -3
keyLight.shadow.camera.right  = 3
keyLight.shadow.camera.top    = 3
keyLight.shadow.camera.bottom = -3
keyLight.shadow.bias          = -0.0005
scene.add(keyLight)

const fillLight = new THREE.DirectionalLight(0x88aadd, 0.7)
fillLight.position.set(-3, 2, -2)
scene.add(fillLight)

const rimLight = new THREE.DirectionalLight(0xaabbff, 0.45)
rimLight.position.set(0, 0.5, -4)
scene.add(rimLight)

// ── Core data + renderer ──────────────────────────────────────────────────────

const assembly  = new AssemblyGraph()
const aRenderer = new AssemblyRenderer(scene)
aRenderer.bind(assembly)
const history = new AssemblyHistory(assembly)

// ── Resize + Animate ──────────────────────────────────────────────────────────

function resize() {
  const header = document.getElementById('viewport-header')!
  const w = viewportPanel.clientWidth
  const h = viewportPanel.clientHeight - header.offsetHeight
  if (w > 0 && h > 0) {
    renderer.setSize(w, h)
    camera.aspect = w / h
    camera.updateProjectionMatrix()
  }
}
resize()
window.addEventListener('resize', resize)

function animate() {
  requestAnimationFrame(animate)
  controls.update()
  renderer.render(scene, camera)
}
animate()

// ── Panels ────────────────────────────────────────────────────────────────────

const panels: Record<string, HTMLElement> = {
  build:     document.getElementById('panel-build')!,
  inspector: document.getElementById('panel-inspector')!,
  toolbox:   document.getElementById('panel-toolbox')!,
}

function switchToPanel(name: string) {
  document.querySelectorAll('.ab-btn').forEach(b => b.classList.remove('active'))
  Object.values(panels).forEach(p => p.classList.add('hidden'))
  const btn = document.querySelector(`.ab-btn[data-panel="${name}"]`) as HTMLElement | null
  if (btn) btn.classList.add('active')
  if (panels[name]) panels[name].classList.remove('hidden')
}

// ── Build hint overlay ────────────────────────────────────────────────────────

function updateHint(msg: string | null) {
  const overlay = document.getElementById('build-hint-overlay')
  const text    = document.getElementById('build-hint-text')
  if (!overlay) return
  overlay.classList.toggle('hidden', !msg)
  if (msg && text) text.textContent = msg
}

// ── Toast ─────────────────────────────────────────────────────────────────────

function showToast(msg: string, type: 'success' | 'info' | 'warning' = 'info') {
  const area = document.getElementById('toast-area')
  if (!area) return
  const t = document.createElement('div')
  t.className = `toast toast-${type}`
  t.textContent = msg
  area.appendChild(t)
  requestAnimationFrame(() => t.classList.add('visible'))
  setTimeout(() => { t.classList.remove('visible'); setTimeout(() => t.remove(), 300) }, 2500)
}

// ── Camera focus ──────────────────────────────────────────────────────────────

let _focusTimer: number | null = null

function focusOn(instanceId: string) {
  const bb = aRenderer.getBoundingBox(instanceId)
  if (!bb) return
  const center = new THREE.Vector3()
  bb.getCenter(center)
  const size   = bb.getSize(new THREE.Vector3()).length()
  const dist   = Math.max(0.4, size * 2.5)
  const endPos = new THREE.Vector3(center.x + dist * 0.6, center.y + dist * 0.4, center.z + dist * 0.8)

  const startPos = camera.position.clone()
  const startTgt = controls.target.clone()
  let t = 0
  if (_focusTimer !== null) clearInterval(_focusTimer)
  _focusTimer = setInterval(() => {
    t = Math.min(t + 0.07, 1)
    const e = 1 - (1 - t) ** 3
    camera.position.lerpVectors(startPos, endPos, e)
    controls.target.lerpVectors(startTgt, center, e)
    controls.update()
    if (t >= 1) { clearInterval(_focusTimer!); _focusTimer = null }
  }, 16) as unknown as number
}

// ── Build state ───────────────────────────────────────────────────────────────

let pendingDefId: string | null = null   // part selected in toolbox, waiting to be placed/connected
let selectedId:   string | null = null   // currently selected instance

// Gizmo drag tracking
let _gizmoBasePos  = new THREE.Vector3()     // group position when gizmo attached
let _gizmoBaseQuat = new THREE.Quaternion()  // group quaternion when gizmo attached

function syncGizmoForMode() {
  // Build mode (pending part) and move mode (gizmo) are mutually exclusive.
  const inBuildMode = !!pendingDefId
  gizmo.enabled = !inBuildMode
  if (inBuildMode) {
    gizmo.detach()
    return
  }
  if (selectedId) {
    const grp = aRenderer.getMeshGroup(selectedId)
    if (grp) {
      gizmo.attach(grp)
      _gizmoBasePos.copy(grp.position)
      _gizmoBaseQuat.copy(grp.quaternion)
    } else {
      gizmo.detach()
    }
  } else {
    gizmo.detach()
  }
}

function commitGizmoDrag() {
  if (!selectedId) return
  const grp = aRenderer.getMeshGroup(selectedId)
  if (!grp) return

  const inst = assembly.getInstance(selectedId)

  // Compute deltas for both position and rotation
  const deltaPos  = grp.position.clone().sub(_gizmoBasePos)
  const deltaQuat = grp.quaternion.clone().multiply(_gizmoBaseQuat.clone().invert())

  const posMoved = deltaPos.lengthSq() > 1e-10
  const rotMoved = 1 - Math.abs(deltaQuat.dot(new THREE.Quaternion())) > 1e-8

  if (!posMoved && !rotMoved) return  // nothing changed, skip

  // Detach BEFORE rebuild: TransformControls requires the object to stay in the scene
  // graph. rebuildInstance() removes the mesh — if gizmo still references it, the mesh
  // becomes parent=null (orphaned), stops rendering, and the controls spam console errors.
  gizmo.detach()

  history.record()

  // ── Commit position delta ────────────────────────────────────────────────
  if (posMoved) {
    const prev = inst?.dragOffset ?? { x: 0, y: 0, z: 0 }
    assembly.setDragOffset(
      selectedId,
      prev.x + deltaPos.x,
      prev.y + deltaPos.y,
      prev.z + deltaPos.z,
    )
  }

  // ── Commit rotation delta ────────────────────────────────────────────────
  if (rotMoved) {
    const prev = inst?.dragRotation ?? { x: 0, y: 0, z: 0, w: 1 }
    const prevQuat = new THREE.Quaternion(prev.x, prev.y, prev.z, prev.w)
    let newQuat: THREE.Quaternion
    if (gizmo.space === 'world') {
      // World-space: premultiply  →  result = delta * prev
      newQuat = deltaQuat.clone().multiply(prevQuat)
    } else {
      // Local-space: postmultiply →  result = prev * delta
      newQuat = prevQuat.clone().multiply(deltaQuat)
    }
    newQuat.normalize()
    assembly.setDragRotation(selectedId, {
      x: newQuat.x, y: newQuat.y, z: newQuat.z, w: newQuat.w,
    })
  }

  // ── Snap-to-connect: check if an interface is close to a compatible target ──
  const snap = aRenderer.findSnapTarget(selectedId, 0.08)
  if (snap) {
    const childDef  = getPartDef(inst!.definitionId)
    const cIface    = childDef?.interfaces.find(i => i.id === snap.draggedIfaceId)
    const ok = assembly.reparent(
      selectedId,
      snap.targetInstanceId,
      snap.targetIfaceId,
      snap.draggedIfaceId,
      { type: cIface?.defaultJointType ?? 'fixed' },
    )
    if (ok) {
      showToast('Snapped & connected', 'success')
      refreshBuildPanel()
    }
  }

  const newGrp = aRenderer.getMeshGroup(selectedId)
  if (newGrp) {
    gizmo.attach(newGrp)
    _gizmoBasePos.copy(newGrp.position)
    _gizmoBaseQuat.copy(newGrp.quaternion)
  }
}

/** Select a part (or deselect with null). Updates inspector + panel. */
function select(instanceId: string | null) {
  selectedId = instanceId
  aRenderer.selectInstance(instanceId)

  if (instanceId && assembly.getInstance(instanceId)) {
    if (!pendingDefId) {
      const grp = aRenderer.getMeshGroup(instanceId)
      if (grp) {
        gizmo.attach(grp)
        _gizmoBasePos.copy(grp.position)
        _gizmoBaseQuat.copy(grp.quaternion)
      }
    }
    showBuildInspectorFor(instanceId)
    switchToPanel('inspector')
    // No automatic camera recenter here; use `F` to focus explicitly.
  } else {
    gizmo.detach()
    hideBuildInspector()
  }
  refreshBuildPanel()
}

/** Activate a pending part from the toolbox, or clear it. */
function setPending(defId: string | null) {
  pendingDefId = defId
  aRenderer.setPendingPart(defId)
  syncGizmoForMode()
  if (!defId) { updateHint(null); return }

  const def = getPartDef(defId)
  updateHint(
    assembly.isEmpty()
      ? `Click the viewport to place ${def?.name ?? defId} as the root`
      : `Click a glowing ring (○) to connect ${def?.name ?? defId}`,
  )
}

/** Delete a part (with confirmation). */
function deletePart(instanceId: string) {
  const inst  = assembly.getInstance(instanceId)
  const label = inst?.label ?? instanceId
  if (!confirm(`Remove "${label}"?`)) return
  if (selectedId === instanceId) {
    gizmo.detach()
    selectedId = null
    aRenderer.selectInstance(null)
    hideBuildInspector()
  }
  history.record()
  assembly.removePart(instanceId)
  refreshBuildPanel()
}

// ── Build panel tree ──────────────────────────────────────────────────────────

function refreshBuildPanel() {
  const emptyDiv = document.getElementById('build-empty')!
  const treeDiv  = document.getElementById('build-tree')!
  const parts    = assembly.size()
  const massG    = Math.round(assembly.totalMass() * 1000)
  const joints   = Math.max(0, parts - 1)

  ;(document.getElementById('bs-parts')  as HTMLElement).textContent = String(parts)
  ;(document.getElementById('bs-mass')   as HTMLElement).textContent = `${massG} g`
  ;(document.getElementById('bs-joints') as HTMLElement).textContent = String(joints)

  emptyDiv.classList.toggle('hidden', parts > 0)
  treeDiv.innerHTML = ''

  assembly.walk((inst, parentConn, depth) => {
    const def        = getPartDef(inst.definitionId)
    const massGPart  = def ? Math.round(def.mass(inst.params) * 1000) : 0
    const jointBadge = parentConn ? `[${parentConn.joint.type.slice(0, 3)}]` : '[root]'

    const row = document.createElement('div')
    row.className = 'bt-row' + (inst.instanceId === selectedId ? ' selected' : '')
    row.style.paddingLeft = `${8 + depth * 14}px`
    row.innerHTML = `
      <span class="bt-joint-badge">${jointBadge}</span>
      <span class="bt-name">${inst.label}</span>
      <span class="bt-mass">${massGPart}g</span>
      <button class="bt-remove" title="Remove">✕</button>
    `
    row.addEventListener('click', () => select(inst.instanceId))
    row.querySelector('.bt-remove')?.addEventListener('click', ev => {
      ev.stopPropagation()
      deletePart(inst.instanceId)
    })
    treeDiv.appendChild(row)
  })
}

assembly.on(ev => {
  if (['instance_added', 'instance_removed', 'cleared', 'restored'].includes(ev.type)) {
    refreshBuildPanel()
  }
})

// ── Inspector & Toolbox init ──────────────────────────────────────────────────

initBuildInspector(assembly, {
  onParamChange:  (id, p)     => { history.record(); assembly.updateParams(id, p) },
  onJointChange:  (cid, j)    => { history.record(); assembly.updateJoint(cid, j) },
  onJointValue:   (cid, v)    => assembly.setJointValue(cid, v),
  onDelete:       id          => deletePart(id),
  onFocus:        id          => focusOn(id),
  onDuplicate:    id          => {
    const inst = assembly.getInstance(id)
    const conn = assembly.getParentConnection(id)
    if (!inst || !conn) { showToast('Cannot duplicate root', 'warning'); return }
    history.record()
    const newId = assembly.addPart(
      inst.definitionId, { ...inst.params },
      conn.parentInstanceId, conn.parentInterfaceId, conn.childInterfaceId,
      { ...conn.joint }, `${inst.label} (copy)`,
    )
    select(newId)
    showToast('Duplicated', 'success')
  },
  onLabelChange:  (id, label) => { assembly.setLabel(id, label); refreshBuildPanel() },
})

initToolbox(defId => setPending(defId))

// ── Raycaster helper ──────────────────────────────────────────────────────────

function makeRaycaster(e: { clientX: number; clientY: number }): THREE.Raycaster {
  const rect = canvas.getBoundingClientRect()
  const ndc  = new THREE.Vector2(
    ((e.clientX - rect.left) / rect.width)  *  2 - 1,
    -((e.clientY - rect.top)  / rect.height) *  2 + 1,
  )
  const ray = new THREE.Raycaster()
  ray.setFromCamera(ndc, camera)
  return ray
}

// ── Click detection (drag guard) ──────────────────────────────────────────────
// OrbitControls uses the same canvas for drag. We track pointer-down position
// so we can ignore the click event if the user actually dragged the view.

let _downX = 0
let _downY = 0
let _downOnGizmo = false   // was the pointerdown over a gizmo handle?

canvas.addEventListener('pointerdown', e => {
  _downX = e.clientX
  _downY = e.clientY
  if (pendingDefId) {
    _downOnGizmo = false
    return
  }
  // gizmo.axis is non-null when the cursor is hovering over a handle
  _downOnGizmo = (gizmo.object !== undefined && gizmo.axis !== null)
})

canvas.addEventListener('click', (e: MouseEvent) => {
  // Ignore drags (pointer moved > 5 px) and gizmo handle interactions
  if (Math.abs(e.clientX - _downX) > 5 || Math.abs(e.clientY - _downY) > 5) return
  if (_downOnGizmo) return

  const hit = aRenderer.raycast(makeRaycaster(e))

  // ── Empty click ───────────────────────────────────────────────────────────
  if (!hit) {
    if (pendingDefId && assembly.isEmpty()) {
      // Place root part at the world origin
      const def = getPartDef(pendingDefId)!
      history.record()
      const pid = assembly.addRoot(pendingDefId, defaultParams(def))
      select(pid)
      showToast(`Placed ${def.name} — click a ring (○) to connect more parts`, 'success')
      updateHint(`Click a glowing ring (○) to connect another part`)
    } else {
      select(null)
    }
    return
  }

  // ── Part body click ───────────────────────────────────────────────────────
  if (hit.type === 'instance') {
    // If a part is pending, treat clicking the body as a "snap to nearest
    // compatible node on this part" instead of requiring a pixel-perfect ring hit.
    if (pendingDefId) {
      const childDef   = getPartDef(pendingDefId)
      const parentInst = assembly.getInstance(hit.instanceId)
      const parentDef  = parentInst ? getPartDef(parentInst.definitionId) : null

      if (childDef && parentDef && parentInst) {
        // Pick the first compatible, unoccupied interface on this instance.
        const occupied = assembly.occupiedInterfaces(hit.instanceId)
        const pIface = parentDef.interfaces.find(iface => {
          if (occupied.has(iface.id)) return false
          return childDef.interfaces.some(ci => interfacesCompatible(ci.type, iface.type))
        })

        if (pIface) {
          const cIface = childDef.interfaces.find(ci => interfacesCompatible(ci.type, pIface.type))!

          aRenderer.clearGhost()
          history.record()
          const newId = assembly.addPart(
            pendingDefId, defaultParams(childDef),
            hit.instanceId, pIface.id, cIface.id,
            { type: cIface.defaultJointType },
          )
          select(newId)
          showToast(`Connected ${childDef.name}`, 'success')
          updateHint(`Click another part or ring (○) to keep building, or Escape to stop`)
          return
        }
      }
    }

    // No pending part or no compatible node → just select the instance.
    select(hit.instanceId)
    return
  }

  // ── Interface ring click → connect pending part ───────────────────────────
  if (hit.type === 'interface') {
    if (!pendingDefId) {
      showToast('Pick a part from the Toolbox first', 'info')
      return
    }

    const childDef   = getPartDef(pendingDefId)!
    const parentInst = assembly.getInstance(hit.instanceId)!
    const parentDef  = getPartDef(parentInst.definitionId)!
    const pIface     = parentDef.interfaces.find(i => i.id === hit.interfaceId)
    if (!pIface) return

    const cIface = childDef.interfaces.find(ci => interfacesCompatible(ci.type, pIface.type))
    if (!cIface) {
      showToast(`${childDef.name} has no compatible interface here`, 'warning')
      return
    }

    aRenderer.clearGhost()
    history.record()
    const newId = assembly.addPart(
      pendingDefId, defaultParams(childDef),
      hit.instanceId, hit.interfaceId!, cIface.id,
      { type: cIface.defaultJointType },
    )
    select(newId)
    showToast(`Connected ${childDef.name}`, 'success')
    updateHint(`Click another ring (○) to keep building, or Escape to stop`)
  }
})

// ── Hover: ghost preview ──────────────────────────────────────────────────────

canvas.addEventListener('mousemove', (e: MouseEvent) => {
  if (!pendingDefId) { aRenderer.clearGhost(); canvas.style.cursor = ''; return }

  const hit = aRenderer.raycast(makeRaycaster(e))

  if (hit?.type === 'interface' && hit.interfaceId) {
    const childDef   = getPartDef(pendingDefId)
    const parentInst = assembly.getInstance(hit.instanceId)
    const parentDef  = parentInst ? getPartDef(parentInst.definitionId) : null

    if (childDef && parentDef && parentInst) {
      const pIface = parentDef.interfaces.find(i => i.id === hit.interfaceId)
      const cIface = pIface
        ? childDef.interfaces.find(ci => interfacesCompatible(ci.type, pIface.type))
        : undefined

      if (cIface) {
        aRenderer.showGhostAt(pendingDefId, defaultParams(childDef), hit.instanceId, hit.interfaceId!, cIface.id)
        canvas.style.cursor = 'pointer'
        return
      }
    }
  }

  aRenderer.clearGhost()
  canvas.style.cursor = hit?.type === 'instance' ? 'pointer' : ''
})

// ── Keyboard ──────────────────────────────────────────────────────────────────

document.addEventListener('keydown', (e: KeyboardEvent) => {
  const inInput = e.target instanceof HTMLInputElement
    || e.target instanceof HTMLTextAreaElement
    || e.target instanceof HTMLSelectElement

  // Undo / Redo (works even in inputs only for Ctrl+Z/Y)
  if ((e.ctrlKey || e.metaKey) && !inInput) {
    if (e.key === 'z' && !e.shiftKey) {
      e.preventDefault()
      if (history.undo()) {
        if (selectedId && !assembly.getInstance(selectedId)) select(null)
        refreshBuildPanel()
        showToast('Undo', 'info')
      }
      return
    }
    if (e.key === 'y' || (e.key === 'z' && e.shiftKey)) {
      e.preventDefault()
      if (history.redo()) { refreshBuildPanel(); showToast('Redo', 'info') }
      return
    }
  }

  if (inInput) return

  const key = e.key.toLowerCase()

  if (e.key === 'Escape') {
    if (pendingDefId) { setPending(null); clearToolboxSelection() }
    else              { select(null) }
    return
  }

  if ((e.key === 'Delete' || e.key === 'Backspace') && selectedId) {
    e.preventDefault()
    deletePart(selectedId)
    return
  }

  if (key === 'f' && selectedId) { focusOn(selectedId); return }

  if (key === 'a') {
    originAxes.visible = !originAxes.visible
    document.getElementById('toggle-axes')?.classList.toggle('active', originAxes.visible)
    return
  }
  if (key === 'g') {
    grid.visible = !grid.visible
    document.getElementById('toggle-grid')?.classList.toggle('active', grid.visible)
    return
  }

  if (key === 't') { switchToPanel('toolbox');   return }
  if (key === 'i') { switchToPanel('inspector'); return }

  // R — toggle gizmo translate / rotate mode
  if (key === 'r' && selectedId && gizmo.object) {
    const nextMode = gizmo.mode === 'translate' ? 'rotate' : 'translate'
    gizmo.setMode(nextMode)
    showToast(`Gizmo: ${nextMode}`, 'info')
    return
  }

  // W — toggle gizmo world / local space
  if (key === 'w' && selectedId && gizmo.object) {
    const nextSpace = gizmo.space === 'world' ? 'local' : 'world'
    gizmo.setSpace(nextSpace)
    showToast(`Gizmo: ${nextSpace}`, 'info')
    return
  }
})

// ── Activity bar ──────────────────────────────────────────────────────────────

document.querySelectorAll<HTMLElement>('.ab-btn').forEach(btn => {
  btn.addEventListener('click', () => switchToPanel(btn.dataset.panel!))
})

// ── Viewport toggle buttons ───────────────────────────────────────────────────

document.getElementById('toggle-axes')?.addEventListener('click', () => {
  originAxes.visible = !originAxes.visible
  document.getElementById('toggle-axes')?.classList.toggle('active', originAxes.visible)
})

document.getElementById('toggle-grid')?.addEventListener('click', () => {
  grid.visible = !grid.visible
  document.getElementById('toggle-grid')?.classList.toggle('active', grid.visible)
})

// ── Export ────────────────────────────────────────────────────────────────────

document.getElementById('btn-export-urdf')?.addEventListener('click', () => {
  if (assembly.isEmpty()) { showToast('Nothing to export', 'warning'); return }
  const blob = new Blob([generateURDF(assembly)], { type: 'text/xml' })
  const url  = URL.createObjectURL(blob)
  Object.assign(document.createElement('a'), { href: url, download: 'robot.urdf' }).click()
  URL.revokeObjectURL(url)
  showToast('Exported robot.urdf', 'success')
})

document.getElementById('btn-export-mjcf')?.addEventListener('click', () => {
  if (assembly.isEmpty()) { showToast('Nothing to export', 'warning'); return }
  const blob = new Blob([generateMJCF(assembly)], { type: 'text/xml' })
  const url  = URL.createObjectURL(blob)
  Object.assign(document.createElement('a'), { href: url, download: 'robot.xml' }).click()
  URL.revokeObjectURL(url)
  showToast('Exported robot.xml', 'success')
})

document.getElementById('btn-clear-assembly')?.addEventListener('click', () => {
  if (!confirm('Clear the entire assembly?')) return
  select(null)
  setPending(null)
  clearToolboxSelection()
  history.record()
  assembly.clear()
  showToast('Assembly cleared', 'info')
  switchToPanel('build')
})

// ── Example assembly ──────────────────────────────────────────────────────────

document.getElementById('btn-load-example')?.addEventListener('click', () => {
  const hingeDef = getPartDef('joint.hinge.block')
  const linkDef  = getPartDef('link.arm.single')
  const footDef  = getPartDef('foot.pad.basic')
  if (!hingeDef || !linkDef) { showToast('Parts not found', 'warning'); return }
  history.record()
  assembly.clear()
  const rootId = assembly.addRoot('joint.hinge.block', defaultParams(hingeDef), 'Hip Joint')
  const legId  = assembly.addPart(
    'link.arm.single', defaultParams(linkDef),
    rootId, 'axle_out', 'root', { type: 'revolute' }, 'Upper Leg',
  )
  if (footDef) {
    const footId = assembly.addPart(
      'foot.pad.basic', defaultParams(footDef),
      legId, 'tip', 'mount', { type: 'fixed' }, 'Foot Pad',
    )
    select(footId)
  } else {
    select(legId)
  }
  showToast('Example mechanical assembly loaded', 'success')
})

// ── Simulate (stub) ───────────────────────────────────────────────────────────

document.getElementById('sim-toggle')?.addEventListener('click', () => {
  if (assembly.isEmpty()) { showToast('Build a robot first', 'warning'); return }
  showToast('Simulation not available in this build', 'info')
})

// ── Start ─────────────────────────────────────────────────────────────────────

switchToPanel('build')
