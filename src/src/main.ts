import './style.css'
import * as THREE from 'three'
import { OrbitControls } from 'three/addons/controls/OrbitControls.js'
import { TransformControls } from 'three/addons/controls/TransformControls.js'
import { SnapGrid, SNAP_SIZES } from './snapGrid'
import type { SnapSize } from './snapGrid'
import { AssemblyGraph } from './assemblyGraph'
import { AssemblyRenderer } from './assemblyRenderer'
import { AssemblyHistory } from './history'
import { generateURDF, generateMJCF } from './urdfGenerator'
import { getPartDef, defaultParams, interfacesCompatible } from './partLibrary'
import { initToolbox, clearToolboxSelection } from './toolbox'
import { initBuildInspector, showBuildInspectorFor, hideBuildInspector } from './buildInspector'
import { invoke } from '@tauri-apps/api/core'

// ── Scene ─────────────────────────────────────────────────────────────────────

const canvas        = document.getElementById('viewport')      as HTMLCanvasElement
const viewportPanel = document.getElementById('viewport-panel') as HTMLDivElement

const renderer = new THREE.WebGLRenderer({ canvas, antialias: true, alpha: false })
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2))
renderer.setClearColor(0x111827)
renderer.shadowMap.enabled = true
renderer.shadowMap.type    = THREE.PCFSoftShadowMap
renderer.toneMapping       = THREE.ACESFilmicToneMapping
renderer.toneMappingExposure = 1.25

const scene = new THREE.Scene()
scene.fog   = new THREE.FogExp2(0x111827, 0.06)

const camera = new THREE.PerspectiveCamera(50, 1, 0.01, 100)
camera.position.set(1.2, 1.0, 1.6)

const controls = new OrbitControls(camera, canvas)
controls.enableDamping = true
controls.dampingFactor = 0.06
controls.target.set(0, 0.35, 0)
controls.minDistance = 0.3
controls.maxDistance = 8

// Ground + grid + axes
const grid = new THREE.GridHelper(8, 40, 0x2a3040, 0x1e2535)
scene.add(grid)

const groundMesh = new THREE.Mesh(
  new THREE.PlaneGeometry(8, 8),
  new THREE.ShadowMaterial({ opacity: 0.25 })
)
groundMesh.rotation.x = -Math.PI / 2
groundMesh.receiveShadow = true
scene.add(groundMesh)

const originAxes = new THREE.AxesHelper(0.5)
scene.add(originAxes)

// Lights — generous ambient so parts are always legible on a dark background
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

// ── Snap Grid ─────────────────────────────────────────────────────────────────

const snapGrid = new SnapGrid(scene)
controls.addEventListener('change', () => snapGrid.updateAround(controls.target))

// ── Assembly + History ────────────────────────────────────────────────────────

const assembly  = new AssemblyGraph()
const aRenderer = new AssemblyRenderer(scene)
aRenderer.bind(assembly)
const history = new AssemblyHistory(assembly)

// ── Transform Gizmo ───────────────────────────────────────────────────────────

const gizmo = new TransformControls(camera, canvas)
gizmo.setMode('translate')
// eslint-disable-next-line @typescript-eslint/no-explicit-any
scene.add(gizmo as any)

function commitGizmoTransform() {
  const selId = aRenderer.getSelectedInstanceId()
  if (!selId) return
  const grp  = aRenderer.getMeshGroup(selId)
  if (!grp) return
  const inst = assembly.getInstance(selId)
  if (!inst) return
  const baseMat = aRenderer.getComputedWorldMatrix(selId)
  if (!baseMat) return

  history.record()

  if (gizmo.getMode() === 'translate') {
    const basePos = new THREE.Vector3().setFromMatrixPosition(baseMat)
    const prev    = inst.dragOffset ?? { x: 0, y: 0, z: 0 }
    assembly.setDragOffset(selId,
      prev.x + grp.position.x - basePos.x,
      prev.y + grp.position.y - basePos.y,
      prev.z + grp.position.z - basePos.z,
    )
  } else if (gizmo.getMode() === 'rotate') {
    const baseQuat  = new THREE.Quaternion().setFromRotationMatrix(baseMat)
    const deltaQuat = baseQuat.clone().invert().multiply(grp.quaternion)
    const prevDR    = inst.dragRotation ?? { x: 0, y: 0, z: 0, w: 1 }
    const prevQuat  = new THREE.Quaternion(prevDR.x, prevDR.y, prevDR.z, prevDR.w)
    prevQuat.multiply(deltaQuat)
    assembly.setDragRotation(selId, prevQuat)
  }

  requestAnimationFrame(() => {
    const newGrp = aRenderer.getMeshGroup(selId)
    if (newGrp) gizmo.attach(newGrp)
  })
}

// Disable OrbitControls while gizmo is dragging; commit on drag-end
// eslint-disable-next-line @typescript-eslint/no-explicit-any
gizmo.addEventListener('dragging-changed', (event: any) => {
  const isDragging = event.value as boolean
  controls.enabled = !isDragging
  if (!isDragging) commitGizmoTransform()
})

// ── Viewport Buttons ──────────────────────────────────────────────────────────

const toggleAxesBtn = document.getElementById('toggle-axes') as HTMLButtonElement
const toggleGridBtn = document.getElementById('toggle-grid') as HTMLButtonElement

let axesVisible = true
let gridVisible = true

toggleAxesBtn.addEventListener('click', () => {
  axesVisible = !axesVisible
  originAxes.visible = axesVisible
  toggleAxesBtn.classList.toggle('active', axesVisible)
})

toggleGridBtn.classList.add('active')
toggleGridBtn.addEventListener('click', () => {
  gridVisible = !gridVisible
  grid.visible = gridVisible
  toggleGridBtn.classList.toggle('active', gridVisible)
})

// ── Snap Toggle ───────────────────────────────────────────────────────────────

const snapToggleBtn = document.getElementById('toggle-snap')  as HTMLButtonElement | null
const snapLabel     = document.getElementById('snap-label')   as HTMLElement       | null
let _snapVisible  = false
let _snapSizeIdx  = 1 // default 0.1 m

function _updateSnapBtn() {
  if (!snapToggleBtn) return
  if (!_snapVisible) {
    snapToggleBtn.classList.remove('active')
    if (snapLabel) snapLabel.textContent = 'Snap'
  } else {
    snapToggleBtn.classList.add('active')
    if (snapLabel) snapLabel.textContent = `${Math.round(snapGrid.snapSize * 100)}cm`
  }
}

snapToggleBtn?.addEventListener('click', () => {
  if (!_snapVisible) {
    _snapVisible = true
    snapGrid.snapEnabled = true
    snapGrid.setSnapSize(SNAP_SIZES[_snapSizeIdx] as SnapSize)
    snapGrid.setVisible(true)
    snapGrid.updateAround(controls.target)
  } else {
    _snapSizeIdx = (_snapSizeIdx + 1) % SNAP_SIZES.length
    if (_snapSizeIdx === 0) {
      _snapVisible = false
      snapGrid.snapEnabled = false
      snapGrid.setVisible(false)
    } else {
      snapGrid.setSnapSize(SNAP_SIZES[_snapSizeIdx] as SnapSize)
      snapGrid.updateAround(controls.target)
    }
  }
  _updateSnapBtn()
})

// ── Simulation ────────────────────────────────────────────────────────────────

const simToggle   = document.getElementById('sim-toggle')   as HTMLButtonElement
const simBar      = document.getElementById('sim-bar')      as HTMLDivElement
const simPlay     = document.getElementById('sim-play')     as HTMLButtonElement
const simPause    = document.getElementById('sim-pause')    as HTMLButtonElement
const simReset    = document.getElementById('sim-reset')    as HTMLButtonElement
const simProgress = document.getElementById('sim-progress') as HTMLDivElement
const simTimeEl   = document.getElementById('sim-time')     as HTMLSpanElement
const viewportLabel  = document.getElementById('viewport-label')  as HTMLSpanElement
const modeIndicator  = document.getElementById('mode-indicator')  as HTMLSpanElement

let simRunning = false
let simActive  = false
let simTime    = 0
let simCoreRunning    = false
let simStepIntervalId: number | null = null

const simStateDisplay = document.createElement('div')
simStateDisplay.id = 'sim-state-display'
simStateDisplay.style.cssText = `
  position:absolute;top:48px;right:12px;background:rgba(28,28,36,.95);
  border:1px solid #4ec9b0;border-radius:8px;padding:12px;font-family:monospace;
  font-size:11px;color:#e0e0e0;max-width:240px;max-height:300px;overflow-y:auto;
  z-index:100;display:none;backdrop-filter:blur(8px);
`
viewportPanel.appendChild(simStateDisplay)

async function initializeSimulation() {
  await invoke('start_core')
  simCoreRunning = true
  await invoke('sim_load', { path: 'core/test_data/simple_arm.urdf' })
  const state = await invoke('sim_get_state')
  simStateDisplay.style.display = 'block'
  _updateSimStateDisplay(state)
}

async function shutdownSimulation() {
  if (simStepIntervalId !== null) { clearInterval(simStepIntervalId); simStepIntervalId = null }
  await invoke('stop_core')
  simCoreRunning = false
  simStateDisplay.style.display = 'none'
}

async function _stepSimulation() {
  if (!simCoreRunning) return
  await invoke('sim_step', { n_steps: 1 })
  const state = await invoke('sim_get_state')
  _updateSimStateDisplay(state)
}

function _updateSimStateDisplay(state: unknown) {
  const s = state as Record<string, unknown> | null
  if (!s) { simStateDisplay.innerHTML = ''; return }
  let html = '<div style="font-weight:bold;color:#4ec9b0;margin-bottom:8px">Simulation State</div>'
  if (typeof s.time === 'number')
    html += `<div><span style="color:#e5c07b">time:</span> ${s.time.toFixed(3)}s</div>`
  simStateDisplay.innerHTML = html
}

function _updateSimUI() {
  simPlay.classList.toggle('active', simRunning)
  simPause.classList.toggle('active', !simRunning && simActive)
  simTimeEl.textContent = simTime.toFixed(3) + 's'
  simProgress.style.width = `${Math.min((simTime / 10) * 100, 100)}%`
}

simToggle.addEventListener('click', async () => {
  simActive = !simActive
  simBar.classList.toggle('hidden', !simActive)
  simToggle.classList.toggle('running', simActive)
  simToggle.querySelector('span')!.textContent = simActive ? 'Exit Sim' : 'Simulate'
  viewportLabel.textContent  = simActive ? 'Simulation — MuJoCo' : '3D Builder'
  modeIndicator.textContent  = simActive ? 'Simulation' : 'Build'

  if (simActive) {
    try {
      await initializeSimulation()
      showToast('Entered simulation mode (MuJoCo)', 'success')
    } catch (err) {
      console.error('[Sim]', err)
      simActive = false
      simToggle.classList.remove('running')
      simBar.classList.add('hidden')
      modeIndicator.textContent = 'Build'
      showToast('Failed to start simulation', 'error')
    }
  } else {
    simRunning = false
    simTime    = 0
    await shutdownSimulation()
    _updateSimUI()
    showToast('Exited simulation mode', 'info')
  }
  resize()
})

simPlay.addEventListener('click', () => {
  if (!simCoreRunning) return
  simRunning = true
  if (simStepIntervalId !== null) clearInterval(simStepIntervalId)
  simStepIntervalId = setInterval(async () => {
    simTime += 1 / 60
    _updateSimUI()
    await _stepSimulation()
  }, 1000 / 60) as unknown as number
  _updateSimUI()
})

simPause.addEventListener('click', () => {
  simRunning = false
  if (simStepIntervalId !== null) { clearInterval(simStepIntervalId); simStepIntervalId = null }
  _updateSimUI()
})

simReset.addEventListener('click', async () => {
  if (!simCoreRunning) return
  simRunning = false
  if (simStepIntervalId !== null) { clearInterval(simStepIntervalId); simStepIntervalId = null }
  try { await invoke('sim_reset'); simTime = 0; _updateSimUI() } catch (e) { console.error('[Sim]', e) }
})

// ── Resize + Animate ──────────────────────────────────────────────────────────

function resize() {
  const header = document.getElementById('viewport-header')!
  const w = viewportPanel.clientWidth
  const h = viewportPanel.clientHeight - header.offsetHeight - (simActive ? simBar.offsetHeight : 0)
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

// ── Build Functions ───────────────────────────────────────────────────────────

let pendingDefId: string | null = null

function setPendingPart(defId: string | null) {
  pendingDefId = defId
  aRenderer.setPendingPart(defId)
  if (defId) {
    const def = getPartDef(defId)
    updateBuildHint(
      assembly.isEmpty()
        ? `Click the viewport to place ${def?.name ?? defId} as the base`
        : `Click a glowing ring (○) to connect ${def?.name ?? defId}`
    )
  } else {
    aRenderer.clearGhost()
    updateBuildHint(null)
  }
}

let _focusTimer: number | null = null
function focusCameraOn(instanceId: string) {
  const bb = aRenderer.getBoundingBox(instanceId)
  if (!bb) return
  const center = new THREE.Vector3()
  bb.getCenter(center)
  const size = bb.getSize(new THREE.Vector3()).length()
  const dist = Math.max(0.4, size * 2.5)
  const targetPos = new THREE.Vector3(center.x + dist * 0.6, center.y + dist * 0.4, center.z + dist * 0.8)

  const startPos = camera.position.clone()
  const startTgt = controls.target.clone()
  let   t = 0
  if (_focusTimer !== null) clearInterval(_focusTimer)
  _focusTimer = setInterval(() => {
    t = Math.min(t + 0.06, 1)
    const e = 1 - (1 - t) ** 3
    camera.position.lerpVectors(startPos, targetPos, e)
    controls.target.lerpVectors(startTgt, center, e)
    controls.update()
    if (t >= 1) { clearInterval(_focusTimer!); _focusTimer = null }
  }, 16) as unknown as number
}

function selectBuildPart(instanceId: string | null) {
  aRenderer.selectInstance(instanceId)

  if (!instanceId) {
    gizmo.detach()
    hideBuildInspector()
    return
  }

  // Attach gizmo
  const grp = aRenderer.getMeshGroup(instanceId)
  if (grp) gizmo.attach(grp)

  // Open properties panel
  if (assembly.getInstance(instanceId)) {
    showBuildInspectorFor(instanceId)
    switchToPanel('inspector')
  }

  focusCameraOn(instanceId)
}

function deleteBuildPart(instanceId: string) {
  const inst     = assembly.getInstance(instanceId)
  const children = assembly.getChildConnections(instanceId)
  const label    = inst?.label ?? instanceId
  const msg      = children.length > 0
    ? `Remove "${label}" and its ${children.length} child part(s)?`
    : `Remove "${label}"?`
  if (!confirm(msg)) return

  gizmo.detach()
  hideBuildInspector()
  
  history.record()
  assembly.removePart(instanceId)
  refreshBuildPanel()
}

function duplicatePart(instanceId: string) {
  const inst = assembly.getInstance(instanceId)
  if (!inst) return
  const conn = assembly.getParentConnection(instanceId)
  if (!conn) { showToast('Cannot duplicate root — clear and rebuild', 'warning'); return }

  history.record()
  const newId = assembly.addPart(
    inst.definitionId, { ...inst.params },
    conn.parentInstanceId, conn.parentInterfaceId, conn.childInterfaceId,
    { ...conn.joint }, `${inst.label} (copy)`,
  )
  selectBuildPart(newId)
  showToast('Duplicated', 'success')
}

function updateBuildHint(msg: string | null) {
  const overlay = document.getElementById('build-hint-overlay')
  const text    = document.getElementById('build-hint-text')
  if (!overlay) return
  overlay.classList.toggle('hidden', !msg)
  if (msg && text) text.textContent = msg
}

// ── Assembly Panel Refresh ────────────────────────────────────────────────────

function refreshBuildPanel() {
  const emptyDiv = document.getElementById('build-empty')!
  const treeDiv  = document.getElementById('build-tree')!

  const parts  = assembly.size()
  const massG  = Math.round(assembly.totalMass() * 1000)
  const joints = Math.max(0, parts - 1)

  ;(document.getElementById('bs-parts') as HTMLElement).textContent = String(parts)
  ;(document.getElementById('bs-mass')  as HTMLElement).textContent = `${massG} g`
  ;(document.getElementById('bs-joints')as HTMLElement).textContent = String(joints)

  emptyDiv.classList.toggle('hidden', parts > 0)
  treeDiv.innerHTML = ''
  const selId = aRenderer.getSelectedInstanceId()

  assembly.walk((inst, parentConn, depth) => {
    const def       = getPartDef(inst.definitionId)
    const massGPart = def ? Math.round(def.mass(inst.params) * 1000) : 0
    const jointBadge = parentConn ? `[${parentConn.joint.type.slice(0, 3)}]` : '[root]'
    const vis       = aRenderer.isInstanceVisible(inst.instanceId)

    const row = document.createElement('div')
    row.className = 'bt-row' + (inst.instanceId === selId ? ' selected' : '')
    row.style.paddingLeft = `${8 + depth * 14}px`

    row.innerHTML = `
      <span class="bt-joint-badge">${jointBadge}</span>
      <span class="bt-name">${inst.label}</span>
      <span class="bt-mass">${massGPart}g</span>
      <button class="bt-vis" title="${vis ? 'Hide (H)' : 'Show'}">${vis ? '👁' : '·'}</button>
      <button class="bt-remove" data-id="${inst.instanceId}" title="Remove">✕</button>
    `

    row.addEventListener('click', () => selectBuildPart(inst.instanceId))

    row.querySelector('.bt-vis')?.addEventListener('click', (e) => {
      e.stopPropagation()
      aRenderer.setInstanceVisible(inst.instanceId, !vis)
      refreshBuildPanel()
    })

    const rmBtn = row.querySelector('.bt-remove') as HTMLButtonElement
    rmBtn.addEventListener('click', (e) => {
      e.stopPropagation()
      deleteBuildPart(inst.instanceId)
    })

    treeDiv.appendChild(row)
  })
}

// Refresh on any graph mutation that adds/removes parts
assembly.on(ev => {
  const rebuildTypes: string[] = ['instance_added', 'instance_removed', 'cleared', 'restored']
  if (rebuildTypes.includes(ev.type)) refreshBuildPanel()
})

// ── Build Inspector Init ──────────────────────────────────────────────────────

initBuildInspector(assembly, {
  onParamChange: (instanceId, params) => {
    history.record()
    assembly.updateParams(instanceId, params)
    requestAnimationFrame(() => {
      const g = aRenderer.getMeshGroup(instanceId)
      if (g) gizmo.attach(g)
    })
  },
  onJointChange: (connectionId, joint) => {
    history.record()
    assembly.updateJoint(connectionId, joint)
  },
  onJointValue: (connectionId, val) => {
    assembly.setJointValue(connectionId, val)
  },
  onDelete: (instanceId) => deleteBuildPart(instanceId),
  onFocus:  (instanceId) => focusCameraOn(instanceId),
  onDuplicate: (instanceId) => duplicatePart(instanceId),
  onLabelChange: (instanceId, label) => {
    assembly.setLabel(instanceId, label)
    refreshBuildPanel()
  },
})

// ── Toolbox ───────────────────────────────────────────────────────────────────

initToolbox((defId) => {
  setPendingPart(defId)
})

// ── Load Example Assembly ─────────────────────────────────────────────────────

function loadExampleAssembly() {
  // Find parts that exist
  const plateDef = getPartDef('flat_plate')
  const servoDef = getPartDef('servo')
  const tubeDef  = getPartDef('tube')
  if (!plateDef) { showToast('Example parts not available', 'warning'); return }

  const rootId = assembly.addRoot('flat_plate', defaultParams(plateDef), 'Base Plate')

  if (servoDef) {
    const pi = plateDef.interfaces[0]
    const ci = servoDef.interfaces.find(i => interfacesCompatible(i.type, pi?.type ?? ''))
    if (pi && ci) {
      const servoId = assembly.addPart(
        'servo', defaultParams(servoDef),
        rootId, pi.id, ci.id,
        { type: 'fixed' }, 'Base Servo',
      )
      if (tubeDef) {
        const si  = servoDef.interfaces.find(i => i.id !== ci.id && interfacesCompatible(i.type, tubeDef.interfaces[0]?.type ?? ''))
        const ti  = tubeDef.interfaces[0]
        if (si && ti) {
          assembly.addPart(
            'tube', defaultParams(tubeDef),
            servoId, si.id, ti.id,
            { type: 'revolute' }, 'Arm Segment',
          )
        }
      }
    }
  }
  refreshBuildPanel()
  showToast('Example loaded — click parts to inspect and connect more from the Toolbox', 'success')
}

document.getElementById('btn-load-example')?.addEventListener('click', () => {
  if (!assembly.isEmpty() && !confirm('Replace current assembly with example?')) return
  history.record()
  assembly.clear()
  loadExampleAssembly()
})

// ── Viewport Interaction ──────────────────────────────────────────────────────

function makeRaycaster(e: { clientX: number; clientY: number }): THREE.Raycaster {
  const rect = canvas.getBoundingClientRect()
  const ndc  = new THREE.Vector2(
    ((e.clientX - rect.left) / rect.width)  *  2 - 1,
    -((e.clientY - rect.top) / rect.height) *  2 + 1,
  )
  const rc = new THREE.Raycaster()
  rc.setFromCamera(ndc, camera)
  return rc
}

const _dragPlane = new THREE.Plane(new THREE.Vector3(0, 1, 0), 0)
function getPlaneHit(e: { clientX: number; clientY: number }): THREE.Vector3 | null {
  const hit = new THREE.Vector3()
  return makeRaycaster(e).ray.intersectPlane(_dragPlane, hit) ? hit : null
}

/** Process a confirmed click (no drag) in the viewport. */
function handleBuildClick(e: { clientX: number; clientY: number }) {
  const hit = aRenderer.raycast(makeRaycaster(e))

  if (!hit) {
    if (assembly.isEmpty() && pendingDefId) {
      const def = getPartDef(pendingDefId)!
      history.record()
      const pid = assembly.addRoot(pendingDefId, defaultParams(def))
      if (_snapVisible) {
        const ph = getPlaneHit(e)
        if (ph) { const s = snapGrid.snapToGrid(ph); assembly.setDragOffset(pid, s.x, 0, s.z) }
      }
      selectBuildPart(pid)
      showToast(`Placed ${def.name} — now click a glowing ring to attach the next part`, 'success')
      // Keep hint visible: user still needs to connect more parts
      updateBuildHint(`Click a glowing ring (○) on ${def.name} to attach another part`)
    } else {
      selectBuildPart(null)
    }
    return
  }

  if (hit.type === 'instance') {
    selectBuildPart(hit.instanceId)
    return
  }

  if (hit.type === 'interface' && hit.interfaceId) {
    if (!pendingDefId) {
      showToast('Select a part from the Toolbox first', 'info')
      return
    }

    const childDef    = getPartDef(pendingDefId)
    if (!childDef) return
    const parentInst  = assembly.getInstance(hit.instanceId)!
    const parentDef   = getPartDef(parentInst.definitionId)!
    const parentIface = parentDef.interfaces.find(i => i.id === hit.interfaceId)
    if (!parentIface) return

    // Auto-pick the first compatible child interface — no dialog needed
    const compatIface = childDef.interfaces.find(ci => interfacesCompatible(ci.type, parentIface.type))
    if (!compatIface) {
      showToast(`${childDef.name} has no interface compatible with this ring`, 'warning')
      return
    }

    aRenderer.clearGhost()
    history.record()
    const newId = assembly.addPart(
      pendingDefId, defaultParams(childDef),
      hit.instanceId, hit.interfaceId, compatIface.id,
      { type: compatIface.defaultJointType },
    )
    selectBuildPart(newId)
    showToast(`Connected ${childDef.name}`, 'success')
    updateBuildHint(`${childDef.name} added — click another ring to keep building, or press Escape to finish`)
  }
}

// Drag state machine
type DragState = 'idle' | 'pressed' | 'dragging'
let _dragState: DragState             = 'idle'
let _dragInstanceId: string | null    = null
let _dragStartMouse                   = { x: 0, y: 0 }
let _dragStartIntersection            = new THREE.Vector3()
const _dragStartPositions             = new Map<string, THREE.Vector3>()

// Track simple click on empty space (separate from the drag state machine)
let _emptyClickPending   = false
let _emptyClickPos       = { x: 0, y: 0 }

function _collectSubtree(id: string): string[] {
  const ids: string[] = [id]
  for (const c of assembly.getChildConnections(id)) ids.push(..._collectSubtree(c.childInstanceId))
  return ids
}

// Intercept before OrbitControls (capture phase)
canvas.addEventListener('pointerdown', (e: PointerEvent) => {
  if (e.button !== 0) return
  if (gizmo.dragging) return

  const hit = aRenderer.raycast(makeRaycaster(e))

  if (hit?.type === 'instance') {
    if (gizmo.getMode() === 'translate') {
      // Enter drag/click state machine in translate mode
      _dragState      = 'pressed'
      _dragInstanceId = hit.instanceId
      _dragStartMouse = { x: e.clientX, y: e.clientY }
      const ph = getPlaneHit(e)
      if (ph) _dragStartIntersection.copy(ph)
      e.stopPropagation()
    } else {
      // Rotate mode — treat as a simple click (no dragging)
      _emptyClickPending = true
      _emptyClickPos     = { x: e.clientX, y: e.clientY }
    }
  } else {
    // Ring or empty space — track for placement / ring-connect / deselect
    _emptyClickPending = true
    _emptyClickPos     = { x: e.clientX, y: e.clientY }
  }
}, { capture: true })

canvas.addEventListener('pointermove', (e: PointerEvent) => {
  if (_dragState === 'idle') {
    if (gizmo.dragging) return
    const hit = aRenderer.raycast(makeRaycaster(e))

    // Ghost preview: hover over a compatible ring
    if (pendingDefId && hit?.type === 'interface' && hit.interfaceId) {
      const childDef    = getPartDef(pendingDefId)
      const parentInst  = assembly.getInstance(hit.instanceId)
      const parentDef   = parentInst ? getPartDef(parentInst.definitionId) : null
      if (childDef && parentDef && parentInst) {
        const pIface = parentDef.interfaces.find(i => i.id === hit.interfaceId)
        const cIface = pIface
          ? childDef.interfaces.find(ci => interfacesCompatible(ci.type, pIface.type))
          : undefined
        if (cIface) {
          aRenderer.showGhostAt(pendingDefId, defaultParams(childDef), hit.instanceId, hit.interfaceId!, cIface.id)
        } else {
          aRenderer.clearGhost()
        }
      }
    } else {
      aRenderer.clearGhost()
    }

    canvas.style.cursor = (hit?.type === 'instance' && !pendingDefId && gizmo.getMode() === 'translate') ? 'grab' : ''
    return
  }

  if (_dragState === 'pressed') {
    const dx = e.clientX - _dragStartMouse.x
    const dy = e.clientY - _dragStartMouse.y
    if (Math.abs(dx) < 6 && Math.abs(dy) < 6) return

    _dragState = 'dragging'
    canvas.style.cursor = 'grabbing'

    _dragStartPositions.clear()
    if (_dragInstanceId) {
      for (const id of _collectSubtree(_dragInstanceId)) {
        const grp = aRenderer.getMeshGroup(id)
        if (grp) _dragStartPositions.set(id, grp.position.clone())
      }
    }
    if (_snapVisible) snapGrid.setVisible(true)
    return
  }

  if (_dragState === 'dragging') {
    const ph = getPlaneHit(e)
    if (!ph) return
    const snapped = _snapVisible ? snapGrid.showHighlightAt(ph) : ph
    const dx = snapped.x - _dragStartIntersection.x
    const dz = snapped.z - _dragStartIntersection.z
    _dragStartPositions.forEach((sp, id) => {
      const g = aRenderer.getMeshGroup(id)
      if (g) g.position.set(sp.x + dx, sp.y, sp.z + dz)
    })
  }
})

canvas.addEventListener('pointerup', (e: PointerEvent) => {
  if (e.button !== 0) return

  // ── Part was clicked (no drag) ─────────────────────────────────────────────
  if (_dragState === 'pressed') {
    _dragState = 'idle'
    canvas.style.cursor = ''
    _emptyClickPending = false
    handleBuildClick(e)
    return
  }

  // ── Click on ring or empty space ───────────────────────────────────────────
  if (_emptyClickPending) {
    const dx = Math.abs(e.clientX - _emptyClickPos.x)
    const dy = Math.abs(e.clientY - _emptyClickPos.y)
    _emptyClickPending = false
    if (dx < 6 && dy < 6) handleBuildClick(e)
    return
  }

  if (_dragState === 'dragging') {
    snapGrid.clearHighlight()
    if (!_snapVisible) snapGrid.setVisible(false)

    if (_dragInstanceId) {
      const grp      = aRenderer.getMeshGroup(_dragInstanceId)
      const startPos = _dragStartPositions.get(_dragInstanceId)
      if (grp && startPos) {
        const prev = assembly.getInstance(_dragInstanceId)?.dragOffset ?? { x: 0, y: 0, z: 0 }
        history.record()
        assembly.setDragOffset(
          _dragInstanceId,
          prev.x + grp.position.x - startPos.x,
          prev.y,
          prev.z + grp.position.z - startPos.z,
        )
        requestAnimationFrame(() => {
          const g = aRenderer.getMeshGroup(_dragInstanceId!)
          if (g) gizmo.attach(g)
        })
      }
    }

    _dragState      = 'idle'
    _dragInstanceId = null
    canvas.style.cursor = ''
  }
}, { capture: true })

// ── Unified Keyboard Handler ──────────────────────────────────────────────────

document.addEventListener('keydown', (e: KeyboardEvent) => {
  const inInput = e.target instanceof HTMLInputElement
    || e.target instanceof HTMLTextAreaElement
    || e.target instanceof HTMLSelectElement

  const key = e.key.toLowerCase()

  // ── Undo / Redo ────────────────────────────────────────────────────────────
  if (!inInput && (e.ctrlKey || e.metaKey) && key === 'z' && !e.shiftKey) {
    e.preventDefault()
    if (history.undo()) {
      const selId = aRenderer.getSelectedInstanceId()
      if (selId && !assembly.getInstance(selId)) {
        gizmo.detach(); hideBuildInspector(); 
      } else if (selId) {
        const g = aRenderer.getMeshGroup(selId); if (g) gizmo.attach(g)
      }
      refreshBuildPanel()
      showToast('Undo', 'info')
    }
    return
  }
  if (!inInput && (e.ctrlKey || e.metaKey) && (key === 'y' || (e.shiftKey && key === 'z'))) {
    e.preventDefault()
    if (history.redo()) { refreshBuildPanel(); showToast('Redo', 'info') }
    return
  }

  if (inInput) return

  // ── Escape ─────────────────────────────────────────────────────────────────
  if (e.key === 'Escape') {
    if (pendingDefId) {
      setPendingPart(null)
      clearToolboxSelection()
      updateBuildHint(null)
    } else {
      selectBuildPart(null)
    }
    return
  }

  // ── Delete selected part ───────────────────────────────────────────────────
  if (e.key === 'Delete' || e.key === 'Backspace') {
    const sel = aRenderer.getSelectedInstanceId()
    if (sel) { e.preventDefault(); deleteBuildPart(sel) }
    return
  }

  // ── Viewport toggles ───────────────────────────────────────────────────────
  if (key === 'a' && !e.ctrlKey)                       { toggleAxesBtn.click(); return }
  if (key === 'g' && !e.ctrlKey && !e.metaKey)         { toggleGridBtn.click(); return }

  // ── Build shortcuts ────────────────────────────────────────────────────────
  if (key === 'f') {
    const sel = aRenderer.getSelectedInstanceId()
    if (sel) focusCameraOn(sel)
    return
  }
  if (key === 'h') {
    const sel = aRenderer.getSelectedInstanceId()
    if (sel) { aRenderer.setInstanceVisible(sel, !aRenderer.isInstanceVisible(sel)); refreshBuildPanel() }
    return
  }

  // ── Gizmo mode ─────────────────────────────────────────────────────────────
  if (key === 'r') { gizmo.setMode('rotate');    return }
  if (key === 'v' && !e.ctrlKey) { gizmo.setMode('translate'); return }

  // ── Panel shortcuts ────────────────────────────────────────────────────────
  if (key === 't') { switchToPanel('toolbox');   return }
  if (key === 'i') { switchToPanel('inspector'); return }
})

// ── Activity Bar / Panels ─────────────────────────────────────────────────────

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

document.querySelectorAll('.ab-btn').forEach(btn => {
  btn.addEventListener('click', () => {
    const panel = (btn as HTMLElement).dataset.panel!
    if (!panel) return
    const wasActive = btn.classList.contains('active')
    document.querySelectorAll('.ab-btn').forEach(b => b.classList.remove('active'))
    Object.values(panels).forEach(p => p.classList.add('hidden'))
    if (!wasActive) {
      btn.classList.add('active')
      if (panels[panel]) panels[panel].classList.remove('hidden')
    }
  })
})

// Start on assembly panel
switchToPanel('build')

// ── Toast Notifications ───────────────────────────────────────────────────────

function showToast(message: string, type: 'success' | 'warning' | 'error' | 'info' = 'info') {
  const area  = document.getElementById('toast-area') as HTMLDivElement
  const toast = document.createElement('div')
  toast.className = `toast ${type}`
  toast.textContent = message
  area.appendChild(toast)
  requestAnimationFrame(() => toast.classList.add('show'))
  setTimeout(() => {
    toast.classList.remove('show')
    setTimeout(() => toast.remove(), 300)
  }, 3000)
}

// ── Export Buttons ────────────────────────────────────────────────────────────

function _download(content: string, filename: string, mime: string) {
  const blob = new Blob([content], { type: mime })
  const url  = URL.createObjectURL(blob)
  const a    = document.createElement('a')
  a.href = url; a.download = filename; a.click()
  URL.revokeObjectURL(url)
}

document.getElementById('btn-export-urdf')?.addEventListener('click', () => {
  if (assembly.isEmpty()) { showToast('Assembly is empty', 'warning'); return }
  _download(generateURDF(assembly, { robotName: 'my_robot', addGravityLink: true }), 'assembly.urdf', 'text/xml')
  showToast('URDF exported', 'success')
})

document.getElementById('btn-export-mjcf')?.addEventListener('click', () => {
  if (assembly.isEmpty()) { showToast('Assembly is empty', 'warning'); return }
  _download(generateMJCF(assembly, 'my_robot'), 'assembly.xml', 'text/xml')
  showToast('MJCF exported', 'success')
})

document.getElementById('btn-clear-assembly')?.addEventListener('click', () => {
  if (assembly.isEmpty()) return
  if (!confirm('Clear the entire assembly?')) return
  gizmo.detach()
  hideBuildInspector()
  
  history.record()
  assembly.clear()
  clearToolboxSelection()
  setPendingPart(null)
  showToast('Assembly cleared', 'info')
})
