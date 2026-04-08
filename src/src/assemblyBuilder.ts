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

export interface AssemblyBuilderContext {
  scene: THREE.Scene
  camera: THREE.PerspectiveCamera
  canvas: HTMLCanvasElement
  controls: OrbitControls
  showToast: (message: string, type?: 'success' | 'warning' | 'error' | 'info') => void
  switchPanel: (name: string) => void
}

function isTypingTarget(t: EventTarget | null): boolean {
  if (!(t instanceof HTMLElement)) return false
  const tag = t.tagName
  if (tag === 'INPUT' || tag === 'TEXTAREA' || tag === 'SELECT') return true
  if (t.closest('.monaco-editor')) return true
  return false
}

/** Parametric assembly placement, gizmo, toolbox, and build sidebar — shares collab scene/camera. */
export function initAssemblyBuilder(ctx: AssemblyBuilderContext): void {
  const {
    scene,
    camera,
    canvas,
    controls,
    showToast,
    switchPanel,
  } = ctx

  const gizmo = new TransformControls(camera, canvas)
  gizmo.setMode('translate')
  gizmo.setSpace('world')
  gizmo.setSize(0.8)
  scene.add(gizmo.getHelper())

  gizmo.addEventListener('dragging-changed', event => {
    const on = Boolean((event as unknown as { value: boolean }).value)
    controls.enabled = !on
    if (on) {
      if (selectedId) aRenderer.beginDrag(selectedId)
    } else {
      commitGizmoDrag()
      aRenderer.endDrag()
    }
  })

  gizmo.addEventListener('objectChange', () => {
    if (!selectedId) return
    aRenderer.updateDragPreview(selectedId, 0.08)
    const collisions = aRenderer.checkCollisions(selectedId)
    aRenderer.highlightCollisions(collisions)
  })

  const assembly = new AssemblyGraph()
  const aRenderer = new AssemblyRenderer(scene)
  aRenderer.bind(assembly)
  const history = new AssemblyHistory(assembly)

  let pendingDefId: string | null = null
  let selectedId: string | null = null

  let _gizmoBasePos = new THREE.Vector3()
  let _gizmoBaseQuat = new THREE.Quaternion()

  function updateHint(msg: string | null) {
    const overlay = document.getElementById('build-hint-overlay')
    const text = document.getElementById('build-hint-text')
    if (!overlay) return
    overlay.classList.toggle('hidden', !msg)
    if (msg && text) text.textContent = msg
  }

  let _focusTimer: number | null = null

  function focusOn(instanceId: string) {
    const bb = aRenderer.getBoundingBox(instanceId)
    if (!bb) return
    const center = new THREE.Vector3()
    bb.getCenter(center)
    const size = bb.getSize(new THREE.Vector3()).length()
    const dist = Math.max(0.4, size * 2.5)
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
      if (t >= 1) {
        clearInterval(_focusTimer!)
        _focusTimer = null
      }
    }, 16) as unknown as number
  }

  function syncGizmoForMode() {
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

    const deltaPos = grp.position.clone().sub(_gizmoBasePos)
    const deltaQuat = grp.quaternion.clone().multiply(_gizmoBaseQuat.clone().invert())

    const posMoved = deltaPos.lengthSq() > 1e-10
    const rotMoved = 1 - Math.abs(deltaQuat.dot(new THREE.Quaternion())) > 1e-8

    if (!posMoved && !rotMoved) return

    gizmo.detach()

    history.record()

    if (posMoved) {
      const prev = inst?.dragOffset ?? { x: 0, y: 0, z: 0 }
      assembly.setDragOffset(
        selectedId,
        prev.x + deltaPos.x,
        prev.y + deltaPos.y,
        prev.z + deltaPos.z,
      )
    }

    if (rotMoved) {
      const prev = inst?.dragRotation ?? { x: 0, y: 0, z: 0, w: 1 }
      const prevQuat = new THREE.Quaternion(prev.x, prev.y, prev.z, prev.w)
      let newQuat: THREE.Quaternion
      if (gizmo.space === 'world') {
        newQuat = deltaQuat.clone().multiply(prevQuat)
      } else {
        newQuat = prevQuat.clone().multiply(deltaQuat)
      }
      newQuat.normalize()
      assembly.setDragRotation(selectedId, {
        x: newQuat.x, y: newQuat.y, z: newQuat.z, w: newQuat.w,
      })
    }

    const snap = aRenderer.findSnapTarget(selectedId, 0.08)
    if (snap) {
      const childDef = getPartDef(inst!.definitionId)
      const cIface = childDef?.interfaces.find(i => i.id === snap.draggedIfaceId)
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
      switchPanel('inspector')
    } else {
      gizmo.detach()
      hideBuildInspector()
    }
    refreshBuildPanel()
  }

  function setPending(defId: string | null) {
    pendingDefId = defId
    aRenderer.setPendingPart(defId)
    syncGizmoForMode()
    if (!defId) {
      updateHint(null)
      return
    }

    const def = getPartDef(defId)
    updateHint(
      assembly.isEmpty()
        ? `Click the viewport to place ${def?.name ?? defId} as the root`
        : `Click a glowing ring (○) to connect ${def?.name ?? defId}`,
    )
  }

  function deletePart(instanceId: string) {
    const inst = assembly.getInstance(instanceId)
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

  function refreshBuildPanel() {
    const emptyDiv = document.getElementById('build-empty')
    const treeDiv = document.getElementById('build-tree')
    if (!emptyDiv || !treeDiv) return

    const parts = assembly.size()
    const massG = Math.round(assembly.totalMass() * 1000)
    const joints = Math.max(0, parts - 1)

    const bsParts = document.getElementById('bs-parts')
    const bsMass = document.getElementById('bs-mass')
    const bsJoints = document.getElementById('bs-joints')
    if (bsParts) bsParts.textContent = String(parts)
    if (bsMass) bsMass.textContent = `${massG} g`
    if (bsJoints) bsJoints.textContent = String(joints)

    emptyDiv.classList.toggle('hidden', parts > 0)
    treeDiv.innerHTML = ''

    assembly.walk((inst, parentConn, depth) => {
      const def = getPartDef(inst.definitionId)
      const massGPart = def ? Math.round(def.mass(inst.params) * 1000) : 0
      const jointBadge = parentConn ? `[${parentConn.joint.type.slice(0, 3)}]` : '[root]'

      const row = document.createElement('div')
      row.className = 'bt-row' + (inst.instanceId === selectedId ? ' selected' : '')
      row.style.paddingLeft = `${8 + depth * 14}px`
      row.innerHTML = `
      <span class="bt-joint-badge">${jointBadge}</span>
      <span class="bt-name">${inst.label}</span>
      <span class="bt-mass">${massGPart}g</span>
      <button type="button" class="bt-remove" title="Remove">✕</button>
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

  initBuildInspector(assembly, {
    onParamChange: (id, p) => { history.record(); assembly.updateParams(id, p) },
    onJointChange: (cid, j) => { history.record(); assembly.updateJoint(cid, j) },
    onJointValue: (cid, v) => assembly.setJointValue(cid, v),
    onDelete: id => deletePart(id),
    onFocus: id => focusOn(id),
    onDuplicate: id => {
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
    onLabelChange: (id, label) => { assembly.setLabel(id, label); refreshBuildPanel() },
  })

  initToolbox(defId => setPending(defId))

  function makeRaycaster(e: { clientX: number; clientY: number }): THREE.Raycaster {
    const rect = canvas.getBoundingClientRect()
    const ndc = new THREE.Vector2(
      ((e.clientX - rect.left) / rect.width) * 2 - 1,
      -((e.clientY - rect.top) / rect.height) * 2 + 1,
    )
    const ray = new THREE.Raycaster()
    ray.setFromCamera(ndc, camera)
    return ray
  }

  let _downX = 0
  let _downY = 0
  let _downOnGizmo = false

  canvas.addEventListener('pointerdown', e => {
    _downX = e.clientX
    _downY = e.clientY
    if (pendingDefId) {
      _downOnGizmo = false
      return
    }
    _downOnGizmo = (gizmo.object !== undefined && gizmo.axis !== null)
  })

  canvas.addEventListener('click', (e: MouseEvent) => {
    if (Math.abs(e.clientX - _downX) > 5 || Math.abs(e.clientY - _downY) > 5) return
    if (_downOnGizmo) return

    const hit = aRenderer.raycast(makeRaycaster(e))

    if (!hit) {
      if (pendingDefId && assembly.isEmpty()) {
        const def = getPartDef(pendingDefId)!
        history.record()
        const pid = assembly.addRoot(pendingDefId, defaultParams(def))
        select(pid)
        showToast(`Placed ${def.name} — click a ring (○) to connect more parts`, 'success')
        updateHint('Click a glowing ring (○) to connect another part')
      } else {
        select(null)
      }
      return
    }

    if (hit.type === 'instance') {
      if (pendingDefId) {
        const childDef = getPartDef(pendingDefId)
        const parentInst = assembly.getInstance(hit.instanceId)
        const parentDef = parentInst ? getPartDef(parentInst.definitionId) : null

        if (childDef && parentDef && parentInst) {
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
            updateHint('Click another part or ring (○) to keep building, or Escape to stop')
            return
          }
        }
      }

      select(hit.instanceId)
      return
    }

    if (hit.type === 'interface') {
      if (!pendingDefId) {
        showToast('Pick a part from the Toolbox first', 'info')
        return
      }

      const childDef = getPartDef(pendingDefId)!
      const parentInst = assembly.getInstance(hit.instanceId)!
      const parentDef = getPartDef(parentInst.definitionId)!
      const pIface = parentDef.interfaces.find(i => i.id === hit.interfaceId)
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
      updateHint('Click another ring (○) to keep building, or Escape to stop')
    }
  })

  canvas.addEventListener('mousemove', (e: MouseEvent) => {
    if (!pendingDefId) { aRenderer.clearGhost(); canvas.style.cursor = ''; return }

    const hit = aRenderer.raycast(makeRaycaster(e))

    if (hit?.type === 'interface' && hit.interfaceId) {
      const childDef = getPartDef(pendingDefId)
      const parentInst = assembly.getInstance(hit.instanceId)
      const parentDef = parentInst ? getPartDef(parentInst.definitionId) : null

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

  document.addEventListener('keydown', (e: KeyboardEvent) => {
    const inTyping = isTypingTarget(e.target)

    if ((e.ctrlKey || e.metaKey) && !inTyping) {
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

    if (inTyping) return

    const key = e.key.toLowerCase()

    if (e.key === 'Escape') {
      if (pendingDefId) { setPending(null); clearToolboxSelection() }
      else { select(null) }
      return
    }

    if ((e.key === 'Delete' || e.key === 'Backspace') && selectedId) {
      e.preventDefault()
      deletePart(selectedId)
      return
    }

    if (key === 'f' && selectedId) { focusOn(selectedId); return }

    if (key === 't') { switchPanel('toolbox'); return }
    if (key === 'i') { switchPanel('inspector'); return }

    if (key === 'r' && selectedId && gizmo.object) {
      const nextMode = gizmo.mode === 'translate' ? 'rotate' : 'translate'
      gizmo.setMode(nextMode)
      showToast(`Gizmo: ${nextMode}`, 'info')
      return
    }

    // Shift+W: gizmo space (W alone is wireframe in collab main)
    if (key === 'w' && e.shiftKey && selectedId && gizmo.object) {
      const nextSpace = gizmo.space === 'world' ? 'local' : 'world'
      gizmo.setSpace(nextSpace)
      showToast(`Gizmo: ${nextSpace}`, 'info')
      return
    }
  })

  document.getElementById('btn-export-urdf')?.addEventListener('click', () => {
    if (assembly.isEmpty()) { showToast('Nothing to export', 'warning'); return }
    const blob = new Blob([generateURDF(assembly)], { type: 'text/xml' })
    const url = URL.createObjectURL(blob)
    Object.assign(document.createElement('a'), { href: url, download: 'robot.urdf' }).click()
    URL.revokeObjectURL(url)
    showToast('Exported robot.urdf', 'success')
  })

  document.getElementById('btn-export-mjcf')?.addEventListener('click', () => {
    if (assembly.isEmpty()) { showToast('Nothing to export', 'warning'); return }
    const blob = new Blob([generateMJCF(assembly)], { type: 'text/xml' })
    const url = URL.createObjectURL(blob)
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
    switchPanel('build')
  })

  document.getElementById('btn-load-example')?.addEventListener('click', () => {
    const hingeDef = getPartDef('joint.hinge.block')
    const linkDef = getPartDef('link.arm.single')
    const footDef = getPartDef('foot.pad.basic')
    if (!hingeDef || !linkDef) { showToast('Parts not found', 'warning'); return }
    history.record()
    assembly.clear()
    const rootId = assembly.addRoot('joint.hinge.block', defaultParams(hingeDef), 'Hip Joint')
    const legId = assembly.addPart(
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

  refreshBuildPanel()
}
