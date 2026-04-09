import * as THREE from 'three'
import { OrbitControls } from 'three/addons/controls/OrbitControls.js'
import { TransformControls } from 'three/addons/controls/TransformControls.js'

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
  isSimActive?: () => boolean
}

export interface UrdfAssemblyApi {
  onModelUpdated(): void
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

  const buildTree = document.getElementById('build-tree') as HTMLDivElement | null
  const buildEmpty = document.getElementById('build-empty') as HTMLDivElement | null
  const bsParts = document.getElementById('bs-parts') as HTMLSpanElement | null
  const bsMass = document.getElementById('bs-mass') as HTMLSpanElement | null
  const bsJoints = document.getElementById('bs-joints') as HTMLSpanElement | null
  const inspBody = document.querySelector('#panel-inspector .insp-body') as HTMLDivElement | null
  const inspTitle = document.getElementById('insp-title') as HTMLSpanElement | null
  const toolboxItems = document.getElementById('toolbox-items') as HTMLDivElement | null
  const toolboxDetail = document.getElementById('toolbox-detail') as HTMLDivElement | null
  const toolboxHint = document.getElementById('toolbox-hint') as HTMLDivElement | null
  const toolboxSearch = document.getElementById('toolbox-search') as HTMLInputElement | null
  const btnFocusBase = document.getElementById('btn-load-example') as HTMLButtonElement | null
  const btnSaveUrdf = document.getElementById('btn-export-urdf') as HTMLButtonElement | null
  const btnCopyUrdf = document.getElementById('btn-export-mjcf') as HTMLButtonElement | null
  const btnResetRobot = document.getElementById('btn-clear-assembly') as HTMLButtonElement | null

  const templates = [
    { id: 'box', title: 'Add Box Link', desc: 'Create a child link with box visual and a fixed joint' },
    { id: 'cylinder', title: 'Add Cylinder Link', desc: 'Create a child link with cylinder visual and a fixed joint' },
    { id: 'sphere', title: 'Add Sphere Link', desc: 'Create a child link with sphere visual and a fixed joint' },
  ]

  function recordUndo() {
    urdfUndo.push(ctx.getUrdfText())
    if (urdfUndo.length > 80) urdfUndo.shift()
    urdfRedo = []
  }

  function commitUrdf(mutator: (doc: Document) => boolean): boolean {
    if (ctx.isSimActive?.()) {
      ctx.showToast('Stop simulation before editing URDF', 'warning')
      return false
    }
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
        if (typeof name === 'string' && name) targets.push(o)
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

  function nextUniqueName(prefix: string, taken: Set<string>): string {
    let idx = 1
    while (taken.has(`${prefix}_${idx}`)) idx++
    return `${prefix}_${idx}`
  }

  function refreshBuildPanel() {
    if (!buildTree || !buildEmpty || !bsParts || !bsMass || !bsJoints) return
    const graph = ctx.getKinematicGraph()
    const joints = ctx.getKinematicJoints()
    const links = Object.values(graph)
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
      el.innerHTML = `
        <span class="bt-joint-badge">${edge ? edge.type.slice(0, 3) : 'root'}</span>
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
    inspTitle.textContent = selectedLink
    inspBody.innerHTML = `
      <div class="bi-section">
        <div class="bi-section-title">Link</div>
        <div class="insp-row"><span class="insp-key">Name</span><span class="insp-val">${selectedLink}</span></div>
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
          <div class="bi-actions" style="padding-top:8px">
            <button type="button" class="bi-action-btn focus-btn" id="urdf-apply-origin"><span class="ba-icon">✓</span>Apply</button>
            <button type="button" class="bi-action-btn del-btn" id="urdf-delete-link"><span class="ba-icon">✕</span>Delete Link</button>
          </div>
        ` : `
          <div class="insp-empty">Root link has no parent joint origin</div>
          <div class="bi-actions" style="padding-top:8px">
            <button type="button" class="bi-action-btn del-btn root-del" id="urdf-delete-link" disabled><span class="ba-icon">✕</span>Delete Link</button>
          </div>
        `}
      </div>
    `
    const deleteBtn = document.getElementById('urdf-delete-link')
    deleteBtn?.addEventListener('click', () => {
      if (!selectedLink) return
      deleteLinkCascade(selectedLink)
    })
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

  function selectLink(name: string | null) {
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
      ctx.switchPanel('inspector')
    }
    refreshBuildPanel()
    renderInspector()
  }

  function deleteLinkCascade(targetLink: string) {
    const graph = ctx.getKinematicGraph()
    const target = graph[targetLink]
    if (!target) return
    if (!target.parent) {
      ctx.showToast('Cannot delete root link. Use Reset to clear robot.', 'warning')
      return
    }

    const deleting = new Set<string>()
    const walk = (name: string) => {
      if (deleting.has(name)) return
      deleting.add(name)
      const node = graph[name]
      if (!node) return
      for (const child of node.children) walk(child)
    }
    walk(targetLink)

    const joints = ctx.getKinematicJoints()
    const jointsRemoved = Object.values(joints).filter(j => deleting.has(j.parentLink) || deleting.has(j.childLink)).length
    const linksRemoved = deleting.size
    const parentToSelect = target.parent ?? null

    if (!confirm(`Delete "${targetLink}" and ${linksRemoved - 1} descendant link(s)?`)) return

    const ok = commitUrdf(doc => {
      let changed = false
      const jointEls = Array.from(doc.querySelectorAll('joint'))
      for (const je of jointEls) {
        const parent = je.querySelector('parent')?.getAttribute('link') || ''
        const child = je.querySelector('child')?.getAttribute('link') || ''
        if (deleting.has(parent) || deleting.has(child)) {
          je.parentNode?.removeChild(je)
          changed = true
        }
      }

      const linkEls = Array.from(doc.querySelectorAll('link'))
      for (const le of linkEls) {
        const name = le.getAttribute('name') || ''
        if (deleting.has(name)) {
          le.parentNode?.removeChild(le)
          changed = true
        }
      }
      return changed
    })

    if (ok) {
      ctx.showToast(`Deleted ${linksRemoved} link(s), ${jointsRemoved} joint(s)`, 'success')
      selectLink(parentToSelect)
    }
  }

  function addTemplate(kind: 'box' | 'cylinder' | 'sphere') {
    if (!selectedLink) {
      ctx.showToast('Select a parent link first', 'warning')
      return
    }
    const parentLink = selectedLink
    const graph = ctx.getKinematicGraph()
    const joints = ctx.getKinematicJoints()
    const linkNames = new Set(Object.keys(graph))
    const jointNames = new Set(Object.keys(joints))
    const childName = nextUniqueName('link', linkNames)
    const jointName = nextUniqueName('joint', jointNames)
    const changed = commitUrdf(doc => {
      const robot = doc.querySelector('robot')
      if (!robot) return false

      const link = doc.createElement('link')
      link.setAttribute('name', childName)

      const inertial = doc.createElement('inertial')
      const mass = doc.createElement('mass')
      mass.setAttribute('value', '0.1')
      const inertia = doc.createElement('inertia')
      inertia.setAttribute('ixx', '0.0001'); inertia.setAttribute('iyy', '0.0001'); inertia.setAttribute('izz', '0.0001')
      inertia.setAttribute('ixy', '0'); inertia.setAttribute('ixz', '0'); inertia.setAttribute('iyz', '0')
      inertial.appendChild(mass)
      inertial.appendChild(inertia)
      link.appendChild(inertial)

      const visual = doc.createElement('visual')
      const vo = doc.createElement('origin')
      vo.setAttribute('xyz', '0 0 0')
      vo.setAttribute('rpy', '0 0 0')
      const geometry = doc.createElement('geometry')
      if (kind === 'box') {
        const box = doc.createElement('box')
        box.setAttribute('size', '0.08 0.04 0.04')
        geometry.appendChild(box)
      } else if (kind === 'cylinder') {
        const c = doc.createElement('cylinder')
        c.setAttribute('radius', '0.02')
        c.setAttribute('length', '0.12')
        geometry.appendChild(c)
      } else {
        const s = doc.createElement('sphere')
        s.setAttribute('radius', '0.03')
        geometry.appendChild(s)
      }
      visual.appendChild(vo)
      visual.appendChild(geometry)
      link.appendChild(visual)

      const joint = doc.createElement('joint')
      joint.setAttribute('name', jointName)
      joint.setAttribute('type', 'fixed')
      const parent = doc.createElement('parent')
      parent.setAttribute('link', parentLink)
      const child = doc.createElement('child')
      child.setAttribute('link', childName)
      const origin = doc.createElement('origin')
      origin.setAttribute('xyz', '0 0 0.1')
      origin.setAttribute('rpy', '0 0 0')
      joint.appendChild(parent)
      joint.appendChild(child)
      joint.appendChild(origin)

      robot.appendChild(link)
      robot.appendChild(joint)
      return true
    })
    if (changed) {
      ctx.showToast(`Added ${kind} link "${childName}"`, 'success')
      selectLink(childName)
    }
  }

  function renderToolbox(filter = '') {
    if (!toolboxItems || !toolboxDetail) return
    const q = filter.trim().toLowerCase()
    const items = templates.filter(t => !q || t.title.toLowerCase().includes(q) || t.desc.toLowerCase().includes(q))
    toolboxItems.innerHTML = ''
    for (const t of items) {
      const el = document.createElement('div')
      el.className = 'tb-item'
      el.innerHTML = `
        <div class="tb-item-info">
          <div class="tb-item-name">${t.title}</div>
          <div class="tb-item-meta">${t.desc}</div>
        </div>
      `
      el.addEventListener('click', () => {
        if (t.id === 'box' || t.id === 'cylinder' || t.id === 'sphere') addTemplate(t.id)
        toolboxDetail.innerHTML = `<div class="tb-detail-name">${t.title}</div><div class="tb-detail-desc">${t.desc}</div>`
      })
      toolboxItems.appendChild(el)
    }
    if (items.length === 0) {
      toolboxItems.innerHTML = '<div class="tb-empty">No matching templates</div>'
    }
  }

  if (toolboxHint) {
    toolboxHint.textContent = 'Select a URDF link, then click a template to add a child link + joint.'
  }
  btnFocusBase?.addEventListener('click', () => {
    const graph = ctx.getKinematicGraph()
    const base = Object.values(graph).find(l => !l.parent)?.name || 'base_link'
    selectLink(base)
  })
  btnSaveUrdf?.addEventListener('click', () => {
    const blob = new Blob([ctx.getUrdfText()], { type: 'application/xml' })
    const url = URL.createObjectURL(blob)
    Object.assign(document.createElement('a'), { href: url, download: 'robot.urdf' }).click()
    URL.revokeObjectURL(url)
    ctx.showToast('Saved URDF snapshot', 'success')
  })
  btnCopyUrdf?.addEventListener('click', async () => {
    try {
      await navigator.clipboard.writeText(ctx.getUrdfText())
      ctx.showToast('URDF copied to clipboard', 'success')
    } catch {
      ctx.showToast('Clipboard copy failed', 'warning')
    }
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
  toolboxSearch?.addEventListener('input', () => renderToolbox(toolboxSearch.value))
  renderToolbox('')

  gizmo.addEventListener('dragging-changed', ev => {
    const on = Boolean((ev as unknown as { value: boolean }).value)
    ctx.controls.enabled = !on
    if (on && selectedLink) {
      const pivot = getPivotGroupForLink(selectedLink)
      if (pivot) {
        pivot.updateMatrixWorld(true)
        gizmoBasePivotWorld.copy(pivot.matrixWorld)
      }
      return
    }
    if (!on && selectedLink) {
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
    if ((k === 'delete' || k === 'backspace') && selectedLink) {
      e.preventDefault()
      deleteLinkCascade(selectedLink)
      return
    }
    if (k === 'r' && selectedLink && gizmo.object) {
      gizmo.setMode(gizmo.mode === 'translate' ? 'rotate' : 'translate')
      ctx.showToast(`Gizmo: ${gizmo.mode}`, 'info')
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
    refreshBuildPanel()
    renderInspector()
  }

  refreshBuildPanel()
  renderInspector()

  return { onModelUpdated }
}

