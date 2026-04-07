import type { AssemblyGraph, JointConfig } from './assemblyGraph'
import { getPartDef } from './partLibrary'
import type { ParamValues } from './partLibrary'

// ── Types ─────────────────────────────────────────────────────────────────────

export interface BuildInspectorCallbacks {
  onParamChange(instanceId: string, params: ParamValues): void
  onJointChange(connectionId: string, joint: Partial<JointConfig>): void
  onJointValue(connectionId: string, value: number): void
  onDelete(instanceId: string): void
  onFocus(instanceId: string): void
  onDuplicate(instanceId: string): void
  onLabelChange(instanceId: string, label: string): void
}

// ── Module state ──────────────────────────────────────────────────────────────

let _graph: AssemblyGraph | null = null
let _callbacks: BuildInspectorCallbacks | null = null
let _currentInstanceId: string | null = null

const CAT_COLOR: Record<string, string> = {
  actuator: '#ff7b45', structural: '#8888bb', sensor: '#3d9bff',
  electrical: '#ffcc00', end_effector: '#2a6dd9',
}
const CAT_ICON: Record<string, string> = {
  actuator: '⚙', structural: '⬡', sensor: '◉', electrical: '⚡', end_effector: '✊',
}
const JOINT_TYPE_LABEL: Record<string, string> = {
  fixed: 'Fixed', revolute: 'Revolute', prismatic: 'Prismatic',
}

// ── Init ──────────────────────────────────────────────────────────────────────

export function initBuildInspector(graph: AssemblyGraph, callbacks: BuildInspectorCallbacks) {
  _graph = graph
  _callbacks = callbacks
}

// ── Show / hide ───────────────────────────────────────────────────────────────

export function hideBuildInspector() {
  _currentInstanceId = null
  const title = document.getElementById('insp-title')
  if (title) title.textContent = 'Inspector'

  const body = document.querySelector('#panel-inspector .insp-body') as HTMLElement | null
  if (!body) return
  body.innerHTML = '<div class="insp-empty">Click a robot part to inspect it</div>'
}

export function showBuildInspectorFor(instanceId: string) {
  if (!_graph || !_callbacks) return
  _currentInstanceId = instanceId

  const inst = _graph.getInstance(instanceId)
  if (!inst) { hideBuildInspector(); return }

  const def = getPartDef(inst.definitionId)
  if (!def) { hideBuildInspector(); return }

  const parentConn = _graph.getParentConnection(instanceId)
  const hasParent  = !!parentConn

  // Swap inspector title
  const title = document.getElementById('insp-title')
  if (title) title.textContent = 'Properties'

  const body = document.querySelector('#panel-inspector .insp-body') as HTMLElement | null
  if (!body) return

  const catColor = CAT_COLOR[def.category] ?? '#888'
  const catIcon  = CAT_ICON[def.category]  ?? '●'
  const ifCount  = def.interfaces.length

  // ── Build HTML ────────────────────────────────────────────────────────────

  body.innerHTML = `
    <div class="bi-header">
      <input class="bi-name-input" id="bi-name-input" value="${escHtml(inst.label)}" maxlength="48" title="Part name (click to rename)"/>
      <div class="bi-meta-row">
        <span class="bi-cat-badge" style="background:${catColor}22;color:${catColor};border-color:${catColor}44">${catIcon} ${def.category}</span>
        <span class="bi-def-badge">${escHtml(def.id)}</span>
        <span class="bi-iface-badge">${ifCount} interface${ifCount !== 1 ? 's' : ''}</span>
      </div>
      ${def.description ? `<div class="bi-desc">${escHtml(def.description)}</div>` : ''}
    </div>

    ${buildParamsHtml(def.parameters, inst.params)}
    ${buildJointHtml(parentConn)}
    ${buildActionsHtml(instanceId, hasParent)}
  `

  // ── Wire up events ────────────────────────────────────────────────────────

  wireNameInput(instanceId, body)
  wireParamSliders(instanceId, def.parameters, inst.params, body)
  wireJointControls(parentConn, body)
  wireActions(instanceId, body)
}

// ── HTML builders ─────────────────────────────────────────────────────────────

function buildParamsHtml(params: import('./partLibrary').PartParameter[], vals: ParamValues): string {
  if (params.length === 0) return ''

  const rows = params.map(p => {
    if (p.type === 'enum') {
      const opts = (p.options ?? []).map(o =>
        `<option value="${o}"${vals[p.id] === o ? ' selected' : ''}>${o}</option>`
      ).join('')
      return `
        <div class="param-row">
          <span class="param-label">${escHtml(p.label)}</span>
          <select class="param-select" data-param-id="${p.id}">${opts}</select>
        </div>`
    }

    const val  = vals[p.id] as number
    const min  = p.min  ?? 0
    const max  = p.max  ?? 1
    const step = p.step ?? 0.001
    const unit = p.unit ?? ''
    const disp = formatParamVal(val, p.type, unit)

    return `
      <div class="param-row">
        <div class="param-top">
          <span class="param-label">${escHtml(p.label)}</span>
          <span class="param-val" data-param-id="${p.id}">${disp}</span>
        </div>
        <input type="range" class="param-range" data-param-id="${p.id}"
          min="${min}" max="${max}" step="${step}" value="${val}"/>
      </div>`
  }).join('')

  return `
    <div class="bi-section">
      <div class="bi-section-title">Parameters</div>
      ${rows}
    </div>`
}

function buildJointHtml(parentConn: import('./assemblyGraph').ConnectionEdge | undefined): string {
  if (!parentConn) return ''
  const j = parentConn.joint
  const isMovable = j.type !== 'fixed'

  const typeButtons = (['fixed', 'revolute', 'prismatic'] as const).map(t =>
    `<button class="joint-type-btn${j.type === t ? ' active' : ''}" data-joint-type="${t}">${JOINT_TYPE_LABEL[t]}</button>`
  ).join('')

  const valSlider = isMovable ? `
    <div class="param-row">
      <div class="param-top">
        <span class="param-label">Current value</span>
        <span class="param-val" id="joint-val-display">${formatJointVal(j.value ?? 0, j.type)}</span>
      </div>
      <input type="range" class="param-range joint-val-range" id="joint-val-slider"
        min="${j.lower ?? -3.14159}" max="${j.upper ?? 3.14159}" step="0.01" value="${j.value ?? 0}"/>
    </div>
    <div class="joint-limits-row">
      <label class="joint-lim-label">Lower</label>
      <input type="number" class="joint-lim-input" id="joint-lower" value="${fmtN(j.lower ?? -3.14159)}" step="0.1">
      <label class="joint-lim-label">Upper</label>
      <input type="number" class="joint-lim-input" id="joint-upper" value="${fmtN(j.upper ?? 3.14159)}" step="0.1">
    </div>` : ''

  return `
    <div class="bi-section">
      <div class="bi-section-title">Joint <span class="joint-conn-id">${parentConn.connectionId}</span></div>
      <div class="joint-type-row">${typeButtons}</div>
      ${valSlider}
    </div>`
}

function buildActionsHtml(instanceId: string, hasParent: boolean): string {
  return `
    <div class="bi-actions">
      <button class="bi-action-btn focus-btn" data-instance-id="${instanceId}" title="Focus camera on this part (F)">
        <span class="ba-icon">⊙</span> Focus
      </button>
      <button class="bi-action-btn dup-btn" data-instance-id="${instanceId}" title="Duplicate: place same part on any interface">
        <span class="ba-icon">⊕</span> Duplicate
      </button>
      <button class="bi-action-btn del-btn${hasParent ? '' : ' root-del'}" data-instance-id="${instanceId}" title="Delete this part${hasParent ? ' and subtree' : ' (clears assembly)'}">
        <span class="ba-icon">⊗</span> Delete
      </button>
    </div>`
}

// ── Event wiring ──────────────────────────────────────────────────────────────

function wireNameInput(instanceId: string, body: HTMLElement) {
  const input = body.querySelector('#bi-name-input') as HTMLInputElement | null
  if (!input) return
  let committed = false
  const commit = () => {
    if (committed) return
    committed = true
    _callbacks?.onLabelChange(instanceId, input.value.trim() || 'Part')
  }
  input.addEventListener('keydown', e => { if (e.key === 'Enter') { input.blur(); e.preventDefault() } })
  input.addEventListener('blur',    commit)
}

function wireParamSliders(
  instanceId: string,
  params: import('./partLibrary').PartParameter[],
  _vals: ParamValues,
  body: HTMLElement,
) {
  // Number sliders
  body.querySelectorAll<HTMLInputElement>('input.param-range').forEach(slider => {
    const paramId = slider.dataset.paramId!
    const display = body.querySelector<HTMLElement>(`.param-val[data-param-id="${paramId}"]`)
    const param   = params.find(p => p.id === paramId)
    if (!param) return

    slider.addEventListener('input', () => {
      const val = parseFloat(slider.value)
      if (display) display.textContent = formatParamVal(val, param.type, param.unit ?? '')
      _callbacks?.onParamChange(instanceId, { [paramId]: val })
    })
  })

  // Enum selects
  body.querySelectorAll<HTMLSelectElement>('select.param-select').forEach(sel => {
    const paramId = sel.dataset.paramId!
    sel.addEventListener('change', () => {
      _callbacks?.onParamChange(instanceId, { [paramId]: sel.value })
    })
  })
}

function wireJointControls(
  parentConn: import('./assemblyGraph').ConnectionEdge | undefined,
  body: HTMLElement,
) {
  if (!parentConn) return
  const connId = parentConn.connectionId

  // Type buttons
  body.querySelectorAll<HTMLButtonElement>('.joint-type-btn').forEach(btn => {
    btn.addEventListener('click', () => {
      const type = btn.dataset.jointType as JointConfig['type']
      body.querySelectorAll('.joint-type-btn').forEach(b => b.classList.remove('active'))
      btn.classList.add('active')
      _callbacks?.onJointChange(connId, { type })

      // Show/hide sliders depending on type
      const isMovable = type !== 'fixed'
      body.querySelectorAll<HTMLElement>('.joint-val-range, .joint-limits-row, #joint-val-display')
        .forEach(el => { el.closest('.param-row, .joint-limits-row')?.classList.toggle('hidden', !isMovable) })
    })
  })

  // Value slider
  const valSlider = body.querySelector<HTMLInputElement>('#joint-val-slider')
  const valDisplay = body.querySelector<HTMLElement>('#joint-val-display')
  if (valSlider && valDisplay) {
    valSlider.addEventListener('input', () => {
      const val = parseFloat(valSlider.value)
      valDisplay.textContent = formatJointVal(val, parentConn.joint.type)
      _callbacks?.onJointValue(connId, val)
    })
  }

  // Limit inputs
  const lowerInput = body.querySelector<HTMLInputElement>('#joint-lower')
  const upperInput = body.querySelector<HTMLInputElement>('#joint-upper')
  lowerInput?.addEventListener('change', () => {
    const lower = parseFloat(lowerInput.value)
    _callbacks?.onJointChange(connId, { lower })
    if (valSlider) valSlider.min = String(lower)
  })
  upperInput?.addEventListener('change', () => {
    const upper = parseFloat(upperInput.value)
    _callbacks?.onJointChange(connId, { upper })
    if (valSlider) valSlider.max = String(upper)
  })
}

function wireActions(instanceId: string, body: HTMLElement) {
  body.querySelector('.focus-btn')?.addEventListener('click', () => {
    _callbacks?.onFocus(instanceId)
  })
  body.querySelector('.dup-btn')?.addEventListener('click', () => {
    _callbacks?.onDuplicate(instanceId)
  })
  body.querySelector('.del-btn')?.addEventListener('click', () => {
    _callbacks?.onDelete(instanceId)
  })
}

// ── Formatters ────────────────────────────────────────────────────────────────

function formatParamVal(val: number, type: string, unit: string): string {
  if (type === 'mass') return `${(val * 1000).toFixed(0)} g`
  if (type === 'length' || type === 'radius') {
    return unit === 'm' ? `${(val * 1000).toFixed(1)} mm` : `${val.toFixed(4)} ${unit}`
  }
  return `${val.toFixed(3)} ${unit}`.trim()
}

function formatJointVal(val: number, type: string): string {
  if (type === 'revolute')  return `${(val * 180 / Math.PI).toFixed(1)}°`
  if (type === 'prismatic') return `${(val * 1000).toFixed(1)} mm`
  return '—'
}

function fmtN(n: number) { return n.toFixed(4) }

function escHtml(s: string): string {
  return s.replace(/&/g,'&amp;').replace(/</g,'&lt;').replace(/>/g,'&gt;').replace(/"/g,'&quot;')
}

// ── Live refresh (called when params rebuild the mesh) ────────────────────────

/** Re-render the inspector for the currently shown instance without closing it. */
export function refreshBuildInspectorParams() {
  if (_currentInstanceId) showBuildInspectorFor(_currentInstanceId)
}
