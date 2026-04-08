import { PART_LIBRARY, defaultParams } from './partLibrary'
import type { RobotPartDefinition } from './partLibrary'

type SelectCallback = (defId: string) => void

let _onSelect: SelectCallback | null = null
let _selectedId: string | null = null

const CAT_ICON: Record<string, string> = {
  structure: '▦', joints: '⟲', links: '⎯', feet: '◔', mounts: '⊞',
}
const CAT_COLOR: Record<string, string> = {
  structure: '#8f98ab', joints: '#ff8a4a', links: '#4fa7ff',
  feet: '#7cd37c', mounts: '#b98cff',
}
const CAT_ORDER = ['structure', 'joints', 'links', 'feet', 'mounts']
const CAT_LABELS: Record<string, string> = {
  structure: 'Structure', joints: 'Joints', links: 'Links',
  feet: 'Feet', mounts: 'Mounts',
}

export function initToolbox(onSelect: SelectCallback) {
  _onSelect = onSelect

  const searchEl = document.getElementById('toolbox-search') as HTMLInputElement | null
  searchEl?.addEventListener('input', () => renderToolboxItems(searchEl.value))

  renderToolboxItems('')
}

export function getToolboxSelectedDefId(): string | null {
  return _selectedId
}

export function clearToolboxSelection() {
  _selectedId = null
  document.querySelectorAll('.tb-item').forEach(i => i.classList.remove('selected'))
  renderDetailPanel(null)
}

export function renderToolboxItems(filter = '') {
  const container = document.getElementById('toolbox-items')
  if (!container) return
  container.innerHTML = ''

  const lFilter = filter.toLowerCase()

  for (const cat of CAT_ORDER) {
    const defs = PART_LIBRARY.filter((d: RobotPartDefinition) =>
      d.category === cat && (!lFilter || d.name.toLowerCase().includes(lFilter) || d.description?.toLowerCase().includes(lFilter))
    )
    if (defs.length === 0) continue

    // Category header (collapsible)
    const catEl = document.createElement('div')
    catEl.className = 'tb-cat'
    catEl.innerHTML = `<span class="tb-cat-arrow">▼</span>${CAT_LABELS[cat].toUpperCase()}`
    catEl.addEventListener('click', () => {
      const list = catEl.nextElementSibling as HTMLElement | null
      if (!list) return
      const collapsed = list.classList.toggle('collapsed')
      ;(catEl.querySelector('.tb-cat-arrow') as HTMLElement).textContent = collapsed ? '▶' : '▼'
    })
    container.appendChild(catEl)

    const listEl = document.createElement('div')
    listEl.className = 'tb-list'

    for (const def of defs) {
      const params  = defaultParams(def)
      const massG   = Math.round(def.mass(params) * 1000)
      const ifCount = def.interfaces.length
      const color   = CAT_COLOR[def.category] ?? '#888'
      const icon    = CAT_ICON[def.category]  ?? '●'
      const paramCount = def.parameters.length

      const itemEl = document.createElement('div')
      itemEl.className = 'tb-item' + (def.id === _selectedId ? ' selected' : '')
      itemEl.dataset.partId = def.id
      itemEl.title = def.description ?? def.name

      itemEl.innerHTML = `
        <span class="tb-thumb"></span>
        <span class="tb-icon" style="background:${color}">${icon}</span>
        <div class="tb-item-info">
          <span class="tb-item-name">${def.name}</span>
          <span class="tb-item-meta">${massG}g · ${ifCount} iface${ifCount !== 1 ? 's' : ''} · ${paramCount} param${paramCount !== 1 ? 's' : ''}</span>
          <span class="tb-item-purpose">${def.description}</span>
        </div>
      `

      itemEl.addEventListener('click', () => {
        document.querySelectorAll('.tb-item').forEach(i => i.classList.remove('selected'))
        itemEl.classList.add('selected')
        _selectedId = def.id
        renderDetailPanel(def)
        _onSelect?.(def.id)
      })

      listEl.appendChild(itemEl)
    }
    container.appendChild(listEl)
  }

  // Empty state
  if (container.children.length === 0) {
    const empty = document.createElement('div')
    empty.className = 'tb-empty'
    empty.textContent = filter ? `No parts match "${filter}"` : 'No parts available'
    container.appendChild(empty)
  }

  const selected = _selectedId ? PART_LIBRARY.find(p => p.id === _selectedId) ?? null : null
  renderDetailPanel(selected)
}

function renderDetailPanel(def: RobotPartDefinition | null) {
  const panel = document.getElementById('toolbox-detail')
  if (!panel) return
  if (!def) {
    panel.innerHTML = '<div class="tb-detail-empty">Select a component to see details</div>'
    return
  }

  const defaults = defaultParams(def)
  const massG = Math.round(def.mass(defaults) * 1000)
  const iface = def.interfaces
  const sampleIfaces = iface.slice(0, 4).map(i => `<div class="tb-listline">• ${i.label} <span style="color:#6b6b76">(${i.type})</span></div>`).join('')
  const more = iface.length > 4 ? `<div class="tb-listline">… +${iface.length - 4} more</div>` : ''
  const sampleParams = def.parameters.slice(0, 4).map(p => `<div class="tb-kv"><span>${p.label}</span><span>${String(p.default)}${p.unit ? ` ${p.unit}` : ''}</span></div>`).join('')
  const color = CAT_COLOR[def.category] ?? '#888'

  panel.innerHTML = `
    <div class="tb-detail-name">${def.name}</div>
    <div class="tb-detail-desc">${def.description}</div>
    <div class="tb-detail-chiprow">
      <span class="tb-chip" style="border-color:${color}55;color:${color}">${def.category}</span>
      <span class="tb-chip">${massG}g default</span>
      <span class="tb-chip">${iface.length} interfaces</span>
      <span class="tb-chip">${def.parameters.length} parameters</span>
    </div>
    <div class="tb-detail-sec">
      <div class="tb-detail-sec-title">Default Parameters</div>
      ${sampleParams || '<div class="tb-listline">No editable parameters</div>'}
    </div>
    <div class="tb-detail-sec">
      <div class="tb-detail-sec-title">Interface Map (summary)</div>
      ${sampleIfaces}${more}
    </div>
  `
}
