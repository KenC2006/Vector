import { PART_LIBRARY, defaultParams } from './partLibrary'
import type { RobotPartDefinition } from './partLibrary'

type SelectCallback = (defId: string) => void

let _onSelect: SelectCallback | null = null
let _selectedId: string | null = null

const CAT_ICON: Record<string, string> = {
  actuator: '⚙', structural: '⬡', sensor: '◉', electrical: '⚡', end_effector: '✊',
}
const CAT_COLOR: Record<string, string> = {
  actuator: '#ff7b45', structural: '#8888aa', sensor: '#3d9bff',
  electrical: '#ffcc00', end_effector: '#2a6dd9',
}
const CAT_ORDER = ['actuator', 'structural', 'sensor', 'electrical', 'end_effector']
const CAT_LABELS: Record<string, string> = {
  actuator: 'Actuators', structural: 'Structural', sensor: 'Sensors',
  electrical: 'Electrical', end_effector: 'End Effectors',
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

      const itemEl = document.createElement('div')
      itemEl.className = 'tb-item' + (def.id === _selectedId ? ' selected' : '')
      itemEl.dataset.partId = def.id
      itemEl.title = def.description ?? def.name

      itemEl.innerHTML = `
        <span class="tb-icon" style="background:${color}">${icon}</span>
        <div class="tb-item-info">
          <span class="tb-item-name">${def.name}</span>
          <span class="tb-item-meta">${massG}g · ${ifCount} iface${ifCount !== 1 ? 's' : ''}</span>
        </div>
      `

      itemEl.addEventListener('click', () => {
        document.querySelectorAll('.tb-item').forEach(i => i.classList.remove('selected'))
        itemEl.classList.add('selected')
        _selectedId = def.id
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
}
