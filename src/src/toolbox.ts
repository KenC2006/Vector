import * as THREE from 'three'
import { PART_LIBRARY, defaultParams } from './partLibrary'
import type { RobotPartDefinition } from './partLibrary'

export interface ToolboxPart {
  id: string
  name: string
  category: 'sensor' | 'actuator' | 'structural' | 'electrical'
  geometry: { type: 'box' | 'cylinder' | 'sphere'; params: Record<string, number> }
  mass: number
  color: number
  defaultJointType: 'fixed' | 'revolute' | 'prismatic'
  icon: string
}

export const TOOLBOX_PARTS: ToolboxPart[] = [
  // Sensors
  { id: 'sensor_camera_rgb',    name: 'RGB Camera',           category: 'sensor',      geometry: { type: 'box',      params: { width: 0.050, height: 0.030, depth: 0.020 } }, mass: 0.050, color: 0x3d9bff, defaultJointType: 'fixed',    icon: '📷' },
  { id: 'sensor_depth_camera',  name: 'Depth Camera',         category: 'sensor',      geometry: { type: 'box',      params: { width: 0.090, height: 0.025, depth: 0.025 } }, mass: 0.072, color: 0x5599ff, defaultJointType: 'fixed',    icon: '🎥' },
  { id: 'sensor_imu',           name: 'IMU',                  category: 'sensor',      geometry: { type: 'box',      params: { width: 0.020, height: 0.020, depth: 0.005 } }, mass: 0.010, color: 0x00c4ff, defaultJointType: 'fixed',    icon: '📡' },
  { id: 'sensor_lidar',         name: 'LiDAR',                category: 'sensor',      geometry: { type: 'cylinder', params: { radius: 0.045, length: 0.070 } },              mass: 0.830, color: 0x00e0ff, defaultJointType: 'fixed',    icon: '🔦' },
  { id: 'sensor_fts',           name: 'Force/Torque Sensor',  category: 'sensor',      geometry: { type: 'cylinder', params: { radius: 0.038, length: 0.026 } },              mass: 0.300, color: 0x66b3ff, defaultJointType: 'fixed',    icon: '⚖️' },
  // Actuators
  { id: 'actuator_servo',       name: 'Servo Motor',          category: 'actuator',    geometry: { type: 'box',      params: { width: 0.032, height: 0.055, depth: 0.036 } }, mass: 0.082, color: 0xff7b45, defaultJointType: 'revolute', icon: '⚙️' },
  { id: 'actuator_linear',      name: 'Linear Actuator',      category: 'actuator',    geometry: { type: 'cylinder', params: { radius: 0.015, length: 0.120 } },              mass: 0.150, color: 0xffaa44, defaultJointType: 'prismatic',icon: '↕️' },
  { id: 'actuator_finger',      name: 'Gripper Finger',       category: 'actuator',    geometry: { type: 'box',      params: { width: 0.010, height: 0.080, depth: 0.020 } }, mass: 0.030, color: 0xff9966, defaultJointType: 'prismatic',icon: '🤏' },
  // Structural
  { id: 'struct_l_bracket',     name: 'L-Bracket',            category: 'structural',  geometry: { type: 'box',      params: { width: 0.040, height: 0.040, depth: 0.005 } }, mass: 0.050, color: 0x8888aa, defaultJointType: 'fixed',    icon: '📐' },
  { id: 'struct_tube',          name: 'Extension Tube',       category: 'structural',  geometry: { type: 'cylinder', params: { radius: 0.012, length: 0.100 } },              mass: 0.040, color: 0x9999bb, defaultJointType: 'fixed',    icon: '⬡' },
  { id: 'struct_plate',         name: 'Base Plate',           category: 'structural',  geometry: { type: 'box',      params: { width: 0.100, height: 0.005, depth: 0.100 } }, mass: 0.120, color: 0x7777aa, defaultJointType: 'fixed',    icon: '▭' },
  // Electrical
  { id: 'elec_battery',         name: 'Battery Pack',         category: 'electrical',  geometry: { type: 'box',      params: { width: 0.080, height: 0.040, depth: 0.030 } }, mass: 0.350, color: 0xffcc00, defaultJointType: 'fixed',    icon: '🔋' },
  { id: 'elec_pdb',             name: 'Power Distribution',   category: 'electrical',  geometry: { type: 'box',      params: { width: 0.060, height: 0.060, depth: 0.010 } }, mass: 0.050, color: 0xffdd44, defaultJointType: 'fixed',    icon: '⚡' },
]

type AttachCallback = (partId: string, nodeId: string, partName: string, mesh: THREE.Mesh) => void
type BuildSelectCallback = (defId: string) => void

let getSelectedNodeFn: () => string | null = () => null
let onAttachCallback: AttachCallback | null = null
let onBuildSelectCallback: BuildSelectCallback | null = null
let selectedPartId: string | null = null
let buildModeActive = false

export function initToolbox(
  getSelectedNode: () => string | null,
  onAttach: AttachCallback,
) {
  getSelectedNodeFn = getSelectedNode
  onAttachCallback = onAttach

  renderToolboxItems()

  const searchEl = document.getElementById('toolbox-search') as HTMLInputElement | null
  searchEl?.addEventListener('input', () => renderToolboxItems(searchEl?.value ?? ''))
}

/**
 * Switch the toolbox into Build Mode, showing PART_LIBRARY definitions.
 * `onSelect` is called when the user clicks a part (selects it as pending).
 */
export function setToolboxBuildMode(active: boolean, onSelect?: BuildSelectCallback) {
  buildModeActive = active
  onBuildSelectCallback = onSelect ?? null
  selectedPartId = null
  renderToolboxItems()

  const hint = document.getElementById('toolbox-hint')
  if (hint) {
    hint.textContent = active
      ? 'Click a part to select it, then click an interface ring (○) to connect'
      : 'Select a node (●) then double-click a part to attach'
  }
}

export function getToolboxSelectedDefId(): string | null {
  return buildModeActive ? selectedPartId : null
}

export function clearToolboxSelection() {
  selectedPartId = null
  document.querySelectorAll('.tb-item').forEach(i => i.classList.remove('selected'))
}

export function renderToolboxItems(filter = '') {
  const container = document.getElementById('toolbox-items')
  if (!container) return
  container.innerHTML = ''

  if (buildModeActive) {
    renderBuildModeItems(container, filter)
  } else {
    renderInspectModeItems(container, filter)
  }
}

const CAT_ICON: Record<string, string> = {
  actuator: '⚙', structural: '⬡', sensor: '◉', electrical: '⚡', end_effector: '✊',
}
const CAT_COLOR: Record<string, string> = {
  actuator: '#ff7b45', structural: '#8888aa', sensor: '#3d9bff', electrical: '#ffcc00', end_effector: '#2a6dd9',
}

function renderBuildModeItems(container: HTMLElement, filter: string) {
  const categories = ['actuator', 'structural', 'sensor', 'electrical', 'end_effector']
  const catLabels: Record<string, string> = {
    actuator: 'Actuators', structural: 'Structural', sensor: 'Sensors',
    electrical: 'Electrical', end_effector: 'End Effectors',
  }

  for (const cat of categories) {
    const defs = PART_LIBRARY.filter((d: RobotPartDefinition) =>
      d.category === cat && (!filter || d.name.toLowerCase().includes(filter.toLowerCase()))
    )
    if (defs.length === 0) continue

    const catEl = document.createElement('div')
    catEl.className = 'tb-cat'
    catEl.innerHTML = `<span class="tb-cat-arrow">▼</span>${catLabels[cat].toUpperCase()}`
    catEl.addEventListener('click', () => {
      const list = catEl.nextElementSibling as HTMLElement
      if (!list) return
      const collapsed = list.classList.toggle('collapsed')
      ;(catEl.querySelector('.tb-cat-arrow') as HTMLElement).textContent = collapsed ? '▶' : '▼'
    })
    container.appendChild(catEl)

    const listEl = document.createElement('div')
    listEl.className = 'tb-list'

    for (const def of defs) {
      const params  = defaultParams(def)
      const massKg  = def.mass(params)
      const ifCount = def.interfaces.length
      const color   = CAT_COLOR[def.category] ?? '#888'
      const icon    = CAT_ICON[def.category] ?? '●'

      const itemEl = document.createElement('div')
      itemEl.className = 'tb-item' + (def.id === selectedPartId ? ' selected' : '')
      itemEl.dataset.partId = def.id
      itemEl.title = def.description
      itemEl.innerHTML = `
        <span class="tb-icon" style="background:${color};font-size:14px;line-height:1">${icon}</span>
        <div class="tb-item-info">
          <span class="tb-item-name">${def.name}</span>
          <span class="tb-item-meta">${Math.round(massKg * 1000)}g · ${ifCount} interface${ifCount !== 1 ? 's' : ''}</span>
        </div>
      `
      itemEl.addEventListener('click', () => {
        document.querySelectorAll('.tb-item').forEach(i => i.classList.remove('selected'))
        itemEl.classList.add('selected')
        selectedPartId = def.id
        onBuildSelectCallback?.(def.id)
      })
      listEl.appendChild(itemEl)
    }
    container.appendChild(listEl)
  }
}

function renderInspectModeItems(container: HTMLElement, filter: string) {
  const categories: ToolboxPart['category'][] = ['sensor', 'actuator', 'structural', 'electrical']
  const catLabels: Record<string, string> = { sensor: 'Sensors', actuator: 'Actuators', structural: 'Structural', electrical: 'Electrical' }

  for (const cat of categories) {
    const parts = TOOLBOX_PARTS.filter(p =>
      p.category === cat && (!filter || p.name.toLowerCase().includes(filter.toLowerCase()))
    )
    if (parts.length === 0) continue

    const catEl = document.createElement('div')
    catEl.className = 'tb-cat'
    catEl.innerHTML = `<span class="tb-cat-arrow">▼</span>${catLabels[cat].toUpperCase()}`
    catEl.addEventListener('click', () => {
      const list = catEl.nextElementSibling as HTMLElement
      if (!list) return
      const collapsed = list.classList.toggle('collapsed')
      ;(catEl.querySelector('.tb-cat-arrow') as HTMLElement).textContent = collapsed ? '▶' : '▼'
    })
    container.appendChild(catEl)

    const listEl = document.createElement('div')
    listEl.className = 'tb-list'

    for (const part of parts) {
      const itemEl = document.createElement('div')
      itemEl.className = 'tb-item' + (part.id === selectedPartId ? ' selected' : '')
      itemEl.dataset.partId = part.id

      const swatch = `background:${'#' + part.color.toString(16).padStart(6, '0')}`
      itemEl.innerHTML = `
        <span class="tb-icon" style="${swatch}">${part.icon}</span>
        <div class="tb-item-info">
          <span class="tb-item-name">${part.name}</span>
          <span class="tb-item-meta">${Math.round(part.mass * 1000)}g · ${part.defaultJointType}</span>
        </div>
      `
      itemEl.addEventListener('click', () => {
        document.querySelectorAll('.tb-item').forEach(i => i.classList.remove('selected'))
        itemEl.classList.add('selected')
        selectedPartId = part.id
      })
      itemEl.addEventListener('dblclick', () => attemptAttach(part.id))
      listEl.appendChild(itemEl)
    }
    container.appendChild(listEl)
  }
}

function attemptAttach(partId: string) {
  const nodeId = getSelectedNodeFn()
  if (!nodeId) {
    const hint = document.getElementById('toolbox-hint')
    if (hint) {
      hint.classList.add('flash')
      setTimeout(() => hint.classList.remove('flash'), 700)
    }
    return
  }
  const part = TOOLBOX_PARTS.find(p => p.id === partId)
  if (!part) return
  const mesh = buildPartMesh(part)
  onAttachCallback?.(partId, nodeId, part.name, mesh)
}

export function attemptAttachSelectedPart() {
  if (selectedPartId) attemptAttach(selectedPartId)
}

function buildPartMesh(part: ToolboxPart): THREE.Mesh {
  let geo: THREE.BufferGeometry
  const p = part.geometry.params
  if (part.geometry.type === 'box')      geo = new THREE.BoxGeometry(p.width,  p.height, p.depth)
  else if (part.geometry.type === 'cylinder') geo = new THREE.CylinderGeometry(p.radius, p.radius, p.length, 14)
  else                                    geo = new THREE.SphereGeometry(p.radius, 10, 10)

  const mat = new THREE.MeshStandardMaterial({
    color: part.color,
    roughness: 0.35,
    metalness: 0.4,
    transparent: true,
    opacity: 0.78,
    emissive: part.color,
    emissiveIntensity: 0.12,
  })
  const mesh = new THREE.Mesh(geo, mat)
  mesh.castShadow = true
  mesh.userData.isPreview = true
  mesh.userData.partName = part.name
  return mesh
}

export function highlightCompatibleParts(nodeId: string | null) {
  document.querySelectorAll('.tb-item').forEach(item => {
    item.classList.toggle('compatible', nodeId !== null)
  })
}

export function getSelectedPartId(): string | null { return selectedPartId }
