import * as THREE from 'three'
import {
  getAllNodeSpheres, showNodesForLink, hideAllNodes,
  selectNodeSphere, clearNodeSelection, getNodesForLink, getNodeById,
} from './nodeManager'
import { showInspector, hideInspector, highlightInspectorNodeRow } from './inspector'
import { LINK_DETAILS } from './robotData'
import { highlightCompatibleParts } from './toolbox'

export let selectedLinkName: string | null = null
export let selectedNodeId: string | null = null

let cameraRef: THREE.Camera
let meshMapRef: Record<string, THREE.Mesh | THREE.Group> = {}
let switchToPanelFn: (name: string) => void = () => {}
let initialized = false

const raycaster = new THREE.Raycaster()
const mouse = new THREE.Vector2()
let mouseDownX = 0
let mouseDownY = 0

// ── Init ────────────────────────────────────────────────────────────────────

export function initSelection(
  canvas: HTMLCanvasElement,
  camera: THREE.Camera,
  meshMap: Record<string, THREE.Mesh | THREE.Group>,
  opts: { switchToPanel?: (name: string) => void } = {},
) {
  cameraRef = camera
  meshMapRef = meshMap
  if (opts.switchToPanel) switchToPanelFn = opts.switchToPanel
  initialized = true

  const robotMeshes = Object.values(meshMap).filter((m): m is THREE.Mesh => m instanceof THREE.Mesh)

  canvas.addEventListener('mousedown', (e) => {
    mouseDownX = e.clientX
    mouseDownY = e.clientY
  })

  canvas.addEventListener('mousemove', (e) => {
    setMouse(e, canvas)
    raycaster.setFromCamera(mouse, cameraRef)

    const spheres = getAllNodeSpheres()
    if (spheres.length > 0) {
      const nodeHits = raycaster.intersectObjects(spheres, false)
      if (nodeHits.length > 0) {
        const nodeId = (nodeHits[0].object as THREE.Mesh).userData.nodeId as string
        showNodeTooltip(e.clientX, e.clientY, nodeId)
        canvas.style.cursor = 'pointer'
        return
      }
    }
    hideNodeTooltip()

    const meshHits = raycaster.intersectObjects(robotMeshes, false)
    canvas.style.cursor = meshHits.length > 0 ? 'pointer' : ''
  })

  canvas.addEventListener('click', (e) => {
    const dx = e.clientX - mouseDownX
    const dy = e.clientY - mouseDownY
    if (dx * dx + dy * dy > 25) return

    setMouse(e, canvas)
    raycaster.setFromCamera(mouse, cameraRef)

    const spheres = getAllNodeSpheres()
    if (spheres.length > 0) {
      const nodeHits = raycaster.intersectObjects(spheres, false)
      if (nodeHits.length > 0) {
        const nodeId = (nodeHits[0].object as THREE.Mesh).userData.nodeId as string
        handleNodeClick(nodeId)
        return
      }
    }

    const meshHits = raycaster.intersectObjects(robotMeshes, false)
    if (meshHits.length > 0) {
      const hitMesh = meshHits[0].object as THREE.Mesh
      const linkName = Object.entries(meshMap).find(([, m]) => m === hitMesh)?.[0]
      if (linkName) { handleLinkClick(linkName); return }
    }

    deselectAll()
  })

  canvas.addEventListener('mouseleave', () => {
    hideNodeTooltip()
    canvas.style.cursor = ''
  })
}

function setMouse(e: MouseEvent, canvas: HTMLCanvasElement) {
  const rect = canvas.getBoundingClientRect()
  mouse.x = ((e.clientX - rect.left) / rect.width) * 2 - 1
  mouse.y = -((e.clientY - rect.top) / rect.height) * 2 + 1
}

// ── Handlers ────────────────────────────────────────────────────────────────

function handleLinkClick(linkName: string) {
  if (selectedLinkName && selectedLinkName !== linkName) {
    clearLinkHighlight(selectedLinkName)
    clearNodeSelection()
    selectedNodeId = null
    highlightInspectorNodeRow(null)
    highlightCompatibleParts(null)
  }

  selectedLinkName = linkName
  applyLinkHighlight(linkName)
  showNodesForLink(linkName)

  const detail = LINK_DETAILS[linkName]
  if (detail) showInspector(linkName, detail, getNodesForLink(linkName))

  switchToPanelFn('inspector')
}

function handleNodeClick(nodeId: string) {
  if (selectedNodeId === nodeId) {
    clearNodeSelection()
    selectedNodeId = null
    highlightInspectorNodeRow(null)
    highlightCompatibleParts(null)
  } else {
    clearNodeSelection()
    selectedNodeId = nodeId
    selectNodeSphere(nodeId)
    highlightInspectorNodeRow(nodeId)
    highlightCompatibleParts(nodeId)
  }
}

function deselectAll() {
  if (selectedLinkName) clearLinkHighlight(selectedLinkName)
  clearNodeSelection()
  hideAllNodes()
  selectedLinkName = null
  selectedNodeId = null
  hideInspector()
  highlightInspectorNodeRow(null)
  highlightCompatibleParts(null)
}

// ── Highlight helpers ────────────────────────────────────────────────────────

function applyLinkHighlight(linkName: string) {
  const mesh = meshMapRef[linkName]
  if (mesh instanceof THREE.Mesh) {
    const mat = mesh.material as THREE.MeshStandardMaterial
    if (mat?.emissive) mat.emissive.setHex(0x003366)
  }
}

function clearLinkHighlight(linkName: string) {
  const mesh = meshMapRef[linkName]
  if (mesh instanceof THREE.Mesh) {
    const mat = mesh.material as THREE.MeshStandardMaterial
    if (mat?.emissive) mat.emissive.setHex(0x000000)
  }
}

// ── Programmatic API ────────────────────────────────────────────────────────

export function programmaticSelectLink(linkName: string) {
  if (!initialized) return
  handleLinkClick(linkName)
}

export function programmaticSelectNode(nodeId: string) {
  if (!initialized) return
  handleNodeClick(nodeId)
}

export function programmaticDeselect() {
  if (!initialized) return
  deselectAll()
}

export function getSelectedNodeId(): string | null { return selectedNodeId }

// ── Node tooltip ─────────────────────────────────────────────────────────────

let tooltipEl: HTMLElement | null = null

function getTooltip(): HTMLElement {
  if (!tooltipEl) {
    tooltipEl = document.getElementById('node-tooltip')
    if (!tooltipEl) {
      tooltipEl = document.createElement('div')
      tooltipEl.id = 'node-tooltip'
      document.body.appendChild(tooltipEl)
    }
  }
  return tooltipEl
}

function showNodeTooltip(x: number, y: number, nodeId: string) {
  const node = getNodeById(nodeId)
  if (!node) return
  const el = getTooltip()
  el.innerHTML = `<strong>${node.label}</strong><span>${node.isOccupied ? '● ' + (node.attachedPartName ?? 'occupied') : '○ empty'}</span>`
  el.style.left = `${x + 16}px`
  el.style.top = `${y - 10}px`
  el.classList.add('visible')
}

function hideNodeTooltip() {
  getTooltip().classList.remove('visible')
}
