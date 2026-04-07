import * as THREE from 'three'
import type { LinkDetail } from './robotData'

export interface AttachmentNode {
  id: string
  linkName: string
  label: string
  localPosition: [number, number, number]
  localNormal: [number, number, number]
  isOccupied: boolean
  attachedPartId?: string
  attachedPartName?: string
}

const NODE_OFFSET = 0.013
const NODE_RADIUS = 0.011
const COLOR_EMPTY = 0x00c4a0
const COLOR_SELECTED = 0x44ff99
const COLOR_OCCUPIED = 0xff8800

const nodeData = new Map<string, AttachmentNode>()
const nodeMeshes = new Map<string, THREE.Mesh>()
const attachedPartMeshes = new Map<string, THREE.Mesh>()

let currentVisibleLink: string | null = null
let sharedSphereGeo: THREE.SphereGeometry | null = null

function getSphereGeo(): THREE.SphereGeometry {
  if (!sharedSphereGeo) sharedSphereGeo = new THREE.SphereGeometry(NODE_RADIUS, 10, 10)
  return sharedSphereGeo
}

function generateNodes(linkName: string, detail: LinkDetail): AttachmentNode[] {
  const nodes: AttachmentNode[] = []
  const g = detail.geometry

  if (g.type === 'box') {
    const hw = (g.params.width ?? 0.05) / 2
    const hh = (g.params.height ?? 0.05) / 2
    const hd = (g.params.depth ?? 0.05) / 2
    const defs = [
      { label: 'top',    pos: [0,  hh, 0]  as [number,number,number], nrm: [0,  1, 0] as [number,number,number] },
      { label: 'bottom', pos: [0, -hh, 0]  as [number,number,number], nrm: [0, -1, 0] as [number,number,number] },
      { label: 'right',  pos: [hw,  0, 0]  as [number,number,number], nrm: [1,  0, 0] as [number,number,number] },
      { label: 'left',   pos: [-hw, 0, 0]  as [number,number,number], nrm: [-1, 0, 0] as [number,number,number] },
      { label: 'front',  pos: [0,   0, hd] as [number,number,number], nrm: [0,  0, 1] as [number,number,number] },
      { label: 'back',   pos: [0,   0, -hd]as [number,number,number], nrm: [0,  0,-1] as [number,number,number] },
    ]
    for (const d of defs) {
      nodes.push({
        id: `${linkName}::${d.label}`, linkName, label: d.label,
        localPosition: [d.pos[0] + d.nrm[0] * NODE_OFFSET, d.pos[1] + d.nrm[1] * NODE_OFFSET, d.pos[2] + d.nrm[2] * NODE_OFFSET],
        localNormal: d.nrm, isOccupied: false,
      })
    }
  } else if (g.type === 'cylinder') {
    const r = g.params.radius ?? 0.05
    const hl = (g.params.length ?? 0.1) / 2
    nodes.push({ id: `${linkName}::top`,    linkName, label: 'top',       localPosition: [0, hl + NODE_OFFSET, 0],   localNormal: [0, 1, 0],  isOccupied: false })
    nodes.push({ id: `${linkName}::bottom`, linkName, label: 'bottom',    localPosition: [0, -(hl + NODE_OFFSET), 0],localNormal: [0,-1, 0],  isOccupied: false })
    const sideDefs = [
      { label: 'side-0deg',   nrm: [1, 0, 0]  as [number,number,number] },
      { label: 'side-90deg',  nrm: [0, 0, 1]  as [number,number,number] },
      { label: 'side-180deg', nrm: [-1,0, 0]  as [number,number,number] },
      { label: 'side-270deg', nrm: [0, 0,-1]  as [number,number,number] },
    ]
    for (const d of sideDefs) {
      nodes.push({
        id: `${linkName}::${d.label}`, linkName, label: d.label,
        localPosition: [d.nrm[0] * (r + NODE_OFFSET), 0, d.nrm[2] * (r + NODE_OFFSET)],
        localNormal: d.nrm, isOccupied: false,
      })
    }
  } else if (g.type === 'sphere') {
    const r = g.params.radius ?? 0.02
    const defs = [
      { label: 'top',    nrm: [0, 1, 0]  as [number,number,number] },
      { label: 'bottom', nrm: [0,-1, 0]  as [number,number,number] },
      { label: 'right',  nrm: [1, 0, 0]  as [number,number,number] },
      { label: 'left',   nrm: [-1,0, 0]  as [number,number,number] },
      { label: 'front',  nrm: [0, 0, 1]  as [number,number,number] },
      { label: 'back',   nrm: [0, 0,-1]  as [number,number,number] },
    ]
    for (const d of defs) {
      nodes.push({
        id: `${linkName}::${d.label}`, linkName, label: d.label,
        localPosition: [d.nrm[0] * (r + NODE_OFFSET), d.nrm[1] * (r + NODE_OFFSET), d.nrm[2] * (r + NODE_OFFSET)],
        localNormal: d.nrm, isOccupied: false,
      })
    }
  }

  return nodes
}

export function initNodes(
  meshMap: Record<string, THREE.Mesh | THREE.Group>,
  linkDetails: Record<string, LinkDetail>,
) {
  for (const sphere of nodeMeshes.values()) sphere.parent?.remove(sphere)
  nodeMeshes.clear()
  nodeData.clear()

  const geo = getSphereGeo()

  for (const [linkName, detail] of Object.entries(linkDetails)) {
    const mesh = meshMap[linkName]
    if (!mesh) continue

    const nodes = generateNodes(linkName, detail)
    for (const node of nodes) {
      nodeData.set(node.id, node)

      const mat = new THREE.MeshBasicMaterial({
        color: COLOR_EMPTY,
        transparent: true,
        opacity: 0.9,
        depthTest: false,
      })
      const sphere = new THREE.Mesh(geo, mat)
      sphere.position.set(...node.localPosition)
      sphere.visible = false
      sphere.userData.nodeId = node.id
      sphere.userData.isNode = true
      sphere.renderOrder = 999
      mesh.add(sphere)
      nodeMeshes.set(node.id, sphere)
    }
  }
}

export function showNodesForLink(linkName: string) {
  if (currentVisibleLink && currentVisibleLink !== linkName) hideAllNodes()
  currentVisibleLink = linkName
  for (const [nodeId, sphere] of nodeMeshes) {
    const node = nodeData.get(nodeId)
    if (node?.linkName === linkName) {
      sphere.visible = true
      ;(sphere.material as THREE.MeshBasicMaterial).color.setHex(node.isOccupied ? COLOR_OCCUPIED : COLOR_EMPTY)
    }
  }
}

export function hideAllNodes() {
  for (const sphere of nodeMeshes.values()) sphere.visible = false
  currentVisibleLink = null
}

export function getNodesForLink(linkName: string): AttachmentNode[] {
  return Array.from(nodeData.values()).filter(n => n.linkName === linkName)
}

export function getNodeById(nodeId: string): AttachmentNode | undefined {
  return nodeData.get(nodeId)
}

export function getAllNodeSpheres(): THREE.Mesh[] {
  return Array.from(nodeMeshes.values()).filter(s => s.visible)
}

export function selectNodeSphere(nodeId: string) {
  for (const [id, sphere] of nodeMeshes) {
    if (!sphere.visible) continue
    const node = nodeData.get(id)
    const mat = sphere.material as THREE.MeshBasicMaterial
    if (id === nodeId) {
      mat.color.setHex(COLOR_SELECTED)
      sphere.scale.setScalar(1.5)
    } else {
      mat.color.setHex(node?.isOccupied ? COLOR_OCCUPIED : COLOR_EMPTY)
      sphere.scale.setScalar(1)
    }
  }
}

export function clearNodeSelection() {
  for (const [id, sphere] of nodeMeshes) {
    if (!sphere.visible) continue
    const node = nodeData.get(id)
    ;(sphere.material as THREE.MeshBasicMaterial).color.setHex(node?.isOccupied ? COLOR_OCCUPIED : COLOR_EMPTY)
    sphere.scale.setScalar(1)
  }
}

export function attachPartToNode(nodeId: string, partId: string, partName: string, previewMesh: THREE.Mesh): boolean {
  const node = nodeData.get(nodeId)
  const sphere = nodeMeshes.get(nodeId)
  if (!node || !sphere || node.isOccupied) return false

  node.isOccupied = true
  node.attachedPartId = partId
  node.attachedPartName = partName

  // Place preview mesh as sibling to the node sphere (child of same parent mesh)
  const parentObj = sphere.parent
  if (parentObj) {
    previewMesh.position.set(...node.localPosition)
    previewMesh.userData.isPreview = true
    parentObj.add(previewMesh)
    attachedPartMeshes.set(nodeId, previewMesh)
  }

  ;(sphere.material as THREE.MeshBasicMaterial).color.setHex(COLOR_OCCUPIED)
  sphere.scale.setScalar(1)
  return true
}

export function detachPartFromNode(nodeId: string): boolean {
  const node = nodeData.get(nodeId)
  const sphere = nodeMeshes.get(nodeId)
  if (!node || !sphere) return false

  const preview = attachedPartMeshes.get(nodeId)
  if (preview) {
    preview.parent?.remove(preview)
    preview.geometry.dispose()
    ;(preview.material as THREE.Material).dispose()
    attachedPartMeshes.delete(nodeId)
  }

  node.isOccupied = false
  node.attachedPartId = undefined
  node.attachedPartName = undefined
  ;(sphere.material as THREE.MeshBasicMaterial).color.setHex(sphere.visible ? COLOR_EMPTY : COLOR_EMPTY)
  return true
}
