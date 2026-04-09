import * as THREE from 'three'
import type { KinematicLink, KinematicJoint, ParsedRobot } from './urdfParser'

interface GraphNode {
  linkName: string
  x: number
  y: number
  width: number
  height: number
}

export function initNodeGraph(deps: {
  viewportPanel: HTMLDivElement
  kinematicGraph: () => Record<string, KinematicLink>
  kinematicJoints: () => Record<string, KinematicJoint>
  parsedRobot: () => ParsedRobot
}): {
  buildNodeGraph: () => void
  clearHighlight: () => void
  highlightMesh: (linkName: string) => void
  isVisible: () => boolean
  toggle: () => void
  hide: () => void
} {
  const { viewportPanel } = deps

  let graphCanvasVisible = false
  let graphCanvas: HTMLCanvasElement | null = null
  let graphCtx: CanvasRenderingContext2D | null = null
  let graphContainer: HTMLDivElement | null = null
  let graphEventsAttached = false
  let graphNodes: GraphNode[] = []
  let currentHighlightedMeshes: THREE.Mesh[] = []

  function highlightMesh(linkName: string) {
    // Clear previous highlights
    for (const mesh of currentHighlightedMeshes) {
      const mat = mesh.material as THREE.MeshStandardMaterial
      if (mat && mat.emissive) mat.emissive.setHex(0x000000)
    }
    currentHighlightedMeshes = []

    // Apply new highlight
    const linkGroup = deps.parsedRobot().linkGroups.get(linkName)
    if (linkGroup) {
      linkGroup.traverse((child) => {
        if (child instanceof THREE.Mesh) {
          const mat = child.material as THREE.MeshStandardMaterial
          if (mat && mat.emissive) {
            mat.emissive.setHex(0x334400)
            currentHighlightedMeshes.push(child)
          }
        }
      })
    }
  }

  function clearHighlight() {
    for (const mesh of currentHighlightedMeshes) {
      const mat = mesh.material as THREE.MeshStandardMaterial
      if (mat && mat.emissive) mat.emissive.setHex(0x000000)
    }
    currentHighlightedMeshes = []
  }

  function buildNodeGraph() {
    const kinematicGraph = deps.kinematicGraph()
    const kinematicJoints = deps.kinematicJoints()

    const dpr = window.devicePixelRatio || 1
    const viewWidth = viewportPanel.clientWidth
    const viewHeight = viewportPanel.clientHeight

    // Layout: top-to-bottom tree
    const nodeWidth = 120
    const nodeHeight = 54
    const levelHeight = 110
    const paddingTop = 50
    const paddingBottom = 30

    // Calculate max tree depth first
    let maxDepth = 0
    function calcDepth(linkName: string, depth: number) {
      if (depth > maxDepth) maxDepth = depth
      const link = kinematicGraph[linkName]
      if (link) {
        for (const child of link.children) {
          calcDepth(child, depth + 1)
        }
      }
    }
    calcDepth('base_link', 0)

    const contentHeight = Math.max(viewHeight, paddingTop + (maxDepth + 1) * levelHeight + paddingBottom)

    // Create scrollable container + canvas
    if (!graphContainer) {
      graphContainer = document.createElement('div')
      graphContainer.id = 'kinematic-graph-container'

      // Close button
      const closeBtn = document.createElement('button')
      closeBtn.textContent = '✕'
      closeBtn.title = 'Close graph (G)'
      closeBtn.style.cssText = `
        position: sticky; top: 8px; float: right; margin-right: 12px;
        z-index: 25; background: #333; color: #ccc; border: 1px solid #555;
        border-radius: 4px; width: 28px; height: 28px; cursor: pointer;
        font-size: 14px; line-height: 1; display: flex; align-items: center;
        justify-content: center;
      `
      closeBtn.addEventListener('click', () => {
        graphCanvasVisible = false
        if (graphContainer) graphContainer.style.display = 'none'
        clearHighlight()
        const toggleGraphBtn = document.getElementById('toggle-graph') as HTMLButtonElement
        toggleGraphBtn.classList.remove('active')
      })
      graphContainer.appendChild(closeBtn)

      graphCanvas = document.createElement('canvas')
      graphCanvas.id = 'kinematic-graph-canvas'
      graphContainer.appendChild(graphCanvas)
      viewportPanel.appendChild(graphContainer)
      graphCtx = graphCanvas.getContext('2d')!
    }

    // Set canvas size with DPI scaling
    graphCanvas!.width = viewWidth * dpr
    graphCanvas!.height = contentHeight * dpr
    graphCanvas!.style.width = viewWidth + 'px'
    graphCanvas!.style.height = contentHeight + 'px'
    graphCanvas!.style.display = 'block'

    graphCtx = graphCanvas!.getContext('2d')!
    graphCtx.setTransform(dpr, 0, 0, dpr, 0, 0)

    graphNodes = []

    // Count siblings at each depth level
    const levelCounts: Record<number, number> = {}
    function countLevels(linkName: string, depth: number) {
      levelCounts[depth] = (levelCounts[depth] || 0) + 1
      const link = kinematicGraph[linkName]
      if (link) {
        for (const child of link.children) {
          countLevels(child, depth + 1)
        }
      }
    }
    countLevels('base_link', 0)

    const levelIndexes: Record<number, number> = {}

    function layoutNode(linkName: string, depth: number) {
      if (!levelIndexes[depth]) levelIndexes[depth] = 0
      const indexInLevel = levelIndexes[depth]
      const total = levelCounts[depth] || 1

      const xSpacing = Math.max(nodeWidth + 30, viewWidth / (total + 1))
      const xOffset = (viewWidth - total * xSpacing) / 2 + xSpacing / 2
      const x = xOffset + indexInLevel * xSpacing
      const y = paddingTop + depth * levelHeight

      graphNodes.push({
        linkName,
        x: x - nodeWidth / 2,
        y: y - nodeHeight / 2,
        width: nodeWidth,
        height: nodeHeight,
      })

      levelIndexes[depth]++
    }

    // Recursively layout
    function walkLayout(linkName: string, depth: number) {
      layoutNode(linkName, depth)
      const link = kinematicGraph[linkName]
      if (link) {
        for (const child of link.children) {
          walkLayout(child, depth + 1)
        }
      }
    }

    walkLayout('base_link', 0)

    // Render graph
    if (graphCtx) {
      graphCtx.clearRect(0, 0, viewWidth, contentHeight)

      // Draw edges first
      graphCtx.strokeStyle = '#569cd6'
      graphCtx.lineWidth = 1.5
      graphCtx.globalAlpha = 0.5

      for (const [linkName, link] of Object.entries(kinematicGraph)) {
        for (const childName of link.children) {
          const parentNode = graphNodes.find((n) => n.linkName === linkName)
          const childNode = graphNodes.find((n) => n.linkName === childName)
          if (parentNode && childNode) {
            graphCtx.beginPath()
            graphCtx.moveTo(parentNode.x + parentNode.width / 2, parentNode.y + parentNode.height)
            graphCtx.lineTo(childNode.x + childNode.width / 2, childNode.y)
            graphCtx.stroke()

            // Draw joint label on edge
            const jx = (parentNode.x + parentNode.width / 2 + childNode.x + childNode.width / 2) / 2
            const jy = (parentNode.y + parentNode.height + childNode.y) / 2

            for (const joint of Object.values(kinematicJoints)) {
              if (joint.parentLink === linkName && joint.childLink === childName) {
                graphCtx.globalAlpha = 1
                graphCtx.fillStyle = '#ce9178'
                graphCtx.font = '10px monospace'
                graphCtx.textAlign = 'center'
                graphCtx.fillText(joint.type, jx, jy - 2)
                graphCtx.globalAlpha = 0.6
                break
              }
            }
          }
        }
      }

      graphCtx.globalAlpha = 1

      // Draw nodes
      for (const node of graphNodes) {
        const link = kinematicGraph[node.linkName]

        // Node background
        graphCtx.fillStyle = '#252526'
        graphCtx.strokeStyle = '#569cd6'
        graphCtx.lineWidth = 2

        // Draw rounded rectangle
        const r = 8
        graphCtx.beginPath()
        graphCtx.moveTo(node.x + r, node.y)
        graphCtx.lineTo(node.x + node.width - r, node.y)
        graphCtx.quadraticCurveTo(node.x + node.width, node.y, node.x + node.width, node.y + r)
        graphCtx.lineTo(node.x + node.width, node.y + node.height - r)
        graphCtx.quadraticCurveTo(node.x + node.width, node.y + node.height, node.x + node.width - r, node.y + node.height)
        graphCtx.lineTo(node.x + r, node.y + node.height)
        graphCtx.quadraticCurveTo(node.x, node.y + node.height, node.x, node.y + node.height - r)
        graphCtx.lineTo(node.x, node.y + r)
        graphCtx.quadraticCurveTo(node.x, node.y, node.x + r, node.y)
        graphCtx.closePath()
        graphCtx.fill()
        graphCtx.stroke()

        // Node text
        graphCtx.fillStyle = '#9cdcfe'
        graphCtx.font = 'bold 12px monospace'
        graphCtx.textAlign = 'center'
        graphCtx.textBaseline = 'top'
        graphCtx.fillText(node.linkName, node.x + node.width / 2, node.y + 8)

        // Mass label
        if (link) {
          graphCtx.fillStyle = '#858585'
          graphCtx.font = '10px monospace'
          graphCtx.fillText(`${link.mass} kg`, node.x + node.width / 2, node.y + 28)
        }
      }
    }

    // Attach interactive events only once
    if (!graphEventsAttached && graphCanvas) {
      graphCanvas.addEventListener('mousemove', (e) => {
        const rect = graphCanvas!.getBoundingClientRect()
        const mx = e.clientX - rect.left
        const my = e.clientY - rect.top + (graphContainer?.scrollTop || 0)

        for (const node of graphNodes) {
          if (
            mx >= node.x &&
            mx <= node.x + node.width &&
            my >= node.y &&
            my <= node.y + node.height
          ) {
            graphCanvas!.style.cursor = 'pointer'
            highlightMesh(node.linkName)
            return
          }
        }
        graphCanvas!.style.cursor = 'default'
        clearHighlight()
      })

      graphCanvas.addEventListener('click', (e) => {
        const rect = graphCanvas!.getBoundingClientRect()
        const mx = e.clientX - rect.left
        const my = e.clientY - rect.top + (graphContainer?.scrollTop || 0)

        for (const node of graphNodes) {
          if (
            mx >= node.x &&
            mx <= node.x + node.width &&
            my >= node.y &&
            my <= node.y + node.height
          ) {
            highlightMesh(node.linkName)
            break
          }
        }
      })

      graphEventsAttached = true
    }
  }

  function isVisible() {
    return graphCanvasVisible
  }

  function toggle() {
    const toggleGraphBtn = document.getElementById('toggle-graph') as HTMLButtonElement
    graphCanvasVisible = !graphCanvasVisible

    if (graphCanvasVisible) {
      if (!graphContainer) buildNodeGraph()
      else {
        graphContainer.style.display = 'block'
        buildNodeGraph() // rebuild to update layout
      }
    } else {
      if (graphContainer) graphContainer.style.display = 'none'
      clearHighlight()
    }

    toggleGraphBtn.classList.toggle('active', graphCanvasVisible)
  }

  function hide() {
    if (graphCanvasVisible) {
      graphCanvasVisible = false
      if (graphContainer) graphContainer.style.display = 'none'
      clearHighlight()
      const toggleGraphBtn = document.getElementById('toggle-graph') as HTMLButtonElement
      toggleGraphBtn.classList.remove('active')
    }
  }

  // Rebuild graph on window resize
  window.addEventListener('resize', () => {
    if (graphContainer && graphCanvasVisible) {
      buildNodeGraph()
    }
  })

  return {
    buildNodeGraph,
    clearHighlight,
    highlightMesh,
    isVisible,
    toggle,
    hide,
  }
}
