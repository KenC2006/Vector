import * as THREE from 'three'
import type { OrbitControls } from 'three/addons/controls/OrbitControls.js'
import type { ParsedRobot } from './urdfParser'

export function initViewportControls(deps: {
  camera: THREE.PerspectiveCamera
  renderer: THREE.WebGLRenderer
  controls: OrbitControls
  robot: THREE.Group
  scene: THREE.Scene
  canvas: HTMLCanvasElement
  viewportPanel: HTMLDivElement
  editorPanel: HTMLDivElement
  handle: HTMLDivElement
  originAxes: THREE.AxesHelper
  grid: THREE.GridHelper
  comGroup: THREE.Group
  wireframeGroup: THREE.Group
  axisVisuals: THREE.Object3D[]
  jointAxisState: { visible: boolean }
  simBar: HTMLDivElement
  simActive: () => boolean
  parsedRobot: () => ParsedRobot
  showToast: (msg: string, type?: 'success' | 'warning' | 'error' | 'info') => void
  onResize?: (w: number, h: number) => void
}): {
  resize: () => void
  focusOnRobot: () => void
  zoomCamera: (factor: number) => void
  setViewportCollapsed: (collapsed: boolean) => void
  setViewportFullscreen: (full: boolean) => void
  setFocusMode: (on: boolean) => void
  updateViewportInfo: () => void
  viewportCollapsed: () => boolean
  viewportFullscreen: () => boolean
  focusMode: () => boolean
} {
  const {
    camera, renderer, controls, robot,
    viewportPanel, editorPanel, handle,
    originAxes, grid, comGroup, wireframeGroup, axisVisuals, jointAxisState,
    simBar, simActive, parsedRobot,
  } = deps

  // ── Viewport controls ────────────────────────────────────────────────────────

  const toggleAxesBtn = document.getElementById('toggle-axes') as HTMLButtonElement
  const toggleComBtn = document.getElementById('toggle-com') as HTMLButtonElement
  const toggleWireBtn = document.getElementById('toggle-wireframe') as HTMLButtonElement
  const toggleGridBtn = document.getElementById('toggle-grid') as HTMLButtonElement
  const toggleJointAxisBtn = document.getElementById('toggle-joint-axis') as HTMLButtonElement | null

  let axesVisible = true
  let gridVisible = true

  toggleAxesBtn.addEventListener('click', () => {
    axesVisible = !axesVisible
    originAxes.visible = axesVisible
    toggleAxesBtn.classList.toggle('active', axesVisible)
  })

  toggleJointAxisBtn?.addEventListener('click', () => {
    jointAxisState.visible = !jointAxisState.visible
    for (const obj of axisVisuals) obj.visible = jointAxisState.visible
    toggleJointAxisBtn.classList.toggle('active', jointAxisState.visible)
  })
  if (toggleJointAxisBtn) {
    for (const obj of axisVisuals) obj.visible = jointAxisState.visible
    toggleJointAxisBtn.classList.toggle('active', jointAxisState.visible)
  }

  toggleComBtn.addEventListener('click', () => {
    comGroup.visible = !comGroup.visible
    toggleComBtn.classList.toggle('active', comGroup.visible)
  })

  toggleWireBtn.addEventListener('click', () => {
    wireframeGroup.visible = !wireframeGroup.visible
    toggleWireBtn.classList.toggle('active', wireframeGroup.visible)
  })

  toggleGridBtn.classList.add('active')
  toggleGridBtn.addEventListener('click', () => {
    gridVisible = !gridVisible
    grid.visible = gridVisible
    toggleGridBtn.classList.toggle('active', gridVisible)
  })

  // ── Viewport zoom controls ──────────────────────────────────────────────────

  const viZoomIn = document.getElementById('vi-zoom-in') as HTMLButtonElement | null
  const viZoomOut = document.getElementById('vi-zoom-out') as HTMLButtonElement | null
  const viResetView = document.getElementById('vi-reset-view') as HTMLButtonElement | null

  const DEFAULT_CAM_POS = new THREE.Vector3(1.2, 1.0, 1.6)
  const DEFAULT_CAM_TARGET = new THREE.Vector3(0, 0.35, 0)

  function zoomCamera(factor: number) {
    const dir = new THREE.Vector3().subVectors(camera.position, controls.target)
    const newDist = Math.max(controls.minDistance, Math.min(controls.maxDistance, dir.length() * factor))
    dir.normalize().multiplyScalar(newDist)
    camera.position.copy(controls.target).add(dir)
    controls.update()
  }

  viZoomIn?.addEventListener('click', () => zoomCamera(0.75))
  viZoomOut?.addEventListener('click', () => zoomCamera(1.33))

  function focusOnRobot() {
    camera.position.copy(DEFAULT_CAM_POS)
    controls.target.copy(DEFAULT_CAM_TARGET)
    controls.update()
  }

  viResetView?.addEventListener('click', focusOnRobot)

  // ── Update viewport info ────────────────────────────────────────────────────

  function updateViewportInfo() {
    const viVerts = document.getElementById('vi-verts')
    const viFaces = document.getElementById('vi-faces')
    const viLinks = document.getElementById('vi-links')
    const viJoints = document.getElementById('vi-joints')

    const pr = parsedRobot()
    if (viVerts) viVerts.textContent = `Verts: ${pr.vertexCount.toLocaleString()}`
    if (viFaces) viFaces.textContent = `Faces: ${pr.faceCount.toLocaleString()}`
    if (viLinks) viLinks.textContent = `Links: ${pr.linkCount}`
    if (viJoints) viJoints.textContent = `Joints: ${pr.jointCount}`
  }

  updateViewportInfo()

  // ── Resize ───────────────────────────────────────────────────────────────────

  function resize() {
    const header = document.getElementById('viewport-header')!
    const w = viewportPanel.clientWidth
    const h = viewportPanel.clientHeight - header.offsetHeight - (simActive() ? simBar.offsetHeight : 0)
    if (w > 0 && h > 0) {
      renderer.setSize(w, h)
      camera.aspect = w / h
      camera.updateProjectionMatrix()
      deps.onResize?.(w, h)
    }
  }
  resize()
  window.addEventListener('resize', resize)

  // ── Viewport collapse toggle ────────────────────────────────────────────────

  let _viewportCollapsed = false
  let savedEditorWidth = '50%'
  const toggleViewportBtn = document.getElementById('toggle-viewport') as HTMLButtonElement
  const expandViewportBtn = document.getElementById('expand-viewport-btn') as HTMLButtonElement

  function setViewportCollapsed(collapsed: boolean) {
    _viewportCollapsed = collapsed

    if (collapsed) {
      savedEditorWidth = editorPanel.style.width || '50%'
      viewportPanel.classList.add('collapsed')
      handle.classList.add('vp-collapsed')
      editorPanel.classList.add('vp-collapsed')
      expandViewportBtn.classList.add('visible')
      toggleViewportBtn.classList.add('active')
    } else {
      expandViewportBtn.classList.remove('visible')
      viewportPanel.classList.remove('collapsed')
      handle.classList.remove('vp-collapsed')
      editorPanel.classList.remove('vp-collapsed')
      editorPanel.style.width = savedEditorWidth
      toggleViewportBtn.classList.remove('active')

      void viewportPanel.offsetHeight
      resize()
    }

    requestAnimationFrame(() => {
      if ((window as any).__vectorEditor) {
        (window as any).__vectorEditor.layout()
      }
    })
  }

  toggleViewportBtn.addEventListener('click', () => setViewportCollapsed(!_viewportCollapsed))
  expandViewportBtn.addEventListener('click', () => setViewportCollapsed(false))

  // ── Fullscreen 3D mode ──────────────────────────────────────────────────────

  let _viewportFullscreen = false
  const fullscreenViewportBtn = document.getElementById('toggle-fullscreen-viewport') as HTMLButtonElement | null
  const sidebarEl = document.getElementById('sidebar') as HTMLDivElement | null
  const activityBarEl = document.getElementById('activity-bar') as HTMLDivElement | null
  const sidebarHandle = document.getElementById('sidebar-resize-handle') as HTMLDivElement | null

  function setViewportFullscreen(full: boolean) {
    _viewportFullscreen = full
    if (full) {
      editorPanel.style.display = 'none'
      handle.style.display = 'none'
      if (sidebarEl) sidebarEl.style.display = 'none'
      if (activityBarEl) activityBarEl.style.display = 'none'
      if (sidebarHandle) sidebarHandle.style.display = 'none'
      viewportPanel.style.flex = '1'
      fullscreenViewportBtn?.classList.add('active')
    } else {
      // Only restore editor visibility if a file is actually open (editor-hidden absent)
      if (!editorPanel.classList.contains('editor-hidden')) editorPanel.style.display = ''
      if (!handle.classList.contains('editor-hidden')) handle.style.display = ''
      if (sidebarEl) sidebarEl.style.display = ''
      if (activityBarEl) activityBarEl.style.display = ''
      if (sidebarHandle) sidebarHandle.style.display = ''
      viewportPanel.style.flex = ''
      editorPanel.style.width = savedEditorWidth
      fullscreenViewportBtn?.classList.remove('active')
    }
    requestAnimationFrame(() => {
      resize()
      if ((window as any).__vectorEditor) (window as any).__vectorEditor.layout()
    })
  }

  fullscreenViewportBtn?.addEventListener('click', () => setViewportFullscreen(!_viewportFullscreen))

  // Double-click resize handle to toggle fullscreen
  handle.addEventListener('dblclick', () => setViewportFullscreen(!_viewportFullscreen))

  // ── Focus mode (sidebar + viewport, no editor) ────────────────────────────

  let _focusMode = false

  function setFocusMode(on: boolean) {
    if (on && _viewportFullscreen) setViewportFullscreen(false)
    if (on && _viewportCollapsed) setViewportCollapsed(false)
    _focusMode = on
    if (on) {
      savedEditorWidth = editorPanel.style.width || '50%'
      editorPanel.style.display = 'none'
      handle.style.display = 'none'
      viewportPanel.style.flex = '1'
    } else {
      // Only restore editor visibility if a file is actually open (editor-hidden absent)
      if (!editorPanel.classList.contains('editor-hidden')) editorPanel.style.display = ''
      if (!handle.classList.contains('editor-hidden')) handle.style.display = ''
      viewportPanel.style.flex = ''
      editorPanel.style.width = savedEditorWidth
    }
    requestAnimationFrame(() => {
      resize()
      if ((window as any).__vectorEditor) (window as any).__vectorEditor.layout()
    })
  }

  return {
    resize,
    focusOnRobot,
    zoomCamera,
    setViewportCollapsed,
    setViewportFullscreen,
    setFocusMode,
    updateViewportInfo,
    viewportCollapsed: () => _viewportCollapsed,
    viewportFullscreen: () => _viewportFullscreen,
    focusMode: () => _focusMode,
  }
}
