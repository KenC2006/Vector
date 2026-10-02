import './style.css'
import * as THREE from 'three'
import { OrbitControls } from 'three/addons/controls/OrbitControls.js'
import { OBJLoader } from 'three/addons/loaders/OBJLoader.js'
import { RoomEnvironment } from 'three/addons/environments/RoomEnvironment.js'
import { EffectComposer } from 'three/addons/postprocessing/EffectComposer.js'
import { RenderPass } from 'three/addons/postprocessing/RenderPass.js'
import { SMAAPass } from 'three/addons/postprocessing/SMAAPass.js'
import { OutputPass } from 'three/addons/postprocessing/OutputPass.js'
import { invoke } from '@tauri-apps/api/core'
import { initAssemblyEditor, type AssemblyEditorApi } from './assemblyEditor'
import { applyRichVisuals, preloadMeshCache } from './richVisuals'
import { componentIdForLink, setIdentitySource } from './design/identity'
import { getComponent, loadCatalog, presetBoundingBoxMm } from './design/catalog'
import { SAMPLE_URDF } from './sampleUrdf'
import { processXacro } from './xacro'
import { registerThemes, initSettings, VIEWPORT_BG, type ThemeId } from './settings'
import { initGitPanel } from './gitPanel'
import { initValidation, validateXMLStructure, validateURDFPerLink } from './validation'
import { parseURDFToScene, buildKinematicGraphFromURDF, setPathResolver, defaultMat } from './urdfParser'
import { rpyToQuat } from './rotationIO'
import type { ParsedRobot, KinematicLink, KinematicJoint } from './urdfParser'
import { initNodeGraph } from './nodeGraph'
import { initViewportControls } from './viewportControls'
import {
  applyInspectDimming,
  cameraPoseForBox,
  computeLinkWorldBox,
  restoreInspectMaterials,
  stepCameraFocusTween,
  type CameraFocusTween,
} from './inspectMode'
import { initChatHistory, type ChatHistoryApi } from './chatHistory'
import { initInlineDiff, type InlineDiffApi } from './inlineDiff'
import { initSimManager, type SimManagerApi } from './simManager'
import { initSimStage } from './simStage'
import { initViewportChat, type ViewportChatApi } from './viewportChat'

// Module-level API handles — initialized during startup sequence
let chatApi: ChatHistoryApi
let inlineDiffApi: InlineDiffApi
let simApi: SimManagerApi
let viewportChatApi: ViewportChatApi

// Viewport interaction state — declared early so simManager callbacks can reference it
let viewportInteractionMode: 'build' | 'inspect' = 'build'
let inspectFocusedLink: string | null = null

/**
 * Compute the lowest world-Y of the assembly's renderable URDF meshes,
 * skipping helpers (collision visuals, wireframes, attachment-node markers).
 * Returns null if no measurable geometry exists. Walks the tree explicitly
 * (not Box3.setFromObject) so deep serial chains don't get silently dropped
 * by stale-matrixWorld traversal — same rationale as `groundRobot` below.
 */
function computeLowestRenderedMeshY(robotGroup: THREE.Group): number | null {
  robotGroup.updateMatrixWorld(true)
  const urdfWorld = robotGroup.getObjectByName('urdf_world')
  const target = urdfWorld || robotGroup
  target.updateMatrixWorld(true)
  let minY = Infinity
  let meshCount = 0
  const tmpBox = new THREE.Box3()
  target.traverse((obj: THREE.Object3D) => {
    const mesh = obj as THREE.Mesh
    if (!mesh.isMesh || !mesh.visible) return
    const ud = mesh.userData as Record<string, unknown> | undefined
    if (ud?.isCollision) return
    const geom = mesh.geometry
    if (!geom) return
    if (!geom.boundingBox) geom.computeBoundingBox()
    const bb = geom.boundingBox
    if (!bb || bb.isEmpty()) return
    tmpBox.copy(bb).applyMatrix4(mesh.matrixWorld)
    if (tmpBox.min.y < minY) minY = tmpBox.min.y
    meshCount++
  })
  if (!isFinite(minY) || meshCount === 0) return null
  return minY
}

/**
 * Lift the assembly so its lowest mesh sits at or above Y=0. Only ever
 * RAISES — never lowers — so it's safe to call on every reparse without the
 * "jumps every edit" behavior that motivated keeping `groundRobot` reserved
 * for initial load. Mirrors the simulator's `_spawn_on_floor` so editor and
 * sim agree on what "on the floor" means.
 *
 * Why this exists: wheels render upright (after the carry/render parity fix)
 * and a wheel link's origin sits at the wheel center, so the tire extends
 * `tireRadius` below Y=0 of its joint origin. Without this lift, AI-generated
 * cars visually sank into the editor floor between the moment the URDF was
 * committed and the moment the user explicitly re-grounded.
 */
// Set whenever liftAboveFloor actually lifts. Cleared by reconcilePostMeshLoadFloor
// after async meshes settle. Used as a guard so we only re-ground when the lift
// was driven by a parser-time measurement (which may be stale if a GLB hadn't
// loaded yet) — never when the assembly is floating because the user dragged it.
let _liftAppliedSinceLastReground = false

function liftAboveFloor(robotGroup: THREE.Group) {
  // computeLowestRenderedMeshY returns world-space minY (mesh.matrixWorld already
  // includes robotGroup.position.y), so it IS the world floor — don't add the
  // group's Y again. Snap to floor in either direction; without the lower path,
  // a wholesale content swap inherits the previous URDF's lift and floats.
  const worldFloorY = computeLowestRenderedMeshY(robotGroup)
  if (worldFloorY === null) return
  if (Math.abs(worldFloorY) < 0.001) return
  robotGroup.position.y -= worldFloorY
  robotGroup.updateMatrixWorld(true)
  urdfAssemblyApi?.refreshOverlay()
  _liftAppliedSinceLastReground = true
}

/** Called once after async mesh loads settle. If liftAboveFloor over-lifted using
 *  a parametric fallback (because the real GLB wasn't cached yet), the assembly
 *  ends up floating above floor with no chance to re-settle — liftAboveFloor only
 *  raises. Here we re-measure with the now-loaded geometry and lower the assembly
 *  back down to the floor when needed. Guarded by _liftAppliedSinceLastReground
 *  so user-positioned floating designs aren't disturbed. */
function reconcilePostMeshLoadFloor(robotGroup: THREE.Group) {
  if (!_liftAppliedSinceLastReground) return
  const worldFloorY = computeLowestRenderedMeshY(robotGroup)
  if (worldFloorY === null) return
  // Only correct meaningful drift (> 1mm). Floor lower only — never raises here
  // (liftAboveFloor handles raising on its own).
  if (worldFloorY > 0.001) {
    robotGroup.position.y -= worldFloorY
    robotGroup.updateMatrixWorld(true)
    urdfAssemblyApi?.refreshOverlay()
  }
  _liftAppliedSinceLastReground = false
}

/**
 * Raise the assembly root group so the lowest geometry point touches Y=0.
 * Call when loading / switching URDF documents or after reset — not on every edit reparse,
 * or the whole robot jumps whenever a new part dips below the floor plane.
 */
function groundRobot(robotGroup: THREE.Group) {
  // Reset Y first so bbox is measured from neutral position
  robotGroup.position.y = 0
  robotGroup.updateMatrixWorld(true)
  // Only measure the URDF world group — exclude wireframe overlays, CoM markers,
  // and other helpers that are children of robotGroup but not actual robot geometry.
  const urdfWorld = robotGroup.getObjectByName('urdf_world')
  const target = urdfWorld || robotGroup
  // Defensive: re-update target's matrices in case async mesh adds happened
  // after robotGroup.updateMatrixWorld but before we got here.
  target.updateMatrixWorld(true)

  // Walk the full scene graph under target. Computing each mesh's world-space AABB
  // from its geometry.boundingBox + matrixWorld explicitly (instead of Box3.setFromObject)
  // avoids silently missing components deep in serial chains when setFromObject's limited
  // (false, false) world-matrix refresh can't recurse — a previous bug where robots floated
  // because the lowest mesh was buried in servo→servo→extrusion chains.
  let minY = Infinity
  let meshCount = 0
  const tmpBox = new THREE.Box3()
  target.traverse((obj: THREE.Object3D) => {
    const mesh = obj as THREE.Mesh
    if (!mesh.isMesh || !mesh.visible) return
    // Skip collision-visual meshes — they share linkGroups with real geometry but
    // shouldn't influence ground offset (especially when toggled visible).
    const ud = mesh.userData as Record<string, unknown> | undefined
    if (ud?.isCollision) return
    const geom = mesh.geometry
    if (!geom) return
    if (!geom.boundingBox) geom.computeBoundingBox()
    const bb = geom.boundingBox
    if (!bb || bb.isEmpty()) return
    tmpBox.copy(bb).applyMatrix4(mesh.matrixWorld)
    if (tmpBox.min.y < minY) {
      minY = tmpBox.min.y
    }
    meshCount++
  })
  if (!isFinite(minY) || meshCount === 0) return
  // In Three.js Y is up; shift so lowest mesh touches Y=0
  robotGroup.position.y = -minY
  // Mount-node meshes live in a separate scene group (not under robotGroup), so
  // their cached world positions from rebuildMountNodes() become stale after this
  // shift — AI-generated robots showed attachment nodes buried under the floor.
  robotGroup.updateMatrixWorld(true)
  urdfAssemblyApi?.refreshOverlay()
}

/**
 * Frame the viewport camera on the assembled robot using its bounding box.
 * Call after assembly or file-open — NOT on every edit reparse (disorienting).
 * Uses the same maxDim * 2.5 heuristic as the offscreen screenshot camera.
 */
function autoFrameRobot(
  robotGroup: THREE.Group,
  cam: THREE.PerspectiveCamera,
  orbitControls: OrbitControls,
) {
  robotGroup.updateMatrixWorld(true)
  const box = new THREE.Box3().setFromObject(robotGroup)
  if (box.isEmpty()) return
  const center = new THREE.Vector3()
  const size = new THREE.Vector3()
  box.getCenter(center)
  box.getSize(size)
  const maxDim = Math.max(size.x, size.y, size.z, 0.1)
  const dist = maxDim * 2.5
  // Front-right isometric view direction (matches offscreen screenshot camera)
  const dir = new THREE.Vector3(0.75, 0.6, 0.75).normalize()
  cam.position.copy(center).addScaledVector(dir, dist)
  orbitControls.target.copy(center)
  orbitControls.update()
}

// Wait for Tauri IPC bridge to be ready (injected async by Tauri)
function waitForTauri(timeoutMs = 5000): Promise<void> {
  return new Promise((resolve, reject) => {
    if ((window as any).__TAURI_INTERNALS__) {
      resolve()
      return
    }
    const start = Date.now()
    const interval = setInterval(() => {
      if ((window as any).__TAURI_INTERNALS__) {
        clearInterval(interval)
        resolve()
      } else if (Date.now() - start > timeoutMs) {
        clearInterval(interval)
        reject(new Error('Tauri IPC not available — are you running inside cargo tauri dev?'))
      }
    }, 50)
  })
}

// Safe invoke that ensures Tauri IPC is ready before calling. The core
// auto-start (and other early IPC) can fire before __TAURI_INTERNALS__ is injected.
async function safeInvoke<T>(cmd: string, args: Record<string, unknown>): Promise<T> {
  await waitForTauri()
  return invoke(cmd, args) as Promise<T>
}

// Invoke with a timeout — rejects if the call takes too long.
function invokeWithTimeout<T>(cmd: string, args: Record<string, unknown>, timeoutMs: number): Promise<T> {
  return Promise.race([
    safeInvoke<T>(cmd, args),
    new Promise<never>((_, reject) =>
      setTimeout(() => reject(new Error(`${cmd} timed out after ${timeoutMs}ms`)), timeoutMs)
    )
  ])
}


// ── Monaco Editor ────────────────────────────────────────────────────────────

import * as monaco from 'monaco-editor'
import editorWorker from 'monaco-editor/esm/vs/editor/editor.worker?worker'
import jsonWorker from 'monaco-editor/esm/vs/language/json/json.worker?worker'

// Configure Monaco web workers
self.MonacoEnvironment = {
  getWorker(_workerId: string, label: string) {
    if (label === 'json') return new jsonWorker()
    return new editorWorker()
  },
}

const cursorPos = document.getElementById('cursor-pos') as HTMLSpanElement
const monacoContainer = document.getElementById('monaco-container') as HTMLDivElement

// Register custom Monaco themes before editor creation
registerThemes(monaco)

// Create Monaco models — start empty
const monacoModels: Record<string, monaco.editor.ITextModel> = {}

// Create Monaco editor instance
const monacoEditor = monaco.editor.create(monacoContainer, {
  model: null,
  theme: 'vector-dark',
  fontSize: 13,
  fontFamily: "'JetBrains Mono', 'Fira Code', 'Cascadia Code', Consolas, monospace",
  fontLigatures: true,
  lineNumbers: 'on',
  minimap: { enabled: true, maxColumn: 80, renderCharacters: false },
  scrollBeyondLastLine: false,
  smoothScrolling: true,
  cursorBlinking: 'smooth',
  cursorSmoothCaretAnimation: 'on',
  renderLineHighlight: 'line',
  automaticLayout: true,
  tabSize: 2,
  insertSpaces: true,
  wordWrap: 'off',
  bracketPairColorization: { enabled: false },
  guides: { indentation: true, bracketPairs: false, highlightActiveBracketPair: false, bracketPairsHorizontal: false },
  padding: { top: 8, bottom: 8 },
  overviewRulerLanes: 2,
  scrollbar: {
    verticalScrollbarSize: 8,
    horizontalScrollbarSize: 8,
  },
  suggest: { showWords: false },
  quickSuggestions: { other: true, strings: true, comments: false },
  inlineSuggest: { enabled: true },
})

// ── Chat History (delegated to chatHistory.ts) ──────────────────────────────

chatApi = initChatHistory({
  getEditorValue: () => monacoEditor.getModel()?.getValue() || '',
  setEditorValue: (v) => monacoEditor.setValue(v),
})

// Update cursor position in status bar
monacoEditor.onDidChangeCursorPosition((e) => {
  cursorPos.textContent = `Ln ${e.position.lineNumber}, Col ${e.position.column}`
})

// Debounced validation on editor content changes
let validationDebounceTimer: ReturnType<typeof setTimeout> | null = null
monacoEditor.onDidChangeModelContent(() => {
  // Only validate URDF files
  if (activeFile !== 'robot.urdf') return

  // Clear existing timer
  if (validationDebounceTimer) clearTimeout(validationDebounceTimer)

  // Set new timer to run local validation after 1 second of inactivity
  // Uses client-side only to avoid blocking the Mutex that AI completions need
  validationDebounceTimer = setTimeout(() => {
    runLocalValidation()
  }, 1000)
})

// Expose the actual Monaco editor instance on window for inline diff and other modules
;(window as any).__vectorEditor = monacoEditor

// ── Tab & File Management ───────────────────────────────────────────────────

let activeFile = ''
const fileTypeLabel = document.getElementById('file-type') as HTMLSpanElement
const tabBar = document.getElementById('tab-bar') as HTMLDivElement
const tabNewBtn = tabBar.querySelector('.tab-new') as HTMLButtonElement
const filesList = document.getElementById('files-list') as HTMLDivElement | null

// Track open files and their paths
let openedFolderPath: string | null = null
const openFiles: string[] = []
const filePaths: Record<string, string | null> = {} // filename → disk path (null = unsaved)
const viewStates: Record<string, monaco.editor.ICodeEditorViewState | null> = {}
const cameraStates: Record<string, { pos: [number, number, number]; target: [number, number, number] }> = {}
interface TabRobotCache {
  parsedRobot: ParsedRobot
  kinematicGraph: Record<string, KinematicLink>
  kinematicJoints: Record<string, KinematicJoint>
  parsedContent: string
}
const tabRobotCache: Record<string, TabRobotCache> = {}
// Buffer keys whose Monaco content has changed since last save. Drives the
// dirty-dot indicator on tabs + tree rows. Untitled buffers are always dirty
// once they have any content (they have nothing on disk to compare to).
const dirtyBuffers = new Set<string>()
// Per-file undo/redo state — saved when leaving a file, restored when returning.
const fileUndoStates: Record<string, { undo: string[]; redo: string[] }> = {}
let untitledCounter = 0
// Per-launch tag stamped into every untitled buffer key. Untitled buffers are
// not persisted to disk, but per-file state keyed off them (chat history,
// inline diff stash) IS persisted to localStorage. Without a per-launch tag,
// next session reuses `untitled:1`, `untitled:2`, etc. — and inherits the
// stale chat bucket left behind by a previous run's discarded buffer.
const UNTITLED_SESSION_TAG = Math.random().toString(36).slice(2, 8)
// Extract the trailing counter from an untitled buffer key for display.
// Keys are `untitled:<sessionTag>:<N>`; the user only ever wants to see N.
function untitledDisplayN(key: string): string {
  const parts = key.split(':')
  return parts[parts.length - 1] || key
}

function getFileExt(filename: string): string {
  const dot = filename.lastIndexOf('.')
  return dot >= 0 ? filename.slice(dot + 1).toLowerCase() : ''
}

function getFileType(filename: string): string {
  const ext = getFileExt(filename)
  const types: Record<string, string> = { urdf: 'URDF', xacro: 'XACRO', xml: 'XML', json: 'JSON', yaml: 'YAML', yml: 'YAML', sdf: 'SDF', mjcf: 'MJCF', txt: 'TEXT' }
  return types[ext] || 'TEXT'
}

function isUrdfLike(filename: string): boolean {
  const ext = getFileExt(filename)
  return ['urdf', 'xacro', 'xml', 'sdf', 'mjcf'].includes(ext)
}

// Buffer-key-aware variant. Buffer keys are either an absolute path (the
// extension is part of the key — `isUrdfLike` works directly) or a synthetic
// `untitled:N` for unsaved buffers (no extension, so the plain check would
// return false). For untitled keys, default to URDF — we always inject
// SAMPLE_URDF as content on creation, so the buffer is parseable URDF.
function isUrdfLikeBuffer(key: string): boolean {
  if (key.startsWith('untitled:')) return true
  return isUrdfLike(key)
}

function getMonacoLang(filename: string): string {
  const ext = getFileExt(filename)
  const langs: Record<string, string> = { urdf: 'xml', xacro: 'xml', xml: 'xml', json: 'json', yaml: 'yaml', yml: 'yaml', sdf: 'xml', mjcf: 'xml' }
  return langs[ext] || 'plaintext'
}

// Buffer key → display label, with parent-folder disambiguation when two
// open buffers share the same basename. Untitled buffers display their key.
function bufferLabel(key: string): string {
  if (key.startsWith('untitled:')) return untitledDisplayN(key)
  const base = key.split(/[\\/]/).pop() || key
  // Disambiguate against other open buffers with the same basename
  const collisions = openFiles.filter(k => {
    if (k === key) return false
    const b = k.startsWith('untitled:') ? untitledDisplayN(k) : (k.split(/[\\/]/).pop() || k)
    return b === base
  })
  if (collisions.length === 0) return base
  // Append parent folder for disambiguation
  const parts = key.split(/[\\/]/)
  const parent = parts.length >= 2 ? parts[parts.length - 2] : ''
  return parent ? `${base} — ${parent}` : base
}

function renderTabs() {
  // Remove all existing tab elements (keep the + button)
  tabBar.querySelectorAll('.tab').forEach(t => t.remove())

  for (const key of openFiles) {
    const tab = document.createElement('div')
    tab.className = 'tab' + (key === activeFile ? ' active' : '')
    tab.dataset.file = key
    const label = bufferLabel(key)
    const isDirty = dirtyBuffers.has(key)
    tab.classList.toggle('dirty', isDirty)
    tab.innerHTML = `
      <span class="tab-label" title="${key.replace(/"/g, '&quot;')}">${label}</span>
      <span class="tab-dirty" title="Unsaved changes">&#9679;</span>
      <span class="tab-close" title="Close">&times;</span>
    `
    tab.addEventListener('click', (e) => {
      if ((e.target as HTMLElement).classList.contains('tab-close')) {
        closeFile(key)
      } else {
        switchToFile(key)
      }
    })
    tabBar.insertBefore(tab, tabNewBtn)
  }
}

function renderExplorer() {
  if (!filesList) return
  // If a folder is open, render the workspace tree (always full re-render so
  // active highlights track open buffers without separate update logic).
  if (openedFolderPath) {
    renderWorkspaceTree()
    return
  }
  // No folder open — show open buffers + recently-closed disk files so the
  // user can re-open a tab they X'd without going back through the file dialog.
  filesList.innerHTML = ''

  // Section 1: currently-open buffers
  for (const key of openFiles) {
    const label = bufferLabel(key)
    const ext = getFileExt(label)
    const isDirty = dirtyBuffers.has(key)
    const item = document.createElement('div')
    item.className = 'file-item open' + (key === activeFile ? ' active' : '') + (isDirty ? ' dirty' : '')
    item.innerHTML = `<span class="fi-dot ${ext}"></span>${label}${isDirty ? '<span class="fi-dirty" title="Unsaved changes">&#9679;</span>' : ''}`
    item.title = key
    item.addEventListener('click', () => switchToFile(key))
    filesList.appendChild(item)
  }

  // Section 2: recent disk-backed files not currently open. Click to reload
  // from disk. Hover-X removes the entry from the recent list (doesn't touch
  // disk).
  const openPaths = new Set(openFiles.filter(k => !k.startsWith('untitled:')))
  const closedRecents = recentFiles.filter(r => !openPaths.has(r.path))
  if (closedRecents.length > 0 && openFiles.length > 0) {
    const sep = document.createElement('div')
    sep.className = 'fi-section-sep'
    sep.textContent = 'Recent'
    filesList.appendChild(sep)
  }
  for (const rf of closedRecents) {
    const ext = rf.name.split('.').pop()?.toLowerCase() || ''
    const item = document.createElement('div')
    item.className = 'file-item recent'
    item.innerHTML = `<span class="fi-dot ${ext}"></span>${rf.name}<span class="fi-remove" title="Remove from list">&times;</span>`
    item.title = rf.path
    item.addEventListener('click', async (e) => {
      if ((e.target as HTMLElement).classList.contains('fi-remove')) {
        e.stopPropagation()
        recentFiles = recentFiles.filter(r => r.path !== rf.path)
        localStorage.setItem('vector_recent_files', JSON.stringify(recentFiles))
        renderRecentFiles()
        renderExplorer()
        return
      }
      try {
        const probe = await invoke<{ exists: boolean; isDir: boolean }>('path_exists', { path: rf.path })
        if (!probe.exists) {
          showToast(`File no longer exists: ${rf.name}`, 'warning')
          recentFiles = recentFiles.filter(r => r.path !== rf.path)
          localStorage.setItem('vector_recent_files', JSON.stringify(recentFiles))
          renderRecentFiles()
          renderExplorer()
          return
        }
        const content = await invoke<string>('open_file', { path: rf.path })
        createNewFile(rf.name, content, rf.path)
        currentFilePath = rf.path
      } catch (err) {
        showToast(`Failed to open ${rf.name}: ${err}`, 'error')
      }
    })
    filesList.appendChild(item)
  }
}

function switchToFile(filename: string) {
  if (filename === activeFile) return
  if (!monacoModels[filename]) return

  // Block tab switching while the AI is mid-generation. Each file owns its
  // own chat session, pending diff, and assembly state — a switch mid-flight
  // would either silently abandon the in-progress edit or land its result on
  // the wrong file. Cleaner to make the user wait for Accept/Dismiss.
  if (document.body.classList.contains('ai-busy')) {
    showToast('Wait for the AI to finish (or Cancel) before switching files', 'warning')
    return
  }

  // Save current 3D state so we can restore it when switching back
  if (activeFile && isUrdfLikeBuffer(activeFile)) {
    tabRobotCache[activeFile] = {
      parsedRobot,
      kinematicGraph,
      kinematicJoints,
      parsedContent: monacoEditor.getModel()?.getValue() || '',
    }
    // Save per-file undo/redo stack before leaving this file
    if (urdfAssemblyApi) fileUndoStates[activeFile] = urdfAssemblyApi.getUndoState()
  }

  // Save current view state (editor + camera)
  viewStates[activeFile] = monacoEditor.saveViewState()
  if (activeFile) {
    cameraStates[activeFile] = {
      pos: [camera.position.x, camera.position.y, camera.position.z],
      target: [controls.target.x, controls.target.y, controls.target.z],
    }
  }

  activeFile = filename
  saveOpenTabsState()
  fileTypeLabel.textContent = getFileType(filename)

  // Pending inline diffs are now stashed per-file by setActiveFile() at the
  // end of this function — don't clear them here. Just drop the debounced
  // reparse from the previous file so the new file's first render isn't
  // racing it.
  if (reparseTimeout !== null) {
    clearTimeout(reparseTimeout)
    reparseTimeout = null
  }

  hideWelcomeState()

  // Switch Monaco model
  const model = monacoModels[filename]
  if (model) {
    monacoEditor.setModel(model)
    const savedState = viewStates[filename]
    if (savedState) monacoEditor.restoreViewState(savedState)
  }

  renderTabs()
  renderExplorer()

  // Update breadcrumb
  const bcFilename = document.getElementById('bc-filename')
  if (bcFilename) bcFilename.textContent = filename

  // Update title
  const path = filePaths[filename]
  document.title = path ? `Vector — ${filename}` : `Vector — ${filename} (unsaved)`

  monacoEditor.focus()

  // Update 3D viewport if switching to a URDF/XML file. Use the buffer-key-
  // aware variant — synthetic `untitled:N` keys don't end in .urdf but the
  // buffer always carries SAMPLE_URDF content, so we still want the parse +
  // viewport refresh path. Without this, clicking "+" leaves the previous
  // tab's robot stuck on screen because the entire 3D-update block is
  // skipped for untitled buffers.
  if (isUrdfLikeBuffer(filename)) {
    const currentContent = monacoModels[filename].getValue()
    const cached = tabRobotCache[filename]

    if (cached && cached.parsedContent === currentContent) {
      // Content unchanged — restore cached 3D state without reparsing
      worldGroup.remove(parsedRobot.group)
      wireframeGroup.clear()
      axisVisuals.length = 0
      parsedRobot = cached.parsedRobot
      kinematicGraph = cached.kinematicGraph
      kinematicJoints = cached.kinematicJoints
      robot.position.set(0, 0, 0)
      worldGroup.add(parsedRobot.group)
      groundRobot(robot)
      rebuildWireframes()
      rebuildJointAxisVisuals()
      updateComMarker()
      rebuildCollisionVisuals(currentContent)
      updateViewportInfo()
      urdfAssemblyApi?.onModelUpdated()
      runLocalValidation()
    } else {
      // First visit or content changed — full reparse. Clear EVERYTHING from
      // the scene's robot group up-front: not just `parsedRobot.group` (the
      // pristine parser output) but every child added by applyRichVisuals,
      // mesh loaders, mate connectors, or carry previews. Otherwise a fresh
      // tab opened via "+" inherits the previous tab's visible geometry when
      // the new content's parsedRobot.group is empty or doesn't fully
      // overlap. Manually clearing the whole worldGroup guarantees a blank
      // slate before reparseURDF rebuilds.
      while (worldGroup.children.length > 0) {
        worldGroup.remove(worldGroup.children[0])
      }
      wireframeGroup.clear()
      axisVisuals.length = 0
      robot.position.set(0, 0, 0)
      // Pass `currentContent` explicitly instead of letting reparseURDF read
      // from monacoEditor.getModel(). For a brand-new tab opened via "+", the
      // model swap is synchronous but the editor's getValue() can still
      // briefly return stale content if other listeners fire in between —
      // pulling the string from monacoModels[filename] directly sidesteps it.
      reparseURDF(currentContent, { ground: true })
      urdfAssemblyApi?.onModelUpdated()
      runLocalValidation()
    }

    // Restore per-file undo/redo stack (or clear it for a brand-new file)
    if (urdfAssemblyApi) {
      urdfAssemblyApi.restoreUndoState(fileUndoStates[filename] ?? { undo: [], redo: [] })
    }

    // Restore saved camera or auto-frame
    const savedCam = cameraStates[filename]
    if (savedCam) {
      camera.position.set(...savedCam.pos)
      controls.target.set(...savedCam.target)
      controls.update()
    } else {
      // First time viewing this file — auto-frame
      setTimeout(focusOnRobot, 100) // slight delay for meshes to load
    }
  }

  // Swap the chat panel scope to this file. chatApi flushes the previous
  // file's chat list back to its bucket and hydrates from this file's bucket,
  // creating a fresh chat if none exists. Inline-diff state is similarly
  // stashed per-file so a pending Accept/Dismiss survives a tab round-trip.
  chatApi?.setActiveFile(filename)
  inlineDiffApi?.setActiveFile(filename)
}

// ── Recent files ────────────────────────────────────────────────────────────

const MAX_RECENT = 5
let recentFiles: Array<{ name: string; path: string }> = JSON.parse(localStorage.getItem('vector_recent_files') || '[]')

function addRecentFile(name: string, path: string) {
  recentFiles = recentFiles.filter(r => r.path !== path)
  recentFiles.unshift({ name, path })
  if (recentFiles.length > MAX_RECENT) recentFiles.pop()
  localStorage.setItem('vector_recent_files', JSON.stringify(recentFiles))
  renderRecentFiles()
}

function renderRecentFiles() {
  const container = document.getElementById('welcome-recent')
  if (!container) return
  if (recentFiles.length === 0) {
    container.innerHTML = ''
    return
  }
  let html = '<div class="welcome-recent-title">Recent</div>'
  for (const file of recentFiles) {
    const shortPath = file.path.length > 50 ? '...' + file.path.slice(-47) : file.path
    html += `<div class="welcome-recent-item" data-path="${file.path.replace(/"/g, '&quot;')}" title="${file.path.replace(/"/g, '&quot;')}">
      <span class="welcome-recent-name">${file.name}</span>
      <span class="welcome-recent-path">${shortPath}</span>
    </div>`
  }
  container.innerHTML = html

  // Wire click handlers
  container.querySelectorAll('.welcome-recent-item').forEach(item => {
    item.addEventListener('click', async () => {
      const path = (item as HTMLElement).dataset.path
      if (!path) return
      try {
        const content = await invoke<string>('open_file', { path })
        const name = path.split(/[\\/]/).pop() || 'file'
        createNewFile(name, content, path)
        currentFilePath = path
      } catch (err) {
        showToast(`Failed to open: ${err}`, 'error')
      }
    })
  })
}

// Initial render
renderRecentFiles()

function createNewFile(filename?: string, content = '', diskPath: string | null = null) {
  // Buffer key: full disk path when available, synthetic "untitled:N" otherwise.
  // This lets two files with the same basename in different folders coexist as
  // distinct tabs, and decouples in-memory buffer identity from display label.
  let key: string
  let displayName: string
  if (diskPath) {
    key = diskPath
    displayName = diskPath.split(/[\\/]/).pop() || filename || 'file'
  } else {
    untitledCounter++
    key = `untitled:${UNTITLED_SESSION_TAG}:${untitledCounter}`
    displayName = filename || `untitled_${untitledCounter}.urdf`
  }

  // New URDF buffers need *some* parseable URDF so the 3D pipeline doesn't
  // either error out (leaving stale geometry on screen) or auto-validate
  // against an empty model. Inject SAMPLE_URDF, but stamp it with a
  // per-buffer robot name so two open untitled tabs are visually
  // distinguishable in the viewport (otherwise every fresh "+" tab looks
  // identical to the previous one and the user thinks the new tab inherited
  // the old one's state).
  if (!content && (isUrdfLike(displayName))) {
    const stamp = key.startsWith('untitled:')
      ? `untitled_${untitledDisplayN(key)}`
      : (displayName.replace(/\.[^.]+$/, '') || 'robot')
    content = SAMPLE_URDF.replace(/<robot\s+name="[^"]*"/, `<robot name="${stamp}"`)
  }

  // If buffer already open for this key, just switch to it
  if (monacoModels[key]) {
    switchToFile(key)
    return key
  }

  const lang = getMonacoLang(displayName)
  monacoModels[key] = monaco.editor.createModel(content, lang)
  openFiles.push(key)
  filePaths[key] = diskPath

  // Track in recent files
  if (diskPath) addRecentFile(displayName, diskPath)

  // Untitled buffers start dirty (they have content but nothing on disk yet).
  if (!diskPath) {
    dirtyBuffers.add(key)
  }

  // Mark buffer dirty on any content change (applies to all file types).
  {
    const fn = key
    const fnModel = monacoModels[key]
    fnModel.onDidChangeContent(() => {
      if (!dirtyBuffers.has(fn)) {
        dirtyBuffers.add(fn)
        renderTabs()
        renderExplorer()
      }
    })
  }

  // Listen for changes on URDF/XML files with debounce
  if (isUrdfLike(displayName)) {
    const fn = key // capture for closure
    const fnModel = monacoModels[key] // capture model reference for closure
    fnModel.onDidChangeContent(() => {
      if (activeFile === fn) {
        // Hot-reload guard: block reparse while sim is running — edits would
        // diverge from the loaded MJCF. User must restart sim to apply changes.
        if (simApi.isSimActive()) {
          showToast('Editor changed — restart simulation to apply', 'warning')
          return
        }
        if (reparseTimeout !== null) clearTimeout(reparseTimeout)
        reparseTimeout = window.setTimeout(() => {
          reparseTimeout = null
          // Suppress auto-reparse while an AI inline diff is pending —
          // the explicit reparseURDF() in acceptInlineDiff/dismissInlineDiff handles it.
          if (inlineDiffApi.getPendingOldText() !== null) return
          // Only reparse if this file is STILL active (user may have switched tabs during debounce)
          if (activeFile !== fn) return
          reparseURDF()
          urdfAssemblyApi?.onModelUpdated()
        }, 500)
      }
    })
  }

  switchToFile(key)
  renderTabs()
  renderExplorer()
  saveOpenTabsState()
  return key
}

function showWelcomeState() {
  activeFile = ''
  monacoEditor.setModel(null)
  editorPanel.classList.add('editor-hidden')
  handle.classList.add('editor-hidden')
  const bcFilename = document.getElementById('bc-filename')
  if (bcFilename) bcFilename.textContent = ''
  // No file open → AI chat has no buffer to operate on. Freeze the chat tab
  // (and bounce off it if it's the active view) until a file is opened.
  viewportChatApi?.setChatEnabled(false)
  renderTabs()
  renderExplorer()
  // Viewport expands to fill the space — update renderer after layout settles
  requestAnimationFrame(() => _resize())
}

function hideWelcomeState() {
  viewportChatApi?.setChatEnabled(true)
  editorPanel.classList.remove('editor-hidden')
  handle.classList.remove('editor-hidden')
  if (!editorPanel.style.width) editorPanel.style.width = '50%'
  // Force reflow so clientWidth is accurate before resize
  void editorPanel.offsetWidth
  requestAnimationFrame(() => {
    _resize()
    ;(window as any).__vectorEditor?.layout()
  })
}

function closeFile(filename: string) {
  const idx = openFiles.indexOf(filename)
  if (idx < 0) return

  // Remove from tracking
  openFiles.splice(idx, 1)
  delete viewStates[filename]
  delete cameraStates[filename]
  delete filePaths[filename]

  // Dispose Monaco model
  const model = monacoModels[filename]
  if (model) {
    model.dispose()
    delete monacoModels[filename]
  }

  // Dispose cached 3D state (if it's not the currently live parsedRobot)
  const cached3d = tabRobotCache[filename]
  if (cached3d && cached3d.parsedRobot !== parsedRobot) {
    cached3d.parsedRobot.group.traverse(obj => {
      if ((obj as THREE.Mesh).geometry) (obj as THREE.Mesh).geometry.dispose()
      if ((obj as THREE.Mesh).material) {
        const mat = (obj as THREE.Mesh).material
        if (Array.isArray(mat)) mat.forEach(m => m.dispose())
        else mat.dispose()
      }
    })
  }
  delete tabRobotCache[filename]
  dirtyBuffers.delete(filename)

  // Switch to adjacent tab or show welcome if no files left
  if (filename === activeFile) {
    if (openFiles.length > 0) {
      const newIdx = Math.min(idx, openFiles.length - 1)
      activeFile = '' // force switch
      switchToFile(openFiles[newIdx])
    } else {
      showWelcomeState()
    }
  }

  renderTabs()
  renderExplorer()
  saveOpenTabsState()
}

/**
 * Migrate every per-buffer map entry from `oldKey` to `newKey`. Used when an
 * untitled buffer is saved to disk for the first time (key changes from
 * `untitled:N` to the absolute path) so that tabs, tree highlights, undo
 * stacks, and 3D caches don't get orphaned. Monaco's model identity is
 * preserved — we move the existing model under the new key rather than
 * disposing it, so undo history survives.
 */
function rekeyBuffer(oldKey: string, newKey: string) {
  if (oldKey === newKey) return
  const idx = openFiles.indexOf(oldKey)
  if (idx >= 0) openFiles[idx] = newKey
  if (monacoModels[oldKey]) { monacoModels[newKey] = monacoModels[oldKey]; delete monacoModels[oldKey] }
  if (oldKey in viewStates) { viewStates[newKey] = viewStates[oldKey]; delete viewStates[oldKey] }
  if (oldKey in cameraStates) { cameraStates[newKey] = cameraStates[oldKey]; delete cameraStates[oldKey] }
  if (oldKey in tabRobotCache) { tabRobotCache[newKey] = tabRobotCache[oldKey]; delete tabRobotCache[oldKey] }
  if (oldKey in fileUndoStates) { fileUndoStates[newKey] = fileUndoStates[oldKey]; delete fileUndoStates[oldKey] }
  if (oldKey in filePaths) { filePaths[newKey] = filePaths[oldKey]; delete filePaths[oldKey] }
  if (dirtyBuffers.has(oldKey)) { dirtyBuffers.delete(oldKey); dirtyBuffers.add(newKey) }
  if (activeFile === oldKey) activeFile = newKey
}

/**
 * Persist the disk-backed open buffers + active key so the next launch can
 * restore them. Untitled buffers are intentionally skipped — they have no
 * disk identity and would either need their content serialized to
 * localStorage (which can quickly blow the quota for big URDFs) or come back
 * as empty stubs that confuse the user. Saving an untitled buffer first
 * promotes it to a path-keyed buffer, after which it WILL be restored.
 */
function saveOpenTabsState() {
  try {
    const diskBacked = openFiles.filter(k => !k.startsWith('untitled:') && filePaths[k])
    const state = { paths: diskBacked, active: diskBacked.includes(activeFile) ? activeFile : null }
    localStorage.setItem('vector_open_tabs', JSON.stringify(state))
  } catch { /* localStorage quota — best effort */ }
}

/** Re-fetch the workspace tree from disk and re-render the explorer. */
async function refreshTree() {
  if (!openedFolderPath) return
  try {
    const entries = await invoke<Array<{ name: string; path: string; isDir: boolean; depth: number }>>(
      'list_directory', { path: openedFolderPath }
    )
    workspaceTreeEntries = entries
    renderWorkspaceTree()
  } catch (err) {
    showToast(`Failed to refresh tree: ${err}`, 'error')
  }
}

// Tracks the entries returned by the most recent list_directory call so we can
// re-render the tree on highlight changes without re-fetching.
let workspaceTreeEntries: Array<{ name: string; path: string; isDir: boolean; depth: number }> = []
const expandedFolders = new Set<string>()
// Children loaded on demand for each expanded folder path.
const folderChildren: Record<string, Array<{ name: string; path: string; isDir: boolean; depth: number }>> = {}

/** Currently focused tree path for context-menu actions. */
let contextMenuTarget: { path: string; isDir: boolean } | null = null

function renderWorkspaceTree() {
  if (!filesList || !openedFolderPath) return
  filesList.innerHTML = ''

  // Recursively render entries with their lazy-loaded children inserted in line.
  const renderEntry = (entry: { name: string; path: string; isDir: boolean; depth: number }) => {
    const el = document.createElement('div')
    const ext = entry.name.split('.').pop()?.toLowerCase() || ''
    el.style.paddingLeft = `${12 + entry.depth * 14}px`
    el.dataset.path = entry.path
    el.dataset.isDir = entry.isDir ? '1' : '0'
    if (entry.isDir) {
      const expanded = expandedFolders.has(entry.path)
      el.className = 'file-item folder' + (expanded ? ' expanded' : '')
      el.innerHTML = `<span class="fi-arrow">${expanded ? '&#9662;' : '&#9656;'}</span>${entry.name}/`
      el.addEventListener('click', async (e) => {
        e.stopPropagation()
        if (expanded) {
          expandedFolders.delete(entry.path)
        } else {
          expandedFolders.add(entry.path)
          if (!folderChildren[entry.path]) {
            try {
              const children = await invoke<Array<{ name: string; path: string; isDir: boolean; depth: number }>>(
                'list_directory', { path: entry.path }
              )
              // Stamp depth relative to root
              folderChildren[entry.path] = children.map(c => ({ ...c, depth: entry.depth + 1 }))
            } catch (err) {
              showToast(`Failed to list ${entry.name}: ${err}`, 'error')
              expandedFolders.delete(entry.path)
            }
          }
        }
        renderWorkspaceTree()
      })
    } else {
      const isOpen = openFiles.includes(entry.path)
      const isActive = entry.path === activeFile
      const isDirty = dirtyBuffers.has(entry.path)
      el.className = 'file-item' + (isActive ? ' active' : '') + (isOpen ? ' open' : '') + (isDirty ? ' dirty' : '')
      el.innerHTML = `<span class="fi-dot ${ext}"></span>${entry.name}${isDirty ? '<span class="fi-dirty" title="Unsaved changes">&#9679;</span>' : ''}`
      el.addEventListener('click', async () => {
        // If already open, just switch — otherwise read from disk and open.
        if (openFiles.includes(entry.path)) {
          switchToFile(entry.path)
        } else {
          try {
            const content = await invoke<string>('open_file', { path: entry.path })
            createNewFile(entry.name, content, entry.path)
            currentFilePath = entry.path
          } catch (err) {
            showToast(`Failed to open ${entry.name}: ${err}`, 'error')
          }
        }
      })
    }
    el.addEventListener('contextmenu', (e) => {
      e.preventDefault()
      contextMenuTarget = { path: entry.path, isDir: entry.isDir }
      showTreeContextMenu(e as MouseEvent)
    })
    filesList!.appendChild(el)

    if (entry.isDir && expandedFolders.has(entry.path) && folderChildren[entry.path]) {
      for (const child of folderChildren[entry.path]) renderEntry(child)
    }
  }

  for (const entry of workspaceTreeEntries) renderEntry(entry)
}

// + button handler
tabNewBtn.addEventListener('click', () => createNewFile())

// ── Tree actions (header buttons + context menu) ─────────────────────────────

function pickWorkspaceTargetDir(): string | null {
  // Where to create a new entry: the focused folder if a folder is selected,
  // the parent of the focused file, or the workspace root.
  if (contextMenuTarget) {
    if (contextMenuTarget.isDir) return contextMenuTarget.path
    return contextMenuTarget.path.replace(/[\\/][^\\/]+$/, '')
  }
  return openedFolderPath
}

async function promptAndCreateFile(parentDir: string | null) {
  if (!parentDir) {
    showToast('Open a folder first to create files in it', 'warning')
    return
  }
  const name = window.prompt('New file name:', 'untitled.urdf')
  if (!name) return
  const sep = parentDir.includes('\\') && !parentDir.includes('/') ? '\\' : '/'
  const newPath = `${parentDir}${parentDir.endsWith(sep) ? '' : sep}${name}`
  try {
    const content = isUrdfLike(name) ? SAMPLE_URDF : ''
    await invoke('create_file', { path: newPath, content, overwrite: false })
    if (parentDir !== openedFolderPath) expandedFolders.add(parentDir)
    // Invalidate cached children for the parent so refresh picks up the new file.
    if (parentDir in folderChildren) delete folderChildren[parentDir]
    if (expandedFolders.has(parentDir)) {
      const children = await invoke<Array<{ name: string; path: string; isDir: boolean; depth: number }>>(
        'list_directory', { path: parentDir }
      )
      const baseDepth = workspaceTreeEntries.find(e => e.path === parentDir)?.depth ?? 0
      folderChildren[parentDir] = children.map(c => ({ ...c, depth: baseDepth + 1 }))
    }
    await refreshTree()
    // Open the new file in a buffer
    const fileContent = await invoke<string>('open_file', { path: newPath })
    createNewFile(name, fileContent, newPath)
  } catch (err) {
    showToast(`Failed to create file: ${err}`, 'error')
  }
}

async function promptAndCreateFolder(parentDir: string | null) {
  if (!parentDir) {
    showToast('Open a folder first to create directories in it', 'warning')
    return
  }
  const name = window.prompt('New folder name:', 'new-folder')
  if (!name) return
  const sep = parentDir.includes('\\') && !parentDir.includes('/') ? '\\' : '/'
  const newPath = `${parentDir}${parentDir.endsWith(sep) ? '' : sep}${name}`
  try {
    await invoke('create_directory', { path: newPath })
    if (parentDir !== openedFolderPath) expandedFolders.add(parentDir)
    if (parentDir in folderChildren) delete folderChildren[parentDir]
    if (expandedFolders.has(parentDir)) {
      const children = await invoke<Array<{ name: string; path: string; isDir: boolean; depth: number }>>(
        'list_directory', { path: parentDir }
      )
      const baseDepth = workspaceTreeEntries.find(e => e.path === parentDir)?.depth ?? 0
      folderChildren[parentDir] = children.map(c => ({ ...c, depth: baseDepth + 1 }))
    }
    await refreshTree()
  } catch (err) {
    showToast(`Failed to create folder: ${err}`, 'error')
  }
}

async function deletePathFromTree(target: { path: string; isDir: boolean }) {
  const ok = window.confirm(`Delete ${target.isDir ? 'folder' : 'file'} "${target.path.split(/[\\/]/).pop()}"?${target.isDir ? '\n\nAll contents will be removed.' : ''}`)
  if (!ok) return
  try {
    await invoke('delete_path', { path: target.path, recursive: target.isDir })
    // Close any open buffer pointing at this path (or a descendant if it's a folder)
    const toClose = openFiles.filter(k =>
      k === target.path || (target.isDir && k.startsWith(target.path + '/')) || (target.isDir && k.startsWith(target.path + '\\'))
    )
    for (const k of toClose) closeFile(k)
    // Drop cached children for the deleted folder + its parent
    if (target.isDir) {
      delete folderChildren[target.path]
      expandedFolders.delete(target.path)
    }
    const parent = target.path.replace(/[\\/][^\\/]+$/, '')
    if (parent in folderChildren) delete folderChildren[parent]
    if (expandedFolders.has(parent)) {
      const children = await invoke<Array<{ name: string; path: string; isDir: boolean; depth: number }>>(
        'list_directory', { path: parent }
      )
      const baseDepth = workspaceTreeEntries.find(e => e.path === parent)?.depth ?? 0
      folderChildren[parent] = children.map(c => ({ ...c, depth: baseDepth + 1 }))
    }
    await refreshTree()
  } catch (err) {
    showToast(`Failed to delete: ${err}`, 'error')
  }
}

async function startInlineRename(target: { path: string; isDir: boolean }) {
  const row = filesList?.querySelector(`.file-item[data-path="${CSS.escape(target.path)}"]`) as HTMLElement | null
  if (!row) return
  const oldName = target.path.split(/[\\/]/).pop() || ''
  const parentDir = target.path.replace(/[\\/][^\\/]+$/, '')
  const labelSpan = row.lastChild as Node | null
  const input = document.createElement('input')
  input.type = 'text'
  input.className = 'fi-rename'
  input.value = oldName
  // Replace the trailing label text with the input
  if (labelSpan && labelSpan.nodeType === Node.TEXT_NODE) row.removeChild(labelSpan)
  row.appendChild(input)
  input.focus()
  input.setSelectionRange(0, oldName.lastIndexOf('.') === -1 ? oldName.length : oldName.lastIndexOf('.'))
  let committed = false
  const commit = async () => {
    if (committed) return
    committed = true
    const newName = input.value.trim()
    if (!newName || newName === oldName) { await refreshTree(); return }
    const sep = parentDir.includes('\\') && !parentDir.includes('/') ? '\\' : '/'
    const newPath = `${parentDir}${sep}${newName}`
    try {
      await invoke('rename_path', { oldPath: target.path, newPath })
      // Re-key any open buffer whose key is the old path or descends from it
      for (const k of [...openFiles]) {
        if (k === target.path) {
          rekeyBuffer(k, newPath)
        } else if (target.isDir && (k.startsWith(target.path + '/') || k.startsWith(target.path + '\\'))) {
          rekeyBuffer(k, newPath + k.slice(target.path.length))
        }
      }
      await refreshTree()
      renderTabs()
    } catch (err) {
      showToast(`Rename failed: ${err}`, 'error')
      await refreshTree()
    }
  }
  input.addEventListener('blur', commit)
  input.addEventListener('keydown', async (e) => {
    if (e.key === 'Enter') { e.preventDefault(); await commit() }
    else if (e.key === 'Escape') { committed = true; await refreshTree() }
  })
}

const treeContextMenu = document.getElementById('tree-context-menu') as HTMLDivElement | null

function showTreeContextMenu(e: MouseEvent) {
  if (!treeContextMenu) return
  treeContextMenu.classList.remove('hidden')
  const x = Math.min(e.clientX, window.innerWidth - 160)
  const y = Math.min(e.clientY, window.innerHeight - 140)
  treeContextMenu.style.left = `${x}px`
  treeContextMenu.style.top = `${y}px`
}

function hideTreeContextMenu() {
  if (!treeContextMenu) return
  treeContextMenu.classList.add('hidden')
}

document.addEventListener('click', hideTreeContextMenu)
document.addEventListener('contextmenu', (e) => {
  // Only hide if the click wasn't on a tree row (which has its own handler)
  const t = e.target as HTMLElement
  if (!t.closest('.file-item') && !t.closest('.tree-ctx-menu')) hideTreeContextMenu()
})

if (treeContextMenu) {
  treeContextMenu.addEventListener('click', async (e) => {
    const btn = (e.target as HTMLElement).closest('button') as HTMLButtonElement | null
    if (!btn) return
    const action = btn.dataset.action
    hideTreeContextMenu()
    const target = contextMenuTarget
    if (!target && action !== 'new-file' && action !== 'new-folder') return
    if (action === 'new-file') await promptAndCreateFile(pickWorkspaceTargetDir())
    else if (action === 'new-folder') await promptAndCreateFolder(pickWorkspaceTargetDir())
    else if (action === 'rename' && target) await startInlineRename(target)
    else if (action === 'delete' && target) await deletePathFromTree(target)
    contextMenuTarget = null
  })
}

document.getElementById('btn-tree-new-file')?.addEventListener('click', (e) => {
  e.stopPropagation()
  contextMenuTarget = null
  promptAndCreateFile(openedFolderPath)
})
document.getElementById('btn-tree-new-folder')?.addEventListener('click', (e) => {
  e.stopPropagation()
  contextMenuTarget = null
  promptAndCreateFolder(openedFolderPath)
})
document.getElementById('btn-tree-refresh')?.addEventListener('click', (e) => {
  e.stopPropagation()
  refreshTree()
})

// Initial render
renderTabs()
renderExplorer()


// ── Three.js ─────────────────────────────────────────────────────────────────

const canvas = document.getElementById('viewport') as HTMLCanvasElement
const viewportPanel = document.getElementById('viewport-panel') as HTMLDivElement

const renderer = new THREE.WebGLRenderer({ canvas, antialias: true, alpha: false })
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2))
renderer.setClearColor(VIEWPORT_BG[(localStorage.getItem('vector_theme') || 'dark') as ThemeId] || 0x1a1a1a)
renderer.shadowMap.enabled = true
renderer.shadowMap.type = THREE.PCFShadowMap
renderer.toneMapping = THREE.ACESFilmicToneMapping
renderer.toneMappingExposure = 1.1

const scene = new THREE.Scene()

// Procedural HDR environment map for realistic PBR metallic reflections
const pmremGenerator = new THREE.PMREMGenerator(renderer)
pmremGenerator.compileEquirectangularShader()
scene.environment = pmremGenerator.fromScene(new RoomEnvironment()).texture
pmremGenerator.dispose()

const camera = new THREE.PerspectiveCamera(50, 1, 0.01, 100)
camera.position.set(1.2, 1.0, 1.6)

const controls = new OrbitControls(camera, canvas)
controls.enableDamping = true
controls.dampingFactor = 0.08
controls.target.set(0, 0.35, 0)
controls.minDistance = 0.05
controls.maxDistance = 50
controls.rotateSpeed = 0.8
controls.panSpeed = 0.8
controls.zoomSpeed = 1.2
controls.enablePan = true
controls.screenSpacePanning = true  // pan moves in screen plane (more intuitive)
// Dev-only handle for scripted camera moves (generation review, demo capture).
if (import.meta.env.DEV) (window as any).__vectorView = { THREE, scene, camera, controls }
controls.mouseButtons = {
  LEFT: THREE.MOUSE.ROTATE,
  MIDDLE: THREE.MOUSE.PAN,
  RIGHT: THREE.MOUSE.PAN,  // right-click also pans (CAD-style)
}

const viewportNavTimer = new THREE.Timer()
const keysViewportPan = { w: false, a: false, s: false, d: false }
let shiftViewportPanHeld = false

document.addEventListener(
  'keydown',
  e => {
    if (e.key === 'Shift') shiftViewportPanHeld = true
  },
  true,
)
document.addEventListener(
  'keyup',
  e => {
    if (e.key === 'Shift') shiftViewportPanHeld = false
  },
  true,
)
window.addEventListener('blur', () => {
  keysViewportPan.w = keysViewportPan.a = keysViewportPan.s = keysViewportPan.d = false
  shiftViewportPanHeld = false
})

let cameraFocusTween: CameraFocusTween | null = null

// ── Post-processing pipeline (MSAA + SMAA + output) ─────────────────────────
// The composer's internal render target otherwise silently bypasses the
// WebGLRenderer antialias:true flag. Provide an MSAA render target explicitly.
const _initSize = renderer.getSize(new THREE.Vector2())
const _dpr = renderer.getPixelRatio()
const msaaRenderTarget = new THREE.WebGLRenderTarget(
  Math.max(1, _initSize.x * _dpr),
  Math.max(1, _initSize.y * _dpr),
  { type: THREE.HalfFloatType, samples: 4 },
)
msaaRenderTarget.texture.name = 'EffectComposer.rt1.msaa'
const composer = new EffectComposer(renderer, msaaRenderTarget)
composer.setPixelRatio(renderer.getPixelRatio())
// Sync initial size after a frame (viewport layout not done yet at this point)
requestAnimationFrame(() => {
  const w = renderer.domElement.clientWidth
  const h = renderer.domElement.clientHeight
  if (w > 0 && h > 0) composer.setSize(w, h)
})

const renderPass = new RenderPass(scene, camera)
composer.addPass(renderPass)

// SMAA: sub-pixel silhouette cleanup. Handles edges that slip past MSAA —
// especially thin rounded parts at far zoom. Sized automatically via composer.setSize.
const smaaPass = new SMAAPass()
composer.addPass(smaaPass)

// Output pass (tone mapping + color space conversion)
const outputPass = new OutputPass()
composer.addPass(outputPass)

// ── Scene setup ──────────────────────────────────────────────────────────────

// Grid
const grid = new THREE.GridHelper(8, 40, 0x3c3c3c, 0x2d2d2d)
scene.add(grid)

// Ground shadow receiver
const groundMesh = new THREE.Mesh(
  new THREE.PlaneGeometry(8, 8),
  new THREE.ShadowMaterial({ opacity: 0.25 })
)
groundMesh.rotation.x = -Math.PI / 2
groundMesh.receiveShadow = true
scene.add(groundMesh)

// Origin axes
const originAxes = new THREE.AxesHelper(0.5)
originAxes.position.y = 0.001
originAxes.renderOrder = 1
;(originAxes.material as THREE.Material).depthTest = false
scene.add(originAxes)

// Lights
scene.add(new THREE.AmbientLight(0xc8cce0, 0.4))

const keyLight = new THREE.DirectionalLight(0xffffff, 1.2)
keyLight.position.set(3, 6, 4)
keyLight.castShadow = true
keyLight.shadow.mapSize.set(2048, 2048)
keyLight.shadow.camera.near = 0.5
keyLight.shadow.camera.far = 20
keyLight.shadow.camera.left = -3
keyLight.shadow.camera.right = 3
keyLight.shadow.camera.top = 3
keyLight.shadow.camera.bottom = -3
keyLight.shadow.bias = -0.0005
scene.add(keyLight)

const fillLight = new THREE.DirectionalLight(0x6688cc, 0.4)
fillLight.position.set(-3, 2, -2)
scene.add(fillLight)

const rimLight = new THREE.DirectionalLight(0x8888ff, 0.25)
rimLight.position.set(0, 0.5, -4)
scene.add(rimLight)

// ── Materials ────────────────────────────────────────────────────────────────

const comMat = new THREE.MeshStandardMaterial({
  color: 0xe5c07b, roughness: 0.3, metalness: 0.2, emissive: 0x665500,
})
const wireMat = new THREE.MeshBasicMaterial({
  color: 0x4a9eff, wireframe: true, transparent: true, opacity: 0.12,
})

// ── Build robot from URDF ───────────────────────────────────────────────────

// Wire up path resolver for mesh loading in urdfParser
// Uses a lazy callback so it captures the current values at load time
setPathResolver(() => ({
  activeFileDiskPath: filePaths[activeFile] || currentFilePath,
  openedFolderPath,
}))

const robot = new THREE.Group()
scene.add(robot)

// Pre-warm GLB mesh cache so the first real URDF render uses real meshes
// instead of parametric fallback. Fire-and-forget — runs in parallel with
// the rest of app init; applyRichVisuals will use cached meshes if ready.
preloadMeshCache()

// Debounced callback: fired once after all async GLBs settle for a given parse cycle.
// Re-runs rebuildMountNodes so attachment rings are placed on the real rendered geometry
// rather than the parametric URDF primitive fallback that was measured at parse time.
//
// Each call to applyRichVisuals passes a closure that captures the parsedRobot at that
// moment. If the robot has been replaced by a later reparse (or a tab switch), the
// closure's stale reference won't match the live `parsedRobot` and the rebuild is skipped.
// The sim guard prevents node positions from being updated mid-simulation.
let _rebuildNodesTimer: ReturnType<typeof setTimeout> | null = null

function makeOnMeshLoaded(robotEpoch: typeof parsedRobot) {
  return (_linkName: string) => {
    if (_rebuildNodesTimer) clearTimeout(_rebuildNodesTimer)
    _rebuildNodesTimer = setTimeout(() => {
      _rebuildNodesTimer = null
      if (parsedRobot !== robotEpoch) return  // stale: robot was replaced
      if (simApi.isSimActive()) return          // don't disturb sim joint state
      // Re-fire alignment after async meshes settle. Avoid full groundRobot here:
      // grounding on every mesh load makes existing components jump when the
      // newly loaded mesh becomes the assembly's lowest point.
      // reconcilePostMeshLoadFloor is the targeted version: it only lowers when
      // the most recent liftAboveFloor over-lifted using a stale parametric
      // measurement (first-of-type carry, GLB not yet cached).
      try { reconcilePostMeshLoadFloor(robot) }
      catch (e) { console.warn('[reconcile] post-mesh-load floor pass failed:', e) }
      // STEP/GLB meshes load async — re-run edges so late arrivals get the
      // feature-edge overlay too. addEdgeLines is idempotent per-mesh.
      addEdgeLines(parsedRobot)
      applyCollisionOnlyView(showCollision)
    }, 150)
  }
}

// worldGroup applies the Z-up → Y-up correction for the scene.
// parsedRobot.group is kept in pure URDF (Z-up) space so exported
// transforms match what the parser reads back.
const worldGroup = new THREE.Group()
worldGroup.name = 'urdf_world'
worldGroup.rotation.x = -Math.PI / 2
robot.add(worldGroup)

// Forward-declared so callbacks below can close over it before the editor
// is initialised further down.
let urdfAssemblyApi: AssemblyEditorApi | null = null

// Catalog bbox for renderers; null for cut-to-length parts (their size is the
// instance's URDF geometry) and before the catalog has loaded.
const getPresetBboxMm = (compId: string) => presetBoundingBoxMm(compId)

setIdentitySource(SAMPLE_URDF)
void loadCatalog()
// The old placement engine kept its AssemblyGraphs here; nothing reads them now
// (the design travels inside the URDF).
try { localStorage.removeItem('vector_assembly_graphs') } catch { /* storage unavailable */ }
let parsedRobot = parseURDFToScene(SAMPLE_URDF)
worldGroup.add(parsedRobot.group)
robot.updateMatrixWorld(true)
applyRichVisuals(parsedRobot, makeOnMeshLoaded(parsedRobot), getPresetBboxMm)
addEdgeLines(parsedRobot)
groundRobot(robot)

// ── Edge lines (CAD-style silhouette edges) ─────────────────────────────────
// Gives the Fusion 360 feature-edge look: silhouettes stay crisp at any zoom
// because edges are 1-px vector lines, independent of triangle tessellation.

const edgeMaterial = new THREE.LineBasicMaterial({
  color: 0x0a0a0a,
  transparent: true,
  opacity: 0.55,
  depthTest: true,
})

function addEdgeLines(parsed: typeof parsedRobot) {
  parsed.group.traverse(child => {
    if (!(child instanceof THREE.Mesh) || !child.geometry) return
    // Idempotency: skip meshes that already have a feature-edge child.
    // Async STEP/GLB loads call this again; don't double up.
    if ((child.userData as Record<string, unknown>).__hasFeatureEdges) return
    const edges = new THREE.EdgesGeometry(child.geometry, 20) // 20° = Fusion-like
    const line = new THREE.LineSegments(edges, edgeMaterial)
    line.raycast = () => {} // don't interfere with raycasting
    ;(line.userData as Record<string, unknown>).__featureEdge = true
    child.add(line)
    ;(child.userData as Record<string, unknown>).__hasFeatureEdges = true
  })
}

// ── Wireframe overlay ────────────────────────────────────────────────────────

const wireframeGroup = new THREE.Group()
wireframeGroup.visible = false
robot.add(wireframeGroup)

// We'll rebuild wireframes after first render in animate

// ── Joint axis lines ─────────────────────────────────────────────────────────

const jointAxisState = { visible: false }
const axisVisuals: THREE.Object3D[] = []

function addJointAxis(parent: THREE.Object3D, dir: THREE.Vector3, color: number) {
  const pts = [
    new THREE.Vector3().copy(dir).multiplyScalar(-0.12),
    new THREE.Vector3().copy(dir).multiplyScalar(0.12),
  ]
  const geo = new THREE.BufferGeometry().setFromPoints(pts)
  const mat = new THREE.LineBasicMaterial({ color, transparent: true, opacity: 0.5 })
  const line = new THREE.Line(geo, mat)

  const ring = new THREE.Mesh(
    new THREE.TorusGeometry(0.05, 0.002, 8, 32),
    new THREE.MeshBasicMaterial({ color, transparent: true, opacity: 0.15 })
  )
  if (Math.abs(dir.z) > 0.5) { /* default orientation */ }
  else if (Math.abs(dir.y) > 0.5) ring.rotation.x = Math.PI / 2
  else ring.rotation.z = Math.PI / 2

  const group = new THREE.Group()
  group.add(line)
  group.add(ring)
  group.visible = jointAxisState.visible
  parent.add(group)
  axisVisuals.push(group)
}

// Add joint axis visuals for parsed joints
const axisColors = [0x4a9eff, 0x4ec9b0, 0xf48771, 0xce9178]
let colorIdx = 0
for (const [, jointInfo] of parsedRobot.joints) {
  const color = axisColors[colorIdx++ % axisColors.length]
  addJointAxis(jointInfo.group, jointInfo.axis, color)
}

// ── Build kinematic graph data (needed before CoM marker) ───────────────────

let { kinematicGraph, kinematicJoints } = buildKinematicGraphFromURDF(SAMPLE_URDF)

// ── CoM marker ───────────────────────────────────────────────────────────────

const comGroup = new THREE.Group()
comGroup.visible = false
robot.add(comGroup)

function updateComMarker() {
  // Clear existing markers
  comGroup.clear()

  // Calculate total mass and weighted CoM position from parsed links
  let totalMass = 0
  let comX = 0, comY = 0, comZ = 0

  for (const [linkName, linkGroup] of parsedRobot.linkGroups) {
    // Get mass from kinematicGraph
    const link = kinematicGraph[linkName]
    if (link) {
      const mass = link.mass
      totalMass += mass

      // Get world position of link
      const worldPos = new THREE.Vector3()
      linkGroup.getWorldPosition(worldPos)

      comX += mass * worldPos.x
      comY += mass * worldPos.y
      comZ += mass * worldPos.z
    }
  }

  if (totalMass > 0) {
    comX /= totalMass
    comY /= totalMass
    comZ /= totalMass
  }

const comMarker = new THREE.Mesh(new THREE.OctahedronGeometry(0.045), comMat)
  comMarker.position.set(comX, comY, comZ)
comGroup.add(comMarker)

  const comLinePts = [new THREE.Vector3(comX, comY, comZ), new THREE.Vector3(comX, 0, comZ)]
const comLineGeo = new THREE.BufferGeometry().setFromPoints(comLinePts)
const comLineMat = new THREE.LineDashedMaterial({ color: 0xe5c07b, dashSize: 0.02, gapSize: 0.01, transparent: true, opacity: 0.7 })
const comLine = new THREE.Line(comLineGeo, comLineMat)
comLine.computeLineDistances()
comGroup.add(comLine)

// CoM label ring
const comRingGround = new THREE.Mesh(
  new THREE.RingGeometry(0.035, 0.045, 24),
  new THREE.MeshBasicMaterial({ color: 0xe5c07b, transparent: true, opacity: 0.4, side: THREE.DoubleSide })
)
comRingGround.rotation.x = -Math.PI / 2
  comRingGround.position.set(comX, 0.001, comZ)
comGroup.add(comRingGround)
}

updateComMarker()

// ── Collision body visuals ────────────────────────────────────────────────────

const collisionMat = new THREE.MeshBasicMaterial({
  color: 0xff4444,
  transparent: true,
  opacity: 0.22,
  depthTest: true,
  side: THREE.DoubleSide,
})
const collisionEdgeMat = new THREE.LineBasicMaterial({ color: 0xff4444, transparent: true, opacity: 0.5 })
const collisionObjLoader = new OBJLoader()

let showCollision = false
let collisionVisualBuildId = 0
const HIDDEN_BY_COLLISION_VIEW = '__hiddenByCollisionView'

function isWheelCollisionLink(linkName: string): boolean {
  const comp = getComponent(componentIdForLink(linkName))
  return String((comp?.sim_metadata as Record<string, unknown> | undefined)?.contact_class ?? '') === 'wheel'
}

function collisionCylinderRadius(collisionEl: Element): number {
  const radius = parseFloat(collisionEl.querySelector('geometry > cylinder')?.getAttribute('radius') || '0')
  return Number.isFinite(radius) ? radius : 0
}

function hasCollisionFlag(obj: THREE.Object3D): boolean {
  let cur: THREE.Object3D | null = obj
  while (cur) {
    if ((cur.userData as Record<string, unknown> | undefined)?.isCollision) return true
    cur = cur.parent
  }
  return false
}

function applyCollisionOnlyView(enabled: boolean) {
  robot.traverse(obj => {
    if (!(obj instanceof THREE.Mesh || obj instanceof THREE.Line)) return
    if (hasCollisionFlag(obj)) return

    const userData = obj.userData as Record<string, unknown>
    if (enabled) {
      if (obj.visible) {
        userData[HIDDEN_BY_COLLISION_VIEW] = true
        obj.visible = false
      }
    } else if (userData[HIDDEN_BY_COLLISION_VIEW]) {
      obj.visible = true
      delete userData[HIDDEN_BY_COLLISION_VIEW]
    }
  })
}

function collisionMeshUrl(filename: string): string {
  const raw = filename.trim()
  if (/^https?:\/\//i.test(raw) || raw.startsWith('/')) return raw
  if (raw.startsWith('package://')) {
    const pkgPath = raw.slice('package://'.length)
    return pkgPath.startsWith('meshes/') ? `/${pkgPath}` : `/${pkgPath.replace(/^[^/]+\//, '')}`
  }
  return raw.startsWith('meshes/') ? `/${raw}` : `/meshes/collision/${raw}`
}

function applyCollisionOrigin(wrapper: THREE.Object3D, collisionEl: Element) {
  const originEl = collisionEl.querySelector('origin')
  if (!originEl) return
  const xyz = (originEl.getAttribute('xyz') || '0 0 0').split(/\s+/).map(parseFloat)
  const rpy = (originEl.getAttribute('rpy') || '0 0 0').split(/\s+/).map(parseFloat)
  wrapper.position.set(xyz[0] || 0, xyz[1] || 0, xyz[2] || 0)
  wrapper.quaternion.copy(rpyToQuat(rpy))
}

function addCollisionMeshEdges(root: THREE.Object3D) {
  root.traverse(obj => {
    if (!(obj instanceof THREE.Mesh) || !obj.geometry) return
    const edges = new THREE.LineSegments(new THREE.EdgesGeometry(obj.geometry, 20), collisionEdgeMat)
    edges.userData.isCollision = true
    edges.visible = showCollision
    edges.raycast = () => {}
    obj.add(edges)
  })
}

async function loadCollisionMeshIntoWrapper(
  filename: string,
  scaleAttr: string | null,
  wrapper: THREE.Group,
  buildId: number,
) {
  try {
    const res = await fetch(collisionMeshUrl(filename))
    if (!res.ok) throw new Error(`HTTP ${res.status}`)
    const obj = collisionObjLoader.parse(await res.text())
    if (buildId !== collisionVisualBuildId || !wrapper.parent) return

    obj.userData.isCollision = true
    obj.visible = showCollision
    if (scaleAttr) {
      const s = scaleAttr.split(/\s+/).map(parseFloat)
      if (s.length >= 3) obj.scale.set(s[0] || 1, s[1] || 1, s[2] || 1)
      else if (s.length === 1 && Number.isFinite(s[0])) obj.scale.setScalar(s[0])
    }
    obj.traverse(child => {
      child.userData.isCollision = true
      child.visible = showCollision
      if (child instanceof THREE.Mesh) {
        child.material = collisionMat
        child.raycast = () => {}
      }
    })
    addCollisionMeshEdges(obj)
    wrapper.add(obj)
    applyCollisionOnlyView(showCollision)
  } catch (err) {
    console.warn(`[collision] Failed to load collision mesh ${filename}:`, err)
  }
}

function rebuildCollisionVisuals(urdfText: string) {
  const buildId = ++collisionVisualBuildId
  // Strip any existing collision meshes from all link groups.
  // Collect first, THEN remove — removing during traverse() corrupts the scene graph
  // and causes the viewport to stop updating on tab switch.
  const toRemove: THREE.Object3D[] = []
  parsedRobot.group.traverse(obj => {
    if ((obj as any).userData?.isCollision) toRemove.push(obj)
  })
  for (const obj of toRemove) obj.parent?.remove(obj)
  if (!urdfText.trim()) return

  let doc: Document
  try {
    doc = new DOMParser().parseFromString(urdfText, 'application/xml')
    if (doc.documentElement.nodeName === 'parsererror') return
  } catch { return }

  for (const linkEl of Array.from(doc.querySelectorAll('link'))) {
    const linkName = linkEl.getAttribute('name')
    if (!linkName) continue
    const linkGroup = parsedRobot.linkGroups.get(linkName)
    if (!linkGroup) continue

    let collisionEls = Array.from(linkEl.querySelectorAll('collision'))
    const wheelCollision = isWheelCollisionLink(linkName)
    if (wheelCollision) {
      const tireCollision = collisionEls
        .filter(el => el.querySelector('geometry > cylinder'))
        .sort((a, b) => collisionCylinderRadius(b) - collisionCylinderRadius(a))[0]
      if (tireCollision) collisionEls = [tireCollision]
    }

    for (const collisionEl of collisionEls) {
      const geomEl = collisionEl.querySelector('geometry')
      if (!geomEl) continue

      let geo: THREE.BufferGeometry | null = null
      const boxEl = geomEl.querySelector('box')
      const cylEl = geomEl.querySelector('cylinder')
      const sphEl = geomEl.querySelector('sphere')
      const meshEl = geomEl.querySelector('mesh')
      let syntheticCyl = false

      if (meshEl && wheelCollision) {
        const compId = componentIdForLink(linkName)
        const bbox = compId ? getPresetBboxMm(compId) : null
        if (bbox) {
          const r = Math.max(bbox[0] || 0, bbox[1] || 0) / 2000
          const h = (bbox[2] || 0) / 1000
          if (r > 0 && h > 0) {
            geo = new THREE.CylinderGeometry(r, r, h, 32)
            syntheticCyl = true
          }
        }
      }

      if (meshEl && !syntheticCyl) {
        const filename = meshEl.getAttribute('filename') || ''
        if (!filename) continue
        const wrapper = new THREE.Group()
        wrapper.userData.isCollision = true
        wrapper.visible = showCollision
        applyCollisionOrigin(wrapper, collisionEl)
        linkGroup.add(wrapper)
        void loadCollisionMeshIntoWrapper(filename, meshEl.getAttribute('scale'), wrapper, buildId)
        continue
      }

      if (geo) {
        // Synthetic sim-only wheel cylinder from preset metadata.
      } else if (boxEl) {
        const s = (boxEl.getAttribute('size') || '0.1 0.1 0.1').split(/\s+/).map(parseFloat)
        geo = new THREE.BoxGeometry(s[0] || 0.1, s[1] || 0.1, s[2] || 0.1)
      } else if (cylEl) {
        const r = parseFloat(cylEl.getAttribute('radius') || '0.05')
        const h = parseFloat(cylEl.getAttribute('length') || '0.1')
        geo = new THREE.CylinderGeometry(r, r, h, 16)
      } else if (sphEl) {
        const r = parseFloat(sphEl.getAttribute('radius') || '0.05')
        geo = new THREE.SphereGeometry(r, 12, 8)
      }
      if (!geo) continue

      const mesh = new THREE.Mesh(geo, collisionMat)
      mesh.userData.isCollision = true
      mesh.visible = showCollision

      // Cylinder in Three.js is along Y; URDF cylinder is along Z — match visual parser
      if (cylEl || syntheticCyl) mesh.rotation.x = Math.PI / 2

      // Apply collision origin using the shared URDF RPY convention.
      const originEl = collisionEl.querySelector('origin')
      if (originEl) {
        const xyz = (originEl.getAttribute('xyz') || '0 0 0').split(/\s+/).map(parseFloat)
        const rpy = (originEl.getAttribute('rpy') || '0 0 0').split(/\s+/).map(parseFloat)
        mesh.position.set(xyz[0] || 0, xyz[1] || 0, xyz[2] || 0)
        // Compose origin rpy on top of cylinder rotation using a parent group
        const wrapper = new THREE.Group()
        wrapper.userData.isCollision = true
        wrapper.visible = showCollision
        // Match the MuJoCo conversion path for wheel links: the link/joint pose
        // already orients the tire, so applying the collision cylinder's own rpy
        // here makes the debug overlay show a misleading flat puck.
        wrapper.quaternion.copy(wheelCollision && (cylEl || syntheticCyl) ? new THREE.Quaternion() : rpyToQuat(rpy))
        wrapper.position.set(xyz[0] || 0, xyz[1] || 0, xyz[2] || 0)
        mesh.position.set(0, 0, 0)
        wrapper.add(mesh)
        // Edges for clarity
        const edges = new THREE.LineSegments(new THREE.EdgesGeometry(geo), collisionEdgeMat)
        edges.userData.isCollision = true
        edges.visible = showCollision
        if (cylEl || syntheticCyl) edges.rotation.x = Math.PI / 2
        wrapper.add(edges)
        linkGroup.add(wrapper)
        continue
      }

      // No origin — add mesh directly
      const edges = new THREE.LineSegments(new THREE.EdgesGeometry(geo), collisionEdgeMat)
      edges.userData.isCollision = true
      edges.visible = showCollision
      if (cylEl || syntheticCyl) edges.rotation.x = Math.PI / 2
      linkGroup.add(mesh)
      linkGroup.add(edges)
    }
  }
  applyCollisionOnlyView(showCollision)
}

const toggleCollisionBtn = document.getElementById('toggle-collision') as HTMLButtonElement | null
toggleCollisionBtn?.addEventListener('click', () => {
  showCollision = !showCollision
  parsedRobot.group.traverse(obj => {
    if ((obj as any).userData?.isCollision) (obj as THREE.Object3D).visible = showCollision
  })
  applyCollisionOnlyView(showCollision)
  toggleCollisionBtn.classList.toggle('active', showCollision)
})

// ── Sim mode (delegated to simManager.ts) ────────────────────────────────────

// Deferred callbacks to break circular init dependency (simManager ↔ vpControls/openSidebarPanel)
let _resize: () => void = () => {}
let _openSidebarPanel: (p: string) => void = () => {}
let _setSimSidebarLocked: (locked: boolean) => void = () => {}
let _createCheckpoint: (label: string, urdf: string, auto: boolean) => void = () => {}

const simStage = initSimStage({
  scene,
  camera,
  controls,
  robot,
  buildVisuals: [grid, groundMesh, originAxes],
})

simApi = initSimManager({
  robot,
  worldGroup,
  camera,
  controls,
  viewportPanel,
  simBar: document.getElementById('sim-bar') as HTMLElement,
  simToggle: document.getElementById('sim-toggle') as HTMLButtonElement,
  simPlay: document.getElementById('sim-play') as HTMLButtonElement,
  simPause: document.getElementById('sim-pause') as HTMLButtonElement,
  simReset: document.getElementById('sim-reset') as HTMLButtonElement,
  simTimeEl: document.getElementById('sim-time') as HTMLElement,
  viewportLabel: document.querySelector('.vp-tab[data-view="3d"]') as HTMLElement,
  getParsedRobot: () => parsedRobot,
  getEditorValue: () => monacoEditor.getModel()?.getValue() || '',
  getActiveFile: () => activeFile,
  getFilePaths: () => filePaths,
  getCurrentFilePath: () => currentFilePath,
  validateXMLStructure,
  validateURDFPerLink,
  showToast,
  openSidebarPanel: (p) => _openSidebarPanel(p),
  resize: () => _resize(),
  setTerrainVisual: (config) => simStage.setTerrain(config),
  onEnterSim: () => {
    viewportInteractionMode = 'inspect'
    syncViewportModeButton()
    urdfAssemblyApi?.onInteractionModeChanged('inspect')
    clearInspectFocus()
    simStage.enter()
    _setSimSidebarLocked(true)
  },
  onExitSim: () => {
    simStage.exit()
    _setSimSidebarLocked(false)
    viewportInteractionMode = 'build'
    syncViewportModeButton()
    urdfAssemblyApi?.onInteractionModeChanged('build')
    // After sim mutated robot.position/quaternion and per-joint group
    // transforms, the cached world positions of attachment-node meshes are
    // stale. Force matrix recomputation then resync node meshes so they
    // line up with the restored build-mode link poses.
    robot.updateMatrixWorld(true)
    urdfAssemblyApi?.refreshOverlay()
  },
})

// ── Viewport controls (delegated to viewportControls.ts) ────────────────────

const handle = document.getElementById('resize-handle') as HTMLDivElement
const editorPanel = document.getElementById('editor-panel') as HTMLDivElement

const vpControls = initViewportControls({
  camera, renderer, controls, robot, scene, canvas, viewportPanel, editorPanel, handle,
  originAxes, grid, comGroup, wireframeGroup, axisVisuals, jointAxisState,
  simBar: document.getElementById('sim-bar') as HTMLDivElement,
  simActive: () => simApi.isSimActive(),
  parsedRobot: () => parsedRobot,
  showToast,
  onResize: (w, h) => { composer.setSize(w, h); composer.setPixelRatio(renderer.getPixelRatio()) },
})

const { resize, focusOnRobot, setViewportCollapsed, setViewportFullscreen, setFocusMode, updateViewportInfo } = vpControls
// Wire up deferred resize callback now that it's available
_resize = resize

// ── Animate ──────────────────────────────────────────────────────────────────

let wireframeBuilt = false

function rebuildWireframes() {
  wireframeGroup.clear()
  wireframeBuilt = false

  // wireframeGroup is a child of robot, which has a non-identity position after
  // groundRobot (robot.position.y = -minY). We capture each mesh's world-space
  // transform and convert to wireframeGroup-local so clones sit where the solids do.
  // Must update matrices first — groundRobot mutates robot.position and we may run
  // before a render cycle refreshes matrixWorld.
  robot.updateMatrixWorld(true)
  robot.traverse(child => {
    if (child instanceof THREE.Mesh && child.material !== wireMat && child.material !== defaultMat && child.geometry) {
      const clone = new THREE.Mesh(child.geometry, wireMat)
      child.getWorldPosition(clone.position)
      wireframeGroup.worldToLocal(clone.position)
      child.getWorldQuaternion(clone.quaternion)
      child.getWorldScale(clone.scale)
      wireframeGroup.add(clone)
    }
  })
  wireframeBuilt = true
  applyCollisionOnlyView(showCollision)
}

function animate() {
  requestAnimationFrame(animate)

  viewportNavTimer.update()
  const navDt = Math.min(viewportNavTimer.getDelta(), 0.05)

  // Build wireframes once
  if (!wireframeBuilt) {
    rebuildWireframes()
  }

  if (
    viewportChatApi?.isViewport3D() &&
    (keysViewportPan.w || keysViewportPan.a || keysViewportPan.s || keysViewportPan.d)
  ) {
    const forward = new THREE.Vector3().subVectors(controls.target, camera.position)
    forward.y = 0
    if (forward.lengthSq() < 1e-10) forward.set(0, 0, -1)
    else forward.normalize()
    const right = new THREE.Vector3().crossVectors(forward, new THREE.Vector3(0, 1, 0)).normalize()
    const move = new THREE.Vector3()
    if (keysViewportPan.w) move.add(forward)
    if (keysViewportPan.s) move.sub(forward)
    if (keysViewportPan.d) move.add(right)
    if (keysViewportPan.a) move.sub(right)
    if (move.lengthSq() > 0) {
      const speed = (shiftViewportPanHeld ? 5.0 : 2.2) * navDt
      move.normalize().multiplyScalar(speed)
      camera.position.add(move)
      controls.target.add(move)
    }
  }

  if (cameraFocusTween) {
    cameraFocusTween = stepCameraFocusTween(cameraFocusTween, camera, controls, performance.now())
  }

  // Sim camera follow (delegated to simManager)
  simApi.tickCameraFollow()

  controls.update()

  // CoM marker spin
  for (const marker of comGroup.children) {
    if (marker instanceof THREE.Mesh) {
      marker.rotation.y += 0.01
    }
  }

  composer.render()
}
animate()


// ── Draggable split ──────────────────────────────────────────────────────────

const main = document.getElementById('main') as HTMLDivElement

let dragging = false

handle.addEventListener('mousedown', () => {
  dragging = true
  handle.classList.add('dragging')
  document.body.style.cursor = 'col-resize'
  document.body.style.userSelect = 'none'
})

document.addEventListener('mousemove', (e) => {
  if (!dragging) return
  const rect = main.getBoundingClientRect()
  const pct = ((e.clientX - rect.left) / rect.width) * 100
  editorPanel.style.width = `${Math.min(Math.max(pct, 15), 85)}%`
  resize()
})

document.addEventListener('mouseup', () => {
  if (!dragging) return
  dragging = false
  handle.classList.remove('dragging')
  document.body.style.cursor = ''
  document.body.style.userSelect = ''
})

// ── Sidebar resize ──────────────────────────────────────────────────────────
const sidebar = document.getElementById('sidebar') as HTMLDivElement
const sidebarHandle = document.getElementById('sidebar-resize-handle') as HTMLDivElement
let sidebarDragging = false

sidebarHandle.addEventListener('mousedown', (e) => {
  sidebarDragging = true
  sidebarHandle.classList.add('dragging')
  document.body.style.cursor = 'col-resize'
  document.body.style.userSelect = 'none'
  e.preventDefault()
})

document.addEventListener('mousemove', (e) => {
  if (!sidebarDragging) return
  // Subtract the activity bar width (48px) from the mouse X
  const activityBar = document.getElementById('activity-bar')!
  const abWidth = activityBar.getBoundingClientRect().width
  const newWidth = e.clientX - abWidth
  const clamped = Math.min(Math.max(newWidth, 140), 500)
  sidebar.style.width = clamped + 'px'
  resize()
})

document.addEventListener('mouseup', () => {
  if (!sidebarDragging) return
  sidebarDragging = false
  sidebarHandle.classList.remove('dragging')
  document.body.style.cursor = ''
  document.body.style.userSelect = ''
})

// ── Build Kinematic Context for AI ──────────────────────────────────────────────
// Generates a structured text summary of the robot's kinematic structure
// to send to Claude for better context-aware edits


// ── Live URDF re-parsing ────────────────────────────────────────────────────

let reparseTimeout: number | null = null
// urdfAssemblyApi is declared near the first applyRichVisuals call (TDZ) and
// initialised at the initAssemblyEditor site below.

function rebuildJointAxisVisuals() {
  axisVisuals.length = 0
  let colorIdx = 0
  for (const [, jointInfo] of parsedRobot.joints) {
    const color = axisColors[colorIdx++ % axisColors.length]
    addJointAxis(jointInfo.group, jointInfo.axis, color)
  }
}

// Monotonically-increasing counter. Incremented each time an async xacro reparse is
// dispatched. The callback checks this before applying results so a superseded async
// reparse (user typed again while xacro was processing) is silently discarded.
let xacroGeneration = 0

function reparseURDF(xmlOverride?: string, opts?: { skipGround?: boolean; ground?: boolean }) {
  try {
    let urdfContent: string
    if (xmlOverride !== undefined) {
      urdfContent = xmlOverride
    } else {
      const model = monacoEditor.getModel()
      if (!model) return  // no file open
      urdfContent = model.getValue()
    }

    // True xacro only: file extension or actual <xacro:…> tags.
    // Do NOT use urdfContent.includes('xacro:') — that matches xmlns:xacro on plain URDF
    // and wrongly runs the preprocessor (often breaking AI-generated robots).
    const isXacro = activeFile.endsWith('.xacro') || /<xacro:/i.test(urdfContent)
    if (isXacro) {
      // Capture generation + active file so the callback can detect if it's stale.
      const myGeneration = ++xacroGeneration
      const capturedFile = activeFile
      // Async xacro processing — fire and forget, reparse when done
      processXacro(urdfContent, {
        basePath: filePaths[activeFile]?.replace(/[\\/][^\\/]+$/, '') || openedFolderPath || '',
        fileLoader: async (filename: string) => {
          try {
            return await invoke<string>('open_file', { path: filename })
          } catch {
            console.warn(`[xacro] Could not load include: ${filename}`)
            return ''
          }
        },
      }).then(processed => {
        // Discard result if a newer reparse was issued or the user switched files.
        if (xacroGeneration !== myGeneration || activeFile !== capturedFile) return
        try {
          setIdentitySource(processed)
          const newParsed = parseURDFToScene(processed)
          const newKinematicData = buildKinematicGraphFromURDF(processed)
          worldGroup.remove(parsedRobot.group)
          wireframeGroup.clear()
          axisVisuals.length = 0
          parsedRobot = newParsed
          kinematicGraph = newKinematicData.kinematicGraph
          kinematicJoints = newKinematicData.kinematicJoints
          worldGroup.add(parsedRobot.group)
          robot.updateMatrixWorld(true) // ensure world matrices are fresh before rich visuals measure dims
          applyRichVisuals(parsedRobot, makeOnMeshLoaded(parsedRobot), getPresetBboxMm)
          const skipHeavy = false
          if (!skipHeavy) addEdgeLines(parsedRobot)
          rebuildJointAxisVisuals()
          updateComMarker()
          if (!skipHeavy) rebuildCollisionVisuals(processed)
          updateViewportInfo()
          urdfAssemblyApi?.onModelUpdated()
          if (opts?.ground === true && !opts?.skipGround) groundRobot(robot)
          else if (!opts?.skipGround) liftAboveFloor(robot)
          robot.updateMatrixWorld(true)
          urdfAssemblyApi?.refreshOverlay()
          // Wireframes must rebuild AFTER groundRobot so world-space capture
          // reflects the final robot position. Also skipped during bulk
          // assembly — the final reparse after the loop runs it once.
          if (!skipHeavy) rebuildWireframes()
        } catch (e) {
          console.warn('[xacro] Parse error after preprocessing:', e)
          showToast(
            `URDF parse failed after xacro: ${e instanceof Error ? e.message : String(e)}`,
            'error',
          )
        }
      }).catch(e => {
        console.warn('[xacro] Preprocessing failed:', e)
        showToast(`Xacro preprocessing failed: ${e instanceof Error ? e.message : String(e)}`, 'error')
      })
      return // async — will reparse when done
    }

    setIdentitySource(urdfContent)
    const newParsed = parseURDFToScene(urdfContent)
    const newKinematicData = buildKinematicGraphFromURDF(urdfContent)

    // Clear old robot geometry
    worldGroup.remove(parsedRobot.group)
    wireframeGroup.clear()
    axisVisuals.length = 0

    // Update parsed data
    parsedRobot = newParsed
    kinematicGraph = newKinematicData.kinematicGraph
    kinematicJoints = newKinematicData.kinematicJoints

    worldGroup.add(parsedRobot.group)
    robot.updateMatrixWorld(true)
    applyRichVisuals(parsedRobot, makeOnMeshLoaded(parsedRobot), getPresetBboxMm)
    const skipHeavy = false
    if (!skipHeavy) addEdgeLines(parsedRobot)

    rebuildJointAxisVisuals()

    // Update CoM marker
    updateComMarker()

    // Collision visuals can rebuild here — they don't depend on ground offset.
    if (!skipHeavy) rebuildCollisionVisuals(urdfContent)

    // Update viewport info
    updateViewportInfo()

    urdfAssemblyApi?.onModelUpdated()

    if (opts?.ground === true && !opts?.skipGround) groundRobot(robot)
    else if (!opts?.skipGround) liftAboveFloor(robot)
    robot.updateMatrixWorld(true)
    urdfAssemblyApi?.refreshOverlay()

    // Wireframes must rebuild AFTER groundRobot so world-space capture
    // reflects the final robot position. Skipped during bulk assembly —
    // the final reparse after the loop runs it once.
    if (!skipHeavy) rebuildWireframes()
  } catch (e) {
    console.error('[URDF] Parse error:', e)
    showToast(`URDF parse failed — 3D not updated: ${e instanceof Error ? e.message : String(e)}`, 'error')
    // Keep old geometry on parse error
  }
}

// Content change listeners are attached in createNewFile() for all URDF/XML files.
// The duplicate robot.urdf listener that used to live here has been removed — the
// createNewFile() path already covers it (and also calls groundRobot).

// ── Validation Panel (delegated to validation.ts) ──────────────────────────

const { runLocalValidation } = initValidation({
  invoke: invoke as (cmd: string, args?: Record<string, unknown>) => Promise<unknown>,
  monacoEditor,
  getKinematicGraph: () => kinematicGraph,
  getKinematicJoints: () => kinematicJoints,
  showToast,
})

// ── Node Graph Visualization ────────────────────────────────────────────────

const nodeGraph = initNodeGraph({
  viewportPanel,
  kinematicGraph: () => kinematicGraph,
  kinematicJoints: () => kinematicJoints,
  parsedRobot: () => parsedRobot,
  onSelectLink: (link) => urdfAssemblyApi?.setSelectedLink(link),
})


const toggleGraphBtn = document.getElementById('toggle-graph') as HTMLButtonElement

toggleGraphBtn.addEventListener('click', () => {
  nodeGraph.toggle()
})

// ── Display popover ──────────────────────────────────────────────────────────
{
  const popoverBtn = document.getElementById('display-popover-btn')!
  const popover = document.getElementById('display-popover')!
  const items = popover.querySelectorAll('.dp-item') as NodeListOf<HTMLElement>

  popoverBtn.addEventListener('click', (e) => {
    e.stopPropagation()
    popover.classList.toggle('hidden')
  })

  // Popover items forward the click to a hidden toggle button; that
  // programmatic click bubbles up to document and would otherwise close
  // the popover. Suppress the next outside-click while we're forwarding.
  let suppressOutsideClick = false

  // Close on outside click
  document.addEventListener('click', (e) => {
    if (suppressOutsideClick) { suppressOutsideClick = false; return }
    if (!popover.contains(e.target as Node) && e.target !== popoverBtn) {
      popover.classList.add('hidden')
    }
  })

  // Sync checkmarks with the hidden toggle buttons
  function syncChecks() {
    items.forEach(item => {
      const targetId = item.dataset.target!
      const btn = document.getElementById(targetId)
      const check = item.querySelector('.dp-check')!
      check.classList.toggle('active', btn?.classList.contains('active') ?? false)
    })
    // Tint the popover trigger if any toggle is non-default
    const anyActive = Array.from(items).some(item => {
      const btn = document.getElementById(item.dataset.target!)
      return btn?.classList.contains('active') ?? false
    })
    popoverBtn.classList.toggle('has-active', anyActive)
  }

  items.forEach(item => {
    item.addEventListener('click', (e) => {
      e.stopPropagation()
      const targetId = item.dataset.target!
      const btn = document.getElementById(targetId)
      suppressOutsideClick = true
      btn?.click()
      // Sync after a microtask so the click handler has toggled .active
      requestAnimationFrame(syncChecks)
    })
  })

  // Initial sync
  requestAnimationFrame(syncChecks)

  // Also sync when keyboard shortcuts toggle visibility
  const observer = new MutationObserver(syncChecks)
  items.forEach(item => {
    const btn = document.getElementById(item.dataset.target!)
    if (btn) observer.observe(btn, { attributes: true, attributeFilter: ['class'] })
  })
}

// ── Keyboard shortcuts for viewport toggles ─────────────────────────────────
document.addEventListener('keydown', (e) => {
  // File I/O shortcuts (work even in editor)
  const isMacCtrl = (e.metaKey || e.ctrlKey)
  if (isMacCtrl && e.key === 's') {
    e.preventDefault()
    if (e.shiftKey) {
      saveFileAs()
    } else {
      saveCurrentFile()
    }
    return
  }
  if (isMacCtrl && e.key === 'o') {
    e.preventDefault()
    openFileDialog()
    return
  }
  if (isMacCtrl && e.shiftKey && e.key === 'G') {
    e.preventDefault()
    const gitBtn = document.querySelector('.ab-btn[data-panel="git"]') as HTMLElement | null
    if (gitBtn) gitBtn.click()
    return
  }
  if (isMacCtrl && e.key === 'b') {
    e.preventDefault()
    const sb = document.getElementById('sidebar') as HTMLDivElement | null
    if (sb) {
      sb.classList.toggle('hidden')
    }
    return
  }

  // Don't trigger shortcuts when typing in inputs or Monaco editor
  const tag = (e.target as HTMLElement).tagName
  if (tag === 'INPUT' || tag === 'TEXTAREA' || tag === 'SELECT') return
  if ((e.target as HTMLElement).closest('.monaco-editor')) return

  // WASD: pan camera on the ground plane (orbit target moves with camera). Shift = faster.
  if (viewportChatApi?.isViewport3D() ?? true) {
    const pk = e.key.toLowerCase()
    if (
      (pk === 'w' || pk === 'a' || pk === 's' || pk === 'd') &&
      !e.ctrlKey &&
      !e.metaKey &&
      !e.altKey &&
      !e.shiftKey
    ) {
      keysViewportPan[pk] = true
      e.preventDefault()
          return
        }
      }

  switch (e.key.toLowerCase()) {
    case 'a':
      if (e.shiftKey) document.getElementById('toggle-axes')?.click()
      break
    case 'c':
      if (!e.shiftKey) document.getElementById('toggle-com')?.click()
      break
    case 'w':
      if (e.shiftKey) document.getElementById('toggle-wireframe')?.click()
      break
    case 'x':
      document.getElementById('toggle-collision')?.click()
      break
    case 'g':
      if (!e.ctrlKey && !e.metaKey) {
        document.getElementById('toggle-grid')?.click()
      }
      break
    case 'n':
      toggleGraphBtn.click()
      break
    case 'k':
      document.getElementById('toggle-joint-axis')?.click()
      break
    case 'p':
      setViewportCollapsed(!vpControls.viewportCollapsed())
      break
    case 'f':
      if (e.shiftKey) {
        setFocusMode(!vpControls.focusMode())
      } else {
        setViewportFullscreen(!vpControls.viewportFullscreen())
      }
      break
    case 'escape':
      if (vpControls.focusMode()) {
        setFocusMode(false)
        break
      }
      if (vpControls.viewportFullscreen()) {
        setViewportFullscreen(false)
        break
      }
      if (nodeGraph.isVisible()) {
        nodeGraph.hide()
      }
      break
  }
})

document.addEventListener('keyup', e => {
  const tag = (e.target as HTMLElement).tagName
  if (tag === 'INPUT' || tag === 'TEXTAREA' || tag === 'SELECT') return
  if ((e.target as HTMLElement).closest('.monaco-editor')) return
  const k = e.key.toLowerCase()
  if (k === 'w' || k === 'a' || k === 's' || k === 'd') {
    keysViewportPan[k] = false
  }
})

// ── Git Source Control Panel ────────────────────────────────────────────────
const { refreshGitStatus } = initGitPanel({ invoke, showToast })

// ── Activity bar ─────────────────────────────────────────────────────────────

const panels: Record<string, HTMLElement> = {
  explorer: document.getElementById('panel-explorer')!,
  build: document.getElementById('panel-build')!,
  inspector: document.getElementById('panel-inspector')!,
  toolbox: document.getElementById('panel-toolbox')!,
  validation: document.getElementById('panel-validation')!,
  sim: document.getElementById('panel-sim')!,
  git: document.getElementById('panel-git')!,
  settings: document.getElementById('panel-settings')!,
}

const activityBar = document.getElementById('activity-bar')!
const simActivityBtn = document.querySelector('.ab-btn[data-panel="sim"]') as HTMLElement | null
let preSimSidebarPanel = 'explorer'

function setSimSidebarLocked(locked: boolean) {
  if (locked) {
    const activeBtn = document.querySelector('.ab-btn.active') as HTMLElement | null
    const activePanel = activeBtn?.dataset.panel
    if (activePanel && activePanel !== 'sim') preSimSidebarPanel = activePanel
  }
  activityBar.classList.toggle('sim-sidebar-locked', locked)
  simActivityBtn?.classList.toggle('hidden', !locked)
  if (locked) {
    openSidebarPanel('sim')
  } else {
    simActivityBtn?.classList.remove('active')
    openSidebarPanel(panels[preSimSidebarPanel] ? preSimSidebarPanel : 'explorer')
  }
}

function openSidebarPanel(panel: string) {
  if (simApi?.isSimActive() && panel !== 'sim') {
    showToast('Exit simulation mode before switching panels', 'info')
    panel = 'sim'
  }
  if (
    (panel === 'build' || panel === 'toolbox' || panel === 'inspector') &&
    !(viewportChatApi?.isViewport3D() ?? true)
  ) {
    viewportChatApi?.switchViewportView('3d')
    showToast('Switched to 3D Preview for URDF editing', 'info')
  }
  if (
    (panel === 'build' || panel === 'toolbox' || panel === 'inspector') &&
    nodeGraph.isVisible()
  ) {
    nodeGraph.hide()
  }
  document.querySelectorAll('.ab-btn').forEach(b => b.classList.remove('active'))
  Object.values(panels).forEach(p => p.classList.add('hidden'))
  const btn = document.querySelector(`.ab-btn[data-panel="${panel}"]`) as HTMLElement | null
  if (panels[panel] && btn) {
    btn.classList.add('active')
    panels[panel].classList.remove('hidden')
    if (panel === 'git') {
      void refreshGitStatus()
    }
  }
}
// Wire up deferred openSidebarPanel callback for simManager
_openSidebarPanel = openSidebarPanel
_setSimSidebarLocked = setSimSidebarLocked

document.querySelectorAll('.ab-btn').forEach(btn => {
  btn.addEventListener('click', () => {
    const panel = (btn as HTMLElement).dataset.panel!
    if (!panel) return
    if (simApi.isSimActive()) {
      if (panel !== 'sim') {
        showToast('Exit simulation mode before switching panels', 'info')
      }
      openSidebarPanel('sim')
      return
    }
    const wasActive = btn.classList.contains('active')
    if (wasActive) {
    document.querySelectorAll('.ab-btn').forEach(b => b.classList.remove('active'))
    Object.values(panels).forEach(p => p.classList.add('hidden'))
    } else {
      openSidebarPanel(panel)
    }
  })
})

// ── Activity bar drag-and-drop reorder ────────────────────────────────────────
{
  const STORAGE_KEY = 'vector_ab_order'
  const spacer = activityBar.querySelector('.ab-spacer')!

  function getDraggableBtns(): HTMLElement[] {
    return Array.from(activityBar.querySelectorAll('.ab-btn.ab-draggable')) as HTMLElement[]
  }

  // Mark draggable buttons (all except settings)
  activityBar.querySelectorAll('.ab-btn[draggable="true"]').forEach(btn => {
    (btn as HTMLElement).removeAttribute('draggable')
    btn.classList.add('ab-draggable')
  })

  // Restore saved order on load
  const savedOrder: string[] | null = JSON.parse(localStorage.getItem(STORAGE_KEY) || 'null')
  if (savedOrder) {
    const btnMap = new Map<string, HTMLElement>()
    getDraggableBtns().forEach(btn => btnMap.set(btn.dataset.panel!, btn))
    for (const panel of savedOrder) {
      const btn = btnMap.get(panel)
      if (btn) activityBar.insertBefore(btn, spacer)
    }
  }

  function saveOrder() {
    const order = getDraggableBtns().map(btn => btn.dataset.panel!)
    localStorage.setItem(STORAGE_KEY, JSON.stringify(order))
  }

  let draggedBtn: HTMLElement | null = null
  let isDragging = false
  let startY = 0
  const DRAG_THRESHOLD = 5

  // Create a reusable drop indicator line
  const indicator = document.createElement('div')
  indicator.className = 'ab-drag-indicator'
  indicator.style.display = 'none'
  activityBar.style.position = 'relative'
  activityBar.appendChild(indicator)

  function findHoverTarget(clientY: number): HTMLElement | null {
    for (const btn of getDraggableBtns()) {
      if (btn === draggedBtn) continue
      const rect = btn.getBoundingClientRect()
      if (clientY >= rect.top && clientY <= rect.bottom) return btn
    }
    return null
  }

  function onPointerDown(e: PointerEvent) {
    // Find the .ab-draggable ancestor from whatever was clicked (svg, path, etc.)
    const btn = (e.target as HTMLElement).closest?.('.ab-draggable') as HTMLElement | null
    if (!btn) return
    draggedBtn = btn
    startY = e.clientY
    isDragging = false
    document.addEventListener('pointermove', onPointerMove)
    document.addEventListener('pointerup', onPointerUp)
  }

  function onPointerMove(e: PointerEvent) {
    if (!draggedBtn) return

    if (!isDragging) {
      if (Math.abs(e.clientY - startY) < DRAG_THRESHOLD) return
      isDragging = true
      draggedBtn.classList.add('ab-dragging')
      draggedBtn.setPointerCapture(e.pointerId)
    }

    const target = findHoverTarget(e.clientY)
    if (!target) {
      indicator.style.display = 'none'
      return
    }

    const rect = target.getBoundingClientRect()
    const barRect = activityBar.getBoundingClientRect()
    const midY = rect.top + rect.height / 2
    const above = e.clientY < midY
    const y = (above ? rect.top : rect.bottom) - barRect.top

    indicator.style.display = 'block'
    indicator.style.top = `${y - 1}px`
  }

  function onPointerUp(e: PointerEvent) {
    document.removeEventListener('pointermove', onPointerMove)
    document.removeEventListener('pointerup', onPointerUp)
    indicator.style.display = 'none'

    if (!draggedBtn) return

    if (isDragging) {
      draggedBtn.releasePointerCapture(e.pointerId)
      draggedBtn.classList.remove('ab-dragging')

      const target = findHoverTarget(e.clientY)
      if (target && target !== draggedBtn) {
        const rect = target.getBoundingClientRect()
        const midY = rect.top + rect.height / 2
        if (e.clientY < midY) {
          activityBar.insertBefore(draggedBtn, target)
        } else {
          activityBar.insertBefore(draggedBtn, target.nextElementSibling)
        }
        saveOrder()
      }
    }

    draggedBtn = null
    isDragging = false
  }

  activityBar.addEventListener('pointerdown', onPointerDown)
}

// Collapsible sidebar sections
document.querySelectorAll('.sb-header').forEach(header => {
  header.addEventListener('click', () => {
    const targetId = (header as HTMLElement).dataset.target
    if (!targetId) return
    const list = document.getElementById(targetId)
    if (!list) return
    const arrow = header.querySelector('.arrow')
    const isHidden = list.style.display === 'none'
    list.style.display = isHidden ? '' : 'none'
    if (arrow) arrow.textContent = isHidden ? '\u25BE' : '\u25B8'
  })
})

// ── Settings / Themes / Checkpoints (initialised after all DOM wiring) ──────
const { createCheckpoint } = initSettings({
  monacoEditor,
  showToast,
  getEditorApi: () => urdfAssemblyApi,
  renderer,
})
// Wire up deferred checkpoint callback for inlineDiff
_createCheckpoint = createCheckpoint

// ── File I/O Buttons ─────────────────────────────────────────────────────────
const btnOpenFile = document.getElementById('btn-open-file') as HTMLButtonElement | null
const btnSaveFile = document.getElementById('btn-save-file') as HTMLButtonElement | null
const btnOpenFolder = document.getElementById('btn-open-folder') as HTMLButtonElement | null

if (btnOpenFile) {
  btnOpenFile.addEventListener('click', openFileDialog)
}
if (btnSaveFile) {
  btnSaveFile.addEventListener('click', saveCurrentFile)
}

// ── Open Folder ─────────────────────────────────────────────────────────────

async function openFolderDialog() {
  try {
    const folderPath = await invoke<string | null>('open_folder_dialog')
    if (!folderPath) return
    await openWorkspaceFolder(folderPath)
  } catch (err) {
    showToast(`Error opening folder: ${err}`, 'error')
  }
}

async function openWorkspaceFolder(folderPath: string) {
  openedFolderPath = folderPath
  expandedFolders.clear()
  for (const k of Object.keys(folderChildren)) delete folderChildren[k]

  const bcProject = document.getElementById('bc-project')
  if (bcProject) bcProject.textContent = folderPath.split(/[\\/]/).pop() || 'Vector'

  const treeLabel = document.getElementById('explorer-tree-label')
  if (treeLabel) treeLabel.textContent = folderPath.split(/[\\/]/).pop() || folderPath

  try { localStorage.setItem('vector_workspace', folderPath) } catch { /* ignore quota errors */ }

  await refreshTree()
  showToast(`Opened folder: ${folderPath.split(/[\\/]/).pop()}`, 'success')
}

// Auto-restore last workspace + open tabs on launch (best-effort — silently
// skip entries whose paths no longer exist).
;(async () => {
  try {
    const last = localStorage.getItem('vector_workspace')
    if (last) {
      const probe = await invoke<{ exists: boolean; isDir: boolean }>('path_exists', { path: last })
      if (probe.exists && probe.isDir) {
        await openWorkspaceFolder(last)
      } else {
        localStorage.removeItem('vector_workspace')
      }
    }

    const raw = localStorage.getItem('vector_open_tabs')
    if (raw) {
      const state = JSON.parse(raw) as { paths: string[]; active: string | null }
      const restored: string[] = []
      for (const p of state.paths || []) {
        try {
          const probe = await invoke<{ exists: boolean; isDir: boolean }>('path_exists', { path: p })
          if (!probe.exists || probe.isDir) continue
          const content = await invoke<string>('open_file', { path: p })
          const name = p.split(/[\\/]/).pop() || 'file'
          createNewFile(name, content, p)
          restored.push(p)
        } catch { /* skip unreadable */ }
      }
      // Drop the default untitled sample buffer if we restored real tabs.
      if (restored.length > 0) {
        for (const k of [...openFiles]) {
          if (k.startsWith('untitled:') && monacoModels[k]?.getValue() === SAMPLE_URDF) {
            closeFile(k)
            break
          }
        }
      }
      if (state.active && restored.includes(state.active)) {
        switchToFile(state.active)
      }
    }
  } catch { /* no-op: best effort */ }
})()

if (btnOpenFolder) {
  btnOpenFolder.addEventListener('click', openFolderDialog)
}

// ── Drag and drop ───────────────────────────────────────────────────────────

// Tauri 2.x file drop events
try {
  const { getCurrentWindow } = await import('@tauri-apps/api/window')
  const appWindow = getCurrentWindow()
  appWindow.onDragDropEvent(async (event) => {
    if (event.payload.type === 'drop') {
      const paths = event.payload.paths
      if (!paths || paths.length === 0) return

      for (const path of paths) {
        const name = path.split(/[\\/]/).pop() || 'file'
        const ext = name.split('.').pop()?.toLowerCase() || ''

        // Check if it's a directory by trying to list it
        try {
          const probe = await invoke<{ exists: boolean; isDir: boolean }>('path_exists', { path })
          if (probe.exists && probe.isDir) {
            await openWorkspaceFolder(path)
            openSidebarPanel('explorer')
            continue
          }
        } catch {
          // path_exists shouldn't fail; fall through to file path branch
        }

        if (['urdf', 'xml', 'sdf', 'mjcf', 'json', 'yaml', 'yml', 'txt', 'obj', 'py'].includes(ext)) {
          try {
            const content = await invoke<string>('open_file', { path })
            createNewFile(name, content, path)
            currentFilePath = path
            showToast(`Opened ${name}`, 'success')
          } catch (err) {
            showToast(`Failed to open ${name}: ${err}`, 'error')
          }
        }
      }
    }
  })
} catch {
  // Tauri drag-drop not available (dev mode without Tauri)
}

// ── Toast notifications ──────────────────────────────────────────────────────

const toastArea = document.getElementById('toast-area') as HTMLDivElement

function showToast(message: string, type: 'success' | 'warning' | 'error' | 'info' = 'info') {
  const toast = document.createElement('div')
  toast.className = `toast ${type}`
  toast.textContent = message
  toastArea.appendChild(toast)
  setTimeout(() => {
    toast.classList.add('fade-out')
    setTimeout(() => toast.remove(), 300)
  }, 3000)
}

// ── Init toast ───────────────────────────────────────────────────────────────
setTimeout(() => {
  if (!activeFile) return
  showToast(`Loaded ${activeFile.split(/[\\/]/).pop() || activeFile} — ${parsedRobot.linkCount} links, ${parsedRobot.jointCount} joints`, 'success')
}, 500)

// ── Auto-start Python core ──────────────────────────────────────────────────
// @ts-ignore — used for future feature gating
let coreAvailable = false
;(async () => {
  console.log('[Core] Waiting for Tauri IPC...')
  try {
    await waitForTauri()
    console.log('[Core] Starting Python core process...')
    await invokeWithTimeout('start_core', {}, 15000)
    coreAvailable = true
    console.log('[Core] ✓ Python core started successfully — AI features available')
  } catch (e) {
    const msg = String(e).toLowerCase()
    if (msg.includes('already running') || msg.includes('already started')) {
      coreAvailable = true
      console.log('[Core] ✓ Python core already running')
    } else {
      console.error('[Core] ✗ Could not start Python core:', e)
      console.error('[Core] AI completions and edits will not work.')
      console.error('[Core] Check: Is Python installed? Run: python --version or python3 --version')
    }
  }
})()

// ── Inline Diff + Viewport Chat (delegated to inlineDiff.ts / viewportChat.ts) ─

inlineDiffApi = initInlineDiff({
  getUrdfAssemblyApi: () => urdfAssemblyApi,
  createCheckpoint: (l, u, a) => _createCheckpoint(l, u, a),
  getReparseTimeout: () => reparseTimeout,
  setReparseTimeout: (id) => { reparseTimeout = id },
  reparseURDF,
  runLocalValidation,
  showToast,
})

// ── Viewport Chat (delegated to viewportChat.ts) ─────────────────────────────
viewportChatApi = initViewportChat({
  getEditorValue: () => monacoEditor.getModel()?.getValue() || '',
  createNewFile: (name, content, path) => createNewFile(name ?? 'robot.urdf', content ?? '', path ?? null),
  getCurrentChatId: () => chatApi.getCurrentChatId(),
  getCurrentChatMessages: () => chatApi.getCurrentChatMessages(),
  recordChatMessage: (role, content) => chatApi.recordChatMessage(role, content),
  attachRewindButton: (el, idx) => chatApi.attachRewindButton(el, idx),
  loadChat: (id) => chatApi.loadChat(id),
  startNewChat: () => chatApi.startNewChat(),
  updateChatDropdown: () => chatApi.updateChatDropdown(),
  showInlineDiff: (o, n, u) => inlineDiffApi.showInlineDiff(o, n, u),
  clearInlineDiff: () => inlineDiffApi.clearInlineDiff(),
  setActiveChatActionsId: (id) => inlineDiffApi.setActiveChatActionsId(id),
  acceptInlineDiff: () => inlineDiffApi.acceptInlineDiff(),
  dismissInlineDiff: () => inlineDiffApi.dismissInlineDiff(),
  autoFrameRobot: () => autoFrameRobot(robot, camera, controls),
  showToast,
  SAMPLE_URDF,
  keysViewportPan,
  resize: () => _resize(),
})

// ── File I/O System ──────────────────────────────────────────────────────────
let currentFilePath: string | null = null

async function openFileDialog() {
  try {
    const path = await invoke<string | null>('open_file_dialog')
    if (!path) return

    const content = await invoke<string>('open_file', { path })
    const filename = path.split(/[\\/]/).pop() || 'untitled'

    // Create a new tab for the opened file (or switch to it if already open)
    createNewFile(filename, content, path)
    currentFilePath = path

    showToast(`Opened ${filename}`, 'success')
  } catch (err) {
    showToast(`Error opening file: ${err}`, 'error')
  }
}

async function saveCurrentFile() {
  try {
    if (!monacoEditor.getModel()) return
    const content = monacoEditor.getValue()
    const wasUntitled = activeFile.startsWith('untitled:')
    // For untitled buffers, NEVER fall back to `currentFilePath` (the legacy
    // global tracks the most-recently-opened disk path, NOT the active
    // buffer's path — falling back to it would silently overwrite the
    // previous file when the user hits Save on a new tab).
    let path = wasUntitled ? null : (filePaths[activeFile] || currentFilePath)

    if (!path) {
      const defaultName = wasUntitled
        ? `untitled_${untitledDisplayN(activeFile)}.urdf`
        : activeFile.split(/[\\/]/).pop() || activeFile
      path = await invoke<string | null>('save_file_dialog', { default_name: defaultName })
      if (!path) return
    }

    await invoke('save_file', { path, content })

    // Promote an untitled buffer to a path-keyed buffer on first save: re-key
    // every per-buffer map from the synthetic untitled key to the new disk path
    // so tabs, tree highlights, and chat history all migrate together.
    if (wasUntitled && path !== activeFile) {
      rekeyBuffer(activeFile, path)
    }

    // Buffer is now in sync with disk.
    dirtyBuffers.delete(activeFile)
    filePaths[activeFile] = path
    currentFilePath = path
    renderTabs()
    renderExplorer()
    saveOpenTabsState()
    const filename = path.split(/[\\/]/).pop() || activeFile
    showToast(`Saved ${filename}`, 'success')
    document.title = `Vector — ${filename}`
    // Refresh tree if the saved path lives in the open workspace
    if (openedFolderPath && path.startsWith(openedFolderPath)) refreshTree()
  } catch (err) {
    showToast(`Error saving file: ${err}`, 'error')
  }
}

async function saveFileAs() {
  try {
    if (!monacoEditor.getModel()) return
    const content = monacoEditor.getValue()
    const path = await invoke<string | null>('save_file_dialog', { default_name: currentFilePath?.split(/[\\/]/).pop() || 'robot.urdf' })
    if (!path) return

    await invoke('save_file', { path, content })
    currentFilePath = path
    const filename = path.split(/[\\/]/).pop() || 'robot.urdf'
    showToast(`Saved ${filename}`, 'success')
    document.title = `Vector — ${filename}`
  } catch (err) {
    showToast(`Error saving file: ${err}`, 'error')
  }
}

function cancelCameraFocusTween() {
  cameraFocusTween = null
}

canvas.addEventListener('pointerdown', () => {
  cameraFocusTween = null
})

function syncViewportModeButton() {
  const btn = document.getElementById('toggle-vp-mode')
  const label = document.getElementById('vp-mode-label')
  if (!btn || !label) return
  btn.classList.toggle('active', viewportInteractionMode === 'build')
  label.textContent = viewportInteractionMode === 'build' ? 'Build' : 'Inspect'
}

function startFocusCameraOnLink(linkName: string) {
  const box = computeLinkWorldBox(parsedRobot.group, linkName)
  if (!box || box.isEmpty()) {
    showToast('No mesh bounds for this link', 'warning')
    return
  }
  const pose = cameraPoseForBox(box, camera, controls)
  cameraFocusTween = {
    startMs: performance.now(),
    durationMs: 420,
    fromPos: camera.position.clone(),
    toPos: pose.position,
    fromTarget: controls.target.clone(),
    toTarget: pose.target,
  }
}

function clearInspectFocus() {
  inspectFocusedLink = null
  cancelCameraFocusTween()
  restoreInspectMaterials(parsedRobot.group)
  urdfAssemblyApi?.setSelectedLink(null)
}

function handleInspectLinkFocused(linkName: string | null) {
  if (viewportInteractionMode !== 'inspect') return
  if (!linkName) {
    clearInspectFocus()
    return
  }
  if (!parsedRobot.linkGroups.has(linkName)) return
  inspectFocusedLink = linkName
  restoreInspectMaterials(parsedRobot.group)
  applyInspectDimming(parsedRobot.group, linkName)
  startFocusCameraOnLink(linkName)
  urdfAssemblyApi?.setSelectedLink(linkName)
  openSidebarPanel('inspector')
}

function refreshInspectAfterModelUpdate() {
  if (viewportInteractionMode !== 'inspect' || !inspectFocusedLink) return
  if (!parsedRobot.linkGroups.has(inspectFocusedLink)) {
    clearInspectFocus()
    showToast('Focused link was removed', 'info')
    return
  }
  restoreInspectMaterials(parsedRobot.group)
  applyInspectDimming(parsedRobot.group, inspectFocusedLink)
  urdfAssemblyApi?.setSelectedLink(inspectFocusedLink)
}

document.getElementById('toggle-vp-mode')?.addEventListener('click', () => {
  if (simApi.isSimActive()) return  // locked to inspect while simulation is running
  if (document.body.classList.contains('ai-busy')) return  // locked to inspect during AI generation
  viewportInteractionMode = viewportInteractionMode === 'build' ? 'inspect' : 'build'
  syncViewportModeButton()
  urdfAssemblyApi?.onInteractionModeChanged(viewportInteractionMode)
  if (viewportInteractionMode === 'build') {
    clearInspectFocus()
  }
})
syncViewportModeButton()

// While the AI is mid-generation, force build → inspect (build mode lets the
// user drag-place components, which would race with the model's tool calls
// rewriting the URDF). The pre-generation mode is captured on the busy → idle
// edge so the user lands back where they were when the apply/dismiss prompt
// resolves. Mirrors the sim-mode lockout but driven by body.ai-busy so the
// editor inline-diff Accept path (which doesn't go through viewportChat's
// setAiBusy) still triggers restoration.
let _modeBeforeAiBusy: 'build' | 'inspect' | null = null
function _applyAiBusyModeFlip(busy: boolean) {
  if (busy && _modeBeforeAiBusy === null) {
    _modeBeforeAiBusy = viewportInteractionMode
    if (viewportInteractionMode === 'build') {
      viewportInteractionMode = 'inspect'
      syncViewportModeButton()
      urdfAssemblyApi?.onInteractionModeChanged('inspect')
    }
  } else if (!busy && _modeBeforeAiBusy !== null) {
    if (viewportInteractionMode !== _modeBeforeAiBusy) {
      viewportInteractionMode = _modeBeforeAiBusy
      syncViewportModeButton()
      urdfAssemblyApi?.onInteractionModeChanged(viewportInteractionMode)
      if (viewportInteractionMode === 'build') clearInspectFocus()
    }
    _modeBeforeAiBusy = null
  }
}
// Synchronous entry point for setAiBusy() in viewportChat — called BEFORE the
// body.ai-busy class change, so the mode flip lands before any AI tool call
// can touch the URDF. The MutationObserver below is the fallback for paths
// that toggle the class without going through viewportChat (e.g. inlineDiff
// Accept). Together they guarantee the flip happens regardless of caller.
;(window as unknown as { __setAiBusyMode: (busy: boolean) => void }).__setAiBusyMode =
  _applyAiBusyModeFlip
new MutationObserver(() => {
  _applyAiBusyModeFlip(document.body.classList.contains('ai-busy'))
}).observe(document.body, { attributes: true, attributeFilter: ['class'] })

// No auto-spawn of a default buffer on launch. The welcome state handles the
// no-tabs case (user clicks "+" or "Open File"/"Open Folder" to start).
// The async workspace-restore IIFE further down may still re-open previously
// open tabs from the last session, which is the intended startup flow.
if (openFiles.length === 0) {
  showWelcomeState()
}

urdfAssemblyApi = initAssemblyEditor({
  scene,
  camera,
  canvas,
  controls,
  showToast,
  switchPanel: openSidebarPanel,
  getUrdfText: () => monacoEditor.getModel()?.getValue() || SAMPLE_URDF,
  setUrdfText: (content: string) => {
    const model = monacoEditor.getModel()
    if (model) {
      model.setValue(content)
    } else {
      // No file open yet — create the model silently (don't reveal the editor panel).
      // The user can open the file from the explorer if they want to edit it.
      const filename = 'robot.urdf'
      if (!monacoModels[filename]) {
        monacoModels[filename] = monaco.editor.createModel(content, 'xml')
        openFiles.push(filename)
        filePaths[filename] = null
        const fn = filename
        const fnModel = monacoModels[filename]
        fnModel.onDidChangeContent(() => {
          if (activeFile !== fn) return
          if (simApi.isSimActive()) { showToast('Editor changed — restart simulation to apply', 'warning'); return }
          if (reparseTimeout !== null) clearTimeout(reparseTimeout)
          reparseTimeout = window.setTimeout(() => {
            reparseTimeout = null
            if (inlineDiffApi.getPendingOldText() !== null) return
            if (activeFile !== fn) return
            reparseURDF()
            urdfAssemblyApi?.onModelUpdated()
          }, 500)
        })
        renderTabs()
        renderExplorer()
      } else {
        monacoModels[filename].setValue(content)
      }
    }
  },
  reparseUrdf: (xml?: string, opts?: { skipGround?: boolean; ground?: boolean }) => reparseURDF(xml, opts),
  getParsedRobot: () => parsedRobot,
  getKinematicGraph: () => kinematicGraph,
  isViewport3D: () => viewportChatApi?.isViewport3D() ?? true,
  getInteractionMode: () => viewportInteractionMode,
  isSimActive: () => simApi.isSimActive(),
  onInspectLinkFocused: handleInspectLinkFocused,
  onAfterModelUpdated: refreshInspectAfterModelUpdate,
  zeroAssemblyWorldPosition: () => {
    robot.position.set(0, 0, 0)
  },
  groundAssembly: () => groundRobot(robot),
})

// ── Debug overlay inspectors (dev-only) ──
// Console-only diagnostics: `__inspectOverlays()` and `__inspectLink(name)`.
{
    // Diagnostic: report global debug-overlay state. Confirms whether
    // wireframeGroup, comGroup, etc. are actually visible (independent of
    // their button state) and whether they hold any clones.
    ;(window as unknown as { __inspectOverlays: () => void }).__inspectOverlays = () => {
      console.log('[overlays] wireframeGroup', {
        visible: wireframeGroup.visible,
        cloneCount: wireframeGroup.children.length,
        someClonesVisible: wireframeGroup.children.some(c => c.visible),
      })
      console.log('[overlays] comGroup', {
        visible: comGroup.visible,
        childCount: comGroup.children.length,
      })
      console.log('[overlays] axisVisuals (joint axes)', {
        count: axisVisuals.length,
        anyVisible: axisVisuals.some(o => o.visible),
      })
      // Walk robot for any Line/LineSegments/wireframe-material meshes that
      // ARE rendering (visible up the chain). If wireframe is "off" but
      // something gripper-shaped renders, this surfaces it.
      const renderingLines: Array<Record<string, unknown>> = []
      const isRenderingChain = (o: THREE.Object3D | null): boolean => {
        let cur = o
        while (cur) { if (!cur.visible) return false; cur = cur.parent }
        return true
      }
      robot.traverse(o => {
        const any = o as THREE.Mesh & { isLine?: boolean; isLineSegments?: boolean; material?: THREE.Material & { wireframe?: boolean } }
        const isLineish = any.isLine || any.isLineSegments
        const isWireMesh = any.isMesh && any.material && (any.material as { wireframe?: boolean }).wireframe === true
        if (!isLineish && !isWireMesh) return
        if (!isRenderingChain(o)) return
        renderingLines.push({
          kind: any.isLineSegments ? 'LineSegments' : any.isLine ? 'Line' : 'WireMesh',
          name: o.name || '<unnamed>',
          parent: o.parent?.name || '<no name>',
          parentChainOK: true,
          urdfLinkName: (o.userData as Record<string, unknown>)?.urdfLinkName,
          materialColor: (any.material as { color?: { getHexString(): string } } | undefined)?.color?.getHexString?.(),
        })
      })
      console.log(`[overlays] ${renderingLines.length} Line/LineSegments/wireframe meshes ACTUALLY RENDERING under robot:`)
      console.table(renderingLines)
    }
    // Diagnostic: dump every mesh under a given link name with its shadow +
    // visibility state. Use to confirm whether a stray mesh is still casting.
    ;(window as unknown as { __inspectLink: (name: string) => void }).__inspectLink = (name: string) => {
      const g = parsedRobot.linkGroups.get(name)
      if (!g) { console.warn(`[inspect] no linkGroup for ${name}`); return }
      const rows: Array<Record<string, unknown>> = []
      g.traverse(o => {
        const m = o as THREE.Mesh & { isLine?: boolean; isLineSegments?: boolean }
        if (!m.isMesh && !m.isLine && !m.isLineSegments) return
        const matAny = m.material as { transparent?: boolean; opacity?: number; color?: { getHexString(): string } } | undefined
        rows.push({
          kind: m.isLineSegments ? 'LineSegments' : m.isLine ? 'Line' : 'Mesh',
          name: m.name || '<unnamed>',
          geom: m.geometry?.type ?? '?',
          castShadow: m.castShadow,
          visible: m.visible,
          opacity: matAny?.transparent ? matAny.opacity : 1,
          color: matAny?.color ? '#' + matAny.color.getHexString() : undefined,
          parent: m.parent?.name || '<no name>',
          parentVisible: m.parent?.visible,
          urdfLinkName: (m.userData as Record<string, unknown>)?.urdfLinkName,
          isCollision: (m.userData as Record<string, unknown>)?.isCollision,
        })
      })
      console.table(rows)
    }
}

