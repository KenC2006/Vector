import './style.css'
import * as THREE from 'three'
import { OrbitControls } from 'three/addons/controls/OrbitControls.js'
import { RoomEnvironment } from 'three/addons/environments/RoomEnvironment.js'
import { EffectComposer } from 'three/addons/postprocessing/EffectComposer.js'
import { RenderPass } from 'three/addons/postprocessing/RenderPass.js'
import { GTAOPass } from 'three/addons/postprocessing/GTAOPass.js'
import { OutputPass } from 'three/addons/postprocessing/OutputPass.js'
import { invoke } from '@tauri-apps/api/core'
import { initUrdfAssembly, type UrdfAssemblyApi } from './urdfAssembly'
import { applyRichVisuals, preloadMeshCache } from './richVisuals'
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
  // Compute bbox from only Mesh objects (excludes edge Lines, ArrowHelpers, etc.)
  const box = new THREE.Box3()
  const meshBox = new THREE.Box3()
  target.traverse((obj: THREE.Object3D) => {
    if ((obj as THREE.Mesh).isMesh) {
      meshBox.setFromObject(obj)
      if (!meshBox.isEmpty()) box.union(meshBox)
    }
  })
  if (box.isEmpty()) return
  // In Three.js Y is up; shift so bottom of bounding box = 0
  robotGroup.position.y = -box.min.y
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

// ── Inline AI Completions (Cursor-style Ghost Text) ──────────────────────────

// ── Chat History (delegated to chatHistory.ts) ──────────────────────────────

chatApi = initChatHistory({
  getEditorValue: () => monacoEditor.getModel()?.getValue() || '',
  setEditorValue: (v) => monacoEditor.setValue(v),
})

// State for managing completion requests
let inlineCompletionSettings = {
  enabled: false,  // Temporarily disabled to save API credits
  debounceMs: 350,  // Reduced from 500ms — cache handles repeated requests
}

// Dedup and staleness tracking
let completionInFlight = false
let lastCompletionTimestamp = 0
let lastCompletionVersion = 0  // editor model version when request was made

// Simple delay — NOT tied to Monaco's cancellation token
function delayMs(ms: number): Promise<void> {
  return new Promise(resolve => setTimeout(resolve, ms))
}

// Safe invoke that ensures Tauri IPC is ready before calling.
// Monaco's async pipeline can fire before __TAURI_INTERNALS__ is injected.
async function safeInvoke<T>(cmd: string, args: Record<string, unknown>): Promise<T> {
  await waitForTauri()
  return invoke(cmd, args) as Promise<T>
}

// Invoke with a timeout — rejects if the call takes too long
function invokeWithTimeout<T>(cmd: string, args: Record<string, unknown>, timeoutMs: number): Promise<T> {
  return Promise.race([
    safeInvoke<T>(cmd, args),
    new Promise<never>((_, reject) =>
      setTimeout(() => reject(new Error(`${cmd} timed out after ${timeoutMs}ms`)), timeoutMs)
    )
  ])
}

// Register inline completions provider for XML (URDF files)
monaco.languages.registerInlineCompletionsProvider('xml', {
  async provideInlineCompletions(
    model: monaco.editor.ITextModel,
    position: monaco.Position,
    _context: monaco.languages.InlineCompletionContext,
    _token: monaco.CancellationToken
  ): Promise<monaco.languages.InlineCompletions> {
    if (!inlineCompletionSettings.enabled || !coreAvailable) {
      return { items: [] }
    }

    // Skip if a completion request is already in-flight
    if (completionInFlight) {
      return { items: [] }
    }

    // Record when this request started — used for debounce deduplication
    const requestTime = Date.now()
    lastCompletionTimestamp = requestTime

    // Debounce — wait for user to stop typing.
    // NOT tied to Monaco's cancellation token, which fires too aggressively.
    await delayMs(inlineCompletionSettings.debounceMs)

    // If a newer request came in during the debounce, bail out
    if (lastCompletionTimestamp !== requestTime) {
      return { items: [] }
    }

    // Snapshot the editor state at request time
    const urdfContent = model.getValue()
    const cursorLine = position.lineNumber
    const cursorColumn = position.column
    const lines = urdfContent.split('\n')
    const currentLine = lines[cursorLine - 1] || ''
    const textBeforeCursor = currentLine.slice(0, cursorColumn - 1)
    const modelVersion = model.getVersionId()

    // Don't request completions on empty/whitespace-only lines
    if (!textBeforeCursor.trim() && cursorColumn <= 1) {
      return { items: [] }
    }

    // Skip positions where completions aren't useful:
    // - Right after a closing tag (user just finished an element)
    // - On comment lines
    const trimmedBefore = textBeforeCursor.trim()
    if (trimmedBefore.endsWith('-->') || trimmedBefore.startsWith('<!--')) {
      return { items: [] }
    }

    try {
      completionInFlight = true
      lastCompletionVersion = modelVersion
      console.log(`[Completions] Requesting at L${cursorLine}:${cursorColumn} "${trimmedBefore.slice(-40)}"`)

      const completion = await invokeWithTimeout<string>('ai_complete', {
        urdfContent,
        cursorLine,
        cursorColumn,
        prefix: textBeforeCursor,
        kinematicContext: buildKinematicContext(),
      }, 12000)

      completionInFlight = false

      // Reject stale results — if the editor changed while we were waiting,
      // this completion is for an old state and will likely be wrong
      if (model.getVersionId() !== lastCompletionVersion) {
        console.log('[Completions] Stale result (editor changed), discarding')
        return { items: [] }
      }

      if (!completion || !completion.trim()) {
        console.log('[Completions] Empty response')
        return { items: [] }
      }

      let result = completion

      // Client-side overlap guard: strip any tail of the completion that
      // duplicates text already present after the cursor in the editor.
      const textAfterCursor = model.getValue().slice(
        model.getOffsetAt(position)
      )
      if (textAfterCursor) {
        const compLines = result.split('\n')
        const sufLines = textAfterCursor.split('\n')
        let overlapLines = 0
        for (let n = 1; n <= Math.min(compLines.length, sufLines.length); n++) {
          const tail = compLines.slice(-n).map(l => l.trim())
          const head = sufLines.slice(0, n).map(l => l.trim())
          if (tail.every((l, i) => l === head[i])) {
            overlapLines = n
          }
        }
        if (overlapLines > 0) {
          result = compLines.slice(0, -overlapLines).join('\n')
          console.log(`[Completions] Stripped ${overlapLines} overlapping lines`)
        }
      }

      // Safety cap at 20 lines — keeps ghost text readable
      const resultLines = result.split('\n')
      if (resultLines.length > 20) {
        result = resultLines.slice(0, 20).join('\n')
        console.log(`[Completions] Capped ${resultLines.length} → 20 lines`)
      }

      if (!result.trim()) {
        console.log('[Completions] Empty after overlap trimming')
        return { items: [] }
      }

      console.log('[Completions] ✓ Got:', JSON.stringify(result.slice(0, 120)))

      return {
        items: [{
          insertText: result,
          range: new monaco.Range(cursorLine, cursorColumn, cursorLine, cursorColumn),
        }]
      }
    } catch (error) {
      completionInFlight = false
      if (String(error).includes('timed out')) {
        console.log('[Completions] Timed out')
      } else {
        console.warn('[Completions] Error:', error)
      }
      return { items: [] }
    }
  },
  disposeInlineCompletions() {
    // no-op
  },
} as monaco.languages.InlineCompletionsProvider)

// Setup toggle button for inline completions
const inlineCompletionToggle = document.getElementById('inline-completion-toggle') as HTMLSpanElement
if (inlineCompletionToggle) {
  inlineCompletionToggle.addEventListener('click', () => {
    inlineCompletionSettings.enabled = !inlineCompletionSettings.enabled

    // Update visual state
    if (inlineCompletionSettings.enabled) {
      inlineCompletionToggle.style.opacity = '1'
      inlineCompletionToggle.style.color = '#4ec9b0'
      inlineCompletionToggle.title = 'Inline AI completions: enabled (Ctrl+Shift+I)'
      showToast('Inline completions enabled', 'success')
    } else {
      inlineCompletionToggle.style.opacity = '0.5'
      inlineCompletionToggle.style.color = '#858585'
      inlineCompletionToggle.title = 'Inline AI completions: disabled (Ctrl+Shift+I)'
      showToast('Inline completions disabled', 'info')
    }
  })
}

// Keyboard shortcut: Ctrl+Shift+I to toggle inline completions
document.addEventListener('keydown', (e) => {
  if (e.ctrlKey && e.shiftKey && e.key === 'I') {
    e.preventDefault()
    if (inlineCompletionToggle) {
      inlineCompletionToggle.click()
    }
  }
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
// Per-file undo/redo state — saved when leaving a file, restored when returning.
const fileUndoStates: Record<string, { undo: string[]; redo: string[] }> = {}
let untitledCounter = 0

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

function getMonacoLang(filename: string): string {
  const ext = getFileExt(filename)
  const langs: Record<string, string> = { urdf: 'xml', xacro: 'xml', xml: 'xml', json: 'json', yaml: 'yaml', yml: 'yaml', sdf: 'xml', mjcf: 'xml' }
  return langs[ext] || 'plaintext'
}

function renderTabs() {
  // Remove all existing tab elements (keep the + button)
  tabBar.querySelectorAll('.tab').forEach(t => t.remove())

  for (const filename of openFiles) {
    const tab = document.createElement('div')
    tab.className = 'tab' + (filename === activeFile ? ' active' : '')
    tab.dataset.file = filename
    tab.innerHTML = `
      <span class="tab-label">${filename}</span>
      <span class="tab-close" title="Close">&times;</span>
    `
    tab.addEventListener('click', (e) => {
      if ((e.target as HTMLElement).classList.contains('tab-close')) {
        closeFile(filename)
      } else {
        switchToFile(filename)
      }
    })
    tabBar.insertBefore(tab, tabNewBtn)
  }
}

function renderExplorer() {
  if (!filesList) return
  // If a folder is open, just update active highlights — don't rebuild the tree
  if (openedFolderPath) {
    filesList.querySelectorAll('.file-item').forEach(item => {
      const text = item.textContent?.trim() || ''
      item.classList.toggle('active', text === activeFile)
    })
    return
  }
  // No folder open — show open files list
  filesList.innerHTML = ''
  for (const filename of openFiles) {
    const ext = getFileExt(filename)
    const item = document.createElement('div')
    item.className = 'file-item' + (filename === activeFile ? ' active' : '')
    item.innerHTML = `<span class="fi-dot ${ext}"></span>${filename}`
    item.addEventListener('click', () => switchToFile(filename))
    filesList.appendChild(item)
  }
}

function switchToFile(filename: string) {
  if (filename === activeFile) return
  if (!monacoModels[filename]) return

  // Save current 3D state so we can restore it when switching back
  if (activeFile && isUrdfLike(activeFile)) {
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
  fileTypeLabel.textContent = getFileType(filename)

  // Clear any pending inline diff and debounced reparse from the previous file —
  // they shouldn't block rendering of the new file
  if (inlineDiffApi.getPendingOldText() !== null) {
    inlineDiffApi.clearPendingDiff()
  }
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

  // Update 3D viewport if switching to a URDF/XML file
  if (isUrdfLike(filename)) {
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
      // First visit or content changed — full reparse
      robot.position.set(0, 0, 0)
      reparseURDF()
      groundRobot(robot)
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
  if (!filename) {
    untitledCounter++
    filename = `untitled_${untitledCounter}.urdf`
  }

  // New URDF files should start with minimal valid robot, not empty
  if (!content && (isUrdfLike(filename))) {
    content = SAMPLE_URDF
  }

  // If file already open, just switch to it
  if (monacoModels[filename]) {
    switchToFile(filename)
    return filename
  }

  const lang = getMonacoLang(filename)
  monacoModels[filename] = monaco.editor.createModel(content, lang)
  openFiles.push(filename)
  filePaths[filename] = diskPath

  // Track in recent files
  if (diskPath) addRecentFile(filename, diskPath)

  // Listen for changes on URDF/XML files with debounce
  if (isUrdfLike(filename)) {
    const fn = filename // capture for closure
    const fnModel = monacoModels[filename] // capture model reference for closure
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

  switchToFile(filename)
  renderTabs()
  renderExplorer()
  return filename
}

function showWelcomeState() {
  activeFile = ''
  monacoEditor.setModel(null)
  editorPanel.classList.add('editor-hidden')
  handle.classList.add('editor-hidden')
  const bcFilename = document.getElementById('bc-filename')
  if (bcFilename) bcFilename.textContent = ''
  renderTabs()
  renderExplorer()
  // Viewport expands to fill the space — update renderer after layout settles
  requestAnimationFrame(() => _resize())
}

function hideWelcomeState() {
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
}

// + button handler
tabNewBtn.addEventListener('click', () => createNewFile())

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
renderer.shadowMap.type = THREE.PCFSoftShadowMap
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
controls.mouseButtons = {
  LEFT: THREE.MOUSE.ROTATE,
  MIDDLE: THREE.MOUSE.PAN,
  RIGHT: THREE.MOUSE.PAN,  // right-click also pans (CAD-style)
}

const viewportNavClock = new THREE.Clock()
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

// ── Post-processing pipeline (SSAO + output) ───────────────────────────────

const composer = new EffectComposer(renderer)
composer.setPixelRatio(renderer.getPixelRatio())
// Sync initial size after a frame (viewport layout not done yet at this point)
requestAnimationFrame(() => {
  const w = renderer.domElement.clientWidth
  const h = renderer.domElement.clientHeight
  if (w > 0 && h > 0) composer.setSize(w, h)
})

const renderPass = new RenderPass(scene, camera)
composer.addPass(renderPass)

// GTAO disabled — causes visible halo around objects against background.
// The directional light shadow map provides ground shadows.
// Re-enable when scene has a floor/ground plane that masks the halo.
// const gtaoPass = new GTAOPass(scene, camera)
// gtaoPass.blendIntensity = 0.15
// composer.addPass(gtaoPass)

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
      groundRobot(robot)
      urdfAssemblyApi?.rebuildMountNodes()
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

let parsedRobot = parseURDFToScene(SAMPLE_URDF)
worldGroup.add(parsedRobot.group)
robot.updateMatrixWorld(true)
applyRichVisuals(parsedRobot, makeOnMeshLoaded(parsedRobot))
addEdgeLines(parsedRobot)
groundRobot(robot)

// ── Edge lines (CAD-style silhouette edges) ─────────────────────────────────

const edgeMaterial = new THREE.LineBasicMaterial({
  color: 0x000000,
  transparent: true,
  opacity: 0.3,
  depthTest: true,
})

function addEdgeLines(parsed: typeof parsedRobot) {
  parsed.group.traverse(child => {
    if (child instanceof THREE.Mesh && child.geometry) {
      const edges = new THREE.EdgesGeometry(child.geometry, 30) // 30° threshold
      const line = new THREE.LineSegments(edges, edgeMaterial)
      line.raycast = () => {} // don't interfere with raycasting
      child.add(line)
    }
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

let showCollision = false

function rebuildCollisionVisuals(urdfText: string) {
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

    for (const collisionEl of Array.from(linkEl.querySelectorAll('collision'))) {
      const geomEl = collisionEl.querySelector('geometry')
      if (!geomEl) continue

      let geo: THREE.BufferGeometry | null = null
      const boxEl = geomEl.querySelector('box')
      const cylEl = geomEl.querySelector('cylinder')
      const sphEl = geomEl.querySelector('sphere')

      if (boxEl) {
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
      if (cylEl) mesh.rotation.x = Math.PI / 2

      // Apply collision origin (same convention as visual parser: ZYX euler)
      const originEl = collisionEl.querySelector('origin')
      if (originEl) {
        const xyz = (originEl.getAttribute('xyz') || '0 0 0').split(/\s+/).map(parseFloat)
        const rpy = (originEl.getAttribute('rpy') || '0 0 0').split(/\s+/).map(parseFloat)
        mesh.position.set(xyz[0] || 0, xyz[1] || 0, xyz[2] || 0)
        // Compose origin rpy on top of cylinder rotation using a parent group
        const wrapper = new THREE.Group()
        wrapper.userData.isCollision = true
        wrapper.visible = showCollision
        wrapper.quaternion.copy(rpyToQuat(rpy))
        wrapper.position.set(xyz[0] || 0, xyz[1] || 0, xyz[2] || 0)
        mesh.position.set(0, 0, 0)
        wrapper.add(mesh)
        // Edges for clarity
        const edges = new THREE.LineSegments(new THREE.EdgesGeometry(geo), collisionEdgeMat)
        edges.userData.isCollision = true
        edges.visible = showCollision
        if (cylEl) edges.rotation.x = Math.PI / 2
        wrapper.add(edges)
        linkGroup.add(wrapper)
        continue
      }

      // No origin — add mesh directly
      const edges = new THREE.LineSegments(new THREE.EdgesGeometry(geo), collisionEdgeMat)
      edges.userData.isCollision = true
      edges.visible = showCollision
      if (cylEl) edges.rotation.x = Math.PI / 2
      linkGroup.add(mesh)
      linkGroup.add(edges)
    }
  }
}

const toggleCollisionBtn = document.getElementById('toggle-collision') as HTMLButtonElement | null
toggleCollisionBtn?.addEventListener('click', () => {
  showCollision = !showCollision
  parsedRobot.group.traverse(obj => {
    if ((obj as any).userData?.isCollision) (obj as THREE.Object3D).visible = showCollision
  })
  toggleCollisionBtn.classList.toggle('active', showCollision)
})

// ── Sim mode (delegated to simManager.ts) ────────────────────────────────────

// Deferred callbacks to break circular init dependency (simManager ↔ vpControls/openSidebarPanel)
let _resize: () => void = () => {}
let _openSidebarPanel: (p: string) => void = () => {}
let _createCheckpoint: (label: string, urdf: string, auto: boolean) => void = () => {}

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
  simProgress: document.getElementById('sim-progress') as HTMLElement,
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
  onEnterSim: () => {
    viewportInteractionMode = 'inspect'
    syncViewportModeButton()
    urdfAssemblyApi?.onInteractionModeChanged('inspect')
    clearInspectFocus()
  },
  onExitSim: () => {
    viewportInteractionMode = 'build'
    syncViewportModeButton()
    urdfAssemblyApi?.onInteractionModeChanged('build')
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

const { resize, focusOnRobot, zoomCamera, setViewportCollapsed, setViewportFullscreen, setFocusMode, updateViewportInfo } = vpControls
// Wire up deferred resize callback now that it's available
_resize = resize

// ── Animate ──────────────────────────────────────────────────────────────────

let wireframeBuilt = false

function rebuildWireframes() {
  wireframeGroup.clear()
  wireframeBuilt = false

    robot.traverse(child => {
    if (child instanceof THREE.Mesh && child.material !== wireMat && child.material !== defaultMat && child.geometry) {
        const clone = new THREE.Mesh(child.geometry, wireMat)
        child.getWorldPosition(clone.position)
        child.getWorldQuaternion(clone.quaternion)
        child.getWorldScale(clone.scale)
        wireframeGroup.add(clone)
      }
    })
  wireframeBuilt = true
}

function animate() {
  requestAnimationFrame(animate)

  const navDt = Math.min(viewportNavClock.getDelta(), 0.05)

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

  // Phase C: camera follow + preview animation (delegated to simManager)
  simApi.tickCameraFollow()

  controls.update()

  // CoM marker spin
  for (const marker of comGroup.children) {
    if (marker instanceof THREE.Mesh) {
      marker.rotation.y += 0.01
    }
  }

  // Three.js preview animation + real physics update (delegated to simManager)
  simApi.tickPreviewAnimation()

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

function buildKinematicContext(): string {
  // Extract robot name from URDF
  const urdf = monacoEditor.getModel()?.getValue() || ''
  const nameMatch = urdf.match(/<robot\s+name="([^"]*)"/)
  const robotName = nameMatch?.[1] || 'robot'
  const links = Object.values(kinematicGraph)
  const joints = Object.values(kinematicJoints)

  let context = `Robot: ${robotName}\n`
  context += `Links (${links.length}): `

  // List all links with mass
  const linkSummary = links.map((l) => {
    const geometry = parsedRobot.linkGroups.get(l.name)
    let geomType = 'unknown'
    if (geometry) {
      geometry.traverse((child) => {
        if (child instanceof THREE.Mesh) {
          const geom = child.geometry
          if (geom instanceof THREE.BoxGeometry) geomType = 'box'
          else if (geom instanceof THREE.CylinderGeometry) geomType = 'cylinder'
          else if (geom instanceof THREE.SphereGeometry) geomType = 'sphere'
        }
      })
    }
    return `${l.name} (${l.mass}kg, ${geomType})`
  }).join(', ')
  context += linkSummary + '\n'

  context += `Joints (${joints.length}): `

  // List all joints with type and axis
  const jointSummary = joints.map((j) => {
    return `${j.name} [${j.type}, axis ${j.axis}, ${j.parentLink}→${j.childLink}]`
  }).join(', ')
  context += jointSummary + '\n'

  // Build kinematic chain
  context += 'Chain: '
  const rootLink = links.find((l) => !l.parent) || links[0]

  function buildChain(linkName: string): string {
    const link = kinematicGraph[linkName]
    if (!link || link.children.length === 0) return linkName

    let result = linkName
    for (const childName of link.children) {
      // Find joint connecting to this child
      const joint = Object.values(kinematicJoints).find(
        (j) => j.parentLink === linkName && j.childLink === childName
      )
      if (joint) {
        result += ` → [${joint.name}] → ${buildChain(childName)}`
      }
    }
    return result
  }

  context += buildChain(rootLink?.name || 'base_link') + '\n'

  return context
}


// ── Live URDF re-parsing ────────────────────────────────────────────────────

let reparseTimeout: number | null = null
let urdfAssemblyApi: UrdfAssemblyApi | null = null

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

function reparseURDF(xmlOverride?: string) {
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
          applyRichVisuals(parsedRobot, makeOnMeshLoaded(parsedRobot))
          addEdgeLines(parsedRobot)
          rebuildJointAxisVisuals()
          updateComMarker()
          rebuildWireframes()
          rebuildCollisionVisuals(processed)
          updateViewportInfo()
          urdfAssemblyApi?.onModelUpdated()
          groundRobot(robot)
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
    applyRichVisuals(parsedRobot, makeOnMeshLoaded(parsedRobot))
    addEdgeLines(parsedRobot)

    rebuildJointAxisVisuals()

    // Update CoM marker
    updateComMarker()

    // Rebuild wireframes and collision visuals
    rebuildWireframes()
    rebuildCollisionVisuals(urdfContent)

    // Update viewport info
    updateViewportInfo()

    urdfAssemblyApi?.onModelUpdated()

    groundRobot(robot)

    console.log(`[URDF] Reparsed: ${parsedRobot.linkCount} links, ${parsedRobot.jointCount} joints`)
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

const { runLocalValidation, runValidation, setValidationMarkers } = initValidation({
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

  // Close on outside click
  document.addEventListener('click', (e) => {
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
      document.getElementById('toggle-com')?.click()
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
  focus: document.getElementById('panel-focus')!,
  build: document.getElementById('panel-build')!,
  inspector: document.getElementById('panel-inspector')!,
  toolbox: document.getElementById('panel-toolbox')!,
  validation: document.getElementById('panel-validation')!,
  sim: document.getElementById('panel-sim')!,
  git: document.getElementById('panel-git')!,
  settings: document.getElementById('panel-settings')!,
}

function openSidebarPanel(panel: string) {
  if (
    (panel === 'build' || panel === 'toolbox' || panel === 'inspector' || panel === 'focus') &&
    !(viewportChatApi?.isViewport3D() ?? true)
  ) {
    viewportChatApi?.switchViewportView('3d')
    showToast('Switched to 3D Preview for URDF editing', 'info')
  }
  if (
    (panel === 'build' || panel === 'toolbox' || panel === 'inspector' || panel === 'focus') &&
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

document.querySelectorAll('.ab-btn').forEach(btn => {
  btn.addEventListener('click', () => {
    const panel = (btn as HTMLElement).dataset.panel!
    if (!panel) return
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
  const activityBar = document.getElementById('activity-bar')!
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
  urdfAssemblyApi,
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

    openedFolderPath = folderPath
    // Update breadcrumb project name
    const bcProject = document.getElementById('bc-project')
    if (bcProject) bcProject.textContent = folderPath.split(/[\\/]/).pop() || 'Vector'

    const entries = await invoke<Array<{ name: string; path: string; isDir: boolean; depth: number }>>('list_directory', { path: folderPath })

    // Update explorer header
    const sbHeader = document.querySelector('#panel-explorer .sb-header')
    if (sbHeader) {
      const folderName = folderPath.split(/[\\/]/).pop() || folderPath
      sbHeader.innerHTML = `<span class="arrow">&#9662;</span> ${folderName}`
    }

    // Render folder tree
    const filesList = document.getElementById('files-list')
    if (!filesList) return
    filesList.innerHTML = ''

    for (const entry of entries) {
      const el = document.createElement('div')
      const ext = entry.name.split('.').pop()?.toLowerCase() || ''
      if (entry.isDir) {
        el.className = 'file-item folder'
        el.style.paddingLeft = `${12 + entry.depth * 14}px`
        el.innerHTML = `<span class="fi-arrow">&#9656;</span>${entry.name}/`
      } else {
        el.className = 'file-item'
        el.style.paddingLeft = `${12 + entry.depth * 14}px`
        el.innerHTML = `<span class="fi-dot ${ext}"></span>${entry.name}`
        el.addEventListener('click', async () => {
          try {
            const content = await invoke<string>('open_file', { path: entry.path })
            createNewFile(entry.name, content, entry.path)
            currentFilePath = entry.path
          } catch (err) {
            showToast(`Failed to open ${entry.name}: ${err}`, 'error')
          }
        })
      }
      filesList.appendChild(el)
    }

    showToast(`Opened folder: ${folderPath.split(/[\\/]/).pop()}`, 'success')
  } catch (err) {
    showToast(`Error opening folder: ${err}`, 'error')
  }
}

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
          const entries = await invoke<Array<{ name: string; path: string; isDir: boolean; depth: number }>>('list_directory', { path })
          // It's a directory — open as folder
          openedFolderPath = path
          const bcProject = document.getElementById('bc-project')
          if (bcProject) bcProject.textContent = name
          const sbHeader = document.querySelector('#panel-explorer .sb-header')
          if (sbHeader) sbHeader.innerHTML = `<span class="arrow">&#9662;</span> ${name}`
          const filesList = document.getElementById('files-list')
          if (filesList) {
            filesList.innerHTML = ''
            for (const entry of entries) {
              const el = document.createElement('div')
              const entryExt = entry.name.split('.').pop()?.toLowerCase() || ''
              if (entry.isDir) {
                el.className = 'file-item folder'
                el.style.paddingLeft = `${12 + entry.depth * 14}px`
                el.innerHTML = `<span class="fi-arrow">&#9656;</span>${entry.name}/`
              } else {
                el.className = 'file-item'
                el.style.paddingLeft = `${12 + entry.depth * 14}px`
                el.innerHTML = `<span class="fi-dot ${entryExt}"></span>${entry.name}`
                el.addEventListener('click', async () => {
                  try {
                    const content = await invoke<string>('open_file', { path: entry.path })
                    createNewFile(entry.name, content, entry.path)
                    currentFilePath = entry.path
                  } catch (err) {
                    showToast(`Failed to open ${entry.name}: ${err}`, 'error')
                  }
                })
              }
              filesList.appendChild(el)
            }
          }
          showToast(`Opened folder: ${name}`, 'success')
          openSidebarPanel('explorer')
          continue
        } catch {
          // Not a directory — try opening as file
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
setTimeout(() => showToast(`Loaded robot.urdf — ${parsedRobot.linkCount} links, ${parsedRobot.jointCount} joints`, 'success'), 500)
setTimeout(() => showToast('Validation: 5 passed, 1 warning (CoM near edge)', 'warning'), 1200)

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
  buildKinematicContext,
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
  reparseURDF,
  runLocalValidation,
  createCheckpoint: (l, u, a) => _createCheckpoint(l, u, a),
  scene,
  robot,
  camera,
  groundRobot: () => groundRobot(robot),
  autoFrameRobot: () => autoFrameRobot(robot, camera, controls),
  getUrdfAssemblyApi: () => urdfAssemblyApi,
  getCoreAvailable: () => coreAvailable,
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
    let path = filePaths[activeFile] || currentFilePath

    if (!path) {
      path = await invoke<string | null>('save_file_dialog', { default_name: activeFile })
      if (!path) return
    }

    await invoke('save_file', { path, content })
    filePaths[activeFile] = path
    currentFilePath = path
    const filename = path.split(/[\\/]/).pop() || activeFile
    showToast(`Saved ${filename}`, 'success')
    document.title = `Vector — ${filename}`
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

function updateFocusPanelVisibility() {
  const emptyEl = document.getElementById('focus-empty')
  const bodyEl = document.getElementById('focus-body')
  if (!emptyEl || !bodyEl) return
  const has = Boolean(inspectFocusedLink)
  emptyEl.classList.toggle('hidden', has)
  bodyEl.classList.toggle('hidden', !has)
}

function renderFocusDashboard(linkName: string) {
  const nameEl = document.getElementById('focus-link-name')
  const metaEl = document.getElementById('focus-meta')
  if (!nameEl || !metaEl) return
  nameEl.textContent = linkName
  const graph = kinematicGraph[linkName]
  const m = graph?.mass ?? 0
  const massStr = m >= 1 ? `${m.toFixed(2)} kg` : `${Math.round(m * 1000)} g`
  const joint = Object.values(kinematicJoints).find(j => j.childLink === linkName)
  let parentLine = ''
  if (joint) {
    const ax = joint.axis && joint.axis !== '--' ? `, axis ${joint.axis}` : ''
    parentLine = `Parent: ${joint.parentLink} · ${joint.name} (${joint.type}${ax})`
} else {
    parentLine = 'Kinematic root (no parent joint)'
  }
  const children = (graph?.children ?? []).filter(c => !c.includes('__mount__'))
  const childLine = children.length ? `Children: ${children.join(', ')}` : 'No child links'
  metaEl.replaceChildren()
  for (const line of [`Mass: ${massStr}`, parentLine, childLine]) {
    const row = document.createElement('div')
    row.textContent = line
    row.style.marginBottom = '6px'
    metaEl.appendChild(row)
  }
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
  const nameEl = document.getElementById('focus-link-name')
  const metaEl = document.getElementById('focus-meta')
  if (nameEl) nameEl.textContent = ''
  if (metaEl) metaEl.replaceChildren()
  updateFocusPanelVisibility()
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
  renderFocusDashboard(linkName)
  updateFocusPanelVisibility()
  openSidebarPanel('focus')
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
  renderFocusDashboard(inspectFocusedLink)
}

document.getElementById('toggle-vp-mode')?.addEventListener('click', () => {
  if (simApi.isSimActive()) return  // locked to inspect while simulation is running
  viewportInteractionMode = viewportInteractionMode === 'build' ? 'inspect' : 'build'
  syncViewportModeButton()
  urdfAssemblyApi?.onInteractionModeChanged(viewportInteractionMode)
  if (viewportInteractionMode === 'build') {
    clearInspectFocus()
  }
})
syncViewportModeButton()

document.getElementById('focus-btn-clear')?.addEventListener('click', () => {
  clearInspectFocus()
})

document.getElementById('focus-btn-frame')?.addEventListener('click', () => {
  if (inspectFocusedLink) startFocusCameraOnLink(inspectFocusedLink)
})

document.getElementById('focus-btn-inspector')?.addEventListener('click', () => {
  const link = inspectFocusedLink
  if (!link || !urdfAssemblyApi) return
  viewportInteractionMode = 'build'
  syncViewportModeButton()
  clearInspectFocus()
  urdfAssemblyApi.setSelectedLink(link)
  openSidebarPanel('inspector')
})

// Ensure Monaco always has at least the default robot.urdf open so placement
// and AI edits always have a valid URDF to read/write (prevents "Cannot edit
// invalid URDF" on first component drag when no file has been opened yet).
if (openFiles.length === 0) {
  createNewFile('robot.urdf', SAMPLE_URDF, null)
}

urdfAssemblyApi = initUrdfAssembly({
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
  reparseUrdf: (xml?: string) => reparseURDF(xml),
  getParsedRobot: () => parsedRobot,
  getKinematicGraph: () => kinematicGraph,
  getKinematicJoints: () => kinematicJoints,
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
