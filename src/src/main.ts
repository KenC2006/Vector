import './style.css'
import * as THREE from 'three'
import { OrbitControls } from 'three/addons/controls/OrbitControls.js'
import { RoomEnvironment } from 'three/addons/environments/RoomEnvironment.js'
import { EffectComposer } from 'three/addons/postprocessing/EffectComposer.js'
import { RenderPass } from 'three/addons/postprocessing/RenderPass.js'
import { GTAOPass } from 'three/addons/postprocessing/GTAOPass.js'
import { OutputPass } from 'three/addons/postprocessing/OutputPass.js'
import { invoke } from '@tauri-apps/api/core'
import { listen } from '@tauri-apps/api/event'
import { initUrdfAssembly } from './urdfAssembly'
import { applyRichVisuals } from './richVisuals'
import { SAMPLE_URDF } from './sampleUrdf'
import { registerThemes, initSettings, VIEWPORT_BG, type ThemeId } from './settings'
import { initGitPanel } from './gitPanel'
import { initValidation } from './validation'
import { parseURDFToScene, buildKinematicGraphFromURDF, setPathResolver, defaultMat } from './urdfParser'
import { initNodeGraph } from './nodeGraph'
import { initViewportControls } from './viewportControls'

/** Raise the robot group so its lowest geometry point touches Y=0 (ground). */
function groundRobot(robotGroup: THREE.Group) {
  robotGroup.updateMatrixWorld(true)
  const box = new THREE.Box3().setFromObject(robotGroup)
  if (box.isEmpty()) return
  // In Three.js Y is up; shift so bottom of bounding box = 0
  if (box.min.y < -0.001) {
    robotGroup.position.y -= box.min.y
  }
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

// ── Chat History System ──────────────────────────────────────────────────────

interface ChatMessage {
  role: 'user' | 'assistant' | 'system'
  content: string
  timestamp: number
}

interface ChatConversation {
  id: string
  title: string
  createdAt: number
  updatedAt: number
  messages: ChatMessage[]
}

const MAX_CHATS = 20
let chatHistory: ChatConversation[] = JSON.parse(localStorage.getItem('vector_chats') || '[]')
let currentChatId: string = ''
let currentChatMessages: ChatMessage[] = []

function generateChatId(): string {
  return `chat_${Date.now()}_${Math.random().toString(36).slice(2, 8)}`
}

function saveChatHistory() {
  while (chatHistory.length > MAX_CHATS) chatHistory.shift()
  localStorage.setItem('vector_chats', JSON.stringify(chatHistory))
}

function getCurrentChat(): ChatConversation | undefined {
  return chatHistory.find(c => c.id === currentChatId)
}

function updateChatDropdown() {
  const select = document.getElementById('vc-chat-select') as HTMLSelectElement | null
  if (!select) return
  select.innerHTML = ''
  // Newest first
  for (let i = chatHistory.length - 1; i >= 0; i--) {
    const chat = chatHistory[i]
    const opt = document.createElement('option')
    opt.value = chat.id
    opt.textContent = chat.title || 'Untitled'
    if (chat.id === currentChatId) opt.selected = true
    select.appendChild(opt)
  }
}

function startNewChat() {
  const id = generateChatId()
  const chat: ChatConversation = {
    id,
    title: 'New Chat',
    createdAt: Date.now(),
    updatedAt: Date.now(),
    messages: [],
  }
  chatHistory.push(chat)
  currentChatId = id
  currentChatMessages = chat.messages
  saveChatHistory()
  updateChatDropdown()

  // Clear chat UI
  const vcMsgs = document.getElementById('vc-messages')
  if (vcMsgs) {
    vcMsgs.innerHTML = `<div class="ai-msg system">
      <div class="ai-msg-content">Describe changes to your robot in natural language. I'll edit the URDF, show you a diff, and highlight changes inline in the editor.</div>
    </div>`
  }
}

function loadChat(chatId: string) {
  const chat = chatHistory.find(c => c.id === chatId)
  if (!chat) return
  currentChatId = chatId
  currentChatMessages = chat.messages

  // Rebuild chat UI from stored messages
  const vcMsgs = document.getElementById('vc-messages')
  if (!vcMsgs) return
  vcMsgs.innerHTML = `<div class="ai-msg system">
    <div class="ai-msg-content">Describe changes to your robot in natural language.</div>
  </div>`
  for (const msg of chat.messages) {
    const el = document.createElement('div')
    el.className = `ai-msg ${msg.role}`
    el.innerHTML = `<div class="ai-msg-content">${msg.role === 'user' ? escapeHtml(msg.content) : msg.content}</div>`
    vcMsgs.appendChild(el)
  }
  vcMsgs.scrollTop = vcMsgs.scrollHeight
  updateChatDropdown()
}

function recordChatMessage(role: 'user' | 'assistant' | 'system', content: string) {
  const msg: ChatMessage = { role, content, timestamp: Date.now() }
  currentChatMessages.push(msg)

  const chat = getCurrentChat()
  if (chat) {
    chat.updatedAt = Date.now()
    // Auto-title from first user message
    if (!chat.title || chat.title === 'New Chat') {
      const firstUser = currentChatMessages.find(m => m.role === 'user')
      if (firstUser) chat.title = firstUser.content.slice(0, 50)
    }
    saveChatHistory()
    updateChatDropdown()
  }
}

// Initialize: load most recent chat or create new one
if (chatHistory.length > 0) {
  const latest = chatHistory[chatHistory.length - 1]
  currentChatId = latest.id
  currentChatMessages = latest.messages
} else {
  startNewChat()
}

// Use currentChatId as the session ID for the AI backend
const aiSessionId = currentChatId

// State for managing completion requests
let inlineCompletionSettings = {
  enabled: true,
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
let untitledCounter = 0

function getFileExt(filename: string): string {
  const dot = filename.lastIndexOf('.')
  return dot >= 0 ? filename.slice(dot + 1).toLowerCase() : ''
}

function getFileType(filename: string): string {
  const ext = getFileExt(filename)
  const types: Record<string, string> = { urdf: 'URDF', xml: 'XML', json: 'JSON', yaml: 'YAML', yml: 'YAML', sdf: 'SDF', mjcf: 'MJCF', txt: 'TEXT' }
  return types[ext] || 'TEXT'
}

function getMonacoLang(filename: string): string {
  const ext = getFileExt(filename)
  const langs: Record<string, string> = { urdf: 'xml', xml: 'xml', json: 'json', yaml: 'yaml', yml: 'yaml', sdf: 'xml', mjcf: 'xml' }
  return langs[ext] || 'plaintext'
}

function getTabIconClass(filename: string): string {
  const ext = getFileExt(filename)
  if (ext === 'urdf' || ext === 'xml' || ext === 'sdf' || ext === 'mjcf') return 'tab-icon-urdf'
  if (ext === 'json') return 'tab-icon-json'
  if (ext === 'yaml' || ext === 'yml') return 'tab-icon-yaml'
  return 'tab-icon-urdf'
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

  // Reparse 3D viewport if switching to a URDF/XML file
  if (getFileExt(filename) === 'urdf' || getFileExt(filename) === 'xml') {
    reparseURDF()
    urdfAssemblyApi?.onModelUpdated()
    runLocalValidation()

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
  if (!content && (getFileExt(filename) === 'urdf' || getFileExt(filename) === 'xml')) {
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
  if (getFileExt(filename) === 'urdf' || getFileExt(filename) === 'xml') {
    const fn = filename // capture for closure
    monacoModels[filename].onDidChangeContent(() => {
      if (activeFile === fn) {
        if (reparseTimeout !== null) clearTimeout(reparseTimeout)
        reparseTimeout = window.setTimeout(() => {
          reparseURDF()
          urdfAssemblyApi?.onModelUpdated()
          reparseTimeout = null
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
  const welcomeEl = document.getElementById('editor-welcome')
  if (welcomeEl) welcomeEl.style.display = 'flex'
  const bcFilename = document.getElementById('bc-filename')
  if (bcFilename) bcFilename.textContent = ''
  renderTabs()
  renderExplorer()
}

function hideWelcomeState() {
  const welcomeEl = document.getElementById('editor-welcome')
  if (welcomeEl) welcomeEl.style.display = 'none'
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

let parsedRobot = parseURDFToScene(SAMPLE_URDF)
// URDF uses Z-up, Three.js uses Y-up: rotate the entire robot -90° around X
parsedRobot.group.rotation.x = -Math.PI / 2
robot.add(parsedRobot.group)
applyRichVisuals(parsedRobot)
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

let jointAxisVisible = true
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
  group.visible = jointAxisVisible
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

// ── Sim mode ─────────────────────────────────────────────────────────────────

const simToggle = document.getElementById('sim-toggle') as HTMLButtonElement
const simBar = document.getElementById('sim-bar') as HTMLDivElement
const simPlay = document.getElementById('sim-play') as HTMLButtonElement
const simPause = document.getElementById('sim-pause') as HTMLButtonElement
const simReset = document.getElementById('sim-reset') as HTMLButtonElement
const simProgress = document.getElementById('sim-progress') as HTMLDivElement
const simTimeEl = document.getElementById('sim-time') as HTMLSpanElement
const viewportLabel = document.querySelector('.vp-tab[data-view="3d"]') as HTMLButtonElement

let simRunning = false
let simActive = false
let simTime = 0

// Original sim event listeners removed — replaced by persistent-core versions below

function updateSimUI() {
  simPlay.classList.toggle('active', simRunning)
  simPause.classList.toggle('active', !simRunning && simActive)
  simTimeEl.textContent = simTime.toFixed(3) + 's'
  simProgress.style.width = `${Math.min((simTime / 10) * 100, 100)}%`
}

// ── Viewport controls (delegated to viewportControls.ts) ────────────────────

const handle = document.getElementById('resize-handle') as HTMLDivElement
const editorPanel = document.getElementById('editor-panel') as HTMLDivElement

const vpControls = initViewportControls({
  camera, renderer, controls, robot, scene, canvas, viewportPanel, editorPanel, handle,
  originAxes, grid, comGroup, wireframeGroup, axisVisuals,
  simBar,
  simActive: () => simActive,
  parsedRobot: () => parsedRobot,
  showToast,
  onResize: (w, h) => { composer.setSize(w, h); composer.setPixelRatio(renderer.getPixelRatio()) },
})

const { resize, focusOnRobot, zoomCamera, setViewportCollapsed, setViewportFullscreen, updateViewportInfo } = vpControls

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
      wireframeGroup.add(clone)
    }
  })
  wireframeBuilt = true
}

function animate() {
  requestAnimationFrame(animate)

  // Build wireframes once
  if (!wireframeBuilt) {
    rebuildWireframes()
  }

  controls.update()

  // CoM marker spin
  for (const marker of comGroup.children) {
    if (marker instanceof THREE.Mesh) {
      marker.rotation.y += 0.01
    }
  }

  // Sim animation
  if (simRunning) {
    simTime += 1 / 60
    updateSimUI()
    const t = simTime

    // Animate revolute joints with smooth sinusoidal motion
    const jointMotion: Record<string, number> = {
      'shoulder_pan': Math.sin(t * 0.8) * 0.6,
      'shoulder_lift': Math.sin(t * 0.6 + 0.5) * 0.3 - 0.2,
      'elbow': Math.sin(t * 1.2) * 0.5 + 0.3,
      'finger_left_joint': Math.sin(t * 2) * 0.5 + 0.5,
      'finger_right_joint': Math.sin(t * 2) * 0.5 + 0.5,
    }

    // Apply motion to parsed joints
    for (const [jointName, motion] of Object.entries(jointMotion)) {
      const jointInfo = parsedRobot.joints.get(jointName)
      if (jointInfo) {
        const axis = jointInfo.axis
        // Create rotation based on axis direction
        const quat = new THREE.Quaternion()
        quat.setFromAxisAngle(axis, motion)
        jointInfo.group.quaternion.copy(quat)
      }
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
let urdfAssemblyApi: { onModelUpdated(): void; recordUndoExternal(content: string): void } | null = null

function reparseURDF() {
  try {
    const model = monacoEditor.getModel()
    if (!model) return  // no file open
    const urdfContent = model.getValue()
    const newParsed = parseURDFToScene(urdfContent)
    const newKinematicData = buildKinematicGraphFromURDF(urdfContent)

    // Clear old robot geometry
    robot.remove(parsedRobot.group)
    wireframeGroup.clear()
    axisVisuals.length = 0

    // Update parsed data
    parsedRobot = newParsed
    kinematicGraph = newKinematicData.kinematicGraph
    kinematicJoints = newKinematicData.kinematicJoints

    // Add new geometry with Z-up → Y-up rotation
    parsedRobot.group.rotation.x = -Math.PI / 2
    robot.add(parsedRobot.group)
    applyRichVisuals(parsedRobot)
    addEdgeLines(parsedRobot)
    groundRobot(robot)

    // Rebuild axis visuals
    const axisColors = [0x4a9eff, 0x4ec9b0, 0xf48771, 0xce9178]
    let colorIdx = 0
    for (const [, jointInfo] of parsedRobot.joints) {
      const color = axisColors[colorIdx++ % axisColors.length]
      addJointAxis(jointInfo.group, jointInfo.axis, color)
    }

    // Update CoM marker
    updateComMarker()

    // Rebuild wireframes
    rebuildWireframes()

    // Update viewport info
    updateViewportInfo()

    // Rebuild kinematic tree
    buildKinematicTreeUI()
    urdfAssemblyApi?.onModelUpdated()

    console.log(`[URDF] Reparsed: ${parsedRobot.linkCount} links, ${parsedRobot.jointCount} joints`)
  } catch (e) {
    console.error('[URDF] Parse error:', e)
    // Keep old geometry on parse error
  }
}

// Content change listener is now handled by createNewFile() for all URDF/XML files.
// Attach to the initial robot.urdf model:
if (monacoModels['robot.urdf']) {
  monacoModels['robot.urdf'].onDidChangeContent(() => {
    if (activeFile === 'robot.urdf') {
      if (reparseTimeout !== null) clearTimeout(reparseTimeout)
      reparseTimeout = window.setTimeout(() => {
        reparseURDF()
        urdfAssemblyApi?.onModelUpdated()
        reparseTimeout = null
      }, 500)
    }
  })
}

// ── Build Kinematic Tree UI ──────────────────────────────────────────────────
// Populates the kinematic tree with links, joints, masses, and geometry types

function buildKinematicTreeUI() {
  const treeContainer = document.getElementById('kinematic-tree')!
  treeContainer.innerHTML = ''

  // Helper to get geometry type for a link
  function getGeometryType(linkName: string): string {
    const linkGroup = parsedRobot.linkGroups.get(linkName)
    if (!linkGroup) return 'unknown'

    let geomType = 'unknown'
    linkGroup.traverse((child) => {
      if (child instanceof THREE.Mesh) {
        const geom = child.geometry
        if (geom instanceof THREE.BoxGeometry) geomType = 'box'
        else if (geom instanceof THREE.CylinderGeometry) geomType = 'cylinder'
        else if (geom instanceof THREE.SphereGeometry) geomType = 'sphere'
      }
    })
    return geomType
  }

  function buildNode(linkName: string, depth: number = 0) {
    const link = kinematicGraph[linkName]
    if (!link) return

    const nodeEl = document.createElement('div')
    nodeEl.className = 'kt-node link'
    nodeEl.style.paddingLeft = `${10 + depth * 12}px`

    const hasChildren = link.children.length > 0
    const toggleEl = document.createElement('div')
    toggleEl.className = `kt-toggle ${hasChildren ? 'expanded' : ''}`
    if (!hasChildren) toggleEl.style.opacity = '0'

    // Extract geometry type
    const geomType = getGeometryType(linkName)

    const labelEl = document.createElement('span')
    labelEl.textContent = `${link.name} (${link.mass}kg, ${geomType})`
    labelEl.title = `Link: ${link.name}\nMass: ${link.mass}kg\nGeometry: ${geomType}`

    nodeEl.appendChild(toggleEl)
    nodeEl.appendChild(labelEl)

    // Click to highlight in 3D
    labelEl.style.cursor = 'pointer'
    labelEl.addEventListener('click', (e) => {
      e.stopPropagation()
      highlightMesh(linkName)
    })

    // Toggle expand/collapse
    if (hasChildren) {
      toggleEl.style.cursor = 'pointer'
      toggleEl.addEventListener('click', (e) => {
        e.stopPropagation()
        const childrenDiv = nodeEl.nextElementSibling
        if (childrenDiv && childrenDiv.classList.contains('kt-children')) {
          childrenDiv.classList.toggle('visible')
          toggleEl.classList.toggle('expanded')
          toggleEl.classList.toggle('collapsed')
        }
      })
    }

    treeContainer.appendChild(nodeEl)

    // Add joints and children
    if (hasChildren) {
      const childrenDiv = document.createElement('div')
      childrenDiv.className = 'kt-children visible'

      for (const childName of link.children) {
        // Find the joint connecting to this child
        for (const joint of Object.values(kinematicJoints)) {
          if (joint.parentLink === linkName && joint.childLink === childName) {
            const jointEl = document.createElement('div')
            jointEl.className = 'kt-node joint'
            jointEl.style.paddingLeft = `${30 + depth * 12}px`
            jointEl.textContent = `↳ ${joint.name} [${joint.type}, ${joint.axis}]`

            // Show tooltip on hover
            jointEl.addEventListener('mouseenter', () => {
              jointEl.title = `Type: ${joint.type}\nAxis: ${joint.axis}\nParent: ${joint.parentLink}\nChild: ${joint.childLink}`
            })

            childrenDiv.appendChild(jointEl)
            break
          }
        }

        // Recursively add child link
        const tempDiv = document.createElement('div')
        treeContainer.appendChild(tempDiv)
        const oldAppend = treeContainer.appendChild
        treeContainer.appendChild = function (el: any) {
          tempDiv.parentElement!.insertBefore(el, tempDiv.nextSibling)
          return el
        }
        buildNode(childName, depth + 1)
        treeContainer.appendChild = oldAppend
        tempDiv.remove()
      }

      treeContainer.appendChild(childrenDiv)
    }
  }

  buildNode('base_link')
}

buildKinematicTreeUI()

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

const { highlightMesh, clearHighlight } = nodeGraph

const toggleGraphBtn = document.getElementById('toggle-graph') as HTMLButtonElement

toggleGraphBtn.addEventListener('click', () => {
  nodeGraph.toggle()
})

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

  // Don't trigger shortcuts when typing in inputs or Monaco editor
  const tag = (e.target as HTMLElement).tagName
  if (tag === 'INPUT' || tag === 'TEXTAREA' || tag === 'SELECT') return
  if ((e.target as HTMLElement).closest('.monaco-editor')) return

  switch (e.key.toLowerCase()) {
    case 'a':
      document.getElementById('toggle-axes')?.click()
      break
    case 'c':
      document.getElementById('toggle-com')?.click()
      break
    case 'w':
      document.getElementById('toggle-wireframe')?.click()
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
      setViewportFullscreen(!vpControls.viewportFullscreen())
      break
    case 'escape':
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

// ── Git Source Control Panel ────────────────────────────────────────────────
const { refreshGitStatus } = initGitPanel({ invoke, showToast })

// ── Activity bar ─────────────────────────────────────────────────────────────

const panels: Record<string, HTMLElement> = {
  explorer: document.getElementById('panel-explorer')!,
  build: document.getElementById('panel-build')!,
  inspector: document.getElementById('panel-inspector')!,
  toolbox: document.getElementById('panel-toolbox')!,
  validation: document.getElementById('panel-validation')!,
  kinematic: document.getElementById('panel-kinematic')!,
  git: document.getElementById('panel-git')!,
  settings: document.getElementById('panel-settings')!,
}

function openSidebarPanel(panel: string) {
  if ((panel === 'build' || panel === 'toolbox' || panel === 'inspector') && activeViewportView !== '3d') {
    switchViewportView('3d')
    showToast('Switched to 3D Preview for URDF editing', 'info')
  }
  if ((panel === 'build' || panel === 'toolbox' || panel === 'inspector') && nodeGraph.isVisible()) {
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

// ── Status badge ─────────────────────────────────────────────────────────────
const statusText = document.getElementById('status-text') as HTMLSpanElement
const statusDot = document.getElementById('status-dot') as HTMLSpanElement
const statusBadge = document.getElementById('status-badge') as HTMLDivElement

function setStatus(text: string, color: string) {
  statusText.textContent = text
  statusDot.style.background = color
  statusBadge.style.color = color
  statusBadge.style.borderColor = color + '33'
  statusBadge.style.background = color + '15'
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

// ── AI Utility Functions (used by viewport chat) ────────────────────────────
function escapeHtml(text: string): string {
  return text.replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;')
}

function computeSimpleDiff(oldText: string, newText: string): { added: string[], removed: string[] } {
  const oldLines = oldText.split('\n')
  const newLines = newText.split('\n')
  const added: string[] = []
  const removed: string[] = []

  const oldSet = new Set(oldLines.map(l => l.trim()))
  const newSet = new Set(newLines.map(l => l.trim()))

  for (const line of oldLines) {
    if (!newSet.has(line.trim()) && line.trim()) removed.push(line)
  }
  for (const line of newLines) {
    if (!oldSet.has(line.trim()) && line.trim()) added.push(line)
  }

  return { added, removed }
}


// ── Viewport Tab Switching (3D Preview / AI Chat) ────────────────────────────
const viewportCanvas = document.getElementById('viewport') as HTMLCanvasElement
const viewportChat = document.getElementById('viewport-chat')!
const vcMessages = document.getElementById('vc-messages')!
const vcInput = document.getElementById('vc-input') as HTMLTextAreaElement
const vcSend = document.getElementById('vc-send') as HTMLButtonElement
const viewportTabs = document.querySelectorAll('.vp-tab')
// @ts-ignore — read by external debug tools
let activeViewportView: '3d' | 'chat' = '3d'

// Initialize chat UI from stored history or show default message
if (currentChatMessages.length > 0) {
  loadChat(currentChatId)
} else {
  vcMessages.innerHTML = `<div class="ai-msg system">
    <div class="ai-msg-content">Describe changes to your robot in natural language. I'll edit the URDF, show you a diff, and highlight changes inline in the editor.</div>
  </div>`
}
updateChatDropdown()

// Wire chat header controls
const vcChatSelect = document.getElementById('vc-chat-select') as HTMLSelectElement | null
const vcNewChatBtn = document.getElementById('vc-new-chat') as HTMLButtonElement | null

vcChatSelect?.addEventListener('change', () => {
  if (vcChatSelect.value && vcChatSelect.value !== currentChatId) {
    loadChat(vcChatSelect.value)
  }
})

vcNewChatBtn?.addEventListener('click', () => startNewChat())

function switchViewportView(view: '3d' | 'chat') {
  activeViewportView = view
  viewportTabs.forEach(tab => {
    tab.classList.toggle('active', (tab as HTMLElement).dataset.view === view)
  })
  if (view === '3d') {
    viewportCanvas.style.display = ''
    viewportChat.classList.add('hidden')
    document.getElementById('viewport-info')!.style.display = ''
    resize()
  } else {
    viewportCanvas.style.display = 'none'
    viewportChat.classList.remove('hidden')
    document.getElementById('viewport-info')!.style.display = 'none'
    vcInput.focus()
  }
}

viewportTabs.forEach(tab => {
  tab.addEventListener('click', () => {
    switchViewportView((tab as HTMLElement).dataset.view as '3d' | 'chat')
  })
})

// Shared function to add message to viewport chat
function addVCMessage(role: 'user' | 'assistant' | 'system', content: string, extras?: {
  diff?: { added: string[], removed: string[] },
  newUrdf?: string,
}) {
  // Persist to chat history (strip HTML tags for storage)
  const plainContent = content.replace(/<[^>]*>/g, '').trim()
  if (plainContent) recordChatMessage(role, plainContent)

  const msg = document.createElement('div')
  msg.className = `ai-msg ${role}`

  if (role === 'user') {
    msg.innerHTML = `<div class="ai-msg-content">${escapeHtml(content)}</div>`
  } else if (role === 'assistant') {
    let html = `<div class="ai-msg-content">${content}</div>`

    if (extras?.diff && (extras.diff.added.length > 0 || extras.diff.removed.length > 0)) {
      html += `<div class="ai-msg-diff">
        <div class="ai-msg-diff-header">
          <span>robot.urdf</span>
          <span>${extras.diff.added.length} added, ${extras.diff.removed.length} removed</span>
        </div>`
      for (const line of extras.diff.removed) {
        html += `<div class="ai-diff-line removed">${escapeHtml(line)}</div>`
      }
      for (const line of extras.diff.added) {
        html += `<div class="ai-diff-line added">${escapeHtml(line)}</div>`
      }
      html += `</div>`
    }

    if (extras?.newUrdf) {
      const msgId = 'vc-msg-' + Date.now()
      activeChatActionsId = msgId
      html += `<div class="ai-msg-actions" id="${msgId}">
        <button class="ai-accept" data-action="accept">Apply Changes</button>
        <button class="ai-reject" data-action="reject">Dismiss</button>
      </div>`
      msg.innerHTML = html

      setTimeout(() => {
        const actions = document.getElementById(msgId)
        if (!actions) return
        const acceptBtn = actions.querySelector('.ai-accept') as HTMLButtonElement
        const rejectBtn = actions.querySelector('.ai-reject') as HTMLButtonElement

        acceptBtn.addEventListener('click', () => {
          activeChatActionsId = null  // prevent syncChatActions from double-updating
          acceptInlineDiff()
          acceptBtn.textContent = '✓ Applied'
          acceptBtn.className = 'ai-applied'
          rejectBtn.style.display = 'none'
        })

        rejectBtn.addEventListener('click', () => {
          activeChatActionsId = null  // prevent syncChatActions from double-updating
          dismissInlineDiff()
          rejectBtn.textContent = '✗ Dismissed'
          rejectBtn.className = 'ai-rejected'
          acceptBtn.style.display = 'none'
        })
      }, 0)
    } else {
      msg.innerHTML = html
    }
  } else {
    msg.innerHTML = `<div class="ai-msg-content">${content}</div>`
  }

  vcMessages.appendChild(msg)
  vcMessages.scrollTop = vcMessages.scrollHeight
  return msg
}

function addVCThinking(): HTMLElement & { updateStage: (stage: string, text: string) => void } {
  const msg = document.createElement('div') as unknown as HTMLElement & { updateStage: (stage: string, text: string) => void }
  msg.className = 'ai-msg assistant'
  msg.innerHTML = `<div class="ai-thinking">
    <span class="dot"></span><span class="dot"></span><span class="dot"></span>
    <span class="ai-thinking-text">Thinking...</span>
  </div>
  <div class="ai-streaming-preview" style="display:none"></div>`
  vcMessages.appendChild(msg)
  vcMessages.scrollTop = vcMessages.scrollHeight

  const stageLabels: Record<string, string> = {
    thinking: 'Analyzing model...',
    generating: 'Generating design...',
    streaming: '',
    applying: 'Applying changes...',
    done: 'Done',
  }

  msg.updateStage = (stage: string, text: string) => {
    const thinkingText = msg.querySelector('.ai-thinking-text') as HTMLElement | null
    const preview = msg.querySelector('.ai-streaming-preview') as HTMLElement | null
    if (!thinkingText) return

    if (stage === 'streaming' && text && preview) {
      thinkingText.textContent = 'Generating...'
      preview.style.display = 'block'
      preview.textContent = text
    } else {
      thinkingText.textContent = stageLabels[stage] || text || 'Processing...'
    }
    vcMessages.scrollTop = vcMessages.scrollHeight
  }

  return msg
}

// ── Inline Diff in Monaco ────────────────────────────────────────────────────
let inlineDiffCollection: monaco.editor.IEditorDecorationsCollection | null = null
let inlineDiffWidget: HTMLElement | null = null
let pendingOldText: string | null = null
let activeChatActionsId: string | null = null  // tracks the chat message's Accept/Dismiss buttons

function showInlineDiff(oldText: string, newText: string, _newUrdf?: string) {
  const editor = (window as any).__vectorEditor as monaco.editor.IStandaloneCodeEditor | undefined
  if (!editor) return

  // Store old text so Dismiss can revert
  pendingOldText = oldText

  const oldLines = oldText.split('\n')
  const newLines = newText.split('\n')

  // Find which lines in the new text differ from old
  const maxLen = Math.max(oldLines.length, newLines.length)
  const changedLines: number[] = []

  for (let i = 0; i < maxLen; i++) {
    const oldLine = i < oldLines.length ? oldLines[i] : undefined
    const newLine = i < newLines.length ? newLines[i] : undefined
    if (oldLine !== newLine && newLine !== undefined) {
      changedLines.push(i + 1) // Monaco is 1-indexed
    }
  }

  // Show the new content in the editor as a preview
  editor.setValue(newText)

  // Build decorations for changed/added lines (green highlight)
  const decorations: monaco.editor.IModelDeltaDecoration[] = changedLines.map(lineNum => ({
    range: new monaco.Range(lineNum, 1, lineNum, 1),
    options: {
      isWholeLine: true,
      className: 'inline-diff-added',
      linesDecorationsClassName: 'inline-diff-gutter-added',
    }
  }))

  // Use createDecorationsCollection (Monaco 0.36+, replaces deprecated deltaDecorations)
  if (inlineDiffCollection) {
    inlineDiffCollection.clear()
  }
  inlineDiffCollection = editor.createDecorationsCollection(decorations)

  // Show floating accept/dismiss bar at top of editor
  if (inlineDiffWidget) inlineDiffWidget.remove()
  const bar = document.createElement('div')
  bar.className = 'inline-diff-bar'
  bar.innerHTML = `
    <span class="idb-label">${changedLines.length} lines changed</span>
    <button class="idb-accept">✓ Accept</button>
    <button class="idb-dismiss">✗ Dismiss</button>
  `
  const editorEl = document.getElementById('monaco-container')!
  const rect = editorEl.getBoundingClientRect()
  bar.style.top = (rect.top + 8) + 'px'
  bar.style.right = (window.innerWidth - rect.right + 20) + 'px'
  document.body.appendChild(bar)
  inlineDiffWidget = bar

  bar.querySelector('.idb-accept')!.addEventListener('click', () => acceptInlineDiff())
  bar.querySelector('.idb-dismiss')!.addEventListener('click', () => dismissInlineDiff())

  // Scroll to the first changed line
  if (changedLines.length > 0) {
    editor.revealLineInCenter(changedLines[0])
  }
}

function acceptInlineDiff() {
  const editor = (window as any).__vectorEditor as monaco.editor.IStandaloneCodeEditor | undefined

  // Push pre-AI state to undo stack so Ctrl+Z works after accepting
  if (pendingOldText && urdfAssemblyApi) {
    urdfAssemblyApi.recordUndoExternal(pendingOldText)
  }

  // Auto-create checkpoint before AI edit
  if (pendingOldText) {
    createCheckpoint('Before AI edit', pendingOldText, true)
  }

  clearInlineDiff()
  pendingOldText = null
  showToast('Changes accepted', 'success')

  // Sync chat buttons to show "Applied"
  syncChatActions('accept')

  // Sync the 3D viewport with the accepted URDF
  if (editor) {
    reparseURDF()

    // Run local validation on the accepted changes (avoids blocking Mutex)
    runLocalValidation()
  }
}

function dismissInlineDiff() {
  const editor = (window as any).__vectorEditor as monaco.editor.IStandaloneCodeEditor | undefined
  if (editor && pendingOldText !== null) {
    editor.setValue(pendingOldText)
  }
  clearInlineDiff()
  pendingOldText = null
  showToast('Changes dismissed', 'info')

  // Sync chat buttons to show "Dismissed"
  syncChatActions('dismiss')
}

/** Update the chat Apply/Dismiss buttons to reflect the action taken (from editor bar or chat) */
function syncChatActions(action: 'accept' | 'dismiss') {
  if (!activeChatActionsId) return
  const actions = document.getElementById(activeChatActionsId)
  if (!actions) return
  const acceptBtn = actions.querySelector('.ai-accept') as HTMLButtonElement | null
  const rejectBtn = actions.querySelector('.ai-reject') as HTMLButtonElement | null
  if (action === 'accept') {
    if (acceptBtn) { acceptBtn.textContent = '✓ Applied'; acceptBtn.className = 'ai-applied' }
    if (rejectBtn) { rejectBtn.style.display = 'none' }
  } else {
    if (rejectBtn) { rejectBtn.textContent = '✗ Dismissed'; rejectBtn.className = 'ai-rejected' }
    if (acceptBtn) { acceptBtn.style.display = 'none' }
  }
  activeChatActionsId = null
}

function clearInlineDiff() {
  if (inlineDiffCollection) {
    inlineDiffCollection.clear()
    inlineDiffCollection = null
  }
  if (inlineDiffWidget) {
    inlineDiffWidget.remove()
    inlineDiffWidget = null
  }
}

// ── Viewport Chat Send ───────────────────────────────────────────────────────
async function sendVCMessage(prompt: string, retryCount = 0) {
  if (!prompt.trim()) return

  // Auto-create a file if none is open
  if (!monacoEditor.getModel()) {
    createNewFile('robot.urdf', SAMPLE_URDF, null)
  }

  if (retryCount === 0) {
    addVCMessage('user', prompt)
    vcInput.value = ''
    vcInput.style.height = 'auto'
  }

  vcSend.disabled = true
  setStatus('thinking', '#569cd6')

  const thinking = addVCThinking()

  // Listen for streaming progress events from Python → Rust → Frontend
  let unlisten: (() => void) | null = null
  try {
    unlisten = await listen<{ stage: string; text: string }>('ai_progress', (event) => {
      thinking.updateStage(event.payload.stage, event.payload.text)
    })
  } catch {
    // listen may fail in dev mode without Tauri — non-critical
  }

  try {
    const editor = (window as any).__vectorEditor
    const currentUrdf = editor?.getValue() || ''
    const kinematicContext = buildKinematicContext()

    const result = await invoke('ai_edit', {
      prompt: prompt,
      urdfContent: currentUrdf,
      kinematicContext: kinematicContext,
      sessionId: currentChatId,
    }) as { explanation: string; new_urdf: string; stats: string }

    thinking.remove()

    const diff = computeSimpleDiff(currentUrdf, result.new_urdf)

    addVCMessage('assistant', `${result.explanation}<br><span style="color:#858585;font-size:11px">${result.stats}</span>`, {
      diff,
      newUrdf: result.new_urdf,
    })

    showInlineDiff(currentUrdf, result.new_urdf, result.new_urdf)

  } catch (err) {
    thinking.remove()
    const errStr = String(err)
    console.warn('[VC] Backend error:', err)

    // Handle rate limit with auto-retry
    if (errStr.includes('429') || errStr.includes('rate_limit') ) {
      if (retryCount < 2) {
        const waitSec = (retryCount + 1) * 5
        addVCMessage('system', `<span style="color:#e5c07b;">Rate limited. Retrying in ${waitSec}s...</span>`)
        await new Promise(r => setTimeout(r, waitSec * 1000))
        // Retry (don't re-add the user message)
        unlisten?.()
        vcSend.disabled = false
        return sendVCMessage(prompt, retryCount + 1)
      }
      addVCMessage('assistant', `<span style="color:#f85149;">Rate limited after ${retryCount + 1} attempts. Please wait a moment and try again.</span>`)
    } else {
      const errorMsg = `<span style="color:#f85149;">Error: ${escapeHtml(errStr.slice(0, 200))}</span>`
      addVCMessage('assistant', errorMsg)
    }
  } finally {
    unlisten?.()
    vcSend.disabled = false
    setStatus('ready', '#608b4e')
  }
}

vcInput.addEventListener('keydown', (e) => {
  if (e.key === 'Enter' && !e.shiftKey) {
    e.preventDefault()
    sendVCMessage(vcInput.value)
  }
})

vcSend.addEventListener('click', () => sendVCMessage(vcInput.value))

vcInput.addEventListener('input', () => {
  vcInput.style.height = 'auto'
  vcInput.style.height = Math.min(vcInput.scrollHeight, 120) + 'px'
})

// Ctrl+L now switches to viewport chat
document.removeEventListener('keydown', () => {}) // cleanup
document.addEventListener('keydown', (e) => {
  if (e.ctrlKey && e.key === 'l') {
    e.preventDefault()
    switchViewportView('chat')
    vcInput.focus()
  }
})

// ── Simulation Mode Integration ──────────────────────────────────────────────

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

// Simulation state
let simCoreRunning = false
let simStepIntervalId: number | null = null
// Store original joint poses before sim so we can restore on exit
const originalJointPoses = new Map<string, { position: THREE.Vector3, quaternion: THREE.Quaternion }>()

// Joint state display
const simStateDisplay = document.createElement('div')
simStateDisplay.id = 'sim-state-display'
simStateDisplay.className = 'sim-state-display'
simStateDisplay.style.cssText = `
  position: absolute;
  top: 48px;
  right: 12px;
  background: rgba(30, 30, 30, 0.95);
  border: 1px solid #3c3c3c;
  border-radius: 6px;
  padding: 12px;
  font-family: monospace;
  font-size: 11px;
  color: #cccccc;
  max-width: 240px;
  max-height: 300px;
  overflow-y: auto;
  z-index: 100;
  display: none;
  backdrop-filter: blur(8px);
`
viewportPanel.appendChild(simStateDisplay)

async function initializeSimulation() {
  try {
    console.log('[Sim] Initializing simulation core...')
    // Try to start core — if already running, that's fine
    try {
      await invoke('start_core')
      console.log('[Sim] Core started successfully')
    } catch (coreErr) {
      const msg = String(coreErr).toLowerCase()
      if (msg.includes('already running') || msg.includes('already started')) {
        console.log('[Sim] Core already running, continuing...')
      } else {
        throw coreErr // Re-throw if it's a different error
      }
    }
    simCoreRunning = true

    console.log('[Sim] Loading robot model...')
    try {
      // TODO: Load the active URDF file path instead of hardcoded test file
      const simPath = currentFilePath || 'core/test_data/simple_arm.urdf'
      await invoke('sim_load', { path: simPath })
      console.log('[Sim] Robot model loaded')
    } catch (loadErr) {
      console.warn('[Sim] Could not load model (sim features limited):', loadErr)
    }

    console.log('[Sim] Getting initial state...')
    try {
      const initialState = await invoke('sim_get_state')
      console.log('[Sim] Initial state:', initialState)
      simStateDisplay.style.display = 'block'
      updateSimStateDisplay(initialState)
    } catch (stateErr) {
      console.warn('[Sim] Could not get initial state:', stateErr)
    }
  } catch (error) {
    console.error('[Sim] Error initializing simulation:', error)
    showToast(`Simulation error: ${String(error)}`, 'error')
    simCoreRunning = false
  }
}

async function shutdownSimulation() {
  try {
    if (simStepIntervalId !== null) {
      clearInterval(simStepIntervalId)
      simStepIntervalId = null
    }
    await invoke('stop_core')
    simCoreRunning = false
    simStateDisplay.style.display = 'none'
    console.log('[Sim] Core stopped')
  } catch (error) {
    console.error('[Sim] Error stopping simulation:', error)
    showToast(`Error stopping simulation: ${String(error)}`, 'error')
  }
}

async function stepSimulation() {
  if (!simCoreRunning) return
  try {
    await invoke('sim_step', { n_steps: 1 })
    const state = await invoke('sim_get_state')
    updateSimStateDisplay(state)
    updateRobotFromSimState(state)
  } catch (error) {
    console.error('[Sim] Error stepping simulation:', error)
  }
}

function updateSimStateDisplay(state: any) {
  try {
    let html = '<div style="font-weight: bold; color: #569cd6; margin-bottom: 8px;">Simulation State</div>'

    if (state && typeof state === 'object') {
      // Display time
      if (state.time !== undefined) {
        html += `<div><span style="color: #dcdcaa;">time:</span> ${(state.time as number).toFixed(3)}s</div>`
      }

      // Display joint states
      if (state.joints && typeof state.joints === 'object') {
        html += '<div style="margin-top: 6px; color: #858585;">Joints:</div>'
        for (const [name, joint] of Object.entries(state.joints)) {
          if (typeof joint === 'object' && joint !== null) {
            const j = joint as any
            const pos = j.position?.toFixed(3) || '0.000'
            const vel = j.velocity?.toFixed(3) || '0.000'
            html += `<div style="margin-left: 8px;">
              <span style="color: #9cdcfe;">${name}</span>
              <div style="margin-left: 8px; color: #858585; font-size: 10px;">
                pos: ${pos} | vel: ${vel}
              </div>
            </div>`
          }
        }
      }

      // Display contact info
      if (state.contacts !== undefined) {
        html += `<div style="margin-top: 6px; color: #858585;">Contacts: <span style="color: #f14c4c;">${state.contacts}</span></div>`
      }

      // Display energy
      if (state.energy !== undefined) {
        html += `<div style="margin-top: 6px; color: #858585;">Energy: <span style="color: #569cd6;">${(state.energy as number).toFixed(3)}J</span></div>`
      }
    }

    simStateDisplay.innerHTML = html
  } catch (e) {
    console.error('[Sim] Error updating display:', e)
  }
}

function updateRobotFromSimState(state: any) {
  try {
    if (!state || !state.joints) return

    const joints = state.joints as any

    // Update parsed joints from simulation state
    for (const [jointName, jointInfo] of parsedRobot.joints) {
      if (joints[jointName]?.position !== undefined) {
        const position = joints[jointName].position as number
        // Create rotation based on axis
        const quat = new THREE.Quaternion()
        quat.setFromAxisAngle(jointInfo.axis, position)
        jointInfo.group.quaternion.copy(quat)
      }
    }
  } catch (e) {
    console.error('[Sim] Error updating robot from state:', e)
  }
}

// Update sim mode toggle to use persistent core
simToggle.addEventListener('click', async () => {
  simActive = !simActive
  simBar.classList.toggle('hidden', !simActive)
  simToggle.classList.toggle('running', simActive)
  simToggle.querySelector('span')!.textContent = simActive ? 'Exit Sim' : 'Simulate'
  viewportLabel.textContent = simActive ? 'Simulation' : '3D Preview'

  if (simActive) {
    // Enter simulation mode — save original joint poses first
    originalJointPoses.clear()
    for (const [jointName, jointInfo] of parsedRobot.joints) {
      originalJointPoses.set(jointName, {
        position: jointInfo.group.position.clone(),
        quaternion: jointInfo.group.quaternion.clone()
      })
    }
    try {
      await initializeSimulation()
      showToast('Entered simulation mode (MuJoCo)', 'success')
    } catch (error) {
      console.error('[Sim] Failed to initialize:', error)
      simActive = false
      simToggle.classList.remove('running')
      simBar.classList.add('hidden')
      showToast('Failed to initialize simulation', 'error')
    }
  } else {
    // Exit simulation mode
    simRunning = false
    simTime = 0
    await shutdownSimulation()

    // Restore original joint poses (don't zero them — that destroys URDF offsets)
    for (const [jointName, jointInfo] of parsedRobot.joints) {
      const original = originalJointPoses.get(jointName)
      if (original) {
        jointInfo.group.position.copy(original.position)
        jointInfo.group.quaternion.copy(original.quaternion)
      } else {
        // Fallback: only reset rotation, never zero position
        jointInfo.group.quaternion.identity()
      }
    }
    originalJointPoses.clear()
    updateSimUI()
    showToast('Exited simulation mode', 'info')
  }
  resize()
})

// Update play/pause to use persistent stepping
simPlay.addEventListener('click', () => {
  if (!simCoreRunning) return
  simRunning = true
  if (simStepIntervalId !== null) {
    clearInterval(simStepIntervalId)
  }
  // Step at ~60 Hz
  simStepIntervalId = setInterval(async () => {
    await stepSimulation()
  }, 1000 / 60) as unknown as number
  updateSimUI()
})

simPause.addEventListener('click', () => {
  simRunning = false
  if (simStepIntervalId !== null) {
    clearInterval(simStepIntervalId)
    simStepIntervalId = null
  }
  updateSimUI()
})

simReset.addEventListener('click', async () => {
  if (!simCoreRunning) return
  simRunning = false
  if (simStepIntervalId !== null) {
    clearInterval(simStepIntervalId)
    simStepIntervalId = null
  }
  try {
    await invoke('sim_reset')
    const state = await invoke('sim_get_state')
    updateRobotFromSimState(state)
    simTime = 0
    updateSimUI()
  } catch (error) {
    console.error('[Sim] Reset error:', error)
  }
})

// Optional: Test core integration on page load
async function testCoreIntegration() {
  try {
    console.log('[Test] Core integration available (invoke function loaded)')
  } catch (error) {
    console.warn('[Test] Tauri invoke not available in this context')
  }
}

if (document.readyState === 'loading') {
  document.addEventListener('DOMContentLoaded', () => {
    setTimeout(testCoreIntegration, 500)
  })
} else {
  setTimeout(testCoreIntegration, 500)
}

urdfAssemblyApi = initUrdfAssembly({
  scene,
  camera,
  canvas,
  controls,
  showToast,
  switchPanel: openSidebarPanel,
  getUrdfText: () => monacoEditor.getModel()?.getValue() || '',
  setUrdfText: (content: string) => { if (monacoEditor.getModel()) monacoEditor.setValue(content) },
  reparseUrdf: reparseURDF,
  getParsedRobot: () => parsedRobot,
  getKinematicGraph: () => kinematicGraph,
  getKinematicJoints: () => kinematicJoints,
  isViewport3D: () => activeViewportView === '3d',
})
