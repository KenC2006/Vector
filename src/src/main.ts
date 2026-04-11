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
import { initUrdfAssembly, type UrdfAssemblyApi } from './urdfAssembly'
import { applyRichVisuals } from './richVisuals'
import { SAMPLE_URDF } from './sampleUrdf'
import { processXacro } from './xacro'
import { registerThemes, initSettings, VIEWPORT_BG, type ThemeId } from './settings'
import { initGitPanel } from './gitPanel'
import { initValidation, validateXMLStructure, validateURDFPerLink } from './validation'
import type { ValResult } from './validation'
import { parseURDFToScene, buildKinematicGraphFromURDF, setPathResolver, defaultMat } from './urdfParser'
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

/**
 * Raise the assembly root group so the lowest geometry point touches Y=0.
 * Call when loading / switching URDF documents or after reset — not on every edit reparse,
 * or the whole robot jumps whenever a new part dips below the floor plane.
 */
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
  urdfSnapshot?: string
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
  // Update hidden select for compatibility
  const select = document.getElementById('vc-chat-select') as HTMLSelectElement | null
  if (select) {
    select.innerHTML = ''
    for (let i = chatHistory.length - 1; i >= 0; i--) {
      const chat = chatHistory[i]
      const opt = document.createElement('option')
      opt.value = chat.id
      opt.textContent = chat.title || 'Untitled'
      if (chat.id === currentChatId) opt.selected = true
      select.appendChild(opt)
    }
  }

  // Update custom dropdown
  const label = document.getElementById('vc-chat-dropdown-label')
  const list = document.getElementById('vc-chat-dropdown-list')
  if (!label || !list) return

  const current = chatHistory.find(c => c.id === currentChatId)
  label.textContent = current?.title || 'New Chat'

  list.innerHTML = ''
  for (let i = chatHistory.length - 1; i >= 0; i--) {
    const chat = chatHistory[i]
    const item = document.createElement('div')
    item.className = 'vc-dd-item' + (chat.id === currentChatId ? ' active' : '')

    const lbl = document.createElement('span')
    lbl.className = 'vc-dd-item-label'
    lbl.textContent = chat.title || 'Untitled'

    const del = document.createElement('button')
    del.className = 'vc-dd-delete'
    del.title = 'Delete chat'
    del.innerHTML = '<svg viewBox="0 0 16 16" width="12" height="12" fill="none" stroke="currentColor" stroke-width="1.4"><path d="M3 4h10M5.5 4V3a1 1 0 011-1h3a1 1 0 011 1v1M6.5 7v4M9.5 7v4M4.5 4l.5 9a1 1 0 001 1h4a1 1 0 001-1l.5-9" stroke-linecap="round" stroke-linejoin="round"/></svg>'
    del.addEventListener('click', (e) => {
      e.stopPropagation()
      deleteChat(chat.id)
    })

    item.addEventListener('click', () => {
      loadChat(chat.id)
      updateChatDropdown()
      list.classList.add('hidden')
    })

    item.appendChild(lbl)
    item.appendChild(del)
    list.appendChild(item)
  }
}

function deleteChat(chatId: string) {
  const idx = chatHistory.findIndex(c => c.id === chatId)
  if (idx === -1) return
  chatHistory.splice(idx, 1)
  saveChatHistory()

  if (chatId === currentChatId) {
    // Deleted the active chat — switch to another or start fresh
    if (chatHistory.length > 0) {
      loadChat(chatHistory[chatHistory.length - 1].id)
    } else {
      startNewChat()
    }
  }
  updateChatDropdown()
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
  for (let i = 0; i < chat.messages.length; i++) {
    const msg = chat.messages[i]
    const el = document.createElement('div')
    el.className = `ai-msg ${msg.role}`
    el.innerHTML = `<div class="ai-msg-content">${msg.role === 'user' ? escapeHtml(msg.content) : msg.content}</div>`
    if (msg.role === 'user' || msg.role === 'assistant') {
      attachRewindButton(el, i)
    }
    vcMsgs.appendChild(el)
  }
  vcMsgs.scrollTop = vcMsgs.scrollHeight
  updateChatDropdown()
}

function recordChatMessage(role: 'user' | 'assistant' | 'system', content: string) {
  const urdfSnapshot = monacoEditor.getModel()?.getValue() || ''
  const msg: ChatMessage = { role, content, timestamp: Date.now(), urdfSnapshot }
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

// ── Chat rewind ──────────────────────────────────────────────────────────────

function rewindChatTo(msgIndex: number, mode: 'conversation' | 'code' | 'both') {
  const chat = getCurrentChat()
  if (!chat) return

  const targetMsg = currentChatMessages[msgIndex]
  if (!targetMsg) return

  if (mode === 'code' || mode === 'both') {
    if (targetMsg.urdfSnapshot) {
      monacoEditor.setValue(targetMsg.urdfSnapshot)
    }
  }

  if (mode === 'conversation' || mode === 'both') {
    // Keep messages up to and including the target index
    currentChatMessages.length = msgIndex + 1
    chat.messages = currentChatMessages
    chat.updatedAt = Date.now()
    saveChatHistory()
    loadChat(currentChatId)
  }
}

function attachRewindButton(msgEl: HTMLElement, msgIndex: number) {
  const wrap = document.createElement('div')
  wrap.className = 'chat-rewind-wrap'

  const btn = document.createElement('button')
  btn.className = 'chat-rewind-btn'
  btn.title = 'Rewind to here'
  btn.innerHTML = '<svg viewBox="0 0 16 16" width="14" height="14" fill="none" stroke="currentColor" stroke-width="2"><path d="M2 8a6 6 0 1 1 1.8 4.3" stroke-linecap="round"/><path d="M2 12V8h4" stroke-linecap="round" stroke-linejoin="round"/></svg>'

  const popover = document.createElement('div')
  popover.className = 'chat-rewind-popover hidden'
  popover.innerHTML = `
    <button class="crp-option" data-mode="conversation">Rewind conversation</button>
    <button class="crp-option" data-mode="code">Rewind code only</button>
    <button class="crp-option" data-mode="both">Rewind both</button>
  `

  btn.addEventListener('click', (e) => {
    e.stopPropagation()
    // Close any other open popovers
    document.querySelectorAll('.chat-rewind-popover').forEach(p => {
      if (p !== popover) p.classList.add('hidden')
    })
    popover.classList.toggle('hidden')
  })

  popover.querySelectorAll('.crp-option').forEach(opt => {
    opt.addEventListener('click', (e) => {
      e.stopPropagation()
      const mode = (opt as HTMLElement).dataset.mode as 'conversation' | 'code' | 'both'
      popover.classList.add('hidden')
      rewindChatTo(msgIndex, mode)
    })
  })

  wrap.appendChild(btn)
  wrap.appendChild(popover)
  // Attach inside the .ai-msg-content bubble
  const bubble = msgEl.querySelector('.ai-msg-content')
  if (bubble) {
    (bubble as HTMLElement).style.position = 'relative'
    bubble.appendChild(wrap)
  } else {
    msgEl.appendChild(wrap)
  }
}

// Close rewind popovers on outside click
document.addEventListener('click', () => {
  document.querySelectorAll('.chat-rewind-popover').forEach(p => p.classList.add('hidden'))
})

// Initialize: load most recent chat or create new one
if (chatHistory.length > 0) {
  const latest = chatHistory[chatHistory.length - 1]
  currentChatId = latest.id
  currentChatMessages = latest.messages
} else {
  startNewChat()
}

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
interface TabRobotCache {
  parsedRobot: ParsedRobot
  kinematicGraph: Record<string, KinematicLink>
  kinematicJoints: Record<string, KinematicJoint>
  parsedContent: string
}
const tabRobotCache: Record<string, TabRobotCache> = {}
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
      robot.remove(parsedRobot.group)
      wireframeGroup.clear()
      axisVisuals.length = 0
      parsedRobot = cached.parsedRobot
      kinematicGraph = cached.kinematicGraph
      kinematicJoints = cached.kinematicJoints
      robot.position.set(0, 0, 0)
      robot.add(parsedRobot.group)
      groundRobot(robot)
      rebuildWireframes()
      rebuildJointAxisVisuals()
      updateComMarker()
      rebuildCollisionVisuals(currentContent)
      updateViewportInfo()
      buildKinematicTreeUI()
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
    monacoModels[filename].onDidChangeContent(() => {
      if (activeFile === fn) {
        if (reparseTimeout !== null) clearTimeout(reparseTimeout)
        reparseTimeout = window.setTimeout(() => {
          reparseTimeout = null
          // Suppress auto-reparse while an AI inline diff is pending —
          // the explicit reparseURDF() in acceptInlineDiff/dismissInlineDiff handles it.
          if (pendingOldText !== null) return
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

// 3D vs AI chat tab (tabs wired later). Declared here so animate() can read it.
let activeViewportView: '3d' | 'chat' = '3d'

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
  // Strip any existing collision meshes from all link groups
  parsedRobot.group.traverse(obj => {
    if ((obj as any).userData?.isCollision) obj.parent?.remove(obj)
  })
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
        const euler = new THREE.Euler(rpy[0] || 0, rpy[1] || 0, rpy[2] || 0, 'ZYX')
        wrapper.quaternion.setFromEuler(euler)
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
  originAxes, grid, comGroup, wireframeGroup, axisVisuals, jointAxisState,
  simBar,
  simActive: () => simActive,
  parsedRobot: () => parsedRobot,
  showToast,
  onResize: (w, h) => { composer.setSize(w, h); composer.setPixelRatio(renderer.getPixelRatio()) },
})

const { resize, focusOnRobot, zoomCamera, setViewportCollapsed, setViewportFullscreen, setFocusMode, updateViewportInfo } = vpControls

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

  const navDt = Math.min(viewportNavClock.getDelta(), 0.05)

  // Build wireframes once
  if (!wireframeBuilt) {
    rebuildWireframes()
  }

  if (
    activeViewportView === '3d' &&
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

  controls.update()

  // CoM marker spin
  for (const marker of comGroup.children) {
    if (marker instanceof THREE.Mesh) {
      marker.rotation.y += 0.01
    }
  }

  // Three.js preview animation — only when MuJoCo physics core is NOT running.
  // When simCoreRunning, updateRobotFromSimState() drives joints from real physics.
  if (simRunning && !simCoreRunning) {
    simTime += 1 / 60
    updateSimUI()
    const t = simTime
    let i = 0
    for (const [jointName, jointInfo] of parsedRobot.joints) {
      const jType = jointInfo.type
      if (jType !== 'revolute' && jType !== 'continuous' && jType !== 'prismatic') { i++; continue }
      // Spread phase so joints don't all move in lockstep
      const phase = i * 1.3
      const quat = new THREE.Quaternion()
      const limits = simPreviewLimits.get(jointName)
      if (jType === 'continuous') {
        // Continuous joints (wheels etc.) — just spin
        quat.setFromAxisAngle(jointInfo.axis, t * 1.5 + phase)
      } else if (limits) {
        // Revolute/prismatic with known limits — sweep full range sinusoidally
        const mid = (limits.lower + limits.upper) / 2
        const amp = (limits.upper - limits.lower) / 2
        quat.setFromAxisAngle(jointInfo.axis, mid + Math.sin(t * 0.7 + phase) * amp)
      } else {
        // No limits found — gentle ±45° sweep
        quat.setFromAxisAngle(jointInfo.axis, Math.sin(t * 0.7 + phase) * (Math.PI / 4))
      }
      jointInfo.group.quaternion.copy(quat)
      i++
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
let urdfAssemblyApi: UrdfAssemblyApi | null = null

function rebuildJointAxisVisuals() {
  axisVisuals.length = 0
  let colorIdx = 0
  for (const [, jointInfo] of parsedRobot.joints) {
    const color = axisColors[colorIdx++ % axisColors.length]
    addJointAxis(jointInfo.group, jointInfo.axis, color)
  }
}

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
        try {
          const newParsed = parseURDFToScene(processed)
          const newKinematicData = buildKinematicGraphFromURDF(processed)
          robot.remove(parsedRobot.group)
          wireframeGroup.clear()
          axisVisuals.length = 0
          parsedRobot = newParsed
          kinematicGraph = newKinematicData.kinematicGraph
          kinematicJoints = newKinematicData.kinematicJoints
          parsedRobot.group.rotation.x = -Math.PI / 2
          robot.add(parsedRobot.group)
          applyRichVisuals(parsedRobot)
          addEdgeLines(parsedRobot)
          rebuildJointAxisVisuals()
          updateComMarker()
          rebuildWireframes()
          rebuildCollisionVisuals(processed)
          updateViewportInfo()
          buildKinematicTreeUI()
          urdfAssemblyApi?.onModelUpdated()
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

    rebuildJointAxisVisuals()

    // Update CoM marker
    updateComMarker()

    // Rebuild wireframes and collision visuals
    rebuildWireframes()
    rebuildCollisionVisuals(urdfContent)

    // Update viewport info
    updateViewportInfo()

    // Rebuild kinematic tree
    buildKinematicTreeUI()
    urdfAssemblyApi?.onModelUpdated()

    console.log(`[URDF] Reparsed: ${parsedRobot.linkCount} links, ${parsedRobot.jointCount} joints`)
  } catch (e) {
    console.error('[URDF] Parse error:', e)
    showToast(`URDF parse failed — 3D not updated: ${e instanceof Error ? e.message : String(e)}`, 'error')
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
        reparseTimeout = null
        if (pendingOldText !== null) return
        reparseURDF()
        urdfAssemblyApi?.onModelUpdated()
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

  // Don't trigger shortcuts when typing in inputs or Monaco editor
  const tag = (e.target as HTMLElement).tagName
  if (tag === 'INPUT' || tag === 'TEXTAREA' || tag === 'SELECT') return
  if ((e.target as HTMLElement).closest('.monaco-editor')) return

  // WASD: pan camera on the ground plane (orbit target moves with camera). Shift = faster.
  if (activeViewportView === '3d') {
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
  kinematic: document.getElementById('panel-kinematic')!,
  git: document.getElementById('panel-git')!,
  settings: document.getElementById('panel-settings')!,
}

function openSidebarPanel(panel: string) {
  if (
    (panel === 'build' || panel === 'toolbox' || panel === 'inspector' || panel === 'focus') &&
    activeViewportView !== '3d'
  ) {
    switchViewportView('3d')
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

// Wire custom chat dropdown toggle
const vcDropdownBtn = document.getElementById('vc-chat-dropdown-btn')
const vcDropdownList = document.getElementById('vc-chat-dropdown-list')
vcDropdownBtn?.addEventListener('click', (e) => {
  e.stopPropagation()
  vcDropdownList?.classList.toggle('hidden')
})
document.addEventListener('click', (e) => {
  if (vcDropdownList && !vcDropdownList.contains(e.target as Node) && e.target !== vcDropdownBtn) {
    vcDropdownList.classList.add('hidden')
  }
})

function switchViewportView(view: '3d' | 'chat') {
  activeViewportView = view
  if (view !== '3d') {
    keysViewportPan.w = keysViewportPan.a = keysViewportPan.s = keysViewportPan.d = false
  }
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

  if (role === 'user' || role === 'assistant') {
    attachRewindButton(msg, currentChatMessages.length - 1)
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
  if (rect.width > 0) {
    // Monaco is visible — anchor bar to top of editor panel
    bar.style.top = (rect.top + 8) + 'px'
    bar.style.right = (window.innerWidth - rect.right + 20) + 'px'
  } else {
    // Monaco is hidden (fullscreen/focus mode) — pin to top-right of viewport
    bar.style.top = '10px'
    bar.style.right = '10px'
  }
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
  pendingOldText = null  // clear before reparse so debounce guard is lifted

  // Cancel any debounce that was triggered by showInlineDiff's setValue call
  if (reparseTimeout !== null) { clearTimeout(reparseTimeout); reparseTimeout = null }

  showToast('Changes accepted', 'success')

  // Sync chat buttons to show "Applied"
  syncChatActions('accept')

  // Apply the accepted URDF to the 3D viewport
  if (editor) {
    reparseURDF()

    // Run local validation on the accepted changes (avoids blocking Mutex)
    runLocalValidation()
  }
}

function dismissInlineDiff() {
  const editor = (window as any).__vectorEditor as monaco.editor.IStandaloneCodeEditor | undefined
  const oldText = pendingOldText  // capture before clearing

  clearInlineDiff()
  pendingOldText = null  // clear before setValue so the debounce guard is lifted

  // Cancel any pending debounce before restoring (avoid a second reparse race)
  if (reparseTimeout !== null) { clearTimeout(reparseTimeout); reparseTimeout = null }

  if (editor && oldText !== null) {
    editor.setValue(oldText)
    reparseURDF()  // revert 3D immediately — no 500ms wait
  }

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

// Per-joint limits cached from URDF for the Three.js preview animation.
// Only populated for revolute/prismatic joints that have a <limit> element.
const simPreviewLimits = new Map<string, { lower: number; upper: number }>()

function refreshSimPreviewLimits() {
  simPreviewLimits.clear()
  const urdf = monacoEditor.getModel()?.getValue() ?? ''
  if (!urdf.trim()) return
  try {
    const doc = new DOMParser().parseFromString(urdf, 'application/xml')
    for (const joint of Array.from(doc.querySelectorAll('joint'))) {
      const name = joint.getAttribute('name')
      const type = joint.getAttribute('type')
      if (!name || (type !== 'revolute' && type !== 'prismatic')) continue
      const limitEl = joint.querySelector('limit')
      if (!limitEl) continue
      const lower = parseFloat(limitEl.getAttribute('lower') || '0')
      const upper = parseFloat(limitEl.getAttribute('upper') || '0')
      if (Number.isFinite(lower) && Number.isFinite(upper) && upper > lower) {
        simPreviewLimits.set(name, { lower, upper })
      }
    }
  } catch { /* ignore parse errors */ }
}

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

/** Path to last staging URDF written for sim_load (cleaned up on exit). */
let lastSimStagingPath: string | null = null

/** MuJoCo `get_state` returns `joint_states` array; UI expects `joints` map by name. */
function normalizeMuJoCoState(state: unknown): Record<string, unknown> {
  if (!state || typeof state !== 'object') return state as Record<string, unknown>
  const s = state as Record<string, unknown>
  if (s.joints && typeof s.joints === 'object') return s
  const jointStates = s.joint_states
  if (!Array.isArray(jointStates)) return s
  const joints: Record<string, { position: number; velocity: number }> = {}
  for (const j of jointStates) {
    if (j && typeof j === 'object' && typeof (j as { name?: string }).name === 'string') {
      const row = j as { name: string; position?: number; velocity?: number }
      joints[row.name] = {
        position: typeof row.position === 'number' ? row.position : 0,
        velocity: typeof row.velocity === 'number' ? row.velocity : 0,
      }
    }
  }
  return { ...s, joints }
}

function summarizeValidationErrors(results: ValResult[]): string {
  const errs = results.filter(r => r.severity === 'error')
  if (errs.length === 0) return 'URDF validation failed'
  return errs
    .slice(0, 6)
    .map(r => `${r.name}: ${r.message}`)
    .join('\n')
}

/** Throws if URDF must not be loaded into MuJoCo (XML or backend validation errors). */
async function assertUrdfReadyForSim(urdf: string): Promise<void> {
  const xmlErrors = validateXMLStructure(urdf)
  if (xmlErrors.length > 0) {
    throw new Error(summarizeValidationErrors(xmlErrors))
  }

  // Client-side per-link checks (no backend needed)
  const perLinkResults = validateURDFPerLink(urdf)
  const perLinkErrors = perLinkResults.filter(r => r.severity === 'error')
  const perLinkWarns  = perLinkResults.filter(r => r.severity === 'warn')
  if (perLinkErrors.length > 0) {
    throw new Error(summarizeValidationErrors(perLinkErrors))
  }
  if (perLinkWarns.length > 0) {
    showToast(`URDF has ${perLinkWarns.length} completeness warning(s) — check Validation panel.`, 'warning')
  }

  let result: { results?: ValResult[]; summary?: { error?: number; warn?: number } }
  try {
    result = await invoke('validate_urdf_content', { urdf_content: urdf }) as typeof result
  } catch (_e) {
    // Backend not available — per-link checks already ran above, allow sim to continue.
    return
  }
  const summary = result.summary
  const results = result.results ?? []
  if (summary && summary.error && summary.error > 0) {
    throw new Error(summarizeValidationErrors(results))
  }
  if (summary && summary.warn && summary.warn > 0) {
    showToast(`URDF has ${summary.warn} validation warning(s); continuing to simulation.`, 'warning')
  }
}

async function initializeSimulation() {
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
      throw coreErr
    }
  }

  const urdf = monacoEditor.getModel()?.getValue() ?? ''
  if (!urdf.trim()) {
    throw new Error('URDF editor is empty')
  }

  await assertUrdfReadyForSim(urdf)

  const neighborPath = (filePaths[activeFile] || currentFilePath || '').trim()
  const neighborUrdfPath =
    neighborPath && /\.urdf$/i.test(neighborPath) ? neighborPath : null

  if (lastSimStagingPath) {
    try {
      await invoke('remove_sim_staging_urdf', { path: lastSimStagingPath })
    } catch {
      /* ignore */
    }
    lastSimStagingPath = null
  }

  const simPath = await invoke<string>('write_sim_staging_urdf', {
    content: urdf,
    neighbor_urdf_path: neighborUrdfPath,
  })

  console.log('[Sim] Loading robot model from', simPath)
  try {
    await invoke('sim_load', { path: simPath })
  } catch (loadErr) {
    try {
      await invoke('remove_sim_staging_urdf', { path: simPath })
    } catch {
      /* ignore */
    }
    throw loadErr
  }
  lastSimStagingPath = simPath
    console.log('[Sim] Robot model loaded')

  simCoreRunning = true

    console.log('[Sim] Getting initial state...')
  const initialState = normalizeMuJoCoState(await invoke('sim_get_state'))
    console.log('[Sim] Initial state:', initialState)
  if (typeof initialState.time === 'number' && !Number.isNaN(initialState.time)) {
    simTime = initialState.time
  }
    simStateDisplay.style.display = 'block'
    updateSimStateDisplay(initialState)
  updateSimUI()
}

async function shutdownSimulation() {
  try {
    if (simStepIntervalId !== null) {
      clearInterval(simStepIntervalId)
      simStepIntervalId = null
    }
    if (lastSimStagingPath) {
      try {
        await invoke('remove_sim_staging_urdf', { path: lastSimStagingPath })
      } catch {
        /* ignore */
      }
      lastSimStagingPath = null
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
    const state = normalizeMuJoCoState(await invoke('sim_get_state'))
    if (typeof state.time === 'number' && !Number.isNaN(state.time)) {
      simTime = state.time
    }
    updateSimStateDisplay(state)
    updateRobotFromSimState(state)
    updateSimUI()
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
    // Cache joint limits from URDF for the Three.js preview animation
    refreshSimPreviewLimits()
    try {
      await initializeSimulation()
      showToast('Entered simulation mode (MuJoCo)', 'success')
    } catch (error) {
      console.error('[Sim] Failed to initialize:', error)
      simActive = false
      simCoreRunning = false
      simToggle.classList.remove('running')
      simBar.classList.add('hidden')
      simToggle.querySelector('span')!.textContent = 'Simulate'
      viewportLabel.textContent = '3D Preview'
      showToast(`Simulation: ${error instanceof Error ? error.message : String(error)}`, 'error')
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
    const state = normalizeMuJoCoState(await invoke('sim_get_state'))
    updateRobotFromSimState(state)
    simTime = typeof state.time === 'number' && !Number.isNaN(state.time) ? state.time : 0
    updateSimUI()
  } catch (error) {
    console.error('[Sim] Reset error:', error)
  }
})

let viewportInteractionMode: 'build' | 'inspect' = 'build'
let inspectFocusedLink: string | null = null

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
    if (monacoEditor.getModel()) {
      monacoEditor.setValue(content)
    } else {
      // Fallback safety: auto-create robot.urdf if somehow still no model
      createNewFile('robot.urdf', content, null)
    }
  },
  reparseUrdf: (xml?: string) => reparseURDF(xml),
  getParsedRobot: () => parsedRobot,
  getKinematicGraph: () => kinematicGraph,
  getKinematicJoints: () => kinematicJoints,
  isViewport3D: () => activeViewportView === '3d',
  getInteractionMode: () => viewportInteractionMode,
  onInspectLinkFocused: handleInspectLinkFocused,
  onAfterModelUpdated: refreshInspectAfterModelUpdate,
  zeroAssemblyWorldPosition: () => {
    robot.position.set(0, 0, 0)
  },
  groundAssembly: () => groundRobot(robot),
})
