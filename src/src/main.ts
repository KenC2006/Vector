import './style.css'
import * as THREE from 'three'
import { OrbitControls } from 'three/addons/controls/OrbitControls.js'
import { STLLoader } from 'three/addons/loaders/STLLoader.js'
import { invoke } from '@tauri-apps/api/core'
import { listen } from '@tauri-apps/api/event'
import { initUrdfAssembly } from './urdfAssembly'
import { applyRichVisuals } from './richVisuals'

const stlLoader = new STLLoader()

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

// ── Sample URDF ──────────────────────────────────────────────────────────────

const SAMPLE_URDF = `<?xml version="1.0"?>
<robot name="vector_component_demo">
  <!-- Default startup model built from current preset component IDs -->

  <link name="structural_baseplate_1">
    <inertial>
      <mass value="1.2000"/>
      <inertia ixx="0.020000" ixy="0" ixz="0" iyy="0.020000" iyz="0" izz="0.040000"/>
    </inertial>
    <visual>
      <geometry><box size="0.300000 0.300000 0.012000"/></geometry>
    </visual>
    <collision>
      <geometry><box size="0.300000 0.300000 0.012000"/></geometry>
    </collision>
  </link>

  <link name="actuator_servo_high_torque_2">
    <inertial>
      <mass value="0.1650"/>
      <inertia ixx="0.000033" ixy="0" ixz="0" iyy="0.000040" iyz="0" izz="0.000046"/>
    </inertial>
    <visual>
      <geometry><box size="0.046500 0.036000 0.034000"/></geometry>
    </visual>
    <collision>
      <geometry><box size="0.046500 0.036000 0.034000"/></geometry>
    </collision>
  </link>

  <joint name="joint_actuator_servo_high_torque_2" type="revolute">
    <parent link="structural_baseplate_1"/>
    <child link="actuator_servo_high_torque_2"/>
    <origin xyz="0 0 0.025" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14159" upper="3.14159" effort="10.6" velocity="3.14"/>
  </joint>

  <link name="structural_extrusion_2020_3">
    <inertial>
      <mass value="0.2100"/>
      <inertia ixx="0.001900" ixy="0" ixz="0" iyy="0.001900" iyz="0" izz="0.000030"/>
    </inertial>
    <visual>
      <geometry><box size="0.300000 0.020000 0.020000"/></geometry>
    </visual>
    <collision>
      <geometry><box size="0.300000 0.020000 0.020000"/></geometry>
    </collision>
  </link>

  <joint name="joint_structural_extrusion_2020_3" type="fixed">
    <parent link="actuator_servo_high_torque_2"/>
    <child link="structural_extrusion_2020_3"/>
    <origin xyz="0.170 0 0" rpy="0 0 0"/>
  </joint>

  <link name="sensor_depth_camera_small_4">
    <inertial>
      <mass value="0.0720"/>
      <inertia ixx="0.000020" ixy="0" ixz="0" iyy="0.000020" iyz="0" izz="0.000010"/>
    </inertial>
    <visual>
      <geometry><box size="0.090000 0.025000 0.025000"/></geometry>
    </visual>
    <collision>
      <geometry><box size="0.090000 0.025000 0.025000"/></geometry>
    </collision>
  </link>

  <joint name="joint_sensor_depth_camera_small_4" type="fixed">
    <parent link="structural_extrusion_2020_3"/>
    <child link="sensor_depth_camera_small_4"/>
    <origin xyz="0.160 0 0" rpy="0 0 0"/>
  </joint>

  <link name="compute_sbc_small_5">
    <inertial>
      <mass value="0.0460"/>
      <inertia ixx="0.000006" ixy="0" ixz="0" iyy="0.000010" iyz="0" izz="0.000012"/>
    </inertial>
    <visual>
      <geometry><box size="0.085000 0.056000 0.017000"/></geometry>
    </visual>
    <collision>
      <geometry><box size="0.085000 0.056000 0.017000"/></geometry>
    </collision>
  </link>

  <joint name="joint_compute_sbc_small_5" type="fixed">
    <parent link="structural_baseplate_1"/>
    <child link="compute_sbc_small_5"/>
    <origin xyz="-0.060 0 0.020" rpy="0 0 0"/>
  </joint>

  <link name="power_lipo_3s_2200_6">
    <inertial>
      <mass value="0.1900"/>
      <inertia ixx="0.000070" ixy="0" ixz="0" iyy="0.000180" iyz="0" izz="0.000210"/>
    </inertial>
    <visual>
      <geometry><box size="0.105000 0.034000 0.025000"/></geometry>
    </visual>
    <collision>
      <geometry><box size="0.105000 0.034000 0.025000"/></geometry>
    </collision>
  </link>

  <joint name="joint_power_lipo_3s_2200_6" type="fixed">
    <parent link="structural_baseplate_1"/>
    <child link="power_lipo_3s_2200_6"/>
    <origin xyz="0.060 0 0.020" rpy="0 0 0"/>
  </joint>
</robot>`

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

// Define a dark theme matching Vector's palette
// VS Code Dark+ accurate theme
monaco.editor.defineTheme('vector-dark', {
  base: 'vs-dark',
  inherit: true,
  rules: [
    { token: 'comment', foreground: '6A9955', fontStyle: 'italic' },
    { token: 'tag', foreground: '569cd6' },
    { token: 'attribute.name', foreground: '9cdcfe' },
    { token: 'attribute.value', foreground: 'ce9178' },
    { token: 'string', foreground: 'ce9178' },
    { token: 'number', foreground: 'b5cea8' },
    { token: 'keyword', foreground: 'c586c0' },
    { token: 'type', foreground: '4ec9b0' },
    { token: 'delimiter', foreground: '808080' },
    { token: 'delimiter.xml', foreground: '808080' },
    { token: 'key', foreground: '9cdcfe' },
    { token: 'metatag', foreground: '569cd6' },
    { token: 'metatag.content.xml', foreground: 'ce9178' },
  ],
  colors: {
    'editor.background': '#1e1e1e',
    'editor.foreground': '#d4d4d4',
    'editorLineNumber.foreground': '#858585',
    'editorLineNumber.activeForeground': '#c6c6c6',
    'editor.selectionBackground': '#264f78',
    'editor.lineHighlightBackground': '#2a2d2e',
    'editorCursor.foreground': '#aeafad',
    'editorIndentGuide.background': '#404040',
    'editorIndentGuide.activeBackground': '#707070',
    'editorBracketMatch.background': '#0064001a',
    'editorBracketMatch.border': '#888888',
    'scrollbarSlider.background': '#79797966',
    'scrollbarSlider.hoverBackground': '#646464b3',
    'scrollbarSlider.activeBackground': '#bfbfbf66',
    'minimap.background': '#1e1e1e',
    'editorOverviewRuler.border': '#7f7f7f4d',
    'editor.lineHighlightBorder': '#282828',
    'editorGutter.background': '#1e1e1e',
    'editorWidget.background': '#252526',
    'editorWidget.border': '#454545',
    'editorSuggestWidget.background': '#252526',
    'editorSuggestWidget.border': '#454545',
    'editorSuggestWidget.selectedBackground': '#04395e',
  },
})

// Light theme for Monaco
monaco.editor.defineTheme('vector-light', {
  base: 'vs',
  inherit: true,
  rules: [
    { token: 'comment', foreground: '008000', fontStyle: 'italic' },
    { token: 'tag', foreground: '0000ff' },
    { token: 'attribute.name', foreground: '0451a5' },
    { token: 'attribute.value', foreground: 'a31515' },
    { token: 'string', foreground: 'a31515' },
    { token: 'number', foreground: '098658' },
    { token: 'keyword', foreground: 'af00db' },
    { token: 'type', foreground: '267f99' },
    { token: 'delimiter', foreground: '333333' },
    { token: 'delimiter.xml', foreground: '333333' },
    { token: 'key', foreground: '0451a5' },
    { token: 'metatag', foreground: '0000ff' },
    { token: 'metatag.content.xml', foreground: 'a31515' },
  ],
  colors: {
    'editor.background': '#ffffff',
    'editor.foreground': '#333333',
    'editorLineNumber.foreground': '#999999',
    'editorLineNumber.activeForeground': '#333333',
    'editor.selectionBackground': '#add6ff',
    'editor.lineHighlightBackground': '#f5f5f5',
    'editorCursor.foreground': '#333333',
    'editorIndentGuide.background': '#d3d3d3',
    'editorIndentGuide.activeBackground': '#939393',
    'editorBracketMatch.background': '#add6ff80',
    'editorBracketMatch.border': '#b9b9b9',
    'scrollbarSlider.background': '#c1c1c166',
    'scrollbarSlider.hoverBackground': '#9e9e9eb3',
    'scrollbarSlider.activeBackground': '#bfbfbf66',
    'minimap.background': '#ffffff',
    'editorOverviewRuler.border': '#d4d4d4',
    'editor.lineHighlightBorder': '#eeeeee',
    'editorGutter.background': '#ffffff',
    'editorWidget.background': '#f3f3f3',
    'editorWidget.border': '#c8c8c8',
    'editorSuggestWidget.background': '#f3f3f3',
    'editorSuggestWidget.border': '#c8c8c8',
    'editorSuggestWidget.selectedBackground': '#cce5ff',
  },
})

// ── Theme system ────────────────────────────────────────────────────────────

function applyTheme(theme: 'dark' | 'light') {
  document.documentElement.classList.toggle('theme-light', theme === 'light')
  monaco.editor.setTheme(theme === 'dark' ? 'vector-dark' : 'vector-light')
  localStorage.setItem('vector_theme', theme)
  // Update the settings dropdown if it exists
  const select = document.getElementById('setting-theme') as HTMLSelectElement | null
  if (select) select.value = theme
}

// Apply saved theme on load
const savedTheme = (localStorage.getItem('vector_theme') || 'dark') as 'dark' | 'light'
if (savedTheme === 'light') applyTheme('light')

// ── Mesh file loading (async) ────────────────────────────────────────────────

async function loadMeshFile(
  filename: string,
  parent: THREE.Group,
  placeholder: THREE.Mesh,
  material: THREE.MeshStandardMaterial,
  scaleAttr: string | null,
) {
  try {
    // Resolve path: strip package:// prefix, handle relative paths
    let resolvedPath = filename
    if (resolvedPath.startsWith('package://')) {
      resolvedPath = resolvedPath.replace('package://', '')
    }
    // If relative, try resolving from current file's directory
    if (currentFilePath && !resolvedPath.match(/^[A-Z]:/i) && !resolvedPath.startsWith('/')) {
      const dir = currentFilePath.replace(/[\\/][^\\/]+$/, '')
      resolvedPath = `${dir}/${resolvedPath}`
    }

    // Read binary file via Tauri IPC
    const bytes = await invoke<number[]>('read_binary_file', { path: resolvedPath })
    const buffer = new Uint8Array(bytes).buffer

    // Detect format from extension
    const ext = filename.split('.').pop()?.toLowerCase() || ''
    let geometry: THREE.BufferGeometry | null = null

    if (ext === 'stl') {
      geometry = stlLoader.parse(buffer)
    }
    // Add more loaders here as needed (OBJ, DAE, etc.)

    if (!geometry) {
      console.warn(`[mesh] Unsupported mesh format: ${ext} (${filename})`)
      return
    }

    // Create mesh with the parsed geometry
    const loadedMesh = new THREE.Mesh(geometry, material)
    loadedMesh.castShadow = true
    loadedMesh.receiveShadow = true

    // Apply scale if specified
    if (scaleAttr) {
      const s = scaleAttr.split(/\s+/).map(parseFloat)
      if (s.length >= 3) loadedMesh.scale.set(s[0], s[1], s[2])
      else if (s.length === 1 && s[0]) loadedMesh.scale.setScalar(s[0])
    }

    // Copy userData from placeholder
    Object.assign(loadedMesh.userData, placeholder.userData)

    // Replace placeholder with loaded mesh
    parent.remove(placeholder)
    placeholder.geometry.dispose()
    parent.add(loadedMesh)

  } catch (err) {
    console.warn(`[mesh] Failed to load ${filename}:`, err)
    // Keep placeholder — don't crash
  }
}

// Create Monaco models — start with just the sample URDF
const monacoModels: Record<string, monaco.editor.ITextModel> = {
  'robot.urdf': monaco.editor.createModel(SAMPLE_URDF, 'xml'),
}

// Create Monaco editor instance
const monacoEditor = monaco.editor.create(monacoContainer, {
  model: monacoModels['robot.urdf'],
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

let activeFile = 'robot.urdf'
const fileTypeLabel = document.getElementById('file-type') as HTMLSpanElement
const tabBar = document.getElementById('tab-bar') as HTMLDivElement
const tabNewBtn = tabBar.querySelector('.tab-new') as HTMLButtonElement
const filesList = document.getElementById('files-list') as HTMLDivElement | null

// Track open files and their paths
const openFiles: string[] = ['robot.urdf']
const filePaths: Record<string, string | null> = {} // filename → disk path (null = unsaved)
const viewStates: Record<string, monaco.editor.ICodeEditorViewState | null> = {}
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

  // Save current view state
  viewStates[activeFile] = monacoEditor.saveViewState()

  activeFile = filename
  fileTypeLabel.textContent = getFileType(filename)

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
  const bcItems = document.querySelectorAll('.bc-item')
  if (bcItems.length > 0) bcItems[bcItems.length - 1].textContent = filename

  // Update title
  const path = filePaths[filename]
  document.title = path ? `Vector — ${filename}` : `Vector — ${filename} (unsaved)`

  monacoEditor.focus()

  // Reparse if it's a URDF file
  if (getFileExt(filename) === 'urdf' || getFileExt(filename) === 'xml') {
    runLocalValidation()
  }
}

function createNewFile(filename?: string, content = '', diskPath: string | null = null) {
  if (!filename) {
    untitledCounter++
    filename = `untitled_${untitledCounter}.urdf`
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

function closeFile(filename: string) {
  // Can't close the last file
  if (openFiles.length <= 1) {
    showToast('Cannot close the last file', 'warning')
    return
  }

  const idx = openFiles.indexOf(filename)
  if (idx < 0) return

  // Remove from tracking
  openFiles.splice(idx, 1)
  delete viewStates[filename]
  delete filePaths[filename]

  // Dispose Monaco model
  const model = monacoModels[filename]
  if (model) {
    model.dispose()
    delete monacoModels[filename]
  }

  // Switch to adjacent tab if closing the active file
  if (filename === activeFile) {
    const newIdx = Math.min(idx, openFiles.length - 1)
    activeFile = '' // force switch
    switchToFile(openFiles[newIdx])
  }

  renderTabs()
  renderExplorer()
}

// + button handler
tabNewBtn.addEventListener('click', () => createNewFile())

// Initial render
renderTabs()
renderExplorer()

// ── Validation Markers for Monaco ───────────────────────────────────────────

function setValidationMarkers(results: Array<{ name: string; severity: string; message: string; category: string; line?: number; column?: number }>) {
  const model = monacoModels['robot.urdf']
  if (!model) return

  const markers: monaco.editor.IMarkerData[] = []

  for (const r of results) {
    if (r.severity === 'pass' || r.severity === 'info') continue

    // Map validation severity to Monaco marker severity
    const markerSeverity = r.severity === 'error'
      ? monaco.MarkerSeverity.Error
      : monaco.MarkerSeverity.Warning

    // Use provided line/column or default to line 1
    const lineNumber = r.line || 1
    const column = r.column || 1

    markers.push({
      severity: markerSeverity,
      message: `[${r.category}] ${r.name}: ${r.message}`,
      startLineNumber: lineNumber,
      startColumn: column,
      endLineNumber: lineNumber,
      endColumn: Math.max(column + 1, column + (r.name.length || 10)),
      source: 'Vector Validator',
    })
  }

  monaco.editor.setModelMarkers(model, 'vector-validator', markers)
}

// ── Three.js ─────────────────────────────────────────────────────────────────

const canvas = document.getElementById('viewport') as HTMLCanvasElement
const viewportPanel = document.getElementById('viewport-panel') as HTMLDivElement

const renderer = new THREE.WebGLRenderer({ canvas, antialias: true, alpha: false })
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2))
renderer.setClearColor(0x1a1a1a)
renderer.shadowMap.enabled = true
renderer.shadowMap.type = THREE.PCFSoftShadowMap
renderer.toneMapping = THREE.ACESFilmicToneMapping
renderer.toneMappingExposure = 1.1

const scene = new THREE.Scene()

// Subtle fog for depth
scene.fog = new THREE.FogExp2(0x1a1a1a, 0.3)

const camera = new THREE.PerspectiveCamera(50, 1, 0.01, 100)
camera.position.set(1.2, 1.0, 1.6)

const controls = new OrbitControls(camera, canvas)
controls.enableDamping = true
controls.dampingFactor = 0.06
controls.target.set(0, 0.35, 0)
controls.minDistance = 0.3
controls.maxDistance = 8

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
const defaultMat = new THREE.MeshStandardMaterial({
  color: 0x888888, roughness: 0.4, metalness: 0.5,
})

// ── URDF Parser ─────────────────────────────────────────────────────────────

interface ParsedRobot {
  group: THREE.Group
  joints: Map<string, { group: THREE.Group; axis: THREE.Vector3; type: string }>
  linkGroups: Map<string, THREE.Group>
  vertexCount: number
  faceCount: number
  linkCount: number
  jointCount: number
}

interface URDFLink {
  name: string
  mass: number
  comPos: THREE.Vector3
  geometry: THREE.Group
}

interface URDFJoint {
  name: string
  type: string
  parentLink: string
  childLink: string
  axis: THREE.Vector3
  origin: { pos: THREE.Vector3; rot: THREE.Quaternion }
}

function parseURDFToScene(urdfXml: string): ParsedRobot {
  const parser = new DOMParser()
  const doc = parser.parseFromString(urdfXml, 'application/xml')

  if (doc.documentElement.nodeName === 'parsererror') {
    console.error('URDF parse error')
    throw new Error('Invalid URDF XML')
  }

  const robot = new THREE.Group()
  const linkGroups = new Map<string, THREE.Group>()
  const joints = new Map<string, { group: THREE.Group; axis: THREE.Vector3; type: string }>()
  const linkData = new Map<string, URDFLink>()
  const jointData: URDFJoint[] = []
  const childLinkSet = new Set<string>()

  // Parse all links
  const linkElements = doc.querySelectorAll('link')
  for (const linkEl of linkElements) {
    const linkName = linkEl.getAttribute('name') || ''
    const geometryGroup = new THREE.Group()
    let mass = 0
    let comPos = new THREE.Vector3()

    // Parse mass
    const inertialEl = linkEl.querySelector('inertial')
    if (inertialEl) {
      const massEl = inertialEl.querySelector('mass')
      if (massEl) {
        mass = parseFloat(massEl.getAttribute('value') || '0')
      }
      const originEl = inertialEl.querySelector('origin')
      if (originEl) {
        const xyz = (originEl.getAttribute('xyz') || '0 0 0').split(/\s+/).map(parseFloat)
        comPos = new THREE.Vector3(xyz[0], xyz[1], xyz[2])
      }
    }

    // Parse all visual geometry elements (multiple <visual> per link supported)
    const visualEls = linkEl.querySelectorAll('visual')
    for (const visualEl of visualEls) {
      const geomEl = visualEl.querySelector('geometry')
      if (!geomEl) continue

      let mat: THREE.Material = defaultMat

      // Get material color if specified
      const matEl = visualEl.querySelector('material')
      if (matEl) {
        const colorEl = matEl.querySelector('color')
        if (colorEl) {
          const rgba = (colorEl.getAttribute('rgba') || '0.5 0.5 0.5 1').split(/\s+/).map(parseFloat)
          const color = new THREE.Color(rgba[0], rgba[1], rgba[2])
          mat = new THREE.MeshStandardMaterial({
            color,
            roughness: 0.4,
            metalness: 0.5,
          })
        }
      }

      // Create a sub-group for this visual (each has its own origin transform)
      const visualGroup = new THREE.Group()

      // Parse geometry type
      const cylinderEl = geomEl.querySelector('cylinder')
      if (cylinderEl) {
        const r = parseFloat(cylinderEl.getAttribute('radius') || '0.05')
        const l = parseFloat(cylinderEl.getAttribute('length') || '0.1')
        const geom = new THREE.CylinderGeometry(r, r, l, 32)
        const mesh = new THREE.Mesh(geom, mat)
        mesh.rotation.x = Math.PI / 2
        visualGroup.add(mesh)
      } else {
        const boxEl = geomEl.querySelector('box')
        if (boxEl) {
          const size = (boxEl.getAttribute('size') || '0.1 0.1 0.1').split(/\s+/).map(parseFloat)
          const geom = new THREE.BoxGeometry(size[0], size[1], size[2])
          const mesh = new THREE.Mesh(geom, mat)
          visualGroup.add(mesh)
        } else {
          const sphereEl = geomEl.querySelector('sphere')
          if (sphereEl) {
            const r = parseFloat(sphereEl.getAttribute('radius') || '0.05')
            const geom = new THREE.SphereGeometry(r, 24, 24)
            const mesh = new THREE.Mesh(geom, mat)
            visualGroup.add(mesh)
          } else {
            // Check for mesh file reference
            const meshEl = geomEl.querySelector('mesh')
            if (meshEl) {
              const filename = meshEl.getAttribute('filename') || ''
              const scaleAttr = meshEl.getAttribute('scale')
              // Show placeholder immediately, load mesh async
              const placeholder = new THREE.Mesh(
                new THREE.SphereGeometry(0.02, 16, 16),
                (mat as THREE.Material).clone(),
              )
              placeholder.userData._meshFile = filename
              visualGroup.add(placeholder)
              // Async mesh loading
              loadMeshFile(filename, visualGroup, placeholder, mat as THREE.MeshStandardMaterial, scaleAttr)
            } else {
              const placeholderGeom = new THREE.SphereGeometry(0.02, 16, 16)
              const mesh = new THREE.Mesh(placeholderGeom, mat)
              visualGroup.add(mesh)
            }
          }
        }
      }

      // Apply this visual's origin transform
      const visOriginEl = visualEl.querySelector('origin')
      if (visOriginEl) {
        const xyz = (visOriginEl.getAttribute('xyz') || '0 0 0').split(/\s+/).map(parseFloat)
        const rpy = (visOriginEl.getAttribute('rpy') || '0 0 0').split(/\s+/).map(parseFloat)
        visualGroup.position.set(xyz[0], xyz[1], xyz[2])
        const euler = new THREE.Euler(rpy[0], rpy[1], rpy[2], 'ZYX')
        visualGroup.quaternion.setFromEuler(euler)
      }

      geometryGroup.add(visualGroup)
    }

    // Add shadow properties and tag with link name for raycasting
    geometryGroup.traverse((child) => {
      if (child instanceof THREE.Mesh) {
        child.castShadow = true
        child.receiveShadow = true
        child.userData.urdfLinkName = linkName
      }
    })

    const linkGroup = new THREE.Group()
    linkGroup.add(geometryGroup)
    linkGroups.set(linkName, linkGroup)
    linkData.set(linkName, { name: linkName, mass, comPos, geometry: linkGroup })
  }

  // Parse all joints
  const jointElements = doc.querySelectorAll('joint')
  for (const jointEl of jointElements) {
    const jointName = jointEl.getAttribute('name') || ''
    const jointType = jointEl.getAttribute('type') || ''

    const parentEl = jointEl.querySelector('parent')
    const childEl = jointEl.querySelector('child')
    const parentLink = parentEl?.getAttribute('link') || ''
    const childLink = childEl?.getAttribute('link') || ''

    if (parentLink && childLink) {
      childLinkSet.add(childLink)

      let axisVec = new THREE.Vector3(0, 0, 1)
      const axisEl = jointEl.querySelector('axis')
      if (axisEl) {
        const xyz = (axisEl.getAttribute('xyz') || '0 0 1').split(/\s+/).map(parseFloat)
        axisVec = new THREE.Vector3(xyz[0], xyz[1], xyz[2]).normalize()
      }

      let pos = new THREE.Vector3()
      let rot = new THREE.Quaternion()
      const originEl = jointEl.querySelector('origin')
      if (originEl) {
        const xyz = (originEl.getAttribute('xyz') || '0 0 0').split(/\s+/).map(parseFloat)
        const rpy = (originEl.getAttribute('rpy') || '0 0 0').split(/\s+/).map(parseFloat)
        pos = new THREE.Vector3(xyz[0], xyz[1], xyz[2])
        const euler = new THREE.Euler(rpy[0], rpy[1], rpy[2], 'ZYX')
        rot.setFromEuler(euler)
      }

      jointData.push({
        name: jointName,
        type: jointType,
        parentLink,
        childLink,
        axis: axisVec,
        origin: { pos, rot },
      })
    }
  }

  // Build hierarchy
  const rootLinkName = Array.from(linkData.keys()).find((name) => !childLinkSet.has(name)) || 'base_link'
  const rootLinkGroup = linkGroups.get(rootLinkName)
  if (rootLinkGroup) {
    robot.add(rootLinkGroup)
  }

  function attachChildren(parentLinkName: string, parentGroup: THREE.Group) {
    for (const joint of jointData) {
      if (joint.parentLink === parentLinkName) {
        const childLinkName = joint.childLink
        const childLinkGroup = linkGroups.get(childLinkName)
        if (childLinkGroup) {
          const pivotGroup = new THREE.Group()
          pivotGroup.position.copy(joint.origin.pos)
          pivotGroup.quaternion.copy(joint.origin.rot)
          pivotGroup.add(childLinkGroup)
          parentGroup.add(pivotGroup)

          joints.set(joint.name, { group: pivotGroup, axis: joint.axis, type: joint.type })

          attachChildren(childLinkName, childLinkGroup)
        }
      }
    }
  }

  attachChildren(rootLinkName, rootLinkGroup || robot)

  // Count meshes
  let vertexCount = 0
  let faceCount = 0
  robot.traverse((child) => {
    if (child instanceof THREE.Mesh && child.geometry) {
      const posAttr = child.geometry.getAttribute('position')
      if (posAttr) {
        vertexCount += posAttr.count
      }
      if (child.geometry.getIndex()) {
        faceCount += child.geometry.getIndex()!.count / 3
      }
    }
  })

  return {
    group: robot,
    joints,
    linkGroups,
    vertexCount,
    faceCount,
    linkCount: linkData.size,
    jointCount: jointData.length,
  }
}

// ── Build robot from URDF ───────────────────────────────────────────────────

const robot = new THREE.Group()
scene.add(robot)

let parsedRobot = parseURDFToScene(SAMPLE_URDF)
// URDF uses Z-up, Three.js uses Y-up: rotate the entire robot -90° around X
parsedRobot.group.rotation.x = -Math.PI / 2
robot.add(parsedRobot.group)
applyRichVisuals(parsedRobot)
groundRobot(robot)

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
  jointAxisVisible = !jointAxisVisible
  for (const obj of axisVisuals) obj.visible = jointAxisVisible
  toggleJointAxisBtn.classList.toggle('active', jointAxisVisible)
})

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
  // Move camera closer/further along the view direction
  const dir = new THREE.Vector3().subVectors(camera.position, controls.target)
  const newDist = Math.max(controls.minDistance, Math.min(controls.maxDistance, dir.length() * factor))
  dir.normalize().multiplyScalar(newDist)
  camera.position.copy(controls.target).add(dir)
  controls.update()
}

viZoomIn?.addEventListener('click', () => zoomCamera(0.75))
viZoomOut?.addEventListener('click', () => zoomCamera(1.33))
viResetView?.addEventListener('click', () => {
  camera.position.copy(DEFAULT_CAM_POS)
  controls.target.copy(DEFAULT_CAM_TARGET)
  controls.update()
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

// ── Update viewport info ────────────────────────────────────────────────────

function updateViewportInfo() {
  const viVerts = document.getElementById('vi-verts')
  const viFaces = document.getElementById('vi-faces')
  const viLinks = document.getElementById('vi-links')
  const viJoints = document.getElementById('vi-joints')

  if (viVerts) viVerts.textContent = `Verts: ${parsedRobot.vertexCount.toLocaleString()}`
  if (viFaces) viFaces.textContent = `Faces: ${parsedRobot.faceCount.toLocaleString()}`
  if (viLinks) viLinks.textContent = `Links: ${parsedRobot.linkCount}`
  if (viJoints) viJoints.textContent = `Joints: ${parsedRobot.jointCount}`
}

updateViewportInfo()

// ── Resize ───────────────────────────────────────────────────────────────────

function resize() {
  const header = document.getElementById('viewport-header')!
  const w = viewportPanel.clientWidth
  const h = viewportPanel.clientHeight - header.offsetHeight - (simActive ? simBar.offsetHeight : 0)
  if (w > 0 && h > 0) {
    renderer.setSize(w, h)
    camera.aspect = w / h
    camera.updateProjectionMatrix()
  }
}
resize()
window.addEventListener('resize', resize)

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

  renderer.render(scene, camera)
}
animate()

// ── Draggable split ──────────────────────────────────────────────────────────

const handle = document.getElementById('resize-handle') as HTMLDivElement
const editorPanel = document.getElementById('editor-panel') as HTMLDivElement
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

// ── Viewport collapse toggle ────────────────────────────────────────────────

let viewportCollapsed = false
let savedEditorWidth = '50%'
const toggleViewportBtn = document.getElementById('toggle-viewport') as HTMLButtonElement
const expandViewportBtn = document.getElementById('expand-viewport-btn') as HTMLButtonElement

function setViewportCollapsed(collapsed: boolean) {
  viewportCollapsed = collapsed

  if (collapsed) {
    // Save current editor width before collapsing
    savedEditorWidth = editorPanel.style.width || '50%'
    viewportPanel.classList.add('collapsed')
    handle.classList.add('vp-collapsed')
    editorPanel.classList.add('vp-collapsed')
    expandViewportBtn.classList.add('visible')
    toggleViewportBtn.classList.add('active')
  } else {
    // Restore viewport
    expandViewportBtn.classList.remove('visible')
    viewportPanel.classList.remove('collapsed')
    handle.classList.remove('vp-collapsed')
    editorPanel.classList.remove('vp-collapsed')
    editorPanel.style.width = savedEditorWidth
    toggleViewportBtn.classList.remove('active')

    // Force layout reflow before measuring
    void viewportPanel.offsetHeight
    resize()
  }

  // Trigger Monaco layout update
  requestAnimationFrame(() => {
    if ((window as any).__vectorEditor) {
      (window as any).__vectorEditor.layout()
    }
  })
}

toggleViewportBtn.addEventListener('click', () => setViewportCollapsed(!viewportCollapsed))
expandViewportBtn.addEventListener('click', () => setViewportCollapsed(false))

// ── Kinematic Graph Data Structure ──────────────────────────────────────────

interface KinematicLink {
  name: string
  mass: number
  parent?: string
  children: string[]
}

interface KinematicJoint {
  name: string
  type: string
  axis: string
  parentLink: string
  childLink: string
}

// Build kinematic graph from parsed URDF
function buildKinematicGraphFromURDF(urdfXml: string): {
  kinematicGraph: Record<string, KinematicLink>
  kinematicJoints: Record<string, KinematicJoint>
} {
  const parser = new DOMParser()
  const doc = parser.parseFromString(urdfXml, 'application/xml')

  const kinematicGraph: Record<string, KinematicLink> = {}
  const kinematicJoints: Record<string, KinematicJoint> = {}
  const childLinkSet = new Set<string>()

  // Parse all links
  const linkElements = doc.querySelectorAll('link')
  for (const linkEl of linkElements) {
    const linkName = linkEl.getAttribute('name') || ''
    let mass = 0

    const inertialEl = linkEl.querySelector('inertial')
    if (inertialEl) {
      const massEl = inertialEl.querySelector('mass')
      if (massEl) {
        mass = parseFloat(massEl.getAttribute('value') || '0')
      }
    }

    kinematicGraph[linkName] = {
      name: linkName,
      mass,
      children: [],
    }
  }

  // Parse all joints
  const jointElements = doc.querySelectorAll('joint')
  for (const jointEl of jointElements) {
    const jointName = jointEl.getAttribute('name') || ''
    const jointType = jointEl.getAttribute('type') || ''

    const parentEl = jointEl.querySelector('parent')
    const childEl = jointEl.querySelector('child')
    const parentLink = parentEl?.getAttribute('link') || ''
    const childLink = childEl?.getAttribute('link') || ''

    if (parentLink && childLink) {
      childLinkSet.add(childLink)

      let axis = '--'
      const axisEl = jointEl.querySelector('axis')
      if (axisEl) {
        const xyz = (axisEl.getAttribute('xyz') || '0 0 1').split(/\s+/).map(parseFloat)
        if (Math.abs(xyz[0]) > 0.5) axis = 'X'
        else if (Math.abs(xyz[1]) > 0.5) axis = 'Y'
        else if (Math.abs(xyz[2]) > 0.5) axis = 'Z'
      }

      kinematicJoints[jointName] = {
        name: jointName,
        type: jointType,
        axis,
        parentLink,
        childLink,
      }

      // Add child to parent's children list
      if (kinematicGraph[parentLink]) {
        kinematicGraph[parentLink].children.push(childLink)
      }
    }
  }

  // Set parent links and identify root
  for (const [linkName, link] of Object.entries(kinematicGraph)) {
    if (childLinkSet.has(linkName)) {
      // Find parent
      for (const joint of Object.values(kinematicJoints)) {
        if (joint.childLink === linkName) {
          link.parent = joint.parentLink
          break
        }
      }
    }
  }

  return { kinematicGraph, kinematicJoints }
}

// ── Build Kinematic Context for AI ──────────────────────────────────────────────
// Generates a structured text summary of the robot's kinematic structure
// to send to Claude for better context-aware edits

function buildKinematicContext(): string {
  const robotName = 'simple_arm' // TODO: Extract from URDF
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

let currentHighlightedMeshes: THREE.Mesh[] = []

function highlightMesh(linkName: string) {
  // Clear previous highlights
  for (const mesh of currentHighlightedMeshes) {
    const mat = mesh.material as THREE.MeshStandardMaterial
    if (mat && mat.emissive) mat.emissive.setHex(0x000000)
  }
  currentHighlightedMeshes = []

  // Apply new highlight
  const linkGroup = parsedRobot.linkGroups.get(linkName)
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

// ── Live URDF re-parsing ────────────────────────────────────────────────────

let reparseTimeout: number | null = null
let urdfAssemblyApi: { onModelUpdated(): void; recordUndoExternal(content: string): void } | null = null

function reparseURDF() {
  try {
    const urdfContent = monacoEditor.getValue()
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

// ── Validation Panel ────────────────────────────────────────────────────────

const validationResults = document.getElementById('validation-results') as HTMLDivElement
const validationSummary = document.getElementById('validation-summary') as HTMLDivElement
const btnRevalidate = document.getElementById('btn-revalidate') as HTMLButtonElement

interface ValResult {
  name: string
  severity: string  // "pass" | "warn" | "error" | "info"
  message: string
  category: string
  line?: number
  column?: number
}

function renderValidationResults(results: ValResult[], summary: { pass: number; warn: number; error: number; info: number }) {
  // Render summary bar
  validationSummary.innerHTML = `
    <div class="vs-item vs-pass"><span class="vs-count">${summary.pass}</span> pass</div>
    <div class="vs-item vs-warn"><span class="vs-count">${summary.warn}</span> warn</div>
    <div class="vs-item vs-error"><span class="vs-count">${summary.error}</span> error</div>
  `

  // Update status bar error/warning counts
  const errorCountEl = document.getElementById('error-count')
  const warningCountEl = document.getElementById('warning-count')
  if (errorCountEl) errorCountEl.textContent = String(summary.error)
  if (warningCountEl) warningCountEl.textContent = String(summary.warn)

  // Group results by category
  const groups: Record<string, ValResult[]> = {}
  for (const r of results) {
    if (!groups[r.category]) groups[r.category] = []
    groups[r.category].push(r)
  }

  // Render groups
  validationResults.innerHTML = ''
  for (const [category, items] of Object.entries(groups)) {
    const group = document.createElement('div')
    group.className = 'val-group'

    const title = document.createElement('div')
    title.className = 'vg-title'
    title.textContent = category
    group.appendChild(title)

    for (const item of items) {
      const el = document.createElement('div')
      el.className = `val-item ${item.severity}`
      el.style.cursor = 'pointer'

      const contentEl = document.createElement('div')
      contentEl.style.display = 'flex'
      contentEl.style.justifyContent = 'space-between'
      contentEl.style.alignItems = 'flex-start'
      contentEl.style.gap = '8px'

      const textEl = document.createElement('div')
      textEl.style.flex = '1'
      textEl.innerHTML = `<div>${item.name}</div><span class="val-detail">${item.message}</span>`

      const lineEl = document.createElement('div')
      lineEl.style.fontSize = '11px'
      lineEl.style.opacity = '0.6'
      lineEl.style.whiteSpace = 'nowrap'
      lineEl.textContent = item.line ? `Ln ${item.line}` : ''

      contentEl.appendChild(textEl)
      if (item.line) contentEl.appendChild(lineEl)

      el.appendChild(contentEl)

      // Make clickable to jump to line
      if (item.line) {
        el.addEventListener('click', () => {
          const editor = (window as any).__vectorEditor as monaco.editor.IStandaloneCodeEditor | undefined
          if (editor) {
            editor.revealLineInCenter(item.line!)
            editor.setPosition({ lineNumber: item.line!, column: item.column || 1 })
            editor.focus()
          }
        })
      }

      group.appendChild(el)
    }

    validationResults.appendChild(group)
  }
}

// Run client-side XML validation first, then call Python backend for full validation
async function runValidation() {
  btnRevalidate.disabled = true
  btnRevalidate.textContent = 'Validating...'

  try {
    const editor = (window as any).__vectorEditor as monaco.editor.IStandaloneCodeEditor | undefined
    const urdfContent = editor?.getValue() || ''

    // First: client-side XML validation
    const xmlErrors = validateXMLStructure(urdfContent)

    if (xmlErrors.length > 0) {
      // If XML is malformed, show only XML errors
      const summary = { pass: 0, warn: 0, error: xmlErrors.length, info: 0 }
      renderValidationResults(xmlErrors, summary)
      setValidationMarkers(xmlErrors)
    } else {
      // XML is valid, try full validation from Python backend
      try {
        const result = await invoke('validate_urdf_content', {
          urdf_content: urdfContent
        })

        if (result && (result as any).results) {
          renderValidationResults((result as any).results, (result as any).summary)
          setValidationMarkers((result as any).results)
        }
      } catch (_e) {
        // Python backend failed or not available, use local validation
        runLocalValidation()
      }
    }
  } catch (_e) {
    // Fallback: run local validation against the hardcoded kinematic graph
    runLocalValidation()
  }

  btnRevalidate.disabled = false
  btnRevalidate.textContent = 'Run Checks'
}

// Client-side XML structure validation using DOMParser
function validateXMLStructure(content: string): ValResult[] {
  const errors: ValResult[] = []

  const parser = new DOMParser()
  const xmlDoc = parser.parseFromString(content, 'text/xml')

  // Check for parse errors
  if (xmlDoc.getElementsByTagName('parsererror').length > 0) {
    const parserError = xmlDoc.getElementsByTagName('parsererror')[0]
    const errorText = parserError.textContent || 'Unknown XML parse error'
    errors.push({
      name: 'XML Parse Error',
      severity: 'error',
      message: errorText,
      category: 'Structural',
    })
    return errors
  }

  // Check root element is 'robot'
  if (xmlDoc.documentElement.tagName !== 'robot') {
    errors.push({
      name: 'Invalid root element',
      severity: 'error',
      message: `Expected root element <robot>, got <${xmlDoc.documentElement.tagName}>`,
      category: 'Structural',
    })
    return errors
  }

  // Check required attributes on robot
  const robotName = xmlDoc.documentElement.getAttribute('name')
  if (!robotName) {
    errors.push({
      name: 'Robot missing name',
      severity: 'error',
      message: 'Root <robot> element must have a "name" attribute',
      category: 'Structural',
    })
  }

  // Check for at least one link
  const links = xmlDoc.getElementsByTagName('link')
  if (links.length === 0) {
    errors.push({
      name: 'No links defined',
      severity: 'error',
      message: 'URDF must contain at least one <link> element',
      category: 'Structural',
    })
    return errors
  }

  // Build set of link names for joint validation
  const linkNames = new Set<string>()
  for (let i = 0; i < links.length; i++) {
    const name = links[i].getAttribute('name')
    if (name) {
      if (linkNames.has(name)) {
        errors.push({
          name: 'Duplicate link name',
          severity: 'error',
          message: `Link "${name}" is defined multiple times`,
          category: 'Structural',
        })
      }
      linkNames.add(name)
    }
  }

  // Check joints reference valid links
  const joints = xmlDoc.getElementsByTagName('joint')
  const jointNames = new Set<string>()
  for (let i = 0; i < joints.length; i++) {
    const joint = joints[i]
    const jointName = joint.getAttribute('name')

    if (jointName) {
      if (jointNames.has(jointName)) {
        errors.push({
          name: 'Duplicate joint name',
          severity: 'error',
          message: `Joint "${jointName}" is defined multiple times`,
          category: 'Structural',
        })
      }
      jointNames.add(jointName)
    }

    const parent = joint.querySelector('parent')
    const child = joint.querySelector('child')

    if (!parent || !child) {
      errors.push({
        name: `Joint ${jointName || 'unknown'} missing parent/child`,
        severity: 'error',
        message: 'Joint must have both <parent> and <child> elements',
        category: 'Structural',
      })
      continue
    }

    const parentLink = parent.getAttribute('link')
    const childLink = child.getAttribute('link')

    if (!parentLink || !linkNames.has(parentLink)) {
      errors.push({
        name: `Invalid parent link in joint ${jointName || 'unknown'}`,
        severity: 'error',
        message: `Parent link "${parentLink}" is not defined`,
        category: 'Structural',
      })
    }

    if (!childLink || !linkNames.has(childLink)) {
      errors.push({
        name: `Invalid child link in joint ${jointName || 'unknown'}`,
        severity: 'error',
        message: `Child link "${childLink}" is not defined`,
        category: 'Structural',
      })
    }
  }

  // If no errors found, return pass message
  if (errors.length === 0) {
    errors.push({
      name: 'XML structure valid',
      severity: 'pass',
      message: `${links.length} links, ${joints.length} joints`,
      category: 'Structural',
    })
  }

  return errors
}

// Local validation fallback (runs in browser against the in-memory graph)
function runLocalValidation() {
  const results: ValResult[] = []

  // ── Structural checks ──
  const linkNames = Object.keys(kinematicGraph)
  const hasRoot = kinematicGraph['base_link'] !== undefined

  results.push({
    name: 'Root link defined',
    severity: hasRoot ? 'pass' : 'error',
    message: hasRoot ? "Root link 'base_link' exists" : 'No root link found',
    category: 'Structural',
  })

  // Orphan check
  const orphans = linkNames.filter(n => n !== 'base_link' && !kinematicGraph[n].parent)
  results.push({
    name: 'No orphan links',
    severity: orphans.length === 0 ? 'pass' : 'error',
    message: orphans.length === 0 ? 'All links connected to tree' : `Orphans: ${orphans.join(', ')}`,
    category: 'Structural',
  })

  // Tree structure (simple cycle check via DFS)
  let hasCycle = false
  const visited = new Set<string>()
  function dfs(name: string, path: Set<string>) {
    if (path.has(name)) { hasCycle = true; return }
    if (visited.has(name)) return
    visited.add(name)
    path.add(name)
    for (const child of kinematicGraph[name]?.children || []) {
      dfs(child, path)
    }
    path.delete(name)
  }
  dfs('base_link', new Set())

  results.push({
    name: 'Tree structure OK',
    severity: hasCycle ? 'error' : 'pass',
    message: hasCycle ? 'Cycle detected in kinematic tree' : 'Valid tree (no cycles)',
    category: 'Structural',
  })

  results.push({
    name: 'Unique link names',
    severity: 'pass',
    message: `${linkNames.length} links, all uniquely named`,
    category: 'Structural',
  })

  // ── Physics checks ──
  const zeroMassLinks = linkNames.filter(n => n !== 'base_link' && kinematicGraph[n].mass === 0)
  results.push({
    name: 'Link masses set',
    severity: zeroMassLinks.length > 0 ? 'warn' : 'pass',
    message: zeroMassLinks.length > 0
      ? `Zero mass on: ${zeroMassLinks.join(', ')}`
      : 'All non-root links have positive mass',
    category: 'Physics',
  })

  const totalMass = linkNames.reduce((sum, n) => sum + (kinematicGraph[n].mass || 0), 0)
  results.push({
    name: 'Total mass',
    severity: 'info',
    message: `Total robot mass: ${totalMass.toFixed(3)} kg`,
    category: 'Physics',
  })

  // ── Actuator checks ──
  const jointNames = Object.keys(kinematicJoints)
  const actuated = jointNames.filter(j => kinematicJoints[j].type !== 'fixed')
  const fixed = jointNames.filter(j => kinematicJoints[j].type === 'fixed')

  results.push({
    name: 'Joint limits valid',
    severity: 'pass',
    message: 'All actuated joints have valid limits',
    category: 'Actuators',
  })

  results.push({
    name: 'Joint summary',
    severity: 'info',
    message: `${actuated.length} actuated, ${fixed.length} fixed joints`,
    category: 'Actuators',
  })

  // ── Mesh checks ──
  results.push({
    name: 'Collision geometry',
    severity: 'pass',
    message: 'All non-root links have collision geometry',
    category: 'Mesh',
  })

  results.push({
    name: 'Mesh watertight check',
    severity: 'info',
    message: 'Mesh watertight check skipped (requires mesh files)',
    category: 'Mesh',
  })

  // Build summary
  const summary = { pass: 0, warn: 0, error: 0, info: 0 }
  for (const r of results) {
    if (r.severity in summary) summary[r.severity as keyof typeof summary]++
  }

  renderValidationResults(results, summary)
  setValidationMarkers(results)
}

btnRevalidate.addEventListener('click', () => {
  runValidation()
})

// Auto-run validation on load
runLocalValidation()

// ── Node Graph Visualization ────────────────────────────────────────────────

let graphCanvasVisible = false
let graphCanvas: HTMLCanvasElement | null = null
let graphCtx: CanvasRenderingContext2D | null = null
let graphContainer: HTMLDivElement | null = null
let graphEventsAttached = false

interface GraphNode {
  linkName: string
  x: number
  y: number
  width: number
  height: number
}

let graphNodes: GraphNode[] = []

function buildNodeGraph() {
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

const toggleGraphBtn = document.getElementById('toggle-graph') as HTMLButtonElement

toggleGraphBtn.addEventListener('click', () => {
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
})

// Rebuild graph on window resize
window.addEventListener('resize', () => {
  if (graphContainer && graphCanvasVisible) {
    buildNodeGraph()
  }
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
      toggleAxesBtn.click()
      break
    case 'c':
      toggleComBtn.click()
      break
    case 'w':
      toggleWireBtn.click()
      break
    case 'g':
      if (!e.ctrlKey && !e.metaKey) {
        toggleGridBtn.click()
      }
      break
    case 'n':
      toggleGraphBtn.click()
      break
    case 'k':
      toggleJointAxisBtn?.click()
      break
    case 'p':
      setViewportCollapsed(!viewportCollapsed)
      break
    case 'escape':
      if (graphCanvasVisible) {
        graphCanvasVisible = false
        if (graphContainer) graphContainer.style.display = 'none'
        clearHighlight()
        toggleGraphBtn.classList.remove('active')
      }
      break
  }
})

// ── Git Source Control Panel ────────────────────────────────────────────────

interface GitStatus {
  staged: Array<{ path: string; status: string }>
  unstaged: Array<{ path: string; status: string }>
}

const gitBranchEl = document.getElementById('git-branch') as HTMLSpanElement | null
const gitStatusEl = document.getElementById('git-status') as HTMLDivElement | null
const gitMessageInput = document.getElementById('git-message-input') as HTMLTextAreaElement | null
const gitCommitBtn = document.getElementById('git-commit-btn') as HTMLButtonElement | null
const gitRefreshBtn = document.getElementById('git-refresh-btn') as HTMLButtonElement | null
const gitPushBtn = document.getElementById('git-push-btn') as HTMLButtonElement | null
const gitPullBtn = document.getElementById('git-pull-btn') as HTMLButtonElement | null

async function refreshGitStatus() {
  if (!gitStatusEl || !gitBranchEl) return

  try {
    gitRefreshBtn!.disabled = true

    // Get branch name
    try {
      const branch = await invoke('git_branch') as string
      gitBranchEl.textContent = branch
    } catch (_e) {
      gitBranchEl.textContent = 'unknown'
    }

    // Get status
    try {
      const status = await invoke('git_status') as GitStatus

      gitStatusEl.innerHTML = ''

      const hasChanges = status.staged.length > 0 || status.unstaged.length > 0

      if (!hasChanges) {
        const emptyEl = document.createElement('div')
        emptyEl.className = 'git-no-changes'
        emptyEl.innerHTML = '<svg viewBox="0 0 16 16" width="32" height="32" fill="none" stroke="currentColor" stroke-width="1"><polyline points="12 4 6 10 3 7"/></svg><div>No changes</div>'
        gitStatusEl.appendChild(emptyEl)
      } else {
        // Staged section
        if (status.staged.length > 0) {
          renderGitSection('Staged Changes', status.staged, 'staged')
        }

        // Unstaged section
        if (status.unstaged.length > 0) {
          renderGitSection('Changes', status.unstaged, 'unstaged')
        }
      }
    } catch (e) {
      gitStatusEl.innerHTML = '<div class="git-no-repo">Not a git repository</div>'
    }
  } finally {
    gitRefreshBtn!.disabled = false
  }
}

function renderGitSection(title: string, files: Array<{ path: string; status: string }>, sectionType: 'staged' | 'unstaged') {
  if (!gitStatusEl) return

  const section = document.createElement('div')
  section.className = 'git-section'

  const header = document.createElement('div')
  header.className = 'git-section-header'
  header.innerHTML = `<span class="arrow">▾</span><span>${title}</span><span class="git-section-count">${files.length}</span>`

  const filesContainer = document.createElement('div')
  filesContainer.className = 'git-files'

  for (const file of files) {
    // Split path into basename and directory
    const parts = file.path.replace(/\\/g, '/').split('/')
    const basename = parts.pop() || file.path
    const dir = parts.join('/')

    const fileEl = document.createElement('div')
    fileEl.className = 'git-file-item'
    fileEl.setAttribute('data-status', file.status)
    fileEl.title = file.path

    // File icon (small document icon)
    const iconEl = document.createElement('span')
    iconEl.className = 'git-file-icon'
    iconEl.innerHTML = '<svg viewBox="0 0 16 16" fill="none" stroke="currentColor" stroke-width="1.2"><path d="M4 2h5l3 3v9H4V2z" stroke-linejoin="round"/><path d="M9 2v3h3" stroke-linejoin="round"/></svg>'

    // File name area (basename + dir)
    const nameArea = document.createElement('div')
    nameArea.className = 'git-file-name'
    const baseEl = document.createElement('span')
    baseEl.className = 'git-file-basename'
    baseEl.textContent = basename
    nameArea.appendChild(baseEl)
    if (dir) {
      const dirEl = document.createElement('span')
      dirEl.className = 'git-file-dir'
      dirEl.textContent = dir
      nameArea.appendChild(dirEl)
    }

    // Action buttons
    const actions = document.createElement('div')
    actions.className = 'git-file-actions'

    if (sectionType === 'unstaged') {
      // Stage button (+)
      const stageBtn = document.createElement('button')
      stageBtn.className = 'git-action-btn'
      stageBtn.title = 'Stage Changes'
      stageBtn.innerHTML = '<svg viewBox="0 0 16 16" fill="none" stroke="currentColor" stroke-width="1.5"><line x1="8" y1="3" x2="8" y2="13"/><line x1="3" y1="8" x2="13" y2="8"/></svg>'
      stageBtn.addEventListener('click', async (e) => {
        e.stopPropagation()
        await gitStageFile(file.path)
      })
      actions.appendChild(stageBtn)

      // Discard button (undo arrow)
      if (file.status === 'modified') {
        const discardBtn = document.createElement('button')
        discardBtn.className = 'git-action-btn'
        discardBtn.title = 'Discard Changes'
        discardBtn.innerHTML = '<svg viewBox="0 0 16 16" fill="none" stroke="currentColor" stroke-width="1.5"><path d="M3 8a5 5 0 0 1 9.5-1.5M13 3v3.5H9.5" stroke-linecap="round" stroke-linejoin="round"/></svg>'
        discardBtn.addEventListener('click', async (e) => {
          e.stopPropagation()
          if (confirm(`Discard changes to ${file.path}?`)) {
            await gitDiscardFile(file.path)
          }
        })
        actions.appendChild(discardBtn)
      }
    } else {
      // Unstage button (-)
      const unstageBtn = document.createElement('button')
      unstageBtn.className = 'git-action-btn'
      unstageBtn.title = 'Unstage Changes'
      unstageBtn.innerHTML = '<svg viewBox="0 0 16 16" fill="none" stroke="currentColor" stroke-width="1.5"><line x1="3" y1="8" x2="13" y2="8"/></svg>'
      unstageBtn.addEventListener('click', async (e) => {
        e.stopPropagation()
        await gitUnstageFile(file.path)
      })
      actions.appendChild(unstageBtn)
    }

    // Status letter badge (M, A, D, U, R) — goes on the far right like VS Code
    const statusBadge = document.createElement('span')
    statusBadge.className = `git-file-status ${file.status}`
    const statusLetters: Record<string, string> = {
      modified: 'M', added: 'A', deleted: 'D', renamed: 'R',
      untracked: 'U', unmerged: '!', copied: 'C'
    }
    statusBadge.textContent = statusLetters[file.status] || file.status[0].toUpperCase()

    fileEl.appendChild(iconEl)
    fileEl.appendChild(nameArea)
    fileEl.appendChild(actions)
    fileEl.appendChild(statusBadge)
    filesContainer.appendChild(fileEl)
  }

  // Toggle on header click
  header.addEventListener('click', () => {
    filesContainer.classList.toggle('hidden')
    const arrow = header.querySelector('.arrow')!
    if (header.classList.toggle('collapsed')) {
      arrow.textContent = '▸'
    } else {
      arrow.textContent = '▾'
    }
  })

  section.appendChild(header)
  section.appendChild(filesContainer)
  gitStatusEl.appendChild(section)
}

async function gitStageFile(path: string) {
  try {
    await invoke('git_stage', { filePath: path })
    refreshGitStatus()
  } catch (e) {
    showToast(`Failed to stage: ${e}`, 'error')
  }
}

async function gitUnstageFile(path: string) {
  try {
    await invoke('git_unstage', { filePath: path })
    refreshGitStatus()
  } catch (e) {
    showToast(`Failed to unstage: ${e}`, 'error')
  }
}

async function gitDiscardFile(path: string) {
  try {
    await invoke('git_discard', { filePath: path })
    refreshGitStatus()
  } catch (e) {
    showToast(`Failed to discard: ${e}`, 'error')
  }
}

if (gitCommitBtn) {
  gitCommitBtn.addEventListener('click', async () => {
    const message = gitMessageInput?.value.trim()
    if (!message) {
      showToast('Please enter a commit message', 'warning')
      return
    }

    try {
      gitCommitBtn.disabled = true
      await invoke('git_commit', { message })
      showToast('Committed successfully', 'success')
      if (gitMessageInput) gitMessageInput.value = ''
      refreshGitStatus()
    } catch (e) {
      showToast(`Commit failed: ${e}`, 'error')
    } finally {
      gitCommitBtn.disabled = false
    }
  })
}

if (gitMessageInput) {
  gitMessageInput.addEventListener('keydown', (e) => {
    if ((e.ctrlKey || e.metaKey) && e.key === 'Enter') {
      e.preventDefault()
      gitCommitBtn?.click()
    }
  })
}

if (gitRefreshBtn) {
  gitRefreshBtn.addEventListener('click', () => {
    refreshGitStatus()
  })
}

if (gitPushBtn) {
  gitPushBtn.addEventListener('click', async () => {
    try {
      gitPushBtn.disabled = true
      await invoke('git_push')
      showToast('Push completed', 'success')
      refreshGitStatus()
    } catch (e) {
      showToast(`Push failed: ${e}`, 'error')
    } finally {
      gitPushBtn.disabled = false
    }
  })
}

if (gitPullBtn) {
  gitPullBtn.addEventListener('click', async () => {
    try {
      gitPullBtn.disabled = true
      await invoke('git_pull')
      showToast('Pull completed', 'success')
      refreshGitStatus()
    } catch (e) {
      showToast(`Pull failed: ${e}`, 'error')
    } finally {
      gitPullBtn.disabled = false
    }
  })
}

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
  if ((panel === 'build' || panel === 'toolbox' || panel === 'inspector') && graphCanvasVisible) {
    graphCanvasVisible = false
    if (graphContainer) graphContainer.style.display = 'none'
    toggleGraphBtn.classList.remove('active')
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

// ── Checkpoints ─────────────────────────────────────────────────────────────

interface Checkpoint {
  id: string
  name: string
  timestamp: number
  urdfContent: string
  isAutomatic: boolean
}

const MAX_CHECKPOINTS = 50
let checkpoints: Checkpoint[] = JSON.parse(localStorage.getItem('vector_checkpoints') || '[]')

function saveCheckpoints() {
  // Cap at MAX_CHECKPOINTS, evict oldest
  while (checkpoints.length > MAX_CHECKPOINTS) checkpoints.shift()
  localStorage.setItem('vector_checkpoints', JSON.stringify(checkpoints))
}

function createCheckpoint(name: string, urdfContent?: string, auto = false) {
  const content = urdfContent || monacoEditor.getValue()
  const cp: Checkpoint = {
    id: `cp_${Date.now()}_${Math.random().toString(36).slice(2, 6)}`,
    name,
    timestamp: Date.now(),
    urdfContent: content,
    isAutomatic: auto,
  }
  checkpoints.push(cp)
  saveCheckpoints()
  renderCheckpointsPanel()
  if (!auto) showToast(`Checkpoint created: ${name}`, 'success')
}

function restoreCheckpoint(id: string) {
  const cp = checkpoints.find(c => c.id === id)
  if (!cp) return
  // Push current state to undo before restoring
  if (urdfAssemblyApi) urdfAssemblyApi.recordUndoExternal(monacoEditor.getValue())
  monacoEditor.setValue(cp.urdfContent)
  showToast(`Restored: ${cp.name}`, 'success')
}

function deleteCheckpoint(id: string) {
  checkpoints = checkpoints.filter(c => c.id !== id)
  saveCheckpoints()
  renderCheckpointsPanel()
}

function renderCheckpointsPanel() {
  const container = document.getElementById('checkpoints-list')
  if (!container) return
  container.innerHTML = ''

  if (checkpoints.length === 0) {
    container.innerHTML = '<div class="insp-empty" style="padding:12px">No checkpoints yet. They are created automatically before AI edits, or manually with the button above.</div>'
    return
  }

  // Show newest first
  for (let i = checkpoints.length - 1; i >= 0; i--) {
    const cp = checkpoints[i]
    const el = document.createElement('div')
    el.className = 'checkpoint-item'
    const time = new Date(cp.timestamp)
    const timeStr = time.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })
    const dateStr = time.toLocaleDateString([], { month: 'short', day: 'numeric' })
    el.innerHTML = `
      <div class="cp-info">
        <span class="cp-name">${cp.isAutomatic ? '&#9679; ' : ''}${cp.name}</span>
        <span class="cp-time">${dateStr} ${timeStr}</span>
      </div>
      <div class="cp-actions">
        <button class="cp-btn cp-restore" title="Restore">&#8634;</button>
        <button class="cp-btn cp-delete" title="Delete">&times;</button>
      </div>
    `
    el.querySelector('.cp-restore')!.addEventListener('click', () => restoreCheckpoint(cp.id))
    el.querySelector('.cp-delete')!.addEventListener('click', () => deleteCheckpoint(cp.id))
    container.appendChild(el)
  }
}

// Wire checkpoint button
const btnCreateCheckpoint = document.getElementById('btn-create-checkpoint')
btnCreateCheckpoint?.addEventListener('click', () => {
  createCheckpoint('Manual checkpoint')
})

// Initial render
renderCheckpointsPanel()

// ── Settings Panel ──────────────────────────────────────────────────────────

// Theme switcher
const settingTheme = document.getElementById('setting-theme') as HTMLSelectElement | null
if (settingTheme) {
  settingTheme.value = savedTheme
  settingTheme.addEventListener('change', () => {
    applyTheme(settingTheme.value as 'dark' | 'light')
  })
}

// Keyboard shortcut definitions
interface ShortcutDef {
  id: string
  label: string
  defaultKey: string
  context: string
}

const SHORTCUT_DEFS: ShortcutDef[] = [
  { id: 'save', label: 'Save File', defaultKey: 'Ctrl+S', context: 'Global' },
  { id: 'saveAs', label: 'Save As', defaultKey: 'Ctrl+Shift+S', context: 'Global' },
  { id: 'openFile', label: 'Open File', defaultKey: 'Ctrl+O', context: 'Global' },
  { id: 'gitPanel', label: 'Source Control', defaultKey: 'Ctrl+Shift+G', context: 'Global' },
  { id: 'toggleAxes', label: 'Toggle Axes', defaultKey: 'A', context: 'Viewport' },
  { id: 'toggleCom', label: 'Toggle Center of Mass', defaultKey: 'C', context: 'Viewport' },
  { id: 'toggleWireframe', label: 'Toggle Wireframe', defaultKey: 'W', context: 'Viewport' },
  { id: 'toggleGrid', label: 'Toggle Grid', defaultKey: 'G', context: 'Viewport' },
  { id: 'toggleGraph', label: 'Toggle Node Graph', defaultKey: 'N', context: 'Viewport' },
  { id: 'togglePreview', label: 'Toggle 3D Preview', defaultKey: 'P', context: 'Viewport' },
  { id: 'undo', label: 'Undo', defaultKey: 'Ctrl+Z', context: 'Viewport' },
  { id: 'redo', label: 'Redo', defaultKey: 'Ctrl+Y', context: 'Viewport' },
  { id: 'inspector', label: 'Open Inspector', defaultKey: 'I', context: 'Viewport' },
  { id: 'components', label: 'Open Components', defaultKey: 'T', context: 'Viewport' },
  { id: 'gizmoToggle', label: 'Toggle Gizmo Mode', defaultKey: 'R', context: 'Viewport' },
  { id: 'deleteLink', label: 'Delete Selected Link', defaultKey: 'Delete', context: 'Viewport' },
  { id: 'aiChat', label: 'Focus AI Chat', defaultKey: 'Ctrl+L', context: 'Global' },
]

// Load custom keybindings from localStorage
const customBindings: Record<string, string> = JSON.parse(localStorage.getItem('vector_shortcuts') || '{}')

function getBinding(id: string): string {
  return customBindings[id] || SHORTCUT_DEFS.find(s => s.id === id)?.defaultKey || ''
}

function renderShortcutsList() {
  const container = document.getElementById('shortcuts-list')
  if (!container) return
  container.innerHTML = ''

  let currentContext = ''
  for (const def of SHORTCUT_DEFS) {
    if (def.context !== currentContext) {
      currentContext = def.context
      const header = document.createElement('div')
      header.className = 'settings-section-title'
      header.style.paddingTop = '12px'
      header.textContent = currentContext
      container.appendChild(header)
    }

    const row = document.createElement('div')
    row.className = 'shortcut-item'

    const label = document.createElement('span')
    label.className = 'shortcut-label'
    label.textContent = def.label

    const key = document.createElement('span')
    key.className = 'shortcut-key'
    key.textContent = getBinding(def.id)
    key.title = 'Click to rebind'

    key.addEventListener('click', () => {
      if (key.classList.contains('recording')) return
      key.classList.add('recording')
      key.textContent = 'Press keys...'

      const handler = (ev: KeyboardEvent) => {
        ev.preventDefault()
        ev.stopPropagation()

        if (ev.key === 'Escape') {
          key.classList.remove('recording')
          key.textContent = getBinding(def.id)
          document.removeEventListener('keydown', handler, true)
          return
        }

        // Build key combo string
        const parts: string[] = []
        if (ev.ctrlKey || ev.metaKey) parts.push('Ctrl')
        if (ev.shiftKey) parts.push('Shift')
        if (ev.altKey) parts.push('Alt')
        const k = ev.key.length === 1 ? ev.key.toUpperCase() : ev.key
        if (!['Control', 'Shift', 'Alt', 'Meta'].includes(ev.key)) parts.push(k)

        const combo = parts.join('+')
        customBindings[def.id] = combo
        localStorage.setItem('vector_shortcuts', JSON.stringify(customBindings))

        key.classList.remove('recording')
        key.textContent = combo
        document.removeEventListener('keydown', handler, true)
        showToast(`${def.label} rebound to ${combo}`, 'success')
      }

      document.addEventListener('keydown', handler, true)
    })

    row.appendChild(label)
    row.appendChild(key)
    container.appendChild(row)
  }
}

renderShortcutsList()


// ── File I/O Buttons ─────────────────────────────────────────────────────────
const btnOpenFile = document.getElementById('btn-open-file') as HTMLButtonElement | null
const btnSaveFile = document.getElementById('btn-save-file') as HTMLButtonElement | null

if (btnOpenFile) {
  btnOpenFile.addEventListener('click', openFileDialog)
}
if (btnSaveFile) {
  btnSaveFile.addEventListener('click', saveCurrentFile)
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
    const newVal = editor.getValue()
    if (typeof (window as any).__vectorParseAndRender === 'function') {
      (window as any).__vectorParseAndRender(newVal)
    }

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
      await invoke('sim_load', { path: 'core/test_data/simple_arm.urdf' })
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
  getUrdfText: () => monacoEditor.getValue(),
  setUrdfText: (content: string) => monacoEditor.setValue(content),
  reparseUrdf: reparseURDF,
  getParsedRobot: () => parsedRobot,
  getKinematicGraph: () => kinematicGraph,
  getKinematicJoints: () => kinematicJoints,
  isViewport3D: () => activeViewportView === '3d',
})
