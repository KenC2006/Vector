/**
 * Settings, Themes, and Checkpoints module for Vector IDE.
 *
 * Extracted from main.ts to keep the main file focused on core editor/viewport logic.
 */

import * as THREE from 'three'
import * as monaco from 'monaco-editor'

// ── Theme system ────────────────────────────────────────────────────────────

export type ThemeId = 'dark' | 'night-owl' | 'tokyo-night'

export const MONACO_THEMES: Record<ThemeId, string> = {
  'dark': 'vector-dark',
  'night-owl': 'vector-night-owl',
  'tokyo-night': 'vector-tokyo-night',
}

export const VIEWPORT_BG: Record<ThemeId, number> = {
  'dark': 0x1a1a1a,
  'night-owl': 0x011627,
  'tokyo-night': 0x16161e,
}

/**
 * Register all custom Monaco themes. Call this BEFORE creating the editor.
 */
export function registerThemes(m: typeof monaco) {
  // VS Code Dark+ accurate theme
  m.editor.defineTheme('vector-dark', {
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

  // Night Owl theme for Monaco
  m.editor.defineTheme('vector-night-owl', {
    base: 'vs-dark',
    inherit: true,
    rules: [
      { token: 'comment', foreground: '637777', fontStyle: 'italic' },
      { token: 'tag', foreground: 'caece6' },
      { token: 'attribute.name', foreground: 'c5e478' },
      { token: 'attribute.value', foreground: 'ecc48d' },
      { token: 'string', foreground: 'ecc48d' },
      { token: 'number', foreground: 'F78C6C' },
      { token: 'keyword', foreground: 'c792ea' },
      { token: 'type', foreground: 'ffcb8b' },
      { token: 'delimiter', foreground: 'd6deeb' },
      { token: 'delimiter.xml', foreground: 'd6deeb' },
      { token: 'key', foreground: 'c5e478' },
      { token: 'metatag', foreground: '82aaff' },
      { token: 'metatag.content.xml', foreground: 'ecc48d' },
    ],
    colors: {
      'editor.background': '#011627',
      'editor.foreground': '#d6deeb',
      'editorLineNumber.foreground': '#4b6479',
      'editorLineNumber.activeForeground': '#89a4bb',
      'editor.selectionBackground': '#1d3b53',
      'editor.lineHighlightBackground': '#28707d29',
      'editorCursor.foreground': '#80a4c2',
      'editorIndentGuide.background': '#122d42',
      'editorIndentGuide.activeBackground': '#1d3b53',
      'scrollbarSlider.background': '#1d3b5366',
      'scrollbarSlider.hoverBackground': '#1d3b53b3',
      'minimap.background': '#011627',
      'editorWidget.background': '#0b2942',
      'editorWidget.border': '#122d42',
      'editorSuggestWidget.background': '#0b2942',
      'editorSuggestWidget.border': '#122d42',
      'editorSuggestWidget.selectedBackground': '#1d3b53',
    },
  })

  // Tokyo Night theme for Monaco
  m.editor.defineTheme('vector-tokyo-night', {
    base: 'vs-dark',
    inherit: true,
    rules: [
      { token: 'comment', foreground: '51597d', fontStyle: 'italic' },
      { token: 'tag', foreground: 'f7768e' },
      { token: 'attribute.name', foreground: 'bb9af7' },
      { token: 'attribute.value', foreground: '9ece6a' },
      { token: 'string', foreground: '9ece6a' },
      { token: 'number', foreground: 'ff9e64' },
      { token: 'keyword', foreground: 'bb9af7' },
      { token: 'type', foreground: '0db9d7' },
      { token: 'delimiter', foreground: 'a9b1d6' },
      { token: 'delimiter.xml', foreground: 'a9b1d6' },
      { token: 'key', foreground: '7aa2f7' },
      { token: 'metatag', foreground: '7aa2f7' },
      { token: 'metatag.content.xml', foreground: '9ece6a' },
    ],
    colors: {
      'editor.background': '#1a1b26',
      'editor.foreground': '#a9b1d6',
      'editorLineNumber.foreground': '#363b54',
      'editorLineNumber.activeForeground': '#787c99',
      'editor.selectionBackground': '#515c7e4d',
      'editor.lineHighlightBackground': '#1e202e',
      'editorCursor.foreground': '#c0caf5',
      'editorIndentGuide.background': '#292e42',
      'editorIndentGuide.activeBackground': '#363b54',
      'scrollbarSlider.background': '#292e4266',
      'scrollbarSlider.hoverBackground': '#363b54b3',
      'minimap.background': '#1a1b26',
      'editorWidget.background': '#1e202e',
      'editorWidget.border': '#292e42',
      'editorSuggestWidget.background': '#1e202e',
      'editorSuggestWidget.border': '#292e42',
      'editorSuggestWidget.selectedBackground': '#24283b',
    },
  })
}

// ── Checkpoints ─────────────────────────────────────────────────────────────

interface Checkpoint {
  id: string
  name: string
  timestamp: number
  urdfContent: string
  isAutomatic: boolean
}

// ── Keyboard Shortcuts ──────────────────────────────────────────────────────

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
  { id: 'toggleAxes', label: 'Toggle Axes', defaultKey: 'Shift+A', context: 'Viewport' },
  { id: 'toggleCom', label: 'Toggle Center of Mass', defaultKey: 'C', context: 'Viewport' },
  { id: 'toggleWireframe', label: 'Toggle Wireframe', defaultKey: 'Shift+W', context: 'Viewport' },
  { id: 'viewportPan', label: 'Pan View (WASD, Shift faster)', defaultKey: 'W A S D', context: 'Viewport' },
  { id: 'toggleGrid', label: 'Toggle Grid', defaultKey: 'G', context: 'Viewport' },
  { id: 'toggleGraph', label: 'Toggle Node Graph', defaultKey: 'N', context: 'Viewport' },
  { id: 'togglePreview', label: 'Toggle 3D Preview', defaultKey: 'P', context: 'Viewport' },
  { id: 'focusMode', label: 'Focus Mode (Sidebar + 3D)', defaultKey: 'Shift+F', context: 'Viewport' },
  { id: 'undo', label: 'Undo', defaultKey: 'Ctrl+Z', context: 'Viewport' },
  { id: 'redo', label: 'Redo', defaultKey: 'Ctrl+Y', context: 'Viewport' },
  { id: 'inspector', label: 'Open Inspector', defaultKey: 'I', context: 'Viewport' },
  { id: 'components', label: 'Open Components', defaultKey: 'T', context: 'Viewport' },
  { id: 'gizmoToggle', label: 'Toggle Gizmo Mode', defaultKey: 'R', context: 'Viewport' },
  { id: 'deleteLink', label: 'Delete Selected Link', defaultKey: 'Delete', context: 'Viewport' },
  { id: 'aiChat', label: 'Focus AI Chat', defaultKey: 'Ctrl+L', context: 'Global' },
  { id: 'toggleSidebar', label: 'Toggle Sidebar', defaultKey: 'Ctrl+B', context: 'Global' },
]

const customBindings: Record<string, string> = JSON.parse(localStorage.getItem('vector_shortcuts') || '{}')

function getBinding(id: string): string {
  return customBindings[id] || SHORTCUT_DEFS.find(s => s.id === id)?.defaultKey || ''
}

// ── Init ────────────────────────────────────────────────────────────────────

export function initSettings(deps: {
  monacoEditor: monaco.editor.IStandaloneCodeEditor
  showToast: (msg: string, type?: 'success' | 'warning' | 'error' | 'info') => void
  /** The editor (for undo); a getter because it's created after settings. */
  getEditorApi: () => { recordUndoExternal(content: string): void } | null
  renderer?: THREE.WebGLRenderer
}): {
  applyTheme: (theme: ThemeId) => void
  createCheckpoint: (name: string, urdfContent?: string, auto?: boolean) => void
  savedTheme: ThemeId
} {
  const { monacoEditor, showToast, renderer } = deps

  // ── applyTheme ──────────────────────────────────────────────────────────
  function applyTheme(theme: ThemeId) {
    document.documentElement.classList.remove('theme-night-owl', 'theme-tokyo-night')
    if (theme !== 'dark') {
      document.documentElement.classList.add(`theme-${theme}`)
    }
    monaco.editor.setTheme(MONACO_THEMES[theme] || 'vector-dark')
    localStorage.setItem('vector_theme', theme)
    const select = document.getElementById('setting-theme') as HTMLSelectElement | null
    if (select) select.value = theme
    try { renderer?.setClearColor(VIEWPORT_BG[theme] || 0x1a1a1a) } catch {}
  }

  // Apply saved theme on load
  const savedTheme = (localStorage.getItem('vector_theme') || 'dark') as ThemeId
  if (savedTheme !== 'dark') applyTheme(savedTheme)

  // ── Checkpoints ─────────────────────────────────────────────────────────
  const MAX_CHECKPOINTS = 50
  let checkpoints: Checkpoint[] = JSON.parse(localStorage.getItem('vector_checkpoints') || '[]')

  function saveCheckpoints() {
    while (checkpoints.length > MAX_CHECKPOINTS) checkpoints.shift()
    localStorage.setItem('vector_checkpoints', JSON.stringify(checkpoints))
  }

  function createCheckpoint(name: string, urdfContent?: string, auto = false) {
    const content = urdfContent || monacoEditor.getModel()?.getValue() || ''
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
    // The URDF carries its design, so restoring the text restores everything.
    const editor = deps.getEditorApi()
    if (editor && monacoEditor.getModel()) editor.recordUndoExternal(monacoEditor.getValue())
    monacoEditor.setValue(cp.urdfContent)
    showToast(`Restored: ${cp.name}`, 'success')
  }

  function deleteCheckpoint(id: string) {
    checkpoints = checkpoints.filter(c => c.id !== id)
    saveCheckpoints()
    renderCheckpointsPanel()
  }

  function renameCheckpoint(id: string, newName: string) {
    const trimmed = newName.trim()
    if (!trimmed) return  // empty rename — drop silently, render restores the old name
    const cp = checkpoints.find(c => c.id === id)
    if (!cp || cp.name === trimmed) return
    cp.name = trimmed
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

    for (let i = checkpoints.length - 1; i >= 0; i--) {
      const cp = checkpoints[i]
      const el = document.createElement('div')
      el.className = 'checkpoint-item'
      const time = new Date(cp.timestamp)
      const timeStr = time.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })
      const dateStr = time.toLocaleDateString([], { month: 'short', day: 'numeric' })
      el.innerHTML = `
        <div class="cp-info">
          <span class="cp-name" title="Double-click to rename"></span>
          <span class="cp-time">${dateStr} ${timeStr}</span>
        </div>
        <div class="cp-actions">
          <button class="cp-btn cp-rename" title="Rename">&#9998;</button>
          <button class="cp-btn cp-restore" title="Restore">&#8634;</button>
          <button class="cp-btn cp-delete" title="Delete">&times;</button>
        </div>
      `
      const nameEl = el.querySelector('.cp-name') as HTMLSpanElement
      // Set name via textContent to avoid HTML injection from user-typed names.
      // The ● prefix marks auto-generated checkpoints (pre-AI-edit snapshots).
      nameEl.textContent = (cp.isAutomatic ? '● ' : '') + cp.name

      const startEdit = () => {
        const input = document.createElement('input')
        input.type = 'text'
        input.className = 'cp-name-input'
        input.value = cp.name
        input.maxLength = 80
        nameEl.replaceWith(input)
        input.focus()
        input.select()
        let committed = false
        const commit = () => {
          if (committed) return
          committed = true
          renameCheckpoint(cp.id, input.value)
          // Either rename succeeded (rerender) or we re-render to restore the
          // original name. saveCheckpoints + render is invoked inside rename.
          renderCheckpointsPanel()
        }
        const cancel = () => {
          if (committed) return
          committed = true
          renderCheckpointsPanel()
        }
        input.addEventListener('keydown', e => {
          if (e.key === 'Enter') { e.preventDefault(); commit() }
          else if (e.key === 'Escape') { e.preventDefault(); cancel() }
        })
        input.addEventListener('blur', commit)
      }

      nameEl.addEventListener('dblclick', startEdit)
      el.querySelector('.cp-rename')!.addEventListener('click', startEdit)
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

  // ── Settings Panel: Theme switcher ──────────────────────────────────────
  const settingTheme = document.getElementById('setting-theme') as HTMLSelectElement | null
  if (settingTheme) {
    settingTheme.value = savedTheme
    settingTheme.addEventListener('change', () => {
      applyTheme(settingTheme.value as ThemeId)
    })
  }

  // ── Shortcuts list ──────────────────────────────────────────────────────
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

  return { applyTheme, createCheckpoint, savedTheme }
}
