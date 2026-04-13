// inlineDiff.ts — Inline diff display and accept/dismiss logic in Monaco

import * as monaco from 'monaco-editor'

export interface InlineDiffApi {
  showInlineDiff(oldText: string, newText: string, newUrdf?: string): void
  acceptInlineDiff(): void
  dismissInlineDiff(): void
  clearInlineDiff(): void
  clearPendingDiff(): void
  syncChatActions(action: 'accept' | 'dismiss'): void
  getPendingOldText(): string | null
  setActiveChatActionsId(id: string | null): void
}

export interface InlineDiffDeps {
  getUrdfAssemblyApi(): { recordUndoExternal(text: string): void } | null
  createCheckpoint(label: string, urdf: string, auto: boolean): void
  getReparseTimeout(): number | null
  setReparseTimeout(id: number | null): void
  reparseURDF(): void
  runLocalValidation(): void
  showToast(msg: string, type?: 'success' | 'error' | 'warning' | 'info'): void
}

export function initInlineDiff(deps: InlineDiffDeps): InlineDiffApi {
  let inlineDiffCollection: monaco.editor.IEditorDecorationsCollection | null = null
  let inlineDiffWidget: HTMLElement | null = null
  let pendingOldText: string | null = null
  let activeChatActionsId: string | null = null

  function showInlineDiff(oldText: string, newText: string, _newUrdf?: string) {
    const editor = (window as any).__vectorEditor as monaco.editor.IStandaloneCodeEditor | undefined
    if (!editor) return

    pendingOldText = oldText

    const oldLines = oldText.split('\n')
    const newLines = newText.split('\n')
    const maxLen = Math.max(oldLines.length, newLines.length)
    const changedLines: number[] = []

    for (let i = 0; i < maxLen; i++) {
      const oldLine = i < oldLines.length ? oldLines[i] : undefined
      const newLine = i < newLines.length ? newLines[i] : undefined
      if (oldLine !== newLine && newLine !== undefined) {
        changedLines.push(i + 1)
      }
    }

    editor.setValue(newText)

    const decorations: monaco.editor.IModelDeltaDecoration[] = changedLines.map(lineNum => ({
      range: new monaco.Range(lineNum, 1, lineNum, 1),
      options: {
        isWholeLine: true,
        className: 'inline-diff-added',
        linesDecorationsClassName: 'inline-diff-gutter-added',
      }
    }))

    if (inlineDiffCollection) inlineDiffCollection.clear()
    inlineDiffCollection = editor.createDecorationsCollection(decorations)

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
      bar.style.top = (rect.top + 8) + 'px'
      bar.style.right = (window.innerWidth - rect.right + 20) + 'px'
    } else {
      bar.style.top = '10px'
      bar.style.right = '10px'
    }
    document.body.appendChild(bar)
    inlineDiffWidget = bar

    bar.querySelector('.idb-accept')!.addEventListener('click', () => acceptInlineDiff())
    bar.querySelector('.idb-dismiss')!.addEventListener('click', () => dismissInlineDiff())

    if (changedLines.length > 0) editor.revealLineInCenter(changedLines[0])
  }

  function acceptInlineDiff() {
    const urdfApi = deps.getUrdfAssemblyApi()
    if (pendingOldText && urdfApi) {
      urdfApi.recordUndoExternal(pendingOldText)
    }

    if (pendingOldText) {
      deps.createCheckpoint('Before AI edit', pendingOldText, true)
    }

    clearInlineDiff()
    pendingOldText = null

    const t = deps.getReparseTimeout()
    if (t !== null) { clearTimeout(t); deps.setReparseTimeout(null) }

    deps.showToast('Changes accepted', 'success')
    syncChatActions('accept')

    const editor = (window as any).__vectorEditor as monaco.editor.IStandaloneCodeEditor | undefined
    if (editor) {
      deps.reparseURDF()
      deps.runLocalValidation()
    }
  }

  function dismissInlineDiff() {
    const editor = (window as any).__vectorEditor as monaco.editor.IStandaloneCodeEditor | undefined
    const oldText = pendingOldText

    clearInlineDiff()
    pendingOldText = null

    const t = deps.getReparseTimeout()
    if (t !== null) { clearTimeout(t); deps.setReparseTimeout(null) }

    if (editor && oldText !== null) {
      editor.setValue(oldText)
      deps.reparseURDF()
    }

    deps.showToast('Changes dismissed', 'info')
    syncChatActions('dismiss')
  }

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

  function clearPendingDiff() {
    clearInlineDiff()
    pendingOldText = null
  }

  return {
    showInlineDiff,
    acceptInlineDiff,
    dismissInlineDiff,
    clearInlineDiff,
    clearPendingDiff,
    syncChatActions,
    getPendingOldText: () => pendingOldText,
    setActiveChatActionsId: (id) => { activeChatActionsId = id },
  }
}
