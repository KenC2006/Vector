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
  let pendingNewText: string | null = null
  let activeChatActionsId: string | null = null

  function showInlineDiff(oldText: string, newText: string, _newUrdf?: string) {
    const editor = (window as any).__vectorEditor as monaco.editor.IStandaloneCodeEditor | undefined
    if (!editor) return

    pendingOldText = oldText
    pendingNewText = newText

    const oldLines = oldText.split('\n')
    const newLines = newText.split('\n')
    const maxLen = Math.max(oldLines.length, newLines.length)
    // Lines in oldText that will be modified or removed — decoratable now.
    const changedOldLines: number[] = []
    let addedCount = 0
    let totalChanged = 0

    for (let i = 0; i < maxLen; i++) {
      const oldLine = i < oldLines.length ? oldLines[i] : undefined
      const newLine = i < newLines.length ? newLines[i] : undefined
      if (oldLine === newLine) continue
      totalChanged++
      if (oldLine !== undefined) {
        changedOldLines.push(i + 1)
      } else {
        addedCount++
      }
    }

    // Show OLD text as the baseline while the user reviews — this makes Accept
    // a visible action (text changes to newText on click) and Dismiss a no-op
    // visually. If the editor already has newText (e.g. the assembly engine
    // committed it before this call), we revert to oldText here and reparse so
    // the 3D viewport also reflects the pre-change state during review.
    const textWasReverted = editor.getValue() !== oldText
    if (textWasReverted) {
      editor.setValue(oldText)
      // The debounced reparse in onDidChangeModelContent is suppressed while a
      // diff is pending (see main.ts), so trigger one explicitly to sync 3D.
      deps.reparseURDF()
    }

    const decorations: monaco.editor.IModelDeltaDecoration[] = changedOldLines.map(lineNum => ({
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
    const addedSuffix = addedCount > 0 ? ` (+${addedCount} new)` : ''
    bar.innerHTML = `
      <span class="idb-label">${totalChanged} lines changed${addedSuffix}</span>
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

    if (changedOldLines.length > 0) editor.revealLineInCenter(changedOldLines[0])
  }

  function acceptInlineDiff() {
    const editor = (window as any).__vectorEditor as monaco.editor.IStandaloneCodeEditor | undefined
    const newText = pendingNewText
    const urdfApi = deps.getUrdfAssemblyApi()
    if (pendingOldText && urdfApi) {
      urdfApi.recordUndoExternal(pendingOldText)
    }

    if (pendingOldText) {
      deps.createCheckpoint('Before AI edit', pendingOldText, true)
    }

    clearInlineDiff()
    pendingOldText = null
    pendingNewText = null

    const t = deps.getReparseTimeout()
    if (t !== null) { clearTimeout(t); deps.setReparseTimeout(null) }

    // Apply newText now — showInlineDiff left oldText in the editor so Accept
    // is the visible commit step. Skip the write if somehow already equal.
    if (editor && newText !== null && editor.getValue() !== newText) {
      editor.setValue(newText)
    }

    deps.showToast('Changes accepted', 'success')
    syncChatActions('accept')

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
    pendingNewText = null

    const t = deps.getReparseTimeout()
    if (t !== null) { clearTimeout(t); deps.setReparseTimeout(null) }

    // showInlineDiff left oldText in the editor, so usually we have nothing to
    // revert in the text. But the 3D scene may still reflect newText from a
    // prior assembly commit — always reparse so the viewport matches oldText.
    if (editor && oldText !== null) {
      if (editor.getValue() !== oldText) editor.setValue(oldText)
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
    pendingNewText = null
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
