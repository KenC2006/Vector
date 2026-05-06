// inlineDiff.ts — Inline diff display and accept/dismiss logic in Monaco.
// State is scoped per file: each URDF tab carries its own pending diff so
// that switching tabs (when allowed) doesn't lose an undecided edit. The
// active-file pointer is set externally via setActiveFile() — when the
// pointer changes, any visible diff for the previous file is stashed and the
// new file's stash (if any) is rehydrated.

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
  setActiveFile(fileKey: string): void
  removeFile(fileKey: string): void
}

export interface InlineDiffDeps {
  getUrdfAssemblyApi(): { recordUndoExternal(text: string): void } | null
  createCheckpoint(label: string, urdf: string, auto: boolean): void
  getReparseTimeout(): number | null
  setReparseTimeout(id: number | null): void
  reparseURDF(xml?: string): void
  runLocalValidation(): void
  showToast(msg: string, type?: 'success' | 'error' | 'warning' | 'info'): void
}

interface DiffStash {
  oldText: string
  newText: string
  newUrdf?: string
  activeChatActionsId: string | null
}

export function initInlineDiff(deps: InlineDiffDeps): InlineDiffApi {
  let inlineDiffCollection: monaco.editor.IEditorDecorationsCollection | null = null
  let inlineDiffWidget: HTMLElement | null = null
  let pendingOldText: string | null = null
  let pendingNewText: string | null = null
  let pendingNewUrdf: string | undefined = undefined
  let activeChatActionsId: string | null = null

  // Per-file stash. Populated when setActiveFile() switches scope away from a
  // file with a live diff; consumed when scope returns. Stash entries survive
  // until acceptInlineDiff/dismissInlineDiff/removeFile clears them.
  const fileStash: Record<string, DiffStash> = {}
  let activeFileKey = ''

  function releaseAiBusyLock(editor?: monaco.editor.IStandaloneCodeEditor) {
    document.body.classList.remove('ai-busy')
    editor?.updateOptions({ readOnly: false })
    // viewportChat.setAiBusy() owns vc-send/vc-input.disabled — but if the
    // user clicked Accept/Dismiss on the inline diff bar (instead of the
    // chat message buttons), that path never fires. Mirror it here so the
    // chat doesn't stay frozen after an editor-bar accept.
    const vcSend = document.getElementById('vc-send') as HTMLButtonElement | null
    const vcInput = document.getElementById('vc-input') as HTMLTextAreaElement | null
    if (vcSend) vcSend.disabled = false
    if (vcInput) vcInput.disabled = false
  }

  function showInlineDiff(oldText: string, newText: string, newUrdf?: string) {
    const editor = (window as any).__vectorEditor as monaco.editor.IStandaloneCodeEditor | undefined
    if (!editor) return

    pendingOldText = oldText
    pendingNewText = newText
    pendingNewUrdf = newUrdf

    const oldLines = oldText.split('\n')
    const newLines = newText.split('\n')
    const maxLen = Math.max(oldLines.length, newLines.length)
    // Lines in oldText that will be modified or removed — decoratable if we
    // revert the editor to oldText.
    const changedOldLines: number[] = []
    // Lines in newText that differ from oldText (or are additions beyond its
    // extent) — decoratable if we keep the editor on newText.
    const changedNewLines: number[] = []
    let addedCount = 0
    let totalChanged = 0

    for (let i = 0; i < maxLen; i++) {
      const oldLine = i < oldLines.length ? oldLines[i] : undefined
      const newLine = i < newLines.length ? newLines[i] : undefined
      if (oldLine === newLine) continue
      totalChanged++
      if (oldLine !== undefined) changedOldLines.push(i + 1)
      if (newLine !== undefined) changedNewLines.push(i + 1)
      if (oldLine === undefined) addedCount++
    }

    // Two display modes:
    //   (A) revert-to-old: editor shows oldText, decorate lines that WILL change.
    //       Good for small edits — Accept is a visible action (text flips to newText).
    //   (B) keep-new:      editor shows newText, decorate lines that ARE the change.
    //       Good for fresh builds — oldText is a 4-line stub, so mode A produces only
    //       a handful of highlights against a "+2388 new" banner, which looks broken.
    // Heuristic: mode B when additions dominate the diff (fresh design / full rewrite).
    // Threshold picked so a 200-line URDF with 50 small edits stays in mode A, while a
    // stub → 2000-line build switches to mode B.
    const useNewTextMode = addedCount > Math.max(50, changedOldLines.length * 5)

    const editorValue = editor.getValue()
    const targetText = useNewTextMode ? newText : oldText
    const textWasChanged = editorValue !== targetText
    if (textWasChanged) {
      editor.setValue(targetText)
    }
    // Render the 3D viewport from the FINAL (post-topology) URDF regardless of
    // which text the editor displays. Without this, revert-to-old mode would
    // leave the viewport showing whatever resolveAssemblyGraph rendered in
    // memory while topology auto-repair changes only land on Accept — making
    // the preview robot drift from what acceptInlineDiff actually commits.
    // Using the explicit URDF override (not editor text) means the preview
    // always matches the to-be-committed state and the line-count label
    // computed below stays consistent with what the user sees.
    const previewUrdf = newUrdf ?? newText
    deps.reparseURDF(previewUrdf)

    const decoratedLines = useNewTextMode ? changedNewLines : changedOldLines
    const decorations: monaco.editor.IModelDeltaDecoration[] = decoratedLines.map(lineNum => ({
      range: new monaco.Range(lineNum, 1, lineNum, 1),
      options: {
        isWholeLine: true,
        className: 'inline-diff-added',
        linesDecorationsClassName: 'inline-diff-gutter-added',
      }
    }))

    if (inlineDiffCollection) inlineDiffCollection.clear()
    // Defer decoration application past the setValue() event-loop tick.
    // Monaco's setValue invalidates any decoration collection created in the
    // same synchronous frame — applying after an rAF means the model is stable
    // by the time our collection is attached. Without this the green gutter
    // and line highlights silently disappear even though the widget bar shows.
    const applyDecorations = () => {
      // Model may have been replaced (tab switch) between setValue and rAF —
      // skip if the pending diff was cleared in that window.
      if (pendingOldText === null) return
      inlineDiffCollection = editor.createDecorationsCollection(decorations)
    }
    if (textWasChanged) {
      requestAnimationFrame(applyDecorations)
    } else {
      applyDecorations()
    }

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

    if (decoratedLines.length > 0) editor.revealLineInCenter(decoratedLines[0])
  }

  function acceptInlineDiff() {
    const editor = (window as any).__vectorEditor as monaco.editor.IStandaloneCodeEditor | undefined
    releaseAiBusyLock(editor)
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
    pendingNewUrdf = undefined
    if (activeFileKey) delete fileStash[activeFileKey]

    const t = deps.getReparseTimeout()
    if (t !== null) { clearTimeout(t); deps.setReparseTimeout(null) }

    // Apply newText now. In revert-to-old mode this is the visible commit step.
    // In keep-new mode the editor already shows newText so this is a no-op —
    // the user still gets visible confirmation via the toast + chat button flip
    // handled just below.
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
    releaseAiBusyLock(editor)
    const oldText = pendingOldText

    clearInlineDiff()
    pendingOldText = null
    pendingNewText = null
    pendingNewUrdf = undefined
    if (activeFileKey) delete fileStash[activeFileKey]

    const t = deps.getReparseTimeout()
    if (t !== null) { clearTimeout(t); deps.setReparseTimeout(null) }

    // Restore oldText in the editor and 3D viewport. In revert-to-old mode
    // showInlineDiff already left oldText in place, so the setValue is a no-op
    // and we just reparse to keep 3D in sync. In keep-new mode the editor
    // holds newText — the getValue check fires and setValue flips it back.
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
    pendingNewUrdf = undefined
  }

  /** Stash any visible diff to the previous file's slot, then rehydrate from
   *  the new file's slot. Idempotent on same-key calls. Pending oldText/
   *  newText/newUrdf and the chat-actions id are preserved across switches.
   *  The Monaco decoration collection and floating bar are recreated on
   *  rehydrate via showInlineDiff(). */
  function setActiveFile(fileKey: string) {
    if (fileKey === activeFileKey) return

    // Stash current visible state under the previous key (if any).
    if (activeFileKey && pendingOldText !== null && pendingNewText !== null) {
      fileStash[activeFileKey] = {
        oldText: pendingOldText,
        newText: pendingNewText,
        newUrdf: pendingNewUrdf,
        activeChatActionsId,
      }
    }

    // Clear visible state — Monaco decorations and the floating bar belong to
    // whichever model the editor is currently showing, and the editor is
    // about to switch (or has just switched) to a different model.
    clearInlineDiff()
    pendingOldText = null
    pendingNewText = null
    pendingNewUrdf = undefined
    activeChatActionsId = null

    activeFileKey = fileKey

    // Rehydrate the new file's stash, if any.
    const stash = fileStash[fileKey]
    if (stash) {
      activeChatActionsId = stash.activeChatActionsId
      // showInlineDiff repopulates pendingOldText/pendingNewText/pendingNewUrdf,
      // re-applies the decoration collection against the now-active model, and
      // re-renders the 3D viewport from the stashed newUrdf.
      showInlineDiff(stash.oldText, stash.newText, stash.newUrdf)
    }
  }

  function removeFile(fileKey: string) {
    delete fileStash[fileKey]
    if (fileKey === activeFileKey) {
      clearInlineDiff()
      pendingOldText = null
      pendingNewText = null
      pendingNewUrdf = undefined
      activeChatActionsId = null
      activeFileKey = ''
    }
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
    setActiveFile,
    removeFile,
  }
}
