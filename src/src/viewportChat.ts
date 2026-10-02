// viewportChat.ts — Viewport chat panel: message rendering, AI send, tab switching

import { invoke } from '@tauri-apps/api/core'
import { listen } from '@tauri-apps/api/event'
import { escapeHtml } from './chatHistory'

export interface ViewportChatDeps {
  // Editor
  getEditorValue(): string
  createNewFile(filename?: string, content?: string, diskPath?: string | null): void
  // Chat history
  getCurrentChatId(): string
  getCurrentChatMessages(): Array<{ role: string; content: string; timestamp: number; urdfSnapshot?: string }>
  recordChatMessage(role: 'user' | 'assistant' | 'system', content: string): void
  attachRewindButton(el: HTMLElement, idx: number): void
  loadChat(id: string): void
  startNewChat(): void
  updateChatDropdown(): void
  // Inline diff
  showInlineDiff(oldText: string, newText: string, newUrdf?: string): void
  clearInlineDiff(): void
  setActiveChatActionsId(id: string | null): void
  acceptInlineDiff(): void
  dismissInlineDiff(): void
  /** Frame the camera on the robot (after a brand-new design appears). */
  autoFrameRobot(): void
  // Misc
  showToast(msg: string, type?: 'success' | 'error' | 'warning' | 'info'): void
  SAMPLE_URDF: string
  keysViewportPan: { w: boolean; a: boolean; s: boolean; d: boolean }
  resize(): void
}

export interface ViewportChatApi {
  switchViewportView(view: '3d' | 'chat'): void
  isViewport3D(): boolean
  getVcInput(): HTMLTextAreaElement
  /** Enable or disable the AI Chat viewport tab. When disabled, the tab is
   *  unclickable and the Ctrl+L shortcut is a no-op. If chat is the active
   *  view at the moment of disabling, the viewport switches to 3D first. */
  setChatEnabled(enabled: boolean): void
}

export interface ImageAttachment {
  media_type: 'image/png' | 'image/jpeg'
  data: string // base64, no data: prefix
  // Frontend-only preview (not sent to backend):
  dataUrl?: string
}

const MAX_IMAGES_PER_PROMPT = 3
const MAX_IMAGE_EDGE_PX = 1568 // Anthropic's recommended ceiling; they downscale past this
const IMAGE_JPEG_QUALITY = 0.85

async function compressImageFile(file: File): Promise<ImageAttachment | null> {
  if (!file.type.startsWith('image/')) return null
  const dataUrl = await new Promise<string>((resolve, reject) => {
    const reader = new FileReader()
    reader.onload = () => resolve(reader.result as string)
    reader.onerror = () => reject(reader.error)
    reader.readAsDataURL(file)
  })
  const img = await new Promise<HTMLImageElement>((resolve, reject) => {
    const el = new Image()
    el.onload = () => resolve(el)
    el.onerror = () => reject(new Error('Image decode failed'))
    el.src = dataUrl
  })
  const scale = Math.min(1, MAX_IMAGE_EDGE_PX / Math.max(img.width, img.height))
  const w = Math.max(1, Math.round(img.width * scale))
  const h = Math.max(1, Math.round(img.height * scale))
  const canvas = document.createElement('canvas')
  canvas.width = w
  canvas.height = h
  const ctx = canvas.getContext('2d')
  if (!ctx) return null
  ctx.drawImage(img, 0, 0, w, h)
  const jpegDataUrl = canvas.toDataURL('image/jpeg', IMAGE_JPEG_QUALITY)
  const base64 = jpegDataUrl.split(',', 2)[1] ?? ''
  return { media_type: 'image/jpeg', data: base64, dataUrl: jpegDataUrl }
}

/**
 * Build a Claude-facing summary of which faces are already occupied on each component.
 * Helps the AI avoid shaft-fanout (2+ children on the same servo face) before it's
 * attempted. Flags overloaded faces explicitly so Claude can reason about free ports
 * rather than re-discovering occupancy via the validator.
 */
function computeSimpleDiff(oldText: string, newText: string): { added: string[]; removed: string[] } {
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

export function initViewportChat(deps: ViewportChatDeps): ViewportChatApi {
  const viewportCanvas = document.getElementById('viewport') as HTMLCanvasElement
  const viewportChat = document.getElementById('viewport-chat')!
  const vcMessages = document.getElementById('vc-messages')!
  const vcInput = document.getElementById('vc-input') as HTMLTextAreaElement
  const vcSend = document.getElementById('vc-send') as HTMLButtonElement
  const vcAttachBtn = document.getElementById('vc-attach-btn') as HTMLButtonElement | null
  const vcAttachInput = document.getElementById('vc-attach-input') as HTMLInputElement | null
  const vcAttachThumbs = document.getElementById('vc-attach-thumbs') as HTMLDivElement | null
  const viewportTabs = document.querySelectorAll('.vp-tab')

  let activeViewportView: '3d' | 'chat' = '3d'

  // ── Image attachments (ephemeral — cleared on send, not persisted) ────────
  let attachedImages: ImageAttachment[] = []

  function renderAttachThumbs() {
    if (!vcAttachThumbs) return
    if (attachedImages.length === 0) {
      vcAttachThumbs.classList.add('hidden')
      vcAttachThumbs.innerHTML = ''
      return
    }
    vcAttachThumbs.classList.remove('hidden')
    vcAttachThumbs.innerHTML = ''
    attachedImages.forEach((img, idx) => {
      const wrap = document.createElement('div')
      wrap.className = 'vc-thumb'
      const el = document.createElement('img')
      el.src = img.dataUrl ?? `data:${img.media_type};base64,${img.data}`
      wrap.appendChild(el)
      const remove = document.createElement('button')
      remove.className = 'vc-thumb-remove'
      remove.type = 'button'
      remove.title = 'Remove'
      remove.textContent = '×'
      remove.addEventListener('click', () => {
        attachedImages.splice(idx, 1)
        renderAttachThumbs()
      })
      wrap.appendChild(remove)
      vcAttachThumbs.appendChild(wrap)
    })
  }

  async function addImageFiles(files: File[] | FileList) {
    const list = Array.from(files).filter(f => f.type.startsWith('image/'))
    for (const f of list) {
      if (attachedImages.length >= MAX_IMAGES_PER_PROMPT) {
        deps.showToast(`Max ${MAX_IMAGES_PER_PROMPT} images per prompt`, 'warning')
        break
      }
      try {
        const att = await compressImageFile(f)
        if (att) attachedImages.push(att)
      } catch (err) {
        console.warn('[VC] Image compression failed:', err)
        deps.showToast('Image failed to load', 'error')
      }
    }
    renderAttachThumbs()
  }

  if (vcAttachBtn && vcAttachInput) {
    vcAttachBtn.addEventListener('click', () => vcAttachInput.click())
    vcAttachInput.addEventListener('change', async () => {
      if (vcAttachInput.files && vcAttachInput.files.length > 0) {
        await addImageFiles(vcAttachInput.files)
      }
      vcAttachInput.value = ''
    })
  }

  // Paste images into the chat input
  vcInput.addEventListener('paste', async (e) => {
    const items = e.clipboardData?.items
    if (!items) return
    const imgFiles: File[] = []
    for (const item of Array.from(items)) {
      if (item.kind === 'file' && item.type.startsWith('image/')) {
        const f = item.getAsFile()
        if (f) imgFiles.push(f)
      }
    }
    if (imgFiles.length > 0) {
      e.preventDefault()
      await addImageFiles(imgFiles)
    }
  })

  // Drag-and-drop onto the chat panel or viewport canvas
  const dragTargets = [viewportChat, viewportCanvas]
  for (const target of dragTargets) {
    if (!target) continue
    target.addEventListener('dragover', (e) => {
      if (e.dataTransfer?.types?.includes('Files')) {
        e.preventDefault()
        target.classList.add('vc-dragover')
      }
    })
    target.addEventListener('dragleave', () => target.classList.remove('vc-dragover'))
    target.addEventListener('drop', async (e) => {
      target.classList.remove('vc-dragover')
      if (!e.dataTransfer?.files || e.dataTransfer.files.length === 0) return
      const imgs = Array.from(e.dataTransfer.files).filter(f => f.type.startsWith('image/'))
      if (imgs.length === 0) return
      e.preventDefault()
      // If user drops on the 3D viewport, auto-switch to the chat view so they
      // see the attachment they just added.
      if (target === viewportCanvas) switchViewportView('chat')
      await addImageFiles(imgs)
    })
  }

  // ── Chat UI init ──────────────────────────────────────────────────────────

  if (deps.getCurrentChatMessages().length > 0) {
    deps.loadChat(deps.getCurrentChatId())
  } else {
    vcMessages.innerHTML = `<div class="ai-msg system">
      <div class="ai-msg-content">Describe changes to your robot in natural language. I'll edit the URDF, show you a diff, and highlight changes inline in the editor.</div>
    </div>`
  }
  deps.updateChatDropdown()

  // Wire chat header controls
  const vcChatSelect = document.getElementById('vc-chat-select') as HTMLSelectElement | null
  const vcNewChatBtn = document.getElementById('vc-new-chat') as HTMLButtonElement | null

  vcChatSelect?.addEventListener('change', () => {
    if (vcChatSelect.value && vcChatSelect.value !== deps.getCurrentChatId()) {
      deps.loadChat(vcChatSelect.value)
    }
  })

  vcNewChatBtn?.addEventListener('click', () => deps.startNewChat())

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

  // ── View switching ────────────────────────────────────────────────────────

  let chatEnabled = true
  const chatVpTab = Array.from(viewportTabs).find(
    t => (t as HTMLElement).dataset.view === 'chat',
  ) as HTMLButtonElement | undefined

  function setChatEnabled(enabled: boolean) {
    chatEnabled = enabled
    if (chatVpTab) {
      chatVpTab.disabled = !enabled
      chatVpTab.classList.toggle('disabled', !enabled)
      chatVpTab.title = enabled ? '' : 'Open a file to chat with the AI'
    }
    if (!enabled && activeViewportView === 'chat') {
      switchViewportView('3d')
    }
  }

  function switchViewportView(view: '3d' | 'chat') {
    if (view === 'chat' && !chatEnabled) return
    activeViewportView = view
    if (view !== '3d') {
      deps.keysViewportPan.w = deps.keysViewportPan.a = deps.keysViewportPan.s = deps.keysViewportPan.d = false
    }
    viewportTabs.forEach(tab => {
      tab.classList.toggle('active', (tab as HTMLElement).dataset.view === view)
    })
    if (view === '3d') {
      viewportCanvas.style.display = ''
      viewportChat.classList.add('hidden')
      document.getElementById('viewport-info')!.style.display = ''
      deps.resize()
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

  // ── Message rendering ──────────────────────────────────────────────────────

  function addVCMessage(role: 'user' | 'assistant' | 'system', content: string, extras?: {
    diff?: { added: string[]; removed: string[] }
    newUrdf?: string
    images?: ImageAttachment[]
  }) {
    const plainContent = content.replace(/<[^>]*>/g, ' ').replace(/\s+/g, ' ').trim()
    if (plainContent) deps.recordChatMessage(role, plainContent)

    const msg = document.createElement('div')
    msg.className = `ai-msg ${role}`

    if (role === 'user') {
      let html = `<div class="ai-msg-content">${escapeHtml(content)}</div>`
      if (extras?.images && extras.images.length > 0) {
        html += `<div class="ai-msg-images">`
        for (const img of extras.images) {
          const src = img.dataUrl ?? `data:${img.media_type};base64,${img.data}`
          html += `<img src="${src}" alt="attached image">`
        }
        html += `</div>`
      }
      msg.innerHTML = html
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
        deps.setActiveChatActionsId(msgId)
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
            setAiBusy(false)
            try {
              deps.acceptInlineDiff()
              acceptBtn.textContent = '✓ Applied'
              acceptBtn.className = 'ai-applied'
              rejectBtn.style.display = 'none'
              // Chat covers the viewport: show the robot that was just applied.
              ;(document.querySelector('.vp-tab[data-view="3d"]') as HTMLElement | null)?.click()
            } finally {
              setAiBusy(false)
              deps.setActiveChatActionsId(null)
            }
          })

          rejectBtn.addEventListener('click', () => {
            setAiBusy(false)
            try {
              deps.dismissInlineDiff()
            rejectBtn.textContent = '✗ Dismissed'
              rejectBtn.className = 'ai-rejected'
              acceptBtn.style.display = 'none'
            } finally {
              setAiBusy(false)
              deps.setActiveChatActionsId(null)
            }
          })
        }, 0)
      } else {
        msg.innerHTML = html
      }
    } else {
      msg.innerHTML = `<div class="ai-msg-content">${content}</div>`
    }

    if (role === 'user' || role === 'assistant') {
      deps.attachRewindButton(msg, deps.getCurrentChatMessages().length - 1)
    }

    vcMessages.appendChild(msg)
    vcMessages.scrollTop = vcMessages.scrollHeight
    return msg
  }

  function addVCThinking(onCancel?: () => void): HTMLElement & { updateStage: (stage: string, text: string) => void } {
    const msg = document.createElement('div') as unknown as HTMLElement & { updateStage: (stage: string, text: string) => void }
    msg.className = 'ai-msg assistant'
    msg.innerHTML = `<div class="ai-thinking">
      <span class="dot"></span><span class="dot"></span><span class="dot"></span>
      <span class="ai-thinking-text">Thinking...</span>
      <button type="button" class="ai-thinking-cancel" title="Cancel generation">✕ Cancel</button>
    </div>
    <div class="ai-streaming-preview" style="display:none"></div>`
    vcMessages.appendChild(msg)
    const cancelBtn = msg.querySelector('.ai-thinking-cancel') as HTMLButtonElement | null
    if (cancelBtn) {
      if (onCancel) cancelBtn.addEventListener('click', onCancel)
      else cancelBtn.style.display = 'none'
    }
    vcMessages.scrollTop = vcMessages.scrollHeight

    const stageLabels: Record<string, string> = {
      thinking: 'Analyzing model...', generating: 'Generating design...',
      streaming: '', applying: 'Applying changes...', done: 'Done',
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

  /** Designer summaries are light markdown: bold, bullets, line breaks. */
  function formatDesignSummary(text: string): string {
    return escapeHtml(text.trim())
      .replace(/\*\*(.+?)\*\*/g, '<b>$1</b>')
      .replace(/^- /gm, '• ')
      .replace(/\n/g, '<br>')
  }

  // ── AI send ───────────────────────────────────────────────────────────────

  function setAiBusy(busy: boolean) {
    vcSend.disabled = busy
    vcInput.disabled = busy
    if (vcNewChatBtn) vcNewChatBtn.disabled = busy
    const dropBtn = document.getElementById('vc-chat-dropdown-btn') as HTMLButtonElement | null
    if (dropBtn) dropBtn.disabled = busy
    if (busy) {
      // Close the chat-switcher dropdown if it was open when generation started.
      document.getElementById('vc-chat-dropdown-list')?.classList.add('hidden')
    }
    // Flip viewport mode synchronously BEFORE toggling the class, so the
    // gizmo/mount-nodes are torn down before any AI tool call can write a new
    // URDF. The class toggle still fires the MutationObserver as a backstop
    // for the inlineDiff path that bypasses setAiBusy.
    const setAiBusyMode = (window as unknown as { __setAiBusyMode?: (busy: boolean) => void }).__setAiBusyMode
    setAiBusyMode?.(busy)
    document.body.classList.toggle('ai-busy', busy)
    const editor = (window as any).__vectorEditor as
      | { updateOptions(opts: { readOnly: boolean }): void } | undefined
    editor?.updateOptions({ readOnly: busy })
  }

  // Cancellation token shared by the active generation. The cancel button on
  // the thinking bubble flips `cancelled = true`; awaits in sendVCMessage and
  // runToolCallEditLoop check this and bail before mutating UI/state.
  type CancelToken = { cancelled: boolean; awaitingDecision: boolean }
  let activeCancel: CancelToken | null = null
  let coreRestartAfterCancel: Promise<void> | null = null

  function scheduleCoreRestartAfterCancel() {
    coreRestartAfterCancel = invoke('cancel_core_request')
      .catch(err => {
        console.warn('[VC] Failed to cancel Python core request:', err)
      })
      .then(() => invoke('start_core'))
      .catch(err => {
        const msg = String(err).toLowerCase()
        if (!msg.includes('already running') && !msg.includes('already started')) {
          console.warn('[VC] Failed to restart Python core after cancel:', err)
        }
      })
      .then(() => undefined)
      .finally(() => {
        coreRestartAfterCancel = null
      })
  }

  function cancelActiveGeneration(thinking: HTMLElement | null) {
    if (!activeCancel) return
    activeCancel.cancelled = true
    activeCancel = null
    thinking?.remove()
    setAiBusy(false)
    addVCMessage('system', '<span style="color:#e5c07b;">Generation cancelled.</span>')
    scheduleCoreRestartAfterCancel()
  }

  async function sendVCMessage(
    prompt: string,
    retryCount = 0,
    imagesOverride?: ImageAttachment[],
    /** Editor URDF at the time the user sent the prompt (kept across
     *  rate-limit retries so the diff baseline is what the user saw). */
    originalUrdfOverride?: string,
  ) {
    if (!prompt.trim()) return
    if (!deps.getEditorValue()) {
      deps.createNewFile('robot.urdf', deps.SAMPLE_URDF, null)
    }

    // First turn: snapshot the attachments so retries re-send the same images.
    const imagesForThisSend: ImageAttachment[] = imagesOverride ?? attachedImages.slice()
    if (retryCount === 0) {
      addVCMessage('user', prompt, imagesForThisSend.length > 0 ? { images: imagesForThisSend } : undefined)
      vcInput.value = ''
      vcInput.style.height = 'auto'
      attachedImages = []
      renderAttachThumbs()
    }

    setAiBusy(true)
    // Reuse the parent's token across retries so one click on Cancel kills the chain.
    const isOutermost = activeCancel === null
    const cancelToken: CancelToken = isOutermost
      ? { cancelled: false, awaitingDecision: false }
      : activeCancel!
    if (isOutermost) activeCancel = cancelToken
    const thinking = addVCThinking(() => cancelActiveGeneration(thinking))

    let unlisten: (() => void) | null = null
    try {
      unlisten = await listen<{ stage: string; text: string }>('ai_progress', (event) => {
        thinking.updateStage(event.payload.stage, event.payload.text)
      })
    } catch {
      // listen may fail in dev mode without Tauri — non-critical
    }

    const fullUrdf = originalUrdfOverride ?? deps.getEditorValue()

    try {
      if (coreRestartAfterCancel) {
        await coreRestartAfterCancel
        if (cancelToken.cancelled) return
      }

      // The designer handles every request: a fresh robot from an empty file,
      // or an edit of the current one (its design is embedded in the URDF, or
      // rebuilt from the URDF when it was written or edited by hand). One
      // backend call runs design -> build -> geometry critique -> revise.
      const out = await invoke('ai_design', {
        prompt,
        urdfContent: fullUrdf,
        images: imagesForThisSend.map(({ media_type, data }) => ({ media_type, data })),
      }) as { urdf: string; summary: string; issues: string[]; rounds: number }
      thinking.remove()
      if (cancelToken.cancelled) return
      const notes = out.issues.length > 0
        ? `<div style="color:#e5c07b;font-size:11px;margin-top:6px">${out.issues.length} geometry note(s): ${escapeHtml(out.issues.join(' '))}</div>`
        : ''
      const diff = computeSimpleDiff(fullUrdf, out.urdf)
      addVCMessage('assistant', `${formatDesignSummary(out.summary)}${notes}`, { diff, newUrdf: out.urdf })
      deps.showInlineDiff(fullUrdf, out.urdf, out.urdf)
      if ((fullUrdf.match(/<link/g) || []).length <= 1) setTimeout(() => deps.autoFrameRobot(), 300)
      cancelToken.awaitingDecision = true
    } catch (err) {
      thinking.remove()
      if (cancelToken.cancelled) return
      const errStr = String(err)
      console.warn('[VC] Backend error:', err)
      if (errStr.includes('429') || errStr.includes('rate_limit')) {
        if (retryCount < 2) {
          const waitSec = (retryCount + 1) * 5
          addVCMessage('system', `<span style="color:#e5c07b;">Rate limited. Retrying in ${waitSec}s...</span>`)
          await new Promise(r => setTimeout(r, waitSec * 1000))
          unlisten?.()
          unlisten = null
          return await sendVCMessage(prompt, retryCount + 1, imagesForThisSend, fullUrdf)
        }
        addVCMessage('assistant', `<span style="color:#f85149;">Rate limited after ${retryCount + 1} attempts. Please wait a moment and try again.</span>`)
      } else {
        addVCMessage('assistant', `<span style="color:#f85149;">Error: ${escapeHtml(errStr.slice(0, 300))}</span>`)
      }
    } finally {
      unlisten?.()
      if (isOutermost) {
        if (activeCancel === cancelToken) activeCancel = null
        // A pending diff keeps the lockout until Apply/Dismiss releases it.
        if (!cancelToken.awaitingDecision && (activeCancel === null || activeCancel === cancelToken)) {
          setAiBusy(false)
        }
      }
    }
  }

  // ── Input handlers ────────────────────────────────────────────────────────

  vcInput.addEventListener('keydown', (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault()
      if (vcSend.disabled) return
      sendVCMessage(vcInput.value)
    }
  })

  vcSend.addEventListener('click', () => sendVCMessage(vcInput.value))

  vcInput.addEventListener('input', () => {
    vcInput.style.height = 'auto'
    vcInput.style.height = Math.min(vcInput.scrollHeight, 120) + 'px'
  })

  // Ctrl+L switches to chat view
  document.addEventListener('keydown', (e) => {
    if (e.ctrlKey && e.key === 'l') {
      e.preventDefault()
      switchViewportView('chat')
      vcInput.focus()
    }
  })

  return {
    switchViewportView,
    isViewport3D: () => activeViewportView === '3d',
    getVcInput: () => vcInput,
    setChatEnabled,
  }
}
