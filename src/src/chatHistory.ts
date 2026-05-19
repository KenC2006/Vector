// chatHistory.ts — Chat history management (CRUD, localStorage, UI rendering).
// Storage is keyed per-file: each URDF/XML tab has its own list of chats and
// its own current-chat pointer. Switching the active file swaps the visible
// chat list. The legacy flat list is migrated under a "__legacy__" bucket on
// first load so a user upgrading mid-session doesn't lose history.

export interface ChatMessage {
  role: 'user' | 'assistant' | 'system'
  content: string
  timestamp: number
  urdfSnapshot?: string
}

export interface ChatConversation {
  id: string
  title: string
  createdAt: number
  updatedAt: number
  messages: ChatMessage[]
}

export interface ChatHistoryApi {
  getChatHistory(): ChatConversation[]
  getCurrentChatId(): string
  getCurrentChatMessages(): ChatMessage[]
  getCurrentChat(): ChatConversation | undefined
  startNewChat(): void
  loadChat(id: string): void
  deleteChat(id: string): void
  recordChatMessage(role: 'user' | 'assistant' | 'system', content: string): void
  updateChatDropdown(): void
  rewindChatTo(msgIndex: number, mode: 'conversation' | 'code' | 'both'): void
  attachRewindButton(msgEl: HTMLElement, msgIndex: number): void
  exportForBackend(chatId?: string): Array<{ role: string; content: string }>
  setActiveFile(fileKey: string): void
  getActiveFile(): string
  removeFile(fileKey: string): void
}

const MAX_CHATS_PER_FILE = 20
const STORAGE_KEY = 'vector_chats_v2'
const LEGACY_STORAGE_KEY = 'vector_chats'
const CURRENT_KEY = 'vector_chat_current_v2'
const LEGACY_BUCKET = '__legacy__'
const ORPHAN_BUCKET = '__orphan__'

type ChatsByFile = Record<string, ChatConversation[]>
type CurrentByFile = Record<string, string>

export function escapeHtml(text: string): string {
  return text.replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;')
}

export function initChatHistory(deps: {
  getEditorValue(): string
  setEditorValue(v: string): void
}): ChatHistoryApi {
  // ── Storage migration ──
  // First load after upgrade: if we have a flat list under the legacy key but
  // nothing under the new key, file the flat list under "__legacy__" so it
  // remains accessible (visible only when the legacy bucket is the active
  // file — which never happens through normal UI, but the data is preserved).
  let chatsByFile: ChatsByFile = {}
  try {
    const v2 = localStorage.getItem(STORAGE_KEY)
    if (v2) {
      chatsByFile = JSON.parse(v2)
    } else {
      const legacy = localStorage.getItem(LEGACY_STORAGE_KEY)
      if (legacy) {
        const flat: ChatConversation[] = JSON.parse(legacy)
        if (Array.isArray(flat) && flat.length > 0) {
          chatsByFile[LEGACY_BUCKET] = flat
          console.log(`[chatHistory] Migrated ${flat.length} legacy chat(s) into "${LEGACY_BUCKET}" bucket`)
        }
      }
    }
  } catch (e) {
    console.warn('[chatHistory] Failed to load chat storage:', e)
    chatsByFile = {}
  }

  let currentByFile: CurrentByFile = {}
  try {
    const raw = localStorage.getItem(CURRENT_KEY)
    if (raw) currentByFile = JSON.parse(raw)
  } catch { /* ignore */ }

  // Orphan sweep: untitled buffers (key prefix "untitled:") never survive an
  // app restart — main.ts intentionally skips them in saveOpenTabsState. Any
  // bucket left under such a key in storage is stale. Without this sweep, a
  // freshly created "+" tab can collide with a previous-session untitled key
  // and inherit its chat history. main.ts now also stamps a per-launch tag
  // into new untitled keys so collisions can't happen even mid-sweep, but the
  // sweep still reclaims storage from runs prior to that change.
  {
    let purged = 0
    for (const k of Object.keys(chatsByFile)) {
      if (k.startsWith('untitled:')) {
        delete chatsByFile[k]
        delete currentByFile[k]
        purged++
      }
    }
    if (purged > 0) {
      console.log(`[chatHistory] Swept ${purged} orphan untitled chat bucket(s) from storage`)
      try {
        localStorage.setItem(STORAGE_KEY, JSON.stringify(chatsByFile))
        localStorage.setItem(CURRENT_KEY, JSON.stringify(currentByFile))
      } catch { /* best-effort; persistAll will retry on next write */ }
    }
  }

  // The active file's chats and current-chat-id are mirrored into these
  // working variables so existing call sites that read getChatHistory() etc.
  // see "the chats relevant right now". On setActiveFile() we flush these
  // back into the per-file map and re-hydrate from the new file.
  let activeFileKey = ''
  let chatHistory: ChatConversation[] = []
  let currentChatId = ''
  let currentChatMessages: ChatMessage[] = []

  function generateChatId(): string {
    return `chat_${Date.now()}_${Math.random().toString(36).slice(2, 8)}`
  }

  function flushActiveToMap() {
    if (!activeFileKey) return
    chatsByFile[activeFileKey] = chatHistory
    if (currentChatId) {
      currentByFile[activeFileKey] = currentChatId
    } else {
      delete currentByFile[activeFileKey]
    }
  }

  function persistAll() {
    flushActiveToMap()
    // Cap each bucket at MAX_CHATS_PER_FILE.
    for (const k of Object.keys(chatsByFile)) {
      while (chatsByFile[k].length > MAX_CHATS_PER_FILE) chatsByFile[k].shift()
    }
    const serialized = serializeForStorage(chatsByFile)
    try {
      localStorage.setItem(STORAGE_KEY, serialized)
      localStorage.setItem(CURRENT_KEY, JSON.stringify(currentByFile))
    } catch (e) {
      if (e instanceof DOMException && (e.name === 'QuotaExceededError' || e.code === 22)) {
        // Quota recovery: drop the oldest chat from the largest bucket; retry.
        // Then strip all snapshots if still failing.
        const buckets = Object.entries(chatsByFile)
          .map(([k, v]) => [k, v.length] as [string, number])
          .sort((a, b) => b[1] - a[1])
        if (buckets.length > 0 && buckets[0][1] > 1) {
          chatsByFile[buckets[0][0]].shift()
          try {
            localStorage.setItem(STORAGE_KEY, serializeForStorage(chatsByFile))
            console.warn(`[chatHistory] Quota hit — evicted oldest chat from "${buckets[0][0]}"`)
            return
          } catch { /* fall through */ }
        }
        try {
          localStorage.setItem(STORAGE_KEY, serializeForStorage(chatsByFile, 0))
          console.warn('[chatHistory] Quota hit — persisted form has no URDF snapshots (rewind still works in-session)')
        } catch {
          console.error('[chatHistory] Save failed even after stripping snapshots')
        }
      } else {
        throw e
      }
    }
  }

  /** Per-bucket snapshot strip: cap each chat to N most-recent snapshots. */
  function serializeForStorage(buckets: ChatsByFile, keepSnapshots = 10): string {
    const out: ChatsByFile = {}
    for (const [k, list] of Object.entries(buckets)) {
      out[k] = list.map(chat => ({
        ...chat,
        messages: stripOldSnapshots(chat.messages, keepSnapshots),
      }))
    }
    return JSON.stringify(out)
  }

  function stripOldSnapshots(messages: ChatMessage[], keep: number): ChatMessage[] {
    let kept = 0
    const result: ChatMessage[] = []
    for (let i = messages.length - 1; i >= 0; i--) {
      const m = messages[i]
      if (m.role === 'system') {
        const { urdfSnapshot: _drop, ...rest } = m
        void _drop
        result.unshift(rest as ChatMessage)
        continue
      }
      if (kept < keep && m.urdfSnapshot) {
        result.unshift(m)
        kept++
      } else {
        const { urdfSnapshot: _drop, ...rest } = m
        void _drop
        result.unshift(rest as ChatMessage)
      }
    }
    return result
  }

  function getCurrentChat(): ChatConversation | undefined {
    return chatHistory.find(c => c.id === currentChatId)
  }

  function renderEmptyMessages() {
    const vcMsgs = document.getElementById('vc-messages')
    if (vcMsgs) {
      vcMsgs.innerHTML = `<div class="ai-msg system">
        <div class="ai-msg-content">Describe changes to your robot in natural language. I'll edit the URDF, show you a diff, and highlight changes inline in the editor.</div>
      </div>`
    }
  }

  function updateChatDropdown() {
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

    const label = document.getElementById('vc-chat-dropdown-label')
    const list = document.getElementById('vc-chat-dropdown-list')
    if (!label || !list) return

    const current = chatHistory.find(c => c.id === currentChatId)
    // Prefix with file name so the user can tell at a glance which scope
    // they're chatting in. Falls back to chat title alone if no active file.
    const filePrefix = activeFileKey && activeFileKey !== LEGACY_BUCKET && activeFileKey !== ORPHAN_BUCKET
      ? `${activeFileKey} · `
      : ''
    label.textContent = `${filePrefix}${current?.title || 'New Chat'}`

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

    if (chatId === currentChatId) {
      if (chatHistory.length > 0) {
        loadChat(chatHistory[chatHistory.length - 1].id)
      } else {
        startNewChat()
      }
    } else {
      persistAll()
      updateChatDropdown()
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
    persistAll()
    updateChatDropdown()
    renderEmptyMessages()
  }

  function loadChat(chatId: string) {
    const chat = chatHistory.find(c => c.id === chatId)
    if (!chat) return
    currentChatId = chatId
    currentChatMessages = chat.messages

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
    persistAll()
    updateChatDropdown()
  }

  function recordChatMessage(role: 'user' | 'assistant' | 'system', content: string) {
    const urdfSnapshot = deps.getEditorValue()
    const msg: ChatMessage = { role, content, timestamp: Date.now(), urdfSnapshot }
    currentChatMessages.push(msg)

    const chat = getCurrentChat()
    if (chat) {
      chat.updatedAt = Date.now()
      if (!chat.title || chat.title === 'New Chat') {
        const firstUser = currentChatMessages.find(m => m.role === 'user')
        if (firstUser) chat.title = firstUser.content.slice(0, 50)
      }
      persistAll()
      updateChatDropdown()
    }
  }

  function rewindChatTo(msgIndex: number, mode: 'conversation' | 'code' | 'both') {
    const chat = getCurrentChat()
    if (!chat) return

    const targetMsg = currentChatMessages[msgIndex]
    if (!targetMsg) return

    if (mode === 'code' || mode === 'both') {
      if (targetMsg.urdfSnapshot) {
        deps.setEditorValue(targetMsg.urdfSnapshot)
      }
    }

    if (mode === 'conversation' || mode === 'both') {
      currentChatMessages.length = msgIndex + 1
      chat.messages = currentChatMessages
      chat.updatedAt = Date.now()
      persistAll()
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

  /** Swap the visible chat scope to the given file. Persists current state
   *  back to its bucket, then hydrates working state from the new bucket. If
   *  the new bucket is empty, starts a fresh chat for it. */
  function setActiveFile(fileKey: string) {
    if (fileKey === activeFileKey) return
    flushActiveToMap()
    activeFileKey = fileKey
    chatHistory = chatsByFile[fileKey] ?? []
    chatsByFile[fileKey] = chatHistory
    const savedCurrent = currentByFile[fileKey] || ''
    if (savedCurrent && chatHistory.find(c => c.id === savedCurrent)) {
      currentChatId = savedCurrent
      const chat = chatHistory.find(c => c.id === savedCurrent)!
      currentChatMessages = chat.messages
      // Render the visible chat scrollback.
      loadChat(currentChatId)
    } else if (chatHistory.length > 0) {
      const latest = chatHistory[chatHistory.length - 1]
      currentChatId = latest.id
      currentChatMessages = latest.messages
      loadChat(currentChatId)
    } else {
      currentChatId = ''
      currentChatMessages = []
      startNewChat()
    }
  }

  function getActiveFile(): string {
    return activeFileKey
  }

  /** Remove all chats associated with a deleted/closed file. No-op if there
   *  are none. Does not touch the active-file pointer. */
  function removeFile(fileKey: string) {
    if (!(fileKey in chatsByFile)) return
    delete chatsByFile[fileKey]
    delete currentByFile[fileKey]
    if (fileKey === activeFileKey) {
      activeFileKey = ''
      chatHistory = []
      currentChatId = ''
      currentChatMessages = []
    }
    persistAll()
  }

  function exportForBackend(chatId?: string): Array<{ role: string; content: string }> {
    const id = chatId || currentChatId
    const chat = chatHistory.find(c => c.id === id)
    if (!chat) return []
    return chat.messages
      .filter(m => m.role === 'user' || m.role === 'assistant')
      .map(m => ({ role: m.role, content: m.content }))
  }

  // No file is active at construction time. main.ts calls setActiveFile()
  // once the first file is opened/created. Until then chatHistory is empty.

  return {
    getChatHistory: () => chatHistory,
    getCurrentChatId: () => currentChatId,
    getCurrentChatMessages: () => currentChatMessages,
    getCurrentChat,
    startNewChat,
    loadChat,
    deleteChat,
    recordChatMessage,
    updateChatDropdown,
    rewindChatTo,
    attachRewindButton,
    exportForBackend,
    setActiveFile,
    getActiveFile,
    removeFile,
  }
}
