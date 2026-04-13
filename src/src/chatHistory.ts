// chatHistory.ts — Chat history management (CRUD, localStorage, UI rendering)

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
}

const MAX_CHATS = 20

export function escapeHtml(text: string): string {
  return text.replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;')
}

export function initChatHistory(deps: {
  getEditorValue(): string
  setEditorValue(v: string): void
}): ChatHistoryApi {
  let chatHistory: ChatConversation[] = JSON.parse(localStorage.getItem('vector_chats') || '[]')
  let currentChatId = ''
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
      saveChatHistory()
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

  // Initialize: load most recent chat or create new one
  if (chatHistory.length > 0) {
    const latest = chatHistory[chatHistory.length - 1]
    currentChatId = latest.id
    currentChatMessages = latest.messages
  } else {
    startNewChat()
  }

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
  }
}
