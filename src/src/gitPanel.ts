// ── Git Source Control Panel ────────────────────────────────────────────────

interface GitStatus {
  staged: Array<{ path: string; status: string }>
  unstaged: Array<{ path: string; status: string }>
}

export function initGitPanel(deps: {
  invoke: (cmd: string, args?: Record<string, unknown>) => Promise<unknown>
  showToast: (msg: string, type?: 'success' | 'warning' | 'error' | 'info') => void
}): {
  refreshGitStatus: () => Promise<void>
} {
  const { invoke, showToast } = deps

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

  // ── Event listeners ──

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

  return { refreshGitStatus }
}
