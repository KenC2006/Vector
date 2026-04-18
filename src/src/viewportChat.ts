// viewportChat.ts — Viewport chat panel: message rendering, AI send, tab switching

import * as THREE from 'three'
import { invoke } from '@tauri-apps/api/core'
import { listen } from '@tauri-apps/api/event'
import { escapeHtml } from './chatHistory'
import type { UrdfAssemblyApi, TopologyOp, AssemblyGraph } from './urdfAssembly'

export interface ViewportChatDeps {
  // Editor
  getEditorValue(): string
  createNewFile(filename?: string, content?: string, diskPath?: string | null): void
  // AI context
  buildKinematicContext(): string
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
  // URDF
  reparseURDF(xml?: string): void
  runLocalValidation(): void
  createCheckpoint(label: string, urdf: string, auto: boolean): void
  // Three.js (for screenshots)
  scene: THREE.Scene
  robot: THREE.Group
  camera: THREE.PerspectiveCamera
  groundRobot(): void
  autoFrameRobot(): void
  // Chat history
  exportForBackend(chatId?: string): Array<{ role: string; content: string }>
  // Misc
  getUrdfAssemblyApi(): UrdfAssemblyApi | null
  getCoreAvailable(): boolean
  showToast(msg: string, type?: 'success' | 'error' | 'warning' | 'info'): void
  SAMPLE_URDF: string
  keysViewportPan: { w: boolean; a: boolean; s: boolean; d: boolean }
  resize(): void
}

export interface ViewportChatApi {
  switchViewportView(view: '3d' | 'chat'): void
  isViewport3D(): boolean
  getVcInput(): HTMLTextAreaElement
}

/**
 * Build a Claude-facing summary of which faces are already occupied on each component.
 * Helps the AI avoid shaft-fanout (2+ children on the same servo face) before it's
 * attempted. Flags overloaded faces explicitly so Claude can reason about free ports
 * rather than re-discovering occupancy via the validator.
 */
function buildPortOccupancyContext(graph: AssemblyGraph): string {
  const allFaces = ['top', 'bottom', 'front', 'back', 'left', 'right']
  // parent link_name → face → child link_names attached there
  const usage = new Map<string, Map<string, string[]>>()
  for (const comp of graph.components) {
    if (!comp.attach_to || !comp.attach_face) continue
    let byFace = usage.get(comp.attach_to)
    if (!byFace) { byFace = new Map(); usage.set(comp.attach_to, byFace) }
    const bucket = byFace.get(comp.attach_face) || []
    bucket.push(comp.link_name)
    byFace.set(comp.attach_face, bucket)
  }
  const lines: string[] = []
  for (const comp of graph.components) {
    const byFace = usage.get(comp.link_name)
    if (!byFace || byFace.size === 0) continue
    const takenEntries: string[] = []
    const overloaded: string[] = []
    for (const [face, children] of byFace) {
      takenEntries.push(`${face}=[${children.join(', ')}]`)
      if (children.length > 1) overloaded.push(face)
    }
    const free = allFaces.filter(f => !byFace.has(f))
    const warn = overloaded.length > 0 ? `  ⚠ OVERLOADED faces (must reparent extras to a structural link): ${overloaded.join(', ')}` : ''
    lines.push(`  - ${comp.link_name} (${comp.component_id}): taken ${takenEntries.join(', ')}; free [${free.join(', ')}]${warn}`)
  }
  if (lines.length === 0) return ''
  return `\n\nPort occupancy (existing children per face). Rules:
- Servos/motors have a shaft output on TOP; attach exactly ONE child to a servo's top face.
- Do NOT attach a second child to an already-taken face of an actuator — reparent to a structural link (extrusion, bracket) instead.
${lines.join('\n')}`
}

/**
 * Build a "I'll change X" banner that names the resolved targets a set of topology
 * operations will affect — Cursor-style echo-before-apply (master plan W2). Surfaced
 * alongside the Apply/Dismiss buttons so the user can sanity-check the resolution
 * before committing (e.g. "move wrist camera" → sensor_depth_camera_small_1).
 */
function summarizeTopologyOps(ops: TopologyOp[], currentGraph: AssemblyGraph): string {
  const byName = new Map(currentGraph.components.map(c => [c.link_name, c]))
  const rows: string[] = []
  for (const op of ops) {
    if (op.op === 'add') {
      const parent = op.attach_to || '(root)'
      rows.push(`<b>add</b> <code>${escapeHtml(op.link_name)}</code> (${escapeHtml(op.component_id || '?')}) on <code>${escapeHtml(parent)}:${escapeHtml(op.attach_face || '?')}</code>`)
    } else if (op.op === 'remove') {
      const existing = byName.get(op.link_name)
      const tag = existing ? ` (${escapeHtml(existing.component_id)})` : ''
      rows.push(`<b>remove</b> <code>${escapeHtml(op.link_name)}</code>${tag} and its subtree`)
    } else if (op.op === 'modify') {
      const existing = byName.get(op.link_name)
      const tag = existing ? ` (${escapeHtml(existing.component_id)})` : ''
      const changes: string[] = []
      if (op.attach_to !== undefined) changes.push(`parent→${escapeHtml(String(op.attach_to))}`)
      if (op.attach_face !== undefined) changes.push(`face→${escapeHtml(op.attach_face)}`)
      if (op.component_id !== undefined) changes.push(`component_id→${escapeHtml(op.component_id)}`)
      if (op.joint_type !== undefined) changes.push(`joint_type→${escapeHtml(op.joint_type)}`)
      if (op.joint_axis !== undefined) changes.push(`joint_axis→${escapeHtml(op.joint_axis)}`)
      if (op.length_mm !== undefined) changes.push(`length_mm→${op.length_mm}`)
      if (op.orientation !== undefined) changes.push(`orientation→${escapeHtml(op.orientation)}`)
      if (op.elevation_angle !== undefined) changes.push(`elevation_angle→${op.elevation_angle}°`)
      const changeText = changes.length > 0 ? changes.join(', ') : '(no field changes)'
      rows.push(`<b>modify</b> <code>${escapeHtml(op.link_name)}</code>${tag}: ${changeText}`)
    }
  }
  if (rows.length === 0) return ''
  return `<div class="ai-resolved-targets" style="margin:6px 0;padding:8px 10px;border-left:3px solid #4d78cc;background:#1b1f2b;border-radius:3px;font-size:12px;line-height:1.6;">
    <div style="color:#9aa7c2;font-weight:600;margin-bottom:4px;">I'll change:</div>
    ${rows.map(r => `<div>• ${r}</div>`).join('')}
  </div>`
}

/**
 * One-line summary of a fresh assembly graph (design_robot path) — helps the user
 * catch wrong-shape outputs before applying (e.g. "expected an arm, got a rover").
 */
function summarizeAssemblyGraph(graph: AssemblyGraph): string {
  const comps = graph.components
  if (comps.length === 0) return ''
  const dof = comps.filter(c => c.joint_type === 'revolute' || c.joint_type === 'prismatic').length
  const sensors = comps.filter(c => c.component_id.startsWith('sensor_')).length
  const actuators = comps.filter(c => c.component_id.startsWith('actuator_') || c.component_id.startsWith('motor_')).length
  const structural = comps.filter(c => c.component_id.startsWith('structural_')).length
  const wheels = comps.filter(c => c.component_id.includes('wheel') || c.component_id.includes('caster')).length
  const effectors = comps.filter(c => c.component_id.includes('gripper') || c.component_id.includes('effector') || c.component_id.includes('suction')).length
  const parts: string[] = [`${comps.length} components`, `${dof} DOF`]
  if (actuators > 0) parts.push(`${actuators} actuator${actuators > 1 ? 's' : ''}`)
  if (structural > 0) parts.push(`${structural} structural`)
  if (sensors > 0) parts.push(`${sensors} sensor${sensors > 1 ? 's' : ''}`)
  if (wheels > 0) parts.push(`${wheels} wheel${wheels > 1 ? 's' : ''}`)
  if (effectors > 0) parts.push(`${effectors} effector${effectors > 1 ? 's' : ''}`)
  return `<div class="ai-resolved-targets" style="margin:6px 0;padding:8px 10px;border-left:3px solid #4d78cc;background:#1b1f2b;border-radius:3px;font-size:12px;">
    <div style="color:#9aa7c2;font-weight:600;margin-bottom:2px;">I'll build:</div>
    <div>${parts.join(', ')}  •  base=<code>${escapeHtml(graph.base_link)}</code></div>
  </div>`
}

/**
 * Compact plain-text AssemblyGraph summary for the AI context (Phase 4).
 *
 * Unlike summarizeAssemblyGraph (HTML, user-facing), this serializes the
 * component tree with the exact fields Claude needs to reason about edits:
 * component_id, link_name, attach_to, attach_face, joint_type/axis, and
 * non-zero attach_rpy. Cheaper than re-parsing URDF and lossless on
 * orientation/elevation_angle/length_mm fields that URDF round-trips drop.
 */
function summarizeAssemblyGraphForAI(graph: AssemblyGraph): string {
  if (!graph.components || graph.components.length === 0) return ''
  const lines: string[] = ['## Current AssemblyGraph (structured)', `base_link: ${graph.base_link}`]
  if (graph.ground_offset) lines.push('ground_offset: true')
  lines.push(`components (${graph.components.length}):`)
  for (const c of graph.components) {
    const parts: string[] = [`  - ${c.link_name}: ${c.component_id}`]
    // Match resolveAssemblyGraph's !c.attach_to root predicate (urdfAssembly.ts:3158)
    // so this view never disagrees with what the engine actually built.
    if (!c.attach_to) {
      parts.push('(root)')
    } else {
      parts.push(`attach_to=${c.attach_to}`)
      if (c.attach_face) parts.push(`face=${c.attach_face}`)
    }
    if (c.joint_type) parts.push(`joint=${c.joint_type}`)
    if (c.joint_axis) parts.push(`axis=${c.joint_axis}`)
    if (c.attach_rpy && c.attach_rpy.some(v => Math.abs(v) > 0.001)) {
      parts.push(`attach_rpy=[${c.attach_rpy.map(v => v.toFixed(3)).join(', ')}]`)
    }
    if (c.length_mm) parts.push(`length_mm=${c.length_mm}`)
    if (c.orientation) parts.push(`orientation="${c.orientation}"`)
    if (typeof c.elevation_angle === 'number') parts.push(`elevation_angle=${c.elevation_angle}`)
    lines.push(parts.join(', '))
  }
  return lines.join('\n') + '\n\n'
}

/** Render topology warnings as an inline amber block under the assistant message. */
function formatWarningsHtml(warnings: string[] | undefined): string {
  if (!warnings || warnings.length === 0) return ''
  const items = warnings.map(w => `<div>${escapeHtml(w)}</div>`).join('')
  return `<div style="margin-top:8px;padding:6px 8px;border-left:3px solid #e5c07b;color:#e5c07b;font-size:11px;background:rgba(229,192,123,0.08);">Topology warnings (non-blocking):${items}</div>`
}

/** Render topology warnings as a plain-text block for redesign/retry prompts. */
function formatWarningsForPrompt(warnings: string[] | undefined): string {
  if (!warnings || warnings.length === 0) return ''
  return `\n\nTopology warnings (non-blocking, but worth addressing in a redesign):\n${warnings.map(w => `- ${w}`).join('\n')}`
}

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
  const viewportTabs = document.querySelectorAll('.vp-tab')

  let activeViewportView: '3d' | 'chat' = '3d'

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

  function switchViewportView(view: '3d' | 'chat') {
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
  }) {
    const plainContent = content.replace(/<[^>]*>/g, ' ').replace(/\s+/g, ' ').trim()
    if (plainContent) deps.recordChatMessage(role, plainContent)

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
            deps.setActiveChatActionsId(null)
            deps.acceptInlineDiff()
            acceptBtn.textContent = '✓ Applied'
            acceptBtn.className = 'ai-applied'
            rejectBtn.style.display = 'none'
          })

          rejectBtn.addEventListener('click', () => {
            deps.setActiveChatActionsId(null)
            deps.dismissInlineDiff()
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
      deps.attachRewindButton(msg, deps.getCurrentChatMessages().length - 1)
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

  // ── AI send ───────────────────────────────────────────────────────────────

  async function sendVCMessage(prompt: string, retryCount = 0) {
    if (!prompt.trim()) return

    if (!deps.getEditorValue()) {
      deps.createNewFile('robot.urdf', deps.SAMPLE_URDF, null)
    }

    if (retryCount === 0) {
      // Resync conversation history to backend BEFORE recording the new message,
      // so the current prompt isn't duplicated (generate_edit adds it separately).
      const chatId = deps.getCurrentChatId()
      const history = deps.exportForBackend(chatId)
      if (history.length > 0) {
        try {
          await invoke('ai_set_history', { sessionId: chatId, history })
          console.log(`[VC] Resynced ${history.length} messages for session ${chatId}`)
        } catch (err) {
          console.warn('[VC] History resync failed (non-critical):', err)
        }
      }

      addVCMessage('user', prompt)
      vcInput.value = ''
      vcInput.style.height = 'auto'
    }

    vcSend.disabled = true
    const thinking = addVCThinking()

    let unlisten: (() => void) | null = null
    try {
      unlisten = await listen<{ stage: string; text: string }>('ai_progress', (event) => {
        thinking.updateStage(event.payload.stage, event.payload.text)
      })
    } catch {
      // listen may fail in dev mode without Tauri — non-critical
    }

    try {

      const fullUrdf = deps.getEditorValue()
      const kinematicContext = deps.buildKinematicContext()
      const isRedesign = retryCount > 0
      const currentUrdf = isRedesign
        ? '<?xml version="1.0"?><robot name="redesign"><link name="base_link"/></robot>'
        : fullUrdf
      if (isRedesign) {
        console.log(`[AI][redesign] Sending minimal URDF for redesign (skipping ${fullUrdf.length} char URDF)`)
      }

      // Augment context with per-component port occupancy so Claude can avoid
      // shaft-fanout before attempting it (master plan W2). Also include the
      // structured AssemblyGraph (Phase 4) so Claude reasons about explicit
      // component_id / attach_face / attach_rpy fields without re-deriving them
      // from URDF text — and sees fields URDF round-trips can't preserve
      // (orientation, elevation_angle, length_mm).
      // Only meaningful for edit/modify_topology calls — redesigns start blank.
      const storedGraphForContext = deps.getUrdfAssemblyApi()?.getLastAssemblyGraph() || null
      const portOccupancyCtx = !isRedesign && storedGraphForContext
        ? buildPortOccupancyContext(storedGraphForContext)
        : ''
      const graphSummaryCtx = !isRedesign && storedGraphForContext
        ? summarizeAssemblyGraphForAI(storedGraphForContext)
        : ''
      const augmentedContext = isRedesign ? '' : (kinematicContext + portOccupancyCtx + graphSummaryCtx)

      const result = await invoke('ai_edit', {
        prompt,
        urdfContent: currentUrdf,
        kinematicContext: augmentedContext,
        sessionId: deps.getCurrentChatId(),
      }) as { explanation: string; new_urdf: string; stats: string; assembly_graph?: unknown; topology_ops?: TopologyOp[] }

      thinking.remove()

      const urdfAssemblyApi = deps.getUrdfAssemblyApi()

      // ── modify_topology path: parse current URDF → apply ops → re-resolve ──
      if (result.topology_ops && result.topology_ops.length > 0 && urdfAssemblyApi) {
        console.log(`[AI] Received ${result.topology_ops.length} topology operations — applying to current assembly`)
        // Prefer stored graph (exact, no round-trip loss) over reverse-parsing (lossy fallback)
        const storedGraph = urdfAssemblyApi.getLastAssemblyGraph()
        const currentGraph = storedGraph || urdfAssemblyApi.urdfToAssemblyGraph(fullUrdf)
        if (currentGraph && !storedGraph) {
          console.warn('[AI] Using lossy reverse-parsed graph — stored graph not available')
        }
        if (!currentGraph) {
          addVCMessage('assistant', `<span style="color:#f85149;">Could not parse current URDF for topology editing. Try "start over" to redesign from scratch.</span>`)
        } else {
          const modifiedGraph = urdfAssemblyApi.applyTopologyOps(currentGraph, result.topology_ops)
          console.log(`[AI] Modified graph: ${modifiedGraph.components.length} components (was ${currentGraph.components.length})`)
          const assemblyOut = urdfAssemblyApi.resolveAssemblyGraph(modifiedGraph)
          if (assemblyOut.urdf) {
            // Ground and frame the modified robot (same as assembly_graph path)
            await new Promise(r => setTimeout(r, 400))
            deps.groundRobot()
            deps.autoFrameRobot()

            const diff = computeSimpleDiff(fullUrdf, assemblyOut.urdf)
            const resolvedTargets = summarizeTopologyOps(result.topology_ops, currentGraph)
            const warnBlock = formatWarningsHtml(assemblyOut.topologyWarnings)
            addVCMessage('assistant', `${resolvedTargets}${result.explanation}${warnBlock}<br><span style="color:#858585;font-size:11px">${result.stats}</span>`, {
              diff, newUrdf: assemblyOut.urdf,
            })
            deps.showInlineDiff(fullUrdf, assemblyOut.urdf, assemblyOut.urdf)
          } else {
            const errors = assemblyOut.topologyErrors?.join(', ') || 'unknown error'
            addVCMessage('assistant', `<span style="color:#f85149;">Topology modification failed: ${escapeHtml(errors)}</span>`)
          }
        }
      } else if (result.assembly_graph && urdfAssemblyApi) {
        console.log('[AI] Received assembly_graph — resolving via frontend snap system')
        const assemblyOut = urdfAssemblyApi.resolveAssemblyGraph(result.assembly_graph as import('./urdfAssembly').AssemblyGraph)
        let assemblyResult = assemblyOut.urdf
        console.log(`[AI] Assembly result: urdf=${assemblyResult ? `${assemblyResult.length} chars` : 'null'}, topologyErrors=${JSON.stringify(assemblyOut.topologyErrors || [])}`)

        if (assemblyResult) {
          try {
            console.log('[AI] Running 2nd-pass AI validation with visual feedback...')
            await new Promise(r => setTimeout(r, 800))

            deps.groundRobot()
            deps.autoFrameRobot()

            const robotBox = new THREE.Box3().setFromObject(deps.robot)
            if (robotBox.isEmpty()) throw new Error('Robot bounding box is empty — meshes may not have loaded')
            const robotCenter = new THREE.Vector3()
            const robotSize = new THREE.Vector3()
            robotBox.getCenter(robotCenter)
            robotBox.getSize(robotSize)
            const maxDim = Math.max(robotSize.x, robotSize.y, robotSize.z, 0.3)
            const captureSize = 768    // VLMs downscale to ~768px tiles; matches their native resolution

            // Three canonical views with per-view distance for optimal framing
            const viewAngles = [
              { label: 'side-low',      az: Math.PI * 0.05, el: Math.PI * 0.06, dist: maxDim * 0.95 },  // ~11° elevation — profile/grounding
              { label: 'three-quarter', az: Math.PI * 0.30, el: Math.PI * 0.10, dist: maxDim * 0.85 },  // ~18° elevation, ~54° azimuth — low 3/4 like standing nearby
              { label: 'overhead',      az: Math.PI * -0.15, el: Math.PI * 0.35, dist: maxDim * 0.85 },  // ~63° elevation — top-down layout
            ]
            const screenshots: string[] = []

            // ── Prepare clean scene for capture ──
            // Hide everything except the robot meshes and lights.
            // Strategy: hide all scene children except the robot group and lights,
            // then inside the robot hide any debug/overlay groups.
            const hiddenObjects: THREE.Object3D[] = []

            // Hide top-level scene objects that aren't the robot or lights
            for (const child of deps.scene.children) {
              if (!child.visible) continue
              if (child === deps.robot) continue
              if (child instanceof THREE.Light) continue
              child.visible = false
              hiddenObjects.push(child)
            }

            // Hide debug overlays inside the robot group (wireframe, CoM, axis visuals, etc.)
            deps.robot.traverse(obj => {
              if (!obj.visible) return
              const dominated =
                obj instanceof THREE.AxesHelper
                || obj instanceof THREE.ArrowHelper
                || obj.name === 'attachment_nodes'
                || obj.name === 'attachment_node_rings'
                || obj.name === 'node-axis-rings'
                || obj.type === 'Line'
                || obj.type === 'LineLoop'
                || obj.type === 'LineSegments'
                // BoxGeometry node meshes (12×12×12mm cubes used for mount nodes)
                || (obj instanceof THREE.Mesh && (obj.geometry as any)?.parameters?.width === 0.012)
                // TorusGeometry axis rings
                || (obj instanceof THREE.Mesh && obj.geometry instanceof THREE.TorusGeometry)
              if (dominated) {
                obj.visible = false
                hiddenObjects.push(obj)
              }
            })

            // Swap to light gray background for better contrast (VLMs parse light BGs better)
            const origBackground = deps.scene.background
            deps.scene.background = new THREE.Color(0xd8dce3)

            // Add a temporary fill light to reduce harsh shadows in captures
            const captureFill = new THREE.HemisphereLight(0xffffff, 0x8899aa, 0.5)
            deps.scene.add(captureFill)

            // Visible ground plane so the floor line is clear in low-angle shots
            const captureGround = new THREE.Mesh(
              new THREE.PlaneGeometry(6, 6),
              new THREE.MeshStandardMaterial({ color: 0xbcc0c8, roughness: 0.9 }),
            )
            captureGround.rotation.x = -Math.PI / 2
            captureGround.position.y = 0.0001  // just above origin to avoid z-fighting
            captureGround.receiveShadow = true
            deps.scene.add(captureGround)

            const offCanvas = document.createElement('canvas')
            offCanvas.width = captureSize
            offCanvas.height = captureSize
            const offRenderer = new THREE.WebGLRenderer({ canvas: offCanvas, antialias: true, preserveDrawingBuffer: true })
            offRenderer.setSize(captureSize, captureSize)
            offRenderer.shadowMap.enabled = true
            offRenderer.setClearColor(0xd8dce3, 1)

            const offCam = deps.camera.clone()
            offCam.aspect = 1

            for (const view of viewAngles) {
              // Spherical coordinates: azimuth around Y-up, elevation from ground plane
              const d = view.dist
              offCam.position.set(
                robotCenter.x + d * Math.cos(view.el) * Math.sin(view.az),
                robotCenter.y + d * Math.sin(view.el),
                robotCenter.z + d * Math.cos(view.el) * Math.cos(view.az),
              )
              offCam.lookAt(robotCenter)
              offCam.updateProjectionMatrix()
              offRenderer.render(deps.scene, offCam)
              const dataUrl = offCanvas.toDataURL('image/png')
              screenshots.push(dataUrl.replace(/^data:image\/png;base64,/, ''))
            }
            offRenderer.dispose()

            // ── Restore scene state ──
            deps.scene.background = origBackground
            deps.scene.remove(captureFill)
            deps.scene.remove(captureGround)
            ;(captureGround.material as THREE.Material).dispose()
            captureGround.geometry.dispose()
            for (const obj of hiddenObjects) obj.visible = true

            const totalKB = screenshots.reduce((sum, s) => sum + s.length, 0) / 1024
            console.log(`[AI] Captured 3 views (${captureSize}x${captureSize}, ${totalKB.toFixed(0)}KB total)`)

            const valResult = await invoke('ai_validate_assembly', {
              urdfContent: assemblyResult,
              originalPrompt: prompt,
              sessionId: deps.getCurrentChatId(),
              screenshotBase64: screenshots[0],
              screenshots,
            }) as { ok: boolean; notes: string; corrected_urdf?: string; edit_count?: number }

            console.log(`[AI][redesign] Validation result: ok=${valResult.ok}, needs_redesign=${(valResult as any).needs_redesign}, retryCount=${retryCount}`)
            console.log(`[AI][redesign] Full valResult:`, JSON.stringify(valResult, null, 2))

            if (!valResult.ok) {
              console.log(`[AI][redesign] Validation FAILED: ${valResult.notes}`)
              const checklist = (valResult as any).checklist as { check: string; pass: boolean; detail: string; fixable_by?: string }[] | undefined
              const needsRedesign = (valResult as any).needs_redesign
              const allFailures = checklist ? checklist.filter(c => !c.pass) : []
              const topoFailures = allFailures.filter(c => c.fixable_by === 'topology')
              const placementFailures = allFailures.filter(c => c.fixable_by === 'placement')

              // Redesign trigger: only when a topology-fixable failure exists.
              // Placement-only failures (grounded fail, splay direction, etc.)
              // are handled by the placement engine in urdfAssembly.ts — Claude's
              // topology redesign can't influence them, so kicking off a redesign
              // just reshuffles a viable topology without addressing the issue
              // (and often regresses the good parts). Gemini already signals
              // needs_redesign:false when only placement failures remain; this
              // heuristic now aligns with that judgment instead of overriding it.
              const shouldRedesign = topoFailures.length > 0

              if (shouldRedesign && retryCount < 1) {
                const failures = allFailures
                  .map(c => `- [${c.fixable_by || '?'}] ${c.check}: ${c.detail}`)
                  .join('\n')
                const reason = needsRedesign
                  ? 'Visual validation found topology issues. Redesigning...'
                  : `Visual validation flagged ${topoFailures.length} topology issue(s) — redesigning.`
                addVCMessage('system', `<span style="color:#e5c07b;">${reason}</span>`)
                const notesLine = valResult.notes ? `\n\nValidator notes: ${valResult.notes}` : ''
                const placementGuidance = placementFailures.length > 0
                  ? `\n\nNote: items tagged [placement] are computed by the placement engine, not by you directly. However, a different component choice, connection order, or attach_face often avoids them — e.g. a wider baseplate preset, a structural bracket between stacked servos, or rest-pose attach_rpy on leg joints.`
                  : ''
                // Phase 4: include the previous (failed) AssemblyGraph so Claude
                // can reason "what did I try, what specifically failed, what to
                // change" instead of redesigning blind from scratch. Trims the
                // search space dramatically on the second attempt. Wording is
                // careful: this is still a full design_robot call (not an
                // incremental edit), so we say "produce a NEW full topology"
                // but encourage reusing whatever the validator did not flag.
                const previousGraph = result.assembly_graph as AssemblyGraph | undefined
                const previousTopologyBlock = previousGraph
                  ? `\n\nPrevious attempt (the one that failed validation):\n${summarizeAssemblyGraphForAI(previousGraph)}`
                  : ''
                const warnLine = formatWarningsForPrompt(assemblyOut.topologyWarnings)
                const redesignPrompt = `${prompt}\n\nIMPORTANT — REDESIGN REQUIRED: The previous assembly was built and visually inspected. The validator found these problems:\n${failures}${notesLine}${warnLine}${placementGuidance}${previousTopologyBlock}\n\nProduce a NEW full topology with design_robot (this is a fresh design call, not an incremental edit). You may reuse component choices, attach_faces, and connections from the previous attempt that the validator did NOT flag — only change what the validator specifically called out.`
                vcSend.disabled = false
                unlisten?.()
                return sendVCMessage(redesignPrompt, retryCount + 1)
              }
              if (placementFailures.length > 0 && topoFailures.length === 0) {
                console.log(`[AI][redesign] Skipping redesign — all ${placementFailures.length} failure(s) are placement-fixable, which Claude's topology can't address. Returning current build as final.`)
              }
            }
          } catch (valErr) {
            console.warn('[AI] Visual validation skipped:', valErr)
          }

          const diff = computeSimpleDiff(fullUrdf, assemblyResult)
          const graphSummary = summarizeAssemblyGraph(result.assembly_graph as AssemblyGraph)
          const warnBlock = formatWarningsHtml(assemblyOut.topologyWarnings)
          addVCMessage('assistant', `${graphSummary}${result.explanation}${warnBlock}<br><span style="color:#858585;font-size:11px">${result.stats}</span>`, {
            diff, newUrdf: assemblyResult,
          })
          deps.showInlineDiff(fullUrdf, assemblyResult, assemblyResult)
        } else if (retryCount < 2) {
          const topoErrors = assemblyOut.topologyErrors
          if (topoErrors && topoErrors.length > 0) {
            addVCMessage('system', `<span style="color:#e5c07b;">Topology validation failed. Redesigning...</span>`)
            const errorList = topoErrors.map(e => `- ${e}`).join('\n')
            const warnLine = formatWarningsForPrompt(assemblyOut.topologyWarnings)
            const retryPrompt = `${prompt}\n\nIMPORTANT — TOPOLOGY REJECTED: The placement engine rejected your topology because of these specific errors:\n${errorList}${warnLine}\n\nPlease fix these issues in your new design.`
            vcSend.disabled = false
            unlisten?.()
            return sendVCMessage(retryPrompt, retryCount + 1)
          } else {
            addVCMessage('system', `<span style="color:#e5c07b;">Assembly placement failed. Retrying with simpler topology...</span>`)
            const retryPrompt = `${prompt}\n\nIMPORTANT: The previous assembly attempt failed because components couldn't be placed. Please use a SIMPLER design with fewer components.`
            vcSend.disabled = false
            unlisten?.()
            return sendVCMessage(retryPrompt, retryCount + 1)
          }
        } else {
          addVCMessage('assistant', `<span style="color:#f85149;">Assembly placement failed after ${retryCount + 1} attempts. Try describing a simpler robot.</span>`)
        }
      } else {
        const diff = computeSimpleDiff(fullUrdf, result.new_urdf)
        addVCMessage('assistant', `${result.explanation}<br><span style="color:#858585;font-size:11px">${result.stats}</span>`, {
          diff, newUrdf: result.new_urdf,
        })
        deps.showInlineDiff(currentUrdf, result.new_urdf, result.new_urdf)
      }

    } catch (err) {
      thinking.remove()
      const errStr = String(err)
      console.warn('[VC] Backend error:', err)

      if (errStr.includes('429') || errStr.includes('rate_limit')) {
        if (retryCount < 2) {
          const waitSec = (retryCount + 1) * 5
          addVCMessage('system', `<span style="color:#e5c07b;">Rate limited. Retrying in ${waitSec}s...</span>`)
          await new Promise(r => setTimeout(r, waitSec * 1000))
          unlisten?.()
          vcSend.disabled = false
          return sendVCMessage(prompt, retryCount + 1)
        }
        addVCMessage('assistant', `<span style="color:#f85149;">Rate limited after ${retryCount + 1} attempts. Please wait a moment and try again.</span>`)
      } else {
        addVCMessage('assistant', `<span style="color:#f85149;">Error: ${escapeHtml(errStr.slice(0, 200))}</span>`)
      }
    } finally {
      unlisten?.()
      vcSend.disabled = false
    }
  }

  // ── Input handlers ────────────────────────────────────────────────────────

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
  }
}
