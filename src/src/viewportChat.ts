// viewportChat.ts — Viewport chat panel: message rendering, AI send, tab switching

import * as THREE from 'three'
import { invoke } from '@tauri-apps/api/core'
import { listen } from '@tauri-apps/api/event'
import { escapeHtml } from './chatHistory'
import type { UrdfAssemblyApi, TopologyOp, AssemblyGraph } from './urdfAssembly'
import { cloneAssemblyGraph } from './urdfGraphEquivalence'
import type { GraphMutation, MutationResult } from './graphMutations'

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
  // Main viewport renderer. Screenshot capture renders into a
  // WebGLRenderTarget on THIS renderer rather than creating a second one —
  // a 2nd live WebGL context can evict the main one on Tauri/WebView2 (no
  // webglcontextlost handler exists), which manifests as a full app reload.
  renderer: THREE.WebGLRenderer
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
- Rotary servos drive exactly ONE child. For joint_axis="x" or "y", the assembler automatically adds side-yoke plus slim horn-link adapter hardware and turns the physical shaft onto that red/blue hinge axis; do not add extra coupler-disc stacks.
- For sleek robot legs/arms, use structural_limb_link_slim for thighs, shins, and forearms instead of bulky 2020/4040 extrusions. Set length_mm for the limb segment length. For robot dogs/quadrupeds, never use structural_extrusion_2020 or structural_extrusion_4040 as thigh/shin bones.
- Never put structural_servo_coupler_disc between a servo and a leg/arm limb; it makes the limb coaxial with the shaft instead of radial to the horn.
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

// ── Workstream #2 tool-call edit surface ─────────────────────────────────────
// The Python side sends Anthropic tool_use blocks; this table resolves each
// back into a typed GraphMutation the TS dispatcher can apply. Keeping the
// (tool_name → mutation.kind) mapping in one place avoids drift between the
// JSON schema declared in claude_client.py and the TS mutator names.

interface ClaudeToolCall { id: string; name: string; input: Record<string, unknown> }

interface ToolResultBlock {
  tool_use_id: string
  ok: boolean
  summary?: string
  warnings?: string[]
  code?: string
  message?: string
  suggested_repair?: string
}

interface EditTurnResponse {
  stop_reason: string
  text: string
  tool_calls: ClaudeToolCall[]
  done: boolean
}

function toolCallToMutation(call: ClaudeToolCall): GraphMutation | { error: string } {
  const input = call.input || {}
  const s = (k: string) => typeof input[k] === 'string' ? input[k] as string : undefined
  const n = (k: string) => typeof input[k] === 'number' ? input[k] as number : undefined
  const b = (k: string) => typeof input[k] === 'boolean' ? input[k] as boolean : undefined
  const a = (k: string): number[] | undefined => {
    const v = input[k]
    return Array.isArray(v) && v.every(x => typeof x === 'number') ? v as number[] : undefined
  }
  switch (call.name) {
    case 'add_link': {
      const link_name = s('link_name'), parent_link = s('parent_link'),
            preset_id = s('preset_id'), attach_face = s('attach_face')
      if (!link_name || !parent_link || !preset_id || !attach_face) {
        return { error: 'add_link missing required field(s): link_name, parent_link, preset_id, attach_face' }
      }
      return { kind: 'add_link', args: {
        link_name, parent_link, component_id: preset_id, attach_face,
        joint_type: s('joint_type'), joint_axis: s('joint_axis'),
        length_mm: n('length_mm'), orientation: s('orientation'),
        elevation_angle: n('elevation_angle'), attach_rpy: a('attach_rpy'),
        attach_connector: s('attach_connector'),
        mate_connector: s('mate_connector'),
        mate_type: s('mate_type'),
      } }
    }
    case 'attach_sensor': {
      const link_name = s('link_name'), parent_link = s('parent_link'),
            preset_id = s('preset_id'), mount_face = s('mount_face')
      if (!link_name || !parent_link || !preset_id || !mount_face) {
        return { error: 'attach_sensor missing required field(s): link_name, parent_link, preset_id, mount_face' }
      }
      return { kind: 'attach_sensor', args: {
        link_name, parent_link, component_id: preset_id, mount_face,
        elevation_angle: n('elevation_angle'),
      } }
    }
    case 'replace_component': {
      const link_name = s('link_name'), new_preset_id = s('new_preset_id')
      if (!link_name || !new_preset_id) {
        return { error: 'replace_component missing required field(s): link_name, new_preset_id' }
      }
      // Tool schema uses new_preset_id (user-facing); TS dispatcher uses
      // new_component_id (internal naming consistent with component_id).
      return { kind: 'replace_component', args: { link_name, new_component_id: new_preset_id } }
    }
    case 'set_joint': {
      const link_name = s('link_name'), joint_type = s('joint_type')
      if (!link_name || !joint_type) {
        return { error: 'set_joint missing required field(s): link_name, joint_type' }
      }
      return { kind: 'set_joint', args: {
        link_name, joint_type, joint_axis: s('joint_axis'), attach_rpy: a('attach_rpy'),
      } }
    }
    case 'remove_link': {
      const link_name = s('link_name')
      if (!link_name) return { error: 'remove_link missing required field: link_name' }
      return { kind: 'remove_link', args: { link_name, reparent_children: b('reparent_children') } }
    }
    default:
      return { error: `unknown tool: ${call.name}` }
  }
}

function mutationResultToToolResult(
  toolUseId: string,
  result: MutationResult,
): ToolResultBlock {
  if (result.ok) {
    return {
      tool_use_id: toolUseId,
      ok: true,
      summary: result.summary,
      warnings: result.warnings,
    }
  }
  return {
    tool_use_id: toolUseId,
    ok: false,
    code: result.code,
    message: result.message,
    suggested_repair: result.suggested_repair,
  }
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

  // ── AI send ───────────────────────────────────────────────────────────────

  /** Run the Workstream #2 tool-call edit loop. Returns true if the loop
   * handled the turn (UI update owned by the loop); false if it stopped
   * with no mutations so the caller should fall through to `ai_edit`. */
  async function runToolCallEditLoop(args: {
    prompt: string
    initialGraph: AssemblyGraph
    kinematicContext: string
    images: ImageAttachment[]
    sessionId: string
    fullUrdf: string
    thinking: HTMLElement & { updateStage: (stage: string, text: string) => void }
    urdfAssemblyApi: UrdfAssemblyApi
    cancelToken: { cancelled: boolean; awaitingDecision: boolean }
  }): Promise<boolean> {
    const { prompt, initialGraph, kinematicContext, images, sessionId, fullUrdf, thinking, urdfAssemblyApi, cancelToken } = args

    // Working copy — each successful tool dispatch replaces this; a failed
    // call feeds the error back without touching the working graph. Final
    // commit via resolveAssemblyGraph happens only after the loop ends.
    let workingGraph: AssemblyGraph = cloneAssemblyGraph(initialGraph)
    const mutationSummaries: string[] = []
    let explanationText = ''
    let turnCount = 0
    // Independent cap on the frontend side — the Python side has its own cap;
    // this is a belt-and-braces guard against a server bug looping forever.
    const MAX_TURNS = 12

    // ── Turn 1: send prompt + graph ───────────────────────────────────────
    args.thinking.updateStage('generating', 'Planning edits...')
    let turn: EditTurnResponse
    try {
      turn = await invoke('ai_edit_turn', {
        sessionId,
        prompt,
        assemblyGraph: workingGraph,
        kinematicContext,
        images: images.map(({ media_type, data }) => ({ media_type, data })),
      }) as EditTurnResponse
    } catch (err) {
      console.warn('[AI][tool-loop] First turn failed, falling back to ai_edit:', err)
      return false
    }
    if (cancelToken.cancelled) return true

    // ── Loop: dispatch → feed tool_results back → next turn ────────────────
    while (!turn.done && turnCount < MAX_TURNS) {
      turnCount++
      if (turn.text) {
        // Claude sometimes narrates before a tool call ("I'll add a camera
        // to the extrusion. Let me call add_link..."). Stash the latest,
        // we'll show the final text block after the loop.
        explanationText = turn.text
      }
      if (turn.tool_calls.length === 0) break

      const toolResults: ToolResultBlock[] = []
      for (const call of turn.tool_calls) {
        const mapped = toolCallToMutation(call)
        if ('error' in mapped) {
          toolResults.push({
            tool_use_id: call.id, ok: false, code: 'BAD_TOOL_INPUT', message: mapped.error,
          })
          continue
        }
        const result = urdfAssemblyApi.applyGraphMutation(workingGraph, mapped)
        if (result.ok) {
          workingGraph = result.graph
          mutationSummaries.push(result.summary)
          console.log(`[AI][tool-loop] ${call.name} OK: ${result.summary}`)
        } else {
          console.log(`[AI][tool-loop] ${call.name} REJECTED [${result.code}]: ${result.message}`)
        }
        toolResults.push(mutationResultToToolResult(call.id, result))
      }

      thinking.updateStage('generating', `Tool round ${turnCount + 1}...`)
      try {
        turn = await invoke('ai_edit_turn', {
          sessionId,
          toolResults,
        }) as EditTurnResponse
      } catch (err) {
        console.warn('[AI][tool-loop] Continuation failed:', err)
        break
      }
      if (cancelToken.cancelled) return true
    }

    if (cancelToken.cancelled) return true
    if (turn.text) explanationText = turn.text

    thinking.remove()

    if (mutationSummaries.length === 0) {
      // Claude ended without applying any mutations (e.g., answered as plain text
      // or asked a clarifying question). Return false so the caller falls through
      // to the legacy ai_edit path; if Claude truly wanted to answer in text,
      // it'll show up there.
      console.log('[AI][tool-loop] Claude stopped without mutations — falling back to ai_edit')
      return false
    }

    // Commit the final graph via the normal placement pipeline, which also
    // runs the full TS validator as a sanity gate (auto-repairs + warnings).
    const assemblyOut = urdfAssemblyApi.resolveAssemblyGraph(workingGraph)
    if (!assemblyOut.urdf) {
      const errText = assemblyOut.topologyErrors?.join(', ') || 'unknown placement error'
      addVCMessage('assistant',
        `<span style="color:#f85149;">Edit applied ${mutationSummaries.length} mutation(s) but final placement failed: ${escapeHtml(errText)}</span>`)
      return true
    }
    await new Promise(r => setTimeout(r, 300))
    deps.groundRobot()
    deps.autoFrameRobot()

    const diff = computeSimpleDiff(fullUrdf, assemblyOut.urdf)
    const mutationBlock = mutationSummaries.length > 0
      ? `<div style="font-size:11px;color:#888;margin-bottom:4px;">${mutationSummaries.length} tool call(s): ${escapeHtml(mutationSummaries.join(' · '))}</div>`
      : ''
    const warnBlock = formatWarningsHtml(assemblyOut.topologyWarnings)
    const explainHtml = explanationText ? escapeHtml(explanationText) : 'Edit applied.'
    addVCMessage('assistant', `${mutationBlock}${explainHtml}${warnBlock}`, {
      diff, newUrdf: assemblyOut.urdf,
    })
    deps.showInlineDiff(fullUrdf, assemblyOut.urdf, assemblyOut.urdf)
    cancelToken.awaitingDecision = true
    return true
  }

  // Global lock while the AI is mid-generation. Freezes user input on the
  // editor and sidebars so they can't fight the model — viewport (3D camera +
  // sim controls) stays interactive. Released as soon as the apply/dismiss
  // prompt is on screen (or on error / cancel / early return).
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
    /** Pre-prompt URDF captured on the outermost call. Forwarded through
     * validator-driven redesign retries so the diff baseline stays anchored
     * to what the user actually saw before sending the prompt — not to the
     * failed first attempt that resolveAssemblyGraph already wrote into the
     * editor. Without this, the inline-diff line count for a redesign turn
     * reports only the deltas vs. the discarded attempt, masking the bulk of
     * what changed since the user's original state. */
    originalUrdfOverride?: string,
  ) {
    if (!prompt.trim()) return

    if (!deps.getEditorValue()) {
      deps.createNewFile('robot.urdf', deps.SAMPLE_URDF, null)
    }

    // On the first turn, snapshot what the user attached so retries can re-send
    // the same reference image(s). Retries pass the captured list back in.
    const imagesForThisSend: ImageAttachment[] = imagesOverride ?? attachedImages.slice()
    let historyForBackend: Array<{ role: string; content: string }> = []
    let historySessionId = deps.getCurrentChatId()

    if (retryCount === 0) {
      // Snapshot history before recording the new message so the current prompt
      // is not duplicated when generate_edit adds it separately.
      historySessionId = deps.getCurrentChatId()
      historyForBackend = deps.exportForBackend(historySessionId)

      addVCMessage('user', prompt, imagesForThisSend.length > 0 ? { images: imagesForThisSend } : undefined)
      vcInput.value = ''
      vcInput.style.height = 'auto'
      // Clear pending attachments now that they're sent — the snapshot above
      // keeps them alive for retries without leaving thumbs in the input bar.
      attachedImages = []
      renderAttachThumbs()
    }

    setAiBusy(true)
    // Reuse the parent's token across recursive retries so a single click on
    // ✕ Cancel kills the whole chain. Only the outermost call owns the token.
    const isOutermost = activeCancel === null
    const cancelToken: CancelToken = isOutermost
      ? { cancelled: false, awaitingDecision: false }
      : activeCancel!
    let thinking!: ReturnType<typeof addVCThinking>
    if (isOutermost) {
      activeCancel = cancelToken
      thinking = addVCThinking(() => cancelActiveGeneration(thinking))
    } else {
      thinking = addVCThinking(() => cancelActiveGeneration(thinking))
    }

    let unlisten: (() => void) | null = null
    try {
      unlisten = await listen<{ stage: string; text: string }>('ai_progress', (event) => {
        thinking.updateStage(event.payload.stage, event.payload.text)
      })
    } catch {
      // listen may fail in dev mode without Tauri — non-critical
    }

    // Hoisted out of the try block so the rate-limit catch path can forward it
    // to its recursive sendVCMessage retry — the diff baseline must stay
    // anchored to what the user actually typed against, not a partial state
    // from a failed first attempt.
    const fullUrdf = originalUrdfOverride ?? deps.getEditorValue()

    try {
      if (coreRestartAfterCancel) {
        await coreRestartAfterCancel
        if (cancelToken.cancelled) return
      }

      if (retryCount === 0 && historyForBackend.length > 0) {
        try {
          await invoke('ai_set_history', { sessionId: historySessionId, history: historyForBackend })
          console.log(`[VC] Resynced ${historyForBackend.length} messages for session ${historySessionId}`)
        } catch (err) {
          console.warn('[VC] History resync failed (non-critical):', err)
        }
        if (cancelToken.cancelled) return
      }

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
      //
      // Guard against cross-session stale context: the stored graph persists
      // across browser reloads. If the editor's current URDF no longer matches
      // it (user opened a different file, reset the editor, or kept the sample
      // URDF from startup), the stored graph is a ghost — sending it as
      // "current assembly" makes fresh design_robot calls look like edits and
      // causes Claude to mimic the stale design. Only inject the graph when
      // its base_link actually appears in the current editor URDF.
      const storedGraphForContext = deps.getUrdfAssemblyApi()?.getLastAssemblyGraph() || null
      // Structural match: require that most of the stored graph's components
      // still appear in the editor URDF. A base_link-only check is too weak
      // because the sample/reset URDF uses the generic `base_link` name, which
      // would falsely match any stored graph whose root is also `base_link`.
      const storedMatchesEditor = (() => {
        if (!storedGraphForContext) return false
        const comps = storedGraphForContext.components
        if (comps.length === 0) return false
        let matched = 0
        for (const c of comps) {
          if (fullUrdf.includes(`name="${c.link_name}"`)) matched++
        }
        // Require a supermajority — if the user deleted a few parts manually,
        // the graph is still "mostly current". But a stale 46-component graph
        // against a 1-link sample URDF matches 0-1 names and rightly fails.
        const threshold = Math.max(2, Math.ceil(comps.length * 0.6))
        return matched >= threshold
      })()
      const portOccupancyCtx = !isRedesign && storedGraphForContext && storedMatchesEditor
        ? buildPortOccupancyContext(storedGraphForContext)
        : ''
      const graphSummaryCtx = !isRedesign && storedGraphForContext && storedMatchesEditor
        ? summarizeAssemblyGraphForAI(storedGraphForContext)
        : ''
      if (storedGraphForContext && !storedMatchesEditor) {
        console.log(`[AI] Stored graph base_link "${storedGraphForContext.base_link}" not found in current editor — skipping Phase 4 context (likely a fresh design after reload)`)
      }
      const augmentedContext = isRedesign ? '' : (kinematicContext + portOccupancyCtx + graphSummaryCtx)

      // Workstream #1 (AssemblyGraph Preservation): pass the canonical graph as a
      // first-class field so the backend can hand Claude the lossless source of
      // truth on edit-retry, not a reconstructed URDF. URDF is a renderer/export
      // serialization — feeding it to Claude wastes tokens and reintroduces the
      // urdfToAssemblyGraph round-trip losses this workstream exists to solve.
      //
      // Non-redesign: send the stored graph when the editor's URDF still matches
      // it (same guard as the text summary — prevents ghosting a stale design).
      // Redesign: send the latest resolved graph — that's the canonical form of
      // the attempt Gemini just rejected, and the thing Claude needs to reason
      // "what did I try, what to change" against. (On first-ever turn with no
      // stored graph, the backend falls back to URDF — see claude_client.py.)
      const canonicalGraphForAi: AssemblyGraph | null = isRedesign
        ? (deps.getUrdfAssemblyApi()?.getLastAssemblyGraph() ?? null)
        : (storedGraphForContext && storedMatchesEditor ? storedGraphForContext : null)

      const urdfAssemblyApiEarly = deps.getUrdfAssemblyApi()

      // ── Workstream #2: Tool-Call Edit Surface ────────────────────────────
      // When we have a real in-hand graph (not a redesign, graph actually
      // matches editor, with ≥1 attached child — i.e. something beyond a
      // lone baseplate root), route through the tool-use loop. Each tool
      // call gets per-call validation and can self-correct in the same turn
      // instead of regenerating the whole graph. Design generation (no
      // graph yet, or only a root) + redesigns stay on the old path.
      const hasAttachedChild = (g: AssemblyGraph | null): boolean =>
        !!g && g.components.some(c => c.attach_to !== null)
      const canRunToolLoop =
        !isRedesign &&
        urdfAssemblyApiEarly !== null &&
        hasAttachedChild(canonicalGraphForAi)

      if (canRunToolLoop && urdfAssemblyApiEarly) {
        const handled = await runToolCallEditLoop({
          prompt,
          initialGraph: canonicalGraphForAi!,
          kinematicContext: augmentedContext,
          images: imagesForThisSend,
          sessionId: deps.getCurrentChatId(),
          fullUrdf,
          thinking,
          urdfAssemblyApi: urdfAssemblyApiEarly,
          cancelToken,
        })
        if (cancelToken.cancelled) return
        if (handled) {
          // Loop owned the UI update (diff + assistant message). Nothing else
          // to do for this send; exit cleanly.
          return
        }
        // Loop declined (e.g. Claude stopped with no mutations) — fall through
        // to the existing `ai_edit` single-shot path for backwards compat.
      }

      const result = await invoke('ai_edit', {
        prompt,
        urdfContent: currentUrdf,
        kinematicContext: augmentedContext,
        sessionId: deps.getCurrentChatId(),
        images: imagesForThisSend.map(({ media_type, data }) => ({ media_type, data })),
        assemblyGraph: canonicalGraphForAi ?? undefined,
      }) as { explanation: string; new_urdf: string; stats: string; assembly_graph?: unknown; topology_ops?: TopologyOp[] }

      if (cancelToken.cancelled) return
      thinking.remove()

      const urdfAssemblyApi = urdfAssemblyApiEarly

      // ── modify_topology path: parse current URDF → apply ops → re-resolve ──
      if (result.topology_ops && result.topology_ops.length > 0 && urdfAssemblyApi) {
        console.log(`[AI] Received ${result.topology_ops.length} topology operations — applying to current assembly`)
        // Prefer stored graph (exact, no round-trip loss) over reverse-parsing (lossy fallback).
        // This is the detect-and-log half of WS1's divergence guard: when we're forced to
        // reverse-parse (no canonical stashed), warn which fields are known to drop so the
        // symptom is obvious if a later turn shows lost orientation / elevation_angle etc.
        const storedGraph = urdfAssemblyApi.getLastAssemblyGraph()
        const currentGraph = storedGraph || urdfAssemblyApi.urdfToAssemblyGraph(fullUrdf)
        if (currentGraph && !storedGraph) {
          console.warn('[AI] Using lossy reverse-parsed graph — stored graph not available. URDF round-trip drops: orientation, elevation_angle, length_mm, attach_rpy; forces ground_offset=true.')
        }
        if (!currentGraph) {
          addVCMessage('assistant', `<span style="color:#f85149;">Could not parse current URDF for topology editing. Try "start over" to redesign from scratch.</span>`)
        } else {
          const modifiedGraph = urdfAssemblyApi.applyTopologyOps(
            currentGraph,
            result.topology_ops,
          )
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
            cancelToken.awaitingDecision = true
          } else {
            const errors = assemblyOut.topologyErrors?.join(', ') || 'unknown error'
            addVCMessage('assistant', `<span style="color:#f85149;">Topology modification failed: ${escapeHtml(errors)}</span>`)
          }
        }
      } else if (result.assembly_graph && urdfAssemblyApi) {
        console.log('[AI] Received assembly_graph — resolving via frontend snap system')
        const assemblyOut = urdfAssemblyApi.resolveAssemblyGraph(result.assembly_graph as import('./urdfAssembly').AssemblyGraph)
        let assemblyResult = assemblyOut.urdf
        const engineSummary = assemblyOut.engineSummary
        console.log(`[AI] Assembly result: urdf=${assemblyResult ? `${assemblyResult.length} chars` : 'null'}, topologyErrors=${JSON.stringify(assemblyOut.topologyErrors || [])}, engineSummary=${engineSummary ? `${engineSummary.placements.length} placements / ${engineSummary.icpGaps.length} ICP gaps` : 'null'}`)

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

            // Render into a WebGLRenderTarget on the EXISTING main renderer
            // instead of spinning up a second WebGLRenderer. A 2nd live WebGL
            // context can evict the main one on WebView2 (no
            // webglcontextlost handler exists in main.ts), which manifests
            // as the entire app refreshing. Render-target capture stays
            // inside the single main context.
            const captureTarget = new THREE.WebGLRenderTarget(captureSize, captureSize, {
              format: THREE.RGBAFormat,
              type: THREE.UnsignedByteType,
            })
            const pixelBuffer = new Uint8Array(captureSize * captureSize * 4)
            const flippedBuffer = new Uint8ClampedArray(captureSize * captureSize * 4)
            const captureCanvas = document.createElement('canvas')
            captureCanvas.width = captureSize
            captureCanvas.height = captureSize
            const captureCtx = captureCanvas.getContext('2d')!

            const prevRenderTarget = deps.renderer.getRenderTarget()
            const prevClearColor = new THREE.Color()
            deps.renderer.getClearColor(prevClearColor)
            const prevClearAlpha = deps.renderer.getClearAlpha()
            deps.renderer.setClearColor(0xd8dce3, 1)

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

              deps.renderer.setRenderTarget(captureTarget)
              deps.renderer.clear()
              deps.renderer.render(deps.scene, offCam)
              deps.renderer.readRenderTargetPixels(captureTarget, 0, 0, captureSize, captureSize, pixelBuffer)

              // WebGL pixel origin is bottom-left, canvas origin is top-left — flip rows.
              const stride = captureSize * 4
              for (let y = 0; y < captureSize; y++) {
                const srcOffset = (captureSize - 1 - y) * stride
                flippedBuffer.set(pixelBuffer.subarray(srcOffset, srcOffset + stride), y * stride)
              }
              captureCtx.putImageData(new ImageData(flippedBuffer, captureSize, captureSize), 0, 0)
              const dataUrl = captureCanvas.toDataURL('image/png')
              screenshots.push(dataUrl.replace(/^data:image\/png;base64,/, ''))
            }

            deps.renderer.setRenderTarget(prevRenderTarget)
            deps.renderer.setClearColor(prevClearColor, prevClearAlpha)
            captureTarget.dispose()

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
              referenceImages: imagesForThisSend.map(({ media_type, data }) => ({ media_type, data })),
              // Engine-computed placement + ICP ground truth (Layer 1 of
              //). Lets Gemini refute
              // "camera floating 45mm" / "shin detached" misreads using the
              // actual xyz/rpy written to URDF plus per-joint ICP gaps.
              engineSummary,
            }) as { ok: boolean; notes: string; corrected_urdf?: string; edit_count?: number }

            console.log(`[AI][redesign] Validation result: ok=${valResult.ok}, needs_redesign=${(valResult as any).needs_redesign}, retryCount=${retryCount}`)
            console.log(`[AI][redesign] Full valResult:`, JSON.stringify(valResult, null, 2))

            if (!valResult.ok) {
              console.log(`[AI][redesign] Validation FAILED: ${valResult.notes}`)
              const checklist = (valResult as any).checklist as { check: string; pass: boolean; detail: string; fixable_by?: string; classifier_drop?: boolean; classifier_reason?: string }[] | undefined
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
              //
              // Additional guard: the server-side critique classifier
              // (core/ai/critique_classifier.py) tags infeasible critiques —
              // e.g. "add a rocker-bogie", "widen feet to 100mm", "baseplate
              // should be 60-100mm thick" — with classifier_drop=true. Those
              // are partitioned off the redesign list; Claude can't satisfy
              // asks outside the catalog and previous attempts produced
              // visibly worse second passes.
              const actionableTopoFailures = topoFailures.filter(f => !f.classifier_drop)
              const aestheticTopoFailures = topoFailures.filter(f => f.classifier_drop)
              const allTopoFailuresAreAesthetic = topoFailures.length > 0 && actionableTopoFailures.length === 0
              // Redesign fires when either (a) there's an actionable topology
              // failure, OR (b) there are placement failures we haven't
              // attempted to fix yet. Placement failures often ARE resolvable
              // by a different component choice / attach_face / orientation
              // (e.g. swap the 137mm battery for a shorter preset when it
              // overhangs the baseplate, add a bracket instead of floating a
              // camera). Without branch (b), the "all topo aesthetic +
              // placement failures" case silently leaves a broken build on
              // screen with no redesign attempt. The retryCount<1 cap still
              // limits the loop to one extra pass.
              const placementOnlyRedesign = actionableTopoFailures.length === 0 && placementFailures.length > 0 && retryCount < 1
              const shouldRedesign = actionableTopoFailures.length > 0 || placementOnlyRedesign

              if (shouldRedesign && retryCount < 1) {
                // Only include actionable topology failures in the "fix these" list.
                // Aesthetic dimension critiques (e.g. "narrow baseplate to 140mm") have
                // no preset that can satisfy them — when we fed them through verbatim
                // Claude tried absurd responses (swapped to a smaller preset, added a
                // central torso-extrusion tower). They go into a separate "ignore"
                // section so Claude sees the reasoning but doesn't act on them.
                const actionableLines = actionableTopoFailures
                  .map(c => `- [${c.fixable_by || '?'}] ${c.check}: ${c.detail}`)
                  .join('\n')
                const placementLines = placementFailures
                  .map(c => `- [${c.fixable_by || '?'}] ${c.check}: ${c.detail}`)
                  .join('\n')
                const aestheticLines = aestheticTopoFailures
                  .map(c => `- [ignored — ${c.classifier_reason || 'no preset fits'}] ${c.check}: ${c.detail}`)
                  .join('\n')
                const failuresBlock = [actionableLines, placementLines].filter(Boolean).join('\n')
                const reason = needsRedesign
                  ? 'Visual validation found topology issues. Redesigning...'
                  : placementOnlyRedesign
                    ? `Visual validation flagged ${placementFailures.length} placement issue(s) — redesigning with different component choices.`
                    : `Visual validation flagged ${actionableTopoFailures.length} actionable topology issue(s) — redesigning.`
                addVCMessage('system', `<span style="color:#e5c07b;">${reason}</span>`)
                const notesLine = valResult.notes ? `\n\nValidator notes: ${valResult.notes}` : ''
                const placementGuidance = placementFailures.length > 0
                  ? `\n\nNote: items tagged [placement] are computed by the placement engine, not by you directly. However, a different component choice, connection order, or attach_face often avoids them — e.g. a wider baseplate preset, a structural bracket between stacked servos, or rest-pose attach_rpy on leg joints.`
                  : ''
                const aestheticGuidance = aestheticTopoFailures.length > 0
                  ? `\n\nDO NOT act on these aesthetic/dimensional critiques — no preset in the palette can satisfy them, and trying (e.g. swapping baseplate size, adding a torso extrusion) produces worse designs:\n${aestheticLines}\n\nKeep the baseplate preset and existing component placements from the previous attempt EXCEPT where the "Fix ONLY these" list above requires adding, removing, or moving a specific component.`
                  : ''
                // Phase 4: include the previous (failed) AssemblyGraph so Claude
                // can reason "what did I try, what specifically failed, what to
                // change" instead of redesigning blind from scratch. Trims the
                // search space dramatically on the second attempt. Wording is
                // careful: this is still a full design_robot call (not an
                // incremental edit), so we say "produce a NEW full topology"
                // but encourage reusing whatever the validator did not flag.
                //
                // Prefer the resolved canonical graph (via getLastAssemblyGraph)
                // over Claude's raw return — the resolved one has link-name
                // remap + any auto-repairs applied, so it matches the URDF
                // Gemini actually critiqued. This is also what the next
                // sendVCMessage call picks up as the `assemblyGraph` IPC param.
                const resolvedPrevious = urdfAssemblyApi?.getLastAssemblyGraph() ?? null
                const previousGraph = resolvedPrevious ?? (result.assembly_graph as AssemblyGraph | undefined)
                const previousTopologyBlock = previousGraph
                  ? `\n\nPrevious attempt (the one that failed validation):\n${summarizeAssemblyGraphForAI(previousGraph)}`
                  : ''
                const warnLine = formatWarningsForPrompt(assemblyOut.topologyWarnings)
                const redesignPrompt = `${prompt}\n\nIMPORTANT — REDESIGN REQUIRED: The previous assembly was built and visually inspected. Fix ONLY these:\n${failuresBlock}${notesLine}${warnLine}${placementGuidance}${aestheticGuidance}${previousTopologyBlock}\n\nProduce a NEW full topology with design_robot (this is a fresh design call, not an incremental edit). You may reuse component choices, attach_faces, and connections from the previous attempt — only change what the "Fix ONLY these" list calls out.`
                unlisten?.()
                return await sendVCMessage(redesignPrompt, retryCount + 1, imagesForThisSend, fullUrdf)
              }
              // Skip-paths: we only reach here when shouldRedesign was false
              // OR retryCount already hit the cap. Log + surface remaining
              // failures so the user isn't left staring at a broken build
              // with no feedback.
              if (!shouldRedesign || retryCount >= 1) {
                if (placementFailures.length > 0 && topoFailures.length === 0) {
                  console.log(`[AI][redesign] Skipping redesign — all ${placementFailures.length} failure(s) are placement-fixable and retry cap reached.`)
                }
                if (allTopoFailuresAreAesthetic) {
                  const lines = topoFailures.map(c => `  • [${c.classifier_reason || 'infeasible'}] ${c.check}: ${c.detail}`).join('\n')
                  console.log(`[AI][redesign] Skipping redesign — all topology failures dropped by critique classifier:\n${lines}`)
                  // Include placement failures in the user-facing note when
                  // present — previously the aesthetic-skip branch swallowed
                  // them, leaving the user with no actionable feedback.
                  const placementNote = placementFailures.length > 0
                    ? ` Placement issues remain: ${placementFailures.map(p => p.detail).join('; ')}.`
                    : ''
                  addVCMessage('system', `<span style="color:#858585;font-size:11px">Validator flagged critiques the available presets can't satisfy (e.g. "add a rocker-bogie", "100mm feet"). Keeping current build — request a different style or part if you want to iterate.${placementNote}</span>`)
                }
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
          cancelToken.awaitingDecision = true
        } else if (retryCount < 2) {
          const topoErrors = assemblyOut.topologyErrors
          if (topoErrors && topoErrors.length > 0) {
            addVCMessage('system', `<span style="color:#e5c07b;">Topology validation failed. Redesigning...</span>`)
            const errorList = topoErrors.map(e => `- ${e}`).join('\n')
            const warnLine = formatWarningsForPrompt(assemblyOut.topologyWarnings)
            const retryPrompt = `${prompt}\n\nIMPORTANT — TOPOLOGY REJECTED: The placement engine rejected your topology because of these specific errors:\n${errorList}${warnLine}\n\nPlease fix these issues in your new design.`
            unlisten?.()
            return await sendVCMessage(retryPrompt, retryCount + 1, imagesForThisSend, fullUrdf)
          } else {
            addVCMessage('system', `<span style="color:#e5c07b;">Assembly placement failed. Retrying with simpler topology...</span>`)
            const retryPrompt = `${prompt}\n\nIMPORTANT: The previous assembly attempt failed because components couldn't be placed. Please use a SIMPLER design with fewer components.`
            unlisten?.()
            return await sendVCMessage(retryPrompt, retryCount + 1, imagesForThisSend, fullUrdf)
          }
        } else {
          addVCMessage('assistant', `<span style="color:#f85149;">Assembly placement failed after ${retryCount + 1} attempts. Try describing a simpler robot.</span>`)
        }
      } else {
        const diff = computeSimpleDiff(fullUrdf, result.new_urdf)
        addVCMessage('assistant', `${result.explanation}<br><span style="color:#858585;font-size:11px">${result.stats}</span>`, {
          diff, newUrdf: result.new_urdf,
        })
        // Use fullUrdf (true editor contents at send time), not currentUrdf —
        // currentUrdf is a 4-line stub on redesigns, and Dismiss would revert
        // the editor to that stub instead of the user's actual original URDF.
        deps.showInlineDiff(fullUrdf, result.new_urdf, result.new_urdf)
        cancelToken.awaitingDecision = true
      }

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
          return await sendVCMessage(prompt, retryCount + 1, imagesForThisSend, fullUrdf)
        }
        addVCMessage('assistant', `<span style="color:#f85149;">Rate limited after ${retryCount + 1} attempts. Please wait a moment and try again.</span>`)
      } else {
        addVCMessage('assistant', `<span style="color:#f85149;">Error: ${escapeHtml(errStr.slice(0, 200))}</span>`)
      }
    } finally {
      unlisten?.()
      if (isOutermost) {
        if (activeCancel === cancelToken) activeCancel = null
        // If a diff prompt is waiting for the user, leave the lockout on —
        // the Apply/Dismiss handlers release it. Otherwise (error / no diff
        // shown / already cancelled), drop it now.
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
