// simManager.ts — MuJoCo simulation lifecycle, UI panel, and visualization

import * as THREE from 'three'
import type { OrbitControls } from 'three/addons/controls/OrbitControls.js'
import { invoke } from '@tauri-apps/api/core'
import type { ParsedRobot } from './urdfParser'
import type { ValResult } from './validation'

export interface SimManagerDeps {
  // Three.js objects
  robot: THREE.Group
  worldGroup: THREE.Group
  camera: THREE.PerspectiveCamera
  controls: OrbitControls
  viewportPanel: HTMLElement
  // DOM refs
  simBar: HTMLElement
  simToggle: HTMLButtonElement
  simPlay: HTMLButtonElement
  simPause: HTMLButtonElement
  simReset: HTMLButtonElement
  simProgress: HTMLElement
  simTimeEl: HTMLElement
  viewportLabel: HTMLElement
  // Reactive getters (values change during session)
  getParsedRobot(): ParsedRobot
  getEditorValue(): string
  getActiveFile(): string
  getFilePaths(): Record<string, string | null>
  getCurrentFilePath(): string | null
  // Validation
  validateXMLStructure(urdf: string): ValResult[]
  validateURDFPerLink(urdf: string): ValResult[]
  // Callbacks — deferred to avoid circular init dependencies
  showToast(msg: string, type?: 'success' | 'error' | 'warning' | 'info'): void
  openSidebarPanel(panel: string): void
  resize(): void
  onEnterSim(): void
  onExitSim(): void
}

export interface SimManagerApi {
  isSimActive(): boolean
  isSimRunning(): boolean
  isSimCoreRunning(): boolean
  getSimTime(): number
  /** Called from animate() each frame. Drives joint preview animation when core isn't running. */
  tickPreviewAnimation(): void
  /** Called from animate() each frame. Smooth camera follow when sim is active. */
  tickCameraFollow(): void
}

export function initSimManager(deps: SimManagerDeps): SimManagerApi {
  // ── State ──────────────────────────────────────────────────────────────────

  let simActive = false
  let simRunning = false
  let simTime = 0
  let simCoreRunning = false
  let simStepIntervalId: number | null = null
  let simModelDt = 0.002
  let simLastStepWallTime = 0
  let simErrorState = false
  let lastSimStagingPath: string | null = null
  let simTraceEnabled = false

  const originalJointPoses = new Map<string, { position: THREE.Vector3; quaternion: THREE.Quaternion }>()
  const simPreviewLimits = new Map<string, { lower: number; upper: number }>()
  const simJointLimits = new Map<string, { lower: number; upper: number; effort: number }>()
  const simCurrentPositions = new Map<string, number>()
  const simTraceData: Array<{ t: number; controls: Record<string, number> }> = []

  let simKeyframes: Record<string, Record<string, number>> = {}

  // ── DOM refs (grabbed lazily) ──────────────────────────────────────────────

  const simNotActive = document.getElementById('sim-not-active')!
  const simControlsBody = document.getElementById('sim-controls-body')!
  const simJointSliders = document.getElementById('sim-joint-sliders')!
  const simKfList = document.getElementById('sim-kf-list')!
  const simGravityEnabled = document.getElementById('sim-gravity-enabled') as HTMLInputElement | null

  // ── State display overlay ─────────────────────────────────────────────────

  const simStateDisplay = document.createElement('div')
  simStateDisplay.id = 'sim-state-display'
  simStateDisplay.className = 'sim-state-display'
  simStateDisplay.style.cssText = `
    position: absolute; top: 48px; right: 12px;
    background: rgba(30, 30, 30, 0.95); border: 1px solid #3c3c3c;
    border-radius: 6px; padding: 12px; font-family: monospace; font-size: 11px;
    color: #cccccc; max-width: 240px; max-height: 300px; overflow-y: auto;
    z-index: 100; display: none; backdrop-filter: blur(8px);
  `
  deps.viewportPanel.appendChild(simStateDisplay)

  // ── Phase D visualization groups ──────────────────────────────────────────

  const simTrailGroup = new THREE.Group()
  simTrailGroup.name = 'sim_com_trail'
  deps.worldGroup.add(simTrailGroup)

  const simContactGroup = new THREE.Group()
  simContactGroup.name = 'sim_contacts'
  deps.worldGroup.add(simContactGroup)

  // ── CoM Trail ─────────────────────────────────────────────────────────────

  const COM_TRAIL_MAX = 300
  const comTrailBuffer: THREE.Vector3[] = []
  let comTrailLine: THREE.Line | null = null

  function tickComTrail(state: Record<string, unknown>) {
    const enabled = (document.getElementById('sim-viz-com-trail') as HTMLInputElement | null)?.checked
    if (!enabled) { simTrailGroup.visible = false; return }
    simTrailGroup.visible = true

    const com = state.com_position as number[] | undefined
    if (!com || com.length < 3) return

    comTrailBuffer.push(new THREE.Vector3(com[0], com[1], com[2]))
    if (comTrailBuffer.length > COM_TRAIL_MAX) comTrailBuffer.shift()

    if (comTrailLine) {
      simTrailGroup.remove(comTrailLine)
      comTrailLine.geometry.dispose()
      ;(comTrailLine.material as THREE.Material).dispose()
    }
    if (comTrailBuffer.length < 2) return

    const positions = new Float32Array(comTrailBuffer.length * 3)
    for (let i = 0; i < comTrailBuffer.length; i++) {
      positions[i * 3] = comTrailBuffer[i].x
      positions[i * 3 + 1] = comTrailBuffer[i].y
      positions[i * 3 + 2] = comTrailBuffer[i].z
    }
    const geo = new THREE.BufferGeometry()
    geo.setAttribute('position', new THREE.BufferAttribute(positions, 3))
    const mat = new THREE.LineBasicMaterial({ color: 0x00e5ff, transparent: true, opacity: 0.7 })
    comTrailLine = new THREE.Line(geo, mat)
    simTrailGroup.add(comTrailLine)
  }

  function clearComTrail() {
    comTrailBuffer.length = 0
    if (comTrailLine) {
      simTrailGroup.remove(comTrailLine)
      comTrailLine.geometry.dispose()
      ;(comTrailLine.material as THREE.Material).dispose()
      comTrailLine = null
    }
    simTrailGroup.visible = false
  }

  // ── Contact-Force Arrows ───────────────────────────────────────────────────

  const CONTACT_ARROW_POOL = 20
  const contactArrows: THREE.ArrowHelper[] = []
  for (let i = 0; i < CONTACT_ARROW_POOL; i++) {
    const arrow = new THREE.ArrowHelper(
      new THREE.Vector3(0, 0, 1), new THREE.Vector3(0, 0, 0), 0.1, 0xff4444, 0.04, 0.025,
    )
    arrow.visible = false
    simContactGroup.add(arrow)
    contactArrows.push(arrow)
  }

  function tickContactArrows(state: Record<string, unknown>) {
    const enabled = (document.getElementById('sim-viz-contacts') as HTMLInputElement | null)?.checked
    if (!enabled) { contactArrows.forEach(a => { a.visible = false }); return }

    const list = state.contacts_list as Array<{
      pos: number[]; normal: number[]; force: number; body1: string; body2: string
    }> | undefined

    const contacts = list ?? []
    for (let i = 0; i < CONTACT_ARROW_POOL; i++) {
      const arrow = contactArrows[i]
      const c = contacts[i]
      if (!c || c.force < 0.001) { arrow.visible = false; continue }
      const [px, py, pz] = c.pos
      const [nx, ny, nz] = c.normal
      const len = Math.min(0.4, Math.max(0.02, c.force * 0.002))
      const dir = new THREE.Vector3(nx, ny, nz).normalize()
      if (dir.lengthSq() < 0.01) { arrow.visible = false; continue }
      arrow.position.set(px, py, pz)
      arrow.setDirection(dir)
      arrow.setLength(len, len * 0.35, len * 0.2)
      const t = Math.min(1, c.force / 100)
      arrow.setColor(new THREE.Color(1, 1 - t, 1 - t))
      arrow.visible = true
    }
  }

  // ── Torque Heatmap ─────────────────────────────────────────────────────────

  const heatmapOriginalEmissive = new Map<string, THREE.Color>()

  function tickTorqueHeatmap(state: Record<string, unknown>) {
    const enabled = (document.getElementById('sim-viz-heatmap') as HTMLInputElement | null)?.checked
    const forces = state.actuator_forces as Record<string, number> | undefined
    const parsedRobot = deps.getParsedRobot()

    for (const [jointName, jointInfo] of parsedRobot.joints) {
      if (jointInfo.type === 'fixed') continue
      const childLinkGroup = parsedRobot.linkGroups.get((jointInfo as any).childLink)
      if (!childLinkGroup) continue

      childLinkGroup.traverse((obj) => {
        if (!(obj instanceof THREE.Mesh)) return
        const mat = obj.material as THREE.MeshStandardMaterial
        if (!mat || !('emissive' in mat)) return

        if (!enabled) {
          const orig = heatmapOriginalEmissive.get(obj.uuid)
          if (orig) { mat.emissive.copy(orig); mat.emissiveIntensity = 0 }
          return
        }

        if (!heatmapOriginalEmissive.has(obj.uuid)) {
          heatmapOriginalEmissive.set(obj.uuid, mat.emissive.clone())
        }

        // Look for position actuator first (_pos), fall back to torque motor (_motor).
        const actName = forces
          ? (`${jointName}_pos` in forces ? `${jointName}_pos` : `${jointName}_motor`)
          : ''
        const force = (forces && actName) ? Math.abs(forces[actName] ?? 0) : 0
        const effort = simJointLimits.get(jointName)?.effort ?? 10
        const t = Math.min(1, force / effort)
        mat.emissive.setRGB(t, 0, 1 - t)
        mat.emissiveIntensity = t * 0.8
      })
    }
  }

  function clearHeatmap() {
    const parsedRobot = deps.getParsedRobot()
    for (const [jointName] of parsedRobot.joints) {
      const joint = parsedRobot.joints.get(jointName)
      const childLinkGroup = parsedRobot.linkGroups.get((joint as any)?.childLink)
      if (!childLinkGroup) continue
      childLinkGroup.traverse((obj) => {
        if (!(obj instanceof THREE.Mesh)) return
        const mat = obj.material as THREE.MeshStandardMaterial
        if (!mat || !('emissive' in mat)) return
        const orig = heatmapOriginalEmissive.get(obj.uuid)
        if (orig) { mat.emissive.copy(orig); mat.emissiveIntensity = 0 }
      })
    }
    heatmapOriginalEmissive.clear()
  }

  function tickSimViz(state: Record<string, unknown>) {
    tickComTrail(state)
    tickContactArrows(state)
    tickTorqueHeatmap(state)
  }

  function clearSimViz() {
    clearComTrail()
    contactArrows.forEach(a => { a.visible = false })
    clearHeatmap()
    ;['sim-viz-com-trail', 'sim-viz-contacts', 'sim-viz-heatmap'].forEach(id => {
      const el = document.getElementById(id) as HTMLInputElement | null
      if (el) el.checked = false
    })
  }

  // ── Control Trace ──────────────────────────────────────────────────────────

  function recordControlTrace(t: number, controlSnapshot: Record<string, number>) {
    if (!simTraceEnabled) return
    simTraceData.push({ t, controls: { ...controlSnapshot } })
    const countEl = document.getElementById('sim-trace-count')
    if (countEl) countEl.textContent = `${simTraceData.length} samples`
    const dlBtn = document.getElementById('sim-trace-download') as HTMLButtonElement | null
    if (dlBtn) dlBtn.disabled = false
  }

  document.getElementById('sim-trace-enabled')?.addEventListener('change', (e) => {
    simTraceEnabled = (e.target as HTMLInputElement).checked
    if (!simTraceEnabled) {
      simTraceData.length = 0
      const countEl = document.getElementById('sim-trace-count')
      if (countEl) countEl.textContent = '0 samples'
      const dlBtn = document.getElementById('sim-trace-download') as HTMLButtonElement | null
      if (dlBtn) dlBtn.disabled = true
    }
  })

  document.getElementById('sim-trace-download')?.addEventListener('click', () => {
    if (simTraceData.length === 0) return
    const allJoints = [...new Set(simTraceData.flatMap(d => Object.keys(d.controls)))]
    const header = ['t', ...allJoints].join(',')
    const rows = simTraceData.map(d =>
      [d.t.toFixed(6), ...allJoints.map(j => (d.controls[j] ?? 0).toFixed(6))].join(',')
    )
    const csv = [header, ...rows].join('\n')
    const blob = new Blob([csv], { type: 'text/csv' })
    const url = URL.createObjectURL(blob)
    const a = document.createElement('a')
    a.href = url; a.download = 'sim_control_trace.csv'
    document.body.appendChild(a); a.click()
    document.body.removeChild(a); URL.revokeObjectURL(url)
  })

  // ── Script Runner ──────────────────────────────────────────────────────────

  function showScriptError(msg: string) {
    const el = document.getElementById('sim-script-error')
    if (el) { el.textContent = msg; el.classList.remove('hidden') }
  }

  function clearScriptError() {
    const el = document.getElementById('sim-script-error')
    if (el) el.classList.add('hidden')
  }

  document.getElementById('sim-script-apply')?.addEventListener('click', async () => {
    if (!simCoreRunning) { deps.showToast('Start simulation first', 'warning'); return }
    const editor = document.getElementById('sim-script-editor') as HTMLTextAreaElement | null
    const code = editor?.value.trim() ?? ''
    try {
      const result = await invoke<{ status: string; message?: string }>('sim_set_script', { code })
      if (result.status === 'error') {
        showScriptError(result.message ?? 'Script error')
        deps.showToast('Script error — check panel', 'error')
      } else if (result.status === 'cleared') {
        clearScriptError(); deps.showToast('Script cleared', 'info')
      } else {
        clearScriptError(); deps.showToast('Script active', 'success')
      }
    } catch (e) {
      showScriptError(String(e)); deps.showToast('Script apply failed', 'error')
    }
  })

  document.getElementById('sim-script-clear')?.addEventListener('click', async () => {
    const editor = document.getElementById('sim-script-editor') as HTMLTextAreaElement | null
    if (editor) editor.value = ''
    clearScriptError()
    if (simCoreRunning) {
      try { await invoke('sim_set_script', { code: '' }) } catch { /* ignore */ }
    }
    deps.showToast('Script cleared', 'info')
  })

  // ── Keyframe Storage ──────────────────────────────────────────────────────

  function loadSimKeyframes() {
    const key = `sim_keyframes::${deps.getCurrentFilePath() || '__default__'}`
    try {
      const raw = localStorage.getItem(key)
      simKeyframes = raw ? JSON.parse(raw) : {}
    } catch { simKeyframes = {} }
  }

  function saveSimKeyframesStorage() {
    const key = `sim_keyframes::${deps.getCurrentFilePath() || '__default__'}`
    try { localStorage.setItem(key, JSON.stringify(simKeyframes)) } catch { /* ignore */ }
  }

  function refreshSimKeyframeList() {
    simKfList.innerHTML = ''
    const names = Object.keys(simKeyframes)
    if (names.length === 0) {
      simKfList.innerHTML = '<div class="sim-kf-empty">No keyframes saved</div>'
      return
    }
    for (const name of names) {
      const row = document.createElement('div')
      row.className = 'sim-kf-row'
      row.innerHTML = `
        <span class="sim-kf-name">${name}</span>
        <button class="sim-kf-load" data-kf="${name}" title="Load keyframe">Load</button>
        <button class="sim-kf-del" data-kf="${name}" title="Delete">✕</button>
      `
      simKfList.appendChild(row)
    }
    simKfList.querySelectorAll<HTMLButtonElement>('.sim-kf-load').forEach(btn => {
      btn.addEventListener('click', () => loadKeyframe(btn.dataset.kf!))
    })
    simKfList.querySelectorAll<HTMLButtonElement>('.sim-kf-del').forEach(btn => {
      btn.addEventListener('click', () => {
        delete simKeyframes[btn.dataset.kf!]
        saveSimKeyframesStorage()
        refreshSimKeyframeList()
      })
    })
  }

  function loadKeyframe(name: string) {
    if (!simCoreRunning) return
    const kf = simKeyframes[name]
    if (!kf) return
    for (const [joint, pos] of Object.entries(kf)) {
      const posSlider = document.getElementById(`ssl-${joint}`) as HTMLInputElement | null
      if (posSlider) posSlider.value = String(pos)
    }
    deps.showToast(`Keyframe "${name}" loaded as target reference`, 'info')
  }

  // Save keyframe button
  document.getElementById('sim-kf-save')?.addEventListener('click', () => {
    if (!simCoreRunning) return
    const timestamp = new Date().toISOString().replace(/[:.]/g, '-').slice(0, 19)
    const name = `kf-${timestamp}`
    const snapshot: Record<string, number> = {}
    simCurrentPositions.forEach((pos, joint) => { snapshot[joint] = pos })
    simKeyframes[name] = snapshot
    saveSimKeyframesStorage()
    refreshSimKeyframeList()
    deps.showToast(`Keyframe "${name}" saved`, 'success')
  })

  // ── Sim Panel UI ──────────────────────────────────────────────────────────

  function refreshSimPreviewLimits() {
    simPreviewLimits.clear()
    const urdf = deps.getEditorValue()
    if (!urdf.trim()) return
    try {
      const doc = new DOMParser().parseFromString(urdf, 'application/xml')
      for (const joint of Array.from(doc.querySelectorAll('joint'))) {
        const name = joint.getAttribute('name')
        const type = joint.getAttribute('type')
        if (!name || (type !== 'revolute' && type !== 'prismatic')) continue
        const limitEl = joint.querySelector('limit')
        if (!limitEl) continue
        const lower = parseFloat(limitEl.getAttribute('lower') || '0')
        const upper = parseFloat(limitEl.getAttribute('upper') || '0')
        if (Number.isFinite(lower) && Number.isFinite(upper) && upper > lower) {
          simPreviewLimits.set(name, { lower, upper })
        }
      }
    } catch { /* ignore parse errors */ }
  }

  function buildSimPanel() {
    simJointLimits.clear()
    simCurrentPositions.clear()
    simJointSliders.innerHTML = ''

    const urdf = deps.getEditorValue()
    const doc = new DOMParser().parseFromString(urdf, 'application/xml')
    const parsedRobot = deps.getParsedRobot()

    for (const [jointName, jointInfo] of parsedRobot.joints) {
      if (jointInfo.type === 'fixed') continue
      const effort = (() => {
        const jEl = Array.from(doc.querySelectorAll('joint')).find(j => j.getAttribute('name') === jointName)
        const lEl = jEl?.querySelector('limit')
        const e = parseFloat(lEl?.getAttribute('effort') || '10')
        return Number.isFinite(e) && e > 0 ? e : 10
      })()
      const lim = simPreviewLimits.get(jointName)
      const lower = lim?.lower ?? -Math.PI
      const upper = lim?.upper ?? Math.PI
      simJointLimits.set(jointName, { lower, upper, effort })

      const row = document.createElement('div')
      row.className = 'sim-slider-row'
      row.dataset.joint = jointName
      row.innerHTML = `
        <div class="sim-slider-label">
          <span class="sim-slider-name">${jointName}</span>
          <span class="sim-slider-val" id="sslv-${jointName}" title="Actual position">0.000</span>
        </div>
        <input type="range" class="sim-slider" id="ssl-${jointName}"
          min="${lower.toFixed(4)}" max="${upper.toFixed(4)}" step="0.001" value="0"
          data-joint="${jointName}" data-effort="${effort}"
          title="Target position (rad)">
        <div class="sim-pos-row">
          <button class="sim-pos-center" data-joint="${jointName}" title="Return to zero">⟳ Zero</button>
        </div>
      `
      simJointSliders.appendChild(row)
    }

    simJointSliders.querySelectorAll<HTMLInputElement>('.sim-slider').forEach(slider => {
      slider.addEventListener('input', () => sendSimControl())
    })
    simJointSliders.querySelectorAll<HTMLButtonElement>('.sim-pos-center').forEach(btn => {
      btn.addEventListener('click', () => {
        const joint = btn.dataset.joint!
        const s = document.getElementById(`ssl-${joint}`) as HTMLInputElement | null
        if (s) s.value = '0'
        sendSimControl()
      })
    })

    refreshSimKeyframeList()
  }

  function sendSimControl() {
    if (!simCoreRunning) return
    const controls: Record<string, number> = {}
    // Position sliders (ssl-) send target joint angles in radians to position actuators.
    simJointSliders.querySelectorAll<HTMLInputElement>('.sim-slider').forEach(s => {
      controls[s.dataset.joint!] = parseFloat(s.value) || 0
    })
    recordControlTrace(simTime, controls)
    invoke('sim_set_control', { controls }).catch(() => { /* ignore */ })
  }

  function updateSimSliders(state: Record<string, unknown>) {
    const joints = state.joints as Record<string, { position: number; velocity: number }> | undefined
    if (!joints) return
    for (const [name, j] of Object.entries(joints)) {
      simCurrentPositions.set(name, j.position)
      // Only update the text readout (actual physics position), not the slider itself.
      // The slider now represents the user's position *target*, not the measured state.
      const valEl = document.getElementById(`sslv-${name}`)
      if (valEl) valEl.textContent = j.position.toFixed(3)
    }
  }

  function enterSimPanel() {
    simNotActive.classList.add('hidden')
    simControlsBody.classList.remove('hidden')
    buildSimPanel()
    loadSimKeyframes()
  }

  function exitSimPanel() {
    simNotActive.classList.remove('hidden')
    simControlsBody.classList.add('hidden')
    simJointSliders.innerHTML = ''
    clearScriptError()
    clearSimViz()
    simTraceEnabled = false
    simTraceData.length = 0
    const traceToggle = document.getElementById('sim-trace-enabled') as HTMLInputElement | null
    if (traceToggle) traceToggle.checked = false
    const countEl = document.getElementById('sim-trace-count')
    if (countEl) countEl.textContent = '0 samples'
    const dlBtn = document.getElementById('sim-trace-download') as HTMLButtonElement | null
    if (dlBtn) dlBtn.disabled = true
    const followEl = document.getElementById('sim-camera-follow') as HTMLInputElement | null
    if (followEl) followEl.checked = false
  }

  // ── State Display ──────────────────────────────────────────────────────────

  function normalizeMuJoCoState(state: unknown): Record<string, unknown> {
    if (!state || typeof state !== 'object') return state as Record<string, unknown>
    const s = state as Record<string, unknown>
    if (s.joints && typeof s.joints === 'object') return s
    const jointStates = s.joint_states
    if (!Array.isArray(jointStates)) return s
    const joints: Record<string, { position: number; velocity: number }> = {}
    for (const j of jointStates) {
      if (j && typeof j === 'object' && typeof (j as { name?: string }).name === 'string') {
        const row = j as { name: string; position?: number; velocity?: number }
        joints[row.name] = {
          position: typeof row.position === 'number' ? row.position : 0,
          velocity: typeof row.velocity === 'number' ? row.velocity : 0,
        }
      }
    }
    return { ...s, joints }
  }

  function updateSimStateDisplay(state: any) {
    try {
      let html = '<div style="font-weight: bold; color: #569cd6; margin-bottom: 8px;">Simulation State</div>'
      if (state && typeof state === 'object') {
        if (state.time !== undefined) {
          html += `<div><span style="color: #dcdcaa;">time:</span> ${(state.time as number).toFixed(3)}s</div>`
        }
        if (state.joints && typeof state.joints === 'object') {
          html += '<div style="margin-top: 6px; color: #858585;">Joints:</div>'
          for (const [name, joint] of Object.entries(state.joints)) {
            if (typeof joint === 'object' && joint !== null) {
              const j = joint as any
              const pos = j.position?.toFixed(3) || '0.000'
              const vel = j.velocity?.toFixed(3) || '0.000'
              html += `<div style="margin-left: 8px;">
                <span style="color: #9cdcfe;">${name}</span>
                <div style="margin-left: 8px; color: #858585; font-size: 10px;">pos: ${pos} | vel: ${vel}</div>
              </div>`
            }
          }
        }
        if (state.contacts !== undefined) {
          html += `<div style="margin-top: 6px; color: #858585;">Contacts: <span style="color: #f14c4c;">${state.contacts}</span></div>`
        }
        if (state.energy !== undefined) {
          html += `<div style="margin-top: 6px; color: #858585;">Energy: <span style="color: #569cd6;">${(state.energy as number).toFixed(3)}J</span></div>`
        }
      }
      simStateDisplay.innerHTML = html
    } catch (e) {
      console.error('[Sim] Error updating display:', e)
    }
  }

  // ── Core Lifecycle ────────────────────────────────────────────────────────

  function summarizeValidationErrors(results: ValResult[]): string {
    const errs = results.filter(r => r.severity === 'error')
    if (errs.length === 0) return 'URDF validation failed'
    return errs.slice(0, 6).map(r => `${r.name}: ${r.message}`).join('\n')
  }

  async function assertUrdfReadyForSim(urdf: string): Promise<void> {
    const xmlErrors = deps.validateXMLStructure(urdf).filter(r => r.severity === 'error')
    if (xmlErrors.length > 0) throw new Error(summarizeValidationErrors(xmlErrors))

    const perLinkResults = deps.validateURDFPerLink(urdf)
    const perLinkErrors = perLinkResults.filter(r => r.severity === 'error')
    const perLinkWarns = perLinkResults.filter(r => r.severity === 'warn')
    if (perLinkErrors.length > 0) throw new Error(summarizeValidationErrors(perLinkErrors))
    if (perLinkWarns.length > 0) {
      deps.showToast(`URDF has ${perLinkWarns.length} completeness warning(s) — check Validation panel.`, 'warning')
    }

    let result: { results?: ValResult[]; summary?: { error?: number; warn?: number } }
    try {
      result = await invoke('validate_urdf_content', { urdf_content: urdf }) as typeof result
    } catch {
      return // Backend not available — per-link checks already ran
    }
    const summary = result.summary
    const results = result.results ?? []
    if (summary?.error && summary.error > 0) throw new Error(summarizeValidationErrors(results))
    if (summary?.warn && summary.warn > 0) {
      deps.showToast(`URDF has ${summary.warn} validation warning(s); continuing to simulation.`, 'warning')
    }
  }

  async function initializeSimulation() {
    console.log('[Sim] Initializing simulation core...')
    try {
      await invoke('start_core')
      console.log('[Sim] Core started successfully')
    } catch (coreErr) {
      const msg = String(coreErr).toLowerCase()
      if (msg.includes('already running') || msg.includes('already started')) {
        console.log('[Sim] Core already running, continuing...')
      } else {
        throw coreErr
      }
    }

    const urdf = deps.getEditorValue()
    if (!urdf.trim()) throw new Error('URDF editor is empty')

    await assertUrdfReadyForSim(urdf)

    const neighborPath = (deps.getFilePaths()[deps.getActiveFile()] || deps.getCurrentFilePath() || '').trim()
    const neighborUrdfPath = neighborPath && /\.urdf$/i.test(neighborPath) ? neighborPath : null

    if (lastSimStagingPath) {
      try { await invoke('remove_sim_staging_urdf', { path: lastSimStagingPath }) } catch { /* ignore */ }
      lastSimStagingPath = null
    }

    const simPath = await invoke<string>('write_sim_staging_urdf', {
      content: urdf,
      neighbor_urdf_path: neighborUrdfPath,
    })

    console.log('[Sim] Loading robot model from', simPath)
    const freeBase = (document.getElementById('sim-free-base') as HTMLInputElement | null)?.checked ?? false
    const seedRaw = (document.getElementById('sim-seed') as HTMLInputElement | null)?.value ?? ''
    const seed = seedRaw.trim() !== '' ? parseInt(seedRaw, 10) : undefined
    let modelInfo: Record<string, unknown> = {}
    try {
      modelInfo = await invoke<Record<string, unknown>>('sim_load', {
        path: simPath,
        freeBase,
        ...(seed !== undefined && Number.isFinite(seed) ? { seed } : {}),
      })
    } catch (loadErr) {
      try { await invoke('remove_sim_staging_urdf', { path: simPath }) } catch { /* ignore */ }
      throw loadErr
    }
    lastSimStagingPath = simPath
    simModelDt = (typeof modelInfo?.timestep === 'number' && modelInfo.timestep > 0)
      ? modelInfo.timestep : 0.002
    simErrorState = false
    console.log('[Sim] Robot model loaded, dt =', simModelDt)

    simCoreRunning = true
    console.log('[Sim] Getting initial state...')
    const initialState = normalizeMuJoCoState(await invoke('sim_get_state'))
    console.log('[Sim] Initial state:', initialState)
    if (typeof initialState.time === 'number' && !Number.isNaN(initialState.time)) {
      simTime = initialState.time
    }
    simStateDisplay.style.display = 'block'
    updateSimStateDisplay(initialState)
    updateSimUI()
  }

  async function shutdownSimulation() {
    try {
      if (simStepIntervalId !== null) { clearInterval(simStepIntervalId); simStepIntervalId = null }
      if (lastSimStagingPath) {
        try { await invoke('remove_sim_staging_urdf', { path: lastSimStagingPath }) } catch { /* ignore */ }
        lastSimStagingPath = null
      }
      await invoke('stop_core')
      simCoreRunning = false
      simStateDisplay.style.display = 'none'
      console.log('[Sim] Core stopped')
    } catch (error) {
      console.error('[Sim] Error stopping simulation:', error)
      deps.showToast(`Error stopping simulation: ${String(error)}`, 'error')
    }
  }

  function showSimError(msg: string) {
    simErrorState = true
    simRunning = false
    if (simStepIntervalId !== null) { clearInterval(simStepIntervalId); simStepIntervalId = null }
    const errEl = document.getElementById('sim-error-overlay')
    if (errEl) { errEl.textContent = `⚠ Sim error: ${msg}`; errEl.classList.remove('hidden') }
    updateSimUI()
  }

  function clearSimError() {
    if (!simErrorState) return
    simErrorState = false
    const errEl = document.getElementById('sim-error-overlay')
    if (errEl) errEl.classList.add('hidden')
  }

  async function stepSimulation() {
    if (!simCoreRunning || simErrorState) return
    const now = performance.now()
    const elapsed = simLastStepWallTime > 0 ? Math.min(now - simLastStepWallTime, 100) : simModelDt * 1000
    simLastStepWallTime = now
    const nSteps = Math.max(1, Math.min(Math.floor(elapsed / (simModelDt * 1000)), 20))

    try {
      const rawState = await invoke('sim_step', { n_steps: nSteps })
      const state = normalizeMuJoCoState(rawState)
      if (typeof state.time === 'number' && !Number.isNaN(state.time)) simTime = state.time
      updateSimStateDisplay(state)
      updateRobotFromSimState(state)
      updateSimSliders(state)
      tickSimViz(state)
      updateSimUI()
      clearSimError()
      if (state.script_error) showScriptError(state.script_error as string)
      else clearScriptError()
    } catch (error) {
      console.error('[Sim] Error stepping simulation:', error)
      showSimError(String(error))
    }
  }

  function updateRobotFromSimState(state: any) {
    try {
      if (!state || !state.joints) return
      const joints = state.joints as any
      const parsedRobot = deps.getParsedRobot()

      for (const [jointName, jointInfo] of parsedRobot.joints) {
        if (joints[jointName]?.position === undefined) continue
        const position = joints[jointName].position as number

        if (jointInfo.type === 'prismatic') {
          const original = originalJointPoses.get(jointName)
          const basePos = original ? original.position : jointInfo.group.position
          jointInfo.group.position.copy(basePos).addScaledVector(jointInfo.axis, position)
        } else {
          const quat = new THREE.Quaternion()
          quat.setFromAxisAngle(jointInfo.axis, position)
          jointInfo.group.quaternion.copy(quat)
        }
      }

      const freeBase = (document.getElementById('sim-free-base') as HTMLInputElement | null)?.checked ?? false
      if (freeBase && Array.isArray(state.body_positions) && state.body_positions.length > 0) {
        const rootBody = state.body_positions[0] as { position?: number[]; rotation?: number[] }
        if (rootBody?.position && rootBody?.rotation) {
          const [px, py, pz] = rootBody.position
          const [qw, qx, qy, qz] = rootBody.rotation
          deps.robot.position.set(px, py, pz)
          deps.robot.quaternion.set(qx, qy, qz, qw)
        }
      }
    } catch (e) {
      console.error('[Sim] Error updating robot from state:', e)
    }
  }

  // ── UI update ─────────────────────────────────────────────────────────────

  function updateSimUI() {
    deps.simPlay.classList.toggle('active', simRunning)
    deps.simPause.classList.toggle('active', !simRunning && simActive)
    deps.simTimeEl.textContent = simTime.toFixed(3) + 's'
    deps.simProgress.style.width = `${Math.min((simTime / 10) * 100, 100)}%`
  }

  // ── Phase C: Camera Follow ────────────────────────────────────────────────

  function tickCameraFollow() {
    const simCameraFollowEl = document.getElementById('sim-camera-follow') as HTMLInputElement | null
    if (!simCameraFollowEl?.checked || !simActive) return
    const worldPos = new THREE.Vector3()
    deps.robot.getWorldPosition(worldPos)
    deps.controls.target.lerp(worldPos, 0.08)
    deps.controls.update()
  }

  // ── Preview Animation (when MuJoCo core is NOT running) ───────────────────

  function tickPreviewAnimation() {
    if (!simRunning || simCoreRunning) return
    simTime += 1 / 60
    updateSimUI()
    const t = simTime
    let i = 0
    const parsedRobot = deps.getParsedRobot()
    for (const [jointName, jointInfo] of parsedRobot.joints) {
      const jType = jointInfo.type
      if (jType !== 'revolute' && jType !== 'continuous' && jType !== 'prismatic') { i++; continue }
      const phase = i * 1.3
      const quat = new THREE.Quaternion()
      const limits = simPreviewLimits.get(jointName)
      if (jType === 'continuous') {
        quat.setFromAxisAngle(jointInfo.axis, t * 1.5 + phase)
      } else if (limits) {
        const mid = (limits.lower + limits.upper) / 2
        const amp = (limits.upper - limits.lower) / 2
        quat.setFromAxisAngle(jointInfo.axis, mid + Math.sin(t * 0.7 + phase) * amp)
      } else {
        quat.setFromAxisAngle(jointInfo.axis, Math.sin(t * 0.7 + phase) * (Math.PI / 4))
      }
      jointInfo.group.quaternion.copy(quat)
      i++
    }
  }

  // ── Event Handlers ────────────────────────────────────────────────────────

  deps.simToggle.addEventListener('click', async () => {
    simActive = !simActive
    deps.simBar.classList.toggle('hidden', !simActive)
    deps.simToggle.classList.toggle('running', simActive)
    deps.simToggle.querySelector('span')!.textContent = simActive ? 'Exit Sim' : 'Simulate'
    deps.viewportLabel.textContent = simActive ? 'Simulation' : '3D Preview'

    if (simActive) {
      deps.onEnterSim()
      originalJointPoses.clear()
      const parsedRobot = deps.getParsedRobot()
      for (const [jointName, jointInfo] of parsedRobot.joints) {
        originalJointPoses.set(jointName, {
          position: jointInfo.group.position.clone(),
          quaternion: jointInfo.group.quaternion.clone(),
        })
      }
      refreshSimPreviewLimits()
      try {
        await initializeSimulation()
        enterSimPanel()
        deps.openSidebarPanel('sim')
        deps.showToast('Entered simulation mode (MuJoCo)', 'success')
      } catch (error) {
        console.error('[Sim] Failed to initialize:', error)
        simActive = false
        simCoreRunning = false
        deps.simToggle.classList.remove('running')
        deps.simBar.classList.add('hidden')
        deps.simToggle.querySelector('span')!.textContent = 'Simulate'
        deps.viewportLabel.textContent = '3D Preview'
        deps.showToast(`Simulation: ${error instanceof Error ? error.message : String(error)}`, 'error')
      }
    } else {
      deps.onExitSim()
      simRunning = false
      simTime = 0
      await shutdownSimulation()

      const parsedRobot = deps.getParsedRobot()
      for (const [jointName, jointInfo] of parsedRobot.joints) {
        const original = originalJointPoses.get(jointName)
        if (original) {
          jointInfo.group.position.copy(original.position)
          jointInfo.group.quaternion.copy(original.quaternion)
        } else {
          jointInfo.group.quaternion.identity()
        }
      }
      originalJointPoses.clear()
      exitSimPanel()
      updateSimUI()
      deps.showToast('Exited simulation mode', 'info')
    }
    deps.resize()
  })

  deps.simPlay.addEventListener('click', () => {
    if (!simCoreRunning) return
    clearSimError()
    simRunning = true
    simLastStepWallTime = 0
    if (simStepIntervalId !== null) clearInterval(simStepIntervalId)
    simStepIntervalId = setInterval(async () => {
      await stepSimulation()
    }, 1000 / 60) as unknown as number
    updateSimUI()
  })

  deps.simPause.addEventListener('click', () => {
    simRunning = false
    if (simStepIntervalId !== null) { clearInterval(simStepIntervalId); simStepIntervalId = null }
    updateSimUI()
  })

  deps.simReset.addEventListener('click', async () => {
    if (!simCoreRunning) return
    simRunning = false
    simErrorState = false
    if (simStepIntervalId !== null) { clearInterval(simStepIntervalId); simStepIntervalId = null }
    try {
      await invoke('sim_reset')
      const state = normalizeMuJoCoState(await invoke('sim_get_state'))
      const parsedRobot = deps.getParsedRobot()
      for (const [jointName, jointInfo] of parsedRobot.joints) {
        const original = originalJointPoses.get(jointName)
        if (original) {
          jointInfo.group.position.copy(original.position)
          jointInfo.group.quaternion.copy(original.quaternion)
        }
      }
      updateRobotFromSimState(state)
      simTime = typeof state.time === 'number' && !Number.isNaN(state.time) ? state.time : 0
      clearSimError()
      updateSimUI()
    } catch (error) {
      console.error('[Sim] Reset error:', error)
    }
  })

  // Gravity toggle
  simGravityEnabled?.addEventListener('change', async () => {
    if (!simCoreRunning) return
    const grav = simGravityEnabled.checked ? [0, 0, -9.81] : [0, 0, 0]
    try {
      await invoke('sim_set_gravity', { gravity: grav })
      deps.showToast(simGravityEnabled.checked ? 'Gravity enabled' : 'Zero-G mode', 'info')
    } catch (e) { deps.showToast(`Gravity toggle failed: ${e}`, 'error') }
  })

  // Reset pose buttons
  document.getElementById('sim-reset-home')?.addEventListener('click', async () => {
    if (!simCoreRunning) return
    simRunning = false
    if (simStepIntervalId !== null) { clearInterval(simStepIntervalId); simStepIntervalId = null }
    try {
      await invoke('sim_reset')
      const state = normalizeMuJoCoState(await invoke('sim_get_state'))
      updateRobotFromSimState(state)
      simTime = 0
      clearSimError()
      updateSimUI()
    } catch (e) { deps.showToast(`Reset failed: ${e}`, 'error') }
  })

  document.getElementById('sim-reset-editor')?.addEventListener('click', async () => {
    if (!simCoreRunning) return
    const parsedRobot = deps.getParsedRobot()
    for (const [jointName, jointInfo] of parsedRobot.joints) {
      const original = originalJointPoses.get(jointName)
      if (original) {
        jointInfo.group.position.copy(original.position)
        jointInfo.group.quaternion.copy(original.quaternion)
      }
    }
    simJointSliders.querySelectorAll<HTMLInputElement>('.sim-torque-slider').forEach(s => { s.value = '0' })
    sendSimControl()
    deps.showToast('Restored editor pose (visual only; physics at home)', 'info')
  })

  // App-close cleanup
  ;(async () => {
    try {
      const { getCurrentWindow } = await import('@tauri-apps/api/window')
      getCurrentWindow().onCloseRequested(async () => {
        if (lastSimStagingPath) {
          try { await invoke('remove_sim_staging_urdf', { path: lastSimStagingPath }) } catch { /* ignore */ }
          lastSimStagingPath = null
        }
      })
    } catch { /* not in Tauri context */ }
  })()

  return {
    isSimActive: () => simActive,
    isSimRunning: () => simRunning,
    isSimCoreRunning: () => simCoreRunning,
    getSimTime: () => simTime,
    tickPreviewAnimation,
    tickCameraFollow,
  }
}
