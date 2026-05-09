// simManager.ts — MuJoCo simulation lifecycle, UI panel, and visualization

import * as THREE from 'three'
import type { OrbitControls } from 'three/addons/controls/OrbitControls.js'
import { invoke } from '@tauri-apps/api/core'
import type { ParsedRobot } from './urdfParser'
import type { ValResult } from './validation'
import { urdfVecToSceneVec } from './coordinates.ts'

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
  setTerrainVisual?(config: SimTerrainConfig): void
}

type SimTerrainType = 'flat' | 'rough' | 'stairs'

interface SimTerrainConfig {
  type: SimTerrainType
  seed: number
  height: number
  scale: number
  roughness: number
  friction: number
  roughHeightfield?: {
    nrow: number
    ncol: number
    elevation: number[]
    size: number[]
  } | null
}

const TERRAIN_SCHEMA = {
  seed: { default: 1, min: 0, max: 2147483647 },
  height: { default: 0.08, min: 0, max: 0.4 },
  scale: { default: 1.0, min: 0.25, max: 3.0 },
  roughness: { default: 0.6, min: 0, max: 1.0 },
  friction: { default: 3.0, min: 0.05, max: 5.0 },
} as const

const DEFAULT_TERRAIN_CONFIG: SimTerrainConfig = {
  type: 'flat',
  seed: TERRAIN_SCHEMA.seed.default,
  height: TERRAIN_SCHEMA.height.default,
  scale: TERRAIN_SCHEMA.scale.default,
  roughness: TERRAIN_SCHEMA.roughness.default,
  friction: TERRAIN_SCHEMA.friction.default,
}

export interface SimManagerApi {
  isSimActive(): boolean
  isSimRunning(): boolean
  getSimTime(): number
  /** Called from animate() each frame. Smooth camera follow when sim is active. */
  tickCameraFollow(): void
}

export function initSimManager(deps: SimManagerDeps): SimManagerApi {
  // ── State ──────────────────────────────────────────────────────────────────

  let simActive = false
  let simRunning = false
  let simTime = 0
  let simCoreRunning = false
  let simAtResetMode = true
  let simRafId: number | null = null       // requestAnimationFrame handle
  let simModelDt = 0.002
  let simWallStart = 0                     // wall clock when sim started (ms)
  let simTimeAtStart = 0                   // simTime when sim started
  let simSpeedMult = 1.0                   // user-controlled speed multiplier
  let simErrorState = false
  let lastSimStagingPath: string | null = null
  let simTransitioning = false
  let simReloading = false
  let aiRequestToken = 0
  let aiGenerating = false
  let activeTerrainConfig: SimTerrainConfig = { ...DEFAULT_TERRAIN_CONFIG }

  const originalJointPoses = new Map<string, { position: THREE.Vector3; quaternion: THREE.Quaternion }>()
  const simPreviewLimits = new Map<string, { lower: number; upper: number }>()
  const simJointLimits = new Map<string, { lower: number; upper: number; effort: number }>()
  // ── DOM refs (grabbed lazily) ──────────────────────────────────────────────

  const simNotActive = document.getElementById('sim-not-active')!
  const simControlsBody = document.getElementById('sim-controls-body')!
  const simGravityEnabled = document.getElementById('sim-gravity-enabled') as HTMLInputElement | null
  const terrainTypeEl = document.getElementById('sim-terrain-type') as HTMLSelectElement | null
  const terrainSeedEl = document.getElementById('sim-terrain-seed') as HTMLInputElement | null
  const terrainRandomizeBtn = document.getElementById('sim-terrain-randomize') as HTMLButtonElement | null
  const terrainHeightEl = document.getElementById('sim-terrain-height') as HTMLInputElement | null
  const terrainHeightValEl = document.getElementById('sim-terrain-height-val')
  const terrainScaleEl = document.getElementById('sim-terrain-scale') as HTMLInputElement | null
  const terrainScaleValEl = document.getElementById('sim-terrain-scale-val')
  const terrainRoughnessEl = document.getElementById('sim-terrain-roughness') as HTMLInputElement | null
  const terrainRoughnessValEl = document.getElementById('sim-terrain-roughness-val')
  const terrainFrictionEl = document.getElementById('sim-terrain-friction') as HTMLInputElement | null
  const terrainFrictionValEl = document.getElementById('sim-terrain-friction-val')

  function clampNumber(value: number, lo: number, hi: number, fallback: number) {
    return Number.isFinite(value) ? Math.max(lo, Math.min(hi, value)) : fallback
  }

  function getTerrainConfig(): SimTerrainConfig {
    const rawType = terrainTypeEl?.value as SimTerrainType | undefined
    const type: SimTerrainType = rawType === 'rough' || rawType === 'stairs' ? rawType : 'flat'
    const seed = clampNumber(
      parseInt(terrainSeedEl?.value || String(TERRAIN_SCHEMA.seed.default), 10),
      TERRAIN_SCHEMA.seed.min, TERRAIN_SCHEMA.seed.max, TERRAIN_SCHEMA.seed.default,
    )
    return {
      type,
      seed: Math.round(seed),
      height: clampNumber(
        parseFloat(terrainHeightEl?.value || String(TERRAIN_SCHEMA.height.default)),
        TERRAIN_SCHEMA.height.min, TERRAIN_SCHEMA.height.max, TERRAIN_SCHEMA.height.default,
      ),
      scale: clampNumber(
        parseFloat(terrainScaleEl?.value || String(TERRAIN_SCHEMA.scale.default)),
        TERRAIN_SCHEMA.scale.min, TERRAIN_SCHEMA.scale.max, TERRAIN_SCHEMA.scale.default,
      ),
      roughness: clampNumber(
        parseFloat(terrainRoughnessEl?.value || String(TERRAIN_SCHEMA.roughness.default)),
        TERRAIN_SCHEMA.roughness.min, TERRAIN_SCHEMA.roughness.max, TERRAIN_SCHEMA.roughness.default,
      ),
      friction: clampNumber(
        parseFloat(terrainFrictionEl?.value || String(TERRAIN_SCHEMA.friction.default)),
        TERRAIN_SCHEMA.friction.min, TERRAIN_SCHEMA.friction.max, TERRAIN_SCHEMA.friction.default,
      ),
    }
  }

  function normalizeTerrainConfig(raw: unknown): SimTerrainConfig {
    const config = (raw && typeof raw === 'object') ? raw as Record<string, unknown> : {}
    const typeRaw = String(config.type ?? DEFAULT_TERRAIN_CONFIG.type).toLowerCase()
    const type: SimTerrainType = typeRaw === 'rough' || typeRaw === 'stairs' ? typeRaw : 'flat'
    return {
      type,
      seed: Math.round(clampNumber(
        Number(config.seed),
        TERRAIN_SCHEMA.seed.min, TERRAIN_SCHEMA.seed.max, TERRAIN_SCHEMA.seed.default,
      )),
      height: clampNumber(
        Number(config.height),
        TERRAIN_SCHEMA.height.min, TERRAIN_SCHEMA.height.max, TERRAIN_SCHEMA.height.default,
      ),
      scale: clampNumber(
        Number(config.scale),
        TERRAIN_SCHEMA.scale.min, TERRAIN_SCHEMA.scale.max, TERRAIN_SCHEMA.scale.default,
      ),
      roughness: clampNumber(
        Number(config.roughness),
        TERRAIN_SCHEMA.roughness.min, TERRAIN_SCHEMA.roughness.max, TERRAIN_SCHEMA.roughness.default,
      ),
      friction: clampNumber(
        Number(config.friction),
        TERRAIN_SCHEMA.friction.min, TERRAIN_SCHEMA.friction.max, TERRAIN_SCHEMA.friction.default,
      ),
      roughHeightfield: null,
    }
  }

  function applyTerrainConfigToUi(config: SimTerrainConfig) {
    if (terrainTypeEl) terrainTypeEl.value = config.type
    if (terrainSeedEl) terrainSeedEl.value = String(config.seed)
    if (terrainHeightEl) terrainHeightEl.value = String(config.height)
    if (terrainScaleEl) terrainScaleEl.value = String(config.scale)
    if (terrainRoughnessEl) terrainRoughnessEl.value = String(config.roughness)
    if (terrainFrictionEl) terrainFrictionEl.value = String(config.friction)
    syncTerrainUi()
  }

  function syncTerrainUi() {
    const config = getTerrainConfig()
    activeTerrainConfig = config
    if (terrainSeedEl) terrainSeedEl.value = String(config.seed)
    if (terrainHeightValEl) terrainHeightValEl.textContent = `${config.height.toFixed(2)} m`
    if (terrainScaleValEl) terrainScaleValEl.textContent = `${config.scale.toFixed(1)}x`
    if (terrainRoughnessValEl) terrainRoughnessValEl.textContent = config.roughness.toFixed(2)
    if (terrainFrictionValEl) terrainFrictionValEl.textContent = config.friction.toFixed(1)
    document.querySelectorAll<HTMLElement>('[data-terrain-advanced]').forEach(el => {
      const mode = el.dataset.terrainAdvanced
      el.hidden = config.type === 'flat' || (mode === 'rough' && config.type !== 'rough')
    })
  }

  function setTerrainControlsDisabled(disabled: boolean) {
    ;[
      terrainTypeEl,
      terrainSeedEl,
      terrainRandomizeBtn,
      terrainHeightEl,
      terrainScaleEl,
      terrainRoughnessEl,
      terrainFrictionEl,
    ].forEach(el => { if (el) el.disabled = disabled })
  }

  // ── Phase D visualization groups ──────────────────────────────────────────

  const simTrailGroup = new THREE.Group()
  simTrailGroup.name = 'sim_com_trail'
  deps.worldGroup.add(simTrailGroup)

  const simContactGroup = new THREE.Group()
  simContactGroup.name = 'sim_contacts'
  deps.worldGroup.add(simContactGroup)

  // MuJoCo world is Z-up; Three.js world is Y-up. MuJoCo's basis matches
  // URDF's, so the conversion is the same `urdfVecToSceneVec` seam used by
  // the placement / carry paths.
  function mjVecToThreeWorld(v: number[]): THREE.Vector3 {
    return urdfVecToSceneVec([v[0] ?? 0, v[1] ?? 0, v[2] ?? 0])
  }

  function mjDirToThreeWorld(v: number[]): THREE.Vector3 {
    return urdfVecToSceneVec([v[0] ?? 0, v[1] ?? 0, v[2] ?? 0])
  }

  function threeWorldToWorldGroupLocal(pWorld: THREE.Vector3): THREE.Vector3 {
    return deps.worldGroup.worldToLocal(pWorld.clone())
  }

  function threeWorldDirToWorldGroupLocal(dirWorld: THREE.Vector3): THREE.Vector3 {
    const qWorldGroup = new THREE.Quaternion()
    deps.worldGroup.getWorldQuaternion(qWorldGroup)
    const qInv = qWorldGroup.invert()
    return dirWorld.clone().applyQuaternion(qInv)
  }

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
    const comWorldThree = mjVecToThreeWorld(com)
    const comLocal = threeWorldToWorldGroupLocal(comWorldThree)
    comTrailBuffer.push(comLocal)
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
      const len = Math.min(0.4, Math.max(0.02, c.force * 0.002))
      const posWorldThree = mjVecToThreeWorld(c.pos)
      const posLocal = threeWorldToWorldGroupLocal(posWorldThree)
      const dirWorldThree = mjDirToThreeWorld(c.normal)
      const dir = threeWorldDirToWorldGroupLocal(dirWorldThree).normalize()
      if (dir.lengthSq() < 0.01) { arrow.visible = false; continue }
      arrow.position.copy(posLocal)
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

  // ── Script Runner ──────────────────────────────────────────────────────────

  const scriptStatusEl = document.getElementById('sim-script-status')
  const scriptFileInput = document.getElementById('sim-script-file') as HTMLInputElement | null
  const scriptUploadBtn = document.getElementById('sim-script-upload-btn') as HTMLButtonElement | null
  const scriptClearBtn = document.getElementById('sim-script-clear') as HTMLButtonElement | null

  function setScriptStatus(text: string, state: 'idle' | 'active' | 'error' = 'idle') {
    if (!scriptStatusEl) return
    scriptStatusEl.textContent = text
    scriptStatusEl.classList.remove('active', 'error')
    if (state !== 'idle') scriptStatusEl.classList.add(state)
  }

  function simConfigEditable() {
    return simActive && simCoreRunning && simAtResetMode && !simRunning && !simTransitioning && !simReloading
  }

  function requireSimResetMode(action: string): boolean {
    if (!simCoreRunning) {
      deps.showToast('Start simulation first', 'warning')
      return false
    }
    if (!simAtResetMode || simRunning) {
      deps.showToast(`Reset simulation before ${action}`, 'warning')
      return false
    }
    return true
  }

  function updateSimConfigLock() {
    const editable = simConfigEditable()
    setTerrainControlsDisabled(!editable)
    if (scriptUploadBtn) scriptUploadBtn.disabled = !editable
    if (scriptFileInput) scriptFileInput.disabled = !editable
    if (scriptClearBtn) scriptClearBtn.disabled = !editable || !activeScriptCode
    if (aiPromptEl) aiPromptEl.disabled = !editable
    if (aiGenBtn) aiGenBtn.disabled = !editable || aiGenerating
    if (modifyBtn) modifyBtn.disabled = !editable || aiGenerating || !activeScriptCode
    if (aiApplyBtn) aiApplyBtn.disabled = !editable || aiGenerating || !pendingAiCode
    if (aiDiscardBtn) aiDiscardBtn.disabled = !editable || aiGenerating || !pendingAiCode

    ;[
      'sim-speed-down',
      'sim-speed-up',
      'sim-camera-follow',
      'sim-viz-com-trail',
      'sim-viz-contacts',
      'sim-viz-heatmap',
      'sim-gravity-enabled',
      'sim-free-base',
    ].forEach(id => {
      const el = document.getElementById(id) as HTMLButtonElement | HTMLInputElement | null
      if (el) el.disabled = !editable
    })
  }

  scriptUploadBtn?.addEventListener('click', () => {
    if (!requireSimResetMode('uploading a controller')) return
    scriptFileInput?.click()
  })

  scriptFileInput?.addEventListener('change', async () => {
    const file = scriptFileInput.files?.[0]
    if (!file) return
    if (!requireSimResetMode('uploading a controller')) {
      scriptFileInput.value = ''
      return
    }
    let code: string
    try {
      code = await file.text()
    } catch (e) {
      setScriptStatus(`${file.name} — read failed`, 'error')
      deps.showToast(`Script upload failed: ${e}`, 'error')
      scriptFileInput.value = ''
      return
    }
    try {
      const result = await invoke<{ status: string; message?: string }>('sim_set_script', { code })
      if (result.status === 'error') {
        setScriptStatus(`${file.name} — error`, 'error')
        deps.showToast(`Script error: ${result.message ?? 'unknown'}`, 'error')
      } else {
        setScriptStatus(`${file.name} — active`, 'active')
        activeScriptCode = code
        updateSimConfigLock()
        deps.showToast('Controller active', 'success')
      }
    } catch (e) {
      setScriptStatus(`${file.name} — failed`, 'error')
      deps.showToast(`Script apply failed: ${e}`, 'error')
    }
    scriptFileInput.value = ''   // allow re-uploading the same file
  })

  scriptClearBtn?.addEventListener('click', async () => {
    if (!requireSimResetMode('clearing the controller')) return
    if (simCoreRunning) {
      try { await invoke('sim_set_script', { code: '' }) } catch { /* ignore */ }
    }
    setScriptStatus('No controller loaded', 'idle')
    activeScriptCode = ''
    updateSimConfigLock()
    deps.showToast('Controller cleared', 'info')
  })

  // ── AI Script Generator ───────────────────────────────────────────────────

  const aiPromptEl = document.getElementById('sim-ai-prompt') as HTMLTextAreaElement | null
  const aiGenBtn = document.getElementById('sim-ai-generate') as HTMLButtonElement | null
  const modifyBtn = document.getElementById('sim-ai-modify') as HTMLButtonElement | null
  const aiStatusEl = document.getElementById('sim-ai-status')
  const aiPreviewEl = document.getElementById('sim-ai-preview') as HTMLPreElement | null
  const aiApplyRow = document.getElementById('sim-ai-apply-row')
  const aiApplyBtn = document.getElementById('sim-ai-apply') as HTMLButtonElement | null
  const aiDiscardBtn = document.getElementById('sim-ai-discard') as HTMLButtonElement | null
  const aiCopyBtn = document.getElementById('sim-ai-copy') as HTMLButtonElement | null

  let pendingAiCode = ''
  let activeScriptCode = ''

  function setAiStatus(text: string, state: 'idle' | 'active' | 'error' = 'idle') {
    if (!aiStatusEl) return
    aiStatusEl.textContent = text
    aiStatusEl.classList.remove('active', 'error')
    if (state !== 'idle') aiStatusEl.classList.add(state)
  }

  function showAiPreview(code: string) {
    pendingAiCode = code
    if (aiPreviewEl) {
      aiPreviewEl.textContent = code
      aiPreviewEl.hidden = false
    }
    if (aiApplyRow) aiApplyRow.hidden = false
    if (aiCopyBtn) aiCopyBtn.hidden = !code
    updateSimConfigLock()
  }

  function hideAiPreview() {
    pendingAiCode = ''
    if (aiPreviewEl) aiPreviewEl.hidden = true
    if (aiApplyRow) aiApplyRow.hidden = true
    if (aiCopyBtn) aiCopyBtn.hidden = true
    updateSimConfigLock()
  }

  if (aiCopyBtn) {
    aiCopyBtn.addEventListener('click', async () => {
      if (!pendingAiCode) return
      try {
        await navigator.clipboard.writeText(pendingAiCode)
        aiCopyBtn.classList.add('copied')
        aiCopyBtn.title = 'Copied!'
        setTimeout(() => {
          aiCopyBtn.classList.remove('copied')
          aiCopyBtn.title = 'Copy script to clipboard'
        }, 1200)
      } catch (err) {
        console.warn('[sim-ai] clipboard write failed:', err)
      }
    })
  }

  function resetSimStatusDisplay() {
    deps.simTimeEl.textContent = '0.000s'
    const timeEl = document.getElementById('sim-status-time')
    if (timeEl) timeEl.textContent = '0.000 s'
    const rtfEl = document.getElementById('sim-status-rtf')
    if (rtfEl) rtfEl.textContent = '-'
    const energyRow = document.getElementById('sim-status-energy-row') as HTMLElement | null
    const energyEl = document.getElementById('sim-status-energy')
    if (energyRow) energyRow.style.display = 'none'
    if (energyEl) energyEl.textContent = '-'
    const contactsRow = document.getElementById('sim-status-contacts-row') as HTMLElement | null
    const contactsEl = document.getElementById('sim-status-contacts')
    if (contactsRow) contactsRow.style.display = 'none'
    if (contactsEl) contactsEl.textContent = '0'
    const massInfoEl = document.getElementById('sim-mass-info') as HTMLElement | null
    if (massInfoEl) { massInfoEl.style.display = 'none'; massInfoEl.innerHTML = '' }
  }

  function resetSimSessionState() {
    aiRequestToken++
    simRunning = false
    simAtResetMode = true
    simTime = 0
    simErrorState = false
    simWallStart = 0
    simTimeAtStart = 0
    simSpeedMult = 1.0
    aiGenerating = false
    pendingAiCode = ''
    activeScriptCode = ''
    hideAiPreview()
    setAiStatus('Ready', 'idle')
    setScriptStatus('No controller loaded', 'idle')
    if (scriptFileInput) scriptFileInput.value = ''
    const errEl = document.getElementById('sim-error-overlay')
    if (errEl) { errEl.textContent = ''; errEl.classList.add('hidden') }
    const gravityEl = document.getElementById('sim-gravity-enabled') as HTMLInputElement | null
    if (gravityEl) gravityEl.checked = true
    const freeBaseEl = document.getElementById('sim-free-base') as HTMLInputElement | null
    if (freeBaseEl) freeBaseEl.checked = true
    const followEl = document.getElementById('sim-camera-follow') as HTMLInputElement | null
    if (followEl) followEl.checked = false
    clearSimViz()
    resetSimStatusDisplay()
    updateSimUI()
  }

  async function cleanupBackendSimSession() {
    activeScriptCode = ''
    stopSimLoop()
    try { await invoke('sim_set_script', { code: '' }) } catch { /* ignore cleanup failure */ }
    try { await invoke('sim_reset') } catch { /* ignore cleanup failure */ }
    if (lastSimStagingPath) {
      try { await invoke('remove_sim_staging_urdf', { path: lastSimStagingPath }) } catch { /* ignore */ }
      lastSimStagingPath = null
    }
    try { await invoke('stop_core') } catch { /* ignore cleanup failure */ }
    simCoreRunning = false
    // Sim shutdown stops the core to release MuJoCo state. The chat / parse
    // panels share the same core and would otherwise hit "Core process not
    // running" on the next AI call. Restart it eagerly so leaving sim is
    // transparent to the rest of the app.
    try { await invoke('start_core') } catch { /* ignore — chat will retry */ }
    const massInfoEl = document.getElementById('sim-mass-info') as HTMLElement | null
    if (massInfoEl) { massInfoEl.style.display = 'none'; massInfoEl.innerHTML = '' }
  }

  async function runGenerate(modify: boolean) {
    if (aiGenerating) return
    if (!requireSimResetMode('generating a controller')) return
    const prompt = aiPromptEl?.value.trim() ?? ''
    const urdf = deps.getEditorValue()
    if (!urdf.trim()) { deps.showToast('Load a URDF first', 'warning'); return }
    aiGenerating = true
    const requestToken = ++aiRequestToken
    setAiStatus('Generating…', 'idle')
    if (aiGenBtn) aiGenBtn.disabled = true
    if (modifyBtn) modifyBtn.disabled = true
    try {
      const result = await invoke<{ status: string; code?: string; message?: string }>(
        'ai_gen_sim_script',
        {
          prompt,
          urdfContent: urdf,
          currentScript: modify ? activeScriptCode : '',
          terrainConfig: activeTerrainConfig,
        }
      )
      if (requestToken !== aiRequestToken || !simActive) return
      if (result.status !== 'ok' || !result.code) {
        setAiStatus(`Error: ${result.message ?? 'unknown'}`, 'error')
        if (result.code) showAiPreview(result.code)   // show rejected code for debugging
        deps.showToast(`AI generation failed: ${result.message ?? 'unknown'}`, 'error')
      } else {
        setAiStatus('Ready — review & apply', 'active')
        showAiPreview(result.code)
      }
    } catch (e) {
      if (requestToken !== aiRequestToken || !simActive) return
      setAiStatus(`Failed: ${e}`, 'error')
      deps.showToast(`AI request failed: ${e}`, 'error')
    } finally {
      if (requestToken === aiRequestToken && simActive) {
        aiGenerating = false
        updateSimConfigLock()
      }
    }
  }

  aiGenBtn?.addEventListener('click', () => runGenerate(false))
  modifyBtn?.addEventListener('click', () => runGenerate(true))

  aiPromptEl?.addEventListener('keydown', (e) => {
    if (e.key === 'Enter' && (e.ctrlKey || e.metaKey)) {
      e.preventDefault()
      runGenerate(!!activeScriptCode)
    }
  })

  aiApplyBtn?.addEventListener('click', async () => {
    if (!pendingAiCode) return
    if (!requireSimResetMode('applying a controller')) return
    try {
      const result = await invoke<{ status: string; message?: string }>(
        'sim_set_script', { code: pendingAiCode }
      )
      if (result.status === 'error') {
        setAiStatus(`Apply failed: ${result.message ?? 'unknown'}`, 'error')
        deps.showToast(`Script error: ${result.message ?? 'unknown'}`, 'error')
      } else {
        activeScriptCode = pendingAiCode
        setAiStatus('Applied', 'active')
        setScriptStatus('Generated controller active', 'active')
        deps.showToast('Generated controller active', 'success')
        hideAiPreview()
        updateSimConfigLock()
      }
    } catch (e) {
      setAiStatus(`Apply failed: ${e}`, 'error')
      deps.showToast(`Apply failed: ${e}`, 'error')
    }
  })

  aiDiscardBtn?.addEventListener('click', () => {
    if (!requireSimResetMode('discarding generated controller code')) return
    hideAiPreview()
    setAiStatus('Discarded', 'idle')
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
      const isTorqueMotor = jointInfo.type === 'continuous'
      const lim = simPreviewLimits.get(jointName)
      const lower = isTorqueMotor ? -effort : (lim?.lower ?? -Math.PI)
      const upper = isTorqueMotor ? effort : (lim?.upper ?? Math.PI)
      simJointLimits.set(jointName, { lower, upper, effort })
    }
  }

  function enterSimPanel() {
    simNotActive.classList.add('hidden')
    simControlsBody.classList.remove('hidden')
    buildSimPanel()
  }

  function exitSimPanel() {
    simNotActive.classList.remove('hidden')
    simControlsBody.classList.add('hidden')
    resetSimSessionState()
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
    const normalized = { ...s, joints } as Record<string, unknown>
    // Backend emits n_contacts; keep a stable contacts alias for UI.
    const nContacts = normalized.n_contacts
    if (typeof nContacts === 'number' && typeof normalized.contacts !== 'number') {
      normalized.contacts = nContacts
    }
    return normalized
  }

  function updateSimStateDisplay(state: any) {
    try {
      if (!state || typeof state !== 'object') return

      const timeEl = document.getElementById('sim-status-time')
      if (timeEl && typeof state.time === 'number') {
        timeEl.textContent = `${state.time.toFixed(3)} s`
      }

      const energyRow = document.getElementById('sim-status-energy-row') as HTMLElement | null
      const energyEl = document.getElementById('sim-status-energy')
      if (energyRow && energyEl) {
        if (typeof state.energy_j === 'number') {
          energyRow.style.display = ''
          const ke = typeof state.kinetic_j === 'number' ? state.kinetic_j : null
          energyEl.textContent = ke !== null
            ? `${state.energy_j.toFixed(2)} J  (KE ${ke.toFixed(2)})`
            : `${state.energy_j.toFixed(3)} J`
        } else {
          energyRow.style.display = 'none'
        }
      }

      const contactsRow = document.getElementById('sim-status-contacts-row') as HTMLElement | null
      const contactsEl = document.getElementById('sim-status-contacts')
      if (contactsRow && contactsEl) {
        if (typeof state.contacts === 'number') {
          contactsRow.style.display = ''
          contactsEl.textContent = String(state.contacts)
        } else {
          contactsRow.style.display = 'none'
        }
      }

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

    // Keep prior staging path alive until the new model is confirmed loaded.
    // If sim_load fails we should not eagerly delete the previous artifact.
    const previousStagingPath = lastSimStagingPath

    const simPath = await invoke<string>('write_sim_staging_urdf', {
      content: urdf,
      neighborUrdfPath,
    })

    console.log('[Sim] Loading robot model from', simPath)
    const freeBase = (document.getElementById('sim-free-base') as HTMLInputElement | null)?.checked ?? false
    const terrainConfig = getTerrainConfig()
    deps.setTerrainVisual?.(terrainConfig)
    let modelInfo: Record<string, unknown> = {}
    try {
      modelInfo = await invoke<Record<string, unknown>>('sim_load', {
        path: simPath,
        freeBase,
        terrainConfig,
      })
    } catch (loadErr) {
      try { await invoke('remove_sim_staging_urdf', { path: simPath }) } catch { /* ignore */ }
      throw loadErr
    }
    lastSimStagingPath = simPath
    const normalizedTerrain = normalizeTerrainConfig(modelInfo?.terrain_config)
    const rawHf = modelInfo?.terrain_hfield
    if (rawHf && typeof rawHf === 'object') {
      const hf = rawHf as Record<string, unknown>
      const nrow = Number(hf.nrow)
      const ncol = Number(hf.ncol)
      const elevation = Array.isArray(hf.elevation) ? hf.elevation.map(Number) : []
      const size = Array.isArray(hf.size) ? hf.size.map(Number) : []
      if (
        Number.isFinite(nrow) &&
        Number.isFinite(ncol) &&
        elevation.length >= Math.max(0, Math.floor(nrow) * Math.floor(ncol)) &&
        size.length >= 3
      ) {
        normalizedTerrain.roughHeightfield = {
          nrow: Math.floor(nrow),
          ncol: Math.floor(ncol),
          elevation,
          size,
        }
      }
    }
    activeTerrainConfig = normalizedTerrain
    applyTerrainConfigToUi(normalizedTerrain)
    deps.setTerrainVisual?.(normalizedTerrain)
    if (previousStagingPath && previousStagingPath !== simPath) {
      try { await invoke('remove_sim_staging_urdf', { path: previousStagingPath }) } catch { /* ignore */ }
    }
    simModelDt = (typeof modelInfo?.timestep === 'number' && modelInfo.timestep > 0)
      ? modelInfo.timestep : 0.001
    simErrorState = false
    console.log('[Sim] Robot model loaded, dt =', simModelDt)

    // Show mass / COM info in sim panel
    const massInfoEl = document.getElementById('sim-mass-info') as HTMLElement | null
    if (massInfoEl) {
      const kg = typeof modelInfo.total_mass_kg === 'number' ? modelInfo.total_mass_kg : null
      const com = Array.isArray(modelInfo.com_m) ? modelInfo.com_m as number[] : null
      const warn = typeof modelInfo.mass_warning === 'string' ? modelInfo.mass_warning : null
      let massHtml = ''
      if (kg !== null) massHtml += `Mass: ${kg >= 1 ? kg.toFixed(2) + ' kg' : (kg * 1000).toFixed(1) + ' g'}`
      if (com) massHtml += `<br>COM: [${com.map(v => v.toFixed(3)).join(', ')}] m`
      if (warn) massHtml += `<br><span style="color:#f14c4c;">⚠ ${warn}</span>`
      massInfoEl.innerHTML = massHtml
      massInfoEl.style.display = massHtml ? 'block' : 'none'
      if (warn) deps.showToast(warn, 'warning')
    }

    simCoreRunning = true
    console.log('[Sim] Getting initial state...')
    const initialState = normalizeMuJoCoState(await invoke('sim_get_state'))
    console.log('[Sim] Initial state:', initialState)
    if (typeof initialState.time === 'number' && !Number.isNaN(initialState.time)) {
      simTime = initialState.time
    }
    updateSimStateDisplay(initialState)
    updateSimUI()
  }

  async function shutdownSimulation() {
    try {
      await cleanupBackendSimSession()
      console.log('[Sim] Core stopped')
    } catch (error) {
      console.error('[Sim] Error stopping simulation:', error)
      deps.showToast(`Error stopping simulation: ${String(error)}`, 'error')
    }
  }

  function showSimError(msg: string) {
    simErrorState = true
    simRunning = false
    stopSimLoop()
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
    // How much simulated time should have elapsed since we started this run?
    const wallElapsed = now - simWallStart                           // ms of real time
    const simTarget = simTimeAtStart + (wallElapsed / 1000) * simSpeedMult
    const simBehind = simTarget - simTime                            // seconds of deficit
    // Cap at 16 substeps (16 ms / 1 ms dt = 16).  Keeps frame budget bounded.
    const maxSteps = 16
    const nSteps = Math.max(1, Math.min(Math.round(simBehind / simModelDt), maxSteps))

    try {
      const rawState = await invoke('sim_step', { nSteps })
      const state = normalizeMuJoCoState(rawState)
      if (typeof state.time === 'number' && !Number.isNaN(state.time)) simTime = state.time
      updateSimStateDisplay(state)
      updateRobotFromSimState(state)
      tickSimViz(state)
      updateSimUI()
      clearSimError()
      if (state.script_error) {
        setScriptStatus(`Script error — ${state.script_error}`, 'error')
      }
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
          const original = originalJointPoses.get(jointName)
          const delta = new THREE.Quaternion().setFromAxisAngle(jointInfo.axis, position)
          if (original) {
            jointInfo.group.quaternion.copy(original.quaternion).multiply(delta)
          } else {
            jointInfo.group.quaternion.copy(delta)
          }
        }
      }

      const freeBase = (document.getElementById('sim-free-base') as HTMLInputElement | null)?.checked ?? false
      if (freeBase && Array.isArray(state.body_positions) && state.body_positions.length > 0) {
        const rootBody = state.body_positions[0] as { position?: number[]; rotation?: number[] }
        if (rootBody?.position && rootBody?.rotation) {
          const [px, py, pz] = rootBody.position
          const [qw, qx, qy, qz] = rootBody.rotation
          // MuJoCo is Z-up; the worldGroup child has rotation.x = -π/2 which converts
          // URDF Z-up geometry to Three.js Y-up.  robot.position is in Three.js world
          // space (Y-up), so we must apply the same mapping to the body position:
          //   Rx(-π/2): (x, y, z)_zup → (x, z, -y)_yup
          deps.robot.position.set(px, pz, -py)
          // For the quaternion, conjugate-rotate by Rx(-π/2):
          //   q_threejs = Rx(-π/2) * q_mujoco * Rx(+π/2)
          // Rx(-π/2) as quaternion: axis=(1,0,0), angle=-π/2 → (w=cos(-π/4), x=sin(-π/4), y=0, z=0)
          const RX_W = Math.SQRT1_2   //  cos(-π/4)
          const RX_X = -Math.SQRT1_2  //  sin(-π/4)
          // q_threejs = rxNeg * q_mujoco * rxPos
          // rxNeg = (RX_W, RX_X, 0, 0),  rxPos = (RX_W, -RX_X, 0, 0)
          const mqw = qw, mqx = qx, mqy = qy, mqz = qz
          // left-multiply by rxNeg
          const lw = RX_W * mqw - RX_X * mqx
          const lx = RX_W * mqx + RX_X * mqw
          const ly = RX_W * mqy - RX_X * mqz  // corrected sign
          const lz = RX_W * mqz + RX_X * mqy  // corrected sign
          // right-multiply by rxPos = (RX_W, -RX_X, 0, 0)
          const fw = lw * RX_W - lx * (-RX_X)
          const fx = lw * (-RX_X) + lx * RX_W
          const fy = ly * RX_W + lz * (-RX_X)
          const fz = -ly * (-RX_X) + lz * RX_W
          deps.robot.quaternion.set(fx, fy, fz, fw)
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
    const speedEl = document.getElementById('sim-speed-display')
    if (speedEl) speedEl.textContent = `${simSpeedMult.toFixed(1)}×`
    updateSimConfigLock()
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

  // ── Event Handlers ────────────────────────────────────────────────────────

  deps.simToggle.addEventListener('click', async () => {
    if (simTransitioning) return
    simTransitioning = true
    deps.simToggle.disabled = true

    try {
      if (!simActive) {
        originalJointPoses.clear()
        const parsedRobot = deps.getParsedRobot()
        for (const [jointName, jointInfo] of parsedRobot.joints) {
          originalJointPoses.set(jointName, {
            position: jointInfo.group.position.clone(),
            quaternion: jointInfo.group.quaternion.clone(),
          })
        }
        refreshSimPreviewLimits()
        const freeBaseEl = document.getElementById('sim-free-base') as HTMLInputElement | null
        if (freeBaseEl) freeBaseEl.checked = true
        try {
          resetSimSessionState()
          await initializeSimulation()
          simActive = true
          simAtResetMode = true
          deps.simBar.classList.remove('hidden')
          deps.simToggle.classList.add('running')
          deps.simToggle.querySelector('span')!.textContent = 'Exit Sim'
          deps.viewportLabel.textContent = 'Simulation'
          deps.onEnterSim()
          enterSimPanel()
          deps.openSidebarPanel('sim')
          updateSimUI()
          deps.showToast('Entered simulation mode (MuJoCo)', 'success')
        } catch (error) {
          console.error('[Sim] Failed to initialize:', error)
          // Unwind: restore joint poses captured before the attempted load and
          // keep the visible UI in build mode. onExitSim is only needed after
          // onEnterSim has actually run.
          simRunning = false
          simActive = false
          simCoreRunning = false
          await cleanupBackendSimSession()
          const parsedRobot = deps.getParsedRobot()
          for (const [jointName, jointInfo] of parsedRobot.joints) {
            const original = originalJointPoses.get(jointName)
            if (original) {
              jointInfo.group.position.copy(original.position)
              jointInfo.group.quaternion.copy(original.quaternion)
            }
          }
          originalJointPoses.clear()
          deps.simToggle.classList.remove('running')
          deps.simBar.classList.add('hidden')
          deps.simToggle.querySelector('span')!.textContent = 'Simulate'
          deps.viewportLabel.textContent = '3D Preview'
          const rawMsg = error instanceof Error ? error.message : String(error)
          let friendly = rawMsg
          try {
            const m = rawMsg.match(/\{.*\}/s)
            if (m) {
              const parsed = JSON.parse(m[0])
              if (typeof parsed.data === 'string') friendly = parsed.data
              else if (typeof parsed.message === 'string') friendly = parsed.message
            }
          } catch {}
          deps.showToast(`Simulation failed: ${friendly}`, 'error')
        }
      } else {
        simActive = false
        deps.simBar.classList.add('hidden')
        deps.simToggle.classList.remove('running')
        deps.simToggle.querySelector('span')!.textContent = 'Simulate'
        deps.viewportLabel.textContent = '3D Preview'

        // Stop the sim loop + core FIRST so no in-flight tick can overwrite
        // the robot transform after we restore it below.
        simRunning = false
        simTime = 0
        stopSimLoop()
        await shutdownSimulation()

        // Restore joint-group transforms captured on enter.
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

        // Restore robot group transform (position/quaternion) + build visuals.
        deps.onExitSim()

        exitSimPanel()
        updateSimUI()
        deps.showToast('Exited simulation mode', 'info')
      }
      deps.resize()
    } finally {
      deps.simToggle.disabled = false
      simTransitioning = false
      updateSimUI()
    }
  })

  // Free-base checkbox: the value is baked into the MJCF at load time, so a
  // toggle has no effect until the model is reloaded. Rebuild the sim when the
  // user changes it mid-session so the click doesn't silently do nothing.
  async function reloadSimulationModel(reason: string) {
    if (!simActive || simReloading) return
    if (!requireSimResetMode(reason)) {
      syncTerrainUi()
      return
    }
    simReloading = true
    simRunning = false
    stopSimLoop()
    try {
      const freeBaseChecked = (document.getElementById('sim-free-base') as HTMLInputElement | null)?.checked
      resetSimSessionState()
      const freeBaseEl = document.getElementById('sim-free-base') as HTMLInputElement | null
      if (freeBaseEl && typeof freeBaseChecked === 'boolean') freeBaseEl.checked = freeBaseChecked
      syncTerrainUi()
      await initializeSimulation()
      deps.showToast(`Simulation reloaded: ${reason}`, 'info')
    } catch (error) {
      console.error('[Sim] Reload failed:', error)
      deps.showToast(`Reload failed: ${error instanceof Error ? error.message : String(error)}`, 'error')
      // Reload failure can leave backend with no loaded model (sim_load replaces
      // the model and may clear it on failure). Force a clean disabled state so
      // frontend/backend don't drift.
      await cleanupBackendSimSession()
      simActive = false
      simRunning = false
      simAtResetMode = true
      resetSimStatusDisplay()
    } finally {
      simReloading = false
      updateSimUI()
    }
  }

  syncTerrainUi()
  terrainTypeEl?.addEventListener('change', () => {
    syncTerrainUi()
    reloadSimulationModel('changing terrain')
  })
  terrainSeedEl?.addEventListener('change', () => {
    syncTerrainUi()
    reloadSimulationModel('changing terrain seed')
  })
  terrainRandomizeBtn?.addEventListener('click', () => {
    if (!requireSimResetMode('randomizing terrain')) return
    if (terrainSeedEl) terrainSeedEl.value = String(Math.floor(Math.random() * 2147483647))
    syncTerrainUi()
    reloadSimulationModel('randomizing terrain')
  })
  ;[
    terrainHeightEl,
    terrainScaleEl,
    terrainRoughnessEl,
    terrainFrictionEl,
  ].forEach(el => {
    el?.addEventListener('input', syncTerrainUi)
    el?.addEventListener('change', () => reloadSimulationModel('changing terrain settings'))
  })

  const freeBaseEl = document.getElementById('sim-free-base') as HTMLInputElement | null
  if (freeBaseEl) {
    freeBaseEl.addEventListener('change', async () => {
      reloadSimulationModel('changing free-base mode')
    })
  }

  function startSimLoop() {
    if (simRafId !== null) cancelAnimationFrame(simRafId)
    // Anchor wall clock to current simTime so we step exactly the deficit.
    simWallStart = performance.now()
    simTimeAtStart = simTime
    let stepping = false
    function rafTick() {
      if (!simRunning || !simCoreRunning) return
      simRafId = requestAnimationFrame(rafTick)
      if (stepping) return          // previous step still in-flight — skip this frame
      stepping = true
      stepSimulation().finally(() => { stepping = false })
    }
    simRafId = requestAnimationFrame(rafTick)
  }

  function stopSimLoop() {
    if (simRafId !== null) { cancelAnimationFrame(simRafId); simRafId = null }
  }

  deps.simPlay.addEventListener('click', () => {
    if (!simCoreRunning) return
    clearSimError()
    aiRequestToken++
    aiGenerating = false
    simAtResetMode = false
    simRunning = true
    startSimLoop()
    updateSimUI()
  })

  deps.simPause.addEventListener('click', () => {
    simRunning = false
    stopSimLoop()
    updateSimUI()
  })

  deps.simReset.addEventListener('click', async () => {
    if (!simCoreRunning) return
    simRunning = false
    simErrorState = false
    stopSimLoop()
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
      simAtResetMode = true
      // Re-anchor wall clock so the next play() starts from this simTime, not a stale delta.
      simWallStart = performance.now()
      simTimeAtStart = simTime
      clearSimError()
      updateSimUI()
    } catch (error) {
      console.error('[Sim] Reset error:', error)
    }
  })

  // Sim speed ±
  document.getElementById('sim-speed-down')?.addEventListener('click', () => {
    if (!requireSimResetMode('changing simulation speed')) return
    simSpeedMult = Math.max(0.1, parseFloat((simSpeedMult - 0.1).toFixed(1)))
    if (simRunning) { simWallStart = performance.now(); simTimeAtStart = simTime }
    updateSimUI()
  })
  document.getElementById('sim-speed-up')?.addEventListener('click', () => {
    if (!requireSimResetMode('changing simulation speed')) return
    simSpeedMult = Math.min(2.0, parseFloat((simSpeedMult + 0.1).toFixed(1)))
    if (simRunning) { simWallStart = performance.now(); simTimeAtStart = simTime }
    updateSimUI()
  })

  // Gravity toggle
  simGravityEnabled?.addEventListener('change', async () => {
    if (!requireSimResetMode('changing gravity')) return
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
    stopSimLoop()
    try {
      await invoke('sim_reset')
      const state = normalizeMuJoCoState(await invoke('sim_get_state'))
      updateRobotFromSimState(state)
      simTime = 0
      simAtResetMode = true
      clearSimError()
      updateSimUI()
    } catch (e) { deps.showToast(`Reset failed: ${e}`, 'error') }
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
    getSimTime: () => simTime,
    tickCameraFollow,
  }
}
