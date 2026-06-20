// Hardcoded "wow" demos — when the user types a recognized phrase the AI
// pipeline is short-circuited and a pre-authored URDF/STL/animation is loaded
// directly. Purpose: reliably impressive output for live demos. Bypass design,
// not a fallback for a working AI path.

import * as THREE from 'three'
import type { ParsedRobot } from '../urdfParser'
import { applyDogStyling } from './dogStyling'

export interface Demo {
  id: string
  /** Lowercased substrings — match if ANY appears in the trimmed lowercased prompt. */
  triggers: string[]
  /** Path under /demos/ in the dev server (served from src/public/demos). */
  urdfPath: string
  /** Path under /demos/ — pre-baked STL bytes returned by Export Robot STL. */
  stlPath: string
  /** Joint keyframes for the demo's playback loop. Angles in radians. */
  keyframes: DemoKeyframe[]
  /** Loop period in seconds; playback wraps modulo this. */
  loopPeriodSec: number
  /** Optional vertical bob (body wobble) applied to robot.position.y in metres. */
  bodyBobAmplitudeM?: number
  /** Optional forward drift in m/s — applied to robot.position.z over time. */
  forwardDriftMps?: number
  /** Chat text shown when the demo loads. */
  successMessage: string
  /** Optional post-load visual styling (body shell, glowing eyes, etc.).
   *  Returns a cleanup that the active-demo state runs on tear-down. */
  applyStyling?: (parsedRobot: ParsedRobot, robotRoot: THREE.Group) => () => void
}

export interface DemoKeyframe {
  jointName: string
  /** 'sine' (default): angle = bias + amp·sin(2π·freq·t + phase).
   *  'spin': continuous monotonic rotation, angle = bias + 2π·freq·t. Use for wheels. */
  mode?: 'sine' | 'spin'
  /** Hertz of the sine cycle (sine mode) or rotations/sec (spin mode). */
  freqHz: number
  /** Amplitude in radians (ignored in spin mode). */
  amplitudeRad: number
  /** Phase offset in radians (0…2π). */
  phaseRad: number
  /** Bias (centre) angle in radians. */
  biasRad?: number
}

// ── Registry ────────────────────────────────────────────────────────────────

// Trot gait for the K-9 quadruped. Diagonally-opposed legs swing in phase
// (FL+RR vs FR+RL, 180° out of phase). Hip pitches drive the stride; knees
// flex inward on the lift half of each cycle so feet clear the ground.
const TROT_HZ = 1.4
const HIP_AMP = 0.32
const HIP_BIAS = 0           // rest pose already crouched via URDF joint origin rpy
const KNEE_AMP = 0.30
const KNEE_BIAS = 0          // rest pose already bent via URDF joint origin rpy
const YAW_AMP = 0.03

const DOG_DEMO: Demo = {
  id: 'dog',
  triggers: ['dog', 'quadruped', 'spot', 'go1', 'puppy', 'doggo', 'k9', 'k-9'],
  urdfPath: 'demos/dog.urdf',
  // STL is generated live from the URDF on export — no prebake needed.
  stlPath: 'demos/dog.urdf',
  loopPeriodSec: 1 / TROT_HZ,
  bodyBobAmplitudeM: 0.012,
  forwardDriftMps: 0,
  successMessage:
    'Designed K-9 — a Spot-family quadruped: tapered carbon-grey body shell, ' +
    'sculpted limb segments, recessed-visor sensor pod, 12 actuated joints. ' +
    'Try Simulate to see it trot.',
  applyStyling: applyDogStyling,
  // Diagonal trot: FL+RR phase 0, FR+RL phase π. Joints from dog.urdf use
  // descriptive names ({leg}_{joint}) per the new hardcoded URDF.
  keyframes: [
    // FL leg (in-phase with RR)
    { jointName: 'joint_FL_hip_yaw',   freqHz: TROT_HZ, amplitudeRad: YAW_AMP,  phaseRad: 0,             biasRad: 0 },
    { jointName: 'joint_FL_hip_pitch', freqHz: TROT_HZ, amplitudeRad: HIP_AMP,  phaseRad: 0,             biasRad: HIP_BIAS },
    { jointName: 'joint_FL_knee',      freqHz: TROT_HZ, amplitudeRad: KNEE_AMP, phaseRad: Math.PI / 2,   biasRad: KNEE_BIAS },
    // RR leg (in-phase with FL)
    { jointName: 'joint_RR_hip_yaw',   freqHz: TROT_HZ, amplitudeRad: YAW_AMP,  phaseRad: 0,             biasRad: 0 },
    { jointName: 'joint_RR_hip_pitch', freqHz: TROT_HZ, amplitudeRad: HIP_AMP,  phaseRad: 0,             biasRad: HIP_BIAS },
    { jointName: 'joint_RR_knee',      freqHz: TROT_HZ, amplitudeRad: KNEE_AMP, phaseRad: Math.PI / 2,   biasRad: KNEE_BIAS },
    // FR leg (anti-phase to FL)
    { jointName: 'joint_FR_hip_yaw',   freqHz: TROT_HZ, amplitudeRad: YAW_AMP,  phaseRad: Math.PI,       biasRad: 0 },
    { jointName: 'joint_FR_hip_pitch', freqHz: TROT_HZ, amplitudeRad: HIP_AMP,  phaseRad: Math.PI,       biasRad: HIP_BIAS },
    { jointName: 'joint_FR_knee',      freqHz: TROT_HZ, amplitudeRad: KNEE_AMP, phaseRad: Math.PI * 1.5, biasRad: KNEE_BIAS },
    // RL leg (anti-phase to FL)
    { jointName: 'joint_RL_hip_yaw',   freqHz: TROT_HZ, amplitudeRad: YAW_AMP,  phaseRad: Math.PI,       biasRad: 0 },
    { jointName: 'joint_RL_hip_pitch', freqHz: TROT_HZ, amplitudeRad: HIP_AMP,  phaseRad: Math.PI,       biasRad: HIP_BIAS },
    { jointName: 'joint_RL_knee',      freqHz: TROT_HZ, amplitudeRad: KNEE_AMP, phaseRad: Math.PI * 1.5, biasRad: KNEE_BIAS },
  ],
}

// ── Humanoid biped — in-place walk cycle ─────────────────────────────────
// Standard human gait: contralateral arm swing (L arm opposite L leg),
// knees bend on lift phase, ankles plantarflex on push-off.
const HUMAN_HZ = 0.85
const HUMAN_DEMO: Demo = {
  id: 'humanoid',
  triggers: ['humanoid', 'biped', 'human', 'atlas', 'optimus', 'android', 'walker'],
  urdfPath: 'demos/humanoid.urdf',
  stlPath: 'demos/humanoid.urdf',
  loopPeriodSec: 1 / HUMAN_HZ,
  bodyBobAmplitudeM: 0.015,
  forwardDriftMps: 0,
  successMessage:
    'Designed BD-1 — a humanoid biped with 22 actuated joints: 6-DOF legs, 4-DOF arms, 2-DOF neck. ' +
    'Carbon shell, harmonic-drive actuators, stereo vision head with LIDAR puck. ' +
    'Try Simulate to see it walk.',
  keyframes: [
    // ── Legs (contralateral swing)
    { jointName: 'joint_L_hip_pitch',   freqHz: HUMAN_HZ, amplitudeRad: 0.35, phaseRad: 0,             biasRad: 0 },
    { jointName: 'joint_R_hip_pitch',   freqHz: HUMAN_HZ, amplitudeRad: 0.35, phaseRad: Math.PI,       biasRad: 0 },
    { jointName: 'joint_L_knee',        freqHz: HUMAN_HZ, amplitudeRad: 0.30, phaseRad: Math.PI / 2,   biasRad: 0.45 },
    { jointName: 'joint_R_knee',        freqHz: HUMAN_HZ, amplitudeRad: 0.30, phaseRad: Math.PI * 1.5, biasRad: 0.45 },
    { jointName: 'joint_L_ankle_pitch', freqHz: HUMAN_HZ, amplitudeRad: 0.18, phaseRad: 0,             biasRad: -0.10 },
    { jointName: 'joint_R_ankle_pitch', freqHz: HUMAN_HZ, amplitudeRad: 0.18, phaseRad: Math.PI,       biasRad: -0.10 },
    // ── Arms (counter-swing)
    { jointName: 'joint_L_shoulder_pitch', freqHz: HUMAN_HZ, amplitudeRad: 0.40, phaseRad: Math.PI, biasRad: 0 },
    { jointName: 'joint_R_shoulder_pitch', freqHz: HUMAN_HZ, amplitudeRad: 0.40, phaseRad: 0,       biasRad: 0 },
    { jointName: 'joint_L_elbow',          freqHz: HUMAN_HZ, amplitudeRad: 0.10, phaseRad: Math.PI, biasRad: -0.45 },
    { jointName: 'joint_R_elbow',          freqHz: HUMAN_HZ, amplitudeRad: 0.10, phaseRad: 0,       biasRad: -0.45 },
    // ── Head idle (slow scan)
    { jointName: 'joint_neck_yaw', freqHz: 0.15, amplitudeRad: 0.35, phaseRad: 0, biasRad: 0 },
  ],
}

// ── Rover — wheels spin continuously, mast pans slowly ───────────────────
const ROVER_WHEEL_RPS = 0.55   // rotations per second
const ROVER_DEMO: Demo = {
  id: 'rover',
  triggers: ['rover', 'mars', 'perseverance', 'curiosity', 'opportunity', 'sojourner'],
  urdfPath: 'demos/rover.urdf',
  stlPath: 'demos/rover.urdf',
  loopPeriodSec: 10,             // 10s mast pan period
  bodyBobAmplitudeM: 0,
  forwardDriftMps: 0,
  successMessage:
    'Designed MR-1 — a six-wheel rover with rocker-bogie suspension, segmented solar deck, ' +
    'RTG power module, and pan/tilt stereo camera mast. Cleated rubber wheels with CNC spoke hubs. ' +
    'Try Simulate to see the wheels spin and the mast scan.',
  keyframes: [
    // ── Wheels (all 6 in phase, continuous forward spin)
    { jointName: 'joint_FL_wheel', mode: 'spin', freqHz: ROVER_WHEEL_RPS, amplitudeRad: 0, phaseRad: 0 },
    { jointName: 'joint_FR_wheel', mode: 'spin', freqHz: ROVER_WHEEL_RPS, amplitudeRad: 0, phaseRad: 0 },
    { jointName: 'joint_ML_wheel', mode: 'spin', freqHz: ROVER_WHEEL_RPS, amplitudeRad: 0, phaseRad: 0 },
    { jointName: 'joint_MR_wheel', mode: 'spin', freqHz: ROVER_WHEEL_RPS, amplitudeRad: 0, phaseRad: 0 },
    { jointName: 'joint_RL_wheel', mode: 'spin', freqHz: ROVER_WHEEL_RPS, amplitudeRad: 0, phaseRad: 0 },
    { jointName: 'joint_RR_wheel', mode: 'spin', freqHz: ROVER_WHEEL_RPS, amplitudeRad: 0, phaseRad: 0 },
    // ── Mast slow scan
    { jointName: 'joint_mast_yaw',   freqHz: 0.10, amplitudeRad: 0.80, phaseRad: 0,           biasRad: 0 },
    { jointName: 'joint_mast_pitch', freqHz: 0.20, amplitudeRad: 0.25, phaseRad: Math.PI / 2, biasRad: 0 },
  ],
}

// ── Industrial arm — pick-and-place loop ─────────────────────────────────
// Multi-joint sweep that looks like reaching for an object, lifting,
// rotating to a place pose, and returning. Gripper jaws (prismatic) are
// left static — playback path only animates revolute joints today.
const ARM_PERIOD = 6              // seconds per pick-place cycle
const ARM_BASE_HZ = 1 / ARM_PERIOD
const ARM_DEMO: Demo = {
  id: 'arm',
  triggers: ['arm', 'manipulator', 'ur5', 'ur10', 'kuka', 'gripper', 'pick and place', 'pick-and-place', 'robotic arm'],
  urdfPath: 'demos/arm.urdf',
  stlPath: 'demos/arm.urdf',
  loopPeriodSec: ARM_PERIOD,
  bodyBobAmplitudeM: 0,
  forwardDriftMps: 0,
  successMessage:
    'Designed RX-6 — a 6-DOF industrial manipulator with harmonic-drive joints, ' +
    'parallel-jaw gripper, and bolted base plate. Runs a continuous pick-and-place loop. ' +
    'Try Simulate to see it cycle.',
  keyframes: [
    // Base yaw — sweep between pick zone and place zone (large arc).
    { jointName: 'joint_J1', freqHz: ARM_BASE_HZ,       amplitudeRad: 1.20, phaseRad: 0,           biasRad: 0 },
    // Shoulder pitch — reach-down twice per cycle (one at pick, one at place).
    { jointName: 'joint_J2', freqHz: ARM_BASE_HZ * 2,   amplitudeRad: 0.45, phaseRad: 0,           biasRad: 0.35 },
    // Elbow — counters shoulder to keep end-effector tracking vertical.
    { jointName: 'joint_J3', freqHz: ARM_BASE_HZ * 2,   amplitudeRad: 0.60, phaseRad: Math.PI,     biasRad: -0.80 },
    // Wrist roll — flip object orientation on the return.
    { jointName: 'joint_J4', freqHz: ARM_BASE_HZ,       amplitudeRad: 0.80, phaseRad: Math.PI / 2, biasRad: 0 },
    // Wrist pitch — keep end-effector roughly down throughout the cycle.
    { jointName: 'joint_J5', freqHz: ARM_BASE_HZ * 2,   amplitudeRad: 0.30, phaseRad: 0,           biasRad: -0.20 },
    // Tool roll — small continuous twist for visual life.
    { jointName: 'joint_J6', freqHz: ARM_BASE_HZ * 1.5, amplitudeRad: 0.60, phaseRad: 0,           biasRad: 0 },
  ],
}

const REGISTRY: Demo[] = [DOG_DEMO, HUMAN_DEMO, ROVER_DEMO, ARM_DEMO]

// ── Active-demo state ───────────────────────────────────────────────────────

interface OriginalPose {
  position: THREE.Vector3
  quaternion: THREE.Quaternion
}

interface ActiveDemoState {
  demo: Demo
  /** Snapshot of joint origins so playback rotates *about* the rest pose,
   *  matching how the real sim mixes the delta into the original quaternion. */
  jointOrigins: Map<string, OriginalPose>
  /** Snapshot of the robot root pose so body-bob can be applied additively. */
  rootOrigin: OriginalPose | null
  /** Joint axes captured from the parsed robot (URDF axis vectors). */
  jointAxes: Map<string, THREE.Vector3>
  /** Cleanup returned from demo.applyStyling, if any. */
  stylingCleanup: (() => void) | null
  startMs: number
  rafId: number | null
  running: boolean
}

let _active: ActiveDemoState | null = null
let _stlCache = new Map<string, string>()

// ── Public API ──────────────────────────────────────────────────────────────

export function matchDemo(prompt: string): Demo | null {
  const p = (prompt || '').trim().toLowerCase()
  if (!p) return null
  for (const demo of REGISTRY) {
    for (const trig of demo.triggers) {
      if (p.includes(trig)) return demo
    }
  }
  return null
}

export function getActiveDemo(): Demo | null {
  return _active?.demo ?? null
}

/** Load the demo's URDF text from /demos/. Throws on fetch failure.
 *  Appends a per-load cache-bust query param — Vite serves /public as static
 *  assets, so without this the WebView2 HTTP cache happily returns the prior
 *  copy after the file was edited (manifests as "my URDF changes don't show
 *  up even after hard reload"). */
export async function loadDemoUrdf(demo: Demo): Promise<string> {
  const res = await fetch(`/${demo.urdfPath}?t=${Date.now()}`, { cache: 'no-store' })
  if (!res.ok) throw new Error(`Failed to fetch ${demo.urdfPath}: HTTP ${res.status}`)
  return await res.text()
}

/** Load the demo's pre-baked STL text. Cached after first call. */
export async function loadDemoStl(demo: Demo): Promise<string> {
  const cached = _stlCache.get(demo.id)
  if (cached) return cached
  const res = await fetch(`/${demo.stlPath}`)
  if (!res.ok) throw new Error(`Failed to fetch ${demo.stlPath}: HTTP ${res.status}`)
  const text = await res.text()
  _stlCache.set(demo.id, text)
  return text
}

/** Activate a demo: snapshot rest poses + kick off the RAF playback (paused). */
export function activateDemo(demo: Demo, parsedRobot: ParsedRobot, robotRoot: THREE.Group): void {
  clearActiveDemo()
  const jointOrigins = new Map<string, OriginalPose>()
  const jointAxes = new Map<string, THREE.Vector3>()
  for (const [name, info] of parsedRobot.joints) {
    jointOrigins.set(name, {
      position: info.group.position.clone(),
      quaternion: info.group.quaternion.clone(),
    })
    jointAxes.set(name, info.axis.clone())
  }
  const stylingCleanup = demo.applyStyling ? demo.applyStyling(parsedRobot, robotRoot) : null
  _active = {
    demo,
    jointOrigins,
    jointAxes,
    rootOrigin: { position: robotRoot.position.clone(), quaternion: robotRoot.quaternion.clone() },
    stylingCleanup,
    startMs: performance.now(),
    rafId: null,
    running: false,
  }
}

export function clearActiveDemo(): void {
  if (_active?.rafId !== null && _active?.rafId !== undefined) {
    cancelAnimationFrame(_active.rafId)
  }
  _active?.stylingCleanup?.()
  _active = null
}

/** Start the animation loop. parsedRobot/robotRoot must match what was passed
 *  to activateDemo (same scene), otherwise reactivate first. */
export function playActiveDemo(parsedRobot: ParsedRobot, robotRoot: THREE.Group): boolean {
  if (!_active) return false
  if (_active.running) return true
  _active.startMs = performance.now()
  _active.running = true
  const tick = () => {
    if (!_active || !_active.running) return
    applyDemoFrame(parsedRobot, robotRoot, (performance.now() - _active.startMs) / 1000)
    _active.rafId = requestAnimationFrame(tick)
  }
  _active.rafId = requestAnimationFrame(tick)
  return true
}

export function pauseActiveDemo(): void {
  if (!_active) return
  _active.running = false
  if (_active.rafId !== null) cancelAnimationFrame(_active.rafId)
  _active.rafId = null
}

/** Reset all joints + body to their captured rest pose and stop playback. */
export function resetActiveDemo(parsedRobot: ParsedRobot, robotRoot: THREE.Group): void {
  if (!_active) return
  pauseActiveDemo()
  for (const [name, info] of parsedRobot.joints) {
    const orig = _active.jointOrigins.get(name)
    if (orig) {
      info.group.position.copy(orig.position)
      info.group.quaternion.copy(orig.quaternion)
    }
  }
  if (_active.rootOrigin) {
    robotRoot.position.copy(_active.rootOrigin.position)
    robotRoot.quaternion.copy(_active.rootOrigin.quaternion)
  }
}

export function isDemoRunning(): boolean {
  return !!_active?.running
}

// ── Per-frame application ───────────────────────────────────────────────────

const _qDelta = new THREE.Quaternion()

function applyDemoFrame(parsedRobot: ParsedRobot, robotRoot: THREE.Group, tSec: number): void {
  if (!_active) return
  const demo = _active.demo
  for (const kf of demo.keyframes) {
    const orig = _active.jointOrigins.get(kf.jointName)
    const axis = _active.jointAxes.get(kf.jointName)
    if (!orig || !axis) continue
    const info = parsedRobot.joints.get(kf.jointName)
    if (!info) continue
    const mode = kf.mode ?? 'sine'
    const angle = mode === 'spin'
      ? (kf.biasRad ?? 0) + 2 * Math.PI * kf.freqHz * tSec + kf.phaseRad
      : (kf.biasRad ?? 0) + kf.amplitudeRad * Math.sin(2 * Math.PI * kf.freqHz * tSec + kf.phaseRad)
    _qDelta.setFromAxisAngle(axis, angle)
    info.group.quaternion.copy(orig.quaternion).multiply(_qDelta)
  }
  if (_active.rootOrigin && demo.bodyBobAmplitudeM) {
    // Bob at 2x stride hz — feet plant twice per stride cycle.
    const bob = demo.bodyBobAmplitudeM * Math.sin(4 * Math.PI * (1 / demo.loopPeriodSec) * tSec)
    robotRoot.position.y = _active.rootOrigin.position.y + bob
  }
  if (_active.rootOrigin && demo.forwardDriftMps) {
    robotRoot.position.z = _active.rootOrigin.position.z + demo.forwardDriftMps * tSec
  }
}
