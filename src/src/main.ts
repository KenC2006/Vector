import './style.css'
import * as THREE from 'three'
import { OrbitControls } from 'three/addons/controls/OrbitControls.js'
import { SnapGrid, SNAP_SIZES } from './snapGrid'
import { LINK_DETAILS } from './robotData'
import { initNodes, detachPartFromNode, attachPartToNode } from './nodeManager'
import { initInspector, updateNodeRow } from './inspector'
import { initToolbox, attemptAttachSelectedPart } from './toolbox'
import { initSelection, programmaticSelectLink, programmaticSelectNode, programmaticDeselect, getSelectedNodeId } from './selectionManager'

void `<?xml version="1.0"?>
<!-- Vector — 3-DOF Robot Arm -->
<robot name="simple_arm" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- ════════════════════════════════════════════ -->
  <!-- base_link: fixed mounting plate              -->
  <!-- mass: 1.2 kg | steel, 300mm dia, 50mm tall  -->
  <!-- ════════════════════════════════════════════ -->
  <link name="base_link">
    <inertial>
      <mass value="1.2"/>
      <origin xyz="0 0 0.025" rpy="0 0 0"/>
      <inertia ixx="0.003" iyy="0.003" izz="0.005"
               ixy="0" ixz="0" iyz="0"/>
    </inertial>
    <visual>
      <geometry>
        <cylinder radius="0.15" length="0.05"/>
      </geometry>
      <material name="dark_steel">
        <color rgba="0.18 0.2 0.25 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.15" length="0.05"/>
      </geometry>
    </collision>
  </link>

  <!-- ════════════════════════════════════════════ -->
  <!-- shoulder_link: servo housing                 -->
  <!-- actuator: Dynamixel XM540 — 10 Nm peak      -->
  <!-- ════════════════════════════════════════════ -->
  <link name="shoulder_link">
    <inertial>
      <mass value="0.18"/>
      <inertia ixx="0.0001" iyy="0.0001" izz="0.0001"
               ixy="0" ixz="0" iyz="0"/>
    </inertial>
    <visual>
      <geometry>
        <cylinder radius="0.035" length="0.055"/>
      </geometry>
      <material name="servo_black">
        <color rgba="0.12 0.12 0.14 1"/>
      </material>
    </visual>
  </link>

  <!-- joint: shoulder_pan — rotates around Z -->
  <joint name="shoulder_pan" type="revolute">
    <parent link="base_link"/>
    <child link="shoulder_link"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14"
           effort="10.0" velocity="2.0"/>
    <dynamics damping="0.5" friction="0.1"/>
  </joint>

  <!-- ════════════════════════════════════════════ -->
  <!-- upper_arm: 350mm aluminum tube, 0.4 kg      -->
  <!-- cross-section: 70×70mm square tube           -->
  <!-- ════════════════════════════════════════════ -->
  <link name="upper_arm">
    <inertial>
      <mass value="0.4"/>
      <origin xyz="0 0 0.175" rpy="0 0 0"/>
      <inertia ixx="0.004" iyy="0.004" izz="0.0005"
               ixy="0" ixz="0" iyz="0"/>
    </inertial>
    <visual>
      <geometry>
        <box size="0.065 0.065 0.35"/>
      </geometry>
      <material name="arm_blue">
        <color rgba="0.16 0.43 0.85 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.065 0.065 0.35"/>
      </geometry>
    </collision>
  </link>

  <!-- joint: shoulder_lift — rotates around Y -->
  <joint name="shoulder_lift" type="revolute">
    <parent link="shoulder_link"/>
    <child link="upper_arm"/>
    <origin xyz="0 0 0.03" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="2.36"
           effort="10.0" velocity="1.5"/>
    <dynamics damping="0.7" friction="0.15"/>
  </joint>

  <!-- ════════════════════════════════════════════ -->
  <!-- elbow_link: servo housing                    -->
  <!-- actuator: Dynamixel XH430 — 1.5 Nm          -->
  <!-- ════════════════════════════════════════════ -->
  <link name="elbow_link">
    <inertial>
      <mass value="0.12"/>
      <inertia ixx="0.00005" iyy="0.00005" izz="0.00005"
               ixy="0" ixz="0" iyz="0"/>
    </inertial>
    <visual>
      <geometry>
        <cylinder radius="0.03" length="0.05"/>
      </geometry>
      <material name="servo_black"/>
    </visual>
  </link>

  <joint name="elbow" type="revolute">
    <parent link="upper_arm"/>
    <child link="elbow_link"/>
    <origin xyz="0 0 0.35" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-2.36" upper="2.36"
           effort="1.5" velocity="2.5"/>
    <dynamics damping="0.3" friction="0.05"/>
  </joint>

  <!-- ════════════════════════════════════════════ -->
  <!-- forearm: 280mm, thinner, 0.25 kg             -->
  <!-- ════════════════════════════════════════════ -->
  <link name="forearm">
    <inertial>
      <mass value="0.25"/>
      <origin xyz="0 0 0.14" rpy="0 0 0"/>
      <inertia ixx="0.002" iyy="0.002" izz="0.0003"
               ixy="0" ixz="0" iyz="0"/>
    </inertial>
    <visual>
      <geometry>
        <box size="0.055 0.055 0.28"/>
      </geometry>
      <material name="arm_blue"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.055 0.055 0.28"/>
      </geometry>
    </collision>
  </link>

  <joint name="forearm_attach" type="fixed">
    <parent link="elbow_link"/>
    <child link="forearm"/>
    <origin xyz="0 0 0.03" rpy="0 0 0"/>
  </joint>

  <!-- ════════════════════════════════════════════ -->
  <!-- wrist + gripper                              -->
  <!-- parallel jaw, 85mm max opening, 40N grip     -->
  <!-- ════════════════════════════════════════════ -->
  <link name="wrist">
    <inertial>
      <mass value="0.08"/>
      <inertia ixx="0.00002" iyy="0.00002" izz="0.00002"
               ixy="0" ixz="0" iyz="0"/>
    </inertial>
    <visual>
      <geometry>
        <sphere radius="0.025"/>
      </geometry>
      <material name="servo_black"/>
    </visual>
  </link>

  <joint name="wrist_attach" type="fixed">
    <parent link="forearm"/>
    <child link="wrist"/>
    <origin xyz="0 0 0.28" rpy="0 0 0"/>
  </joint>

  <link name="gripper_base">
    <inertial>
      <mass value="0.06"/>
      <inertia ixx="0.00001" iyy="0.00001" izz="0.00001"
               ixy="0" ixz="0" iyz="0"/>
    </inertial>
    <visual>
      <geometry>
        <box size="0.06 0.02 0.03"/>
      </geometry>
      <material name="dark_steel"/>
    </visual>
  </link>

  <joint name="gripper_attach" type="fixed">
    <parent link="wrist"/>
    <child link="gripper_base"/>
    <origin xyz="0 0 0.025" rpy="0 0 0"/>
  </joint>

  <link name="finger_left">
    <inertial>
      <mass value="0.02"/>
      <inertia ixx="0.000005" iyy="0.000005" izz="0.000005"
               ixy="0" ixz="0" iyz="0"/>
    </inertial>
    <visual>
      <geometry>
        <box size="0.008 0.06 0.025"/>
      </geometry>
      <material name="arm_blue"/>
    </visual>
  </link>

  <joint name="finger_left_joint" type="prismatic">
    <parent link="gripper_base"/>
    <child link="finger_left"/>
    <origin xyz="-0.02 0.04 0" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-0.01" upper="0.02" effort="40" velocity="0.1"/>
  </joint>

  <link name="finger_right">
    <inertial>
      <mass value="0.02"/>
      <inertia ixx="0.000005" iyy="0.000005" izz="0.000005"
               ixy="0" ixz="0" iyz="0"/>
    </inertial>
    <visual>
      <geometry>
        <box size="0.008 0.06 0.025"/>
      </geometry>
      <material name="arm_blue"/>
    </visual>
  </link>

  <joint name="finger_right_joint" type="prismatic">
    <parent link="gripper_base"/>
    <child link="finger_right"/>
    <origin xyz="0.02 0.04 0" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-0.02" upper="0.01" effort="40" velocity="0.1"/>
  </joint>

</robot>`

// ── Three.js ─────────────────────────────────────────────────────────────────

const canvas = document.getElementById('viewport') as HTMLCanvasElement
const viewportPanel = document.getElementById('viewport-panel') as HTMLDivElement

const renderer = new THREE.WebGLRenderer({ canvas, antialias: true, alpha: false })
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2))
renderer.setClearColor(0x0c0c0c)
renderer.shadowMap.enabled = true
renderer.shadowMap.type = THREE.PCFSoftShadowMap
renderer.toneMapping = THREE.ACESFilmicToneMapping
renderer.toneMappingExposure = 1.1

const scene = new THREE.Scene()

// Subtle fog for depth
scene.fog = new THREE.FogExp2(0x0c0c0c, 0.3)

const camera = new THREE.PerspectiveCamera(50, 1, 0.01, 100)
camera.position.set(1.2, 1.0, 1.6)

const controls = new OrbitControls(camera, canvas)
controls.enableDamping = true
controls.dampingFactor = 0.06
controls.target.set(0, 0.35, 0)
controls.minDistance = 0.3
controls.maxDistance = 8

// ── Snap grid ─────────────────────────────────────────────────────────────────

const snapGrid = new SnapGrid(scene)

// Re-centre grid dots when camera moves
controls.addEventListener('change', () => {
  snapGrid.updateAround(controls.target)
})

// ── Scene setup ──────────────────────────────────────────────────────────────

// Grid
const grid = new THREE.GridHelper(8, 40, 0x3a3a44, 0x2a2a32)
scene.add(grid)

// Ground shadow receiver
const groundMesh = new THREE.Mesh(
  new THREE.PlaneGeometry(8, 8),
  new THREE.ShadowMaterial({ opacity: 0.25 })
)
groundMesh.rotation.x = -Math.PI / 2
groundMesh.receiveShadow = true
scene.add(groundMesh)

// Origin axes
const originAxes = new THREE.AxesHelper(0.5)
scene.add(originAxes)

// Lights
scene.add(new THREE.AmbientLight(0xc8cce0, 0.4))

const keyLight = new THREE.DirectionalLight(0xffffff, 1.2)
keyLight.position.set(3, 6, 4)
keyLight.castShadow = true
keyLight.shadow.mapSize.set(2048, 2048)
keyLight.shadow.camera.near = 0.5
keyLight.shadow.camera.far = 20
keyLight.shadow.camera.left = -3
keyLight.shadow.camera.right = 3
keyLight.shadow.camera.top = 3
keyLight.shadow.camera.bottom = -3
keyLight.shadow.bias = -0.0005
scene.add(keyLight)

const fillLight = new THREE.DirectionalLight(0x6688cc, 0.4)
fillLight.position.set(-3, 2, -2)
scene.add(fillLight)

const rimLight = new THREE.DirectionalLight(0x8888ff, 0.25)
rimLight.position.set(0, 0.5, -4)
scene.add(rimLight)

// ── Materials ────────────────────────────────────────────────────────────────

const steelMat = new THREE.MeshStandardMaterial({
  color: 0x2a2e38, roughness: 0.4, metalness: 0.7,
})
const armMat = new THREE.MeshStandardMaterial({
  color: 0x2a6dd9, roughness: 0.35, metalness: 0.5,
})
const servoMat = new THREE.MeshStandardMaterial({
  color: 0x1a1a22, roughness: 0.5, metalness: 0.6,
})
const gripperMat = new THREE.MeshStandardMaterial({
  color: 0x3a7ae8, roughness: 0.3, metalness: 0.5,
})
const jointAccentMat = new THREE.MeshStandardMaterial({
  color: 0xff7b45, roughness: 0.4, metalness: 0.3, emissive: 0x331500,
})
const comMat = new THREE.MeshStandardMaterial({
  color: 0xe5c07b, roughness: 0.3, metalness: 0.2, emissive: 0x665500,
})
const wireMat = new THREE.MeshBasicMaterial({
  color: 0x4a9eff, wireframe: true, transparent: true, opacity: 0.12,
})

// ── Build robot (hierarchical) ───────────────────────────────────────────────

const robot = new THREE.Group()
scene.add(robot)

function makeShadowed(mesh: THREE.Mesh) {
  mesh.castShadow = true
  mesh.receiveShadow = true
  return mesh
}

// Base
const baseGroup = new THREE.Group()
robot.add(baseGroup)

const basePlate = makeShadowed(new THREE.Mesh(new THREE.CylinderGeometry(0.15, 0.16, 0.05, 48), steelMat))
basePlate.position.y = 0.025
baseGroup.add(basePlate)

// Base ring accent
const baseRing = new THREE.Mesh(new THREE.TorusGeometry(0.15, 0.005, 8, 48), jointAccentMat)
baseRing.rotation.x = Math.PI / 2
baseRing.position.y = 0.05
baseGroup.add(baseRing)

// Shoulder pivot (rotates around Y via shoulder_pan)
const shoulderPivot = new THREE.Group()
shoulderPivot.position.y = 0.05
baseGroup.add(shoulderPivot)

// Shoulder servo housing
const shoulderServo = makeShadowed(new THREE.Mesh(new THREE.CylinderGeometry(0.035, 0.035, 0.055, 24), servoMat))
shoulderServo.position.y = 0.0275
shoulderPivot.add(shoulderServo)

// Shoulder joint ring
const shoulderRing = new THREE.Mesh(new THREE.TorusGeometry(0.038, 0.004, 8, 24), jointAccentMat)
shoulderRing.rotation.x = Math.PI / 2
shoulderRing.position.y = 0.02
shoulderPivot.add(shoulderRing)

// Upper arm pivot (rotates around X for shoulder_lift)
const upperArmPivot = new THREE.Group()
upperArmPivot.position.y = 0.055
shoulderPivot.add(upperArmPivot)

// Upper arm
const upperArmMesh = makeShadowed(new THREE.Mesh(new THREE.BoxGeometry(0.065, 0.35, 0.065), armMat))
upperArmMesh.position.y = 0.175
upperArmPivot.add(upperArmMesh)

// Edge highlight strips on upper arm
const stripGeo = new THREE.BoxGeometry(0.002, 0.35, 0.067)
const stripMat = new THREE.MeshBasicMaterial({ color: 0x4a9eff, transparent: true, opacity: 0.15 })
const stripL = new THREE.Mesh(stripGeo, stripMat)
stripL.position.set(-0.033, 0.175, 0)
upperArmPivot.add(stripL)
const stripR = new THREE.Mesh(stripGeo, stripMat)
stripR.position.set(0.033, 0.175, 0)
upperArmPivot.add(stripR)

// Elbow pivot
const elbowPivot = new THREE.Group()
elbowPivot.position.y = 0.35
upperArmPivot.add(elbowPivot)

// Elbow servo
const elbowServo = makeShadowed(new THREE.Mesh(new THREE.CylinderGeometry(0.03, 0.03, 0.05, 24), servoMat))
elbowServo.position.y = 0.025
elbowPivot.add(elbowServo)

const elbowRing = new THREE.Mesh(new THREE.TorusGeometry(0.033, 0.004, 8, 24), jointAccentMat)
elbowRing.rotation.x = Math.PI / 2
elbowRing.position.y = 0.015
elbowPivot.add(elbowRing)

// Forearm
const forearmPivot = new THREE.Group()
forearmPivot.position.y = 0.05
elbowPivot.add(forearmPivot)

const forearmMesh = makeShadowed(new THREE.Mesh(new THREE.BoxGeometry(0.055, 0.28, 0.055), armMat))
forearmMesh.position.y = 0.14
forearmPivot.add(forearmMesh)

// Forearm strips
const fstripGeo = new THREE.BoxGeometry(0.002, 0.28, 0.057)
const fstripL = new THREE.Mesh(fstripGeo, stripMat)
fstripL.position.set(-0.028, 0.14, 0)
forearmPivot.add(fstripL)
const fstripR = new THREE.Mesh(fstripGeo, stripMat)
fstripR.position.set(0.028, 0.14, 0)
forearmPivot.add(fstripR)

// Wrist
const wrist = makeShadowed(new THREE.Mesh(new THREE.SphereGeometry(0.022, 16, 16), servoMat))
wrist.position.y = 0.28
forearmPivot.add(wrist)

// Gripper base
const gripperBase = makeShadowed(new THREE.Mesh(new THREE.BoxGeometry(0.06, 0.025, 0.035), steelMat))
gripperBase.position.y = 0.305
forearmPivot.add(gripperBase)

// Fingers
const fingerGeo = new THREE.BoxGeometry(0.008, 0.055, 0.025)
const fingerL = makeShadowed(new THREE.Mesh(fingerGeo, gripperMat))
fingerL.position.set(-0.022, 0.34, 0)
forearmPivot.add(fingerL)

const fingerR = makeShadowed(new THREE.Mesh(fingerGeo, gripperMat))
fingerR.position.set(0.022, 0.34, 0)
forearmPivot.add(fingerR)

// Finger tips (accent)
const tipGeo = new THREE.BoxGeometry(0.01, 0.008, 0.027)
const tipL = new THREE.Mesh(tipGeo, jointAccentMat)
tipL.position.set(-0.022, 0.37, 0)
forearmPivot.add(tipL)
const tipR = new THREE.Mesh(tipGeo, jointAccentMat)
tipR.position.set(0.022, 0.37, 0)
forearmPivot.add(tipR)

// ── Wireframe overlay ────────────────────────────────────────────────────────

const wireframeGroup = new THREE.Group()
wireframeGroup.visible = false
robot.add(wireframeGroup)

// We'll rebuild wireframes after first render in animate

// ── Joint axis lines ─────────────────────────────────────────────────────────

const axisVisuals = new THREE.Group()
robot.add(axisVisuals)

function addJointAxis(parent: THREE.Object3D, dir: THREE.Vector3, color: number) {
  const pts = [
    new THREE.Vector3().copy(dir).multiplyScalar(-0.12),
    new THREE.Vector3().copy(dir).multiplyScalar(0.12),
  ]
  const geo = new THREE.BufferGeometry().setFromPoints(pts)
  const mat = new THREE.LineBasicMaterial({ color, transparent: true, opacity: 0.5 })
  const line = new THREE.Line(geo, mat)
  parent.add(line)
  axisVisuals.children // just to reference

  const ring = new THREE.Mesh(
    new THREE.TorusGeometry(0.05, 0.002, 8, 32),
    new THREE.MeshBasicMaterial({ color, transparent: true, opacity: 0.15 })
  )
  if (Math.abs(dir.z) > 0.5) { /* default orientation */ }
  else if (Math.abs(dir.y) > 0.5) ring.rotation.x = Math.PI / 2
  else ring.rotation.z = Math.PI / 2
  parent.add(ring)
}

addJointAxis(shoulderPivot, new THREE.Vector3(0, 0, 1), 0x4a9eff)  // shoulder pan: Z
addJointAxis(upperArmPivot, new THREE.Vector3(0, 1, 0), 0x4ec9b0)  // shoulder lift: Y
addJointAxis(elbowPivot, new THREE.Vector3(0, 1, 0), 0x4ec9b0)     // elbow: Y

// ── CoM marker ───────────────────────────────────────────────────────────────

const comGroup = new THREE.Group()
robot.add(comGroup)

const totalMass = 1.2 + 0.18 + 0.4 + 0.12 + 0.25 + 0.08 + 0.06 + 0.04
const comY = (1.2*0.025 + 0.18*0.08 + 0.4*0.28 + 0.12*0.46 + 0.25*0.55 + 0.08*0.7 + 0.06*0.72 + 0.04*0.75) / totalMass

const comMarker = new THREE.Mesh(new THREE.OctahedronGeometry(0.045), comMat)
comMarker.position.set(0, comY, 0)
comGroup.add(comMarker)

const comLinePts = [new THREE.Vector3(0, comY, 0), new THREE.Vector3(0, 0, 0)]
const comLineGeo = new THREE.BufferGeometry().setFromPoints(comLinePts)
const comLineMat = new THREE.LineDashedMaterial({ color: 0xe5c07b, dashSize: 0.02, gapSize: 0.01, transparent: true, opacity: 0.7 })
const comLine = new THREE.Line(comLineGeo, comLineMat)
comLine.computeLineDistances()
comGroup.add(comLine)

// CoM label ring
const comRingGround = new THREE.Mesh(
  new THREE.RingGeometry(0.035, 0.045, 24),
  new THREE.MeshBasicMaterial({ color: 0xe5c07b, transparent: true, opacity: 0.4, side: THREE.DoubleSide })
)
comRingGround.rotation.x = -Math.PI / 2
comRingGround.position.y = 0.001
comGroup.add(comRingGround)

// ── Viewport controls ────────────────────────────────────────────────────────

const toggleAxesBtn = document.getElementById('toggle-axes') as HTMLButtonElement
const toggleComBtn = document.getElementById('toggle-com') as HTMLButtonElement
const toggleWireBtn = document.getElementById('toggle-wireframe') as HTMLButtonElement
const toggleGridBtn = document.getElementById('toggle-grid') as HTMLButtonElement

let axesVisible = true
let gridVisible = true

toggleAxesBtn.addEventListener('click', () => {
  axesVisible = !axesVisible
  originAxes.visible = axesVisible
  axisVisuals.visible = axesVisible
  toggleAxesBtn.classList.toggle('active', axesVisible)
})

toggleComBtn.addEventListener('click', () => {
  comGroup.visible = !comGroup.visible
  toggleComBtn.classList.toggle('active', comGroup.visible)
})

toggleWireBtn.addEventListener('click', () => {
  wireframeGroup.visible = !wireframeGroup.visible
  toggleWireBtn.classList.toggle('active', wireframeGroup.visible)
})

toggleGridBtn.classList.add('active')
toggleGridBtn.addEventListener('click', () => {
  gridVisible = !gridVisible
  grid.visible = gridVisible
  toggleGridBtn.classList.toggle('active', gridVisible)
})

// ── Sim mode ─────────────────────────────────────────────────────────────────

const simToggle = document.getElementById('sim-toggle') as HTMLButtonElement
const simBar = document.getElementById('sim-bar') as HTMLDivElement
const simPlay = document.getElementById('sim-play') as HTMLButtonElement
const simPause = document.getElementById('sim-pause') as HTMLButtonElement
const simReset = document.getElementById('sim-reset') as HTMLButtonElement
const simProgress = document.getElementById('sim-progress') as HTMLDivElement
const simTimeEl = document.getElementById('sim-time') as HTMLSpanElement
const viewportLabel = document.getElementById('viewport-label') as HTMLSpanElement

let simRunning = false
let simActive = false
let simTime = 0

// Original sim event listeners removed — replaced by persistent-core versions below

function updateSimUI() {
  simPlay.classList.toggle('active', simRunning)
  simPause.classList.toggle('active', !simRunning && simActive)
  simTimeEl.textContent = simTime.toFixed(3) + 's'
  simProgress.style.width = `${Math.min((simTime / 10) * 100, 100)}%`
}

// ── Resize ───────────────────────────────────────────────────────────────────

function resize() {
  const header = document.getElementById('viewport-header')!
  const w = viewportPanel.clientWidth
  const h = viewportPanel.clientHeight - header.offsetHeight - (simActive ? simBar.offsetHeight : 0)
  if (w > 0 && h > 0) {
    renderer.setSize(w, h)
    camera.aspect = w / h
    camera.updateProjectionMatrix()
  }
}
resize()
window.addEventListener('resize', resize)

// ── Animate ──────────────────────────────────────────────────────────────────

let wireframeBuilt = false

function animate() {
  requestAnimationFrame(animate)

  // Build wireframes once
  if (!wireframeBuilt) {
    wireframeBuilt = true
    robot.traverse(child => {
      if (child instanceof THREE.Mesh && child.material !== wireMat && child.geometry) {
        const clone = new THREE.Mesh(child.geometry, wireMat)
        child.getWorldPosition(clone.position)
        child.getWorldQuaternion(clone.quaternion)
        wireframeGroup.add(clone)
      }
    })
  }

  controls.update()

  // CoM diamond gentle spin
  comMarker.rotation.y += 0.01

  // Sim animation
  if (simRunning) {
    simTime += 1 / 60
    updateSimUI()
    const t = simTime

    // Smooth sinusoidal motion across joints
    shoulderPivot.rotation.y = Math.sin(t * 0.8) * 0.6
    upperArmPivot.rotation.x = Math.sin(t * 0.6 + 0.5) * 0.3 - 0.2
    elbowPivot.rotation.x = Math.sin(t * 1.2) * 0.5 + 0.3

    // Gripper open/close cycle
    const grip = Math.sin(t * 2) * 0.5 + 0.5 // 0 to 1
    fingerL.position.x = -0.022 - grip * 0.012
    fingerR.position.x = 0.022 + grip * 0.012
    tipL.position.x = fingerL.position.x
    tipR.position.x = fingerR.position.x
  }

  renderer.render(scene, camera)
}
animate()


// ── Kinematic Graph Data Structure ──────────────────────────────────────────

interface KinematicLink {
  name: string
  mass: number
  parent?: string
  children: string[]
}

interface KinematicJoint {
  name: string
  type: string
  axis: string
  parentLink: string
  childLink: string
}

// Define the robot kinematic structure based on SAMPLE_URDF
const kinematicGraph: Record<string, KinematicLink> = {
  'base_link': {
    name: 'base_link',
    mass: 1.2,
    children: ['shoulder_link'],
  },
  'shoulder_link': {
    name: 'shoulder_link',
    mass: 0.18,
    parent: 'base_link',
    children: ['upper_arm'],
  },
  'upper_arm': {
    name: 'upper_arm',
    mass: 0.4,
    parent: 'shoulder_link',
    children: ['elbow_link'],
  },
  'elbow_link': {
    name: 'elbow_link',
    mass: 0.12,
    parent: 'upper_arm',
    children: ['forearm'],
  },
  'forearm': {
    name: 'forearm',
    mass: 0.25,
    parent: 'elbow_link',
    children: ['wrist'],
  },
  'wrist': {
    name: 'wrist',
    mass: 0.08,
    parent: 'forearm',
    children: ['gripper_base'],
  },
  'gripper_base': {
    name: 'gripper_base',
    mass: 0.06,
    parent: 'wrist',
    children: ['finger_left', 'finger_right'],
  },
  'finger_left': {
    name: 'finger_left',
    mass: 0.02,
    parent: 'gripper_base',
    children: [],
  },
  'finger_right': {
    name: 'finger_right',
    mass: 0.02,
    parent: 'gripper_base',
    children: [],
  },
}

const kinematicJoints: Record<string, KinematicJoint> = {
  'shoulder_pan': {
    name: 'shoulder_pan',
    type: 'revolute',
    axis: 'Z',
    parentLink: 'base_link',
    childLink: 'shoulder_link',
  },
  'shoulder_lift': {
    name: 'shoulder_lift',
    type: 'revolute',
    axis: 'Y',
    parentLink: 'shoulder_link',
    childLink: 'upper_arm',
  },
  'elbow': {
    name: 'elbow',
    type: 'revolute',
    axis: 'Y',
    parentLink: 'upper_arm',
    childLink: 'elbow_link',
  },
  'forearm_attach': {
    name: 'forearm_attach',
    type: 'fixed',
    axis: '--',
    parentLink: 'elbow_link',
    childLink: 'forearm',
  },
  'wrist_attach': {
    name: 'wrist_attach',
    type: 'fixed',
    axis: '--',
    parentLink: 'forearm',
    childLink: 'wrist',
  },
  'gripper_attach': {
    name: 'gripper_attach',
    type: 'fixed',
    axis: '--',
    parentLink: 'wrist',
    childLink: 'gripper_base',
  },
  'finger_left_joint': {
    name: 'finger_left_joint',
    type: 'prismatic',
    axis: 'X',
    parentLink: 'gripper_base',
    childLink: 'finger_left',
  },
  'finger_right_joint': {
    name: 'finger_right_joint',
    type: 'prismatic',
    axis: 'X',
    parentLink: 'gripper_base',
    childLink: 'finger_right',
  },
}

// Mapping from link names to Three.js meshes for highlighting
const meshMap: Record<string, THREE.Mesh | THREE.Group> = {
  'base_link': basePlate,
  'shoulder_link': shoulderServo,
  'upper_arm': upperArmMesh,
  'elbow_link': elbowServo,
  'forearm': forearmMesh,
  'wrist': wrist,
  'gripper_base': gripperBase,
  'finger_left': fingerL,
  'finger_right': fingerR,
}


// ── Build Kinematic Tree UI ──────────────────────────────────────────────────

function buildKinematicTreeUI() {
  const treeContainer = document.getElementById('kinematic-tree')!
  treeContainer.innerHTML = ''

  function buildNode(linkName: string, depth: number = 0) {
    const link = kinematicGraph[linkName]
    if (!link) return

    const nodeEl = document.createElement('div')
    nodeEl.className = 'kt-node link'
    nodeEl.style.paddingLeft = `${10 + depth * 12}px`

    const hasChildren = link.children.length > 0
    const toggleEl = document.createElement('div')
    toggleEl.className = `kt-toggle ${hasChildren ? 'expanded' : ''}`
    if (!hasChildren) toggleEl.style.opacity = '0'

    const labelEl = document.createElement('span')
    labelEl.textContent = `${link.name} (${link.mass} kg)`

    nodeEl.appendChild(toggleEl)
    nodeEl.appendChild(labelEl)

    // Click to select in 3D and open inspector
    labelEl.style.cursor = 'pointer'
    labelEl.addEventListener('click', (e) => {
      e.stopPropagation()
      programmaticSelectLink(linkName)
    })

    // Toggle expand/collapse
    if (hasChildren) {
      toggleEl.style.cursor = 'pointer'
      toggleEl.addEventListener('click', (e) => {
        e.stopPropagation()
        const childrenDiv = nodeEl.nextElementSibling
        if (childrenDiv && childrenDiv.classList.contains('kt-children')) {
          childrenDiv.classList.toggle('visible')
          toggleEl.classList.toggle('expanded')
          toggleEl.classList.toggle('collapsed')
        }
      })
    }

    treeContainer.appendChild(nodeEl)

    // Add joints and children
    if (hasChildren) {
      const childrenDiv = document.createElement('div')
      childrenDiv.className = 'kt-children visible'

      for (const childName of link.children) {
        // Find the joint connecting to this child
        for (const joint of Object.values(kinematicJoints)) {
          if (joint.parentLink === linkName && joint.childLink === childName) {
            const jointEl = document.createElement('div')
            jointEl.className = 'kt-node joint'
            jointEl.style.paddingLeft = `${30 + depth * 12}px`
            jointEl.textContent = `↳ ${joint.name} [${joint.type}, ${joint.axis}]`

            // Show tooltip on hover
            jointEl.addEventListener('mouseenter', () => {
              jointEl.title = `Type: ${joint.type}\nAxis: ${joint.axis}\nParent: ${joint.parentLink}\nChild: ${joint.childLink}`
            })

            childrenDiv.appendChild(jointEl)
            break
          }
        }

        // Recursively add child link
        const tempDiv = document.createElement('div')
        treeContainer.appendChild(tempDiv)
        const oldAppend = treeContainer.appendChild
        treeContainer.appendChild = function (el: any) {
          tempDiv.parentElement!.insertBefore(el, tempDiv.nextSibling)
          return el
        }
        buildNode(childName, depth + 1)
        treeContainer.appendChild = oldAppend
        tempDiv.remove()
      }

      treeContainer.appendChild(childrenDiv)
    }
  }

  buildNode('base_link')
}

buildKinematicTreeUI()

// ── Node Graph Visualization ────────────────────────────────────────────────

let graphCanvasVisible = false
let graphCanvas: HTMLCanvasElement | null = null
let graphCtx: CanvasRenderingContext2D | null = null
let graphContainer: HTMLDivElement | null = null
let graphEventsAttached = false

interface GraphNode {
  linkName: string
  x: number
  y: number
  width: number
  height: number
}

let graphNodes: GraphNode[] = []

function buildNodeGraph() {
  const dpr = window.devicePixelRatio || 1
  const viewWidth = viewportPanel.clientWidth
  const viewHeight = viewportPanel.clientHeight

  // Layout: top-to-bottom tree
  const nodeWidth = 120
  const nodeHeight = 54
  const levelHeight = 110
  const paddingTop = 50
  const paddingBottom = 30

  // Calculate max tree depth first
  let maxDepth = 0
  function calcDepth(linkName: string, depth: number) {
    if (depth > maxDepth) maxDepth = depth
    const link = kinematicGraph[linkName]
    if (link) {
      for (const child of link.children) {
        calcDepth(child, depth + 1)
      }
    }
  }
  calcDepth('base_link', 0)

  const contentHeight = Math.max(viewHeight, paddingTop + (maxDepth + 1) * levelHeight + paddingBottom)

  // Create scrollable container + canvas
  if (!graphContainer) {
    graphContainer = document.createElement('div')
    graphContainer.id = 'kinematic-graph-container'

    // Close button
    const closeBtn = document.createElement('button')
    closeBtn.textContent = '✕'
    closeBtn.title = 'Close graph (G)'
    closeBtn.style.cssText = `
      position: sticky; top: 8px; float: right; margin-right: 12px;
      z-index: 25; background: #333; color: #ccc; border: 1px solid #555;
      border-radius: 4px; width: 28px; height: 28px; cursor: pointer;
      font-size: 14px; line-height: 1; display: flex; align-items: center;
      justify-content: center;
    `
    closeBtn.addEventListener('click', () => {
      graphCanvasVisible = false
      if (graphContainer) graphContainer.style.display = 'none'
      toggleGraphBtn.classList.remove('active')
    })
    graphContainer.appendChild(closeBtn)

    graphCanvas = document.createElement('canvas')
    graphCanvas.id = 'kinematic-graph-canvas'
    graphContainer.appendChild(graphCanvas)
    viewportPanel.appendChild(graphContainer)
    graphCtx = graphCanvas.getContext('2d')!
  }

  // Set canvas size with DPI scaling
  graphCanvas!.width = viewWidth * dpr
  graphCanvas!.height = contentHeight * dpr
  graphCanvas!.style.width = viewWidth + 'px'
  graphCanvas!.style.height = contentHeight + 'px'
  graphCanvas!.style.display = 'block'

  graphCtx = graphCanvas!.getContext('2d')!
  graphCtx.setTransform(dpr, 0, 0, dpr, 0, 0)

  graphNodes = []

  // Count siblings at each depth level
  const levelCounts: Record<number, number> = {}
  function countLevels(linkName: string, depth: number) {
    levelCounts[depth] = (levelCounts[depth] || 0) + 1
    const link = kinematicGraph[linkName]
    if (link) {
      for (const child of link.children) {
        countLevels(child, depth + 1)
      }
    }
  }
  countLevels('base_link', 0)

  const levelIndexes: Record<number, number> = {}

  function layoutNode(linkName: string, depth: number) {
    if (!levelIndexes[depth]) levelIndexes[depth] = 0
    const indexInLevel = levelIndexes[depth]
    const total = levelCounts[depth] || 1

    const xSpacing = Math.max(nodeWidth + 30, viewWidth / (total + 1))
    const xOffset = (viewWidth - total * xSpacing) / 2 + xSpacing / 2
    const x = xOffset + indexInLevel * xSpacing
    const y = paddingTop + depth * levelHeight

    graphNodes.push({
      linkName,
      x: x - nodeWidth / 2,
      y: y - nodeHeight / 2,
      width: nodeWidth,
      height: nodeHeight,
    })

    levelIndexes[depth]++
  }

  // Recursively layout
  function walkLayout(linkName: string, depth: number) {
    layoutNode(linkName, depth)
    const link = kinematicGraph[linkName]
    if (link) {
      for (const child of link.children) {
        walkLayout(child, depth + 1)
      }
    }
  }

  walkLayout('base_link', 0)

  // Render graph
  if (graphCtx) {
    graphCtx.clearRect(0, 0, viewWidth, contentHeight)

    // Draw edges first
    graphCtx.strokeStyle = '#4ec9b0'
    graphCtx.lineWidth = 1.5
    graphCtx.globalAlpha = 0.6

    for (const [linkName, link] of Object.entries(kinematicGraph)) {
      for (const childName of link.children) {
        const parentNode = graphNodes.find((n) => n.linkName === linkName)
        const childNode = graphNodes.find((n) => n.linkName === childName)
        if (parentNode && childNode) {
          graphCtx.beginPath()
          graphCtx.moveTo(parentNode.x + parentNode.width / 2, parentNode.y + parentNode.height)
          graphCtx.lineTo(childNode.x + childNode.width / 2, childNode.y)
          graphCtx.stroke()

          // Draw joint label on edge
          const jx = (parentNode.x + parentNode.width / 2 + childNode.x + childNode.width / 2) / 2
          const jy = (parentNode.y + parentNode.height + childNode.y) / 2

          for (const joint of Object.values(kinematicJoints)) {
            if (joint.parentLink === linkName && joint.childLink === childName) {
              graphCtx.globalAlpha = 1
              graphCtx.fillStyle = '#e5c07b'
              graphCtx.font = '10px monospace'
              graphCtx.textAlign = 'center'
              graphCtx.fillText(joint.type, jx, jy - 2)
              graphCtx.globalAlpha = 0.6
              break
            }
          }
        }
      }
    }

    graphCtx.globalAlpha = 1

    // Draw nodes
    for (const node of graphNodes) {
      const link = kinematicGraph[node.linkName]

      // Node background
      graphCtx.fillStyle = '#1e1e22'
      graphCtx.strokeStyle = '#4ec9b0'
      graphCtx.lineWidth = 2

      // Draw rounded rectangle
      const r = 8
      graphCtx.beginPath()
      graphCtx.moveTo(node.x + r, node.y)
      graphCtx.lineTo(node.x + node.width - r, node.y)
      graphCtx.quadraticCurveTo(node.x + node.width, node.y, node.x + node.width, node.y + r)
      graphCtx.lineTo(node.x + node.width, node.y + node.height - r)
      graphCtx.quadraticCurveTo(node.x + node.width, node.y + node.height, node.x + node.width - r, node.y + node.height)
      graphCtx.lineTo(node.x + r, node.y + node.height)
      graphCtx.quadraticCurveTo(node.x, node.y + node.height, node.x, node.y + node.height - r)
      graphCtx.lineTo(node.x, node.y + r)
      graphCtx.quadraticCurveTo(node.x, node.y, node.x + r, node.y)
      graphCtx.closePath()
      graphCtx.fill()
      graphCtx.stroke()

      // Node text
      graphCtx.fillStyle = '#4ec9b0'
      graphCtx.font = 'bold 12px monospace'
      graphCtx.textAlign = 'center'
      graphCtx.textBaseline = 'top'
      graphCtx.fillText(node.linkName, node.x + node.width / 2, node.y + 8)

      // Mass label
      if (link) {
        graphCtx.fillStyle = '#aaa'
        graphCtx.font = '10px monospace'
        graphCtx.fillText(`${link.mass} kg`, node.x + node.width / 2, node.y + 28)
      }
    }
  }

  // Attach interactive events only once
  if (!graphEventsAttached && graphCanvas) {
    graphCanvas.addEventListener('mousemove', (e) => {
      const rect = graphCanvas!.getBoundingClientRect()
      const mx = e.clientX - rect.left
      const my = e.clientY - rect.top + (graphContainer?.scrollTop || 0)

      for (const node of graphNodes) {
        if (mx >= node.x && mx <= node.x + node.width && my >= node.y && my <= node.y + node.height) {
          graphCanvas!.style.cursor = 'pointer'
          return
        }
      }
      graphCanvas!.style.cursor = 'default'
    })

    graphCanvas.addEventListener('click', (e) => {
      const rect = graphCanvas!.getBoundingClientRect()
      const mx = e.clientX - rect.left
      const my = e.clientY - rect.top + (graphContainer?.scrollTop || 0)

      for (const node of graphNodes) {
        if (mx >= node.x && mx <= node.x + node.width && my >= node.y && my <= node.y + node.height) {
          programmaticSelectLink(node.linkName)
          break
        }
      }
    })

    graphEventsAttached = true
  }
}

const toggleGraphBtn = document.getElementById('toggle-graph') as HTMLButtonElement

toggleGraphBtn.addEventListener('click', () => {
  graphCanvasVisible = !graphCanvasVisible

  if (graphCanvasVisible) {
    if (!graphContainer) buildNodeGraph()
    else {
      graphContainer.style.display = 'block'
      buildNodeGraph() // rebuild to update layout
    }
  } else {
    if (graphContainer) graphContainer.style.display = 'none'
  }

  toggleGraphBtn.classList.toggle('active', graphCanvasVisible)
})

// Rebuild graph on window resize
window.addEventListener('resize', () => {
  if (graphContainer && graphCanvasVisible) {
    buildNodeGraph()
  }
})

// ── Keyboard shortcuts for viewport toggles ─────────────────────────────────
document.addEventListener('keydown', (e) => {
  // Don't trigger shortcuts when typing in inputs
  const tag = (e.target as HTMLElement).tagName
  if (tag === 'INPUT' || tag === 'TEXTAREA' || tag === 'SELECT') return

  switch (e.key.toLowerCase()) {
    case 'a':
      toggleAxesBtn.click()
      break
    case 'c':
      toggleComBtn.click()
      break
    case 'w':
      toggleWireBtn.click()
      break
    case 'g':
      if (!e.ctrlKey && !e.metaKey) toggleGridBtn.click()
      break
    case 'n':
      toggleGraphBtn.click()
      break
    case 'i':
      switchToPanel('inspector')
      break
    case 't':
      switchToPanel('toolbox')
      break
    case 'enter':
      // Attach selected toolbox part to selected node
      attemptAttachSelectedPart()
      break
    case 'escape':
      if (graphCanvasVisible) {
        graphCanvasVisible = false
        if (graphContainer) graphContainer.style.display = 'none'
        toggleGraphBtn.classList.remove('active')
      }
      programmaticDeselect()
      break
  }
})

// ── Activity bar ─────────────────────────────────────────────────────────────

const panels: Record<string, HTMLElement> = {
  kinematic: document.getElementById('panel-kinematic')!,
  build: document.getElementById('panel-build')!,
  inspector: document.getElementById('panel-inspector')!,
  toolbox: document.getElementById('panel-toolbox')!,
}

function switchToPanel(panelName: string) {
  document.querySelectorAll('.ab-btn').forEach(b => b.classList.remove('active'))
  Object.values(panels).forEach(p => p.classList.add('hidden'))
  const btn = document.querySelector(`.ab-btn[data-panel="${panelName}"]`) as HTMLElement | null
  if (btn) btn.classList.add('active')
  if (panels[panelName]) panels[panelName].classList.remove('hidden')
}

document.querySelectorAll('.ab-btn').forEach(btn => {
  btn.addEventListener('click', () => {
    const panel = (btn as HTMLElement).dataset.panel!
    if (!panel) return
    const wasActive = btn.classList.contains('active')
    document.querySelectorAll('.ab-btn').forEach(b => b.classList.remove('active'))
    Object.values(panels).forEach(p => p.classList.add('hidden'))
    if (!wasActive) {
      btn.classList.add('active')
      if (panels[panel]) panels[panel].classList.remove('hidden')
    }
  })
})


// ── Toast notifications ──────────────────────────────────────────────────────

const toastArea = document.getElementById('toast-area') as HTMLDivElement

function showToast(message: string, type: 'success' | 'warning' | 'error' | 'info' = 'info') {
  const toast = document.createElement('div')
  toast.className = `toast ${type}`
  toast.textContent = message
  toastArea.appendChild(toast)
  setTimeout(() => {
    toast.classList.add('fade-out')
    setTimeout(() => toast.remove(), 300)
  }, 3000)
}


// ── Mode indicator ────────────────────────────────────────────────────────────

const modeIndicator = document.getElementById('mode-indicator') as HTMLSpanElement

function updateModeIndicator(mode: 'Demo' | 'Build Mode' | 'Simulation') {
  if (modeIndicator) modeIndicator.textContent = mode
}

// ── Simulation Mode Integration ──────────────────────────────────────────────
import { invoke } from '@tauri-apps/api/core'

// Simulation state
let simCoreRunning = false
let simStepIntervalId: number | null = null

// Joint state display
const simStateDisplay = document.createElement('div')
simStateDisplay.id = 'sim-state-display'
simStateDisplay.className = 'sim-state-display'
simStateDisplay.style.cssText = `
  position: absolute;
  top: 48px;
  right: 12px;
  background: rgba(28, 28, 36, 0.95);
  border: 1px solid #4ec9b0;
  border-radius: 8px;
  padding: 12px;
  font-family: monospace;
  font-size: 11px;
  color: #e0e0e0;
  max-width: 240px;
  max-height: 300px;
  overflow-y: auto;
  z-index: 100;
  display: none;
  backdrop-filter: blur(8px);
`
viewportPanel.appendChild(simStateDisplay)

async function initializeSimulation() {
  try {
    console.log('[Sim] Initializing simulation core...')
    await invoke('start_core')
    simCoreRunning = true
    console.log('[Sim] Core started successfully')

    console.log('[Sim] Loading robot model...')
    await invoke('sim_load', { path: 'core/test_data/simple_arm.urdf' })
    console.log('[Sim] Robot model loaded')

    console.log('[Sim] Getting initial state...')
    const initialState = await invoke('sim_get_state')
    console.log('[Sim] Initial state:', initialState)

    // Show state display
    simStateDisplay.style.display = 'block'
    updateSimStateDisplay(initialState)
  } catch (error) {
    console.error('[Sim] Error initializing simulation:', error)
    showToast(`Simulation error: ${String(error)}`, 'error')
    simCoreRunning = false
  }
}

async function shutdownSimulation() {
  try {
    if (simStepIntervalId !== null) {
      clearInterval(simStepIntervalId)
      simStepIntervalId = null
    }
    await invoke('stop_core')
    simCoreRunning = false
    simStateDisplay.style.display = 'none'
    console.log('[Sim] Core stopped')
  } catch (error) {
    console.error('[Sim] Error stopping simulation:', error)
    showToast(`Error stopping simulation: ${String(error)}`, 'error')
  }
}

async function stepSimulation() {
  if (!simCoreRunning) return
  try {
    await invoke('sim_step', { n_steps: 1 })
    const state = await invoke('sim_get_state')
    updateSimStateDisplay(state)
    updateRobotFromSimState(state)
  } catch (error) {
    console.error('[Sim] Error stepping simulation:', error)
  }
}

function updateSimStateDisplay(state: any) {
  try {
    let html = '<div style="font-weight: bold; color: #4ec9b0; margin-bottom: 8px;">Simulation State</div>'

    if (state && typeof state === 'object') {
      // Display time
      if (state.time !== undefined) {
        html += `<div><span style="color: #e5c07b;">time:</span> ${(state.time as number).toFixed(3)}s</div>`
      }

      // Display joint states
      if (state.joints && typeof state.joints === 'object') {
        html += '<div style="margin-top: 6px; color: #999;">Joints:</div>'
        for (const [name, joint] of Object.entries(state.joints)) {
          if (typeof joint === 'object' && joint !== null) {
            const j = joint as any
            const pos = j.position?.toFixed(3) || '0.000'
            const vel = j.velocity?.toFixed(3) || '0.000'
            html += `<div style="margin-left: 8px;">
              <span style="color: #99ccff;">${name}</span>
              <div style="margin-left: 8px; color: #999; font-size: 10px;">
                pos: ${pos} | vel: ${vel}
              </div>
            </div>`
          }
        }
      }

      // Display contact info
      if (state.contacts !== undefined) {
        html += `<div style="margin-top: 6px; color: #999;">Contacts: <span style="color: #f44336;">${state.contacts}</span></div>`
      }

      // Display energy
      if (state.energy !== undefined) {
        html += `<div style="margin-top: 6px; color: #999;">Energy: <span style="color: #4ec9b0;">${(state.energy as number).toFixed(3)}J</span></div>`
      }
    }

    simStateDisplay.innerHTML = html
  } catch (e) {
    console.error('[Sim] Error updating display:', e)
  }
}

function updateRobotFromSimState(state: any) {
  try {
    if (!state || !state.joints) return

    const joints = state.joints as any

    // Update shoulder pan (Z rotation)
    if (joints.shoulder_pan?.position !== undefined) {
      shoulderPivot.rotation.y = joints.shoulder_pan.position
    }

    // Update shoulder lift (X rotation)
    if (joints.shoulder_lift?.position !== undefined) {
      upperArmPivot.rotation.x = joints.shoulder_lift.position
    }

    // Update elbow (X rotation)
    if (joints.elbow?.position !== undefined) {
      elbowPivot.rotation.x = joints.elbow.position
    }

    // Update gripper fingers
    if (joints.finger_left_joint?.position !== undefined) {
      fingerL.position.x = -0.022 + joints.finger_left_joint.position
      tipL.position.x = fingerL.position.x
    }

    if (joints.finger_right_joint?.position !== undefined) {
      fingerR.position.x = 0.022 + joints.finger_right_joint.position
      tipR.position.x = fingerR.position.x
    }
  } catch (e) {
    console.error('[Sim] Error updating robot from state:', e)
  }
}

// Update sim mode toggle to use persistent core
simToggle.addEventListener('click', async () => {
  simActive = !simActive
  simBar.classList.toggle('hidden', !simActive)
  simToggle.classList.toggle('running', simActive)
  simToggle.querySelector('span')!.textContent = simActive ? 'Exit Sim' : 'Simulate'
  viewportLabel.textContent = simActive ? 'Simulation — MuJoCo' : '3D Preview'
  updateModeIndicator(simActive ? 'Simulation' : 'Demo')

  if (simActive) {
    // Enter simulation mode
    try {
      await initializeSimulation()
      showToast('Entered simulation mode (MuJoCo)', 'success')
    } catch (error) {
      console.error('[Sim] Failed to initialize:', error)
      simActive = false
      simToggle.classList.remove('running')
      simBar.classList.add('hidden')
      showToast('Failed to initialize simulation', 'error')
    }
  } else {
    // Exit simulation mode
    simRunning = false
    simTime = 0
    await shutdownSimulation()

    // Reset pose
    shoulderPivot.rotation.y = 0
    upperArmPivot.rotation.x = 0
    elbowPivot.rotation.x = 0
    fingerL.position.x = -0.022
    fingerR.position.x = 0.022
    updateSimUI()
    showToast('Exited simulation mode', 'info')
  }
  resize()
})

// Update play/pause to use persistent stepping
simPlay.addEventListener('click', () => {
  if (!simCoreRunning) return
  simRunning = true
  if (simStepIntervalId !== null) {
    clearInterval(simStepIntervalId)
  }
  // Step at ~60 Hz
  simStepIntervalId = setInterval(async () => {
    await stepSimulation()
  }, 1000 / 60) as unknown as number
  updateSimUI()
})

simPause.addEventListener('click', () => {
  simRunning = false
  if (simStepIntervalId !== null) {
    clearInterval(simStepIntervalId)
    simStepIntervalId = null
  }
  updateSimUI()
})

simReset.addEventListener('click', async () => {
  if (!simCoreRunning) return
  simRunning = false
  if (simStepIntervalId !== null) {
    clearInterval(simStepIntervalId)
    simStepIntervalId = null
  }
  try {
    await invoke('sim_reset')
    const state = await invoke('sim_get_state')
    updateRobotFromSimState(state)
    simTime = 0
    updateSimUI()
  } catch (error) {
    console.error('[Sim] Reset error:', error)
  }
})

// Optional: Test core integration on page load
async function testCoreIntegration() {
  try {
    console.log('[Test] Core integration available (invoke function loaded)')
  } catch (error) {
    console.warn('[Test] Tauri invoke not available in this context')
  }
}

if (document.readyState === 'loading') {
  document.addEventListener('DOMContentLoaded', () => {
    setTimeout(testCoreIntegration, 500)
  })
} else {
  setTimeout(testCoreIntegration, 500)
}

// ── Part Inspector + Nodes + Toolbox ────────────────────────────────────────

initInspector()

initNodes(meshMap, LINK_DETAILS)

initSelection(canvas, camera, meshMap, { switchToPanel })

function handleAttach(partId: string, nodeId: string, partName: string, mesh: THREE.Mesh) {
  const ok = attachPartToNode(nodeId, partId, partName, mesh)
  if (ok) {
    updateNodeRow(nodeId, true, partName)
    showToast(`Attached ${partName}`, 'success')
  } else {
    showToast('Node already occupied', 'warning')
  }
}

initToolbox(getSelectedNodeId, handleAttach)

// Inspector panel: event delegation for node row clicks and detach buttons
document.getElementById('panel-inspector')?.addEventListener('click', (e) => {
  const target = e.target as HTMLElement

  if (target.classList.contains('insp-node-detach')) {
    const nodeId = target.dataset.nodeId
    if (nodeId) {
      detachPartFromNode(nodeId)
      updateNodeRow(nodeId, false)
      showToast('Part removed', 'info')
    }
    return
  }

  const row = target.closest('.insp-node-row') as HTMLElement | null
  if (row?.dataset.nodeId) {
    programmaticSelectNode(row.dataset.nodeId)
  }
})

// ── Build Mode ────────────────────────────────────────────────────────────────

import { AssemblyGraph }     from './assemblyGraph'
import type { JointConfig }  from './assemblyGraph'
import { AssemblyRenderer }  from './assemblyRenderer'
import { generateURDF, generateMJCF } from './urdfGenerator'
import { getPartDef, defaultParams, interfacesCompatible } from './partLibrary'
import { setToolboxBuildMode, getToolboxSelectedDefId, clearToolboxSelection } from './toolbox'
import {
  initBuildInspector,
  showBuildInspectorFor,
  hideBuildInspector,
} from './buildInspector'

const assembly  = new AssemblyGraph()
const aRenderer = new AssemblyRenderer(scene)
aRenderer.bind(assembly)

let buildMode = false
let pendingDefId: string | null = null
let pendingParentInstanceId: string | null = null
let pendingParentInterfaceId: string | null = null

// ── Build Inspector init ──────────────────────────────────────────────────────

initBuildInspector(assembly, {
  onParamChange(instanceId, params) {
    assembly.updateParams(instanceId, params)
    // Inspector auto-refreshes via graph event → rebuildInstance → showBuildInspectorFor
    // but we re-show so sliders reflect the new mesh's actual state
    setTimeout(() => showBuildInspectorFor(instanceId), 50)
  },
  onJointChange(connectionId, joint) {
    assembly.updateJoint(connectionId, joint)
  },
  onJointValue(connectionId, value) {
    assembly.setJointValue(connectionId, value)
  },
  onDelete(instanceId) {
    deleteBuildPart(instanceId)
  },
  onFocus(instanceId) {
    const bb = aRenderer.getBoundingBox(instanceId)
    if (bb) focusCameraOn(bb)
  },
  onDuplicate(instanceId) {
    duplicatePart(instanceId)
  },
  onLabelChange(instanceId, label) {
    assembly.setLabel(instanceId, label)
    refreshBuildPanel()
  },
})

// ── Camera focus ──────────────────────────────────────────────────────────────

let _cameraAnimId: ReturnType<typeof setInterval> | null = null

function focusCameraOn(bb: THREE.Box3) {
  if (_cameraAnimId !== null) { clearInterval(_cameraAnimId); _cameraAnimId = null }

  const center  = bb.getCenter(new THREE.Vector3())
  const sphere  = bb.getBoundingSphere(new THREE.Sphere())
  const radius  = Math.max(sphere.radius, 0.05)

  const dir     = camera.position.clone().sub(controls.target).normalize()
  if (dir.lengthSq() < 0.001) dir.set(0.6, 0.5, 0.8).normalize()
  const dist    = Math.max(radius * 3.5, 0.25)
  const newPos  = center.clone().add(dir.multiplyScalar(dist))

  const startPos    = camera.position.clone()
  const startTarget = controls.target.clone()
  let t = 0
  const FRAMES = 36

  _cameraAnimId = setInterval(() => {
    t += 1 / FRAMES
    if (t >= 1) { t = 1; clearInterval(_cameraAnimId!); _cameraAnimId = null }
    const ease = 1 - Math.pow(1 - t, 3)   // ease-out cubic
    camera.position.lerpVectors(startPos, newPos, ease)
    controls.target.lerpVectors(startTarget, center, ease)
    controls.update()
  }, 16)
}

// ── Central selection function ────────────────────────────────────────────────

function selectBuildPart(instanceId: string | null) {
  if (!instanceId) {
    aRenderer.selectInstance(null)
    aRenderer.clearXRay()
    hideBuildInspector()
    refreshBuildPanel()
    return
  }
  aRenderer.selectInstance(instanceId)
  aRenderer.applyXRay(instanceId)

  const bb = aRenderer.getBoundingBox(instanceId)
  if (bb) focusCameraOn(bb)

  showBuildInspectorFor(instanceId)
  switchToPanel('inspector')
  refreshBuildPanel()
}

// ── Delete / duplicate ────────────────────────────────────────────────────────

function deleteBuildPart(instanceId: string) {
  if (!assembly.getInstance(instanceId)) return

  // Count subtree size
  let subtreeSize = 0
  const recurse = (id: string) => {
    subtreeSize++
    for (const c of assembly.getChildConnections(id)) recurse(c.childInstanceId)
  }
  recurse(instanceId)

  if (subtreeSize > 1) {
    if (!confirm(`Remove this part and its ${subtreeSize - 1} connected part(s)?`)) return
  }

  const def = getPartDef(assembly.getInstance(instanceId)?.definitionId ?? '')
  assembly.removePart(instanceId)
  aRenderer.clearXRay()
  selectBuildPart(null)
  refreshBuildPanel()
  showToast(`Removed ${def?.name ?? instanceId}`, 'info')
}

function duplicatePart(instanceId: string) {
  const inst = assembly.getInstance(instanceId)
  if (!inst) return
  pendingDefId = inst.definitionId
  aRenderer.setPendingPart(pendingDefId)
  const def = getPartDef(pendingDefId)
  updateBuildHint(`Click an interface ring (○) to place a copy of ${def?.name ?? pendingDefId}`)
  switchToPanel('toolbox')
  showToast(`Select an interface ring to place a copy`, 'info')
}

// ── Connection dialog ─────────────────────────────────────────────────────────

const connDialog     = document.getElementById('connection-dialog')!
const connSubtitle   = document.getElementById('conn-subtitle')!
const connConfirmBtn = document.getElementById('conn-confirm') as HTMLButtonElement
const connCancelBtn  = document.getElementById('conn-cancel')  as HTMLButtonElement
const connLimitsField = document.getElementById('conn-limits-field')!
const connAxisField   = document.getElementById('conn-axis-field')!
const connLower = document.getElementById('conn-lower') as HTMLInputElement
const connUpper = document.getElementById('conn-upper') as HTMLInputElement

function getConnJointType(): 'fixed' | 'revolute' | 'prismatic' {
  return (document.querySelector('input[name="joint-type"]:checked') as HTMLInputElement)?.value as any ?? 'fixed'
}
function getConnAxis(): [number,number,number] {
  const v = (document.querySelector('input[name="joint-axis"]:checked') as HTMLInputElement)?.value ?? 'y'
  return v === 'x' ? [1,0,0] : v === 'z' ? [0,0,1] : [0,1,0]
}

document.querySelectorAll('input[name="joint-type"]').forEach(radio => {
  radio.addEventListener('change', () => {
    const type = getConnJointType()
    const showExtra = type !== 'fixed'
    connLimitsField.classList.toggle('hidden', !showExtra)
    connAxisField.classList.toggle('hidden', !showExtra)
  })
})

function openConnDialog(parentInstanceId: string, parentInterfaceId: string, childDefId: string, childInterfaceId: string) {
  pendingParentInstanceId = parentInstanceId
  pendingParentInterfaceId = parentInterfaceId

  const parentInst = assembly.getInstance(parentInstanceId)
  const parentDef  = parentInst ? getPartDef(parentInst.definitionId) : null
  const childDef   = getPartDef(childDefId)
  const childIface = childDef?.interfaces.find(i => i.id === childInterfaceId)

  connSubtitle.textContent = `${parentDef?.name ?? parentInstanceId}  →  ${childDef?.name ?? childDefId}`
  connLimitsField.classList.add('hidden')
  connAxisField.classList.add('hidden')
  ;(document.querySelector('input[name="joint-type"][value="fixed"]') as HTMLInputElement).checked = true

  if (childIface) {
    const ax = childIface.localAxis(defaultParams(childDef!))
    const axName = ax.y > 0.5 ? 'y' : ax.x > 0.5 ? 'x' : 'z'
    ;(document.querySelector(`input[name="joint-axis"][value="${axName}"]`) as HTMLInputElement).checked = true
  }

  connDialog.dataset.childDefId = childDefId
  connDialog.dataset.childInterfaceId = childInterfaceId
  connDialog.classList.remove('hidden')
}

connCancelBtn.addEventListener('click', () => {
  connDialog.classList.add('hidden')
  aRenderer.clearGhost()
})

connConfirmBtn.addEventListener('click', () => {
  connDialog.classList.add('hidden')
  if (!pendingParentInstanceId || !pendingParentInterfaceId) return

  const childDefId       = connDialog.dataset.childDefId!
  const childInterfaceId = connDialog.dataset.childInterfaceId!
  const childDef         = getPartDef(childDefId)
  if (!childDef) return

  const type = getConnJointType()
  const axis = getConnAxis()
  const joint: Partial<JointConfig> = {
    type, axis,
    lower:    parseFloat(connLower.value),
    upper:    parseFloat(connUpper.value),
    effort:   10, velocity: 2, damping: 0.3, friction: 0.05,
  }

  try {
    const newId = assembly.addPart(
      childDefId, defaultParams(childDef),
      pendingParentInstanceId, pendingParentInterfaceId,
      childInterfaceId, joint,
    )
    selectBuildPart(newId)
    showToast(`Added ${childDef.name}`, 'success')
  } catch (err) {
    showToast(`Connection failed: ${String(err)}`, 'error')
  }

  aRenderer.clearGhost()
  pendingDefId = getToolboxSelectedDefId()
  if (pendingDefId) aRenderer.setPendingPart(pendingDefId)
})

// ── Viewport interaction (click + drag state machine) ────────────────────────

/** Make a Raycaster from a pointer event over the canvas. */
function makeRaycaster(e: { clientX: number; clientY: number }): THREE.Raycaster {
  const rect = canvas.getBoundingClientRect()
  const ndc  = new THREE.Vector2(
    ((e.clientX - rect.left) / rect.width)  * 2 - 1,
    -((e.clientY - rect.top) / rect.height) * 2 + 1,
  )
  const rc = new THREE.Raycaster()
  rc.setFromCamera(ndc, camera)
  return rc
}

/** Intersect the XZ plane at Y=0 with the pointer ray. */
const _dragPlane = new THREE.Plane(new THREE.Vector3(0, 1, 0), 0)
function getPlaneHit(e: { clientX: number; clientY: number }): THREE.Vector3 | null {
  const hit = new THREE.Vector3()
  return makeRaycaster(e).ray.intersectPlane(_dragPlane, hit) ? hit : null
}

/** Central handler for a confirmed click (no drag) in build mode. */
function handleBuildClick(e: { clientX: number; clientY: number }) {
  if (!connDialog.classList.contains('hidden')) return

  const hit = aRenderer.raycast(makeRaycaster(e))

  if (!hit) {
    if (assembly.isEmpty() && pendingDefId) {
      const def      = getPartDef(pendingDefId)!
      const placedId = assembly.addRoot(pendingDefId, defaultParams(def))

      // Snap the new root to the grid if snapping is active
      if (snapGrid.snapEnabled && snapGrid.visible) {
        const ph = getPlaneHit(e)
        if (ph) {
          const s = snapGrid.snapToGrid(ph)
          assembly.setDragOffset(placedId, s.x, 0, s.z)
        }
      }

      selectBuildPart(placedId)
      showToast(`Placed ${def.name} as base`, 'success')
      updateBuildHint(null)
    } else {
      selectBuildPart(null)
    }
    return
  }

  if (hit.type === 'instance') {
    selectBuildPart(hit.instanceId)
  } else if (hit.type === 'interface' && hit.interfaceId) {
    if (!pendingDefId) {
      showToast('Select a part from the Toolbox first', 'warning')
      return
    }
    const childDef = getPartDef(pendingDefId)
    if (!childDef) return

    const parentInst  = assembly.getInstance(hit.instanceId)!
    const parentDef   = getPartDef(parentInst.definitionId)!
    const parentIface = parentDef.interfaces.find(i => i.id === hit.interfaceId)
    if (!parentIface) return

    const compatChildIface = childDef.interfaces.find(ci => interfacesCompatible(ci.type, parentIface.type))
    const childInterfaceId = compatChildIface?.id ?? childDef.interfaces[0]?.id
    if (!childInterfaceId) {
      showToast('No compatible interface on selected part', 'warning')
      return
    }

    aRenderer.showGhostAt(pendingDefId, defaultParams(childDef), hit.instanceId, hit.interfaceId, childInterfaceId)
    openConnDialog(hit.instanceId, hit.interfaceId, pendingDefId, childInterfaceId)
  }
}

// ── Drag state ────────────────────────────────────────────────────────────────

type BuildDragState = 'idle' | 'pressed' | 'dragging'
let _dragState: BuildDragState       = 'idle'
let _dragInstanceId: string | null   = null
let _dragStartMouse                  = { x: 0, y: 0 }
let _dragStartIntersection           = new THREE.Vector3()
/** World positions of the dragged part and all its descendants at drag-start. */
const _dragStartPositions            = new Map<string, THREE.Vector3>()

/** Collect the instanceId subtree rooted at id (inclusive). */
function collectSubtree(id: string): string[] {
  const ids: string[] = [id]
  for (const conn of assembly.getChildConnections(id)) {
    ids.push(...collectSubtree(conn.childInstanceId))
  }
  return ids
}

// pointerdown — intercept before OrbitControls via capture phase
canvas.addEventListener('pointerdown', (e: PointerEvent) => {
  if (!buildMode || e.button !== 0) return
  if (!connDialog.classList.contains('hidden')) return

  const hit = aRenderer.raycast(makeRaycaster(e))

  if (hit?.type === 'instance') {
    _dragState      = 'pressed'
    _dragInstanceId = hit.instanceId
    _dragStartMouse = { x: e.clientX, y: e.clientY }

    const ph = getPlaneHit(e)
    if (ph) _dragStartIntersection.copy(ph)

    // Stop OrbitControls from starting an orbit on this event
    e.stopPropagation()
  }
  // No hit → let OrbitControls handle orbit normally
}, { capture: true })

// pointermove — hover cursor + live drag
canvas.addEventListener('pointermove', (e: PointerEvent) => {
  if (!buildMode) return

  // IDLE: update grab cursor when hovering over draggable parts
  if (_dragState === 'idle') {
    const hit = aRenderer.raycast(makeRaycaster(e))
    canvas.style.cursor = hit?.type === 'instance' ? 'grab' : ''
    return
  }

  // PRESSED → check if we crossed the drag threshold
  if (_dragState === 'pressed') {
    const dx = e.clientX - _dragStartMouse.x
    const dy = e.clientY - _dragStartMouse.y
    if (Math.abs(dx) < 6 && Math.abs(dy) < 6) return

    // ── Transition to DRAGGING ──────────────────────────────────────────────
    _dragState = 'dragging'
    canvas.style.cursor = 'grabbing'

    // Record each subtree part's mesh position at drag-start
    _dragStartPositions.clear()
    if (_dragInstanceId) {
      for (const id of collectSubtree(_dragInstanceId)) {
        const grp = aRenderer.getMeshGroup(id)
        if (grp) _dragStartPositions.set(id, grp.position.clone())
      }
    }

    // Show snap grid while dragging
    snapGrid.setVisible(true)
    return
  }

  // DRAGGING: move all subtree meshes in world space, snap to grid
  if (_dragState === 'dragging') {
    const ph = getPlaneHit(e)
    if (!ph) return

    const snapped = snapGrid.showHighlightAt(ph)

    const dx = snapped.x - _dragStartIntersection.x
    const dz = snapped.z - _dragStartIntersection.z

    _dragStartPositions.forEach((startPos, id) => {
      const grp = aRenderer.getMeshGroup(id)
      if (grp) grp.position.set(startPos.x + dx, startPos.y, startPos.z + dz)
    })
  }
})

// pointerup — commit drag or fire click
canvas.addEventListener('pointerup', (e: PointerEvent) => {
  if (!buildMode || e.button !== 0) return

  // ── Was a click (pressed but not dragged) ──────────────────────────────────
  if (_dragState === 'pressed') {
    _dragState = 'idle'
    canvas.style.cursor = ''
    handleBuildClick(e)
    return
  }

  // ── Commit the drag ────────────────────────────────────────────────────────
  if (_dragState === 'dragging') {
    snapGrid.clearHighlight()

    if (_dragInstanceId) {
      const grp      = aRenderer.getMeshGroup(_dragInstanceId)
      const startPos = _dragStartPositions.get(_dragInstanceId)

      if (grp && startPos) {
        const dx = grp.position.x - startPos.x
        const dz = grp.position.z - startPos.z

        // Accumulate on top of any previously committed offset
        const prev = assembly.getInstance(_dragInstanceId)?.dragOffset ?? { x: 0, y: 0, z: 0 }
        assembly.setDragOffset(_dragInstanceId, prev.x + dx, prev.y, prev.z + dz)
      }
    }

    _dragState      = 'idle'
    _dragInstanceId = null
    canvas.style.cursor = ''

    // Hide snap grid again if snap toggle is off
    if (!_snapVisible) snapGrid.setVisible(false)
  }
}, { capture: true })

// ── Snap toggle (cycles through sizes, then off) ──────────────────────────────

const snapToggleBtn = document.getElementById('toggle-snap') as HTMLButtonElement | null
const snapLabel     = document.getElementById('snap-label') as HTMLElement | null
let _snapVisible    = false
let _snapSizeIdx    = 1 // default 0.1 m

function _updateSnapBtn() {
  if (!snapToggleBtn) return
  if (!_snapVisible) {
    snapToggleBtn.classList.remove('active')
    if (snapLabel) snapLabel.textContent = 'Snap'
  } else {
    snapToggleBtn.classList.add('active')
    const cm = Math.round(snapGrid.snapSize * 100)
    if (snapLabel) snapLabel.textContent = `${cm}cm`
  }
}

snapToggleBtn?.addEventListener('click', () => {
  if (!_snapVisible) {
    // Turn on at current size
    _snapVisible = true
    snapGrid.snapEnabled = true
    snapGrid.setSnapSize(SNAP_SIZES[_snapSizeIdx])
    snapGrid.setVisible(true)
    snapGrid.updateAround(controls.target)
  } else {
    // Cycle to next size; after last size → turn off
    _snapSizeIdx = (_snapSizeIdx + 1) % SNAP_SIZES.length
    if (_snapSizeIdx === 0) {
      // Wrapped around → turn off
      _snapVisible = false
      snapGrid.snapEnabled = false
      snapGrid.setVisible(false)
    } else {
      snapGrid.setSnapSize(SNAP_SIZES[_snapSizeIdx])
      snapGrid.updateAround(controls.target)
    }
  }
  _updateSnapBtn()
})

// ── Build panel refresh ───────────────────────────────────────────────────────

function refreshBuildPanel() {
  const emptyDiv = document.getElementById('build-empty')
  const treeDiv  = document.getElementById('build-tree')
  if (!emptyDiv || !treeDiv) return

  const parts  = assembly.size()
  const massG  = Math.round(assembly.totalMass() * 1000)
  const joints = Math.max(0, parts - 1)

  ;(document.getElementById('bs-parts')  as HTMLElement).textContent = String(parts)
  ;(document.getElementById('bs-mass')   as HTMLElement).textContent = `${massG} g`
  ;(document.getElementById('bs-joints') as HTMLElement).textContent = String(joints)

  emptyDiv.style.display = parts === 0 ? '' : 'none'
  treeDiv.innerHTML = ''

  const selectedId = aRenderer.getSelectedInstanceId()

  assembly.walk((inst, parentConn, depth) => {
    const def      = getPartDef(inst.definitionId)
    const isActive = inst.instanceId === selectedId
    const isVis    = aRenderer.isInstanceVisible(inst.instanceId)

    const row = document.createElement('div')
    row.className = 'bt-node' + (isActive ? ' active' : '')
    row.dataset.instanceId = inst.instanceId

    // Indent guide lines via left padding
    const padLeft = depth * 16 + 6
    const icon = def?.category === 'actuator' ? '⚙' : def?.category === 'sensor' ? '◉'
               : def?.category === 'electrical' ? '⚡' : def?.category === 'end_effector' ? '✊' : '⬡'
    const joint = parentConn?.joint.type ?? ''
    const jointBadge = joint ? `<span class="bt-joint">${joint}</span>` : ''
    const rootBadge  = !parentConn ? `<span class="bt-joint bt-root">root</span>` : ''

    row.innerHTML = `
      <span class="bt-vis${isVis ? '' : ' part-hidden'}" data-instance-id="${inst.instanceId}" title="Toggle visibility (G)">
        ${isVis ? '👁' : '○'}
      </span>
      <span class="bt-icon" style="padding-left:${padLeft}px">${icon}</span>
      <span class="bt-label" data-instance-id="${inst.instanceId}" title="Double-click to rename">${inst.label}</span>
      ${rootBadge}${jointBadge}
      <span class="bt-del" data-instance-id="${inst.instanceId}" title="Delete part">✕</span>
    `

    // Visibility toggle
    row.querySelector('.bt-vis')?.addEventListener('click', (e) => {
      e.stopPropagation()
      const id = (e.currentTarget as HTMLElement).dataset.instanceId!
      const nowVis = !aRenderer.isInstanceVisible(id)
      aRenderer.setInstanceVisible(id, nowVis)
      refreshBuildPanel()
    })

    // Delete button
    row.querySelector('.bt-del')?.addEventListener('click', (e) => {
      e.stopPropagation()
      const id = (e.currentTarget as HTMLElement).dataset.instanceId!
      deleteBuildPart(id)
    })

    // Label double-click to rename
    const labelEl = row.querySelector('.bt-label') as HTMLElement | null
    labelEl?.addEventListener('dblclick', (e) => {
      e.stopPropagation()
      const id   = labelEl.dataset.instanceId!
      const orig = assembly.getInstance(id)?.label ?? ''
      const inp  = document.createElement('input')
      inp.className = 'bt-label-input'
      inp.value     = orig
      inp.maxLength = 48
      labelEl.replaceWith(inp)
      inp.focus(); inp.select()

      const commit = () => {
        const newLabel = inp.value.trim() || orig
        assembly.setLabel(id, newLabel)
        refreshBuildPanel()
      }
      inp.addEventListener('blur',    commit)
      inp.addEventListener('keydown', ev => {
        if (ev.key === 'Enter')  { inp.blur(); ev.preventDefault() }
        if (ev.key === 'Escape') { inp.value = orig; inp.blur() }
        ev.stopPropagation()
      })
    })

    // Row click → select part
    row.addEventListener('click', () => selectBuildPart(inst.instanceId))

    treeDiv.appendChild(row)
  })
}

// ── Build Mode toggle ─────────────────────────────────────────────────────────

const buildToggleBtn   = document.getElementById('build-toggle') as HTMLButtonElement
const buildHintOverlay = document.getElementById('build-hint-overlay') as HTMLDivElement
const buildHintText    = document.getElementById('build-hint-text') as HTMLSpanElement

function updateBuildHint(msg: string | null) {
  if (!buildMode || !msg) { buildHintOverlay.classList.add('hidden'); return }
  buildHintText.textContent = msg
  buildHintOverlay.classList.remove('hidden')
}

function enterBuildMode() {
  buildMode = true
  robot.visible = false
  buildToggleBtn.classList.add('active')
  updateModeIndicator('Build Mode')
  switchToPanel('build')
  switchToPanel('toolbox')
  setToolboxBuildMode(true, (defId) => {
    pendingDefId = defId
    aRenderer.setPendingPart(defId)
    if (assembly.isEmpty()) {
      updateBuildHint(`Click in viewport to place ${getPartDef(defId)?.name ?? defId} as base`)
    } else {
      updateBuildHint(`Click an interface ring (○) to connect ${getPartDef(defId)?.name ?? defId}`)
    }
  })
  // Show snap grid if it was active
  if (_snapVisible) {
    snapGrid.setVisible(true)
    snapGrid.updateAround(controls.target)
  }
  refreshBuildPanel()
  showToast('Build Mode — open Toolbox to add parts', 'info')
}

function exitBuildMode() {
  buildMode = false
  robot.visible = true
  buildToggleBtn.classList.remove('active')
  updateModeIndicator('Demo')
  aRenderer.selectInstance(null)
  aRenderer.clearXRay()
  aRenderer.hideAllRings()
  aRenderer.clearGhost()
  hideBuildInspector()
  setToolboxBuildMode(false)
  clearToolboxSelection()
  pendingDefId = null
  buildHintOverlay.classList.add('hidden')
  // Hide snap grid (but remember _snapVisible state for when re-entering)
  snapGrid.setVisible(false)
  snapGrid.clearHighlight()
  _dragState = 'idle'
  _dragInstanceId = null
  canvas.style.cursor = ''
  showToast('Exited Build Mode', 'info')
}

buildToggleBtn.addEventListener('click', () => {
  if (buildMode) exitBuildMode(); else enterBuildMode()
})

document.querySelectorAll('.ab-btn[data-panel="build"]').forEach(btn => {
  btn.addEventListener('click', () => { if (!buildMode) enterBuildMode() })
})

// ── Keyboard shortcuts ────────────────────────────────────────────────────────

document.addEventListener('keydown', (e) => {
  const inInput = e.target instanceof HTMLInputElement || e.target instanceof HTMLTextAreaElement

  // B — toggle build mode (always available)
  if (!inInput && (e.key === 'b' || e.key === 'B')) {
    if (buildMode) exitBuildMode(); else enterBuildMode()
    return
  }

  if (!buildMode) return

  // Delete / Backspace — remove selected part
  if (!inInput && (e.key === 'Delete' || e.key === 'Backspace')) {
    const sel = aRenderer.getSelectedInstanceId()
    if (sel) { e.preventDefault(); deleteBuildPart(sel) }
    return
  }

  // F — focus camera on selected part
  if (!inInput && (e.key === 'f' || e.key === 'F')) {
    const sel = aRenderer.getSelectedInstanceId()
    if (sel) {
      const bb = aRenderer.getBoundingBox(sel)
      if (bb) focusCameraOn(bb)
    }
    return
  }

  // G — toggle visibility of selected part
  if (!inInput && (e.key === 'g' || e.key === 'G')) {
    const sel = aRenderer.getSelectedInstanceId()
    if (sel) {
      aRenderer.setInstanceVisible(sel, !aRenderer.isInstanceVisible(sel))
      refreshBuildPanel()
    }
    return
  }

  // Escape — deselect
  if (e.key === 'Escape') {
    if (!connDialog.classList.contains('hidden')) {
      connDialog.classList.add('hidden')
      aRenderer.clearGhost()
      return
    }
    selectBuildPart(null)
  }
})

// ── Export buttons ────────────────────────────────────────────────────────────

document.getElementById('btn-export-urdf')?.addEventListener('click', () => {
  if (assembly.isEmpty()) { showToast('Assembly is empty', 'warning'); return }
  const urdf = generateURDF(assembly, { robotName: 'my_robot', addGravityLink: true })
  const blob = new Blob([urdf], { type: 'text/xml' })
  const url  = URL.createObjectURL(blob)
  const a    = document.createElement('a')
  a.href = url; a.download = 'assembly.urdf'; a.click()
  URL.revokeObjectURL(url)
  showToast('URDF exported', 'success')
})

document.getElementById('btn-export-mjcf')?.addEventListener('click', () => {
  if (assembly.isEmpty()) { showToast('Assembly is empty', 'warning'); return }
  const mjcf = generateMJCF(assembly, 'my_robot')
  const blob  = new Blob([mjcf], { type: 'text/xml' })
  const url   = URL.createObjectURL(blob)
  const a     = document.createElement('a')
  a.href = url; a.download = 'assembly.xml'; a.click()
  URL.revokeObjectURL(url)
  showToast('MJCF exported', 'success')
})

document.getElementById('btn-clear-assembly')?.addEventListener('click', () => {
  if (assembly.isEmpty()) return
  if (confirm('Clear the entire assembly? This cannot be undone.')) {
    assembly.clear()
    aRenderer.clearXRay()
    hideBuildInspector()
    selectBuildPart(null)
    showToast('Assembly cleared', 'info')
  }
})
