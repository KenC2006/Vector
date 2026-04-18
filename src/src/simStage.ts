// simStage.ts — dedicated 3D environment shown while simulation is active.
//
// Enter/exit is strict: enter() snapshots build-mode state (visibility,
// background, camera, robot transform) and swaps in a stage environment.
// exit() restores every captured field so build mode looks exactly as it did.

import * as THREE from 'three'
import type { OrbitControls } from 'three/addons/controls/OrbitControls.js'

export interface SimStageDeps {
  scene: THREE.Scene
  camera: THREE.PerspectiveCamera
  controls: OrbitControls
  robot: THREE.Group
  /** Build-mode environment objects to hide while in sim (grid, ground, axes). */
  buildVisuals: THREE.Object3D[]
}

export interface SimStageApi {
  enter(): void
  exit(): void
  isActive(): boolean
}

export function initSimStage(deps: SimStageDeps): SimStageApi {
  const stageGroup = new THREE.Group()
  stageGroup.name = 'sim_stage'
  stageGroup.visible = false
  deps.scene.add(stageGroup)

  // Gradient skybox (inverted sphere, drawn behind everything).
  const bgMat = new THREE.ShaderMaterial({
    side: THREE.BackSide,
    depthWrite: false,
    uniforms: {
      topColor: { value: new THREE.Color(0x0a0e14) },
      bottomColor: { value: new THREE.Color(0x202838) },
    },
    vertexShader: `
      varying vec3 vDir;
      void main() {
        vDir = position;
        gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0);
      }
    `,
    fragmentShader: `
      uniform vec3 topColor;
      uniform vec3 bottomColor;
      varying vec3 vDir;
      void main() {
        float h = clamp(normalize(vDir).y * 0.5 + 0.5, 0.0, 1.0);
        gl_FragColor = vec4(mix(bottomColor, topColor, pow(h, 0.7)), 1.0);
      }
    `,
  })
  const bgSphere = new THREE.Mesh(new THREE.SphereGeometry(80, 32, 32), bgMat)
  bgSphere.renderOrder = -999
  bgSphere.frustumCulled = false
  stageGroup.add(bgSphere)

  // Stage disk — circular platform under the robot.
  const stageDisk = new THREE.Mesh(
    new THREE.CircleGeometry(4, 96),
    new THREE.MeshStandardMaterial({
      color: 0x161a22, roughness: 0.85, metalness: 0.05,
    })
  )
  stageDisk.rotation.x = -Math.PI / 2
  stageDisk.receiveShadow = true
  stageGroup.add(stageDisk)

  // Accent rim.
  const rim = new THREE.Mesh(
    new THREE.RingGeometry(3.96, 4.02, 160),
    new THREE.MeshBasicMaterial({ color: 0x3a7bd5, transparent: true, opacity: 0.55 })
  )
  rim.rotation.x = -Math.PI / 2
  rim.position.y = 0.001
  stageGroup.add(rim)

  // Subtle range rings at 0.5, 1, 2 m for scale reference.
  for (const r of [0.5, 1.0, 2.0]) {
    const ring = new THREE.Mesh(
      new THREE.RingGeometry(r - 0.004, r + 0.004, 96),
      new THREE.MeshBasicMaterial({ color: 0x3a4455, transparent: true, opacity: 0.45 })
    )
    ring.rotation.x = -Math.PI / 2
    ring.position.y = 0.002
    stageGroup.add(ring)
  }

  // Origin tick.
  const originTick = new THREE.Mesh(
    new THREE.CircleGeometry(0.025, 24),
    new THREE.MeshBasicMaterial({ color: 0x569cd6 })
  )
  originTick.rotation.x = -Math.PI / 2
  originTick.position.y = 0.003
  stageGroup.add(originTick)

  // Make the stage disk double-sided so it stays visible from below
  // rather than back-face culling to invisibility.
  ;(stageDisk.material as THREE.MeshStandardMaterial).side = THREE.DoubleSide

  // Snapshot of build-mode state, captured in enter(), restored in exit().
  type Snapshot = {
    visibility: Array<{ obj: THREE.Object3D; visible: boolean }>
    sceneBackground: THREE.Scene['background']
    cameraPos: THREE.Vector3
    cameraQuat: THREE.Quaternion
    controlsTarget: THREE.Vector3
    robotPos: THREE.Vector3
    robotQuat: THREE.Quaternion
  }
  let snap: Snapshot | null = null

  function enter() {
    if (snap) return
    snap = {
      visibility: deps.buildVisuals.map(obj => ({ obj, visible: obj.visible })),
      sceneBackground: deps.scene.background,
      cameraPos: deps.camera.position.clone(),
      cameraQuat: deps.camera.quaternion.clone(),
      controlsTarget: deps.controls.target.clone(),
      robotPos: deps.robot.position.clone(),
      robotQuat: deps.robot.quaternion.clone(),
    }
    for (const obj of deps.buildVisuals) obj.visible = false
    deps.scene.background = null   // let the gradient sphere show through
    stageGroup.visible = true
    deps.controls.update()
  }

  function exit() {
    if (!snap) return
    stageGroup.visible = false
    for (const { obj, visible } of snap.visibility) obj.visible = visible
    deps.scene.background = snap.sceneBackground
    deps.camera.position.copy(snap.cameraPos)
    deps.camera.quaternion.copy(snap.cameraQuat)
    deps.controls.target.copy(snap.controlsTarget)
    deps.controls.update()
    deps.robot.position.copy(snap.robotPos)
    deps.robot.quaternion.copy(snap.robotQuat)
    snap = null
  }

  return { enter, exit, isActive: () => snap !== null }
}
