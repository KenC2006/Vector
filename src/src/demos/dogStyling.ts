// dogStyling.ts — minimal polish overlay for the K-9 demo.
//
// Intentionally lightweight: the real visual content comes from the URDF
// (which uses preset component names so the rich-visuals system auto-loads
// detailed motor/servo/battery/sensor GLBs). This module only adds a soft
// under-robot contact shadow to ground the assembly to the stage — the
// components themselves are the visible robot, fully exposed.
//
// Returns a cleanup that removes the shadow plane and disposes its texture/
// material when the demo is torn down.

import * as THREE from 'three'
import type { ParsedRobot } from '../urdfParser'

export function applyDogStyling(_parsedRobot: ParsedRobot, robotRoot: THREE.Group): () => void {
  // Procedural radial-gradient alpha texture — fakes ambient occlusion under
  // the chassis without needing a real AO pass.
  const canvas = document.createElement('canvas')
  canvas.width = 256
  canvas.height = 256
  const ctx = canvas.getContext('2d')!
  const grad = ctx.createRadialGradient(128, 128, 12, 128, 128, 124)
  grad.addColorStop(0.0, 'rgba(0,0,0,0.45)')
  grad.addColorStop(0.6, 'rgba(0,0,0,0.18)')
  grad.addColorStop(1.0, 'rgba(0,0,0,0.00)')
  ctx.fillStyle = grad
  ctx.fillRect(0, 0, 256, 256)
  const tex = new THREE.CanvasTexture(canvas)
  tex.colorSpace = THREE.SRGBColorSpace
  const mat = new THREE.MeshBasicMaterial({ map: tex, transparent: true, depthWrite: false })
  const mesh = new THREE.Mesh(new THREE.PlaneGeometry(0.85, 0.55), mat)
  // Scene is Y-up; rotate the plane onto the ground (XZ). Attached to robot
  // root so the shadow tracks the robot when it bobs/drifts.
  mesh.rotation.x = -Math.PI / 2
  mesh.position.y = 0.002
  mesh.renderOrder = -1
  robotRoot.add(mesh)

  return () => {
    robotRoot.remove(mesh)
    mesh.geometry.dispose()
    mat.dispose()
    tex.dispose()
  }
}
