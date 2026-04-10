/**
 * Rich Visual Override System
 *
 * After parseURDFToScene() builds the scene from URDF primitives, this module
 * scans all links. For any link whose name matches a preset component ID, it
 * replaces the primitive meshes with rich Three.js geometry.
 *
 * URDF stays clean with simple collision primitives. Only the viewport gets
 * rich visuals. Falls back gracefully for non-preset links.
 */
import * as THREE from 'three'
import { findRichGenerator } from './generators'
import type { GeneratorDims } from './generators'

interface ParsedRobotLike {
  group: THREE.Group
  linkGroups: Map<string, THREE.Group>
}

/**
 * Extract the component ID from a link name.
 * Link names use the pattern: "{component_id}_{instance_number}"
 * e.g., "actuator_servo_high_torque_4" → "actuator_servo_high_torque"
 *
 * Returns null if the link name doesn't match a preset pattern.
 */
function extractComponentId(linkName: string): string | null {
  // Strip the trailing _N instance number
  const match = linkName.match(/^(.+)_(\d+)$/)
  if (!match) return null
  return match[1]
}

/**
 * Measure the bounding box of existing geometry in a link group.
 * Used to determine dimensions for the rich generator.
 */
function measureLinkDims(linkGroup: THREE.Group): GeneratorDims {
  const box = new THREE.Box3()
  linkGroup.traverse(child => {
    if (child instanceof THREE.Mesh) {
      child.updateWorldMatrix(true, false)
      const childBox = new THREE.Box3().setFromObject(child)
      box.union(childBox)
    }
  })

  if (box.isEmpty()) return { x: 0.04, y: 0.04, z: 0.04 }

  const size = new THREE.Vector3()
  box.getSize(size)
  return { x: Math.max(size.x, 0.005), y: Math.max(size.y, 0.005), z: Math.max(size.z, 0.005) }
}

/**
 * Apply rich visuals to all preset-derived links in the parsed robot.
 * Call this after parseURDFToScene() and after adding the group to the scene.
 */
export function applyRichVisuals(parsedRobot: ParsedRobotLike): void {
  for (const [linkName, linkGroup] of parsedRobot.linkGroups) {
    const compId = extractComponentId(linkName)
    if (!compId) continue

    const generator = findRichGenerator(compId)
    if (!generator) continue

    // Measure existing primitive geometry to get dimensions
    const dims = measureLinkDims(linkGroup)

    // Generate rich visual group
    let richGroup: THREE.Group
    try {
      richGroup = generator(compId, dims)
    } catch (e) {
      console.warn(`[richVisuals] Generator failed for ${compId}:`, e)
      continue  // keep primitive visuals
    }

    // Tag all meshes for raycasting (preserve existing contract)
    richGroup.traverse(child => {
      if (child instanceof THREE.Mesh) {
        child.castShadow = true
        child.receiveShadow = true
        ;(child.userData as Record<string, unknown>).urdfLinkName = linkName
      }
    })

    // Find the geometryGroup (first child of linkGroup that contains any mesh descendants)
    const geometryChild = linkGroup.children.find(c => {
      if (!(c instanceof THREE.Group)) return false
      let hasMesh = false
      c.traverse(gc => { if (gc instanceof THREE.Mesh) hasMesh = true })
      return hasMesh
    }) as THREE.Group | undefined

    if (geometryChild) {
      // Dispose old primitive geometry to prevent GPU memory leaks
      geometryChild.traverse(child => {
        if (child instanceof THREE.Mesh) {
          child.geometry?.dispose()
          // Don't dispose shared defaultMat — only dispose per-visual materials
          if (child.material && 'dispose' in child.material) {
            const mat = child.material as THREE.Material
            if (mat.uuid) {
              // Only dispose if it's not the shared default material
              // (materials from the preset system always have a color set)
              const stdMat = mat as THREE.MeshStandardMaterial
              if (stdMat.color && stdMat.color.getHex() !== 0x888888) {
                mat.dispose()
              }
            }
          }
        }
      })

      // Replace: clear primitive children, add rich group
      // Preserve the geometryGroup's transform
      richGroup.position.copy(geometryChild.position)
      richGroup.quaternion.copy(geometryChild.quaternion)
      richGroup.scale.copy(geometryChild.scale)

      // Remove all children from geometryChild, then add rich geometry
      while (geometryChild.children.length > 0) {
        geometryChild.remove(geometryChild.children[0])
      }

      // Add rich meshes directly into the geometryGroup
      while (richGroup.children.length > 0) {
        const child = richGroup.children[0]
        richGroup.remove(child)
        geometryChild.add(child)
      }
    }
  }
}
