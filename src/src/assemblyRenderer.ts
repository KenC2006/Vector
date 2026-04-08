import * as THREE from 'three'
import type { AssemblyGraph, PartInstance, ConnectionEdge } from './assemblyGraph'
import { getPartDef, interfacesCompatible } from './partLibrary'
import type { ParamValues } from './partLibrary'

// ── Interface node colours ───────────────────────────────────────────────────

const COL_AVAILABLE = 0x44aaff
const COL_COMPAT    = 0x44ffaa
const COL_OCCUPIED  = 0x555566
const COL_SNAP      = 0x66ffcc   // highlight when a snap target is active
const COL_COLLISION = 0xff4444   // red tint on colliding parts

// ── Shared cube geometry (reused for every node) ─────────────────────────────

const _nodeGeo = new THREE.BoxGeometry(0.018, 0.018, 0.018)

function makeNodeMesh(color: number): THREE.Mesh {
  const mat = new THREE.MeshStandardMaterial({
    color, emissive: color, emissiveIntensity: 0.7,
    roughness: 0.25, metalness: 0.0,
    transparent: true, opacity: 0.92, depthWrite: false,
  })
  const m = new THREE.Mesh(_nodeGeo, mat)
  m.userData.isRing = true   // keep flag name for raycasting compat
  return m
}

// ── Connection transform ──────────────────────────────────────────────────────

export function computeConnectionTransform(
  parentDef: ReturnType<typeof getPartDef>,
  parentParams: ParamValues,
  parentIfaceId: string,
  childDef: ReturnType<typeof getPartDef>,
  childParams: ParamValues,
  childIfaceId: string,
): THREE.Matrix4 {
  if (!parentDef || !childDef) return new THREE.Matrix4()

  const pIface = parentDef.interfaces.find(i => i.id === parentIfaceId)
  const cIface = childDef.interfaces.find(i => i.id === childIfaceId)
  if (!pIface || !cIface) return new THREE.Matrix4()

  const pPos    = pIface.localPosition(parentParams)
  const pNormal = pIface.localNormal(parentParams).normalize()

  const cPos    = cIface.localPosition(childParams)
  const cNormal = cIface.localNormal(childParams).normalize()

  // Align child interface normal to opposite of parent interface normal
  const targetNormal = pNormal.clone().negate()
  const sourceNormal = cNormal.clone()

  let rot: THREE.Quaternion
  const dot = sourceNormal.dot(targetNormal)
  if (dot < -0.9999) {
    const perp = new THREE.Vector3(1, 0, 0)
    if (Math.abs(sourceNormal.dot(perp)) > 0.9) perp.set(0, 1, 0)
    const axis = perp.cross(sourceNormal).normalize()
    rot = new THREE.Quaternion().setFromAxisAngle(axis, Math.PI)
  } else {
    rot = new THREE.Quaternion().setFromUnitVectors(sourceNormal, targetNormal)
  }

  // Child position in parent space: parent interface pos – rotated child interface pos
  const rotatedChildPos = cPos.clone().applyQuaternion(rot)
  const childOrigin     = pPos.clone().sub(rotatedChildPos)

  return new THREE.Matrix4().compose(childOrigin, rot, new THREE.Vector3(1, 1, 1))
}

// ── HitResult ─────────────────────────────────────────────────────────────────

export type HitResult =
  | { type: 'instance';  instanceId: string; interfaceId?: undefined }
  | { type: 'interface'; instanceId: string; interfaceId: string }

// ── AssemblyRenderer ──────────────────────────────────────────────────────────

export class AssemblyRenderer {
  readonly group = new THREE.Group()

  private graph: AssemblyGraph | null = null
  private unsubscribe: (() => void) | null = null

  private meshMap = new Map<string, THREE.Group>()   // instanceId → part group
  private ringMap = new Map<string, THREE.Mesh>()    // "instanceId:ifaceId" → ring mesh
  private connectionLines: THREE.Line[] = []

  private selectedInstanceId: string | null = null
  private pendingPartDefId:   string | null = null

  private ghostGroup: THREE.Group | null = null

  /** Ghost shown during gizmo drag to preview where the part would snap */
  private snapGhostGroup: THREE.Group | null = null

  private readonly _connLineMat = new THREE.LineBasicMaterial({ color: 0x4488aa, transparent: true, opacity: 0.5 })

  constructor(scene: THREE.Scene) {
    scene.add(this.group)
  }

  // ── Bind / Unbind ─────────────────────────────────────────────────────────

  bind(graph: AssemblyGraph) {
    this.unbind()
    this.graph = graph
    this.unsubscribe = graph.on(ev => {
      switch (ev.type) {
        case 'instance_added':   this.rebuildAll();               break
        case 'instance_removed': this.rebuildAll();               break
        case 'params_changed':   this.rebuildInstance(ev.instanceId); break
        case 'joint_changed':    this.applyJointValue(ev.connectionId); break
        case 'cleared':          this.clear();                    break
        case 'restored':         this.rebuildAll();               break
      }
    })
  }

  unbind() {
    this.unsubscribe?.()
    this.unsubscribe = null
    this.graph = null
  }

  // ── Full rebuild ──────────────────────────────────────────────────────────

  rebuildAll() {
    if (!this.graph) return

    while (this.group.children.length) this.group.remove(this.group.children[0])
    this.meshMap.clear()
    this.ringMap.clear()
    this.connectionLines = []

    this.graph.walk((inst, parentConn) => {
      this.addInstanceMesh(inst, parentConn)
    })

    this._drawConnectionLines()

    // Restore rings
    if (this.pendingPartDefId) {
      this._showAllCompatibleRings(this.pendingPartDefId)
    } else if (this.selectedInstanceId && this.meshMap.has(this.selectedInstanceId)) {
      this._showInterfacesInner(this.selectedInstanceId)
    }

    // Restore selection highlight
    if (this.selectedInstanceId) {
      this._applyHighlight(this.selectedInstanceId)
    }
  }

  // ── Per-instance mesh ─────────────────────────────────────────────────────

  private addInstanceMesh(inst: PartInstance, parentConn: ConnectionEdge | undefined) {
    const def = getPartDef(inst.definitionId)
    if (!def) return

    const partGroup = def.buildMesh(inst.params)
    partGroup.name = `part_${inst.instanceId}`
    partGroup.userData.instanceId = inst.instanceId

    // Clone every mesh material so instances never share material objects.
    // Shared materials cause selection / visibility changes to bleed across parts.
    partGroup.traverse(obj => {
      if (obj instanceof THREE.Mesh) {
        obj.userData.instanceId = inst.instanceId
        if (obj.material) obj.material = (obj.material as THREE.Material).clone()
      }
    })

    const worldMat = this._computeWorldMatrix(inst, parentConn)
    partGroup.position.setFromMatrixPosition(worldMat)
    partGroup.quaternion.setFromRotationMatrix(worldMat)

    // Apply user-controlled world-space rotation (from gizmo rotate mode)
    if (inst.dragRotation) {
      const dq = new THREE.Quaternion(
        inst.dragRotation.x, inst.dragRotation.y,
        inst.dragRotation.z, inst.dragRotation.w,
      )
      // premultiply = world-space rotation on top of connection orientation
      partGroup.quaternion.premultiply(dq)
    }

    // Apply user-controlled world-space offset (from gizmo translate mode)
    if (inst.dragOffset) {
      partGroup.position.x += inst.dragOffset.x
      partGroup.position.y += inst.dragOffset.y
      partGroup.position.z += inst.dragOffset.z
    }

    this.group.add(partGroup)
    this.meshMap.set(inst.instanceId, partGroup)
  }

  private _computeWorldMatrix(inst: PartInstance, parentConn: ConnectionEdge | undefined): THREE.Matrix4 {
    if (!parentConn || !this.graph) return new THREE.Matrix4()

    const parentGroup = this.meshMap.get(parentConn.parentInstanceId)
    if (!parentGroup) return new THREE.Matrix4()

    const parentWorldMat = new THREE.Matrix4().compose(parentGroup.position, parentGroup.quaternion, parentGroup.scale)

    const parentInst   = this.graph.getInstance(parentConn.parentInstanceId)!
    const parentDef    = getPartDef(parentInst.definitionId)!
    const childDef     = getPartDef(inst.definitionId)!

    const localMat = computeConnectionTransform(
      parentDef, parentInst.params, parentConn.parentInterfaceId,
      childDef,  inst.params,       parentConn.childInterfaceId,
    )
    return parentWorldMat.clone().multiply(localMat)
  }

  // ── Param-only rebuild ────────────────────────────────────────────────────

  private rebuildInstance(instanceId: string) {
    const old = this.meshMap.get(instanceId)
    if (old) this.group.remove(old)
    this.meshMap.delete(instanceId)

    for (const key of [...this.ringMap.keys()]) {
      if (key.startsWith(instanceId + ':')) {
        this.ringMap.get(key)!.parent?.remove(this.ringMap.get(key)!)
        this.ringMap.delete(key)
      }
    }

    if (!this.graph) return
    const inst = this.graph.getInstance(instanceId)
    if (!inst) return
    const parentConn = this.graph.getParentConnection(instanceId)
    this.addInstanceMesh(inst, parentConn)

    if (instanceId === this.selectedInstanceId) {
      this._applyHighlight(instanceId)
      this._showInterfacesInner(instanceId)
    }

    for (const conn of this.graph.getChildConnections(instanceId)) {
      this.rebuildInstance(conn.childInstanceId)
    }
  }

  // ── Connection lines ──────────────────────────────────────────────────────

  private _drawConnectionLines() {
    if (!this.graph) return
    this.graph.walk((inst, parentConn) => {
      if (!parentConn) return
      const pGrp = this.meshMap.get(parentConn.parentInstanceId)
      const cGrp = this.meshMap.get(inst.instanceId)
      if (!pGrp || !cGrp) return
      const geo  = new THREE.BufferGeometry().setFromPoints([pGrp.position.clone(), cGrp.position.clone()])
      const line = new THREE.Line(geo, this._connLineMat)
      line.renderOrder = -1
      this.group.add(line)
      this.connectionLines.push(line)
    })
  }

  // ── Joint animation ───────────────────────────────────────────────────────

  private applyJointValue(connectionId: string) {
    if (!this.graph) return
    const conn = this.graph.getConnection(connectionId)
    if (!conn) return
    const childGroup = this.meshMap.get(conn.childInstanceId)
    if (!childGroup) return
    const joint = conn.joint
    if (joint.type === 'fixed' || joint.value === undefined) return
    const inst       = this.graph.getInstance(conn.childInstanceId)!
    const parentConn = this.graph.getParentConnection(conn.childInstanceId)
    const baseMat    = this._computeWorldMatrix(inst, parentConn)
    if (joint.type === 'revolute') {
      const axis   = new THREE.Vector3(...joint.axis).normalize()
      baseMat.multiply(new THREE.Matrix4().makeRotationAxis(axis, joint.value))
    } else if (joint.type === 'prismatic') {
      const axis   = new THREE.Vector3(...joint.axis).normalize()
      baseMat.multiply(new THREE.Matrix4().makeTranslation(axis.x * joint.value, axis.y * joint.value, axis.z * joint.value))
    }
    childGroup.position.setFromMatrixPosition(baseMat)
    childGroup.quaternion.setFromRotationMatrix(baseMat)
    for (const c of this.graph.getChildConnections(conn.childInstanceId)) {
      this.rebuildInstance(c.childInstanceId)
    }
  }

  // ── Interface rings ───────────────────────────────────────────────────────

  showInterfacesFor(instanceId: string) {
    this.hideAllRings()
    this._showInterfacesInner(instanceId)
  }

  hideAllRings() {
    for (const ring of this.ringMap.values()) ring.parent?.remove(ring)
    this.ringMap.clear()
  }

  setPendingPart(defId: string | null) {
    this.pendingPartDefId = defId
    if (defId) {
      this._showAllCompatibleRings(defId)
    } else if (this.selectedInstanceId) {
      this.showInterfacesFor(this.selectedInstanceId)
    } else {
      this.hideAllRings()
    }
  }

  private _showAllCompatibleRings(_defId: string) {
    this.hideAllRings()
    if (!this.graph) return
    this.graph.walk(inst => this._showInterfacesInner(inst.instanceId))
  }

  private _showInterfacesInner(instanceId: string) {
    if (!this.graph) return
    const inst = this.graph.getInstance(instanceId)
    if (!inst) return
    const def = getPartDef(inst.definitionId)
    if (!def) return
    const partGroup = this.meshMap.get(instanceId)
    if (!partGroup) return
    const occupied = this.graph.occupiedInterfaces(instanceId)

    for (const iface of def.interfaces) {
      const key   = `${instanceId}:${iface.id}`
      if (this.ringMap.has(key)) continue
      const isOcc = occupied.has(iface.id)
      let color   = isOcc ? COL_OCCUPIED : COL_AVAILABLE
      if (!isOcc && this.pendingPartDefId) {
        const pendingDef = getPartDef(this.pendingPartDefId)
        if (pendingDef) {
          const hasCompat = pendingDef.interfaces.some(pi => interfacesCompatible(pi.type, iface.type))
          color = hasCompat ? COL_COMPAT : COL_AVAILABLE
        }
      }
      const node = makeNodeMesh(color)
      node.position.copy(iface.localPosition(inst.params))
      node.userData.instanceId  = instanceId
      node.userData.interfaceId = iface.id
      node.userData.isRing      = true
      partGroup.add(node)
      this.ringMap.set(key, node)
    }
  }

  // ── Selection highlight ───────────────────────────────────────────────────

  selectInstance(instanceId: string | null) {
    // Remove highlight from previously selected
    if (this.selectedInstanceId) {
      this._clearHighlight(this.selectedInstanceId)
    }

    this.hideAllRings()
    this.selectedInstanceId = instanceId

    if (!instanceId) {
      if (this.pendingPartDefId) this._showAllCompatibleRings(this.pendingPartDefId)
      return
    }

    this._applyHighlight(instanceId)

    if (this.pendingPartDefId) {
      this._showAllCompatibleRings(this.pendingPartDefId)
    } else {
      this._showInterfacesInner(instanceId)
    }
  }

  private _applyHighlight(instanceId: string) {
    const grp = this.meshMap.get(instanceId)
    if (!grp) return
    grp.traverse(o => {
      if (!(o instanceof THREE.Mesh) || o.userData.isRing) return
      const mat = o.material as THREE.MeshStandardMaterial
      if (!('_origEmissiveHex' in mat.userData)) {
        mat.userData._origEmissiveHex       = mat.emissive.getHex()
        mat.userData._origEmissiveIntensity = mat.emissiveIntensity
      }
      mat.emissive.setHex(0x224488)
      mat.emissiveIntensity = 0.9
    })
  }

  private _clearHighlight(instanceId: string) {
    const grp = this.meshMap.get(instanceId)
    if (!grp) return
    grp.traverse(o => {
      if (!(o instanceof THREE.Mesh) || o.userData.isRing) return
      const mat = o.material as THREE.MeshStandardMaterial
      mat.emissive.setHex(mat.userData._origEmissiveHex       ?? 0x000000)
      mat.emissiveIntensity =          mat.userData._origEmissiveIntensity ?? 0
    })
  }

  getSelectedInstanceId() { return this.selectedInstanceId }

  // ── Raycasting ────────────────────────────────────────────────────────────

  raycast(raycaster: THREE.Raycaster): HitResult | null {
    const targets: THREE.Object3D[] = []
    this.group.traverse(o => { if (o instanceof THREE.Mesh) targets.push(o) })
    const hits = raycaster.intersectObjects(targets, false)

    // Rings take priority
    for (const h of hits) {
      if (h.object.userData.isRing) {
        return { type: 'interface', instanceId: h.object.userData.instanceId, interfaceId: h.object.userData.interfaceId }
      }
    }
    for (const h of hits) {
      if (h.object.userData.instanceId) {
        return { type: 'instance', instanceId: h.object.userData.instanceId }
      }
    }
    return null
  }

  // ── Ghost preview ─────────────────────────────────────────────────────────

  showGhostAt(defId: string, params: ParamValues, parentInstanceId: string, parentIfaceId: string, childIfaceId: string) {
    this.clearGhost()
    if (!this.graph) return
    const def   = getPartDef(defId)
    const pInst = this.graph.getInstance(parentInstanceId)
    const pDef  = pInst ? getPartDef(pInst.definitionId) : null
    if (!def || !pInst || !pDef) return

    const ghost = def.buildMesh(params)
    ghost.traverse(o => {
      if (o instanceof THREE.Mesh) {
        const mat = (o.material as THREE.MeshStandardMaterial).clone()
        mat.transparent = true
        mat.opacity     = 0.40
        mat.depthWrite  = false
        o.material = mat
      }
    })

    const pGrp     = this.meshMap.get(parentInstanceId)
    if (!pGrp) return
    const pWorldMat = new THREE.Matrix4().compose(pGrp.position, pGrp.quaternion, pGrp.scale)
    const localMat  = computeConnectionTransform(pDef, pInst.params, parentIfaceId, def, params, childIfaceId)
    const worldMat  = pWorldMat.clone().multiply(localMat)

    ghost.position.setFromMatrixPosition(worldMat)
    ghost.quaternion.setFromRotationMatrix(worldMat)
    this.group.add(ghost)
    this.ghostGroup = ghost
  }

  clearGhost() {
    if (this.ghostGroup) { this.group.remove(this.ghostGroup); this.ghostGroup = null }
  }

  // ── Drag-mode: show all nodes + live snap ghost ────────────────────────────

  /** Call when gizmo drag starts — shows interface nodes on every part. */
  beginDrag(_instanceId: string) {
    this._showAllNodes()
  }

  /**
   * Call continuously while gizmo is dragging.
   * Checks for snap targets and shows a translucent ghost preview at the
   * would-be snap position. Returns the snap target (or null).
   */
  updateDragPreview(draggedInstanceId: string, threshold = 0.08) {
    this.clearSnapGhost()
    this._resetAllNodeColors()

    const snap = this.findSnapTarget(draggedInstanceId, threshold)
    if (!snap || !this.graph) return snap

    // Highlight the target node with snap colour
    const targetKey = `${snap.targetInstanceId}:${snap.targetIfaceId}`
    const targetNode = this.ringMap.get(targetKey)
    if (targetNode) {
      const mat = targetNode.material as THREE.MeshStandardMaterial
      mat.color.setHex(COL_SNAP)
      mat.emissive.setHex(COL_SNAP)
    }

    // Build a translucent ghost showing where the part would end up after snap
    const draggedInst = this.graph.getInstance(draggedInstanceId)
    if (!draggedInst) return snap
    const draggedDef = getPartDef(draggedInst.definitionId)
    if (!draggedDef) return snap

    const targetInst = this.graph.getInstance(snap.targetInstanceId)
    const targetDef  = targetInst ? getPartDef(targetInst.definitionId) : null
    const targetGrp  = this.meshMap.get(snap.targetInstanceId)
    if (!targetInst || !targetDef || !targetGrp) return snap

    const ghost = draggedDef.buildMesh(draggedInst.params)
    ghost.traverse(o => {
      if (o instanceof THREE.Mesh) {
        const mat = (o.material as THREE.MeshStandardMaterial).clone()
        mat.transparent = true
        mat.opacity     = 0.30
        mat.depthWrite  = false
        mat.color.setHex(0x44aaff)
        mat.emissive.setHex(0x224466)
        mat.emissiveIntensity = 0.6
        o.material = mat
      }
    })

    const pWorldMat = new THREE.Matrix4().compose(targetGrp.position, targetGrp.quaternion, targetGrp.scale)
    const localMat  = computeConnectionTransform(
      targetDef, targetInst.params, snap.targetIfaceId,
      draggedDef, draggedInst.params, snap.draggedIfaceId,
    )
    const worldMat = pWorldMat.clone().multiply(localMat)
    ghost.position.setFromMatrixPosition(worldMat)
    ghost.quaternion.setFromRotationMatrix(worldMat)
    this.group.add(ghost)
    this.snapGhostGroup = ghost

    return snap
  }

  /** Call when gizmo drag ends — hide nodes and snap ghost. */
  endDrag() {
    this.clearSnapGhost()
    this.clearCollisionHighlights()
    // Restore normal node visibility (selected instance only, or pending, etc.)
    this.hideAllRings()
    if (this.pendingPartDefId) {
      this._showAllCompatibleRings(this.pendingPartDefId)
    } else if (this.selectedInstanceId && this.meshMap.has(this.selectedInstanceId)) {
      this._showInterfacesInner(this.selectedInstanceId)
    }
  }

  clearSnapGhost() {
    if (this.snapGhostGroup) {
      this.group.remove(this.snapGhostGroup)
      this.snapGhostGroup = null
    }
  }

  /** Show nodes on ALL parts (used during drag). */
  private _showAllNodes() {
    this.hideAllRings()
    if (!this.graph) return
    this.graph.walk(inst => this._showInterfacesInner(inst.instanceId))
  }

  /** Reset all visible nodes back to their default colour. */
  private _resetAllNodeColors() {
    if (!this.graph) return
    for (const [key, node] of this.ringMap) {
      const [instanceId, ifaceId] = key.split(':')
      const inst = this.graph.getInstance(instanceId)
      if (!inst) continue
      const def = getPartDef(inst.definitionId)
      if (!def) continue
      const occupied = this.graph.occupiedInterfaces(instanceId)
      const isOcc    = occupied.has(ifaceId)
      const color    = isOcc ? COL_OCCUPIED : COL_AVAILABLE
      const mat = node.material as THREE.MeshStandardMaterial
      mat.color.setHex(color)
      mat.emissive.setHex(color)
    }
  }

  // ── Snap detection ─────────────────────────────────────────────────────────

  /**
   * Find the best interface-to-interface snap target for a dragged part.
   * Returns null if nothing is within `threshold` distance.
   */
  findSnapTarget(draggedInstanceId: string, threshold = 0.08): {
    draggedIfaceId:  string
    targetInstanceId: string
    targetIfaceId:   string
    distance:        number
  } | null {
    if (!this.graph) return null

    const draggedInst = this.graph.getInstance(draggedInstanceId)
    if (!draggedInst) return null
    const draggedDef = getPartDef(draggedInst.definitionId)
    if (!draggedDef) return null
    const draggedGroup = this.meshMap.get(draggedInstanceId)
    if (!draggedGroup) return null

    // Force fresh world matrix — gizmo modifies position/quaternion directly
    // but matrixWorld may be stale between render frames
    draggedGroup.updateMatrixWorld(true)

    // Interfaces occupied by children of the dragged part (these can't be used)
    const childConns = this.graph.getChildConnections(draggedInstanceId)
    const occupiedByChildren = new Set(childConns.map(c => c.parentInterfaceId))

    // Build world positions for every eligible interface on the dragged part.
    // The child-interface currently connecting it to its parent IS eligible
    // because a reparent will free it.
    const draggedIfaces = draggedDef.interfaces
      .filter(iface => !occupiedByChildren.has(iface.id))
      .map(iface => ({
        iface,
        worldPos: draggedGroup.localToWorld(iface.localPosition(draggedInst.params).clone()),
      }))

    // If the dragged part has a parent, that parent's interface will be freed
    // by reparent — so don't count it as occupied on the target side.
    const currentParentConn = this.graph.getParentConnection(draggedInstanceId)

    let best: { draggedIfaceId: string; targetInstanceId: string; targetIfaceId: string; distance: number } | null = null

    this.graph.walk(inst => {
      if (inst.instanceId === draggedInstanceId) return
      // Skip descendants of dragged part (would create a cycle)
      if (this.graph!.isDescendantOf(inst.instanceId, draggedInstanceId)) return

      const def = getPartDef(inst.definitionId)
      if (!def) return
      const grp = this.meshMap.get(inst.instanceId)
      if (!grp) return

      grp.updateMatrixWorld(true)
      const occupied = this.graph!.occupiedInterfaces(inst.instanceId)

      // The interface on the current parent that connects the dragged part
      // will be freed by reparent, so treat it as available.
      const freedIfaceId = (currentParentConn && currentParentConn.parentInstanceId === inst.instanceId)
        ? currentParentConn.parentInterfaceId
        : null

      for (const tIface of def.interfaces) {
        if (occupied.has(tIface.id) && tIface.id !== freedIfaceId) continue
        const tWorld = grp.localToWorld(tIface.localPosition(inst.params).clone())

        for (const { iface: dIface, worldPos: dWorld } of draggedIfaces) {
          if (!interfacesCompatible(dIface.type, tIface.type)) continue
          const dist = dWorld.distanceTo(tWorld)
          if (dist < threshold && (!best || dist < best.distance)) {
            best = {
              draggedIfaceId:  dIface.id,
              targetInstanceId: inst.instanceId,
              targetIfaceId:   tIface.id,
              distance:        dist,
            }
          }
        }
      }
    })

    return best
  }

  // ── Collision detection (AABB) ──────────────────────────────────────────────

  private _collisionHighlighted = new Set<string>()

  /** Return IDs of parts whose AABB overlaps the dragged part. */
  checkCollisions(draggedInstanceId: string): string[] {
    const draggedBox = this.getBoundingBox(draggedInstanceId)
    if (!draggedBox || !this.graph) return []

    const collisions: string[] = []
    this.graph.walk(inst => {
      if (inst.instanceId === draggedInstanceId) return
      if (this.graph!.isDescendantOf(inst.instanceId, draggedInstanceId)) return
      const box = this.getBoundingBox(inst.instanceId)
      if (box && draggedBox.intersectsBox(box)) {
        collisions.push(inst.instanceId)
      }
    })
    return collisions
  }

  /** Tint the given parts red to signal collision. */
  highlightCollisions(ids: string[]) {
    // Clear previous collision highlights
    this.clearCollisionHighlights()

    for (const id of ids) {
      const grp = this.meshMap.get(id)
      if (!grp) continue
      this._collisionHighlighted.add(id)
      grp.traverse(o => {
        if (!(o instanceof THREE.Mesh) || o.userData.isRing) return
        const mat = o.material as THREE.MeshStandardMaterial
        if (!('_preCollisionEmissiveHex' in mat.userData)) {
          mat.userData._preCollisionEmissiveHex       = mat.emissive.getHex()
          mat.userData._preCollisionEmissiveIntensity = mat.emissiveIntensity
        }
        mat.emissive.setHex(COL_COLLISION)
        mat.emissiveIntensity = 0.8
      })
    }
  }

  /** Remove red collision tint from all previously highlighted parts. */
  clearCollisionHighlights() {
    for (const id of this._collisionHighlighted) {
      const grp = this.meshMap.get(id)
      if (!grp) continue
      grp.traverse(o => {
        if (!(o instanceof THREE.Mesh) || o.userData.isRing) return
        const mat = o.material as THREE.MeshStandardMaterial
        if ('_preCollisionEmissiveHex' in mat.userData) {
          mat.emissive.setHex(mat.userData._preCollisionEmissiveHex)
          mat.emissiveIntensity = mat.userData._preCollisionEmissiveIntensity
          delete mat.userData._preCollisionEmissiveHex
          delete mat.userData._preCollisionEmissiveIntensity
        }
      })
    }
    this._collisionHighlighted.clear()
  }

  // ── Queries ───────────────────────────────────────────────────────────────

  getMeshGroup(instanceId: string): THREE.Group | undefined {
    return this.meshMap.get(instanceId)
  }

  getBoundingBox(instanceId: string): THREE.Box3 | null {
    const grp = this.meshMap.get(instanceId)
    if (!grp) return null
    const box = new THREE.Box3()
    grp.traverse(o => {
      if (o instanceof THREE.Mesh && !o.userData.isRing) box.union(new THREE.Box3().setFromObject(o))
    })
    return box.isEmpty() ? null : box
  }

  // ── Cleanup ───────────────────────────────────────────────────────────────

  clear() {
    while (this.group.children.length) this.group.remove(this.group.children[0])
    this.meshMap.clear()
    this.ringMap.clear()
    this.connectionLines = []
    this.ghostGroup = null
    this.snapGhostGroup = null
    this.selectedInstanceId = null
  }

  dispose() {
    this.unbind()
    this.clear()
    this.group.parent?.remove(this.group)
  }
}
