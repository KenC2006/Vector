import { getPartDef, defaultParams } from './partLibrary'
import type { ParamValues } from './partLibrary'

// ── Types ─────────────────────────────────────────────────────────────────────

export interface JointConfig {
  type: 'fixed' | 'revolute' | 'prismatic'
  axis: [number, number, number]
  lower?: number
  upper?: number
  effort?: number
  velocity?: number
  damping?: number
  friction?: number
  // current value (for sim / visualization)
  value?: number
}

export interface PartInstance {
  instanceId: string
  definitionId: string
  params: ParamValues
  label: string           // user-editable name
  dragOffset?:   { x: number; y: number; z: number }        // world-space translation on top of connection transform
  dragRotation?: { x: number; y: number; z: number; w: number } // local-space quaternion delta on top of connection rotation
}

export interface ConnectionEdge {
  connectionId: string
  parentInstanceId: string
  parentInterfaceId: string
  childInstanceId: string
  childInterfaceId: string
  joint: JointConfig
}

export type AssemblyEvent =
  | { type: 'instance_added';   instanceId: string }
  | { type: 'instance_removed'; instanceId: string; subtree: string[] }
  | { type: 'params_changed';   instanceId: string }
  | { type: 'joint_changed';    connectionId: string }
  | { type: 'cleared' }
  | { type: 'restored' }        // emitted after deserialize so renderer can rebuildAll

type AssemblyListener = (event: AssemblyEvent) => void

// ── AssemblyGraph ─────────────────────────────────────────────────────────────

export class AssemblyGraph {
  private instances   = new Map<string, PartInstance>()
  private connections = new Map<string, ConnectionEdge>()
  /** connectionId -> connectionId for child lookup (parentInstanceId -> [connectionId]) */
  private childMap    = new Map<string, string[]>()
  /** instanceId -> connectionId that made it a child */
  private parentConnMap = new Map<string, string>()
  private rootInstanceId: string | null = null
  private listeners: AssemblyListener[] = []
  private _nextId = 0

  // ── Pub/sub ────────────────────────────────────────────────────────────────

  on(fn: AssemblyListener): () => void {
    this.listeners.push(fn)
    return () => { this.listeners = this.listeners.filter(l => l !== fn) }
  }

  private emit(event: AssemblyEvent) {
    for (const l of this.listeners) l(event)
  }

  // ── Helpers ───────────────────────────────────────────────────────────────

  private newId(prefix: string) { return `${prefix}_${++this._nextId}` }

  // ── Queries ───────────────────────────────────────────────────────────────

  isEmpty() { return this.instances.size === 0 }
  size()    { return this.instances.size }

  getRootId()                  { return this.rootInstanceId }
  getInstance(id: string)      { return this.instances.get(id) }
  getConnection(id: string)    { return this.connections.get(id) }

  getAllInstances(): PartInstance[] {
    return [...this.instances.values()]
  }

  getChildConnections(instanceId: string): ConnectionEdge[] {
    const ids = this.childMap.get(instanceId) ?? []
    return ids.map(id => this.connections.get(id)!).filter(Boolean)
  }

  getParentConnection(instanceId: string): ConnectionEdge | undefined {
    const connId = this.parentConnMap.get(instanceId)
    return connId ? this.connections.get(connId) : undefined
  }

  /** Which interface IDs are already occupied (as parent) on a given instance */
  occupiedInterfaces(instanceId: string): Set<string> {
    const s = new Set<string>()
    for (const c of this.getChildConnections(instanceId)) s.add(c.parentInterfaceId)
    const pc = this.getParentConnection(instanceId)
    if (pc) s.add(pc.childInterfaceId)
    return s
  }

  // ── DFS walk ──────────────────────────────────────────────────────────────

  walk(cb: (inst: PartInstance, parentConn: ConnectionEdge | undefined, depth: number) => void) {
    if (!this.rootInstanceId) return
    const recurse = (id: string, pc: ConnectionEdge | undefined, depth: number) => {
      const inst = this.instances.get(id)
      if (!inst) return
      cb(inst, pc, depth)
      for (const conn of this.getChildConnections(id)) {
        recurse(conn.childInstanceId, conn, depth + 1)
      }
    }
    recurse(this.rootInstanceId, undefined, 0)
  }

  /** Return all instanceIds in DFS order */
  dfsOrder(): string[] {
    const order: string[] = []
    this.walk(inst => order.push(inst.instanceId))
    return order
  }

  /** Collect the subtree rooted at instanceId (inclusive) */
  private collectSubtree(instanceId: string): string[] {
    const ids: string[] = []
    const recurse = (id: string) => {
      ids.push(id)
      for (const conn of this.getChildConnections(id)) recurse(conn.childInstanceId)
    }
    recurse(instanceId)
    return ids
  }

  // ── Mutations ─────────────────────────────────────────────────────────────

  /**
   * Add the first part (root) to an empty assembly.
   * Returns the new instanceId.
   */
  addRoot(definitionId: string, params?: ParamValues, label?: string): string {
    if (!this.isEmpty()) throw new Error('Assembly is not empty — use addPart() to connect parts')
    const def = getPartDef(definitionId)
    if (!def) throw new Error(`Unknown part definition: "${definitionId}"`)

    const instanceId = this.newId('inst')
    this.instances.set(instanceId, {
      instanceId,
      definitionId,
      params: params ?? defaultParams(def),
      label: label ?? def.name,
    })
    this.childMap.set(instanceId, [])
    this.rootInstanceId = instanceId
    this.emit({ type: 'instance_added', instanceId })
    return instanceId
  }

  /**
   * Add a child part connected to an existing part.
   * Returns the new instanceId.
   */
  addPart(
    definitionId: string,
    params: ParamValues | undefined,
    parentInstanceId: string,
    parentInterfaceId: string,
    childInterfaceId: string,
    joint?: Partial<JointConfig>,
    label?: string,
  ): string {
    const def = getPartDef(definitionId)
    if (!def) throw new Error(`Unknown part definition: "${definitionId}"`)
    if (!this.instances.has(parentInstanceId)) throw new Error(`Parent instance "${parentInstanceId}" not found`)

    const parentDef = getPartDef(this.instances.get(parentInstanceId)!.definitionId)!
    const pIface = parentDef.interfaces.find(i => i.id === parentInterfaceId)
    const cIface = def.interfaces.find(i => i.id === childInterfaceId)
    if (!pIface) throw new Error(`Interface "${parentInterfaceId}" not found on "${parentDef.id}"`)
    if (!cIface) throw new Error(`Interface "${childInterfaceId}" not found on "${definitionId}"`)

    const instanceId   = this.newId('inst')
    const connectionId = this.newId('conn')

    const resolvedParams = params ?? defaultParams(def)

    // Build joint config — prefer joint arg, fallback to interface defaults
    const jointConfig: JointConfig = {
      type: joint?.type ?? cIface.defaultJointType,
      axis: joint?.axis ?? cIface.localAxis(resolvedParams).toArray() as [number,number,number],
      lower:    joint?.lower    ?? (cIface.defaultJointType === 'revolute' ? -3.14159 : 0),
      upper:    joint?.upper    ?? (cIface.defaultJointType === 'revolute' ?  3.14159 : 0),
      effort:   joint?.effort   ?? 10,
      velocity: joint?.velocity ?? 2,
      damping:  joint?.damping  ?? 0.3,
      friction: joint?.friction ?? 0.05,
      value:    0,
    }

    this.instances.set(instanceId, {
      instanceId,
      definitionId,
      params: resolvedParams,
      label: label ?? def.name,
    })
    this.childMap.set(instanceId, [])

    const conn: ConnectionEdge = {
      connectionId,
      parentInstanceId,
      parentInterfaceId,
      childInstanceId: instanceId,
      childInterfaceId,
      joint: jointConfig,
    }
    this.connections.set(connectionId, conn)

    const siblings = this.childMap.get(parentInstanceId) ?? []
    siblings.push(connectionId)
    this.childMap.set(parentInstanceId, siblings)
    this.parentConnMap.set(instanceId, connectionId)

    this.emit({ type: 'instance_added', instanceId })
    return instanceId
  }

  /**
   * Remove a part and its entire subtree.
   */
  removePart(instanceId: string) {
    if (instanceId === this.rootInstanceId && this.instances.size > 1) {
      // If removing root, clear everything
      this.clear(); return
    }

    const subtree = this.collectSubtree(instanceId)

    // Remove parent connection edge
    const parentConnId = this.parentConnMap.get(instanceId)
    if (parentConnId) {
      const conn = this.connections.get(parentConnId)!
      const siblings = this.childMap.get(conn.parentInstanceId) ?? []
      this.childMap.set(conn.parentInstanceId, siblings.filter(id => id !== parentConnId))
      this.connections.delete(parentConnId)
    }

    // Remove all instances + connections in subtree
    for (const id of subtree) {
      this.instances.delete(id)
      this.childMap.delete(id)
      const pconn = this.parentConnMap.get(id)
      if (pconn) { this.connections.delete(pconn); this.parentConnMap.delete(id) }
    }

    if (instanceId === this.rootInstanceId) this.rootInstanceId = null

    this.emit({ type: 'instance_removed', instanceId, subtree })
  }

  /**
   * Update the parameters for a part instance.
   * Emits params_changed so the renderer can rebuild the mesh.
   */
  updateParams(instanceId: string, newParams: Partial<ParamValues>) {
    const inst = this.instances.get(instanceId)
    if (!inst) return
    inst.params = { ...inst.params, ...newParams } as ParamValues
    this.emit({ type: 'params_changed', instanceId })
  }

  /**
   * Set a world-space XYZ translation offset on a part, on top of its connection-derived position.
   * Emits params_changed so the renderer rebuilds with the new offset applied.
   */
  setDragOffset(instanceId: string, x: number, y: number, z: number) {
    const inst = this.instances.get(instanceId)
    if (!inst) return
    inst.dragOffset = { x, y, z }
    this.emit({ type: 'params_changed', instanceId })
  }

  /** Remove the drag offset from a part. */
  clearDragOffset(instanceId: string) {
    const inst = this.instances.get(instanceId)
    if (!inst) return
    delete inst.dragOffset
    this.emit({ type: 'params_changed', instanceId })
  }

  /**
   * Set a local-space quaternion rotation delta on a part, applied on top of its
   * connection-derived orientation.
   */
  setDragRotation(instanceId: string, q: { x: number; y: number; z: number; w: number }) {
    const inst = this.instances.get(instanceId)
    if (!inst) return
    inst.dragRotation = { ...q }
    this.emit({ type: 'params_changed', instanceId })
  }

  /** Remove the drag rotation delta. */
  clearDragRotation(instanceId: string) {
    const inst = this.instances.get(instanceId)
    if (!inst) return
    delete inst.dragRotation
    this.emit({ type: 'params_changed', instanceId })
  }

  // ── Serialization (for undo/redo snapshots) ───────────────────────────────

  serialize(): string {
    return JSON.stringify({
      instances:     [...this.instances.entries()],
      connections:   [...this.connections.entries()],
      childMap:      [...this.childMap.entries()],
      parentConnMap: [...this.parentConnMap.entries()],
      rootInstanceId: this.rootInstanceId,
      _nextId:        this._nextId,
    })
  }

  deserialize(json: string) {
    const d = JSON.parse(json)
    this.instances     = new Map(d.instances)
    this.connections   = new Map(d.connections)
    this.childMap      = new Map(d.childMap)
    this.parentConnMap = new Map(d.parentConnMap)
    this.rootInstanceId = d.rootInstanceId
    this._nextId        = d._nextId
    this.listeners = this.listeners // keep listeners intact
    this.emit({ type: 'restored' })
  }

  /**
   * Update joint configuration for a connection.
   */
  updateJoint(connectionId: string, joint: Partial<JointConfig>) {
    const conn = this.connections.get(connectionId)
    if (!conn) return
    conn.joint = { ...conn.joint, ...joint }
    this.emit({ type: 'joint_changed', connectionId })
  }

  /**
   * Update the live joint value (for simulation / slider).
   */
  setJointValue(connectionId: string, value: number) {
    const conn = this.connections.get(connectionId)
    if (!conn) return
    conn.joint.value = value
    this.emit({ type: 'joint_changed', connectionId })
  }

  /** Rename a part instance */
  setLabel(instanceId: string, label: string) {
    const inst = this.instances.get(instanceId)
    if (inst) inst.label = label
  }

  clear() {
    this.instances.clear()
    this.connections.clear()
    this.childMap.clear()
    this.parentConnMap.clear()
    this.rootInstanceId = null
    this.emit({ type: 'cleared' })
  }

  // ── Kinematic tree for sidebar panel ──────────────────────────────────────

  toTreeJSON(): TreeNode | null {
    if (!this.rootInstanceId) return null
    const build = (id: string): TreeNode => {
      const inst = this.instances.get(id)!
      const conn = this.getParentConnection(id)
      return {
        instanceId:  id,
        definitionId: inst.definitionId,
        label:        inst.label,
        joint:        conn?.joint,
        children:     this.getChildConnections(id).map(c => build(c.childInstanceId)),
      }
    }
    return build(this.rootInstanceId)
  }

  /** Total mass of the assembly (kg) */
  totalMass(): number {
    let m = 0
    for (const inst of this.instances.values()) {
      const def = getPartDef(inst.definitionId)
      if (def) m += def.mass(inst.params)
    }
    return m
  }
}

export interface TreeNode {
  instanceId:  string
  definitionId: string
  label:        string
  joint?:       JointConfig
  children:     TreeNode[]
}
