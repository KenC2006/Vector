import type { AssemblyGraph } from './assemblyGraph'

/**
 * Snapshot-based undo/redo for AssemblyGraph.
 *
 * Call `record()` BEFORE every mutation to save a restore point.
 * Then `undo()` / `redo()` to navigate the stack.
 */
export class AssemblyHistory {
  private _undoStack: string[] = []
  private _redoStack: string[] = []
  private readonly _graph: AssemblyGraph
  readonly maxSize: number

  constructor(graph: AssemblyGraph, maxSize = 60) {
    this._graph = graph
    this.maxSize = maxSize
  }

  /** Save current state before a mutation. Clears the redo stack. */
  record() {
    this._undoStack.push(this._graph.serialize())
    if (this._undoStack.length > this.maxSize) this._undoStack.shift()
    this._redoStack = []
  }

  /** Restore the previous state. Returns true if anything was undone. */
  undo(): boolean {
    const snap = this._undoStack.pop()
    if (!snap) return false
    this._redoStack.push(this._graph.serialize())
    this._graph.deserialize(snap)
    return true
  }

  /** Reapply a previously undone state. Returns true if anything was redone. */
  redo(): boolean {
    const snap = this._redoStack.pop()
    if (!snap) return false
    this._undoStack.push(this._graph.serialize())
    this._graph.deserialize(snap)
    return true
  }

  canUndo() { return this._undoStack.length > 0 }
  canRedo() { return this._redoStack.length > 0 }

  /** Wipe both stacks (e.g. on full assembly clear). */
  clear() {
    this._undoStack = []
    this._redoStack = []
  }
}
