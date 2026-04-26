import * as THREE from 'three'

const meshCache = new Map<string, THREE.Group>()
const loadingInProgress = new Set<string>()

export function getCachedMeshGroup(compId: string): THREE.Group | null {
  return meshCache.get(compId) ?? null
}

export function hasCachedMeshGroup(compId: string): boolean {
  return meshCache.has(compId)
}

export function setCachedMeshGroup(compId: string, group: THREE.Group): void {
  meshCache.set(compId, group)
}

export function isMeshLoadInProgress(compId: string): boolean {
  return loadingInProgress.has(compId)
}

export function markMeshLoadInProgress(compId: string): void {
  loadingInProgress.add(compId)
}

export function clearMeshLoadInProgress(compId: string): void {
  loadingInProgress.delete(compId)
}
