// Phase 3b — link-name / component-id pattern matchers extracted from
// urdfAssembly.ts. Pure string predicates; no preset/scene/DOM access.

export function componentIdFromLinkName(linkName: string): string {
  const match = linkName.match(/^(.+)_\d+$/)
  return match ? match[1] : linkName
}

export function isSplitServoComponentId(componentId: string): boolean {
  return (
    componentId.startsWith('actuator_servo') ||
    componentId.startsWith('actuator_continuous_rotation_servo') ||
    componentId.startsWith('actuator_high_speed')
  )
}

export function isDistalBeamComponentId(componentId: string | null | undefined): boolean {
  return componentId === 'structural_limb_link_slim'
    || componentId?.startsWith('structural_extrusion_') === true
}
