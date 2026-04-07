export interface LinkDetail {
  name: string
  mass: number
  geometry: {
    type: 'box' | 'cylinder' | 'sphere'
    params: {
      width?: number
      height?: number
      depth?: number
      radius?: number
      length?: number
    }
  }
  inertia: {
    ixx: number; ixy: number; ixz: number
    iyy: number; iyz: number; izz: number
  }
}

export interface JointDetail {
  name: string
  type: string
  axis: string
  parentLink: string
  childLink: string
  limits?: { lower: number; upper: number; effort: number; velocity: number }
  dynamics?: { damping: number; friction: number }
}

export const LINK_DETAILS: Record<string, LinkDetail> = {
  'base_link': {
    name: 'base_link', mass: 1.2,
    geometry: { type: 'cylinder', params: { radius: 0.15, length: 0.05 } },
    inertia: { ixx: 0.003, ixy: 0, ixz: 0, iyy: 0.003, iyz: 0, izz: 0.005 },
  },
  'shoulder_link': {
    name: 'shoulder_link', mass: 0.18,
    geometry: { type: 'cylinder', params: { radius: 0.035, length: 0.055 } },
    inertia: { ixx: 0.0001, ixy: 0, ixz: 0, iyy: 0.0001, iyz: 0, izz: 0.0001 },
  },
  'upper_arm': {
    name: 'upper_arm', mass: 0.4,
    geometry: { type: 'box', params: { width: 0.065, height: 0.35, depth: 0.065 } },
    inertia: { ixx: 0.004, ixy: 0, ixz: 0, iyy: 0.004, iyz: 0, izz: 0.0005 },
  },
  'elbow_link': {
    name: 'elbow_link', mass: 0.12,
    geometry: { type: 'cylinder', params: { radius: 0.03, length: 0.05 } },
    inertia: { ixx: 0.00005, ixy: 0, ixz: 0, iyy: 0.00005, iyz: 0, izz: 0.00005 },
  },
  'forearm': {
    name: 'forearm', mass: 0.25,
    geometry: { type: 'box', params: { width: 0.055, height: 0.28, depth: 0.055 } },
    inertia: { ixx: 0.002, ixy: 0, ixz: 0, iyy: 0.002, iyz: 0, izz: 0.0003 },
  },
  'wrist': {
    name: 'wrist', mass: 0.08,
    geometry: { type: 'sphere', params: { radius: 0.022 } },
    inertia: { ixx: 0.00002, ixy: 0, ixz: 0, iyy: 0.00002, iyz: 0, izz: 0.00002 },
  },
  'gripper_base': {
    name: 'gripper_base', mass: 0.06,
    geometry: { type: 'box', params: { width: 0.06, height: 0.025, depth: 0.035 } },
    inertia: { ixx: 0.00001, ixy: 0, ixz: 0, iyy: 0.00001, iyz: 0, izz: 0.00001 },
  },
  'finger_left': {
    name: 'finger_left', mass: 0.02,
    geometry: { type: 'box', params: { width: 0.008, height: 0.055, depth: 0.025 } },
    inertia: { ixx: 0.000005, ixy: 0, ixz: 0, iyy: 0.000005, iyz: 0, izz: 0.000005 },
  },
  'finger_right': {
    name: 'finger_right', mass: 0.02,
    geometry: { type: 'box', params: { width: 0.008, height: 0.055, depth: 0.025 } },
    inertia: { ixx: 0.000005, ixy: 0, ixz: 0, iyy: 0.000005, iyz: 0, izz: 0.000005 },
  },
}

export const JOINT_DETAILS: Record<string, JointDetail> = {
  'shoulder_pan': {
    name: 'shoulder_pan', type: 'revolute', axis: 'Z',
    parentLink: 'base_link', childLink: 'shoulder_link',
    limits: { lower: -3.14, upper: 3.14, effort: 10.0, velocity: 2.0 },
    dynamics: { damping: 0.5, friction: 0.1 },
  },
  'shoulder_lift': {
    name: 'shoulder_lift', type: 'revolute', axis: 'Y',
    parentLink: 'shoulder_link', childLink: 'upper_arm',
    limits: { lower: -1.57, upper: 2.36, effort: 10.0, velocity: 1.5 },
    dynamics: { damping: 0.7, friction: 0.15 },
  },
  'elbow': {
    name: 'elbow', type: 'revolute', axis: 'Y',
    parentLink: 'upper_arm', childLink: 'elbow_link',
    limits: { lower: -2.36, upper: 2.36, effort: 1.5, velocity: 2.5 },
    dynamics: { damping: 0.3, friction: 0.05 },
  },
  'forearm_attach': {
    name: 'forearm_attach', type: 'fixed', axis: '--',
    parentLink: 'elbow_link', childLink: 'forearm',
  },
  'wrist_attach': {
    name: 'wrist_attach', type: 'fixed', axis: '--',
    parentLink: 'forearm', childLink: 'wrist',
  },
  'gripper_attach': {
    name: 'gripper_attach', type: 'fixed', axis: '--',
    parentLink: 'wrist', childLink: 'gripper_base',
  },
  'finger_left_joint': {
    name: 'finger_left_joint', type: 'prismatic', axis: 'X',
    parentLink: 'gripper_base', childLink: 'finger_left',
    limits: { lower: -0.01, upper: 0.02, effort: 40, velocity: 0.1 },
  },
  'finger_right_joint': {
    name: 'finger_right_joint', type: 'prismatic', axis: 'X',
    parentLink: 'gripper_base', childLink: 'finger_right',
    limits: { lower: -0.02, upper: 0.01, effort: 40, velocity: 0.1 },
  },
}

export function getParentJoint(linkName: string): JointDetail | undefined {
  return Object.values(JOINT_DETAILS).find(j => j.childLink === linkName)
}

export function getChildJoints(linkName: string): JointDetail[] {
  return Object.values(JOINT_DETAILS).filter(j => j.parentLink === linkName)
}
