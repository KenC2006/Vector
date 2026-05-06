// Mate connectors for end-effectors. Convention: tool extends along
// +Z (away from the robot wrist), mount face is at -Z. Each preset
// authors mount_back (planar, -Z) plus one working face named for its
// function:
//   - grippers: gripper_open (planar) — mid-plane between the fingers
//   - suction / vacuum / magnetic: tool_face or suction_face (planar)
//   - tool_changer: tool_face (planar) — slave-side mating plane
//   - holders for pen / screwdriver / torch: tip (point)
module.exports = {
  effector_parallel_gripper_small: [
    { id: "mount_back",   origin_xyz_mm: [0, 0, -20], axis_xyz: [0, 0, -1], type: "planar" },
    { id: "gripper_open", origin_xyz_mm: [0, 0, 20], axis_xyz: [0, 0, 1], type: "planar" },
  ],
  effector_parallel_gripper_large: [
    { id: "mount_back",   origin_xyz_mm: [0, 0, -27.5], axis_xyz: [0, 0, -1], type: "planar" },
    { id: "gripper_open", origin_xyz_mm: [0, 0, 27.5], axis_xyz: [0, 0, 1], type: "planar" },
  ],
  effector_3finger_adaptive: [
    { id: "mount_back",   origin_xyz_mm: [0, 0, -45], axis_xyz: [0, 0, -1], type: "planar" },
    { id: "gripper_open", origin_xyz_mm: [0, 0, 45], axis_xyz: [0, 0, 1], type: "planar" },
  ],
  effector_suction_cup: [
    { id: "mount_back",   origin_xyz_mm: [0, 0, -25], axis_xyz: [0, 0, -1], type: "planar" },
    { id: "suction_face", origin_xyz_mm: [0, 0, 25], axis_xyz: [0, 0, 1], type: "planar" },
  ],
  effector_magnetic_tool: [
    { id: "mount_back", origin_xyz_mm: [0, 0, -15], axis_xyz: [0, 0, -1], type: "planar" },
    { id: "tool_face",  origin_xyz_mm: [0, 0, 15], axis_xyz: [0, 0, 1], type: "planar" },
  ],
  effector_tool_changer: [
    { id: "mount_back", origin_xyz_mm: [0, 0, -12.5], axis_xyz: [0, 0, -1], type: "planar" },
    { id: "tool_face",  origin_xyz_mm: [0, 0, 12.5], axis_xyz: [0, 0, 1], type: "planar" },
  ],
  effector_soft_gripper: [
    { id: "mount_back",   origin_xyz_mm: [0, 0, -30], axis_xyz: [0, 0, -1], type: "planar" },
    { id: "gripper_open", origin_xyz_mm: [0, 0, 30], axis_xyz: [0, 0, 1], type: "planar" },
  ],
  effector_vacuum_pad_array: [
    { id: "mount_back",   origin_xyz_mm: [0, 0, -20], axis_xyz: [0, 0, -1], type: "planar" },
    { id: "suction_face", origin_xyz_mm: [0, 0, 20], axis_xyz: [0, 0, 1], type: "planar" },
  ],
  effector_welding_torch_holder: [
    { id: "mount_back", origin_xyz_mm: [0, 0, -40], axis_xyz: [0, 0, -1], type: "planar" },
    { id: "tip",        origin_xyz_mm: [0, 0, 40], axis_xyz: [0, 0, 1], type: "point" },
  ],
  effector_pen_marker_holder: [
    { id: "mount_back", origin_xyz_mm: [0, 0, -30], axis_xyz: [0, 0, -1], type: "planar" },
    { id: "tip",        origin_xyz_mm: [0, 0, 30], axis_xyz: [0, 0, 1], type: "point" },
  ],
  effector_screwdriver_holder: [
    { id: "mount_back", origin_xyz_mm: [0, 0, -50], axis_xyz: [0, 0, -1], type: "planar" },
    { id: "tip",        origin_xyz_mm: [0, 0, 50], axis_xyz: [0, 0, 1], type: "point" },
  ],
};
