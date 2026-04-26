// Mate connectors for sensors. Rules of thumb:
//
//  A. Elongated sensors (one dim clearly longest — depth cameras, load
//     cells, ultrasonic) follow the existing sensor_depth_camera_small
//     convention: X is the principal axis, mount_back at -X, the named
//     working face at +X.
//
//  B. Flat PCB-style modules (Z is the thinnest axis) bolt on -Z and
//     present their working face on +Z. Color sensor inverts because
//     sensor_face=downward in the catalog data.
//
//  C. Spinning LiDARs (omnidirectional in XY) and inline F/T / load
//     cells get mount_back + top / tool_face on Z; no cylindrical
//     authoring because there is no shaft.
//
//  D. Parts with an actual shaft (rotary potentiometer, joint encoder
//     bore) get a cylindrical connector pulled from the catalog's
//     shaft_diameter_mm / shaft_bore_mm.
module.exports = {
  // A — long camera barrel
  sensor_depth_camera_wide: [
    { id: "mount_back",    origin_xyz_mm: [-62, 0, 0], axis_xyz: [-1, 0, 0], type: "planar" },
    { id: "optical_front", origin_xyz_mm: [62, 0, 0], axis_xyz: [1, 0, 0], type: "point" },
  ],
  // C — spinning LiDARs
  sensor_lidar_2d: [
    { id: "mount_back", origin_xyz_mm: [0, 0, -20.5], axis_xyz: [0, 0, -1], type: "planar" },
    { id: "top",        origin_xyz_mm: [0, 0, 20.5], axis_xyz: [0, 0, 1], type: "planar" },
  ],
  sensor_lidar_3d: [
    { id: "mount_back", origin_xyz_mm: [0, 0, -35.85], axis_xyz: [0, 0, -1], type: "planar" },
    { id: "top",        origin_xyz_mm: [0, 0, 35.85], axis_xyz: [0, 0, 1], type: "planar" },
  ],
  // B — flat PCB modules (no directional sense; mount on -Z)
  sensor_imu_6dof: [
    { id: "mount_back", origin_xyz_mm: [0, 0, -1.5], axis_xyz: [0, 0, -1], type: "planar" },
    { id: "top",        origin_xyz_mm: [0, 0, 1.5], axis_xyz: [0, 0, 1], type: "planar" },
  ],
  sensor_imu_9dof: [
    { id: "mount_back", origin_xyz_mm: [0, 0, -2], axis_xyz: [0, 0, -1], type: "planar" },
    { id: "top",        origin_xyz_mm: [0, 0, 2], axis_xyz: [0, 0, 1], type: "planar" },
  ],
  // A — long ultrasonic body, sensing along +X
  sensor_ultrasonic: [
    { id: "mount_back",   origin_xyz_mm: [-22.5, 0, 0], axis_xyz: [-1, 0, 0], type: "planar" },
    { id: "sensing_face", origin_xyz_mm: [22.5, 0, 0], axis_xyz: [1, 0, 0], type: "planar" },
  ],
  // B — ToF: lens emits out of PCB face (+Z)
  sensor_tof: [
    { id: "mount_back",   origin_xyz_mm: [0, 0, -1], axis_xyz: [0, 0, -1], type: "planar" },
    { id: "sensing_face", origin_xyz_mm: [0, 0, 1], axis_xyz: [0, 0, 1], type: "planar" },
  ],
  // C — inline F/T sensor: fixed plate at -Z, tool plate at +Z
  sensor_force_torque_6axis: [
    { id: "mount_back", origin_xyz_mm: [0, 0, -10], axis_xyz: [0, 0, -1], type: "planar" },
    { id: "tool_face",  origin_xyz_mm: [0, 0, 10], axis_xyz: [0, 0, 1], type: "planar" },
  ],
  // D — encoder body with through-bore for joint shaft (6mm bore on Z)
  sensor_joint_encoder_absolute: [
    { id: "shaft_hole", origin_xyz_mm: [0, 0, 5], axis_xyz: [0, 0, 1], type: "cylindrical", diameter_mm: 6 },
    { id: "mount_back", origin_xyz_mm: [0, 0, -5], axis_xyz: [0, 0, -1], type: "planar" },
    { id: "top",        origin_xyz_mm: [0, 0, 5], axis_xyz: [0, 0, 1], type: "planar" },
  ],
  // B — thermal IR breakout module: lens out +Z
  sensor_thermal_ir_camera: [
    { id: "mount_back",   origin_xyz_mm: [0, 0, -5], axis_xyz: [0, 0, -1], type: "planar" },
    { id: "sensing_face", origin_xyz_mm: [0, 0, 5], axis_xyz: [0, 0, 1], type: "planar" },
  ],
  // A — bumper: contact face on +X
  sensor_contact_bumper: [
    { id: "mount_back",   origin_xyz_mm: [-14, 0, 0], axis_xyz: [-1, 0, 0], type: "planar" },
    { id: "sensing_face", origin_xyz_mm: [14, 0, 0], axis_xyz: [1, 0, 0], type: "planar" },
  ],
  // A — limit switch: lever extends along +X past the body
  sensor_limit_switch: [
    { id: "mount_back",   origin_xyz_mm: [-10, 0, 0], axis_xyz: [-1, 0, 0], type: "planar" },
    { id: "sensing_face", origin_xyz_mm: [10, 0, 0], axis_xyz: [1, 0, 0], type: "planar" },
  ],
  // D — pot: panel-mount shaft on +Z
  sensor_rotary_potentiometer: [
    { id: "shaft_out",  origin_xyz_mm: [0, 0, 6], axis_xyz: [0, 0, 1], type: "cylindrical", diameter_mm: 6 },
    { id: "mount_back", origin_xyz_mm: [0, 0, -6], axis_xyz: [0, 0, -1], type: "planar" },
    { id: "top",        origin_xyz_mm: [0, 0, 6], axis_xyz: [0, 0, 1], type: "planar" },
  ],
  // A — inline load cell along X
  sensor_load_cell: [
    { id: "mount_back",   origin_xyz_mm: [-27.5, 0, 0], axis_xyz: [-1, 0, 0], type: "planar" },
    { id: "payload_face", origin_xyz_mm: [27.5, 0, 0], axis_xyz: [1, 0, 0], type: "planar" },
  ],
  // B — current sensor breakout
  sensor_current: [
    { id: "mount_back", origin_xyz_mm: [0, 0, -6], axis_xyz: [0, 0, -1], type: "planar" },
    { id: "top",        origin_xyz_mm: [0, 0, 6], axis_xyz: [0, 0, 1], type: "planar" },
  ],
  // B — voltage divider PCB
  sensor_voltage_divider: [
    { id: "mount_back", origin_xyz_mm: [0, 0, -2.5], axis_xyz: [0, 0, -1], type: "planar" },
    { id: "top",        origin_xyz_mm: [0, 0, 2.5], axis_xyz: [0, 0, 1], type: "planar" },
  ],
  // B inverted — color sensor faces downward (-Z)
  sensor_color_light: [
    { id: "mount_back",   origin_xyz_mm: [0, 0, 1.5], axis_xyz: [0, 0, 1], type: "planar" },
    { id: "sensing_face", origin_xyz_mm: [0, 0, -1.5], axis_xyz: [0, 0, -1], type: "planar" },
  ],
  // B — barometric breakout
  sensor_barometric_pressure: [
    { id: "mount_back", origin_xyz_mm: [0, 0, -1.5], axis_xyz: [0, 0, -1], type: "planar" },
    { id: "top",        origin_xyz_mm: [0, 0, 1.5], axis_xyz: [0, 0, 1], type: "planar" },
  ],
};
