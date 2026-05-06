// Mate connectors for the motors category. All motors take +Z as the
// principal axis (longest body dimension); axial-shaft motors author
// `shaft_out` (cylindrical) at the +Z face plus planar top/bottom; hub
// motors author `hub_bore` (cylindrical along the axle) plus planar
// top/bottom. Harmonic drives have no exposed shaft diameter so they
// ship planar-only per rule 4 (don't fabricate).
module.exports = {
  motor_dc_small_130: [
    { id: "shaft_out", origin_xyz_mm: [0, 0, 12.5], axis_xyz: [0, 0, 1], type: "cylindrical", diameter_mm: 2 },
    { id: "top",       origin_xyz_mm: [0, 0, 12.5], axis_xyz: [0, 0, 1], type: "planar" },
    { id: "bottom",    origin_xyz_mm: [0, 0, -12.5], axis_xyz: [0, 0, -1], type: "planar" },
  ],
  motor_dc_medium_540: [
    { id: "shaft_out", origin_xyz_mm: [0, 0, 27], axis_xyz: [0, 0, 1], type: "cylindrical", diameter_mm: 3.17 },
    { id: "top",       origin_xyz_mm: [0, 0, 27], axis_xyz: [0, 0, 1], type: "planar" },
    { id: "bottom",    origin_xyz_mm: [0, 0, -27], axis_xyz: [0, 0, -1], type: "planar" },
  ],
  motor_dc_large_775: [
    { id: "shaft_out", origin_xyz_mm: [0, 0, 33], axis_xyz: [0, 0, 1], type: "cylindrical", diameter_mm: 5 },
    { id: "top",       origin_xyz_mm: [0, 0, 33], axis_xyz: [0, 0, 1], type: "planar" },
    { id: "bottom",    origin_xyz_mm: [0, 0, -33], axis_xyz: [0, 0, -1], type: "planar" },
  ],
  motor_gear_small_n20: [
    { id: "shaft_out", origin_xyz_mm: [0, 0, 12.5], axis_xyz: [0, 0, 1], type: "cylindrical", diameter_mm: 3 },
    { id: "top",       origin_xyz_mm: [0, 0, 12.5], axis_xyz: [0, 0, 1], type: "planar" },
    { id: "bottom",    origin_xyz_mm: [0, 0, -12.5], axis_xyz: [0, 0, -1], type: "planar" },
  ],
  motor_gear_medium_37mm: [
    { id: "shaft_out", origin_xyz_mm: [0, 0, 35], axis_xyz: [0, 0, 1], type: "cylindrical", diameter_mm: 6 },
    { id: "top",       origin_xyz_mm: [0, 0, 35], axis_xyz: [0, 0, 1], type: "planar" },
    { id: "bottom",    origin_xyz_mm: [0, 0, -35], axis_xyz: [0, 0, -1], type: "planar" },
  ],
  motor_gear_heavy_50mm: [
    { id: "shaft_out", origin_xyz_mm: [0, 0, 47.5], axis_xyz: [0, 0, 1], type: "cylindrical", diameter_mm: 8 },
    { id: "top",       origin_xyz_mm: [0, 0, 47.5], axis_xyz: [0, 0, 1], type: "planar" },
    { id: "bottom",    origin_xyz_mm: [0, 0, -47.5], axis_xyz: [0, 0, -1], type: "planar" },
  ],
  motor_coreless_dc: [
    { id: "shaft_out", origin_xyz_mm: [0, 0, 8], axis_xyz: [0, 0, 1], type: "cylindrical", diameter_mm: 1 },
    { id: "top",       origin_xyz_mm: [0, 0, 8], axis_xyz: [0, 0, 1], type: "planar" },
    { id: "bottom",    origin_xyz_mm: [0, 0, -8], axis_xyz: [0, 0, -1], type: "planar" },
  ],
  motor_worm_gear: [
    { id: "shaft_out", origin_xyz_mm: [0, 0, 32.5], axis_xyz: [0, 0, 1], type: "cylindrical", diameter_mm: 6 },
    { id: "top",       origin_xyz_mm: [0, 0, 32.5], axis_xyz: [0, 0, 1], type: "planar" },
    { id: "bottom",    origin_xyz_mm: [0, 0, -32.5], axis_xyz: [0, 0, -1], type: "planar" },
  ],
  // Hub motors: through-axle along Z (the thinnest dim of the wheel disc),
  // no protruding shaft. hub_bore opens at +Z; top/bottom = the two flat
  // disc faces.
  motor_hub_80mm: [
    { id: "hub_bore", origin_xyz_mm: [0, 0, 17.5], axis_xyz: [0, 0, 1], type: "cylindrical", diameter_mm: 10 },
    { id: "top",      origin_xyz_mm: [0, 0, 17.5], axis_xyz: [0, 0, 1], type: "planar" },
    { id: "bottom",   origin_xyz_mm: [0, 0, -17.5], axis_xyz: [0, 0, -1], type: "planar" },
  ],
  motor_hub_120mm: [
    { id: "hub_bore", origin_xyz_mm: [0, 0, 27.5], axis_xyz: [0, 0, 1], type: "cylindrical", diameter_mm: 14 },
    { id: "top",      origin_xyz_mm: [0, 0, 27.5], axis_xyz: [0, 0, 1], type: "planar" },
    { id: "bottom",   origin_xyz_mm: [0, 0, -27.5], axis_xyz: [0, 0, -1], type: "planar" },
  ],
  // Harmonic drives: hollow-shaft flanges, no authored shaft diameter -> planar only.
  motor_harmonic_drive_compact: [
    { id: "top",    origin_xyz_mm: [0, 0, 22.5], axis_xyz: [0, 0, 1], type: "planar" },
    { id: "bottom", origin_xyz_mm: [0, 0, -22.5], axis_xyz: [0, 0, -1], type: "planar" },
  ],
  motor_harmonic_drive_large: [
    { id: "top",    origin_xyz_mm: [0, 0, 32.5], axis_xyz: [0, 0, 1], type: "planar" },
    { id: "bottom", origin_xyz_mm: [0, 0, -32.5], axis_xyz: [0, 0, -1], type: "planar" },
  ],
  motor_pancake_dc: [
    { id: "shaft_out", origin_xyz_mm: [0, 0, 9], axis_xyz: [0, 0, 1], type: "cylindrical", diameter_mm: 4 },
    { id: "top",       origin_xyz_mm: [0, 0, 9], axis_xyz: [0, 0, 1], type: "planar" },
    { id: "bottom",    origin_xyz_mm: [0, 0, -9], axis_xyz: [0, 0, -1], type: "planar" },
  ],
  motor_brushless_inrunner_micro: [
    { id: "shaft_out", origin_xyz_mm: [0, 0, 15], axis_xyz: [0, 0, 1], type: "cylindrical", diameter_mm: 2 },
    { id: "top",       origin_xyz_mm: [0, 0, 15], axis_xyz: [0, 0, 1], type: "planar" },
    { id: "bottom",    origin_xyz_mm: [0, 0, -15], axis_xyz: [0, 0, -1], type: "planar" },
  ],
  motor_brushless_inrunner_medium: [
    { id: "shaft_out", origin_xyz_mm: [0, 0, 26], axis_xyz: [0, 0, 1], type: "cylindrical", diameter_mm: 3.17 },
    { id: "top",       origin_xyz_mm: [0, 0, 26], axis_xyz: [0, 0, 1], type: "planar" },
    { id: "bottom",    origin_xyz_mm: [0, 0, -26], axis_xyz: [0, 0, -1], type: "planar" },
  ],
};
