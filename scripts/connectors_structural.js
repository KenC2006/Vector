// Mate connectors for non-parametric structural parts. Per rule 5,
// the parametric-length structural family (extrusions, tubes,
// I-beams, channels, angles, flat bars, round bars, threaded rods,
// sheets, linear rails, DIN rail) is intentionally NOT authored —
// the bake path special-cases them and bbox-based origins would be
// wrong at any non-default length.
//
// Authored (9):
//   bracket_u, hip_housing_2dof, standoff_m3/m4, t_bracket,
//   corner_cube, pillow_block_mount, shaft_collar,
//   linear_rail_carriage_mgn12h
//
// Convention notes:
//   - bracket_u and corner_cube have multiple functionally mountable
//     faces, so they ship the side-face set (front/back/left/right).
//   - hip_housing_2dof is a multi-face servo housing — all 6 faces
//     are potential mount surfaces.
//   - pillow_block_mount and shaft_collar both have an authored 8mm
//     bore: pillow block's bore is along its long axis (X), collar's
//     bore is through the disc (Z).
//   - standoffs and the rail carriage stay simple top/bottom on the
//     thin / through axis.
module.exports = {
  // U-bracket — servo mounts inside the U.
  structural_bracket_u: [
    { id: "top",    origin_xyz_mm: [0, 0, 20], axis_xyz: [0, 0, 1], type: "planar" },
    { id: "bottom", origin_xyz_mm: [0, 0, -20], axis_xyz: [0, 0, -1], type: "planar" },
    { id: "left",   origin_xyz_mm: [-25, 0, 0], axis_xyz: [-1, 0, 0], type: "planar" },
    { id: "right",  origin_xyz_mm: [25, 0, 0], axis_xyz: [1, 0, 0], type: "planar" },
    { id: "front",  origin_xyz_mm: [0, 15, 0], axis_xyz: [0, 1, 0], type: "planar" },
    { id: "back",   origin_xyz_mm: [0, -15, 0], axis_xyz: [0, -1, 0], type: "planar" },
  ],
  // 2-DOF hip housing — every face can mount something.
  structural_hip_housing_2dof: [
    { id: "top",    origin_xyz_mm: [0, 0, 20], axis_xyz: [0, 0, 1], type: "planar" },
    { id: "bottom", origin_xyz_mm: [0, 0, -20], axis_xyz: [0, 0, -1], type: "planar" },
    { id: "left",   origin_xyz_mm: [-33, 0, 0], axis_xyz: [-1, 0, 0], type: "planar" },
    { id: "right",  origin_xyz_mm: [33, 0, 0], axis_xyz: [1, 0, 0], type: "planar" },
    { id: "front",  origin_xyz_mm: [0, 29, 0], axis_xyz: [0, 1, 0], type: "planar" },
    { id: "back",   origin_xyz_mm: [0, -29, 0], axis_xyz: [0, -1, 0], type: "planar" },
  ],
  // M3 hex standoff — top/bottom thread faces only.
  structural_standoff_m3: [
    { id: "top",    origin_xyz_mm: [0, 0, 7.5], axis_xyz: [0, 0, 1], type: "planar" },
    { id: "bottom", origin_xyz_mm: [0, 0, -7.5], axis_xyz: [0, 0, -1], type: "planar" },
  ],
  structural_standoff_m4: [
    { id: "top",    origin_xyz_mm: [0, 0, 10], axis_xyz: [0, 0, 1], type: "planar" },
    { id: "bottom", origin_xyz_mm: [0, 0, -10], axis_xyz: [0, 0, -1], type: "planar" },
  ],
  // T-bracket: 3mm-thick flat plate.
  structural_t_bracket: [
    { id: "top",    origin_xyz_mm: [0, 0, 1.5], axis_xyz: [0, 0, 1], type: "planar" },
    { id: "bottom", origin_xyz_mm: [0, 0, -1.5], axis_xyz: [0, 0, -1], type: "planar" },
  ],
  // Corner cube — 3-way orthogonal connector. All 6 faces mate.
  structural_corner_cube: [
    { id: "top",    origin_xyz_mm: [0, 0, 10], axis_xyz: [0, 0, 1], type: "planar" },
    { id: "bottom", origin_xyz_mm: [0, 0, -10], axis_xyz: [0, 0, -1], type: "planar" },
    { id: "left",   origin_xyz_mm: [-10, 0, 0], axis_xyz: [-1, 0, 0], type: "planar" },
    { id: "right",  origin_xyz_mm: [10, 0, 0], axis_xyz: [1, 0, 0], type: "planar" },
    { id: "front",  origin_xyz_mm: [0, 10, 0], axis_xyz: [0, 1, 0], type: "planar" },
    { id: "back",   origin_xyz_mm: [0, -10, 0], axis_xyz: [0, -1, 0], type: "planar" },
  ],
  // Pillow block: shaft passes horizontally along the body's longest
  // axis (X). 8mm bore from catalog.
  structural_pillow_block_mount: [
    { id: "shaft_hole", origin_xyz_mm: [15, 0, 0], axis_xyz: [1, 0, 0], type: "cylindrical", diameter_mm: 8 },
    { id: "top",        origin_xyz_mm: [0, 0, 12.5], axis_xyz: [0, 0, 1], type: "planar" },
    { id: "bottom",     origin_xyz_mm: [0, 0, -12.5], axis_xyz: [0, 0, -1], type: "planar" },
  ],
  // Shaft collar: through-bore along Z (the thin disc axis), 8mm.
  structural_shaft_collar: [
    { id: "shaft_hole", origin_xyz_mm: [0, 0, 4.5], axis_xyz: [0, 0, 1], type: "cylindrical", diameter_mm: 8 },
    { id: "top",        origin_xyz_mm: [0, 0, 4.5], axis_xyz: [0, 0, 1], type: "planar" },
    { id: "bottom",     origin_xyz_mm: [0, 0, -4.5], axis_xyz: [0, 0, -1], type: "planar" },
  ],
  // MGN12H carriage: top mounts to payload, bottom slides on rail.
  structural_linear_rail_carriage_mgn12h: [
    { id: "top",    origin_xyz_mm: [0, 0, 5], axis_xyz: [0, 0, 1], type: "planar" },
    { id: "bottom", origin_xyz_mm: [0, 0, -5], axis_xyz: [0, 0, -1], type: "planar" },
  ],
};
