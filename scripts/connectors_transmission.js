// Mate connectors for transmission components.
// Single-axis parts (couplings, bearings, planetary gearbox, U-joint,
// bushings) get authored cylindrical bores from their explicit bore_mm
// / output_shaft_mm. Gear pairs/sets with two different axes get
// planar top/bottom only — picking one bore as "the" shaft would
// silently mislead the engine on the orthogonal mate.
//
// DEFERRED (logged in commit): timing_belt, chain_sprocket, rack_pinion
// (parametric belt/chain/rack length), and leadscrew/ballscrew (no
// bounding_box_mm at all — parametric length).
module.exports = {
  // Deep-groove ball bearing: inner bore Z + the two flat ring faces.
  transmission_bearing_deep_groove: [
    { id: "inner_bore", origin_xyz_mm: [0, 0, 3.5], axis_xyz: [0, 0, 1], type: "cylindrical", diameter_mm: 8 },
    { id: "top",        origin_xyz_mm: [0, 0, 3.5], axis_xyz: [0, 0, 1], type: "planar" },
    { id: "bottom",     origin_xyz_mm: [0, 0, -3.5], axis_xyz: [0, 0, -1], type: "planar" },
  ],
  transmission_bearing_large: [
    { id: "inner_bore", origin_xyz_mm: [0, 0, 7.5], axis_xyz: [0, 0, 1], type: "cylindrical", diameter_mm: 25 },
    { id: "top",        origin_xyz_mm: [0, 0, 7.5], axis_xyz: [0, 0, 1], type: "planar" },
    { id: "bottom",     origin_xyz_mm: [0, 0, -7.5], axis_xyz: [0, 0, -1], type: "planar" },
  ],
  // Flanged bushing: bore through Z; flange on +Z side.
  transmission_bushing_flanged: [
    { id: "inner_bore", origin_xyz_mm: [0, 0, 5], axis_xyz: [0, 0, 1], type: "cylindrical", diameter_mm: 8 },
    { id: "top",        origin_xyz_mm: [0, 0, 5], axis_xyz: [0, 0, 1], type: "planar" },
    { id: "bottom",     origin_xyz_mm: [0, 0, -5], axis_xyz: [0, 0, -1], type: "planar" },
  ],
  // Planetary gearbox: NEMA17-class face mount on -Z, axial output on
  // +Z (output_shaft_mm = 8). Input bore not exposed in catalog data.
  transmission_planetary_gearbox: [
    { id: "shaft_out", origin_xyz_mm: [0, 0, 19], axis_xyz: [0, 0, 1], type: "cylindrical", diameter_mm: 8 },
    { id: "top",       origin_xyz_mm: [0, 0, 19], axis_xyz: [0, 0, 1], type: "planar" },
    { id: "bottom",    origin_xyz_mm: [0, 0, -19], axis_xyz: [0, 0, -1], type: "planar" },
  ],
  // Spur-gear PAIR: two gears side-by-side, single shared face plane —
  // no single shaft to anchor cylindrical to. Planar only.
  transmission_spur_gear_pair: [
    { id: "top",    origin_xyz_mm: [0, 0, 5], axis_xyz: [0, 0, 1], type: "planar" },
    { id: "bottom", origin_xyz_mm: [0, 0, -5], axis_xyz: [0, 0, -1], type: "planar" },
  ],
  // Bevel-gear PAIR: 90-deg shaft angle. Two non-parallel axes; planar
  // only is the safe choice (rule 4 — don't fabricate axis identity).
  transmission_bevel_gear_pair: [
    { id: "top",    origin_xyz_mm: [0, 0, 17.5], axis_xyz: [0, 0, 1], type: "planar" },
    { id: "bottom", origin_xyz_mm: [0, 0, -17.5], axis_xyz: [0, 0, -1], type: "planar" },
  ],
  // Worm-gear SET: also 90-deg angle. Planar only.
  transmission_worm_gear_set: [
    { id: "top",    origin_xyz_mm: [0, 0, 15], axis_xyz: [0, 0, 1], type: "planar" },
    { id: "bottom", origin_xyz_mm: [0, 0, -15], axis_xyz: [0, 0, -1], type: "planar" },
  ],
  // Universal joint: equal bores at each Z end (8mm both sides).
  transmission_universal_joint: [
    { id: "shaft_in",  origin_xyz_mm: [0, 0, -17.5], axis_xyz: [0, 0, -1], type: "cylindrical", diameter_mm: 8 },
    { id: "shaft_out", origin_xyz_mm: [0, 0, 17.5], axis_xyz: [0, 0, 1], type: "cylindrical", diameter_mm: 8 },
    { id: "top",       origin_xyz_mm: [0, 0, 17.5], axis_xyz: [0, 0, 1], type: "planar" },
    { id: "bottom",    origin_xyz_mm: [0, 0, -17.5], axis_xyz: [0, 0, -1], type: "planar" },
  ],
  // Jaw coupling: asymmetric bore (5mm input, 8mm output).
  transmission_flexible_coupling_jaw: [
    { id: "shaft_in",  origin_xyz_mm: [0, 0, -15], axis_xyz: [0, 0, -1], type: "cylindrical", diameter_mm: 5 },
    { id: "shaft_out", origin_xyz_mm: [0, 0, 15], axis_xyz: [0, 0, 1], type: "cylindrical", diameter_mm: 8 },
    { id: "top",       origin_xyz_mm: [0, 0, 15], axis_xyz: [0, 0, 1], type: "planar" },
    { id: "bottom",    origin_xyz_mm: [0, 0, -15], axis_xyz: [0, 0, -1], type: "planar" },
  ],
  // Rigid shaft coupling: equal bores both sides (8mm).
  transmission_rigid_shaft_coupling: [
    { id: "shaft_in",  origin_xyz_mm: [0, 0, -12.5], axis_xyz: [0, 0, -1], type: "cylindrical", diameter_mm: 8 },
    { id: "shaft_out", origin_xyz_mm: [0, 0, 12.5], axis_xyz: [0, 0, 1], type: "cylindrical", diameter_mm: 8 },
    { id: "top",       origin_xyz_mm: [0, 0, 12.5], axis_xyz: [0, 0, 1], type: "planar" },
    { id: "bottom",    origin_xyz_mm: [0, 0, -12.5], axis_xyz: [0, 0, -1], type: "planar" },
  ],
  // Crossed-roller bearing: large bore (50mm) on Z + ring faces.
  transmission_crossed_roller_bearing: [
    { id: "inner_bore", origin_xyz_mm: [0, 0, 7.5], axis_xyz: [0, 0, 1], type: "cylindrical", diameter_mm: 50 },
    { id: "top",        origin_xyz_mm: [0, 0, 7.5], axis_xyz: [0, 0, 1], type: "planar" },
    { id: "bottom",     origin_xyz_mm: [0, 0, -7.5], axis_xyz: [0, 0, -1], type: "planar" },
  ],
  transmission_slewing_ring_bearing: [
    { id: "inner_bore", origin_xyz_mm: [0, 0, 12.5], axis_xyz: [0, 0, 1], type: "cylindrical", diameter_mm: 130 },
    { id: "top",        origin_xyz_mm: [0, 0, 12.5], axis_xyz: [0, 0, 1], type: "planar" },
    { id: "bottom",     origin_xyz_mm: [0, 0, -12.5], axis_xyz: [0, 0, -1], type: "planar" },
  ],
};
