// Mate connectors for drivetrain components. None of the five presets
// expose a numeric shaft_diameter_mm in catalog data — they declare
// "output: axial_shaft" without a size — so per rule 4 they ship
// planar top/bottom only. Hub motor / geared motor mate to wheels via
// the wheel's hub_bore (authored in mobility); the driveshaft axis is
// inferred from the parent's +Z, not from a shaft_out connector here.
module.exports = {
  drivetrain_hub_motor_80: [
    { id: "top",    origin_xyz_mm: [0, 0, 22.5], axis_xyz: [0, 0, 1], type: "planar" },
    { id: "bottom", origin_xyz_mm: [0, 0, -22.5], axis_xyz: [0, 0, -1], type: "planar" },
  ],
  drivetrain_geared_dc_with_coupler: [
    { id: "top",    origin_xyz_mm: [0, 0, 45], axis_xyz: [0, 0, 1], type: "planar" },
    { id: "bottom", origin_xyz_mm: [0, 0, -45], axis_xyz: [0, 0, -1], type: "planar" },
  ],
  drivetrain_stub_axle_passive: [
    { id: "top",    origin_xyz_mm: [0, 0, 15], axis_xyz: [0, 0, 1], type: "planar" },
    { id: "bottom", origin_xyz_mm: [0, 0, -15], axis_xyz: [0, 0, -1], type: "planar" },
  ],
  drivetrain_caster_swivel: [
    { id: "top",    origin_xyz_mm: [0, 0, 35], axis_xyz: [0, 0, 1], type: "planar" },
    { id: "bottom", origin_xyz_mm: [0, 0, -35], axis_xyz: [0, 0, -1], type: "planar" },
  ],
  drivetrain_steering_knuckle: [
    { id: "top",    origin_xyz_mm: [0, 0, 40], axis_xyz: [0, 0, 1], type: "planar" },
    { id: "bottom", origin_xyz_mm: [0, 0, -40], axis_xyz: [0, 0, -1], type: "planar" },
  ],
};
