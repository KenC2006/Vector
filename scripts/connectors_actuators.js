// Mate connectors for the non-servo actuators. Steppers / BLDCs follow
// the servo pattern (shaft_out + top + bottom). Linear actuators get
// rod_out (cylindrical, no authored diameter — clevis_pin_mm is the
// transverse pin, not the rod cross-section, so per rule 4 we omit
// diameter_mm rather than fabricate one). Continuous-rotation and
// high-speed mini servos take shaft_out from their authored
// shaft_diameter_mm.
module.exports = {
  actuator_bldc_small: [
    { id: "shaft_out", origin_xyz_mm: [0, 0, 24], axis_xyz: [0, 0, 1], type: "cylindrical", diameter_mm: 10 },
    { id: "top",       origin_xyz_mm: [0, 0, 24], axis_xyz: [0, 0, 1], type: "planar" },
    { id: "bottom",    origin_xyz_mm: [0, 0, -24], axis_xyz: [0, 0, -1], type: "planar" },
  ],
  actuator_bldc_large: [
    { id: "shaft_out", origin_xyz_mm: [0, 0, 27.5], axis_xyz: [0, 0, 1], type: "cylindrical", diameter_mm: 14 },
    { id: "top",       origin_xyz_mm: [0, 0, 27.5], axis_xyz: [0, 0, 1], type: "planar" },
    { id: "bottom",    origin_xyz_mm: [0, 0, -27.5], axis_xyz: [0, 0, -1], type: "planar" },
  ],
  actuator_stepper_nema17: [
    { id: "shaft_out", origin_xyz_mm: [0, 0, 24], axis_xyz: [0, 0, 1], type: "cylindrical", diameter_mm: 5 },
    { id: "top",       origin_xyz_mm: [0, 0, 24], axis_xyz: [0, 0, 1], type: "planar" },
    { id: "bottom",    origin_xyz_mm: [0, 0, -24], axis_xyz: [0, 0, -1], type: "planar" },
  ],
  actuator_stepper_nema23: [
    { id: "shaft_out", origin_xyz_mm: [0, 0, 28], axis_xyz: [0, 0, 1], type: "cylindrical", diameter_mm: 6.35 },
    { id: "top",       origin_xyz_mm: [0, 0, 28], axis_xyz: [0, 0, 1], type: "planar" },
    { id: "bottom",    origin_xyz_mm: [0, 0, -28], axis_xyz: [0, 0, -1], type: "planar" },
  ],
  // Linear actuators: rod extends along +Z. No rod-cross-section diameter
  // in catalog data, so cylindrical connector is authored without
  // diameter_mm (schema marks it optional).
  actuator_linear_small: [
    { id: "rod_out", origin_xyz_mm: [0, 0, 65], axis_xyz: [0, 0, 1], type: "cylindrical" },
    { id: "top",     origin_xyz_mm: [0, 0, 65], axis_xyz: [0, 0, 1], type: "planar" },
    { id: "bottom",  origin_xyz_mm: [0, 0, -65], axis_xyz: [0, 0, -1], type: "planar" },
  ],
  actuator_linear_heavy: [
    { id: "rod_out", origin_xyz_mm: [0, 0, 175], axis_xyz: [0, 0, 1], type: "cylindrical" },
    { id: "top",     origin_xyz_mm: [0, 0, 175], axis_xyz: [0, 0, 1], type: "planar" },
    { id: "bottom",  origin_xyz_mm: [0, 0, -175], axis_xyz: [0, 0, -1], type: "planar" },
  ],
  actuator_micro_linear_servo: [
    { id: "rod_out", origin_xyz_mm: [0, 0, 16], axis_xyz: [0, 0, 1], type: "cylindrical" },
    { id: "top",     origin_xyz_mm: [0, 0, 16], axis_xyz: [0, 0, 1], type: "planar" },
    { id: "bottom",  origin_xyz_mm: [0, 0, -16], axis_xyz: [0, 0, -1], type: "planar" },
  ],
  actuator_continuous_rotation_servo: [
    { id: "shaft_out", origin_xyz_mm: [0, 0, 18.5], axis_xyz: [0, 0, 1], type: "cylindrical", diameter_mm: 5.9 },
    { id: "top",       origin_xyz_mm: [0, 0, 18.5], axis_xyz: [0, 0, 1], type: "planar" },
    { id: "bottom",    origin_xyz_mm: [0, 0, -18.5], axis_xyz: [0, 0, -1], type: "planar" },
  ],
  actuator_high_speed_mini_servo: [
    { id: "shaft_out", origin_xyz_mm: [0, 0, 15], axis_xyz: [0, 0, 1], type: "cylindrical", diameter_mm: 4.6 },
    { id: "top",       origin_xyz_mm: [0, 0, 15], axis_xyz: [0, 0, 1], type: "planar" },
    { id: "bottom",    origin_xyz_mm: [0, 0, -15], axis_xyz: [0, 0, -1], type: "planar" },
  ],
};
