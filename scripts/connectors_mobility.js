// Mate connectors for mobility components.
// Wheels: axle through Z (perpendicular to the disc), so hub_bore is
// cylindrical along Z. Catalog data only states "input: axial_shaft"
// without a numeric bore — per rule 4 we author cylindrical without
// diameter_mm rather than fabricate one. tread_outer ships as a point
// on the rim (radial outward, picked along -Y as a stable canonical
// direction) so the engine knows the wheel has a ground-contact rim.
// Casters and rubber foot pads follow the prompt's mount_top /
// contact_bottom convention.
module.exports = {
  mobility_wheel_driven: [
    { id: "hub_bore",     origin_xyz_mm: [0, 0, 15], axis_xyz: [0, 0, 1], type: "cylindrical" },
    { id: "tread_outer",  origin_xyz_mm: [0, -50, 0], axis_xyz: [0, -1, 0], type: "point" },
    { id: "top",          origin_xyz_mm: [0, 0, 15], axis_xyz: [0, 0, 1], type: "planar" },
    { id: "bottom",       origin_xyz_mm: [0, 0, -15], axis_xyz: [0, 0, -1], type: "planar" },
  ],
  mobility_caster_wheel: [
    { id: "mount_top",       origin_xyz_mm: [0, 0, 32.5], axis_xyz: [0, 0, 1], type: "planar" },
    { id: "contact_bottom",  origin_xyz_mm: [0, 0, -32.5], axis_xyz: [0, 0, -1], type: "planar" },
  ],
  mobility_mecanum_wheel: [
    { id: "hub_bore",     origin_xyz_mm: [0, 0, 24], axis_xyz: [0, 0, 1], type: "cylindrical" },
    { id: "tread_outer",  origin_xyz_mm: [0, -50, 0], axis_xyz: [0, -1, 0], type: "point" },
    { id: "top",          origin_xyz_mm: [0, 0, 24], axis_xyz: [0, 0, 1], type: "planar" },
    { id: "bottom",       origin_xyz_mm: [0, 0, -24], axis_xyz: [0, 0, -1], type: "planar" },
  ],
  mobility_omni_wheel: [
    { id: "hub_bore",     origin_xyz_mm: [0, 0, 12.5], axis_xyz: [0, 0, 1], type: "cylindrical" },
    { id: "tread_outer",  origin_xyz_mm: [0, -30, 0], axis_xyz: [0, -1, 0], type: "point" },
    { id: "top",          origin_xyz_mm: [0, 0, 12.5], axis_xyz: [0, 0, 1], type: "planar" },
    { id: "bottom",       origin_xyz_mm: [0, 0, -12.5], axis_xyz: [0, 0, -1], type: "planar" },
  ],
  // Track system: long rubber tread along X, mounts to chassis on +Z,
  // rolls along -Z. Drive-sprocket shaft axis is unknown (no bore_mm),
  // so no cylindrical authoring.
  mobility_track_tread_system: [
    { id: "mount_top",      origin_xyz_mm: [0, 0, 30], axis_xyz: [0, 0, 1], type: "planar" },
    { id: "contact_bottom", origin_xyz_mm: [0, 0, -30], axis_xyz: [0, 0, -1], type: "planar" },
  ],
  // Swerve module hangs below chassis. Top of module bolts to chassis,
  // wheel rim contacts ground at -Z.
  mobility_swerve_drive_module: [
    { id: "mount_top",      origin_xyz_mm: [0, 0, 55], axis_xyz: [0, 0, 1], type: "planar" },
    { id: "contact_bottom", origin_xyz_mm: [0, 0, -55], axis_xyz: [0, 0, -1], type: "planar" },
  ],
  // Ball transfer: flange bolts up (+Z), ball rolls on -Z.
  mobility_ball_transfer_unit: [
    { id: "mount_top",      origin_xyz_mm: [0, 0, 11], axis_xyz: [0, 0, 1], type: "planar" },
    { id: "contact_bottom", origin_xyz_mm: [0, 0, -11], axis_xyz: [0, 0, -1], type: "planar" },
  ],
  // Foot pad: M6 stud screws into chassis on +Z, rubber pad on -Z.
  mobility_rubber_foot_pad: [
    { id: "mount_top",      origin_xyz_mm: [0, 0, 7.5], axis_xyz: [0, 0, 1], type: "planar" },
    { id: "contact_bottom", origin_xyz_mm: [0, 0, -7.5], axis_xyz: [0, 0, -1], type: "planar" },
  ],
};
