// Mate connectors for power components (batteries, converters,
// distribution, supercaps, solar, e-stop). Per the prompt: top/bottom
// planar; no cylindrical (none of these expose a shaft / bore size in
// catalog data, so rule 4 — don't fabricate — applies). terminal_front
// is omitted because the wire-exit direction is not specified per
// preset; better to leave it for the connector authoring UI than to
// guess.
module.exports = {
  power_lipo_3s_2200: [
    { id: "top",    origin_xyz_mm: [0, 0, 12], axis_xyz: [0, 0, 1], type: "planar" },
    { id: "bottom", origin_xyz_mm: [0, 0, -12], axis_xyz: [0, 0, -1], type: "planar" },
  ],
  power_lipo_4s_5000: [
    { id: "top",    origin_xyz_mm: [0, 0, 15], axis_xyz: [0, 0, 1], type: "planar" },
    { id: "bottom", origin_xyz_mm: [0, 0, -15], axis_xyz: [0, 0, -1], type: "planar" },
  ],
  power_lipo_6s_10000: [
    { id: "top",    origin_xyz_mm: [0, 0, 21], axis_xyz: [0, 0, 1], type: "planar" },
    { id: "bottom", origin_xyz_mm: [0, 0, -21], axis_xyz: [0, 0, -1], type: "planar" },
  ],
  power_buck_converter_5v: [
    { id: "top",    origin_xyz_mm: [0, 0, 4], axis_xyz: [0, 0, 1], type: "planar" },
    { id: "bottom", origin_xyz_mm: [0, 0, -4], axis_xyz: [0, 0, -1], type: "planar" },
  ],
  power_buck_converter_12v: [
    { id: "top",    origin_xyz_mm: [0, 0, 6], axis_xyz: [0, 0, 1], type: "planar" },
    { id: "bottom", origin_xyz_mm: [0, 0, -6], axis_xyz: [0, 0, -1], type: "planar" },
  ],
  power_distribution_unit: [
    { id: "top",    origin_xyz_mm: [0, 0, 7.5], axis_xyz: [0, 0, 1], type: "planar" },
    { id: "bottom", origin_xyz_mm: [0, 0, -7.5], axis_xyz: [0, 0, -1], type: "planar" },
  ],
  power_18650_cell_holder_1s: [
    { id: "top",    origin_xyz_mm: [0, 0, 10.5], axis_xyz: [0, 0, 1], type: "planar" },
    { id: "bottom", origin_xyz_mm: [0, 0, -10.5], axis_xyz: [0, 0, -1], type: "planar" },
  ],
  power_18650_4s2p_pack: [
    { id: "top",    origin_xyz_mm: [0, 0, 21], axis_xyz: [0, 0, 1], type: "planar" },
    { id: "bottom", origin_xyz_mm: [0, 0, -21], axis_xyz: [0, 0, -1], type: "planar" },
  ],
  power_supercapacitor_module: [
    { id: "top",    origin_xyz_mm: [0, 0, 12.5], axis_xyz: [0, 0, 1], type: "planar" },
    { id: "bottom", origin_xyz_mm: [0, 0, -12.5], axis_xyz: [0, 0, -1], type: "planar" },
  ],
  power_solar_panel_small: [
    { id: "top",    origin_xyz_mm: [0, 0, 1.5], axis_xyz: [0, 0, 1], type: "planar" },
    { id: "bottom", origin_xyz_mm: [0, 0, -1.5], axis_xyz: [0, 0, -1], type: "planar" },
  ],
  power_usbc_pd_trigger: [
    { id: "top",    origin_xyz_mm: [0, 0, 3], axis_xyz: [0, 0, 1], type: "planar" },
    { id: "bottom", origin_xyz_mm: [0, 0, -3], axis_xyz: [0, 0, -1], type: "planar" },
  ],
  // E-stop button: Z = button axis (mushroom presses along -Z).
  power_estop_switch: [
    { id: "top",    origin_xyz_mm: [0, 0, 27.5], axis_xyz: [0, 0, 1], type: "planar" },
    { id: "bottom", origin_xyz_mm: [0, 0, -27.5], axis_xyz: [0, 0, -1], type: "planar" },
  ],
};
