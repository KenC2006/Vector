// Mate connectors for compute boards. All 11 presets are flat PCBs:
// top/bottom planar on Z (the thinnest dim). connector_edge is omitted
// because the catalog doesn't surface which side the USB/I2C edge
// faces; better to defer that detail to the connector authoring UI
// than guess.
module.exports = {
  compute_mcu_small: [
    { id: "top",    origin_xyz_mm: [0, 0, 2.5], axis_xyz: [0, 0, 1], type: "planar" },
    { id: "bottom", origin_xyz_mm: [0, 0, -2.5], axis_xyz: [0, 0, -1], type: "planar" },
  ],
  compute_sbc_small: [
    { id: "top",    origin_xyz_mm: [0, 0, 8.5], axis_xyz: [0, 0, 1], type: "planar" },
    { id: "bottom", origin_xyz_mm: [0, 0, -8.5], axis_xyz: [0, 0, -1], type: "planar" },
  ],
  compute_sbc_gpu: [
    { id: "top",    origin_xyz_mm: [0, 0, 10.5], axis_xyz: [0, 0, 1], type: "planar" },
    { id: "bottom", origin_xyz_mm: [0, 0, -10.5], axis_xyz: [0, 0, -1], type: "planar" },
  ],
  compute_motor_driver_dual: [
    { id: "top",    origin_xyz_mm: [0, 0, 7.5], axis_xyz: [0, 0, 1], type: "planar" },
    { id: "bottom", origin_xyz_mm: [0, 0, -7.5], axis_xyz: [0, 0, -1], type: "planar" },
  ],
  compute_foc_controller: [
    { id: "top",    origin_xyz_mm: [0, 0, 6], axis_xyz: [0, 0, 1], type: "planar" },
    { id: "bottom", origin_xyz_mm: [0, 0, -6], axis_xyz: [0, 0, -1], type: "planar" },
  ],
  compute_fpga_dev_board: [
    { id: "top",    origin_xyz_mm: [0, 0, 5], axis_xyz: [0, 0, 1], type: "planar" },
    { id: "bottom", origin_xyz_mm: [0, 0, -5], axis_xyz: [0, 0, -1], type: "planar" },
  ],
  compute_can_transceiver: [
    { id: "top",    origin_xyz_mm: [0, 0, 4], axis_xyz: [0, 0, 1], type: "planar" },
    { id: "bottom", origin_xyz_mm: [0, 0, -4], axis_xyz: [0, 0, -1], type: "planar" },
  ],
  compute_usb_hub: [
    { id: "top",    origin_xyz_mm: [0, 0, 4], axis_xyz: [0, 0, 1], type: "planar" },
    { id: "bottom", origin_xyz_mm: [0, 0, -4], axis_xyz: [0, 0, -1], type: "planar" },
  ],
  compute_wireless_24ghz: [
    { id: "top",    origin_xyz_mm: [0, 0, 2.5], axis_xyz: [0, 0, 1], type: "planar" },
    { id: "bottom", origin_xyz_mm: [0, 0, -2.5], axis_xyz: [0, 0, -1], type: "planar" },
  ],
  compute_lora_radio: [
    { id: "top",    origin_xyz_mm: [0, 0, 2.5], axis_xyz: [0, 0, 1], type: "planar" },
    { id: "bottom", origin_xyz_mm: [0, 0, -2.5], axis_xyz: [0, 0, -1], type: "planar" },
  ],
  compute_gps_gnss: [
    { id: "top",    origin_xyz_mm: [0, 0, 4], axis_xyz: [0, 0, 1], type: "planar" },
    { id: "bottom", origin_xyz_mm: [0, 0, -4], axis_xyz: [0, 0, -1], type: "planar" },
  ],
};
