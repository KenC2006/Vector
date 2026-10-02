"""
MuJoCo simulation adapter.

Provides a high-level interface to MuJoCo physics simulation.
Handles loading URDF/MJCF, stepping simulation, rendering, and state management.
"""
from typing import Dict, List, Optional, Any
import numpy as np
from core.sim.urdf_to_mjcf import urdf_to_mjcf, normalize_terrain_config

# Spawn gap between the robot's lowest collision point and the floor (m).
SPAWN_CLEARANCE = 0.001


class MuJoCoSimulator:
    """
    MuJoCo physics simulator wrapper.

    Manages robot model loading, simulation stepping, and state queries.
    """

    def __init__(self):
        """Initialize the simulator with no model loaded."""
        try:
            import mujoco
        except ImportError:
            raise ImportError("mujoco not installed. Run: pip install mujoco>=3.5.0")

        self.mujoco = mujoco
        self.model = None
        self.data = None
        # Snapshots set after _spawn_on_floor so reset() restores the spawn pose exactly.
        self._initial_qpos: Optional[Any] = None
        self._initial_qvel: Optional[Any] = None
        self._initial_ctrl: Optional[Any] = None
        # joint name → actuator id (built at load); avoids O(nu) name lookup per control.
        self._actuator_by_joint: Dict[str, int] = {}
        self._terrain_config_active: Dict[str, Any] = normalize_terrain_config(None)

    def load_urdf(
        self,
        urdf_path: str,
        free_base: bool = False,
        terrain_config: Optional[Dict[str, Any]] = None,
    ) -> Dict[str, Any]:
        """
        Load a URDF file into MuJoCo.

        Converts URDF to MJCF, adds actuators, and loads into MuJoCo.

        Args:
            urdf_path: Path to the URDF file.

        Returns:
            Model info dict with metadata.

        Raises:
            FileNotFoundError: If URDF file doesn't exist.
            ValueError: If URDF is invalid or MuJoCo load fails.
        """
        try:
            # Convert URDF to MJCF
            mjcf_xml = urdf_to_mjcf(
                urdf_path,
                free_base=free_base,
                terrain_config=terrain_config,
            )
            self._terrain_config_active = normalize_terrain_config(terrain_config)

            # Load into MuJoCo
            self.model = self.mujoco.MjModel.from_xml_string(mjcf_xml)
            self.data = self.mujoco.MjData(self.model)

            # Reset to initial state
            self.mujoco.mj_resetData(self.model, self.data)

            # Spawn on the floor (z=0 in MuJoCo's Z-up frame): free-floating
            # robots are placed with their lowest collision point
            # SPAWN_CLEARANCE above it (no drop, no penetration); fixed-base
            # robots stay where they were authored unless they would start
            # inside the floor.
            self._spawn_on_floor(free_base)

            # Snapshot the spawned initial state so reset() can restore it exactly
            # without re-running _spawn_on_floor (which would re-probe geometry).
            self._initial_qpos = self.data.qpos.copy()
            self._initial_qvel = self.data.qvel.copy()
            self._initial_ctrl = self.data.ctrl.copy()

            # Build joint→actuator id cache. Actuator names follow the convention
            # "{joint}_pos" (position), "{joint}_vel" (velocity, continuous
            # joints) or "{joint}_motor" (torque, legacy) emitted by urdf_to_mjcf.
            self._actuator_by_joint.clear()
            for i in range(self.model.nu):
                act_name = self.mujoco.mj_id2name(
                    self.model, self.mujoco.mjtObj.mjOBJ_ACTUATOR, i
                ) or ""
                for suffix in ("_pos", "_vel", "_motor"):
                    if act_name.endswith(suffix):
                        self._actuator_by_joint[act_name[:-len(suffix)]] = i
                        break

            return self.get_model_info()

        except Exception as e:
            self.model = None
            self.data = None
            self._terrain_config_active = normalize_terrain_config(None)
            raise ValueError(f"Failed to load URDF: {e}")

    def _geom_min_z(self, i: int) -> float:
        """
        Minimum world-Z of geom i at the current kinematics (exact).

        Primitives use the support function of the oriented shape along world
        -Z; meshes use their actual vertices (the bounding-sphere radius put
        mesh robots centimetres above the floor, so they spawned high and
        dropped). Requires mj_kinematics so geom_xpos/geom_xmat are valid.
        """
        mujoco = self.mujoco
        model = self.model
        data = self.data

        z_cen = float(data.geom_xpos[i, 2])
        geom_type = int(model.geom_type[i])
        size = model.geom_size[i]
        # xmat is row-major; R[2, j] is the world-Z component of local axis j.
        R = data.geom_xmat[i].reshape(3, 3)
        rz = np.abs(R[2, :])

        if geom_type == int(mujoco.mjtGeom.mjGEOM_SPHERE):
            return z_cen - float(size[0])
        if geom_type == int(mujoco.mjtGeom.mjGEOM_BOX):
            return z_cen - float(rz @ size[:3])
        if geom_type == int(mujoco.mjtGeom.mjGEOM_ELLIPSOID):
            return z_cen - float(np.sqrt(np.sum((R[2, :] * size[:3]) ** 2)))
        if geom_type in (int(mujoco.mjtGeom.mjGEOM_CYLINDER), int(mujoco.mjtGeom.mjGEOM_CAPSULE)):
            r, hl = float(size[0]), float(size[1])
            # Axis is local Z: the end disc/cap contributes hl·|cos| and the rim r·|sin|.
            axis_dot_z = float(rz[2])
            perp = float(np.sqrt(max(0.0, 1.0 - axis_dot_z ** 2)))
            if geom_type == int(mujoco.mjtGeom.mjGEOM_CAPSULE):
                return z_cen - (hl * axis_dot_z + r)
            return z_cen - (hl * axis_dot_z + r * perp)
        if geom_type == int(mujoco.mjtGeom.mjGEOM_MESH):
            mesh_id = int(model.geom_dataid[i])
            if mesh_id >= 0:
                adr, num = int(model.mesh_vertadr[mesh_id]), int(model.mesh_vertnum[mesh_id])
                if num > 0:
                    verts = model.mesh_vert[adr:adr + num]   # geom frame
                    return z_cen + float(np.min(verts @ R[2, :]))
        # Height fields / anything else: bounding sphere.
        rbound = float(model.geom_rbound[i])
        return z_cen - (rbound if rbound > 0.0 else float(np.max(np.abs(size[:3]))))

    def _robot_min_z(self) -> float:
        """Lowest world-Z over the robot's collision geoms (inf if none).

        Visual-only geoms (contype = conaffinity = 0, e.g. a track envelope
        whose rollers carry the contact) cannot touch the floor, so they do
        not decide the spawn height — unless nothing collides at all.
        """
        model = self.model
        robot = [i for i in range(model.ngeom)
                 if int(model.geom_bodyid[i]) != 0
                 and int(model.geom_type[i]) != int(self.mujoco.mjtGeom.mjGEOM_PLANE)]
        solid = [i for i in robot if int(model.geom_contype[i]) or int(model.geom_conaffinity[i])]
        return min((self._geom_min_z(i) for i in (solid or robot)), default=float("inf"))

    def _spawn_on_floor(self, free_base: bool, clearance: float = SPAWN_CLEARANCE) -> None:
        """
        Place the robot so its lowest collision point is `clearance` above the
        floor (z=0) in the zero pose.

        Free base: move the freejoint's z (qpos [tx, ty, tz, qw, qx, qy, qz])
        up OR down, so the robot neither drops nor starts in the floor.
        Fixed base: lift the root body's model position only if the robot
        would start inside the floor; model.body_pos persists through
        mj_resetData, so resets need no re-probe.
        """
        mujoco = self.mujoco
        mujoco.mj_kinematics(self.model, self.data)
        min_z = self._robot_min_z()
        if min_z == float("inf"):
            return
        shift = clearance - min_z
        if free_base:
            if abs(shift) < 1e-9:
                return
            for i in range(self.model.njnt):
                if int(self.model.jnt_type[i]) == int(mujoco.mjtJoint.mjJNT_FREE):
                    self.data.qpos[int(self.model.jnt_qposadr[i]) + 2] += shift
                    break
        else:
            if shift <= 0:
                return
            root = next((b for b in range(1, self.model.nbody)
                         if int(self.model.body_parentid[b]) == 0), -1)
            if root < 0:
                return
            self.model.body_pos[root, 2] += shift
        # Recompute kinematics so the rest of load_urdf sees consistent state.
        mujoco.mj_kinematics(self.model, self.data)

    def step(self, n_steps: int = 1) -> None:
        """
        Step the simulation forward.

        Args:
            n_steps: Number of simulation steps to advance.

        Raises:
            RuntimeError: If no model is loaded.
        """
        if self.model is None or self.data is None:
            raise RuntimeError("No model loaded. Call load_urdf() first.")

        for _ in range(n_steps):
            self.mujoco.mj_step(self.model, self.data)

        # Surface divergence as a clean error instead of letting NaN/inf leak into
        # get_state() (where it becomes invalid JSON). Usually a degenerate inertia/
        # mass or too large a timestep for the model.
        if not np.isfinite(self.data.qpos).all():
            raise RuntimeError(
                "Simulation diverged (non-finite state) — check link inertias/masses "
                "or reduce the model timestep."
            )

    def reset(self) -> None:
        """
        Reset simulation to initial state.

        Raises:
            RuntimeError: If no model is loaded.
        """
        if self.model is None or self.data is None:
            raise RuntimeError("No model loaded. Call load_urdf() first.")

        self.mujoco.mj_resetData(self.model, self.data)

        if self._initial_qpos is not None:
            # Restore the exact lifted pose captured after load — avoids re-probing
            # geometry and guarantees bitwise-identical initial conditions.
            self.data.qpos[:] = self._initial_qpos
            self.data.qvel[:] = self._initial_qvel
            if self._initial_ctrl is not None and len(self._initial_ctrl) == len(self.data.ctrl):
                self.data.ctrl[:] = self._initial_ctrl
            self.mujoco.mj_forward(self.model, self.data)
        else:
            # Fallback: re-apply the spawn placement (free-base only).
            has_free = any(
                int(self.model.jnt_type[i]) == int(self.mujoco.mjtJoint.mjJNT_FREE)
                for i in range(self.model.njnt)
            )
            if has_free:
                self._spawn_on_floor(True)

    def get_state(self) -> Dict[str, Any]:
        """
        Get current simulation state.

        Returns:
            State dict with time, joint states, body positions, and COM.
        """
        if self.model is None or self.data is None:
            raise RuntimeError("No model loaded. Call load_urdf() first.")

        # Ensure body positions/orientations reflect the latest qpos before reading.
        # mj_step already calls forward internally, but set_control alone does not,
        # so this guarantees the UI never shows a one-step-stale pose.
        self.mujoco.mj_forward(self.model, self.data)

        state = {
            "time": float(self.data.time),
            "joint_states": [],
            "body_positions": [],
            "com_position": [float(x) for x in self.data.subtree_com[0]],
        }

        # Get joint states
        for i in range(self.model.njnt):
            joint_name = self.mujoco.mj_id2name(self.model, self.mujoco.mjtObj.mjOBJ_JOINT, i)
            if joint_name and joint_name != "":
                # Position address (into qpos) and DOF address (into qvel) for this
                # joint. They differ for multi-DOF joints; the single-DOF revolute/
                # prismatic joints we report below are scalar at both.
                qpos_adr = self.model.jnt_qposadr[i]
                dof_adr = self.model.jnt_dofadr[i]

                # Extract position (single value for revolute/prismatic)
                if qpos_adr >= 0 and qpos_adr < len(self.data.qpos):
                    position = float(self.data.qpos[qpos_adr])
                    velocity = float(self.data.qvel[dof_adr]) if dof_adr >= 0 and dof_adr < len(self.data.qvel) else 0.0

                    state["joint_states"].append({
                        "name": joint_name,
                        "position": position,
                        "velocity": velocity,
                    })

        # Get body positions and orientations.
        # Skip body 0 (the world body, always at origin) so body_positions[0] is
        # the actual robot root — the frontend relies on this for free-base pose.
        for i in range(1, self.model.nbody):
            body_name = self.mujoco.mj_id2name(self.model, self.mujoco.mjtObj.mjOBJ_BODY, i)
            if body_name and body_name != "" and body_name != "world":
                position = [float(x) for x in self.data.xpos[i]]
                rotation = [float(x) for x in self.data.xquat[i]]  # [w, x, y, z]

                state["body_positions"].append({
                    "name": body_name,
                    "position": position,
                    "rotation": rotation,
                })

        # ── Phase D: contact forces ───────────────────────────────────────────
        contacts_list: List[Dict[str, Any]] = []
        try:
            ncon = int(self.data.ncon)
            state["n_contacts"] = ncon
            for i in range(min(ncon, 20)):
                c = self.data.contact[i]
                pos = [float(c.pos[0]), float(c.pos[1]), float(c.pos[2])]
                # Contact frame: first 3 elements are the contact normal (row 0)
                normal = [float(c.frame[0]), float(c.frame[1]), float(c.frame[2])]
                # Normal force magnitude from efc_force at efc_address
                force = 0.0
                efc_adr = int(c.efc_address)
                if 0 <= efc_adr < len(self.data.efc_force):
                    force = abs(float(self.data.efc_force[efc_adr]))
                b1_id = int(self.model.geom_bodyid[int(c.geom1)])
                b2_id = int(self.model.geom_bodyid[int(c.geom2)])
                b1n = self.mujoco.mj_id2name(self.model, self.mujoco.mjtObj.mjOBJ_BODY, b1_id) or ""
                b2n = self.mujoco.mj_id2name(self.model, self.mujoco.mjtObj.mjOBJ_BODY, b2_id) or ""
                contacts_list.append({
                    "pos": pos,
                    "normal": normal,
                    "force": force,
                    "body1": b1n,
                    "body2": b2n,
                })
        except Exception:
            state["n_contacts"] = 0
        state["contacts_list"] = contacts_list

        # ── Phase D: actuator forces for torque heatmap ───────────────────────
        actuator_forces: Dict[str, float] = {}
        try:
            for i in range(self.model.nu):
                act_name = self.mujoco.mj_id2name(self.model, self.mujoco.mjtObj.mjOBJ_ACTUATOR, i)
                if act_name:
                    actuator_forces[act_name] = float(self.data.actuator_force[i])
        except Exception:
            pass
        state["actuator_forces"] = actuator_forces

        # ── Energy (KE + PE) — available because we emit <flag energy="enable"/> ──
        try:
            ke = float(self.data.energy[0])
            pe = float(self.data.energy[1])
            state["energy_j"] = ke + pe
            state["kinetic_j"] = ke
            state["potential_j"] = pe
        except Exception:
            pass

        return state

    def set_gravity(self, gravity: List[float]) -> None:
        """
        Set gravity vector.

        Args:
            gravity: [gx, gy, gz] in m/s². URDF/MuJoCo is Z-up, so default is [0, 0, -9.81].
        """
        if self.model is None:
            raise RuntimeError("No model loaded.")
        if len(gravity) == 3:
            self.model.opt.gravity[:] = gravity

    def set_control(self, controls: Dict[str, float]) -> None:
        """
        Set actuator control values.

        Args:
            controls: Dict mapping joint names to control values.
                     For motors, this is typically a torque or desired position.

        Raises:
            RuntimeError: If no model is loaded.
        """
        if self.model is None or self.data is None:
            raise RuntimeError("No model loaded. Call load_urdf() first.")

        # Clear all controls first
        self.data.ctrl[:] = 0.0

        # Look up actuator id from precomputed joint→id cache (built in load_urdf).
        for ctrl_name, ctrl_value in controls.items():
            actuator_id = self._actuator_by_joint.get(ctrl_name, -1)
            if actuator_id >= 0:
                self.data.ctrl[actuator_id] = float(ctrl_value)

    def get_model_info(self) -> Dict[str, Any]:
        """
        Get model metadata.

        Returns:
            Dict with model info (n_bodies, n_joints, n_actuators, timestep).

        Raises:
            RuntimeError: If no model is loaded.
        """
        if self.model is None or self.data is None:
            raise RuntimeError("No model loaded. Call load_urdf() first.")

        # Total robot mass (excludes the world body at index 0).
        total_mass = float(np.sum(self.model.body_mass[1:]))
        # Whole-system COM in world frame (valid after mj_forward in load_urdf).
        com = [float(x) for x in self.data.subtree_com[0]]

        mass_warning: Optional[str] = None
        if total_mass < 0.01:
            mass_warning = (
                f"Total mass {total_mass * 1000:.1f} g is very low — "
                "URDF inertial tags may be missing or incorrect."
            )

        terrain_hfield: Optional[Dict[str, Any]] = None
        try:
            if self._terrain_config_active.get("type") == "rough" and int(self.model.nhfield) > 0:
                hfield_id = 0
                nrow = int(self.model.hfield_nrow[hfield_id])
                ncol = int(self.model.hfield_ncol[hfield_id])
                adr = int(self.model.hfield_adr[hfield_id])
                count = nrow * ncol
                elev = [float(x) for x in self.model.hfield_data[adr:adr + count]]
                size = [float(x) for x in self.model.hfield_size[hfield_id]]
                terrain_hfield = {
                    "nrow": nrow,
                    "ncol": ncol,
                    "elevation": elev,
                    "size": size,
                }
        except Exception:
            terrain_hfield = None

        return {
            "name": "robot",
            "n_bodies": int(self.model.nbody),
            "n_joints": int(self.model.njnt),
            "n_actuators": int(self.model.nu),
            "n_dofs": int(self.model.nq),
            "timestep": float(self.model.opt.timestep),
            "total_mass_kg": total_mass,
            "com_m": com,
            "mass_warning": mass_warning,
            "terrain_config": dict(self._terrain_config_active),
            "terrain_hfield": terrain_hfield,
        }
