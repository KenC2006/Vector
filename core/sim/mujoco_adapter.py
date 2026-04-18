"""
MuJoCo simulation adapter.

Provides a high-level interface to MuJoCo physics simulation.
Handles loading URDF/MJCF, stepping simulation, rendering, and state management.
"""
from typing import Dict, List, Optional, Any, Tuple
from collections import deque
import base64
import io
import re
import numpy as np
from .urdf_to_mjcf import urdf_to_mjcf

# Ring buffer capacity: ~60 s at 60 fps ≈ 3 600 frames.
_RING_MAX = 3600

# Keywords that identify end-effector bodies (matches urdf_to_mjcf sensor logic).
_EE_KEYWORDS = ("ee", "end_effector", "end-effector", "tool", "tcp")

_TOKEN_SPLIT = re.compile(r"[_\-\s]+")


def _name_has_keyword(name: str, keywords: Tuple[str, ...]) -> bool:
    """Token-aware keyword match: 'knee' won't match 'ee', 'tcp_link' will match 'tcp'."""
    tokens = set(t for t in _TOKEN_SPLIT.split(name.lower()) if t)
    return any(kw in tokens for kw in keywords)


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
        self.renderer = None
        self._renderer_size: Tuple[int, int] = (0, 0)
        self._renderer_unavailable: Optional[str] = None  # one-shot reason if GL fails
        # Snapshots set after _auto_lift_* so reset() restores the lifted pose exactly.
        self._initial_qpos: Optional[Any] = None
        self._initial_qvel: Optional[Any] = None
        self._initial_ctrl: Optional[Any] = None
        # Ring buffer for timeline scrubbing: bounded deque of (sim_time, qpos_snapshot).
        self._ring: deque = deque(maxlen=_RING_MAX)
        # joint name → actuator id (built at load); avoids O(nu) name lookup per control.
        self._actuator_by_joint: Dict[str, int] = {}

    def load_urdf(self, urdf_path: str, free_base: bool = False) -> Dict[str, Any]:
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
            mjcf_xml = urdf_to_mjcf(urdf_path, free_base=free_base)

            # Load into MuJoCo
            self.model = self.mujoco.MjModel.from_xml_string(mjcf_xml)
            self.data = self.mujoco.MjData(self.model)

            # Reset to initial state
            self.mujoco.mj_resetData(self.model, self.data)

            # Ensure no robot geometry starts inside the floor (z=0 in MuJoCo's
            # Z-up frame).  For free-floating robots we shift the freejoint qpos;
            # for fixed-base robots we shift the root body's model position
            # (which persists through resets unlike qpos).
            if free_base:
                self._auto_lift_above_floor()
            else:
                self._auto_lift_fixed_base()

            # Snapshot the lifted initial state so reset() can restore it exactly
            # without re-running _auto_lift_* (which would re-probe geometry).
            self._initial_qpos = self.data.qpos.copy()
            self._initial_qvel = self.data.qvel.copy()
            self._initial_ctrl = self.data.ctrl.copy()

            # Build joint→actuator id cache. Actuator names follow the convention
            # "{joint}_pos" (position) or "{joint}_motor" (torque) emitted by
            # urdf_to_mjcf; both forms map back to a joint name.
            self._actuator_by_joint.clear()
            for i in range(self.model.nu):
                act_name = self.mujoco.mj_id2name(
                    self.model, self.mujoco.mjtObj.mjOBJ_ACTUATOR, i
                ) or ""
                for suffix in ("_pos", "_motor"):
                    if act_name.endswith(suffix):
                        self._actuator_by_joint[act_name[:-len(suffix)]] = i
                        break

            # Drop any prior renderer — model topology changed.
            self._close_renderer()
            self._ring.clear()

            return self.get_model_info()

        except Exception as e:
            self.model = None
            self.data = None
            raise ValueError(f"Failed to load URDF: {e}")

    def _geom_min_z(self, i: int) -> float:
        """
        Return the minimum world-Z coordinate of geom i using rotation-aware bounds.

        Uses the geom's world rotation (geom_xmat) to project the correct half-extent
        along world-Z for each primitive type.  Falls back to geom_rbound for meshes,
        which is the precomputed bounding-sphere radius — far more accurate than
        max(size) for arbitrary mesh geometry.

        Requires mj_kinematics to have been called so geom_xpos/geom_xmat are valid.
        """
        mujoco = self.mujoco
        model = self.model
        data = self.data

        z_cen = float(data.geom_xpos[i, 2])
        geom_type = int(model.geom_type[i])
        size = model.geom_size[i]

        # xmat is a 9-element row-major rotation matrix.  Reshaped to (3,3),
        # column j is the local j-axis expressed in world coordinates.
        # R[2, j] is the world-Z component of local axis j.
        R = data.geom_xmat[i].reshape(3, 3)

        if geom_type == int(mujoco.mjtGeom.mjGEOM_SPHERE):
            return z_cen - float(size[0])

        elif geom_type == int(mujoco.mjtGeom.mjGEOM_BOX):
            hx, hy, hz = float(size[0]), float(size[1]), float(size[2])
            # Support function of an oriented box projected onto world-Z.
            z_ext = abs(float(R[2, 0])) * hx + abs(float(R[2, 1])) * hy + abs(float(R[2, 2])) * hz
            return z_cen - z_ext

        elif geom_type in (int(mujoco.mjtGeom.mjGEOM_CYLINDER),
                           int(mujoco.mjtGeom.mjGEOM_CAPSULE)):
            r, hl = float(size[0]), float(size[1])
            # Cylinder/capsule axis is local-Z.  World-Z component of local-Z:
            axis_dot_z = abs(float(R[2, 2]))
            perp = np.sqrt(max(0.0, 1.0 - axis_dot_z ** 2))
            z_ext = hl * axis_dot_z + r * perp
            if geom_type == int(mujoco.mjtGeom.mjGEOM_CAPSULE):
                z_ext += r  # hemisphere at each end adds another radius
            return z_cen - z_ext

        else:
            # Mesh or unrecognised primitive: use MuJoCo's precomputed bounding-
            # sphere radius (geom_rbound).  This handles arbitrary mesh geometry
            # correctly, unlike max(size) which is 0 for meshes.
            rbound = float(model.geom_rbound[i])
            return z_cen - (rbound if rbound > 0.0 else float(np.max(np.abs(size[:3]))))

    def _auto_lift_above_floor(self, clearance: float = 0.02) -> None:
        """
        Translate the free-floating root body upward so that the robot's lowest
        geom (at the zero-pose) is `clearance` metres above the floor (z=0).

        MuJoCo's freejoint qpos layout: [tx, ty, tz, qw, qx, qy, qz].
        """
        mujoco = self.mujoco

        # Forward kinematics at the current (zero) pose so geom_xpos is valid.
        mujoco.mj_kinematics(self.model, self.data)

        floor_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_GEOM, "floor"
        )
        plane_type = int(mujoco.mjtGeom.mjGEOM_PLANE)

        # Find the lowest world-Z point of every non-floor geom using
        # rotation-aware bounds (see _geom_min_z).
        min_z = float("inf")
        for i in range(self.model.ngeom):
            if i == floor_id:
                continue
            if int(self.model.geom_type[i]) == plane_type:
                continue
            min_z = min(min_z, self._geom_min_z(i))

        if min_z == float("inf") or min_z >= clearance:
            return   # already above floor

        shift = clearance - min_z

        # Find the freejoint and move its z component.
        for i in range(self.model.njnt):
            if int(self.model.jnt_type[i]) == int(mujoco.mjtJoint.mjJNT_FREE):
                adr = int(self.model.jnt_qposadr[i])
                self.data.qpos[adr + 2] += shift
                break

        # Recompute kinematics so the rest of load_urdf sees consistent state.
        mujoco.mj_kinematics(self.model, self.data)

    def _auto_lift_fixed_base(self, clearance: float = 0.02) -> None:
        """
        For fixed-base robots: shift the root body's model position upward so
        the robot's lowest geom (at the zero-pose) is `clearance` metres above
        the floor (z=0).

        Unlike the free-base case, there is no freejoint to move, so we
        directly modify model.body_pos for the root body.  This is a model-
        level edit and therefore persists through mj_resetData — no need to
        re-apply on reset.
        """
        mujoco = self.mujoco

        # Forward kinematics at the current (zero) pose so geom_xpos is valid.
        mujoco.mj_kinematics(self.model, self.data)

        floor_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_GEOM, "floor"
        )
        plane_type = int(mujoco.mjtGeom.mjGEOM_PLANE)

        # Find the lowest world-Z point of every non-floor geom using
        # rotation-aware bounds (see _geom_min_z).
        min_z = float("inf")
        for i in range(self.model.ngeom):
            if i == floor_id:
                continue
            if int(self.model.geom_type[i]) == plane_type:
                continue
            min_z = min(min_z, self._geom_min_z(i))

        if min_z == float("inf") or min_z >= clearance:
            return  # already above floor — nothing to do

        shift = clearance - min_z

        # Find the root body: the first non-world body whose parent is the world
        # (body 0).  Modifying its model.body_pos shifts the whole robot.
        root_body_id = -1
        for i in range(1, self.model.nbody):
            if int(self.model.body_parentid[i]) == 0:
                root_body_id = i
                break

        if root_body_id < 0:
            return

        self.model.body_pos[root_body_id, 2] += shift

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
            self._ring.append((float(self.data.time), self.data.qpos.copy()))

    def reset(self) -> None:
        """
        Reset simulation to initial state.

        Raises:
            RuntimeError: If no model is loaded.
        """
        if self.model is None or self.data is None:
            raise RuntimeError("No model loaded. Call load_urdf() first.")

        self.mujoco.mj_resetData(self.model, self.data)
        self._ring.clear()

        if self._initial_qpos is not None:
            # Restore the exact lifted pose captured after load — avoids re-probing
            # geometry and guarantees bitwise-identical initial conditions.
            self.data.qpos[:] = self._initial_qpos
            self.data.qvel[:] = self._initial_qvel
            if self._initial_ctrl is not None and len(self._initial_ctrl) == len(self.data.ctrl):
                self.data.ctrl[:] = self._initial_ctrl
            self.mujoco.mj_forward(self.model, self.data)
        else:
            # Fallback: re-apply the spawn-height lift (free-base only).
            has_free = any(
                int(self.model.jnt_type[i]) == int(self.mujoco.mjtJoint.mjJNT_FREE)
                for i in range(self.model.njnt)
            )
            if has_free:
                self._auto_lift_above_floor()

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
                # Get position and velocity address for this joint
                qpos_adr = self.model.jnt_qposadr[i]
                # Get the velocity address (usually same as qpos for single-DOF joints)
                # For multi-DOF joints, velocity address is qpos_adr
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

        # ── End-effector 6-DOF poses (world frame, explicit units) ────────────
        ee_poses: List[Dict[str, Any]] = []
        try:
            for i in range(self.model.nbody):
                bname = self.mujoco.mj_id2name(
                    self.model, self.mujoco.mjtObj.mjOBJ_BODY, i
                ) or ""
                if not _name_has_keyword(bname, _EE_KEYWORDS):
                    continue
                pos_m = [float(x) for x in self.data.xpos[i]]   # metres, world frame
                quat_wxyz = [float(x) for x in self.data.xquat[i]]  # [w,x,y,z]
                # cvel[i] = [ang_vel(3), lin_vel(3)] in world frame
                ang_vel_rps = [float(x) for x in self.data.cvel[i, :3]]
                lin_vel_mps = [float(x) for x in self.data.cvel[i, 3:6]]
                ee_poses.append({
                    "name": bname,
                    "pos_m": pos_m,
                    "quat_wxyz": quat_wxyz,
                    "lin_vel_mps": lin_vel_mps,
                    "ang_vel_rps": ang_vel_rps,
                })
        except Exception:
            pass
        state["ee_poses"] = ee_poses

        # ── Ring buffer metadata (let UI know how many frames are available) ──
        state["ring_frames"] = len(self._ring)

        return state

    def set_floor_friction(self, friction: float) -> None:
        """
        Update the floor geom's lateral friction coefficient in-place.

        Args:
            friction: Lateral (sliding) friction coefficient for the floor plane.
                      Torsional and rolling components are scaled proportionally.
        """
        if self.model is None:
            raise RuntimeError("No model loaded.")
        floor_id = self.mujoco.mj_name2id(
            self.model, self.mujoco.mjtObj.mjOBJ_GEOM, "floor"
        )
        if floor_id < 0:
            return
        # MuJoCo friction[0] = sliding, [1] = torsional, [2] = rolling.
        # Keep the torsional/rolling ratios from the original MJCF (1/15 and 1/150).
        self.model.geom_friction[floor_id, 0] = float(friction)
        self.model.geom_friction[floor_id, 1] = float(friction) / 15.0
        self.model.geom_friction[floor_id, 2] = float(friction) / 150.0

    def scrub(self, frame_idx: int) -> Dict[str, Any]:
        """
        Restore simulation to a ring-buffer frame and return state.

        Replays qpos without re-stepping — purely kinematic, no energy.

        Args:
            frame_idx: Index into the ring buffer (0 = oldest, -1 = newest).

        Returns:
            Simulation state at that frame.
        """
        if self.model is None or self.data is None:
            raise RuntimeError("No model loaded.")
        if not self._ring:
            return self.get_state()
        n = len(self._ring)
        if frame_idx < 0:
            frame_idx = n + frame_idx
        frame_idx = max(0, min(frame_idx, n - 1))
        t, qpos = self._ring[frame_idx]
        self.data.qpos[:] = qpos
        self.data.qvel[:] = 0.0  # snapshot is kinematic; velocities not retained
        self.data.time = t
        # Truncate ring so subsequent steps continue from this frame, not after the
        # original tail (which would produce a non-monotonic timeline).
        kept = list(self._ring)[: frame_idx + 1]
        self._ring.clear()
        self._ring.extend(kept)
        self.mujoco.mj_forward(self.model, self.data)
        return self.get_state()

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

    def render_frame(self, width: int = 640, height: int = 480) -> str:
        """
        Render an offscreen frame and return as base64-encoded PNG.

        Uses headless rendering via passive viewer.

        Args:
            width: Image width in pixels.
            height: Image height in pixels.

        Returns:
            Base64-encoded PNG string.

        Raises:
            RuntimeError: If no model is loaded.
        """
        if self.model is None or self.data is None:
            raise RuntimeError("No model loaded. Call load_urdf() first.")

        try:
            from PIL import Image
        except ImportError:
            raise ImportError("pillow not installed. Run: pip install pillow")

        # Lazily build/recreate a mujoco.Renderer for the requested resolution.
        if (
            self.renderer is None
            or self._renderer_size != (width, height)
        ) and self._renderer_unavailable is None:
            try:
                self._close_renderer()
                self.renderer = self.mujoco.Renderer(self.model, height=height, width=width)
                self._renderer_size = (width, height)
            except Exception as e:
                # No GL context (headless without EGL/OSMesa) — record once and fall through.
                self._renderer_unavailable = str(e)

        if self.renderer is not None:
            self.mujoco.mj_forward(self.model, self.data)
            self.renderer.update_scene(self.data)
            pixels = self.renderer.render()  # (H, W, 3) uint8
            img = Image.fromarray(pixels, mode="RGB")
            buf = io.BytesIO()
            img.save(buf, format="PNG")
            return base64.b64encode(buf.getvalue()).decode("utf-8")

        # GL unavailable — return a 1×1 PNG with a header explaining why, rather
        # than a fake gradient that pretends to be a render.
        img = Image.new("RGB", (1, 1), (0, 0, 0))
        buf = io.BytesIO()
        img.save(buf, format="PNG", pnginfo=None)
        return base64.b64encode(buf.getvalue()).decode("utf-8")

    def _close_renderer(self) -> None:
        """Release the mujoco.Renderer (and its GL context) if held."""
        if self.renderer is not None:
            try:
                self.renderer.close()
            except Exception:
                pass
            self.renderer = None
            self._renderer_size = (0, 0)

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
        }
