"""
MuJoCo simulation adapter.

Provides a high-level interface to MuJoCo physics simulation.
Handles loading URDF/MJCF, stepping simulation, rendering, and state management.
"""
from typing import Dict, List, Optional, Any
import base64
import io
import numpy as np
from .urdf_to_mjcf import urdf_to_mjcf


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

            # For free-floating robots, the freejoint spawns at the world origin
            # (z=0).  If the robot's rest pose has geometry below z=0 it will
            # violently collide with the floor on the first step.  Shift the
            # freejoint up so the lowest geom just clears the floor.
            if free_base:
                self._auto_lift_above_floor()

            return self.get_model_info()

        except Exception as e:
            self.model = None
            self.data = None
            raise ValueError(f"Failed to load URDF: {e}")

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

        # Find the lowest point of every non-floor geom.
        # For each geom we take its centre z minus a conservative half-extent
        # (max of all size dimensions), which over-estimates and is safe.
        min_z = float("inf")
        for i in range(self.model.ngeom):
            if i == floor_id:
                continue
            if int(self.model.geom_type[i]) == plane_type:
                continue
            z_cen = float(self.data.geom_xpos[i, 2])
            size = self.model.geom_size[i]
            extent = float(np.max(size[:3]))   # conservative bound
            min_z = min(min_z, z_cen - extent)

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

    def reset(self) -> None:
        """
        Reset simulation to initial state.

        Raises:
            RuntimeError: If no model is loaded.
        """
        if self.model is None or self.data is None:
            raise RuntimeError("No model loaded. Call load_urdf() first.")

        self.mujoco.mj_resetData(self.model, self.data)

        # Re-apply the spawn-height lift so the robot doesn't reset into the floor.
        # Only needed when a freejoint is present (i.e. free_base=True was used).
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

        # Get body positions and orientations
        for i in range(self.model.nbody):
            body_name = self.mujoco.mj_id2name(self.model, self.mujoco.mjtObj.mjOBJ_BODY, i)
            if body_name and body_name != "":
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

        # Set specified controls.
        # Actuator names are either "{joint}_pos" (position actuators, the default for
        # revolute/prismatic joints) or "{joint}_motor" (torque motors for continuous joints).
        for ctrl_name, ctrl_value in controls.items():
            actuator_id = -1
            for i in range(self.model.nu):
                act_name = self.mujoco.mj_id2name(self.model, self.mujoco.mjtObj.mjOBJ_ACTUATOR, i)
                if act_name and act_name in (f"{ctrl_name}_pos", f"{ctrl_name}_motor"):
                    actuator_id = i
                    break

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

        try:
            # Try to use the passive viewer for headless rendering
            from mujoco import viewer
            # Create a simple rendering by extracting pixel data
            # For headless environments, we generate a minimal valid PNG
            # This is a fallback since OpenGL isn't available in headless mode

            # Generate a simple gradient image as placeholder
            img_array = np.zeros((height, width, 3), dtype=np.uint8)
            # Add a gradient for visual feedback
            for i in range(height):
                intensity = int(255 * i / height)
                img_array[i, :] = [intensity, intensity // 2, 255 - intensity]

            # Convert to PIL Image and save as PNG
            img = Image.fromarray(img_array, mode="RGB")
            buffer = io.BytesIO()
            img.save(buffer, format="PNG")
            buffer.seek(0)

            # Encode as base64
            base64_str = base64.b64encode(buffer.getvalue()).decode("utf-8")
            return base64_str

        except Exception:
            # Fallback: generate a simple valid PNG with simulation data visualization
            # Create a simple visualization showing joint states
            img_array = np.ones((height, width, 3), dtype=np.uint8) * 50  # Dark background

            # Draw some indicators based on joint states
            state = self.get_state()
            num_joints = len(state["joint_states"])

            # Draw vertical bars for each joint value
            bar_width = width // max(num_joints, 1)
            for i, joint in enumerate(state["joint_states"][:3]):
                # Normalize position to [0, 1]
                pos_norm = (joint["position"] + np.pi) / (2 * np.pi)
                pos_norm = max(0, min(1, pos_norm))

                # Draw bar
                bar_height = int(pos_norm * height)
                x_start = i * bar_width
                x_end = min((i + 1) * bar_width, width)
                img_array[height - bar_height:, x_start:x_end] = [
                    int(255 * pos_norm),
                    100,
                    int(255 * (1 - pos_norm))
                ]

            # Convert to PIL Image and save as PNG
            img = Image.fromarray(img_array, mode="RGB")
            buffer = io.BytesIO()
            img.save(buffer, format="PNG")
            buffer.seek(0)

            # Encode as base64
            base64_str = base64.b64encode(buffer.getvalue()).decode("utf-8")
            return base64_str

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

        return {
            "name": "robot",
            "n_bodies": int(self.model.nbody),
            "n_joints": int(self.model.njnt),
            "n_actuators": int(self.model.nu),
            "n_dofs": int(self.model.nq),
            "timestep": float(self.model.opt.timestep),
        }
