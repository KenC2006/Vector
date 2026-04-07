#!/usr/bin/env python3
"""
JSON-RPC 2.0 server over stdio.

Reads line-delimited JSON from stdin, writes responses to stdout.
Minimal implementation without external JSON-RPC libraries.
"""
import json
import sys
from typing import Any, Dict, Optional
import traceback
import os

# Add current directory to path for imports
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from model.urdf_parser import parse_urdf
from model.kinematic_graph import KinematicGraph

# Lazy import MuJoCo — it may not be installed
_MuJoCoSimulator = None
_mujoco_import_error = None

try:
    from sim.mujoco_adapter import MuJoCoSimulator as _MuJoCoSimulator
except ImportError as e:
    _mujoco_import_error = str(e)
    print(f"Warning: MuJoCo not available: {e}", file=sys.stderr)


class JSONRPCServer:
    """Simple JSON-RPC 2.0 server."""

    def __init__(self):
        """Initialize the server."""
        self.simulator = _MuJoCoSimulator() if _MuJoCoSimulator else None
        self.methods = {
            "parse_urdf": self.handle_parse_urdf,
            "ping": self.handle_ping,
            "sim_load": self.handle_sim_load,
            "sim_step": self.handle_sim_step,
            "sim_reset": self.handle_sim_reset,
            "sim_set_control": self.handle_sim_set_control,
            "sim_get_state": self.handle_sim_get_state,
            "sim_render": self.handle_sim_render,
        }

    def handle_parse_urdf(self, params: Dict[str, Any]) -> Dict[str, Any]:
        """
        Parse a URDF file and return the kinematic graph as JSON.

        Params:
            path (str): Path to the URDF file.

        Returns:
            Kinematic graph JSON.
        """
        if "path" not in params:
            raise ValueError("Missing required parameter: path")

        path = params["path"]
        if not isinstance(path, str):
            raise ValueError("Parameter 'path' must be a string")

        try:
            kg = parse_urdf(path)
            return kg.to_json()
        except FileNotFoundError as e:
            raise ValueError(f"File not found: {e}")
        except Exception as e:
            raise ValueError(f"Failed to parse URDF: {e}")

    def handle_ping(self, params: Dict[str, Any]) -> str:
        """
        Health check method.

        Returns:
            "pong"
        """
        return "pong"

    def _require_simulator(self):
        """Check that the simulator is available."""
        if self.simulator is None:
            raise ValueError(
                f"MuJoCo not installed. Run: pip install mujoco pillow\n"
                f"Import error: {_mujoco_import_error}"
            )

    def handle_sim_load(self, params: Dict[str, Any]) -> Dict[str, Any]:
        """
        Load a URDF file into the simulator.

        Params:
            path (str): Path to the URDF file.

        Returns:
            Model info dict.
        """
        self._require_simulator()
        if "path" not in params:
            raise ValueError("Missing required parameter: path")

        path = params["path"]
        if not isinstance(path, str):
            raise ValueError("Parameter 'path' must be a string")

        try:
            return self.simulator.load_urdf(path)
        except FileNotFoundError as e:
            raise ValueError(f"File not found: {e}")
        except Exception as e:
            raise ValueError(f"Failed to load URDF: {e}")

    def handle_sim_step(self, params: Dict[str, Any]) -> Dict[str, Any]:
        """
        Step the simulation forward.

        Params:
            n_steps (int, optional): Number of steps to advance. Default: 1.

        Returns:
            Current simulation state.
        """
        self._require_simulator()
        n_steps = params.get("n_steps", 1)
        if not isinstance(n_steps, int) or n_steps < 1:
            n_steps = 1

        try:
            self.simulator.step(n_steps)
            return self.simulator.get_state()
        except Exception as e:
            raise ValueError(f"Failed to step simulation: {e}")

    def handle_sim_reset(self, params: Dict[str, Any]) -> Dict[str, Any]:
        """
        Reset the simulation to initial state.

        Params:
            (none)

        Returns:
            Current simulation state after reset.
        """
        self._require_simulator()
        try:
            self.simulator.reset()
            return self.simulator.get_state()
        except Exception as e:
            raise ValueError(f"Failed to reset simulation: {e}")

    def handle_sim_set_control(self, params: Dict[str, Any]) -> None:
        """
        Set actuator control values.

        Params:
            controls (dict): Mapping of joint name to control value.

        Returns:
            null
        """
        self._require_simulator()
        if "controls" not in params:
            raise ValueError("Missing required parameter: controls")

        controls = params["controls"]
        if not isinstance(controls, dict):
            raise ValueError("Parameter 'controls' must be a dict")

        try:
            self.simulator.set_control(controls)
        except Exception as e:
            raise ValueError(f"Failed to set controls: {e}")

    def handle_sim_get_state(self, params: Dict[str, Any]) -> Dict[str, Any]:
        """
        Get current simulation state.

        Params:
            (none)

        Returns:
            Simulation state dict.
        """
        self._require_simulator()
        try:
            return self.simulator.get_state()
        except Exception as e:
            raise ValueError(f"Failed to get state: {e}")

    def handle_sim_render(self, params: Dict[str, Any]) -> str:
        """
        Render a frame and return as base64-encoded PNG.

        Params:
            width (int, optional): Image width. Default: 640.
            height (int, optional): Image height. Default: 480.

        Returns:
            Base64-encoded PNG string.
        """
        self._require_simulator()
        width = params.get("width", 640)
        height = params.get("height", 480)

        if not isinstance(width, int) or width < 1:
            width = 640
        if not isinstance(height, int) or height < 1:
            height = 480

        try:
            return self.simulator.render_frame(width, height)
        except Exception as e:
            raise ValueError(f"Failed to render frame: {e}")

    def process_request(self, request: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        """
        Process a single JSON-RPC 2.0 request.

        Args:
            request: JSON-RPC request dict.

        Returns:
            JSON-RPC response dict, or None if notification.
        """
        # Extract fields
        jsonrpc = request.get("jsonrpc")
        method = request.get("method")
        params = request.get("params", {})
        request_id = request.get("id")

        # Validate JSON-RPC version
        if jsonrpc != "2.0":
            return {
                "jsonrpc": "2.0",
                "error": {
                    "code": -32600,
                    "message": "Invalid Request",
                    "data": "jsonrpc must be 2.0"
                },
                "id": request_id,
            }

        # Check if method exists
        if method not in self.methods:
            return {
                "jsonrpc": "2.0",
                "error": {
                    "code": -32601,
                    "message": "Method not found",
                    "data": f"Unknown method: {method}"
                },
                "id": request_id,
            }

        # Call method
        try:
            handler = self.methods[method]
            result = handler(params)

            # If this is a notification (no id), return None
            if "id" not in request:
                return None

            return {
                "jsonrpc": "2.0",
                "result": result,
                "id": request_id,
            }
        except Exception as e:
            # If this is a notification, still log but don't return error
            if "id" not in request:
                print(f"Error in notification: {e}", file=sys.stderr)
                traceback.print_exc(file=sys.stderr)
                return None

            return {
                "jsonrpc": "2.0",
                "error": {
                    "code": -32603,
                    "message": "Internal error",
                    "data": str(e)
                },
                "id": request_id,
            }

    def run(self) -> None:
        """
        Main server loop.
        Reads line-delimited JSON from stdin, writes responses to stdout.
        """
        while True:
            try:
                line = sys.stdin.readline()
                if not line:
                    # EOF
                    break

                line = line.strip()
                if not line:
                    continue

                try:
                    request = json.loads(line)
                except json.JSONDecodeError as e:
                    response = {
                        "jsonrpc": "2.0",
                        "error": {
                            "code": -32700,
                            "message": "Parse error",
                            "data": str(e)
                        },
                        "id": None,
                    }
                    sys.stdout.write(json.dumps(response) + "\n")
                    sys.stdout.flush()
                    continue

                # Process the request
                response = self.process_request(request)

                # Write response (skip for notifications)
                if response is not None:
                    sys.stdout.write(json.dumps(response) + "\n")
                    sys.stdout.flush()

            except KeyboardInterrupt:
                break
            except Exception as e:
                print(f"Server error: {e}", file=sys.stderr)
                traceback.print_exc(file=sys.stderr)
                break


if __name__ == "__main__":
    server = JSONRPCServer()
    server.run()
