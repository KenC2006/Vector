#!/usr/bin/env python3
"""
JSON-RPC 2.0 server over stdio.

Reads line-delimited JSON from stdin, writes responses to stdout.
Minimal implementation without external JSON-RPC libraries.
"""
import json
import math
import sys
from typing import Any, Dict, Optional
import traceback
import os
import io

# stdout is the JSON-RPC channel. Keep a private handle on it and point file
# descriptor 1 at stderr, so nothing else in the process — a print() in a
# library, a sim script, C code writing to fd 1 — can inject a line into the
# response stream.
_RPC_OUT = sys.stdout
if __name__ == "__main__":
    _RPC_OUT = io.TextIOWrapper(io.FileIO(os.dup(sys.stdout.fileno()), "w"), encoding="utf-8",
                                newline="\n", write_through=True)
    sys.stdout.flush()
    os.dup2(sys.stderr.fileno(), sys.stdout.fileno())
    sys.stdout = sys.stderr


def _send(message: Dict[str, Any]) -> None:
    _RPC_OUT.write(json.dumps(message, ensure_ascii=False) + "\n")
    _RPC_OUT.flush()

# Run as `python -m core.server` from the project root: every project import is
# a `core.*` package import (a bare `sim`/`model` path would load modules twice).

# Auto-load .env file from project root (if python-dotenv is installed)
try:
    from dotenv import load_dotenv
    # Walk up from core/ to project root to find .env
    _project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    _env_path = os.path.join(_project_root, '.env')
    if os.path.exists(_env_path):
        load_dotenv(_env_path)
        print(f"Loaded .env from {_env_path}", file=sys.stderr)
    else:
        # Also check current working directory
        if os.path.exists('.env'):
            load_dotenv('.env')
            print("Loaded .env from cwd", file=sys.stderr)
except ImportError:
    # dotenv not installed — env vars must be set manually
    pass

# Lazy imports — these modules may have missing dependencies (networkx, etc.)
_parse_urdf = None
_parse_urdf_string = None
_KinematicGraph = None
_serialize_to_urdf = None
_validate_kinematic_graph = None
_model_import_error = None

try:
    from core.model.urdf_parser import parse_urdf, parse_urdf_string
    from core.model.kinematic_graph import KinematicGraph
    from core.model.urdf_serializer import serialize_to_urdf
    from core.validation.validator import validate_kinematic_graph
    _parse_urdf = parse_urdf
    _parse_urdf_string = parse_urdf_string
    _KinematicGraph = KinematicGraph
    _serialize_to_urdf = serialize_to_urdf
    _validate_kinematic_graph = validate_kinematic_graph
except ImportError as e:
    _model_import_error = str(e)
    print(f"Warning: Model modules not fully available: {e}", file=sys.stderr)
    print(f"  Install missing deps: pip install networkx", file=sys.stderr)

# Lazy import MuJoCo — it may not be installed
_MuJoCoSimulator = None
_mujoco_import_error = None

try:
    from core.sim.mujoco_adapter import MuJoCoSimulator as _MuJoCoSimulator
except ImportError as e:
    _mujoco_import_error = str(e)
    print(f"Warning: MuJoCo not available: {e}", file=sys.stderr)

# ── sim scripts ───────────────────────────────────────────────────────────────
# The sandbox, the fixed-rate controller runtime and the state handed to
# step(t, state) live in sim/control.py (shared with the headless evaluator the
# controller agent tests against, so the app and the tests run identical loops).
from core.sim.control import compile_script as _compile_sim_script
try:
    from core.sim.control import ControllerRuntime as _ControllerRuntime
except Exception:  # pragma: no cover — only when numpy/mujoco are unavailable
    _ControllerRuntime = None


def _design_critic_results(urdf_content: str) -> list:
    """Geometry-critic findings (floating/clipping parts, ground contact,
    tipping, overloaded joints), measured on the URDF's design."""
    try:
        from core.designer.compile import compile_design
        from core.designer.critic import critique
        from core.designer.importer import import_urdf
    except Exception:
        return []
    try:
        design = import_urdf(urdf_content)["design"]
    except Exception:
        return []
    try:
        issues = critique(compile_design(design))["issues"]
    except Exception as e:
        return [{"name": "Design", "severity": "warn", "message": f"Design could not be re-checked: {e}", "category": "Design"}]
    if not issues:
        return [{"name": "Design", "severity": "pass", "message": "Parts attached, grounded and within actuator ratings", "category": "Design"}]
    return [{"name": i.split(":", 1)[0], "severity": "warn", "message": i.split(":", 1)[-1].strip(), "category": "Design"}
            for i in issues]


def _quick_summary(ev: Dict[str, Any]) -> str:
    """One measured line about a baseline controller's 6 s test run."""
    if not ev.get("ok"):
        return f"Test run failed: {ev.get('error')}"
    if not ev.get("free_base"):
        return "Holds the as-designed pose (fixed base). Use Generate for a motion routine."
    if ev.get("fell_at_s") is not None:
        return f"Falls over after {ev['fell_at_s']:.1f} s — use Generate for a balancing controller."
    speed = ev.get("avg_forward_speed_mps", 0.0) * 100
    if abs(speed) < 0.5:
        return f"Stands steady (max tilt {ev['max_tilt_deg']:.0f}°). Drive it with WASD, or Generate for a gait."
    return (f"Measured: {speed:+.1f} cm/s forward, heading drift {ev['yaw_change_deg']:+.0f}°, "
            f"max tilt {ev['max_tilt_deg']:.0f}°. Drive with WASD.")


class JSONRPCServer:
    """Simple JSON-RPC 2.0 server."""

    def __init__(self):
        """Initialize the server."""
        self.simulator = _MuJoCoSimulator() if _MuJoCoSimulator else None
        # Controller runtime for the loaded model (runs step(t, state) at a fixed rate).
        self.sim_runtime = None
        self.methods = {
            "ping": self.handle_ping,
            "sim_load": self.handle_sim_load,
            "sim_step": self.handle_sim_step,
            "sim_reset": self.handle_sim_reset,
            "sim_get_state": self.handle_sim_get_state,
            "sim_set_gravity": self.handle_sim_set_gravity,
            "sim_set_script": self.handle_sim_set_script,
            "sim_set_command": self.handle_sim_set_command,
            "validate_urdf_content": self.handle_validate_urdf_content,
            "design_compile": self.handle_design_compile,
            "design_import": self.handle_design_import,
            "ai_design": self.handle_ai_design,
            "ai_gen_sim_script": self.handle_ai_gen_sim_script,
        }

    def handle_validate_urdf_content(self, params: Dict[str, Any]) -> Dict[str, Any]:
        """
        Validate URDF content from a string (not a file path).
        Used by the frontend editor to validate as-you-type.

        Params:
            urdf_content (str): URDF XML as a string.

        Returns:
            Dict with 'results' list and 'summary' counts.
        """
        if "urdf_content" not in params:
            raise ValueError("Missing required parameter: urdf_content")

        urdf_content = params["urdf_content"]
        if not isinstance(urdf_content, str):
            raise ValueError("Parameter 'urdf_content' must be a string")

        if _parse_urdf_string is None or _validate_kinematic_graph is None:
            return {
                "results": [{
                    "name": "Dependencies Missing",
                    "severity": "warn",
                    "message": f"Model modules not available: {_model_import_error}. Install: pip install networkx",
                    "category": "Setup"
                }],
                "summary": {"pass": 0, "warn": 1, "error": 0, "info": 0}
            }

        try:
            kg = _parse_urdf_string(urdf_content)
            results = _validate_kinematic_graph(kg) + _design_critic_results(urdf_content)

            # Build summary counts
            summary = {"pass": 0, "warn": 0, "error": 0, "info": 0}
            for r in results:
                sev = r.get("severity", "info")
                if sev in summary:
                    summary[sev] += 1

            return {"results": results, "summary": summary}
        except Exception as e:
            # If parsing fails, return a parse error
            return {
                "results": [{
                    "name": "URDF Parse Error",
                    "severity": "error",
                    "message": str(e),
                    "category": "Structural"
                }],
                "summary": {"pass": 0, "warn": 0, "error": 1, "info": 0}
            }

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

        free_base = bool(params.get("free_base", False))
        terrain_config = params.get("terrain_config", params.get("terrainConfig", None))
        if terrain_config is not None and not isinstance(terrain_config, dict):
            terrain_config = None
        seed = params.get("seed", None)
        # Loading a model clears any active script.
        self.sim_runtime = None
        try:
            if seed is not None:
                try:
                    import numpy as np
                    np.random.seed(int(seed))
                except Exception:
                    pass
                try:
                    import mujoco as _mj
                    _mj.mj_setSeed(int(seed))
                except Exception:
                    pass
            info = self.simulator.load_urdf(
                path,
                free_base=free_base,
                terrain_config=terrain_config,
            )
            if _ControllerRuntime is not None:
                self.sim_runtime = _ControllerRuntime(self.simulator)
            return info
        except FileNotFoundError as e:
            raise ValueError(f"File not found: {e}")
        except Exception as e:
            raise ValueError(f"Failed to load URDF: {e}")

    def handle_sim_step(self, params: Dict[str, Any]) -> Dict[str, Any]:
        """
        Step the simulation forward.

        Params:
            n_steps (int, optional): Number of physics steps to advance. Default: 1.

        The active script (if any) runs every 5 ms of sim time inside this call,
        so control is independent of how many steps the UI asks for per frame.

        Returns:
            Current simulation state.
        """
        self._require_simulator()
        n_steps = params.get("n_steps", 1)
        if not isinstance(n_steps, int) or n_steps < 1:
            n_steps = 1

        try:
            script_error: Optional[str] = None
            rt = self.sim_runtime
            if rt is not None and rt.fn is not None:
                try:
                    rt.advance(n_steps)
                except Exception as se:
                    if "diverged" in str(se):
                        raise
                    script_error = str(se)
                    # Fail safe: stop the script and zero the commands so stale
                    # targets don't keep driving the robot.
                    rt.set_script(None)
                    try:
                        self.simulator.set_control({})
                    except Exception:
                        pass
            else:
                self.simulator.step(n_steps)
            state = self.simulator.get_state()
            if script_error:
                state["script_error"] = script_error
            return state
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
            if self.sim_runtime is not None:
                self.sim_runtime.reset()
            return self.simulator.get_state()
        except Exception as e:
            raise ValueError(f"Failed to reset simulation: {e}")

    def handle_sim_set_gravity(self, params: Dict[str, Any]) -> None:
        """
        Set gravity vector.

        Params:
            gravity (list[float]): [gx, gy, gz] in m/s². Pass [0,0,0] for zero-G.
        """
        self._require_simulator()
        gravity = params.get("gravity", [0.0, 0.0, -9.81])
        if not isinstance(gravity, list) or len(gravity) != 3:
            raise ValueError("Parameter 'gravity' must be a list of 3 floats")
        try:
            self.simulator.set_gravity([float(g) for g in gravity])
        except Exception as e:
            raise ValueError(f"Failed to set gravity: {e}")

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

    def handle_sim_set_script(self, params: Dict[str, Any]) -> Dict[str, Any]:
        """
        Compile and install a Python step-callback script.

        Params:
            code (str): Python source defining step(t, state) -> dict.
                        Pass empty string to clear.

        Returns:
            {"status": "ok" | "cleared" | "error", "message": str (on error)}
        """
        code = params.get("code", "").strip()
        rt = self.sim_runtime
        if not code:
            if rt is not None:
                rt.set_script(None)
            return {"status": "cleared"}
        try:
            fn = _compile_sim_script(code)
            if rt is None:
                raise ValueError("Load a model into the simulator first")
            rt.set_script(fn)
            return {"status": "ok"}
        except Exception as e:
            if rt is not None:
                rt.set_script(None)
            return {"status": "error", "message": str(e)}

    def handle_sim_set_command(self, params: Dict[str, Any]) -> Dict[str, Any]:
        """
        Set the operator command controllers see as state["cmd"] (keyboard teleop).

        Params: active (bool), vx (m/s), vy (m/s), yaw_rate (rad/s).
        """
        rt = self.sim_runtime
        if rt is None:
            return {"status": "no_model"}
        cmd: Dict[str, Any] = {"active": bool(params.get("active", False))}
        for k in ("vx", "vy", "yaw_rate"):
            try:
                cmd[k] = float(params.get(k, 0.0) or 0.0)
            except (TypeError, ValueError):
                cmd[k] = 0.0
        rt.cmd = cmd
        return {"status": "ok"}

    def _emit_progress(self, stage: str, text: str) -> None:
        """Emit a JSON-RPC notification for AI progress (no id = notification)."""
        notification = {
            "jsonrpc": "2.0",
            "method": "ai_progress",
            "params": {"stage": stage, "text": text}
        }
        _send(notification)

    def handle_design_compile(self, params: Dict[str, Any]) -> Dict[str, Any]:
        """
        Compile a design (the explicit-pose part list) to URDF. Deterministic —
        this is what every manual edit (placing, moving, rotating, joint
        settings) goes through.

        Params: design (dict), check (bool, optional — also run the geometry
        critic). Returns: {urdf, root, parts: {part: {link, component, parent,
        mirror_of, p, R, frame_p, size, center_local, joint}}, issues: [str]}.
        Poses are in the design world frame (mm, X forward, Z up, grounded);
        `R` rows, columns = the part's local axes; the URDF link frame of a
        part is (R, frame_p).
        """
        from core.designer.compile import DesignError, compile_design
        design = params.get("design")
        if not isinstance(design, dict):
            raise ValueError("Missing required parameter: design")
        try:
            asm = compile_design(design)
        except DesignError as e:
            return {"error": str(e)}
        def r(v):
            return [round(float(x), 6) for x in v]
        parts = {n: {"link": q.link, "component": q.component["id"] if q.component else None,
                     "parent": q.parent, "mirror_of": q.mirror_of,
                     "p": r(q.p), "R": [r(row) for row in q.R], "frame_p": r(q.frame_p),
                     "size": r(q.size), "center_local": r(q.center_local),
                     "joint": ({"type": q.joint["type"], "passive": bool(q.joint.get("passive")),
                                **({"pivot": r(q.joint["pivot"]), "axis": r(q.joint["axis"])}
                                   if "pivot" in q.joint else {})} if q.joint else None)}
                 for n, q in asm.parts.items()}
        issues = []
        if params.get("check"):
            from core.designer.critic import critique
            issues = critique(asm)["issues"]
        return {"urdf": asm.to_urdf(design.get("name") or "robot"), "root": asm.root, "parts": parts, "issues": issues}

    def handle_design_import(self, params: Dict[str, Any]) -> Dict[str, Any]:
        """
        URDF -> design. Returns the embedded design when the URDF is untouched
        designer output; otherwise rebuilds one from the URDF's geometry (hand
        edits, hand-written robots). Returns: {design, notes, imported: bool}.
        """
        from core.designer.importer import ImportError_, import_urdf
        urdf = params.get("urdf_content")
        if not isinstance(urdf, str) or not urdf.strip():
            raise ValueError("Missing required parameter: urdf_content")
        try:
            out = import_urdf(urdf)
        except ImportError_ as e:
            return {"error": str(e)}
        return out

    def handle_ai_design(self, params: Dict[str, Any]) -> Dict[str, Any]:
        """
        Design (or redesign) a robot with the explicit-pose designer.

        Params: prompt (str), urdf_content (str, optional — when it carries an
        embedded design, the request is treated as an edit of it), images.
        Returns: {urdf, summary, report, issues, rounds}.
        """
        from core.designer.agent import design_robot
        prompt = params.get("prompt")
        if not isinstance(prompt, str) or not prompt.strip():
            raise ValueError("Missing required parameter: prompt")
        current = params.get("urdf_content") or ""
        images = []
        for img in params.get("images") or []:
            if not isinstance(img, dict):
                continue
            media_type, data = img.get("media_type"), img.get("data", "")
            if not isinstance(media_type, str) or not isinstance(data, str) or not data:
                continue
            if data.startswith("data:") and "," in data:
                data = data.split(",", 1)[1]
            images.append({"media_type": media_type, "data": data})
        out = design_robot(prompt, current_urdf=current if isinstance(current, str) else "",
                           progress=self._emit_progress, images=images or None)
        return {k: out[k] for k in ("urdf", "summary", "report", "issues", "rounds")}

    def handle_ai_gen_sim_script(self, params: Dict[str, Any]) -> Dict[str, Any]:
        """
        Generate a controller with the controller agent: it measures the robot,
        writes a controller, tests it headless in MuJoCo, and revises until it
        works, then returns the tested code.

        Params:
            prompt (str): Natural-language request (may be empty: default behaviour).
            urdf_content (str): Current URDF.
            current_script (str, optional): Treat the prompt as a change to this script.
            terrain_config (dict, optional): Active sim terrain.

        Returns:
            {"status": "ok", "code", "summary", "report", "tests"} or {"status": "error", "message"}.
        """
        prompt = params.get("prompt", "") or ""
        urdf_content = params.get("urdf_content", "")
        current_script = params.get("current_script", "") or ""
        terrain_config = params.get("terrain_config", params.get("terrainConfig", None))
        if terrain_config is not None and not isinstance(terrain_config, dict):
            terrain_config = None
        if not isinstance(prompt, str):
            raise ValueError("Parameter 'prompt' must be a string")
        if not isinstance(urdf_content, str) or not urdf_content.strip():
            raise ValueError("Parameter 'urdf_content' must be a non-empty string")
        if params.get("quick"):
            try:
                from core.sim.control import evaluate_controller, format_evaluation, robot_brief_data
                from core.sim.controllers import baseline_controller
                code = baseline_controller(robot_brief_data(urdf_content))
                ev = evaluate_controller(urdf_content, code, terrain_config=terrain_config)
            except Exception as e:
                return {"status": "error", "message": f"Could not build a controller: {e}"}
            return {"status": "ok", "code": code, "summary": _quick_summary(ev),
                    "report": format_evaluation(ev), "tests": 1}
        try:
            from core.ai.controller_agent import generate_controller
            out = generate_controller(prompt, urdf_content, current_script=current_script,
                                      terrain_config=terrain_config, progress=self._emit_progress)
        except Exception as e:
            return {"status": "error", "message": f"Controller generation failed: {e}"}
        return {"status": "ok", "code": out["code"], "summary": out["summary"],
                "report": out["report"], "tests": out["tests"]}

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
        # Force UTF-8 on Windows (default is often cp1252)
        if sys.platform == 'win32':
            sys.stdin = io.TextIOWrapper(sys.stdin.buffer, encoding='utf-8')

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
                    _send(response)
                    continue

                # Process the request
                response = self.process_request(request)

                # Write response (skip for notifications)
                if response is not None:
                    _send(response)

            except KeyboardInterrupt:
                break
            except Exception as e:
                print(f"Server error: {e}", file=sys.stderr)
                traceback.print_exc(file=sys.stderr)
                break


if __name__ == "__main__":
    server = JSONRPCServer()
    server.run()
