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
    from model.urdf_parser import parse_urdf, parse_urdf_string
    from model.kinematic_graph import KinematicGraph
    from model.urdf_serializer import serialize_to_urdf
    from validation.validator import validate_kinematic_graph
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
    from sim.mujoco_adapter import MuJoCoSimulator as _MuJoCoSimulator
except ImportError as e:
    _mujoco_import_error = str(e)
    print(f"Warning: MuJoCo not available: {e}", file=sys.stderr)

# Lazy import AI client — anthropic may not be installed
_generate_edit = None
_generate_edit_streaming = None
_generate_completion = None
_ai_import_error = None

_generate_assembly_with_tools = None
_validate_assembly = None
_set_conversation_history = None
try:
    from ai.claude_client import generate_edit as _generate_edit, generate_edit_streaming as _generate_edit_streaming, generate_completion as _generate_completion, generate_assembly_with_tools as _generate_assembly_with_tools, validate_assembly as _validate_assembly, set_conversation_history as _set_conversation_history
except ImportError as e:
    _ai_import_error = str(e)
    print(f"Warning: AI client not available: {e}", file=sys.stderr)

# Always-available local completion engine (no external dependencies)
from ai.local_completions import generate_local_completion as _generate_local_completion


class JSONRPCServer:
    """Simple JSON-RPC 2.0 server."""

    def __init__(self):
        """Initialize the server."""
        self.simulator = _MuJoCoSimulator() if _MuJoCoSimulator else None
        # Script runner state (Phase C)
        self.sim_script_fn = None   # compiled step(t, state) callable or None
        self.sim_script_error: Optional[str] = None
        self.methods = {
            "parse_urdf": self.handle_parse_urdf,
            "ping": self.handle_ping,
            "sim_load": self.handle_sim_load,
            "sim_step": self.handle_sim_step,
            "sim_reset": self.handle_sim_reset,
            "sim_set_control": self.handle_sim_set_control,
            "sim_get_state": self.handle_sim_get_state,
            "sim_render": self.handle_sim_render,
            "sim_set_gravity": self.handle_sim_set_gravity,
            "sim_set_script": self.handle_sim_set_script,
            "validate_urdf": self.handle_validate_urdf,
            "validate_urdf_content": self.handle_validate_urdf_content,
            "ai_edit": self.handle_ai_edit,
            "ai_complete": self.handle_ai_complete,
            "ai_validate_assembly": self.handle_ai_validate_assembly,
            "ai_set_history": self.handle_ai_set_history,
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

        if _parse_urdf is None:
            raise ValueError(f"Model modules not available: {_model_import_error}")

        try:
            kg = _parse_urdf(path)
            return kg.to_json()
        except FileNotFoundError as e:
            raise ValueError(f"File not found: {e}")
        except Exception as e:
            raise ValueError(f"Failed to parse URDF: {e}")

    def handle_validate_urdf(self, params: Dict[str, Any]) -> Dict[str, Any]:
        """
        Parse a URDF file and run validation checks.

        Params:
            path (str): Path to the URDF file.

        Returns:
            Dict with 'results' list and 'summary' counts.
        """
        if "path" not in params:
            raise ValueError("Missing required parameter: path")

        path = params["path"]
        if not isinstance(path, str):
            raise ValueError("Parameter 'path' must be a string")

        if _parse_urdf is None or _validate_kinematic_graph is None:
            raise ValueError(f"Model modules not available: {_model_import_error}")

        try:
            kg = _parse_urdf(path)
            results = _validate_kinematic_graph(kg)

            # Build summary counts
            summary = {"pass": 0, "warn": 0, "error": 0, "info": 0}
            for r in results:
                sev = r.get("severity", "info")
                if sev in summary:
                    summary[sev] += 1

            return {"results": results, "summary": summary}
        except FileNotFoundError as e:
            raise ValueError(f"File not found: {e}")
        except Exception as e:
            raise ValueError(f"Validation failed: {e}")

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
            results = _validate_kinematic_graph(kg)

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
        seed = params.get("seed", None)
        # Clear any active script when loading a new model
        self.sim_script_fn = None
        self.sim_script_error = None
        try:
            if seed is not None:
                try:
                    import numpy as np
                    np.random.seed(int(seed))
                except Exception:
                    pass
            return self.simulator.load_urdf(path, free_base=free_base)
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
            # Script runner: observe state → compute controls → apply before advancing
            script_error: Optional[str] = None
            if self.sim_script_fn is not None:
                try:
                    current_state = self.simulator.get_state()
                    controls = self.sim_script_fn(current_state["time"], current_state)
                    if isinstance(controls, dict):
                        self.simulator.set_control(controls)
                except Exception as se:
                    script_error = str(se)

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
            code (str): Python source defining ``def step(t, state) -> dict``.
                        Pass empty string to clear.

        Returns:
            {"status": "ok" | "cleared" | "error", "message": str (on error)}
        """
        code = params.get("code", "").strip()
        if not code:
            self.sim_script_fn = None
            self.sim_script_error = None
            return {"status": "cleared"}
        try:
            namespace: Dict[str, Any] = {"__builtins__": __builtins__}
            exec(compile(code, "<sim_script>", "exec"), namespace)
            fn = namespace.get("step")
            if fn is None or not callable(fn):
                raise ValueError("Script must define a callable 'step(t, state)' function")
            self.sim_script_fn = fn
            self.sim_script_error = None
            return {"status": "ok"}
        except Exception as e:
            self.sim_script_fn = None
            self.sim_script_error = str(e)
            return {"status": "error", "message": str(e)}

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

    def _emit_progress(self, stage: str, text: str) -> None:
        """Emit a JSON-RPC notification for AI progress (no id = notification)."""
        notification = {
            "jsonrpc": "2.0",
            "method": "ai_progress",
            "params": {"stage": stage, "text": text}
        }
        sys.stdout.write(json.dumps(notification, ensure_ascii=False) + "\n")
        sys.stdout.flush()

    def handle_ai_edit(self, params: Dict[str, Any]) -> Dict[str, Any]:
        """
        Use Claude AI to generate a robot model edit from natural language.
        Uses streaming API with progress notifications when available.
        """
        if _generate_edit is None:
            raise ValueError(
                f"Claude AI not installed. Run: pip install anthropic\n"
                f"Error: {_ai_import_error}"
            )

        if "prompt" not in params or "urdf_content" not in params:
            raise ValueError("Missing required parameters: prompt, urdf_content")

        prompt = params["prompt"]
        urdf_content = params["urdf_content"]
        kinematic_context = params.get("kinematic_context", None)
        session_id = params.get("session_id", "default")

        if not isinstance(prompt, str):
            raise ValueError("Parameter 'prompt' must be a string")
        if not isinstance(urdf_content, str):
            raise ValueError("Parameter 'urdf_content' must be a string")

        # Model whitelist — reject unknown IDs by falling back to Sonnet
        _ALLOWED_MODELS = {"claude-sonnet-4-6", "claude-opus-4-7"}
        model = params.get("model") or "claude-sonnet-4-6"
        if model not in _ALLOWED_MODELS:
            print(f"[ai_edit] Unknown model '{model}', falling back to claude-sonnet-4-6", file=sys.stderr)
            model = "claude-sonnet-4-6"

        # Normalize images: strip any data:image/...;base64, prefix the frontend
        # may have included (Anthropic SDK rejects it).
        raw_images = params.get("images") or []
        images: list = []
        if isinstance(raw_images, list):
            for img in raw_images:
                if not isinstance(img, dict):
                    continue
                media_type = img.get("media_type")
                data = img.get("data", "")
                if not isinstance(data, str) or not isinstance(media_type, str):
                    continue
                if data.startswith("data:"):
                    comma = data.find(",")
                    if comma != -1:
                        data = data[comma + 1:]
                images.append({"media_type": media_type, "data": data})

        try:
            # Tool-use assembly agent (disabled by default — too many API calls / expensive)
            # To enable: pass "use_tools": true in params
            if params.get("use_tools") and _generate_assembly_with_tools is not None:
                print(f"[ai_edit] Using tool-use assembly agent for: {prompt[:80]}", file=sys.stderr)
                result = _generate_assembly_with_tools(
                    prompt, session_id,
                    on_progress=self._emit_progress,
                    model=model,
                    images=images,
                )
                self._emit_progress("done", "Complete")
                return {
                    "explanation": result.get("explanation", "Assembly complete"),
                    "new_urdf": result.get("new_urdf", urdf_content),
                    "stats": result.get("stats", "Assembly complete"),
                }

            # Standard edit path
            kg_json = {}
            try:
                if _parse_urdf_string is not None:
                    kg = _parse_urdf_string(urdf_content)
                    kg_json = kg.to_json()
            except Exception as parse_err:
                print(f"[ai_edit] URDF pre-parse skipped: {parse_err}", file=sys.stderr)

            if _generate_edit_streaming is not None:
                result = _generate_edit_streaming(
                    prompt, urdf_content, kg_json, kinematic_context, session_id,
                    on_progress=self._emit_progress,
                    model=model,
                    images=images,
                )
            else:
                self._emit_progress("thinking", "Processing request...")
                result = _generate_edit(
                    prompt, urdf_content, kg_json, kinematic_context, session_id,
                    model=model,
                    images=images,
                )

            self._emit_progress("done", "Complete")

            response = {
                "explanation": result.get("explanation", "Edit applied"),
                "new_urdf": result.get("new_urdf", urdf_content),
                "stats": result.get("stats", "Edit complete"),
            }
            if "assembly_graph" in result:
                response["assembly_graph"] = result["assembly_graph"]
            if "topology_ops" in result:
                response["topology_ops"] = result["topology_ops"]
            return response
        except Exception as e:
            raise ValueError(f"AI edit failed: {e}")

    def handle_ai_complete(self, params: Dict[str, Any]) -> str:
        """
        Generate inline completions for URDF/XML editing.
        Uses Claude API if available, falls back to local pattern-based completions.

        Params:
            urdf_content (str): Current URDF XML as string.
            cursor_line (int): Current cursor line (1-indexed).
            cursor_column (int): Current cursor column (1-indexed).
            prefix (str): Optional prefix context (e.g., recent characters typed).

        Returns:
            Completion text string (the text to insert at cursor).
        """
        if "urdf_content" not in params:
            raise ValueError("Missing required parameter: urdf_content")

        urdf_content = params["urdf_content"]
        cursor_line = params.get("cursor_line", 1)
        cursor_column = params.get("cursor_column", 1)
        prefix = params.get("prefix", "")
        kinematic_context = params.get("kinematic_context", "")

        if not isinstance(urdf_content, str):
            raise ValueError("Parameter 'urdf_content' must be a string")
        if not isinstance(cursor_line, int) or not isinstance(cursor_column, int):
            raise ValueError("Parameters 'cursor_line' and 'cursor_column' must be integers")

        print(f"[server] ai_complete: line={cursor_line}, col={cursor_column}", file=sys.stderr)

        # Try Claude API first, fall back to local completions
        if _generate_completion is not None:
            try:
                completion = _generate_completion(
                    urdf_content,
                    cursor_line,
                    cursor_column,
                    prefix,
                    kinematic_context
                )
                print(f"[server] ai_complete (claude): got {len(completion)} chars", file=sys.stderr)
                return completion
            except Exception as e:
                print(f"[server] Claude completion failed: {e}", file=sys.stderr)
                return ""

        # No Claude API available — return empty rather than low-quality local suggestions
        print(f"[server] ai_complete: Claude not available, returning empty", file=sys.stderr)
        return ""

    def handle_ai_validate_assembly(self, params: Dict[str, Any]) -> Dict[str, Any]:
        """
        Second-pass AI validation of an assembled URDF.
        Sends the URDF back to Claude (Haiku) for spatial correctness checks.
        Returns corrections if needed (~$0.01-0.02 per call).

        Params:
            urdf_content (str): The assembled URDF XML.
            original_prompt (str): The user's original build request.
            session_id (str, optional): Session identifier.

        Returns:
            Dict with 'ok' bool, 'notes' str, and optional 'corrected_urdf' str.
        """
        if _validate_assembly is None:
            raise ValueError(
                f"Claude AI not installed. Run: pip install anthropic\n"
                f"Error: {_ai_import_error}"
            )

        if "urdf_content" not in params or "original_prompt" not in params:
            raise ValueError("Missing required parameters: urdf_content, original_prompt")

        urdf_content = params["urdf_content"]
        original_prompt = params["original_prompt"]
        session_id = params.get("session_id", "default")
        screenshot_base64 = params.get("screenshot_base64")
        screenshots = params.get("screenshots")  # array of 3 base64 PNGs

        try:
            self._emit_progress("validating", "Checking assembly with visual feedback...")
            result = _validate_assembly(urdf_content, original_prompt, session_id, screenshot_base64, screenshots)
            self._emit_progress("done", "Validation complete")
            return result
        except Exception as e:
            raise ValueError(f"Assembly validation failed: {e}")

    def handle_ai_set_history(self, params: Dict[str, Any]) -> Dict[str, Any]:
        """
        Restore conversation history for a session from frontend localStorage.
        Called on reconnect to maintain context across backend restarts.

        Params:
            session_id (str): Session identifier.
            history (list): List of {role, content} message dicts.

        Returns:
            Dict with 'status' and 'count'.
        """
        if _set_conversation_history is None:
            raise ValueError(
                f"Claude AI not installed. Run: pip install anthropic\n"
                f"Error: {_ai_import_error}"
            )

        if "session_id" not in params or "history" not in params:
            raise ValueError("Missing required parameters: session_id, history")

        session_id = params["session_id"]
        history = params["history"]

        if not isinstance(session_id, str):
            raise ValueError("Parameter 'session_id' must be a string")
        if not isinstance(history, list):
            raise ValueError("Parameter 'history' must be a list")

        try:
            return _set_conversation_history(session_id, history)
        except Exception as e:
            raise ValueError(f"Failed to set history: {e}")

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
            import io
            sys.stdin = io.TextIOWrapper(sys.stdin.buffer, encoding='utf-8')
            sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')
            sys.stderr = io.TextIOWrapper(sys.stderr.buffer, encoding='utf-8')

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
                    sys.stdout.write(json.dumps(response, ensure_ascii=False) + "\n")
                    sys.stdout.flush()
                    continue

                # Process the request
                response = self.process_request(request)

                # Write response (skip for notifications)
                if response is not None:
                    sys.stdout.write(json.dumps(response, ensure_ascii=False) + "\n")
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
