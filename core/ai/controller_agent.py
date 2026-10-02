"""
Controller agent: Claude writes a sim controller, tests it headless in MuJoCo,
reads what the robot actually did, and revises — then submits the version that
works. It starts from a measured robot brief (what every joint's positive
direction does) and a baseline controller with its own test report, so it never
guesses joint polarity from names.
"""
from __future__ import annotations

import os
import sys
import time
from typing import Any, Callable, Dict, List, Optional

from core.sim.control import (
    compile_script, evaluate_controller, format_evaluation, format_robot_brief, robot_brief_data,
)
from core.sim.controllers import baseline_controller

CONTROLLER_MODEL = os.environ.get("VECTOR_CONTROLLER_MODEL", os.environ.get("VECTOR_DESIGN_MODEL", "claude-opus-5"))
CONTROLLER_EFFORT = os.environ.get("VECTOR_CONTROLLER_EFFORT", "medium")
MAX_TESTS = 6

TEST_TOOL = {
    "name": "test_controller",
    "description": ("Run a controller headless in the simulator (same physics, terrain and control rate as the app) "
                    "and get a report of what the robot actually did."),
    "input_schema": {
        "type": "object",
        "properties": {
            "code": {"type": "string", "description": "complete Python module defining step(t, state)"},
            "seconds": {"type": "number", "description": "sim duration, default 6, max 12"},
            "cmd": {"type": "object", "description": "optional operator command to test teleop, e.g. "
                    "{\"active\": true, \"vx\": 0.2, \"yaw_rate\": 0.5}"},
        },
        "required": ["code"],
    },
}
SUBMIT_TOOL = {
    "name": "submit_controller",
    "description": "Hand the final controller to the user. Only submit code you have tested.",
    "input_schema": {
        "type": "object",
        "properties": {
            "code": {"type": "string"},
            "summary": {"type": "string", "description": "1-2 sentences for the user: what it does and how well it works (measured)"},
        },
        "required": ["code", "summary"],
    },
}

SYSTEM_PROMPT = r"""You write controllers for robots in Vector's MuJoCo simulator.

# Contract
A controller is a Python module defining `step(t, state)` that returns `{joint_name: command}`.
It is called at a fixed 200 Hz (state["dt"] = 0.005 s), independent of the display.

state = {
  "t": sim time (s),  "dt": control period (s),
  "q":  {joint: position},        # rad (revolute), m (prismatic); continuous joints: unwrapped angle
  "qd": {joint: velocity},        # rad/s or m/s
  "effort": {joint: actuator torque/force now},
  "base": {"pos": [x,y,z] m world, "rpy": [roll, pitch, yaw] rad,
           "lin_vel": [vx, vy, vz] m/s in the heading frame (x = robot forward),
           "ang_vel": [wx, wy, wz] rad/s body frame},
  "contacts": [names of robot links touching the ground now],
  "cmd": {"active": bool, "vx": m/s, "vy": m/s, "yaw_rate": rad/s},   # operator teleop (keyboard)
}

Commands:
- revolute / prismatic joints: TARGET POSITION (rad / m). Each is a position servo (stiff PD, torque-capped
  at the joint's effort). Targets are clamped to the joint limits.
- continuous joints (wheels, track drives): TARGET SPEED in rad/s (a speed-controlled motor, capped at its
  max speed and effort). Ground speed = rad/s x wheel radius.
- PASSIVE joints have no motor — never command them.
- Omitted joints keep their previous command (initially 0 = the as-designed pose / stopped).

# Sandbox (hard rules)
No imports (`math` is already available). No eval/exec/open/getattr/globals, no dunder attributes.
Builtins: abs min max round sum len range enumerate zip map filter sorted reversed all any int float bool
str list tuple dict set print isinstance. Module-level constants and helper functions are fine.
Module-level mutable state (e.g. a dict you update each tick) is allowed for filters/phase tracking, but
the controller must still be deterministic.

# Understanding the robot
The ROBOT brief is measured from the model, not guessed: for every joint it says what a positive command
physically does (which way the foot/tip moves, which way a wheel drives the robot). Use those signs and
magnitudes. Never infer polarity or roles from joint names.

# How to write good controllers
- Ease in over ~0.5-1 s from the zero pose; never snap.
- Mobile robots: when state["cmd"]["active"] is true, follow cmd (vx forward, yaw_rate left-positive);
  otherwise perform the requested behaviour (default: move forward at a moderate speed).
- Wheels/tracks: differential drive  v_wheel = (vx ∓ yaw_rate * half_track) / r * forward_sign  (minus on the
  right side). Hold heading with feedback on base yaw / ang_vel[2] when driving straight.
- Legged: a foot path (forward while lifted, backward while planted) mapped to joint angles with the brief's
  per-joint foot motions. Use feedback: base pitch/roll to shift feet or bias hips, contacts to time phases.
  Bipeds cannot walk open-loop — they need balance feedback (and standing still is an acceptable fallback).
- Arms: smooth joint-space trajectories (cosine blends) between poses inside the limits; slow near the
  gravity-loaded joints. Watch the effort report: a joint at its torque limit can't track.
- Keep amplitudes inside limits and speeds within the brief's maximums.

# Process
You are given the robot brief, a baseline controller and the baseline's test report. Improve on it for the
user's request. Test every candidate with test_controller and read the report critically: did it move the
requested way and distance, stay upright (tilt), go straight (heading), keep feet/wheels on the ground,
track its targets without saturating? Fix what the numbers show is wrong, then test again. You have at most
6 tests. Submit (submit_controller) the best version you actually tested, with an honest one-line measured
summary (e.g. "trots forward at 9 cm/s, heading drift 3°"). If the request is impossible for this robot,
submit the closest safe behaviour and say so."""


def _get_client():
    from core.ai.client import get_client as get
    return get()


def generate_controller(prompt: str, urdf_text: str, current_script: str = "",
                        terrain_config: Optional[Dict[str, Any]] = None,
                        progress: Optional[Callable[[str, str], None]] = None,
                        max_tests: int = MAX_TESTS) -> Dict[str, Any]:
    """Returns {code, summary, report, tests}."""
    def emit(msg: str) -> None:
        if progress:
            progress("status", msg)

    emit("Measuring the robot...")
    brief = robot_brief_data(urdf_text, free_base=False)
    brief_text = format_robot_brief(brief)
    base_code = current_script.strip() or baseline_controller(brief)
    base_eval = evaluate_controller(urdf_text, base_code, terrain_config=terrain_config)
    base_report = format_evaluation(base_eval)

    request = prompt.strip() or ("Make it move in the way this robot is built for — drive or walk forward "
                                 "steadily for a mobile robot, a smooth demonstration motion for an arm.")
    terrain = (terrain_config or {}).get("type", "flat")
    user = (f"{brief_text}\n\nTERRAIN: {terrain}\n\n"
            f"{'CURRENT' if current_script.strip() else 'BASELINE'} CONTROLLER:\n```python\n{base_code}\n```\n\n"
            f"ITS TEST REPORT:\n{base_report}\n\nREQUEST: {request}")

    client = _get_client()
    system = [{"type": "text", "text": SYSTEM_PROMPT, "cache_control": {"type": "ephemeral"}}]
    messages: List[Dict[str, Any]] = [{"role": "user", "content": user}]
    tested: Dict[str, Dict[str, Any]] = {base_code.strip(): base_eval}
    tests = 0
    final: Optional[Dict[str, Any]] = None
    for rnd in range(1, max_tests + 3):
        emit("Writing controller..." if rnd == 1 else f"Revising controller (test {tests} done)...")
        t0 = time.time()
        with client.messages.stream(
            model=CONTROLLER_MODEL, max_tokens=32000, system=system,
            tools=[TEST_TOOL, SUBMIT_TOOL], messages=messages,
            output_config={"effort": CONTROLLER_EFFORT},
        ) as stream:
            resp = stream.get_final_message()
        print(f"[controller] turn {rnd}: {time.time() - t0:.1f}s stop={resp.stop_reason} "
              f"out={resp.usage.output_tokens}", file=sys.stderr)
        if resp.stop_reason == "refusal":
            raise RuntimeError("The controller request was declined.")
        messages.append({"role": "assistant", "content": resp.content})
        calls = [b for b in resp.content if b.type == "tool_use"]
        if not calls:
            break
        results = []
        for call in calls:
            args = call.input if isinstance(call.input, dict) else {}
            code = str(args.get("code", "")).strip()
            if call.name == "submit_controller":
                try:
                    compile_script(code)
                except Exception as e:  # noqa: BLE001
                    results.append({"type": "tool_result", "tool_use_id": call.id, "is_error": True,
                                    "content": f"Not a valid controller: {e}"})
                    continue
                ev = tested.get(code) or evaluate_controller(urdf_text, code, terrain_config=terrain_config)
                final = {"code": code, "summary": str(args.get("summary", "")).strip(), "evaluation": ev}
                results.append({"type": "tool_result", "tool_use_id": call.id, "content": "Submitted."})
                continue
            if tests >= max_tests:
                results.append({"type": "tool_result", "tool_use_id": call.id, "is_error": True,
                                "content": "Test budget used up. Submit the best version you tested."})
                continue
            tests += 1
            secs = max(1.0, min(12.0, float(args.get("seconds") or 6.0)))
            cmd = args.get("cmd") if isinstance(args.get("cmd"), dict) else None
            emit(f"Testing controller in simulation ({tests}/{max_tests})...")
            ev = evaluate_controller(urdf_text, code, seconds=secs, terrain_config=terrain_config, cmd=cmd)
            if not cmd:
                tested[code] = ev
            rep = format_evaluation(ev)
            if progress and ev.get("ok") and ev.get("free_base"):
                emit(f"Test {tests}: moved {ev['forward_m'] * 100:+.0f} cm, tilt {ev['max_tilt_deg']:.0f}°"
                     + (", fell" if ev.get("fell_at_s") is not None else ""))
            results.append({"type": "tool_result", "tool_use_id": call.id, "content": rep})
        messages.append({"role": "user", "content": results})
        if final:
            break

    if final is None:
        # No submission: fall back to the best tested code (or the baseline).
        def score(ev):
            if not ev.get("ok"):
                return -1e9
            s = ev.get("forward_m", 0.0) if ev.get("free_base") else 0.0
            return s - (5.0 if ev.get("fell_at_s") is not None else 0.0)
        code, ev = max(tested.items(), key=lambda kv: score(kv[1]))
        final = {"code": code, "summary": "Best tested controller (the agent did not submit one).", "evaluation": ev}
    final["report"] = format_evaluation(final["evaluation"])
    final["tests"] = tests
    return final
