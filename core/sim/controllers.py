"""
Baseline controllers built from the measured robot brief (sim/control.py).

No joint names, chain depths or naming conventions are guessed: every sign and
gain comes from measuring what a joint's positive direction does to the robot
in the zero pose. The result is a readable, sandbox-safe script that

  * drives wheels/tracks as differential (skid) steering,
  * walks legged robots with a trot / tripod / alternating gait whose foot
    path is mapped to joint angles through the leg's measured Jacobian,
  * holds every other motor at its zero pose,
  * follows the operator's velocity command (``state["cmd"]``) when active,
    otherwise moves forward at a moderate default speed.

It is the instant default and the starting point the controller agent tests
and improves on.
"""
from __future__ import annotations

import math
from typing import Any, Dict, List, Optional

G = 9.81


def _leg_plan(brief: Dict[str, Any]) -> Optional[Dict[str, Any]]:
    groups = brief.get("leg_groups") or []
    if len(groups) < 2:
        return None
    by_name = {j["name"]: j for j in brief["joints"]}
    legs = []
    for g in groups:
        members = [by_name[n] for n in g["joints"] if n in by_name and by_name[n]["type"] == "revolute"]
        if not members:
            continue
        # Pick the two joints whose foot motions best span forward (x) and up (z).
        best, best_det = None, 0.0
        for a in members:
            for b in members:
                if a is b:
                    continue
                ea, eb = a["plus_moves_probe"], b["plus_moves_probe"]
                det = ea[0] * eb[2] - eb[0] * ea[2]
                if abs(det) > abs(best_det):
                    best, best_det = (a, b), det
        root = by_name[g["root_joint"]]
        leg_len = max(0.03, root["pivot"][2] - g["foot"][2])
        if best is None or abs(best_det) < 1e-4:
            # One sagittal joint only: swing it, no lift.
            a = max(members, key=lambda j: abs(j["plus_moves_probe"][0]))
            ex = a["plus_moves_probe"][0]
            if abs(ex) < 1e-4:
                continue
            legs.append({"foot": g["foot"], "len": leg_len, "hold": [m["name"] for m in members if m is not a],
                         "j": [a["name"]], "inv": [[1.0 / ex, 0.0]], "lim": [a["limits"]]})
            continue
        a, b = best
        ea, eb = a["plus_moves_probe"], b["plus_moves_probe"]
        # [dx, dz] = J [qa, qb]  ->  [qa, qb] = J^-1 [dx, dz]
        inv = [[eb[2] / best_det, -eb[0] / best_det],
               [-ea[2] / best_det, ea[0] / best_det]]
        legs.append({"foot": g["foot"], "len": leg_len,
                     "hold": [m["name"] for m in members if m is not a and m is not b],
                     "j": [a["name"], b["name"]], "inv": inv, "lim": [a["limits"], b["limits"]]})
    if len(legs) < 4:
        # Two legs can't walk open-loop (it falls within a second); a biped's
        # default is to stand. Walking needs feedback — the controller agent's job.
        return None
    # Phases: along each side, alternate front-to-back, and the right side is
    # offset by half a cycle -> diagonal trot (4), tripod (6), alternate (2).
    for side, off in (("left", 0), ("right", 1)):
        mine = sorted([l for l in legs if (l["foot"][1] > 0) == (side == "left")], key=lambda l: -l["foot"][0])
        for i, l in enumerate(mine):
            l["phase"] = math.pi * ((i + off) % 2)
            l["side"] = 1.0 if side == "left" else -1.0
    L = sum(l["len"] for l in legs) / len(legs)
    freq = max(0.8, min(2.5, math.sqrt(G / L) / (2 * math.pi)))
    stride, lift = 0.35 * L, 0.12 * L
    # The foot->joint map is linearised at the zero pose, so it only holds for
    # small joint motions; a near-straight leg (both joints moving the foot the
    # same way) would demand huge angles. Shrink each leg's motion until no joint
    # travels more than MAX_Q from its pose.
    MAX_Q = 0.35
    for l in legs:
        peak = 0.0
        for row in l["inv"]:
            peak = max(peak, abs(row[0]) * stride / 2 + abs(row[1]) * lift)
        l["k"] = min(1.0, MAX_Q / peak) if peak > 1e-9 else 1.0
    return {"legs": legs, "freq": freq, "stride": stride, "lift": lift, "leg_len": L}


def _drive_plan(brief: Dict[str, Any]) -> Optional[Dict[str, Any]]:
    wheels = [j for j in brief["joints"] if j.get("is_wheel") and "plus_drives_robot" in j]
    if not wheels:
        return None
    rows = []
    for w in wheels:
        dx = w["plus_drives_robot"][0]
        if abs(dx) < 0.5:
            continue  # sideways-mounted roller (e.g. omni): not part of straight drive
        rows.append({"joint": w["name"], "sign": 1.0 if dx > 0 else -1.0, "r": w["wheel_radius"],
                     "y": w["pos"][1], "vmax": w["max_velocity"] or 10.0})
    if not rows:
        return None
    half_track = max(0.02, sum(abs(r["y"]) for r in rows) / len(rows))
    vmax_lin = min(r["vmax"] * r["r"] for r in rows)
    return {"wheels": rows, "half_track": half_track, "vmax": vmax_lin}


def _steer_joints(brief: Dict[str, Any], wheel_names: List[str]) -> List[str]:
    """Revolute joints about a vertical axis whose subtree contains a drive wheel."""
    out = []
    by = {j["child"]: j for j in brief["joints"]}
    for j in brief["joints"]:
        if j["type"] != "revolute":
            continue
        ax = j["axis_world"]
        if abs(ax[2]) < 0.9:
            continue
        subtree_wheels = [w for w in brief["joints"] if w["name"] in wheel_names and w["child"] in j["moves"]]
        if subtree_wheels:
            out.append(j["name"])
    return out


def baseline_controller(brief: Dict[str, Any]) -> str:
    """Return a sandbox-safe step(t, state) script for the robot."""
    drive = _drive_plan(brief)
    legs = _leg_plan(brief) if not drive else None
    motors = [j for j in brief["joints"] if j["type"] in ("revolute", "prismatic", "continuous")]
    used = set()
    L: List[str] = ["# Baseline controller generated from the measured robot brief.",
                    "# Follows state['cmd'] (vx m/s, yaw_rate rad/s) when the operator drives;",
                    "# otherwise moves forward at DEFAULT_VX.", ""]
    body: List[str] = []
    if drive:
        steer = _steer_joints(brief, [w["joint"] for w in drive["wheels"]])
        used |= {w["joint"] for w in drive["wheels"]} | set(steer)
        L += [f"WHEELS = {[{k: (round(v, 4) if isinstance(v, float) else v) for k, v in w.items()} for w in drive['wheels']]!r}",
              f"HALF_TRACK = {drive['half_track']:.4f}   # m, centre to wheel",
              f"MAX_SPEED = {drive['vmax']:.3f}   # m/s at full wheel speed",
              f"DEFAULT_VX = {0.5 * drive['vmax']:.3f}",
              f"STEER = {steer!r}",
              "RAMP = 0.6   # s", ""]
        body += ["    vx, wz = command(state, DEFAULT_VX)",
                 "    ramp = min(1.0, t / RAMP)",
                 "    for w in WHEELS:",
                 "        # + yaw_rate turns left: the right side runs faster than the left",
                 "        v = vx + wz * HALF_TRACK if w['y'] < 0 else vx - wz * HALF_TRACK",
                 "        out[w['joint']] = ramp * w['sign'] * v / w['r']",
                 "    for name in STEER:",
                 "        out[name] = 0.0"]
    elif legs:
        rows = []
        for l in legs["legs"]:
            used |= set(l["j"])
            rows.append({"joints": l["j"], "inv": [[round(v, 4) for v in r] for r in l["inv"]],
                         "lim": l["lim"], "phase": round(l["phase"], 4), "y": round(l["foot"][1], 4),
                         "k": round(l["k"], 3)})
        # Legs whose motion was capped cover less ground per step.
        k_min = min(l["k"] for l in legs["legs"])
        speed = k_min * legs["stride"] * legs["freq"]
        L += [f"LEGS = {rows!r}",
              f"FREQ = {legs['freq']:.3f}   # Hz, from leg-length pendulum rate",
              f"STRIDE = {legs['stride']:.4f}   # m foot sweep per step at full speed",
              f"LIFT = {legs['lift']:.4f}   # m foot clearance in swing",
              f"FULL_SPEED = {speed:.3f}   # m/s the stride gives at FREQ",
              f"DEFAULT_VX = {0.6 * speed:.3f}",
              "RAMP = 1.0   # s", ""]
        body += ["    vx, wz = command(state, DEFAULT_VX)",
                 "    ramp = min(1.0, t / RAMP)",
                 "    w = 2.0 * math.pi * FREQ",
                 "    for leg in LEGS:",
                 "        # each foot strides at the ground speed its position needs: + wz = left turn",
                 "        scale = (vx - wz * leg['y']) / max(FULL_SPEED, 1e-6)",
                 "        scale = max(-1.0, min(1.0, scale))",
                 "        ph = w * t + leg['phase']",
                 "        # foot path: forward while cos>0 (lifted), back while cos<0 (planted)",
                 "        dx = ramp * scale * 0.5 * STRIDE * math.sin(ph)",
                 "        dz = ramp * min(1.0, abs(scale) * 3.0) * LIFT * max(0.0, math.cos(ph) if scale >= 0 else -math.cos(ph))",
                 "        for k, name in enumerate(leg['joints']):",
                 "            row = leg['inv'][k]",
                 "            q = leg['k'] * (row[0] * dx + row[1] * dz)",
                 "            lim = leg['lim'][k]",
                 "            if lim:",
                 "                q = max(lim[0], min(lim[1], q))",
                 "            out[name] = q"]
    hold = [j["name"] for j in motors if j["name"] not in used and j["type"] != "continuous"]
    spin = [j["name"] for j in motors if j["name"] not in used and j["type"] == "continuous"]
    L += [f"HOLD = {hold!r}   # held at the zero (as-designed) pose",
          f"STOP = {spin!r}   # speed-controlled joints kept still", "",
          "def command(state, default_vx):",
          "    cmd = state.get('cmd') or {}",
          "    if cmd.get('active'):",
          "        return cmd.get('vx', 0.0), cmd.get('yaw_rate', 0.0)",
          "    return default_vx, 0.0", "",
          "def step(t, state):",
          "    out = {}",
          "    for name in HOLD:",
          "        out[name] = 0.0",
          "    for name in STOP:",
          "        out[name] = 0.0"]
    L += body
    L += ["    return out", ""]
    return "\n".join(L)
