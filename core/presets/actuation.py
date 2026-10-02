"""
Actuator ratings from the catalog.

Every actuator / motor / drivetrain part carries an ``actuation`` block:

    {"kind": "rotary" | "linear",
     "rated_effort": N·m (rotary) or N (linear),
     "rated_speed":  rad/s (rotary) or m/s (linear),
     "stroke_mm":    travel (linear only),
     "speed_assumed": true   # only when the datasheet gives no speed}

It is derived from the part's ``mechanical_electrical`` datasheet fields by
:func:`derive_actuation` (scripts/add_catalog_sim_fields.py writes it into the
catalog) and read back by :func:`actuator_rating`, which every consumer
(designer joint ratings, critic load check, MJCF actuators) uses instead of
picking torque fields itself.
"""
from __future__ import annotations

import math
from typing import Any, Dict, Optional, Tuple

RPM = 2.0 * math.pi / 60.0

# Torque fields in precedence order: the continuous rating a datasheet gives
# (servo/BLDC max torque, gearmotor rated output torque, stepper holding
# torque) before stall / peak torque, which a motor only reaches momentarily.
_TORQUE_FIELDS = ("max_torque_nm", "output_torque_nm", "holding_torque_nm",
                  "stall_torque_nm", "peak_torque_nm", "drive_torque_nm")
_RPM_FIELDS = ("no_load_speed_rpm", "output_speed_rpm", "no_load_rpm", "max_speed_rpm", "speed_rpm")

# Datasheets without a speed: a conservative usable speed (steppers lose
# torque fast above a few hundred rpm; hub motors spec torque only).
_ASSUMED_RPM = 120.0

_POWERED_TYPES = frozenset(("position", "velocity", "motor"))


def _num(me: Dict[str, Any], key: str) -> Optional[float]:
    v = me.get(key)
    if isinstance(v, bool) or not isinstance(v, (int, float)):
        return None
    v = float(v)
    return v if math.isfinite(v) and v > 0 else None


def _first(me: Dict[str, Any], keys) -> Optional[float]:
    for k in keys:
        v = _num(me, k)
        if v is not None:
            return v
    return None


def derive_actuation(component: Dict[str, Any]) -> Optional[Dict[str, Any]]:
    """Actuation block from datasheet fields, or None for unpowered parts."""
    sim = component.get("sim_metadata") or {}
    me = component.get("mechanical_electrical") or {}
    # Only powered parts: a joint actuator type, or a drive module rating.
    # Clamps, couplings and gears also list torque fields (what they can
    # transmit) but move nothing; stub axles / swivels are explicitly None;
    # suction and magnets ("adhesion") hold rather than move a joint.
    if sim.get("mjcf_actuator_type") not in _POWERED_TYPES and _num(me, "drive_torque_nm") is None:
        return None

    force = _num(me, "max_force_n")
    stroke = _num(me, "stroke_mm")
    if force is not None and stroke is not None:
        speed = _num(me, "speed_mm_per_s")
        out = {"kind": "linear", "rated_effort": force,
               "rated_speed": round(speed / 1000.0, 6) if speed else 0.01, "stroke_mm": stroke}
        if not speed:
            out["speed_assumed"] = True
        return out

    # Parallel grippers: a linear finger stage rated by grip force.
    grip = _num(me, "grip_force_n")
    finger_stroke = _num(me, "stroke_per_finger_mm")
    if grip is not None and finger_stroke is not None and sim.get("joint_type") == "slide":
        close = _num(me, "close_time_s")
        out = {"kind": "linear", "rated_effort": grip,
               "rated_speed": round(finger_stroke / 1000.0 / close, 6) if close else 0.05,
               "stroke_mm": finger_stroke}
        if not close:
            out["speed_assumed"] = True
        return out

    torque = _first(me, _TORQUE_FIELDS)
    if torque is None:
        # Brushless outrunners/inrunners spec Kv and current, not torque:
        # Kt = 60 / (2π·Kv) N·m/A.
        kv = _num(me, "kv_rpm_per_v")
        amps = _first(me, ("max_continuous_current_a", "continuous_current_a"))
        if kv and amps:
            torque = 60.0 / (2.0 * math.pi * kv) * amps
    if torque is None:
        return None

    rpm = _first(me, _RPM_FIELDS)
    speed = rpm * RPM if rpm else None
    if speed is None:
        t60 = _num(me, "transit_time_60deg_s")
        if t60:
            speed = (math.pi / 3.0) / t60
    out = {"kind": "rotary", "rated_effort": round(torque, 6),
           "rated_speed": round(speed if speed else _ASSUMED_RPM * RPM, 6)}
    if speed is None:
        out["speed_assumed"] = True
    return out


def actuator_rating(component: Optional[Dict[str, Any]]) -> Optional[Tuple[str, float, float]]:
    """(kind, rated_effort, rated_speed) of a catalog part, or None if it is not an actuator.

    kind is "rotary" (effort N·m, speed rad/s) or "linear" (effort N, speed m/s).
    Reads the catalog ``actuation`` block; falls back to deriving it from the
    datasheet fields for parts not yet annotated.
    """
    if not component:
        return None
    act = component.get("actuation")
    if not isinstance(act, dict):
        act = derive_actuation(component)
    if not act:
        return None
    try:
        kind = str(act["kind"])
        effort = float(act["rated_effort"])
        speed = float(act["rated_speed"])
    except (KeyError, TypeError, ValueError):
        return None
    if kind not in ("rotary", "linear") or not (effort > 0 and speed > 0):
        return None
    return kind, effort, speed
