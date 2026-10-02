"""Small rigid-transform helpers (numpy). Units: millimetres, radians."""
from __future__ import annotations

import math
import re
from typing import Sequence

import numpy as np

AXIS_WORDS = {
    "+x": (1, 0, 0), "-x": (-1, 0, 0), "x": (1, 0, 0),
    "+y": (0, 1, 0), "-y": (0, -1, 0), "y": (0, 1, 0),
    "+z": (0, 0, 1), "-z": (0, 0, -1), "z": (0, 0, 1),
    "forward": (1, 0, 0), "back": (-1, 0, 0), "backward": (-1, 0, 0),
    "left": (0, 1, 0), "right": (0, -1, 0), "up": (0, 0, 1), "down": (0, 0, -1),
}

# Mirror across the robot's sagittal (XZ) plane.
S = np.diag([1.0, -1.0, 1.0])


def vec(v, what: str = "vector") -> np.ndarray:
    """Parse a direction/position given as a word ('+y', 'up') or 3 numbers."""
    if isinstance(v, str):
        key = v.strip().lower()
        if key in AXIS_WORDS:
            return np.array(AXIS_WORDS[key], dtype=float)
        raise ValueError(f"unknown {what} '{v}' (use +x/-x/+y/-y/+z/-z or [x,y,z])")
    arr = np.asarray(v, dtype=float).reshape(-1)
    if arr.shape != (3,) or not np.all(np.isfinite(arr)):
        raise ValueError(f"{what} must have 3 finite numbers, got {v!r}")
    return arr


def unit(v: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(v)
    if n < 1e-9:
        raise ValueError("zero-length direction")
    return v / n


def frame_from_axes(z_axis=None, x_axis=None) -> np.ndarray:
    """Rotation whose local +Z/+X point along the given world directions.
    Missing axes default to world +Z / +X; x is re-orthogonalised against z."""
    z = unit(vec(z_axis, "z_axis")) if z_axis is not None else np.array([0.0, 0, 1])
    if x_axis is not None:
        x = unit(vec(x_axis, "x_axis"))
    else:
        x = np.array([1.0, 0, 0]) if abs(z[0]) < 0.9 else np.array([0.0, 0, 1]) * -np.sign(z[0])
    x = x - z * float(x @ z)
    if np.linalg.norm(x) < 1e-6:
        # x parallel to z: pick any perpendicular
        x = np.cross(z, [0, 1, 0]) if abs(z[1]) < 0.9 else np.cross(z, [1, 0, 0])
    x = unit(x)
    y = np.cross(z, x)
    return np.column_stack([x, y, z])


def rot_axis_angle(axis: np.ndarray, angle: float) -> np.ndarray:
    a = unit(axis)
    k = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(angle) * k + (1 - math.cos(angle)) * (k @ k)


def rpy_to_matrix(rpy: Sequence[float]) -> np.ndarray:
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def matrix_to_rpy(R: np.ndarray) -> tuple[float, float, float]:
    sp = -R[2, 0]
    if abs(sp) > 1 - 1e-9:
        p = math.copysign(math.pi / 2, sp)
        r = 0.0
        y = math.atan2(-R[0, 1], R[1, 1])
    else:
        p = math.asin(sp)
        r = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(R[1, 0], R[0, 0])
    return (r, p, y)


def rotation_between(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Smallest rotation taking unit vector a onto unit vector b."""
    a, b = unit(a), unit(b)
    c = float(a @ b)
    if c > 1 - 1e-12:
        return np.eye(3)
    if c < -1 + 1e-12:
        perp = np.cross(a, [1, 0, 0])
        if np.linalg.norm(perp) < 1e-6:
            perp = np.cross(a, [0, 1, 0])
        return rot_axis_angle(perp, math.pi)
    return rot_axis_angle(np.cross(a, b), math.acos(c))


_MIRROR_TOKENS = {
    "left": "right", "l": "r", "fl": "fr", "rl": "rr", "ml": "mr", "bl": "br",
    "lf": "rf", "lr": "rr", "lh": "rh",
}
_MIRROR_TOKENS.update({v: k for k, v in list(_MIRROR_TOKENS.items())})
_MIRROR_TOKENS["rr"] = "rl"   # rear-right <-> rear-left (lr/rr pairing is rarer)


def mirror_name(name: str) -> str:
    """left<->right swap for the mirrored copy of a part, token by token
    (`_`-separated), so `rocker_front_l` becomes `rocker_front_r` rather than
    having the `fr` inside `front` rewritten."""
    tokens = name.split("_")
    for i, t in enumerate(tokens):
        low = t.lower()
        if low in _MIRROR_TOKENS:
            swap = _MIRROR_TOKENS[low]
            tokens[i] = swap.capitalize() if t[:1].isupper() else swap
            return "_".join(tokens)
        for a, b in (("Left", "Right"), ("Right", "Left"), ("left", "right"), ("right", "left")):
            if a in t:
                tokens[i] = t.replace(a, b)
                return "_".join(tokens)
    return name + "_mirror"
