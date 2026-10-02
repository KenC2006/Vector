"""Forward kinematics of a URDF at zero joint state (link frames in world, mm)."""
from __future__ import annotations

from typing import Dict, Tuple
import xml.etree.ElementTree as ET

import numpy as np

from core.designer.geometry import rpy_to_matrix


def link_frames(urdf: str) -> Dict[str, Tuple[np.ndarray, np.ndarray]]:
    root = ET.fromstring(urdf)
    joints = {}
    children = set()
    for j in root.findall("joint"):
        par, ch = j.find("parent").get("link"), j.find("child").get("link")
        o = j.find("origin")
        xyz = np.array([float(v) for v in (o.get("xyz", "0 0 0") if o is not None else "0 0 0").split()])
        rpy = [float(v) for v in (o.get("rpy", "0 0 0") if o is not None else "0 0 0").split()]
        joints.setdefault(par, []).append((ch, xyz * 1000.0, rpy_to_matrix(rpy)))
        children.add(ch)
    links = [l.get("name") for l in root.findall("link")]
    base = next(l for l in links if l not in children)
    out = {base: (np.zeros(3), np.eye(3))}
    stack = [base]
    while stack:
        p = stack.pop()
        pp, pR = out[p]
        for ch, xyz, R in joints.get(p, []):
            out[ch] = (pp + pR @ xyz, pR @ R)
            stack.append(ch)
    return out
