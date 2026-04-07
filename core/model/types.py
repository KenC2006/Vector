"""
Data structures for kinematic graph representation.
"""
from dataclasses import dataclass, field, asdict
from typing import Dict, List, Optional, Tuple
import json


@dataclass
class Inertia:
    """Represents 3D rotational inertia (6 values: ixx, ixy, ixz, iyy, iyz, izz)."""
    ixx: float = 0.0
    ixy: float = 0.0
    ixz: float = 0.0
    iyy: float = 0.0
    iyz: float = 0.0
    izz: float = 0.0

    def to_dict(self) -> Dict:
        return asdict(self)


@dataclass
class LinkData:
    """Represents a link in the kinematic graph."""
    name: str
    mass: float = 0.0
    inertia: Optional[Inertia] = None
    visual_mesh: Optional[str] = None  # Path to visual mesh (STL/DAE/etc)
    collision_geometry: Optional[Dict] = None  # Geometry dict: {"type": "box"|"cylinder"|"sphere", "params": {...}}

    def to_dict(self) -> Dict:
        data = {
            "name": self.name,
            "mass": self.mass,
            "inertia": self.inertia.to_dict() if self.inertia else None,
            "visual_mesh": self.visual_mesh,
            "collision_geometry": self.collision_geometry,
        }
        return data


@dataclass
class Limits:
    """Joint limit specification."""
    lower: float = 0.0
    upper: float = 0.0
    effort: Optional[float] = None
    velocity: Optional[float] = None

    def to_dict(self) -> Dict:
        return asdict(self)


@dataclass
class JointData:
    """Represents a joint in the kinematic graph."""
    name: str
    joint_type: str  # "revolute", "prismatic", "fixed", "floating", "planar"
    parent_link: str
    child_link: str
    axis: Tuple[float, float, float] = (0.0, 0.0, 1.0)  # x, y, z rotation/translation axis
    origin_xyz: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    origin_rpy: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    limits: Optional[Limits] = None
    dynamics: Optional[Dict] = None  # {"damping": float, "friction": float}

    def to_dict(self) -> Dict:
        data = {
            "name": self.name,
            "joint_type": self.joint_type,
            "parent_link": self.parent_link,
            "child_link": self.child_link,
            "axis": list(self.axis),
            "origin_xyz": list(self.origin_xyz),
            "origin_rpy": list(self.origin_rpy),
            "limits": self.limits.to_dict() if self.limits else None,
            "dynamics": self.dynamics,
        }
        return data
