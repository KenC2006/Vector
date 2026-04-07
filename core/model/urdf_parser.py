"""
URDF parser using yourdfpy to build a kinematic graph.
"""
from typing import Optional, Dict, Tuple, Any
import os
from .kinematic_graph import KinematicGraph
from .types import LinkData, JointData, Inertia, Limits


def _extract_inertia(inertia_elem: Any) -> Optional[Inertia]:
    """Extract inertia from a yourdfpy inertia element (3x3 matrix)."""
    try:
        if inertia_elem is None:
            return None
        # yourdfpy represents inertia as a 3x3 numpy array
        # Convert matrix to URDF format: ixx, ixy, ixz, iyy, iyz, izz
        inertia_matrix = inertia_elem
        return Inertia(
            ixx=float(inertia_matrix[0, 0]),
            ixy=float(inertia_matrix[0, 1]),
            ixz=float(inertia_matrix[0, 2]),
            iyy=float(inertia_matrix[1, 1]),
            iyz=float(inertia_matrix[1, 2]),
            izz=float(inertia_matrix[2, 2]),
        )
    except (AttributeError, ValueError, TypeError, IndexError):
        return None


def _extract_geometry(geom_elem: Any) -> Optional[Dict[str, Any]]:
    """Extract geometry from a yourdfpy geometry element."""
    try:
        if geom_elem is None:
            return None

        # Check for box
        if hasattr(geom_elem, "box") and geom_elem.box is not None:
            size = getattr(geom_elem.box, "size", [1, 1, 1])
            try:
                size_list = list(size) if hasattr(size, "__iter__") else [1, 1, 1]
            except (TypeError, ValueError):
                size_list = [1, 1, 1]
            return {
                "type": "box",
                "params": {
                    "size": size_list
                }
            }

        # Check for cylinder
        if hasattr(geom_elem, "cylinder") and geom_elem.cylinder is not None:
            cylinder = geom_elem.cylinder
            radius = float(getattr(cylinder, "radius", 0.1))
            length = float(getattr(cylinder, "length", 1.0))
            return {
                "type": "cylinder",
                "params": {
                    "radius": radius,
                    "length": length,
                }
            }

        # Check for sphere
        if hasattr(geom_elem, "sphere") and geom_elem.sphere is not None:
            sphere = geom_elem.sphere
            radius = float(getattr(sphere, "radius", 0.1))
            return {
                "type": "sphere",
                "params": {
                    "radius": radius,
                }
            }

        # Check for mesh
        if hasattr(geom_elem, "mesh") and geom_elem.mesh is not None:
            filename = getattr(geom_elem.mesh, "filename", "")
            return {
                "type": "mesh",
                "params": {
                    "filename": str(filename),
                }
            }

        return None
    except (AttributeError, ValueError, TypeError):
        return None


def _extract_mesh_path(visual_elem: Any) -> Optional[str]:
    """Extract mesh filename from a visual element."""
    try:
        if visual_elem is None:
            return None
        if hasattr(visual_elem, "geometry") and visual_elem.geometry is not None:
            geom = visual_elem.geometry
            if hasattr(geom, "mesh") and geom.mesh is not None:
                return str(getattr(geom.mesh, "filename", ""))
        return None
    except (AttributeError, ValueError, TypeError):
        return None


def _extract_origin(origin_elem: Any) -> Tuple[Tuple[float, float, float], Tuple[float, float, float]]:
    """
    Extract origin (position and rotation) from an element.
    Returns (xyz, rpy) tuples.
    """
    try:
        xyz = (0.0, 0.0, 0.0)
        rpy = (0.0, 0.0, 0.0)

        if origin_elem is not None:
            xyz_val = getattr(origin_elem, "xyz", None)
            if xyz_val is not None:
                xyz = tuple(float(x) for x in xyz_val)

            rpy_val = getattr(origin_elem, "rpy", None)
            if rpy_val is not None:
                rpy = tuple(float(x) for x in rpy_val)

        return xyz, rpy
    except (AttributeError, ValueError, TypeError):
        return (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)


def parse_urdf(file_path: str) -> KinematicGraph:
    """
    Parse a URDF file and build a kinematic graph.

    Args:
        file_path: Path to the URDF file.

    Returns:
        KinematicGraph: The parsed robot model.

    Raises:
        FileNotFoundError: If the file doesn't exist.
        ValueError: If the URDF is invalid.
    """
    if not os.path.exists(file_path):
        raise FileNotFoundError(f"URDF file not found: {file_path}")

    try:
        import yourdfpy
    except ImportError:
        raise ImportError("yourdfpy not installed. Run: pip install yourdfpy")

    # Parse the URDF
    try:
        urdf = yourdfpy.URDF.load(file_path)
    except Exception as e:
        raise ValueError(f"Failed to parse URDF: {e}")

    # Create kinematic graph
    kg = KinematicGraph()

    # Add all links
    if not urdf.link_map:
        raise ValueError("URDF has no links")

    for link_name, link in urdf.link_map.items():
        # Extract inertia
        inertia = None
        mass = 0.0
        if link.inertial is not None:
            inertial = link.inertial
            mass = float(getattr(inertial, "mass", 0.0))
            inertia = _extract_inertia(getattr(inertial, "inertia", None))

        # Extract collision geometry (prefer collision, fall back to visual)
        collision_geom = None
        if link.collisions:
            # link.collisions is a list
            if len(link.collisions) > 0:
                collision_geom = _extract_geometry(link.collisions[0].geometry)

        # Extract visual mesh path
        visual_mesh = None
        if link.visuals:
            if len(link.visuals) > 0:
                visual_mesh = _extract_mesh_path(link.visuals[0])

        link_data = LinkData(
            name=link.name,
            mass=mass,
            inertia=inertia,
            visual_mesh=visual_mesh,
            collision_geometry=collision_geom,
        )
        kg.add_link(link_data)

    # Set root link (base link)
    if urdf.base_link:
        # urdf.base_link is a string (link name)
        kg.set_root_link(urdf.base_link)
    elif urdf.link_map:
        kg.set_root_link(list(urdf.link_map.keys())[0])

    # Add all joints
    if not urdf.joint_map:
        # No joints is valid (single-link robot)
        return kg

    for joint_name, joint in urdf.joint_map.items():
        joint_type = str(joint.type)
        parent_name = str(joint.parent)
        child_name = str(joint.child)

        if not parent_name or not child_name:
            continue

        # Extract axis (yourdfpy provides axis as numpy array)
        axis = (0.0, 0.0, 1.0)
        if joint.axis is not None:
            try:
                axis = tuple(float(x) for x in joint.axis)
            except (ValueError, TypeError):
                axis = (0.0, 0.0, 1.0)

        # Extract origin from transformation matrix
        origin_xyz = (0.0, 0.0, 0.0)
        origin_rpy = (0.0, 0.0, 0.0)
        if joint.origin is not None:
            try:
                # origin is a 4x4 transformation matrix
                origin_xyz = tuple(float(x) for x in joint.origin[:3, 3])
                # For RPY, we'd need to decompose the rotation matrix
                # For simplicity, set to zero (could be enhanced)
                origin_rpy = (0.0, 0.0, 0.0)
            except (ValueError, TypeError, IndexError):
                pass

        # Extract limits
        limits = None
        if joint.limit is not None:
            limit_obj = joint.limit
            limits = Limits(
                lower=float(limit_obj.lower) if hasattr(limit_obj, "lower") else 0.0,
                upper=float(limit_obj.upper) if hasattr(limit_obj, "upper") else 0.0,
                effort=float(limit_obj.effort) if hasattr(limit_obj, "effort") and limit_obj.effort else None,
                velocity=float(limit_obj.velocity) if hasattr(limit_obj, "velocity") and limit_obj.velocity else None,
            )

        # Extract dynamics
        dynamics = None
        if joint.dynamics is not None:
            dyn_obj = joint.dynamics
            dynamics = {
                "damping": float(dyn_obj.damping) if hasattr(dyn_obj, "damping") and dyn_obj.damping else 0.0,
                "friction": float(dyn_obj.friction) if hasattr(dyn_obj, "friction") and dyn_obj.friction else 0.0,
            }

        joint_data = JointData(
            name=joint.name,
            joint_type=joint_type,
            parent_link=parent_name,
            child_link=child_name,
            axis=axis,
            origin_xyz=origin_xyz,
            origin_rpy=origin_rpy,
            limits=limits,
            dynamics=dynamics,
        )
        kg.add_joint(joint_data)

    return kg
