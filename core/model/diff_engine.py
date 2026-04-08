"""
Kinematic graph diff engine: compute and apply structured diffs between graphs.
"""
from typing import Dict, List, Any
from .kinematic_graph import KinematicGraph
from .types import LinkData, JointData


def compute_diff(old_kg: KinematicGraph, new_kg: KinematicGraph) -> Dict[str, Any]:
    """
    Compute a structured diff between two kinematic graphs.

    Args:
        old_kg: The original kinematic graph.
        new_kg: The modified kinematic graph.

    Returns:
        A dict with keys:
        - "added_links": list of LinkData dicts for new links
        - "removed_links": list of link names that were removed
        - "modified_links": list of dicts with "name" and "changes" (field: {"old": val, "new": val})
        - "added_joints": list of JointData dicts for new joints
        - "removed_joints": list of joint names that were removed
        - "modified_joints": list of dicts with "name" and "changes"
    """
    diff = {
        "added_links": [],
        "removed_links": [],
        "modified_links": [],
        "added_joints": [],
        "removed_joints": [],
        "modified_joints": [],
    }

    # Get all link names
    old_links = set(old_kg.get_links())
    new_links = set(new_kg.get_links())

    # Added links
    for link_name in new_links - old_links:
        link_data = new_kg.get_link_data(link_name)
        if link_data:
            diff["added_links"].append(link_data.to_dict())

    # Removed links
    for link_name in old_links - new_links:
        diff["removed_links"].append(link_name)

    # Modified links
    for link_name in old_links & new_links:
        old_data = old_kg.get_link_data(link_name)
        new_data = new_kg.get_link_data(link_name)

        if old_data and new_data:
            changes = {}

            # Compare relevant fields
            if old_data.mass != new_data.mass:
                changes["mass"] = {"old": old_data.mass, "new": new_data.mass}

            if _inertia_changed(old_data.inertia, new_data.inertia):
                old_iner = old_data.inertia.to_dict() if old_data.inertia else None
                new_iner = new_data.inertia.to_dict() if new_data.inertia else None
                changes["inertia"] = {"old": old_iner, "new": new_iner}

            if old_data.visual_mesh != new_data.visual_mesh:
                changes["visual_mesh"] = {"old": old_data.visual_mesh, "new": new_data.visual_mesh}

            if old_data.visual_geometry != new_data.visual_geometry:
                changes["visual_geometry"] = {"old": old_data.visual_geometry, "new": new_data.visual_geometry}

            if old_data.visual_origin != new_data.visual_origin:
                changes["visual_origin"] = {"old": old_data.visual_origin, "new": new_data.visual_origin}

            if old_data.material != new_data.material:
                changes["material"] = {"old": old_data.material, "new": new_data.material}

            if old_data.collision_geometry != new_data.collision_geometry:
                changes["collision_geometry"] = {"old": old_data.collision_geometry, "new": new_data.collision_geometry}

            if old_data.collision_origin != new_data.collision_origin:
                changes["collision_origin"] = {"old": old_data.collision_origin, "new": new_data.collision_origin}

            if changes:
                diff["modified_links"].append({
                    "name": link_name,
                    "changes": changes
                })

    # Get all joint names (by joint object, not edge)
    old_joints = {j: old_kg.get_joint_data(j) for j in old_kg.get_joints()}
    new_joints = {j: new_kg.get_joint_data(j) for j in new_kg.get_joints()}

    old_joint_names = set(old_joints.keys())
    new_joint_names = set(new_joints.keys())

    # Added joints
    for joint_name in new_joint_names - old_joint_names:
        joint_data = new_joints[joint_name]
        if joint_data:
            diff["added_joints"].append(joint_data.to_dict())

    # Removed joints
    for joint_name in old_joint_names - new_joint_names:
        diff["removed_joints"].append(joint_name)

    # Modified joints
    for joint_name in old_joint_names & new_joint_names:
        old_data = old_joints[joint_name]
        new_data = new_joints[joint_name]

        if old_data and new_data:
            changes = {}

            if old_data.joint_type != new_data.joint_type:
                changes["joint_type"] = {"old": old_data.joint_type, "new": new_data.joint_type}

            if old_data.axis != new_data.axis:
                changes["axis"] = {"old": list(old_data.axis), "new": list(new_data.axis)}

            if old_data.origin_xyz != new_data.origin_xyz:
                changes["origin_xyz"] = {"old": list(old_data.origin_xyz), "new": list(new_data.origin_xyz)}

            if old_data.origin_rpy != new_data.origin_rpy:
                changes["origin_rpy"] = {"old": list(old_data.origin_rpy), "new": list(new_data.origin_rpy)}

            if _limits_changed(old_data.limits, new_data.limits):
                old_lim = old_data.limits.to_dict() if old_data.limits else None
                new_lim = new_data.limits.to_dict() if new_data.limits else None
                changes["limits"] = {"old": old_lim, "new": new_lim}

            if old_data.dynamics != new_data.dynamics:
                changes["dynamics"] = {"old": old_data.dynamics, "new": new_data.dynamics}

            if changes:
                diff["modified_joints"].append({
                    "name": joint_name,
                    "changes": changes
                })

    return diff


def apply_diff(kg: KinematicGraph, diff: Dict[str, Any]) -> KinematicGraph:
    """
    Apply a diff to a kinematic graph, returning a new modified graph.

    Args:
        kg: The original kinematic graph.
        diff: The diff dict produced by compute_diff.

    Returns:
        A new KinematicGraph with the diff applied.
    """
    from .types import Inertia, Limits

    # Create a copy by reconstructing from JSON
    new_kg = KinematicGraph.from_json(kg.to_json())

    # Remove links
    for link_name in diff.get("removed_links", []):
        if link_name in new_kg.graph:
            # Also remove any joints connected to this link
            edges_to_remove = []
            for u, v in new_kg.graph.edges():
                if u == link_name or v == link_name:
                    edges_to_remove.append((u, v))
            for u, v in edges_to_remove:
                new_kg.graph.remove_edge(u, v)
            # Remove the node
            new_kg.graph.remove_node(link_name)

    # Add new links
    for link_dict in diff.get("added_links", []):
        inertia = None
        if link_dict.get("inertia"):
            inertia_data = link_dict["inertia"]
            inertia = Inertia(**inertia_data)

        link_data = LinkData(
            name=link_dict["name"],
            mass=link_dict.get("mass", 0.0),
            inertia=inertia,
            visual_mesh=link_dict.get("visual_mesh"),
            visual_geometry=link_dict.get("visual_geometry"),
            visual_origin=link_dict.get("visual_origin"),
            material=link_dict.get("material"),
            collision_geometry=link_dict.get("collision_geometry"),
            collision_origin=link_dict.get("collision_origin"),
        )
        new_kg.add_link(link_data)

    # Modify links
    for mod_link in diff.get("modified_links", []):
        link_name = mod_link["name"]
        link_data = new_kg.get_link_data(link_name)
        if link_data:
            changes = mod_link.get("changes", {})

            for field, change in changes.items():
                new_val = change.get("new")
                if field == "mass":
                    link_data.mass = new_val
                elif field == "inertia":
                    if new_val:
                        link_data.inertia = Inertia(**new_val)
                    else:
                        link_data.inertia = None
                elif field == "visual_mesh":
                    link_data.visual_mesh = new_val
                elif field == "visual_geometry":
                    link_data.visual_geometry = new_val
                elif field == "visual_origin":
                    link_data.visual_origin = new_val
                elif field == "material":
                    link_data.material = new_val
                elif field == "collision_geometry":
                    link_data.collision_geometry = new_val
                elif field == "collision_origin":
                    link_data.collision_origin = new_val

    # Remove joints
    for joint_name in diff.get("removed_joints", []):
        for u, v in list(new_kg.graph.edges()):
            joint_data = new_kg.graph[u][v]["data"]
            if joint_data.name == joint_name:
                new_kg.graph.remove_edge(u, v)

    # Add new joints
    for joint_dict in diff.get("added_joints", []):
        limits = None
        if joint_dict.get("limits"):
            limits_data = joint_dict["limits"]
            limits = Limits(**limits_data)

        joint_data = JointData(
            name=joint_dict["name"],
            joint_type=joint_dict["joint_type"],
            parent_link=joint_dict["parent_link"],
            child_link=joint_dict["child_link"],
            axis=tuple(joint_dict.get("axis", [0.0, 0.0, 1.0])),
            origin_xyz=tuple(joint_dict.get("origin_xyz", [0.0, 0.0, 0.0])),
            origin_rpy=tuple(joint_dict.get("origin_rpy", [0.0, 0.0, 0.0])),
            limits=limits,
            dynamics=joint_dict.get("dynamics"),
        )
        new_kg.add_joint(joint_data)

    # Modify joints
    for mod_joint in diff.get("modified_joints", []):
        joint_name = mod_joint["name"]
        joint_data = new_kg.get_joint_data(joint_name)
        if joint_data:
            changes = mod_joint.get("changes", {})

            for field, change in changes.items():
                new_val = change.get("new")
                if field == "joint_type":
                    joint_data.joint_type = new_val
                elif field == "axis":
                    joint_data.axis = tuple(new_val)
                elif field == "origin_xyz":
                    joint_data.origin_xyz = tuple(new_val)
                elif field == "origin_rpy":
                    joint_data.origin_rpy = tuple(new_val)
                elif field == "limits":
                    if new_val:
                        joint_data.limits = Limits(**new_val)
                    else:
                        joint_data.limits = None
                elif field == "dynamics":
                    joint_data.dynamics = new_val

    return new_kg


def describe_diff(diff: Dict[str, Any]) -> str:
    """
    Create a human-readable summary of a diff.

    Args:
        diff: The diff dict produced by compute_diff.

    Returns:
        A string like "Added 1 link, removed 0 links, modified 2 links, added 1 joint, ..."
    """
    added_links = len(diff.get("added_links", []))
    removed_links = len(diff.get("removed_links", []))
    modified_links = len(diff.get("modified_links", []))
    added_joints = len(diff.get("added_joints", []))
    removed_joints = len(diff.get("removed_joints", []))
    modified_joints = len(diff.get("modified_joints", []))

    parts = []
    if added_links > 0:
        parts.append(f"Added {added_links} link{'s' if added_links != 1 else ''}")
    if removed_links > 0:
        parts.append(f"removed {removed_links} link{'s' if removed_links != 1 else ''}")
    if modified_links > 0:
        parts.append(f"modified {modified_links} link{'s' if modified_links != 1 else ''}")
    if added_joints > 0:
        parts.append(f"added {added_joints} joint{'s' if added_joints != 1 else ''}")
    if removed_joints > 0:
        parts.append(f"removed {removed_joints} joint{'s' if removed_joints != 1 else ''}")
    if modified_joints > 0:
        parts.append(f"modified {modified_joints} joint{'s' if modified_joints != 1 else ''}")

    if not parts:
        return "No changes"

    # Capitalize the first part
    result = parts[0].capitalize()
    if len(parts) > 1:
        result += ", " + ", ".join(parts[1:])

    return result


def _inertia_changed(old_iner, new_iner) -> bool:
    """Check if inertia changed."""
    if old_iner is None and new_iner is None:
        return False
    if old_iner is None or new_iner is None:
        return True
    return old_iner.to_dict() != new_iner.to_dict()


def _limits_changed(old_lim, new_lim) -> bool:
    """Check if limits changed."""
    if old_lim is None and new_lim is None:
        return False
    if old_lim is None or new_lim is None:
        return True
    return old_lim.to_dict() != new_lim.to_dict()
