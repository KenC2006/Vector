"""
URDF validation engine.

Runs structural, physics, actuator, and mesh checks against a KinematicGraph.
Returns categorized results with severity levels: pass, warn, error, info.
"""
from typing import Any, Dict, List, Optional
import math

from model.kinematic_graph import KinematicGraph
from model.types import LinkData, JointData


class ValidationResult:
    """Single validation check result."""

    def __init__(self, name: str, severity: str, message: str, category: str):
        self.name = name
        self.severity = severity  # "pass", "warn", "error", "info"
        self.message = message
        self.category = category

    def to_dict(self) -> Dict[str, str]:
        return {
            "name": self.name,
            "severity": self.severity,
            "message": self.message,
            "category": self.category,
        }


def validate_kinematic_graph(kg: KinematicGraph) -> List[Dict[str, str]]:
    """
    Run all validation checks on a KinematicGraph.

    Returns list of ValidationResult dicts grouped by category.
    """
    results: List[ValidationResult] = []

    # ── Structural checks ────────────────────────────────────────────────
    results.extend(_check_structural(kg))

    # ── Physics checks ───────────────────────────────────────────────────
    results.extend(_check_physics(kg))

    # ── Actuator checks ──────────────────────────────────────────────────
    results.extend(_check_actuators(kg))

    # ── Mesh checks ──────────────────────────────────────────────────────
    results.extend(_check_meshes(kg))

    return [r.to_dict() for r in results]


# ── Structural ───────────────────────────────────────────────────────────────

def _check_structural(kg: KinematicGraph) -> List[ValidationResult]:
    results = []

    # 1. Root link exists
    if kg.root_link and kg.root_link in kg.graph:
        results.append(ValidationResult(
            "Root link defined",
            "pass",
            f"Root link '{kg.root_link}' exists",
            "Structural",
        ))
    else:
        results.append(ValidationResult(
            "Root link defined",
            "error",
            "No root link defined or root link not in graph",
            "Structural",
        ))

    # 2. Check for orphan links (no parent and not root)
    orphans = []
    for link_name in kg.get_links():
        parent = kg.get_parent_link(link_name)
        if parent is None and link_name != kg.root_link:
            orphans.append(link_name)

    if orphans:
        results.append(ValidationResult(
            "No orphan links",
            "error",
            f"Orphan links (no parent, not root): {', '.join(orphans)}",
            "Structural",
        ))
    else:
        results.append(ValidationResult(
            "No orphan links",
            "pass",
            "All links are connected to the tree",
            "Structural",
        ))

    # 3. Tree structure (DAG, no cycles)
    import networkx as nx
    if nx.is_directed_acyclic_graph(kg.graph):
        results.append(ValidationResult(
            "Tree structure OK",
            "pass",
            "Graph is a valid DAG (no cycles)",
            "Structural",
        ))
    else:
        results.append(ValidationResult(
            "Tree structure OK",
            "error",
            "Graph contains cycles — invalid for URDF",
            "Structural",
        ))

    # 4. Check for duplicate link names
    link_names = kg.get_links()
    if len(link_names) != len(set(link_names)):
        seen = set()
        dupes = [n for n in link_names if n in seen or seen.add(n)]  # type: ignore
        results.append(ValidationResult(
            "Unique link names",
            "error",
            f"Duplicate link names: {', '.join(dupes)}",
            "Structural",
        ))
    else:
        results.append(ValidationResult(
            "Unique link names",
            "pass",
            f"{len(link_names)} links, all uniquely named",
            "Structural",
        ))

    # 5. Check for duplicate joint names
    joint_names = kg.get_joints()
    if len(joint_names) != len(set(joint_names)):
        seen = set()
        dupes = [n for n in joint_names if n in seen or seen.add(n)]  # type: ignore
        results.append(ValidationResult(
            "Unique joint names",
            "error",
            f"Duplicate joint names: {', '.join(dupes)}",
            "Structural",
        ))
    else:
        results.append(ValidationResult(
            "Unique joint names",
            "pass",
            f"{len(joint_names)} joints, all uniquely named",
            "Structural",
        ))

    # 6. Single connected component
    if kg.graph.number_of_nodes() > 0:
        undirected = kg.graph.to_undirected()
        components = list(nx.connected_components(undirected))
        if len(components) == 1:
            results.append(ValidationResult(
                "Single component",
                "pass",
                "All links form one connected tree",
                "Structural",
            ))
        else:
            results.append(ValidationResult(
                "Single component",
                "error",
                f"Graph has {len(components)} disconnected components",
                "Structural",
            ))

    return results


# ── Physics ──────────────────────────────────────────────────────────────────

def _check_physics(kg: KinematicGraph) -> List[ValidationResult]:
    results = []

    links_missing_mass = []
    links_missing_inertia = []
    links_zero_mass = []
    links_bad_inertia = []

    for link_name in kg.get_links():
        link_data = kg.get_link_data(link_name)
        if not link_data:
            continue

        # Base/root links are allowed to have zero mass
        is_root = (link_name == kg.root_link)

        # Check mass
        if link_data.mass == 0.0 and not is_root:
            links_zero_mass.append(link_name)
        elif link_data.mass < 0:
            links_missing_mass.append(link_name)

        # Check inertia
        if link_data.inertia is None and not is_root and link_data.mass > 0:
            links_missing_inertia.append(link_name)
        elif link_data.inertia is not None:
            inertia = link_data.inertia
            # Check diagonal elements are positive
            if inertia.ixx <= 0 or inertia.iyy <= 0 or inertia.izz <= 0:
                if not is_root:
                    links_bad_inertia.append(link_name)

            # Triangle inequality check for physically realizable inertia
            if inertia.ixx > 0 and inertia.iyy > 0 and inertia.izz > 0:
                if not (inertia.ixx + inertia.iyy >= inertia.izz and
                        inertia.ixx + inertia.izz >= inertia.iyy and
                        inertia.iyy + inertia.izz >= inertia.ixx):
                    if link_name not in links_bad_inertia:
                        links_bad_inertia.append(link_name)

    # Mass check
    if links_zero_mass:
        results.append(ValidationResult(
            "Link masses set",
            "warn",
            f"Zero mass on non-root links: {', '.join(links_zero_mass)}",
            "Physics",
        ))
    elif links_missing_mass:
        results.append(ValidationResult(
            "Link masses set",
            "error",
            f"Negative mass on links: {', '.join(links_missing_mass)}",
            "Physics",
        ))
    else:
        results.append(ValidationResult(
            "Link masses set",
            "pass",
            "All non-root links have positive mass",
            "Physics",
        ))

    # Inertia check
    if links_bad_inertia:
        results.append(ValidationResult(
            "Inertia tensors plausible",
            "warn",
            f"Non-physical inertia on: {', '.join(links_bad_inertia)}",
            "Physics",
        ))
    elif links_missing_inertia:
        results.append(ValidationResult(
            "Inertia tensors plausible",
            "warn",
            f"Missing inertia on: {', '.join(links_missing_inertia)}",
            "Physics",
        ))
    else:
        results.append(ValidationResult(
            "Inertia tensors plausible",
            "pass",
            "All inertia tensors are physically valid",
            "Physics",
        ))

    # Total mass check
    total_mass = sum(
        (kg.get_link_data(l).mass if kg.get_link_data(l) else 0)
        for l in kg.get_links()
    )
    if total_mass > 0:
        results.append(ValidationResult(
            "Total mass",
            "info",
            f"Total robot mass: {total_mass:.3f} kg",
            "Physics",
        ))
    else:
        results.append(ValidationResult(
            "Total mass",
            "warn",
            "Total robot mass is 0 kg",
            "Physics",
        ))

    return results


# ── Actuators ────────────────────────────────────────────────────────────────

def _check_actuators(kg: KinematicGraph) -> List[ValidationResult]:
    results = []

    joints_no_limits = []
    joints_bad_limits = []
    joints_no_effort = []
    actuated_count = 0
    fixed_count = 0

    for u, v in kg.graph.edges():
        joint_data: JointData = kg.graph[u][v]["data"]

        if joint_data.joint_type == "fixed":
            fixed_count += 1
            continue

        actuated_count += 1

        # Check limits on actuated joints
        if joint_data.joint_type in ("revolute", "prismatic"):
            if joint_data.limits is None:
                joints_no_limits.append(joint_data.name)
            else:
                # Check lower < upper
                if joint_data.limits.lower >= joint_data.limits.upper:
                    joints_bad_limits.append(joint_data.name)

                # Check effort limit is set
                if joint_data.limits.effort is None or joint_data.limits.effort <= 0:
                    joints_no_effort.append(joint_data.name)

    # Joint limits check
    if joints_bad_limits:
        results.append(ValidationResult(
            "Joint limits valid",
            "error",
            f"Invalid limits (lower >= upper): {', '.join(joints_bad_limits)}",
            "Actuators",
        ))
    elif joints_no_limits:
        results.append(ValidationResult(
            "Joint limits valid",
            "warn",
            f"Missing limits on: {', '.join(joints_no_limits)}",
            "Actuators",
        ))
    else:
        results.append(ValidationResult(
            "Joint limits valid",
            "pass",
            "All actuated joints have valid limits",
            "Actuators",
        ))

    # Effort/torque limits check
    if joints_no_effort:
        results.append(ValidationResult(
            "Torque limits set",
            "warn",
            f"No effort limit on: {', '.join(joints_no_effort)}",
            "Actuators",
        ))
    else:
        results.append(ValidationResult(
            "Torque limits set",
            "pass",
            "All actuated joints have effort limits",
            "Actuators",
        ))

    # Joint count info
    results.append(ValidationResult(
        "Joint summary",
        "info",
        f"{actuated_count} actuated, {fixed_count} fixed joints",
        "Actuators",
    ))

    return results


# ── Meshes ───────────────────────────────────────────────────────────────────

def _check_meshes(kg: KinematicGraph) -> List[ValidationResult]:
    results = []

    links_no_collision = []
    links_no_visual = []
    has_any_mesh = False

    for link_name in kg.get_links():
        link_data = kg.get_link_data(link_name)
        if not link_data:
            continue

        is_root = (link_name == kg.root_link)

        if link_data.collision_geometry is None and not is_root:
            links_no_collision.append(link_name)

        if link_data.visual_mesh is not None:
            has_any_mesh = True

    # Collision geometry check
    if links_no_collision:
        results.append(ValidationResult(
            "Collision geometry",
            "warn",
            f"Missing collision on: {', '.join(links_no_collision)}",
            "Mesh",
        ))
    else:
        results.append(ValidationResult(
            "Collision geometry",
            "pass",
            "All non-root links have collision geometry",
            "Mesh",
        ))

    # Mesh watertight check (we can't actually do this without the mesh files)
    results.append(ValidationResult(
        "Mesh watertight check",
        "info",
        "Mesh watertight check skipped (requires mesh files)",
        "Mesh",
    ))

    return results
