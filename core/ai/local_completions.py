"""
Local URDF/XML completion engine — no external API required.
Provides context-aware completions based on URDF structure and common patterns.
Falls back to this when the Claude API is unavailable.
"""
import re
from typing import Optional, List, Dict

# Lazy-load preset catalog
_preset_components: Optional[List[Dict]] = None

def _get_presets() -> List[Dict]:
    global _preset_components
    if _preset_components is None:
        try:
            from core.presets import list_components
            _preset_components = list_components()
        except Exception:
            _preset_components = []
    return _preset_components


def _find_preset_by_partial(partial: str) -> Optional[Dict]:
    """Find the best matching preset for a partial ID or name."""
    partial_lower = partial.lower().replace('-', '_').replace(' ', '_')
    presets = _get_presets()
    # Exact prefix match on ID
    for p in presets:
        if p['id'].startswith(partial_lower):
            return p
    # Substring match on name
    for p in presets:
        if partial_lower in p['name'].lower():
            return p
    return None


def _preset_link_snippet(comp: Dict, idx: int) -> str:
    """Generate a full URDF link+joint snippet from a preset component."""
    phys = comp['physical']
    mass = phys.get('mass_kg') or phys.get('mass_kg_per_100mm', 0.1)
    bb = phys.get('bounding_box_mm', [40, 40, 40])
    shape = phys.get('inertia_primitive', 'box')
    xm = bb[0] / 1000 if len(bb) > 0 else 0.04
    ym = bb[1] / 1000 if len(bb) > 1 else 0.04
    zm = bb[2] / 1000 if len(bb) > 2 else 0.04

    # Inertia
    if shape == 'cylinder':
        r = max(xm, ym) / 2
        ixx = mass / 12 * (3 * r * r + zm * zm)
        iyy = ixx
        izz = mass / 2 * r * r
    elif shape == 'sphere':
        r = max(xm, ym, zm) / 2
        ixx = iyy = izz = 2 / 5 * mass * r * r
    else:
        ixx = mass / 12 * (ym**2 + zm**2)
        iyy = mass / 12 * (xm**2 + zm**2)
        izz = mass / 12 * (xm**2 + ym**2)

    link_name = f"{comp['id']}_{idx}"
    joint_name = f"joint_{comp['id']}_{idx}"

    # Geometry
    if shape == 'cylinder':
        geom = f'<cylinder radius="{max(xm,ym)/2:.4f}" length="{zm:.4f}"/>'
    elif shape == 'sphere':
        geom = f'<sphere radius="{max(xm,ym,zm)/2:.4f}"/>'
    else:
        geom = f'<box size="{xm:.4f} {ym:.4f} {zm:.4f}"/>'

    cat = comp['id'].split('_')[0]
    is_actuated = cat in ('actuator', 'motor')
    joint_type = 'revolute' if is_actuated else 'fixed'

    snippet = f'''<link name="{link_name}">
    <inertial>
      <mass value="{mass:.4f}"/>
      <inertia ixx="{ixx:.6f}" ixy="0" ixz="0" iyy="{iyy:.6f}" iyz="0" izz="{izz:.6f}"/>
    </inertial>
    <visual>
      <geometry>
        {geom}
      </geometry>
    </visual>
    <collision>
      <geometry>
        {geom}
      </geometry>
    </collision>
  </link>
  <joint name="{joint_name}" type="{joint_type}">
    <origin xyz="0 0 {zm + 0.01:.4f}" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>'''

    if is_actuated:
        torque = comp.get('mechanical_electrical', {}).get('max_torque_nm') or \
                 comp.get('mechanical_electrical', {}).get('holding_torque_nm', 10)
        snippet += f'\n    <limit lower="-3.14159" upper="3.14159" effort="{torque}" velocity="3.14"/>'

    return snippet


def generate_local_completion(
    urdf_content: str,
    cursor_line: int,
    cursor_column: int,
    prefix: str = ""
) -> Optional[str]:
    """
    Generate a completion based on local pattern matching.
    Uses the prefix (last ~100 chars before cursor) as primary input.

    Returns None if no good completion can be determined.
    """
    lines = urdf_content.split('\n')
    if cursor_line < 1 or cursor_line > len(lines):
        return None

    current_line = lines[cursor_line - 1] if cursor_line <= len(lines) else ""

    # Use prefix for matching — it's the most reliable signal from the frontend.
    # The prefix contains the last ~100 chars ending right at the cursor.
    # We primarily care about the last line of the prefix (the current line up to cursor).
    prefix_lines = prefix.split('\n')
    typing = prefix_lines[-1].strip() if prefix_lines else ""

    # Also compute from cursor position as backup
    text_before_cursor = current_line[:cursor_column - 1] if cursor_column > 1 else ""
    stripped = text_before_cursor.strip()

    # Use whichever has more content
    if len(typing) > len(stripped):
        stripped = typing

    # Get previous non-empty lines for context
    prev_line = ""
    prev_prev_line = ""
    found = 0
    for i in range(cursor_line - 2, -1, -1):
        if lines[i].strip():
            if found == 0:
                prev_line = lines[i].strip()
                found += 1
            elif found == 1:
                prev_prev_line = lines[i].strip()
                break

    # Collect existing names for smart suggestions
    link_names = re.findall(r'<link\s+name="([^"]*)"', urdf_content)
    joint_names = re.findall(r'<joint\s+name="([^"]*)"', urdf_content)

    # ── Attribute value completion ──────────────────────────────────────

    # type="  →  suggest joint type
    if re.search(r'type="$', stripped):
        return 'revolute">'

    # <joint name="  →  suggest a name
    if re.search(r'<joint\s+name="$', stripped):
        idx = len(joint_names) + 1
        return f'joint_{idx}" type="revolute">'

    # <link name="  →  suggest a name (preset-aware)
    if re.search(r'<link\s+name="$', stripped):
        idx = len(link_names) + 1
        # Check if previous context hints at a component type
        context_window = '\n'.join(lines[max(0, cursor_line-5):cursor_line])
        for keyword in ('servo', 'motor', 'sensor', 'camera', 'lidar', 'imu', 'gripper', 'battery', 'wheel'):
            if keyword in context_window.lower():
                comp = _find_preset_by_partial(keyword)
                if comp:
                    return f'{comp["id"]}_{idx}">'
        return f'link_{idx}">'

    # name="  →  generic name
    if stripped.endswith('name="'):
        return 'name_1"'

    # link="  →  suggest a link name
    if stripped.endswith('link="'):
        if link_names:
            return f'{link_names[0]}"/'
        return 'base_link"/'

    # xyz="  →  suggest coordinates
    if stripped.endswith('xyz="'):
        return '0 0 0"'

    # rpy="  →  suggest rotation
    if stripped.endswith('rpy="'):
        return '0 0 0"'

    # value="  →  suggest a value (preset-aware for mass)
    if stripped.endswith('value="'):
        # Check if we're inside a preset-named link's <mass> element
        if '<mass' in stripped:
            for i in range(cursor_line - 2, max(0, cursor_line - 10), -1):
                link_match = re.search(r'<link\s+name="([^"]*)"', lines[i])
                if link_match:
                    comp = _find_preset_by_partial(link_match.group(1))
                    if comp:
                        mass = comp['physical'].get('mass_kg') or comp['physical'].get('mass_kg_per_100mm', 0.1)
                        return f'{mass:.4f}"/'
                    break
        return '1.0"/'

    # size="  →  suggest box dimensions
    if stripped.endswith('size="'):
        return '0.1 0.1 0.1"/'

    # radius="  →  suggest radius
    if stripped.endswith('radius="'):
        return '0.05"'

    # length="  →  suggest length
    if stripped.endswith('length="'):
        return '0.1"/'

    # filename="  →  suggest mesh path
    if stripped.endswith('filename="'):
        return 'package://meshes/part.stl"/'

    # ── Blank line: suggest child elements based on parent context ──────

    if stripped == "":
        # Inside <link ...>  →  suggest inertial
        if re.match(r'<link\s+', prev_line):
            return '<inertial>\n      <mass value="1.0"/>\n      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>\n    </inertial>'

        # Inside <joint ...>  →  suggest parent/child
        if re.match(r'<joint\s+', prev_line):
            parent = link_names[0] if link_names else "base_link"
            child = link_names[1] if len(link_names) > 1 else "link_1"
            return f'<parent link="{parent}"/>\n    <child link="{child}"/>\n    <origin xyz="0 0 0" rpy="0 0 0"/>\n    <axis xyz="0 0 1"/>'

        # Inside <visual> or <collision>  →  suggest geometry
        if "<visual>" in prev_line or "<collision>" in prev_line:
            return '<geometry>\n        <box size="0.1 0.1 0.1"/>\n      </geometry>'

        # Inside <geometry>  →  suggest a shape
        if "<geometry>" in prev_line:
            return '<box size="0.1 0.1 0.1"/>'

        # Inside <inertial>  →  suggest mass
        if "<inertial>" in prev_line:
            return '<mass value="1.0"/>'

        # After </link> or </joint>  →  suggest next element
        if prev_line.startswith("</link>"):
            idx = len(link_names) + 1
            return f'<link name="link_{idx}">'
        if prev_line.startswith("</joint>"):
            idx = len(joint_names) + 1
            return f'<joint name="joint_{idx}" type="revolute">'

        # After a comment mentioning a component type → suggest preset snippet
        if prev_line.startswith("<!--"):
            comment_text = prev_line.lower()
            for keyword in ('servo', 'motor', 'sensor', 'camera', 'lidar', 'imu', 'gripper',
                            'battery', 'wheel', 'stepper', 'actuator', 'bearing', 'bracket'):
                if keyword in comment_text:
                    comp = _find_preset_by_partial(keyword)
                    if comp:
                        idx = len(link_names) + 1
                        return _preset_link_snippet(comp, idx)
            return None

        # After a self-closing parent/child tag  →  suggest next joint child
        if prev_line.startswith("<parent "):
            child = link_names[-1] if link_names else "link_1"
            return f'<child link="{child}"/>'
        if prev_line.startswith("<child "):
            return '<origin xyz="0 0 0" rpy="0 0 0"/>'
        if prev_line.startswith("<origin "):
            return '<axis xyz="0 0 1"/>'
        if prev_line.startswith("<axis "):
            return '<limit lower="-1.57" upper="1.57" effort="100" velocity="1.0"/>'
        if prev_line.startswith("<mass "):
            return '<inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>'

        return None

    # ── Partial tag completion ──────────────────────────────────────────

    # Build a table of partial-tag → completion
    partial_completions = {
        "<par": ('ent link="', link_names[0] if link_names else "base_link", '"/>'),
        "<chi": ('ld link="', link_names[-1] if link_names else "link_1", '"/>'),
        "<ori": ('gin xyz="0 0 0" rpy="0 0 0"/>', ),
        "<ax": ('is xyz="0 0 1"/>', ),
        "<ma": ('ss value="1.0"/>', ),
        "<ge": ('ometry>', ),
        "<vi": ('sual>', ),
        "<col": ('lision>', ),
        "<in": ('ertial>', ),
        "<bo": ('x size="0.1 0.1 0.1"/>', ),
        "<cy": ('linder radius="0.05" length="0.1"/>', ),
        "<sp": ('here radius="0.05"/>', ),
        "<me": ('sh filename="package://meshes/part.stl"/>', ),
        "<lim": ('it lower="-1.57" upper="1.57" effort="100" velocity="1.0"/>', ),
        "<ro": ('bot name="my_robot">', ),
    }

    # <li  →  could be <link> or <limit> — context-dependent
    if stripped.startswith("<li"):
        if any("<inertial" in lines[j] for j in range(max(0, cursor_line - 4), cursor_line)):
            return _complete_partial(stripped, "<lim", 'it lower="-1.57" upper="1.57" effort="100" velocity="1.0"/>')
        idx = len(link_names) + 1
        return _complete_partial(stripped, "<li", f'nk name="link_{idx}">')

    # <jo  →  <joint ...>
    if stripped.startswith("<jo"):
        idx = len(joint_names) + 1
        return _complete_partial(stripped, "<jo", f'int name="joint_{idx}" type="revolute">')

    # <in  →  could be <inertial> or <inertia> — context-dependent
    if stripped.startswith("<in"):
        if any("<inertial" in lines[j] for j in range(max(0, cursor_line - 3), cursor_line)):
            return _complete_partial(stripped, "<in", 'ertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>')
        return _complete_partial(stripped, "<in", "ertial>")

    # Try all other partial completions
    for partial, parts in partial_completions.items():
        if stripped.startswith(partial):
            completion = ''.join(parts)
            full = partial + completion
            if full.startswith(stripped):
                return full[len(stripped):]
            return completion

    # ── Closing tag completion ──────────────────────────────────────────

    if stripped.startswith("</"):
        # Look backwards for the nearest unclosed tag
        tag_typed = stripped[2:]  # what's after </
        open_stack = []
        for i in range(cursor_line - 1, -1, -1):
            line = lines[i]
            # Find opening and closing tags
            opens = re.findall(r'<(\w+)[\s>]', line)
            closes = re.findall(r'</(\w+)>', line)
            # Process in reverse order for correct nesting
            for tag in reversed(opens):
                open_stack.append(tag)
            for tag in reversed(closes):
                if open_stack and open_stack[-1] == tag:
                    open_stack.pop()
                elif tag in open_stack:
                    open_stack.remove(tag)
        if open_stack:
            closest = open_stack[-1]
            # If user already typed part of the tag name, check if it matches
            if tag_typed and closest.startswith(tag_typed):
                return closest[len(tag_typed):] + ">"
            elif not tag_typed:
                return closest + ">"

    # ── Attribute completion after a space inside a tag ──────────────────

    # Inside an opening tag, after a space  →  suggest attributes
    tag_match = re.match(r'<(\w+)\s', stripped)
    if tag_match and not stripped.endswith('>') and not stripped.endswith('/>'):
        tag_name = tag_match.group(1)
        if tag_name == "joint" and 'type=' not in stripped:
            return 'type="revolute">'
        if tag_name in ("joint", "link") and 'name=' not in stripped:
            return 'name=""'
        if tag_name == "origin":
            if 'xyz=' not in stripped:
                return 'xyz="0 0 0" rpy="0 0 0"/>'
            if 'rpy=' not in stripped:
                return 'rpy="0 0 0"/>'

    return None


def _complete_partial(typed: str, prefix: str, suffix: str) -> Optional[str]:
    """Complete a partial tag match. Returns only the untyped portion."""
    full = prefix + suffix
    if full.startswith(typed):
        return full[len(typed):]
    return suffix
