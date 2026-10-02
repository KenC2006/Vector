"""
Which catalog part (if any) a URDF link is — the Python twin of
src/src/design/identity.ts.

Designer-built URDFs carry an explicit map in a
``<!-- vector:parts {"<link>": {"part": .., "component": <id or null>}} -->``
comment; that map is the identity. URDFs without it (hand-written, or from
older Vector builds) fall back to the ``<component_id>_<N>`` link-name
convention (or a link named exactly after a catalog id), accepted only when
the id is a real catalog part. No other name matching: what a link *is* comes
from the catalog, never from words in its name.

Everything in core/sim that needs "what part is this link" goes through
:class:`PartIdentity`.
"""
from __future__ import annotations

import json
import re
from typing import Any, Dict, Optional

from core.presets import get_component

_PARTS_RE = re.compile(r"<!-- vector:parts (.*?) -->", re.S)
_INSTANCE_RE = re.compile(r"^(.+?)_(\d+)$")

# sim_metadata.contact_class vocabulary. "drivetrain" is kept for the
# frontend's placement capabilities (hub motors, axles, knuckles) and is not a
# contact surface.
CONTACT_CLASSES = ("foot", "wheel", "track", "caster", "suction", "gripper", "none")
# Classes whose job is to touch the ground (feet, rolling gear, adhesion pads).
GROUND_CONTACT_CLASSES = frozenset(("foot", "wheel", "track", "caster", "suction"))


def parse_part_map(urdf_text: str) -> Optional[Dict[str, Dict[str, Any]]]:
    """The explicit link -> {part, component, ...} map, or None when absent/malformed."""
    m = _PARTS_RE.search(urdf_text or "")
    if not m:
        return None
    try:
        v = json.loads(m.group(1))
    except json.JSONDecodeError:
        return None
    return v if isinstance(v, dict) else None


def contact_class_of(component: Optional[Dict[str, Any]]) -> str:
    """The component's contact class ("none" for custom bodies and non-contact parts)."""
    if not component:
        return "none"
    cls = (component.get("sim_metadata") or {}).get("contact_class")
    return cls if isinstance(cls, str) and cls in CONTACT_CLASSES else "none"


class PartIdentity:
    """Link-name -> catalog component resolver for one URDF."""

    def __init__(self, urdf_text: str = ""):
        self.part_map = parse_part_map(urdf_text)
        self._cache: Dict[str, Optional[Dict[str, Any]]] = {}

    def component_id(self, link: str) -> Optional[str]:
        """Catalog component id of a link, or None for custom bodies / unknown links."""
        if self.part_map is not None:
            entry = self.part_map.get(link)
            cid = entry.get("component") if isinstance(entry, dict) else None
            return cid if isinstance(cid, str) and get_component(cid) else None
        m = _INSTANCE_RE.match(link)
        if m and get_component(m.group(1)):
            return m.group(1)
        return link if get_component(link) else None

    def component(self, link: str) -> Optional[Dict[str, Any]]:
        if link not in self._cache:
            cid = self.component_id(link)
            self._cache[link] = get_component(cid) if cid else None
        return self._cache[link]

    def contact_class(self, link: str) -> str:
        return contact_class_of(self.component(link))

    def part_name(self, link: str) -> str:
        """Design part name for a link (the link name itself when there is no map)."""
        entry = (self.part_map or {}).get(link)
        part = entry.get("part") if isinstance(entry, dict) else None
        return part if isinstance(part, str) else link
