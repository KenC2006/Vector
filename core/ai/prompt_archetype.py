"""
Prompt-level archetype classification.

The system prompt and post-LLM normalizers contain extensive scaffolding for
three known archetypes (quadruped, arm, wheeled). That scaffolding is helpful
when the user is asking for one of those — but actively harms designs that
fall outside it (crabs, snakes, octopuses, sprawled hexapods, etc.) because
detection thresholds are loose and the prompt's archetype rules dominate.

This module classifies the user prompt into one of:

  - "standard": prompt names a known archetype. Normal scaffolding applies.
  - "novel":    prompt names a non-standard creature/topology. The caller
                should switch to freedom mode: full catalog, no normalizer
                enforcement, freedom-mode preamble in the system prompt.
  - "unknown":  no clear signal either way. Caller falls back to existing
                behavior (scaffolding partially applies via topology
                inference).

Detection is keyword-based on purpose. A prompt-level signal is reliable
(the user typed the word "crab"); topology-based detection (≥4 foot pads
→ quadruped) is what regressed novel designs in the first place. We do not
try to be clever.
"""
from __future__ import annotations

import re

# Standard archetypes — when any of these match, the existing prompt rules
# and normalizer apply as designed. Order doesn't matter; first standard
# match wins over any novel match (a "robotic arm with a crab gripper" is
# still primarily an arm).
_STANDARD_KEYWORDS: tuple[str, ...] = (
    "dog", "doggo", "puppy", "quadruped", "spot", "go1", "anymal",
    "arm", "manipulator", "robotic arm", "robot arm", "6dof", "6-dof",
    "7dof", "7-dof",
    "rover", "car", "vehicle", "truck", "buggy", "cart", "bike",
    "motorcycle", "unicycle", "tank",
    "drone", "quadcopter",
    "biped", "humanoid",
)

# Novel archetypes — creature names and structural descriptors that do NOT
# fit dog/arm/wheeled. Hexapod is included even though catalog_selector
# already lists it: the existing scaffolding still tries to normalize it
# toward the quadruped template, which is why crabs come out as dogs.
_NOVEL_KEYWORDS: tuple[str, ...] = (
    "crab", "spider", "octopus", "octopod", "snake", "starfish",
    "centipede", "scorpion", "lobster", "ant", "insect",
    "hexapod", "octopod", "tripod",
    "sprawled", "radial", "starfish-like",
    "novel", "custom", "weird", "unusual", "experimental", "prototype",
    "non-standard", "nonstandard",
)

# Numeric-leg-count phrases that imply non-quadruped morphology. Match as
# regex on the lowercased prompt to catch "8 legs", "8-legged", "eight legs",
# etc. Anything other than 2 or 4 legs is structurally novel.
_NOVEL_LEGCOUNT_RE = re.compile(
    r"\b(?:6|8|10|six|eight|ten)[ \-]?(?:leg|legs|legged|footed)\b"
)


def classify_prompt(prompt: str | None) -> str:
    """Return 'standard' | 'novel' | 'unknown' for a user prompt.

    Standard wins ties: "build a robot arm with crab-claw fingers" → standard
    (it's an arm, the crab phrasing only describes the gripper). The user
    has to actually request a non-standard *primary structure* to get
    freedom mode.
    """
    if not prompt:
        return "unknown"
    text = prompt.lower()
    # Strip punctuation so "crab," and "crab." still match the keyword.
    text_words = re.sub(r"[^a-z0-9 \-]", " ", text)

    for kw in _STANDARD_KEYWORDS:
        # Word-boundary check so "scar" doesn't match "car".
        if re.search(rf"\b{re.escape(kw)}\b", text_words):
            return "standard"

    for kw in _NOVEL_KEYWORDS:
        if re.search(rf"\b{re.escape(kw)}\b", text_words):
            return "novel"

    if _NOVEL_LEGCOUNT_RE.search(text_words):
        return "novel"

    return "unknown"


def is_novel(prompt: str | None) -> bool:
    """Convenience predicate for the common 'switch to freedom mode' check."""
    return classify_prompt(prompt) == "novel"
