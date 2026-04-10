"""
Claude AI client for robot design editing.
Communicates with Claude API to generate robot model edits based on natural language requests.
"""
import json
import os
import re
import sys
import time
from collections import defaultdict

# Lazy import — anthropic may not be installed
_anthropic = None
_anthropic_error = None
try:
    import anthropic
    _anthropic = anthropic
except ImportError as e:
    _anthropic_error = str(e)

# ── Singleton client ──────────────────────────────────────────────────────────
_client = None

def _get_client():
    """Return a shared Anthropic client instance (reuses HTTP connection pool)."""
    global _client
    if _client is None:
        if _anthropic is None:
            raise ImportError(
                f"anthropic package not installed. Run: pip install anthropic\n"
                f"Error: {_anthropic_error}"
            )
        api_key = os.environ.get("ANTHROPIC_API_KEY")
        if not api_key:
            raise ValueError("ANTHROPIC_API_KEY environment variable not set")
        _client = _anthropic.Anthropic(api_key=api_key)
    return _client

# ── Conversation history store (keyed by session_id) ──────────────────────────
# Each entry is a list of {"role": "user"|"assistant", "content": str} dicts.
# Capped to last 20 messages to avoid unbounded token growth.
_conversation_history: dict[str, list] = defaultdict(list)
_MAX_HISTORY_MESSAGES = 20


def _build_component_catalog() -> str:
    """Build a compact summary of available preset components for the AI system prompt."""
    try:
        from core.presets import list_components, get_all_categories, get_category
        lines = []
        for cat_name in get_all_categories():
            cat = get_category(cat_name)
            label = cat_name.replace("_", " ").title()
            comps = cat["components"]
            items = []
            for c in comps:
                phys = c["physical"]
                mass = phys.get("mass_kg") or phys.get("mass_kg_per_100mm")
                mass_str = f"{mass}kg" if mass and mass >= 1 else f"{round(mass*1000)}g" if mass else "?"
                bb = phys.get("bounding_box_mm", [])
                bb_str = f"{bb[0]}x{bb[1]}x{bb[2]}mm" if len(bb) >= 3 else ""
                shape = phys.get("inertia_primitive", "box")
                # Key spec
                me = c.get("mechanical_electrical", {})
                spec = ""
                if "max_torque_nm" in me: spec = f"{me['max_torque_nm']}Nm"
                elif "holding_torque_nm" in me: spec = f"{me['holding_torque_nm']}Nm"
                elif "max_force_n" in me: spec = f"{me['max_force_n']}N"
                elif "grip_force_n" in me: spec = f"{me['grip_force_n']}N"
                elif "capacity_mah" in me: spec = f"{me['capacity_mah']}mAh"
                elif "fov_h_deg" in me: spec = f"{me['fov_h_deg']}°FOV"
                elif "range_m" in me: spec = f"{me['range_m']}m"
                items.append(f"  - {c['id']}: {c['name']} [{mass_str}, {bb_str}, {shape}]{(' ' + spec) if spec else ''}")
            lines.append(f"\n{label} ({len(comps)}):")
            lines.extend(items)
        return "\n".join(lines)
    except Exception as e:
        return f"(Component catalog unavailable: {e})"

_COMPONENT_CATALOG = None

def _get_component_catalog() -> str:
    global _COMPONENT_CATALOG
    if _COMPONENT_CATALOG is None:
        _COMPONENT_CATALOG = _build_component_catalog()
    return _COMPONENT_CATALOG

SYSTEM_PROMPT = """You are a robot design assistant for Vector IDE.

CRITICAL: You must return ONLY valid JSON. No English text, no markdown, no code blocks. Just a JSON object.

You receive:
1. The current URDF XML of a robot
2. The kinematic graph (links, joints, masses, geometries)
3. A Robot Structure Summary (link names, joint types, kinematic chain)
4. A natural language edit request from the user

You have conversation memory for multi-turn context.

## Component Preset Library

When adding components, use values from this library (never invent masses/dimensions).
Use the component ID as the link name prefix (e.g., "actuator_servo_high_torque_4").

Component rules:
- mass_kg → <mass value="..."/>
- bounding_box_mm (÷1000) → geometry size in meters
- inertia_primitive → geometry type (box/cylinder/sphere)
- Actuators/motors → joint type="revolute", effort = max_torque_nm
- Everything else → joint type="fixed"
- Inertia: Box Ixx=m/12*(h²+d²), Cylinder Ixx=m/12*(3r²+h²), Sphere Ixx=2/5*m*r²

Available components:
{COMPONENT_CATALOG}

## Output Format

Return ONLY this JSON structure (no other text). You have TWO options:

Option A — For modifications to existing URDF (adding/removing/changing parts):
{
    "explanation": "Human-readable description of changes",
    "edits": [
        {"search": "exact text to find in URDF", "replace": "replacement text"},
        ...
    ],
    "changes_summary": "Short stats"
}

Option B — For creating a new robot from scratch or replacing the entire URDF:
{
    "explanation": "Human-readable description of the design",
    "full_urdf": "<?xml version=\"1.0\"?>\n<robot name=\"...\">\n  ... complete URDF ...\n</robot>",
    "changes_summary": "Short stats"
}

Use Option B when the user says "build me", "create", "design", or "make" a robot from scratch.
Use Option A for incremental edits like "add a sensor", "change the arm length", etc.

Edit rules (Option A only):
- "search" must be an EXACT substring of the current URDF (verbatim, including whitespace)
- "replace" is what replaces it
- Edits are applied in order, each on the result of the previous
- Keep edits minimal — only change what's needed

Rules:
- ALWAYS use component preset values for physical properties
- Maintain valid URDF XML structure
- Use SI units: meters, kilograms, radians
- If adding links, include inertial, visual, and collision elements
- Position robots so the ground contact points (feet, wheels, base) are at Z=0 and the body is ABOVE the ground. The grid plane is at Z=0 — nothing should be below it.
- For legged robots: set joint origins so the legs are in a natural standing pose at rest (knees slightly bent, not straight). Use negative Z offsets from hip to knee to foot. The body should be at a realistic height above ground.

IMPORTANT: Keep URDF output concise. Omit comments. Use minimal whitespace. Don't add redundant collision elements if they match the visual geometry exactly.

REMINDER: Return ONLY JSON. No English preamble. Start your response with { and end with }.
"""

COMPLETION_SYSTEM_PROMPT = """You are a URDF/XML code completion engine for a robotics IDE.

You receive:
- ===CURSOR_PREFIX===: the last ~100 characters IMMEDIATELY before the cursor. THIS IS WHAT YOU ARE CONTINUING. Your output is concatenated directly after this text.
- ===CONTEXT===: broader surrounding code for reference (before and after cursor).
- ===ROBOT===: (optional) summary of the robot's links, joints, and kinematic chain.

Your job: output the raw XML that comes IMMEDIATELY after ===CURSOR_PREFIX===.

Rules:
- Output raw XML only. No English, no explanations, no markdown, no code fences.
- CRITICAL: Your very first character must be the correct next character after the prefix.
  Examples:
  - Prefix ends with `<link` → your output starts with ` name="...">`  (SPACE then name)
  - Prefix ends with `<joint` → your output starts with ` name="...">`  (SPACE then name)
  - Prefix ends with `name="gripper_` → your output starts with `finger">`  (finish the value)
  - Prefix ends with `</collision>` on a blank line → output the next element
- Produce VALID XML. The concatenation prefix + your_output must be well-formed.
- Complete up to ~15 lines. Stop at a natural boundary (closing tag, end of element).
- Do NOT repeat code that already exists after the cursor in ===CONTEXT===.
- Use link/joint names from ===ROBOT=== when available.
- Match the indentation style of the surrounding code."""


def _apply_edits(urdf: str, edits: list) -> str:
    """
    Apply a list of search/replace edit operations to a URDF string.

    Each edit is a dict with "search" and "replace" keys. Edits are applied
    sequentially — each one modifies the URDF for the next.

    If a search string is not found, it tries whitespace-normalized matching
    as a fallback (handles minor indentation differences from the model).

    Raises ValueError if a search string cannot be found at all.
    """
    result = urdf

    for i, edit in enumerate(edits):
        if not isinstance(edit, dict):
            print(f"[ai_edit] Skipping non-dict edit at index {i}", file=sys.stderr)
            continue

        search = edit.get("search", "")
        replace = edit.get("replace", "")

        if not search:
            print(f"[ai_edit] Skipping edit {i} with empty search string", file=sys.stderr)
            continue

        # Try exact match first
        if search in result:
            result = result.replace(search, replace, 1)
            print(f"[ai_edit] Applied edit {i}: exact match ({len(search)}c → {len(replace)}c)", file=sys.stderr)
            continue

        # Fallback: whitespace-normalized matching
        # Normalize both the search and every possible window of the URDF
        search_normalized = re.sub(r'[ \t]+', ' ', search.strip())
        lines = result.split('\n')

        # Try to find a contiguous block of lines that matches when normalized
        search_line_count = len(search.strip().split('\n'))
        matched = False

        for start in range(len(lines)):
            end = min(start + search_line_count + 2, len(lines))  # +2 for tolerance
            for e in range(start + 1, end + 1):
                candidate = '\n'.join(lines[start:e])
                candidate_normalized = re.sub(r'[ \t]+', ' ', candidate.strip())
                if candidate_normalized == search_normalized:
                    result = result.replace(candidate, replace, 1)
                    print(f"[ai_edit] Applied edit {i}: whitespace-normalized match (lines {start+1}-{e})", file=sys.stderr)
                    matched = True
                    break
            if matched:
                break

        if not matched:
            print(f"[ai_edit] WARNING: Could not find search text for edit {i}: {search[:80]!r}...", file=sys.stderr)

    return result


def generate_edit(prompt: str, current_urdf: str, kinematic_graph_json: dict,
                   kinematic_context: str = None, session_id: str = "default") -> dict:
    """
    Call Claude API to generate a robot edit based on natural language.
    Maintains conversation history per session for multi-turn context.

    Args:
        prompt: User's natural language edit request
        current_urdf: Current URDF XML as string
        kinematic_graph_json: Kinematic graph as dict (from kg.to_json())
        kinematic_context: Optional structured text summary of robot structure from frontend
        session_id: Session identifier for conversation history tracking

    Returns:
        Dict with keys:
        - "explanation": str
        - "new_urdf": str
        - "stats": str

    Raises:
        ImportError: If anthropic package is not installed
        ValueError: If ANTHROPIC_API_KEY env var not set
        Exception: On API errors or JSON parsing issues
    """
    client = _get_client()

    # Build user message with available context
    user_message = f"""Current URDF:
```xml
{current_urdf}
```

Kinematic Graph:
```json
{json.dumps(kinematic_graph_json, indent=2)}
```"""

    if kinematic_context:
        user_message += f"""

Robot Structure Summary:
{kinematic_context}"""

    user_message += f"""

User Request: {prompt}"""

    # Build messages array with conversation history
    history = _conversation_history[session_id]
    messages = list(history) + [{"role": "user", "content": user_message}]

    response = client.messages.create(
        model="claude-sonnet-4-20250514",
        max_tokens=65536,
        system=SYSTEM_PROMPT.replace("{COMPONENT_CATALOG}", _get_component_catalog()),
        messages=messages,
        timeout=180.0,
    )

    # Parse the response
    response_text = response.content[0].text

    # Store conversation turn in history (compact: just prompt + explanation, not full URDF)
    history.append({"role": "user", "content": f"[Edit request] {prompt}"})
    history.append({"role": "assistant", "content": response_text})

    # Trim history to cap
    while len(history) > _MAX_HISTORY_MESSAGES:
        history.pop(0)

    # Try to extract JSON from the response (handle markdown code blocks)
    result = _parse_json_response(response_text)

    # Check if response uses full_urdf (Option B: complete replacement)
    if "full_urdf" in result and result["full_urdf"]:
        new_urdf = result["full_urdf"]
        print(f"[ai_edit] Using full_urdf replacement ({len(new_urdf)} chars)", file=sys.stderr)
    else:
        # Apply search/replace edits (Option A: incremental)
        edits = result.get("edits", [])
        new_urdf = _apply_edits(current_urdf, edits)

    return {
        "explanation": result.get("explanation", "Changes applied"),
        "new_urdf": new_urdf,
        "stats": result.get("changes_summary", "Edit complete"),
    }


def generate_edit_streaming(prompt: str, current_urdf: str, kinematic_graph_json: dict,
                             kinematic_context: str = None, session_id: str = "default",
                             on_progress=None) -> dict:
    """
    Streaming version of generate_edit. Calls on_progress(stage, text) as tokens arrive.
    Stages: "thinking", "generating", "applying"
    """
    client = _get_client()

    user_message = f"""Current URDF:
```xml
{current_urdf}
```

Kinematic Graph:
```json
{json.dumps(kinematic_graph_json, indent=2)}
```"""

    if kinematic_context:
        user_message += f"""

Robot Structure Summary:
{kinematic_context}"""

    user_message += f"""

User Request: {prompt}"""

    history = _conversation_history[session_id]
    messages = list(history) + [{"role": "user", "content": user_message}]

    if on_progress:
        on_progress("thinking", "Analyzing model...")

    # Use streaming API
    accumulated_text = ""
    try:
        with client.messages.stream(
            model="claude-sonnet-4-20250514",
            max_tokens=65536,
            system=SYSTEM_PROMPT.replace("{COMPONENT_CATALOG}", _get_component_catalog()),
            messages=messages,
        ) as stream:
            token_count = 0
            sent_generating = False
            for text in stream.text_stream:
                accumulated_text += text
                token_count += 1

                if not sent_generating and token_count > 2:
                    if on_progress:
                        on_progress("generating", "Generating design...")
                    sent_generating = True

                # Stream partial explanation every ~8 tokens for smooth updates
                if on_progress and token_count % 8 == 0:
                    partial = _extract_partial_explanation(accumulated_text)
                    if partial:
                        on_progress("streaming", partial)

    except Exception as e:
        raise ValueError(f"Streaming API call failed: {e}")

    response_text = accumulated_text

    if on_progress:
        on_progress("applying", "Applying changes...")

    # Store conversation history
    history.append({"role": "user", "content": f"[Edit request] {prompt}"})
    history.append({"role": "assistant", "content": response_text})
    while len(history) > _MAX_HISTORY_MESSAGES:
        history.pop(0)

    # Parse and apply
    result = _parse_json_response(response_text)

    if "full_urdf" in result and result["full_urdf"]:
        new_urdf = result["full_urdf"]
        print(f"[ai_edit] Using full_urdf replacement ({len(new_urdf)} chars)", file=sys.stderr)
    else:
        edits = result.get("edits", [])
        new_urdf = _apply_edits(current_urdf, edits)

    return {
        "explanation": result.get("explanation", "Changes applied"),
        "new_urdf": new_urdf,
        "stats": result.get("changes_summary", "Edit complete"),
    }


def _extract_partial_explanation(text: str) -> str:
    """Try to extract the explanation field from partial JSON for live preview."""
    # Look for "explanation": "..." pattern
    match = re.search(r'"explanation"\s*:\s*"((?:[^"\\]|\\.)*)', text)
    if match:
        return match.group(1).replace('\\"', '"').replace('\\n', ' ')
    return ""


def clear_conversation(session_id: str = "default") -> None:
    """Clear conversation history for a session."""
    _conversation_history.pop(session_id, None)


def _parse_json_response(text: str) -> dict:
    """
    Parse JSON from Claude's response, handling markdown code blocks.

    Args:
        text: Response text that may contain JSON with or without markdown

    Returns:
        Parsed JSON dict

    Raises:
        ValueError: If JSON cannot be parsed
    """
    # Try direct JSON parsing first
    try:
        return json.loads(text)
    except json.JSONDecodeError:
        pass

    # Try to extract JSON from markdown code blocks
    # Look for ```json ... ``` or ``` ... ```
    patterns = [
        r"```(?:json)?\s*\n?(.*?)\n?```",
        r"```(.*?)```",
    ]

    for pattern in patterns:
        matches = re.findall(pattern, text, re.DOTALL)
        for match in matches:
            try:
                return json.loads(match)
            except json.JSONDecodeError:
                continue

    # If all else fails, raise error with context
    raise ValueError(
        f"Failed to parse JSON from response. Response text:\n{text[:500]}"
    )


def _postprocess_completion(completion: str, cursor_prefix: str) -> str:
    """
    Fix common model mistakes in completions by examining what the prefix
    ends with and what the completion starts with.
    """
    if not completion or not cursor_prefix:
        return completion

    prefix = cursor_prefix.rstrip('\n')  # preserve spaces, strip trailing newlines

    # ── Fix 1: Missing space before XML attribute ──
    # If prefix ends with a tag name like `<link` or `<joint` and the completion
    # starts with an attribute like `name=` or `type=`, inject a space.
    # Also handles `</link` or `</joint` that shouldn't get attributes at all.
    if completion and completion[0] not in (' ', '\n', '\t', '>', '/', '"'):
        # Check if prefix ends with an XML tag name (opening or closing)
        tag_match = re.search(r'</?(\w+)$', prefix)
        if tag_match:
            # Check if completion starts with an attribute-like pattern
            if re.match(r'\w+=', completion) or re.match(r'\w+\s', completion):
                completion = ' ' + completion
                print(f"[ai_complete] Post-fix: injected space after tag name", file=sys.stderr)

    # ── Fix 2: Strip stray "> on its own or after closing tags ──
    # The model sometimes leaves orphaned "> from broken tag constructions
    completion = re.sub(r'(</\w+>)\s*">', r'\1', completion)
    # Also strip lines that are ONLY "> or "/>
    completion = re.sub(r'\n\s*">\s*$', '', completion)
    completion = re.sub(r'^\s*">\s*\n', '', completion)

    # ── Fix 3: Detect repeated/duplicate content ──
    # If the completion contains a block of lines that duplicates lines already
    # present earlier in the completion (the model looping), truncate at the repeat.
    lines = completion.split('\n')
    if len(lines) > 4:
        seen_stripped: list[str] = []
        for i, line in enumerate(lines):
            s = line.strip()
            if not s:
                seen_stripped.append(s)
                continue
            # Check if this line + next line match a pair we already saw
            # (single line matches are too aggressive — common XML like </geometry> repeats legitimately)
            if i > 0 and i < len(lines) - 1:
                pair = (s, lines[i + 1].strip())
                for j in range(len(seen_stripped) - 1):
                    if (seen_stripped[j], seen_stripped[j + 1]) == pair and s not in ('', '<visual>', '<collision>', '<geometry>'):
                        print(f"[ai_complete] Post-fix: truncated at repeated content (line {i}): {s[:50]}", file=sys.stderr)
                        completion = '\n'.join(lines[:i])
                        return completion
            seen_stripped.append(s)

    # ── Fix 4: Validate tag nesting ──
    # If completion closes a tag that was never opened within the completion
    # AND is not the expected parent close, truncate there.
    lines = completion.split('\n')
    tag_stack: list[str] = []
    valid_lines: list[str] = []
    # Allow exactly one "parent close" (closing a tag from the prefix, like </link>)
    parent_closes_allowed = 1
    parent_closes_seen = 0

    for line in lines:
        stripped_line = line.strip()

        opens = re.findall(r'<(\w+)[\s>]', stripped_line)
        self_closing = re.findall(r'<(\w+)\s[^>]*/>', stripped_line)
        closes = re.findall(r'</(\w+)>', stripped_line)
        real_opens = [t for t in opens if t not in self_closing]

        bad_close = False
        for tag in closes:
            if tag in tag_stack:
                idx = len(tag_stack) - 1 - tag_stack[::-1].index(tag)
                tag_stack = tag_stack[:idx]
            elif not tag_stack:
                parent_closes_seen += 1
                if parent_closes_seen > parent_closes_allowed:
                    bad_close = True
            else:
                bad_close = True

        if bad_close:
            print(f"[ai_complete] Post-fix: truncated at bad nesting: {stripped_line[:50]}", file=sys.stderr)
            break

        for tag in real_opens:
            tag_stack.append(tag)
        valid_lines.append(line)

    if len(valid_lines) < len(lines):
        completion = '\n'.join(valid_lines)

    return completion


def _is_low_quality_completion(completion: str, cursor_prefix: str, context_after: str) -> bool:
    """
    Filter out low-value completions that would annoy more than help.
    Returns True if the completion should be suppressed.
    """
    stripped = completion.strip()

    # Too short to be useful (< 3 non-whitespace chars)
    if len(stripped) < 3:
        return True

    # Just closing a tag the user is already typing
    # e.g., prefix ends with `</link` and completion is just `>`
    if stripped in ('>', '/>', '">', '"/>'):
        return True

    # Completion is just a closing tag for something the user already has
    # e.g., prefix has `</collision>` and completion is `\n  </link>` — that's just boilerplate closing
    if re.match(r'^\s*</\w+>\s*$', stripped):
        # Check if this closing tag already exists right after cursor
        close_tag = stripped.strip()
        after_stripped = context_after.lstrip()
        if after_stripped.startswith(close_tag):
            return True

    # Completion is identical to what's already after the cursor (pure duplication)
    if stripped and context_after.strip().startswith(stripped):
        return True

    return False


def _strip_suffix_overlap(completion: str, suffix: str) -> str:
    """
    Remove the longest tail of `completion` that matches a prefix of `suffix`.
    This prevents duplication when the model's output continues into code that
    already exists after the cursor.

    Uses line-level matching first (faster, handles the common case), then
    falls back to character-level for partial-line overlaps.
    """
    if not completion or not suffix:
        return completion

    # Normalize for comparison (strip trailing whitespace per line)
    comp_lines = completion.split('\n')
    suf_lines = suffix.split('\n')

    # Line-level overlap: find the longest tail of comp_lines that matches
    # the head of suf_lines (comparing stripped content to be whitespace-tolerant)
    best_overlap = 0
    for overlap_len in range(1, min(len(comp_lines), len(suf_lines)) + 1):
        tail = [l.strip() for l in comp_lines[-overlap_len:]]
        head = [l.strip() for l in suf_lines[:overlap_len]]
        if tail == head:
            best_overlap = overlap_len

    if best_overlap > 0:
        trimmed = '\n'.join(comp_lines[:-best_overlap])
        print(f"[ai_complete] Stripped {best_overlap} overlapping lines from completion", file=sys.stderr)
        return trimmed

    # Character-level fallback: find longest suffix of completion that matches
    # prefix of suffix text (for partial-line overlaps)
    max_check = min(len(completion), len(suffix), 200)
    for length in range(max_check, 0, -1):
        if completion[-length:] == suffix[:length]:
            print(f"[ai_complete] Stripped {length} overlapping chars from completion", file=sys.stderr)
            return completion[:-length]

    return completion


# ── Completion cache ──────────────────────────────────────────────────────────
# Simple LRU-style cache keyed by (cursor_prefix_hash, context_hash).
# Avoids hitting the API when the user triggers completion at the same position.
import hashlib
_completion_cache: dict[str, str] = {}
_CACHE_MAX_SIZE = 50


def _cache_key(cursor_prefix: str, context_before: str, context_after: str) -> str:
    """Generate a cache key from the cursor context."""
    raw = f"{cursor_prefix}||{context_before[-200:]}||{context_after[:200]}"
    return hashlib.md5(raw.encode()).hexdigest()


def _find_parent_element(urdf_content: str, cursor_index: int) -> tuple[str, str]:
    """
    XML-aware context extraction: find the innermost open element containing
    the cursor, then return (before_context, after_context) that includes
    the full parent element boundaries.

    Falls back to flat char windows if parsing fails.
    """
    try:
        # Walk backwards from cursor to find the nearest unclosed opening tag
        before = urdf_content[:cursor_index]
        after = urdf_content[cursor_index:]

        # Find all opening and closing tags before cursor
        open_stack: list[tuple[str, int]] = []  # (tag_name, position)
        for m in re.finditer(r'<(/?)(\w+)[\s>]', before):
            is_close = m.group(1) == '/'
            tag_name = m.group(2)
            if is_close:
                # Pop matching open
                for i in range(len(open_stack) - 1, -1, -1):
                    if open_stack[i][0] == tag_name:
                        open_stack.pop(i)
                        break
            else:
                # Check if self-closing
                # Find the end of this tag
                tag_start = m.start()
                tag_end_search = before[tag_start:]
                if '/>' in tag_end_search.split('>')[0] if '>' in tag_end_search else False:
                    continue  # self-closing, skip
                open_stack.append((tag_name, tag_start))

        if open_stack:
            # Use the innermost parent element's start as context anchor
            parent_tag, parent_pos = open_stack[-1]
            # Include from parent start, but cap at reasonable size
            context_start = max(0, parent_pos - 200)
            context_before = urdf_content[context_start:cursor_index]

            # Find the matching close tag after cursor
            close_pattern = f'</{parent_tag}>'
            close_match = re.search(re.escape(close_pattern), after)
            if close_match:
                context_end = cursor_index + close_match.end() + 200
                context_after = urdf_content[cursor_index:min(len(urdf_content), context_end)]
                return context_before, context_after

    except Exception as e:
        print(f"[ai_complete] XML-aware context failed, using flat window: {e}", file=sys.stderr)

    # Fallback: flat character windows
    context_before = urdf_content[max(0, cursor_index - 2000):cursor_index]
    context_after = urdf_content[cursor_index:min(len(urdf_content), cursor_index + 1000)]
    return context_before, context_after


def generate_completion(
    urdf_content: str,
    cursor_line: int,
    cursor_column: int,
    prefix: str = "",
    kinematic_context: str = ""
) -> str:
    """
    Generate an inline completion suggestion for URDF/XML editing.

    Uses XML-aware context extraction, response caching, and quality filtering.

    Args:
        urdf_content: Current URDF XML as string
        cursor_line: Cursor line number (1-indexed)
        cursor_column: Cursor column number (1-indexed)
        prefix: Optional recent characters typed (for additional context)
        kinematic_context: Optional structured summary of robot structure

    Returns:
        Completion text string (what to insert at cursor)
    """
    global _completion_cache
    client = _get_client()

    # Convert cursor position to character index
    lines = urdf_content.split('\n')
    cursor_index = sum(len(line) + 1 for line in lines[:cursor_line - 1]) + (cursor_column - 1)

    # XML-aware context extraction — includes full parent element
    context_before, context_after = _find_parent_element(urdf_content, cursor_index)

    # Also get broader context for the prompt (sibling elements, etc.)
    broad_before = urdf_content[max(0, cursor_index - 4000):cursor_index]
    broad_after = urdf_content[cursor_index:min(len(urdf_content), cursor_index + 2000)]

    # Extract immediate prefix (last ~100 chars) — most important signal
    cursor_prefix = broad_before[-100:] if len(broad_before) > 100 else broad_before

    # ── Check cache ──
    cache_k = _cache_key(cursor_prefix, context_before, context_after)
    if cache_k in _completion_cache:
        cached = _completion_cache[cache_k]
        print(f"[ai_complete] Cache hit ({len(cached)} chars)", file=sys.stderr)
        return cached

    # Build prompt with cursor prefix as the most prominent field
    user_message = f"""===CURSOR_PREFIX===
{cursor_prefix}
===END_CURSOR_PREFIX===

===CONTEXT===
...{context_before[-800:]}|CURSOR|{context_after[:800]}...
===END_CONTEXT==="""

    if kinematic_context:
        user_message += f"""

===ROBOT===
{kinematic_context}
===END_ROBOT==="""

    print(f"[ai_complete] Calling Claude Haiku at line {cursor_line} (prefix: {len(cursor_prefix)}c, ctx: {len(context_before)}+{len(context_after)}c)...", file=sys.stderr)
    t0 = time.time()

    response = client.messages.create(
        model="claude-haiku-4-5-20251001",
        max_tokens=600,
        temperature=0,
        system=COMPLETION_SYSTEM_PROMPT,
        messages=[{"role": "user", "content": user_message}],
        stop_sequences=["</robot>", "<!--"],
        timeout=10.0,
    )

    elapsed = time.time() - t0
    print(f"[ai_complete] Claude responded in {elapsed:.1f}s", file=sys.stderr)

    completion_text = response.content[0].text

    # Strip markdown artifacts
    if completion_text.startswith('```'):
        inner = completion_text.split('```')
        completion_text = inner[1] if len(inner) > 1 else completion_text
        if completion_text.startswith('xml\n'):
            completion_text = completion_text[4:]
        elif completion_text.startswith('xml'):
            completion_text = completion_text[3:]
    if completion_text.endswith('```'):
        completion_text = completion_text.rsplit('```', 1)[0]

    # Reject prose responses
    stripped = completion_text.strip()
    if stripped and stripped[0].isupper() and len(stripped) > 10:
        first_word = stripped.split()[0] if stripped.split() else ""
        prose_starters = {"The", "This", "Here", "I", "You", "To", "In", "It", "Note", "For", "A", "An"}
        if first_word in prose_starters:
            print(f"[ai_complete] Rejected prose response: {stripped[:50]}", file=sys.stderr)
            return ""

    # Post-process: fix common model mistakes
    completion_text = _postprocess_completion(completion_text, cursor_prefix)

    # Strip overlap with existing code after cursor
    completion_text = _strip_suffix_overlap(completion_text, broad_after)

    # Quality filter: suppress low-value completions
    if _is_low_quality_completion(completion_text, cursor_prefix, broad_after):
        print(f"[ai_complete] Filtered low-quality completion: {completion_text.strip()[:50]!r}", file=sys.stderr)
        return ""

    # Cache the result
    _completion_cache[cache_k] = completion_text
    if len(_completion_cache) > _CACHE_MAX_SIZE:
        # Evict oldest entry
        oldest_key = next(iter(_completion_cache))
        del _completion_cache[oldest_key]

    return completion_text
