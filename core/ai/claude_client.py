"""
Claude AI client for robot design editing.
Communicates with Claude API to generate robot model edits based on natural language requests.
"""
import json
import os
import re
import sys
import time

# Lazy import — anthropic may not be installed
_anthropic = None
_anthropic_error = None
try:
    import anthropic
    _anthropic = anthropic
except ImportError as e:
    _anthropic_error = str(e)


SYSTEM_PROMPT = """You are a robot design assistant for Vector IDE. You receive:
1. The current URDF XML of a robot
2. The kinematic graph (links, joints, masses, geometries)
3. A natural language edit request from the user

You must return ONLY valid JSON (no markdown, no code blocks) with this structure:
{
    "explanation": "Human-readable description of changes made",
    "new_urdf": "The complete modified URDF XML",
    "changes_summary": "Short stats like 'Modified 2 links, added 1 joint'"
}

Rules:
- Only make the changes the user requested, preserve everything else exactly
- Ensure physical plausibility (reasonable masses, dimensions in meters, etc.)
- Maintain valid URDF XML structure with proper nesting
- Return the COMPLETE modified URDF, not a partial patch
- Keep all existing comments and formatting where possible
- If adding new links, include proper inertial, visual, and collision elements
- Use SI units: meters, kilograms, radians
"""

COMPLETION_SYSTEM_PROMPT = """You are a URDF/XML code completion engine. You receive code with a <FILL> marker where the cursor is.
Output ONLY the code that replaces <FILL>. Rules:
- Output raw XML code only. Never output English text, explanations, markdown, or code fences.
- Complete 1-3 lines maximum. Stop at a natural boundary (closing tag, end of element).
- Be context-aware: use existing link/joint names, maintain consistent naming and indentation.
- If the cursor is mid-attribute, complete just the attribute value and closing quote.
- If the cursor is at a new line inside an element, complete the next child element.
- Match the indentation style of the surrounding code exactly."""


def generate_edit(prompt: str, current_urdf: str, kinematic_graph_json: dict, kinematic_context: str = None) -> dict:
    """
    Call Claude API to generate a robot edit based on natural language.

    Args:
        prompt: User's natural language edit request
        current_urdf: Current URDF XML as string
        kinematic_graph_json: Kinematic graph as dict (from kg.to_json())
        kinematic_context: Optional structured text summary of robot structure from frontend

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
    if _anthropic is None:
        raise ImportError(
            f"anthropic package not installed. Run: pip install anthropic\n"
            f"Error: {_anthropic_error}"
        )

    api_key = os.environ.get("ANTHROPIC_API_KEY")
    if not api_key:
        raise ValueError("ANTHROPIC_API_KEY environment variable not set")

    client = _anthropic.Anthropic(api_key=api_key)

    # Build user message with available context
    user_message = f"""Current URDF:
```xml
{current_urdf}
```

Kinematic Graph:
```json
{json.dumps(kinematic_graph_json, indent=2)}
```"""

    # Include frontend-provided kinematic context if available
    if kinematic_context:
        user_message += f"""

Robot Structure Summary:
{kinematic_context}"""

    user_message += f"""

User Request: {prompt}"""

    response = client.messages.create(
        model="claude-sonnet-4-20250514",
        max_tokens=8192,
        system=SYSTEM_PROMPT,
        messages=[{"role": "user", "content": user_message}],
        timeout=120.0,
    )

    # Parse the response
    response_text = response.content[0].text

    # Try to extract JSON from the response (handle markdown code blocks)
    result = _parse_json_response(response_text)

    return {
        "explanation": result.get("explanation", "Changes applied"),
        "new_urdf": result.get("new_urdf", current_urdf),
        "stats": result.get("changes_summary", "Edit complete"),
    }


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


def generate_completion(
    urdf_content: str,
    cursor_line: int,
    cursor_column: int,
    prefix: str = ""
) -> str:
    """
    Generate an inline completion suggestion for URDF/XML editing.

    This is optimized for speed and responsiveness — uses a fast model,
    short timeout, and minimal prompt.

    Args:
        urdf_content: Current URDF XML as string
        cursor_line: Cursor line number (1-indexed)
        cursor_column: Cursor column number (1-indexed)
        prefix: Optional recent characters typed (for additional context)

    Returns:
        Completion text string (what to insert at cursor)

    Raises:
        ImportError: If anthropic package is not installed
        ValueError: If ANTHROPIC_API_KEY env var not set
        Exception: On API errors or timeouts
    """
    if _anthropic is None:
        raise ImportError(
            f"anthropic package not installed. Run: pip install anthropic\n"
            f"Error: {_anthropic_error}"
        )

    api_key = os.environ.get("ANTHROPIC_API_KEY")
    if not api_key:
        raise ValueError("ANTHROPIC_API_KEY environment variable not set")

    client = _anthropic.Anthropic(api_key=api_key)

    # Convert cursor position to character index
    lines = urdf_content.split('\n')
    cursor_index = sum(len(line) + 1 for line in lines[:cursor_line - 1]) + (cursor_column - 1)

    # FIM: Extract substantial context before AND after cursor
    context_before = urdf_content[max(0, cursor_index - 1500):cursor_index]
    context_after = urdf_content[cursor_index:min(len(urdf_content), cursor_index + 500)]

    # Build FIM prompt — code before + marker + code after
    user_message = f"""{context_before}<FILL>{context_after}"""

    print(f"[ai_complete] Calling Claude Haiku at line {cursor_line} (prefix: {len(context_before)}c, suffix: {len(context_after)}c)...", file=sys.stderr)
    t0 = time.time()

    response = client.messages.create(
        model="claude-haiku-4-5-20251001",
        max_tokens=120,        # Short completions — 1-3 lines
        temperature=0,         # Deterministic, no creativity
        system=COMPLETION_SYSTEM_PROMPT,
        messages=[{"role": "user", "content": user_message}],
        stop_sequences=["\n\n", "</robot>", "<!--"],  # Stop at natural boundaries
        timeout=8.0,
    )

    elapsed = time.time() - t0
    print(f"[ai_complete] Claude responded in {elapsed:.1f}s", file=sys.stderr)

    # Extract raw completion text
    completion_text = response.content[0].text

    # Strip markdown artifacts if the model wrapped output
    if completion_text.startswith('```'):
        inner = completion_text.split('```')
        completion_text = inner[1] if len(inner) > 1 else completion_text
        if completion_text.startswith('xml\n'):
            completion_text = completion_text[4:]
        elif completion_text.startswith('xml'):
            completion_text = completion_text[3:]
    if completion_text.endswith('```'):
        completion_text = completion_text.rsplit('```', 1)[0]

    # If the response looks like prose (starts with a capital letter and space),
    # it's probably an explanation — reject it
    stripped = completion_text.strip()
    if stripped and stripped[0].isupper() and len(stripped) > 10:
        first_word = stripped.split()[0] if stripped.split() else ""
        prose_starters = {"The", "This", "Here", "I", "You", "To", "In", "It", "Note", "For", "A", "An"}
        if first_word in prose_starters:
            print(f"[ai_complete] Rejected prose response: {stripped[:50]}", file=sys.stderr)
            return ""

    return completion_text
