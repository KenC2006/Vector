"""Shared Anthropic client for the designer and controller agents."""
from __future__ import annotations

import os

_client = None


def get_client():
    """Return a process-wide Anthropic client (one HTTP connection pool)."""
    global _client
    if _client is None:
        try:
            import anthropic
        except ImportError as e:
            raise ImportError(f"anthropic package not installed. Run: pip install anthropic ({e})")
        api_key = os.environ.get("ANTHROPIC_API_KEY")
        if not api_key:
            raise ValueError("ANTHROPIC_API_KEY environment variable not set")
        # Org-level (unscoped) keys must name the workspace on every request.
        workspace_id = os.environ.get("ANTHROPIC_WORKSPACE_ID")
        headers = {"anthropic-workspace-id": workspace_id} if workspace_id else None
        _client = anthropic.Anthropic(api_key=api_key, default_headers=headers)
    return _client
