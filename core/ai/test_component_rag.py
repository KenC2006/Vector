"""Unit tests for the component RAG retrieval."""

import os

import pytest

from core.ai import component_rag


@pytest.fixture(autouse=True)
def _reset_rag_singleton():
    component_rag.reset_for_tests()
    yield
    component_rag.reset_for_tests()


def test_index_builds_and_covers_known_components():
    rag = component_rag.get_rag()
    assert rag.n_documents > 100, "preset corpus should have well over 100 entries"
    # The backend the tests run against in CI is sparse — sentence-transformers
    # is an optional dep that's not installed in the default venv.
    assert rag.backend in ("dense-MiniLM-L6-v2", "tfidf")


def test_paraphrastic_query_retrieves_camera_for_eyes():
    """Sanity check that 'eyes' or 'vision' surfaces a camera preset.

    Tolerant: TF-IDF only matches if the word literally appears in the
    description (it does — most camera presets mention 'vision' or 'eye').
    Dense embeddings should be more robust. Either way, top-10 must include
    at least one sensor preset that's plausibly a camera.
    """
    rag = component_rag.get_rag()
    hits = rag.retrieve("I need eyes so the robot can see", k=10)
    ids = [pid for pid, _ in hits]
    assert any("camera" in pid or "depth" in pid for pid in ids), (
        f"camera-like preset missing from top-10 for vision query: {ids}"
    )


def test_servo_query_returns_actuators():
    rag = component_rag.get_rag()
    hits = rag.retrieve("strong servo for shoulder joint", k=5)
    ids = [pid for pid, _ in hits]
    assert any(pid.startswith("actuator_servo") for pid in ids), (
        f"servo not in top-5 for servo query: {ids}"
    )


def test_empty_query_returns_empty():
    rag = component_rag.get_rag()
    assert rag.retrieve("", k=10) == []
    assert rag.retrieve("   ", k=10) == []


def test_disabled_by_default(monkeypatch):
    monkeypatch.delenv("VECTOR_RAG_ENABLED", raising=False)
    assert component_rag.is_enabled() is False


def test_enabled_via_env(monkeypatch):
    monkeypatch.setenv("VECTOR_RAG_ENABLED", "1")
    assert component_rag.is_enabled() is True
    monkeypatch.setenv("VECTOR_RAG_ENABLED", "true")
    assert component_rag.is_enabled() is True
    monkeypatch.setenv("VECTOR_RAG_ENABLED", "0")
    assert component_rag.is_enabled() is False
