"""
Retrieval-Augmented Generation (RAG) over the component preset catalog.

The Claude system prompt today either dumps the full catalog or relies on a
keyword-scored subset (`catalog_selector.build_scoped_catalog`). Both miss
paraphrastic queries — "see in low light" → IR camera, "absorb shock" →
rubber foot, "carry payload" → load cell — because the keyword index has
no notion of semantic similarity.

This module adds a retrieval pass that ranks presets by cosine similarity
between the user prompt and each preset's authored description. Two
backends, picked at module load:

  - dense  (preferred): sentence-transformers all-MiniLM-L6-v2 embeddings.
            Captures paraphrase, ~22MB model, ~5ms per query after warmup.
  - sparse (fallback):  pure-numpy TF-IDF with cosine similarity. Zero new
            dependencies, faster, less robust to paraphrase. Good enough
            for the 147-component corpus and used in tests so CI doesn't
            need the embedding model.

Either backend exposes the same retrieve(query, k) signature. The catalog
selector consumes top-K IDs as a relevance boost merged with the existing
keyword score.

Activation:
  VECTOR_RAG_ENABLED=1   turn on retrieval inside build_scoped_catalog
  VECTOR_RAG_BACKEND=tfidf|dense   force a backend (default: try dense, fall
                                   back to tfidf)

Index lifecycle:
  Built lazily on first call. Cached in-process — no disk persistence (the
  full build is ~30ms TF-IDF, ~1.5s dense; both are amortized over the
  lifetime of the assembler process).
"""

from __future__ import annotations

import json
import os
import re
from pathlib import Path
from typing import Optional

import numpy as np

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
PRESETS_PATH = REPO_ROOT / "core" / "presets" / "generic_presets.json"

_BACKEND_FORCE = os.environ.get("VECTOR_RAG_BACKEND", "").strip().lower()


# ── Document construction ────────────────────────────────────────────────────

def _spec_summary(component: dict) -> str:
    """Flatten mechanical/electrical fields into a short token-rich blob.

    Numeric specs get included as 'key=value' so units/quantities survive
    tokenization. Keeps the document terse enough that one preset doesn't
    dominate IDF — typical doc is ~30-80 tokens after tokenization.
    """
    me = component.get("mechanical_electrical", {})
    bits: list[str] = []
    for k, v in me.items():
        if isinstance(v, (str, int, float)):
            bits.append(f"{k}={v}")
        elif isinstance(v, list) and all(isinstance(x, (int, float)) for x in v):
            bits.append(f"{k}={'-'.join(str(x) for x in v)}")
    return " ".join(bits)


def _build_document(component: dict) -> str:
    """Concatenate the retrievable surface of a preset.

    Document order matters for sparse retrieval: the highest-signal terms
    (name, description) come first so they get clipped last if a downstream
    backend imposes a length limit.
    """
    name = component.get("name", "")
    desc = component.get("description", "")
    cid = component.get("id", "")
    spec = _spec_summary(component)
    return f"{name}. {desc} [{cid}] {spec}".strip()


# ── Sparse backend: TF-IDF + cosine ──────────────────────────────────────────

_WORD_RE = re.compile(r"[a-z0-9]+")


def _tokenize(text: str) -> list[str]:
    return _WORD_RE.findall(text.lower()) if text else []


def _build_tfidf(documents: list[str]) -> tuple[np.ndarray, dict[str, int], np.ndarray]:
    """Compute an L2-normalized TF-IDF matrix over `documents`.

    Returns (matrix, vocab, idf):
      matrix: (n_docs, n_terms) float32, each row unit-norm
      vocab:  token → column index
      idf:    (n_terms,) float32 inverse-document-frequency vector
    """
    tokenized = [_tokenize(d) for d in documents]
    vocab: dict[str, int] = {}
    for tokens in tokenized:
        for tok in tokens:
            if tok not in vocab:
                vocab[tok] = len(vocab)
    n_docs = len(documents)
    n_terms = len(vocab)
    tf = np.zeros((n_docs, n_terms), dtype=np.float32)
    for i, tokens in enumerate(tokenized):
        for tok in tokens:
            tf[i, vocab[tok]] += 1
    df = (tf > 0).sum(axis=0).astype(np.float32)
    idf = np.log((n_docs + 1.0) / (df + 1.0)) + 1.0
    weighted = tf * idf
    norms = np.linalg.norm(weighted, axis=1, keepdims=True)
    norms[norms == 0] = 1.0
    return (weighted / norms).astype(np.float32), vocab, idf.astype(np.float32)


def _tfidf_query_vector(query: str, vocab: dict[str, int], idf: np.ndarray) -> np.ndarray:
    """Embed a query string into the same TF-IDF space as the corpus."""
    tokens = _tokenize(query)
    vec = np.zeros(len(vocab), dtype=np.float32)
    for tok in tokens:
        col = vocab.get(tok)
        if col is not None:
            vec[col] += 1.0
    vec = vec * idf
    n = float(np.linalg.norm(vec))
    if n > 0:
        vec = vec / n
    return vec


# ── Dense backend: sentence-transformers (optional) ──────────────────────────

def _try_load_dense_model():
    """Return a SentenceTransformer instance, or None if the dep is missing
    or model load fails (e.g. no network on first run)."""
    if _BACKEND_FORCE == "tfidf":
        return None
    try:
        from sentence_transformers import SentenceTransformer  # type: ignore
        return SentenceTransformer("sentence-transformers/all-MiniLM-L6-v2")
    except Exception:
        return None


# ── Public class ─────────────────────────────────────────────────────────────

class ComponentRAG:
    """Cosine-similarity retrieval over the component catalog.

    Built once per process. Exposes retrieve(query, k) returning a ranked
    list of (component_id, similarity_score) tuples.
    """

    def __init__(self) -> None:
        self._ids: list[str] = []
        self._docs: list[str] = []
        self._dense_model = None
        self._dense_matrix: Optional[np.ndarray] = None
        self._tfidf_matrix: Optional[np.ndarray] = None
        self._tfidf_vocab: Optional[dict[str, int]] = None
        self._tfidf_idf: Optional[np.ndarray] = None
        self._build()

    def _build(self) -> None:
        with open(PRESETS_PATH, encoding="utf-8") as f:
            data = json.load(f)
        for cat in data.get("categories", {}).values():
            for comp in cat.get("components", []):
                cid = comp.get("id")
                if not cid:
                    continue
                self._ids.append(cid)
                self._docs.append(_build_document(comp))

        # Try dense first; fall back to sparse on any failure.
        self._dense_model = _try_load_dense_model()
        if self._dense_model is not None:
            try:
                emb = self._dense_model.encode(
                    self._docs, normalize_embeddings=True, show_progress_bar=False
                )
                self._dense_matrix = np.asarray(emb, dtype=np.float32)
            except Exception:
                self._dense_model = None
                self._dense_matrix = None

        if self._dense_matrix is None:
            self._tfidf_matrix, self._tfidf_vocab, self._tfidf_idf = _build_tfidf(self._docs)

    @property
    def backend(self) -> str:
        return "dense-MiniLM-L6-v2" if self._dense_matrix is not None else "tfidf"

    @property
    def n_documents(self) -> int:
        return len(self._ids)

    def retrieve(self, query: str, k: int = 20) -> list[tuple[str, float]]:
        """Return the top-`k` (component_id, score) pairs for `query`.

        Score is cosine similarity in [0, 1] for both backends (matrix rows
        are L2-normalized at index time and queries are normalized inline).
        Empty/whitespace queries return [] without computation — the keyword
        path in catalog_selector handles the no-signal fallback.
        """
        if not query or not query.strip() or not self._ids:
            return []
        if self._dense_matrix is not None and self._dense_model is not None:
            q = self._dense_model.encode(
                [query], normalize_embeddings=True, show_progress_bar=False
            )
            q_vec = np.asarray(q[0], dtype=np.float32)
            scores = self._dense_matrix @ q_vec
        else:
            assert self._tfidf_matrix is not None and self._tfidf_vocab is not None and self._tfidf_idf is not None
            q_vec = _tfidf_query_vector(query, self._tfidf_vocab, self._tfidf_idf)
            scores = self._tfidf_matrix @ q_vec
        order = np.argsort(-scores)[:k]
        return [(self._ids[i], float(scores[i])) for i in order if scores[i] > 0.0]


_singleton: Optional[ComponentRAG] = None


def get_rag() -> ComponentRAG:
    """Return the process-wide ComponentRAG instance, building it on first call."""
    global _singleton
    if _singleton is None:
        _singleton = ComponentRAG()
    return _singleton


def reset_for_tests() -> None:
    """Drop the cached singleton so a fresh build runs in the next test."""
    global _singleton
    _singleton = None


def is_enabled() -> bool:
    """True when VECTOR_RAG_ENABLED is set to a truthy value.

    The catalog selector reads this to decide whether to merge RAG hits into
    its scoring. Off by default so the existing keyword path stays the
    A/B baseline.
    """
    val = os.environ.get("VECTOR_RAG_ENABLED", "").strip().lower()
    return val in ("1", "true", "yes", "on")
