# Research: RAG Retrieval Pipeline Validation

**Feature**: 001-rag-retrieval-validation
**Date**: 2025-12-25

## Research Areas

### 1. Cohere Embedding Input Types

**Question**: What input_type should be used for query embeddings vs document embeddings?

**Finding**: Cohere's embed-english-v3.0 uses asymmetric embeddings:
- Documents: `input_type="search_document"` (used during indexing in Spec 1)
- Queries: `input_type="search_query"` (should be used for retrieval)

**Source**: Cohere API documentation recommends asymmetric embedding for search/retrieval tasks.

**Decision**: Use `input_type="search_query"` for all query embeddings.

---

### 2. Qdrant Search API

**Question**: How to perform similarity search in Qdrant with score filtering?

**Finding**: Qdrant's `search()` method accepts:
- `query_vector`: The embedding vector to search for
- `limit`: Maximum number of results (top-k)
- `score_threshold`: Minimum similarity score (optional)
- Returns: List of `ScoredPoint` objects with `id`, `score`, `payload`

**Code Pattern**:
```python
results = client.search(
    collection_name="rag-embedding",
    query_vector=query_embedding,
    limit=top_k,
    score_threshold=score_threshold  # Optional
)
```

**Decision**: Use Qdrant's native search with optional score_threshold parameter.

---

### 3. Score Normalization

**Question**: What is the score range for cosine similarity in Qdrant?

**Finding**:
- Cosine similarity range: [-1, 1]
- Qdrant returns raw cosine scores
- Most results for well-indexed content will be in [0, 1] range
- Negative scores indicate very dissimilar content

**Decision**: Normalize scores to [0, 1] using formula: `(score + 1) / 2`
This maps -1 → 0, 0 → 0.5, 1 → 1

---

### 4. Existing Infrastructure

**Question**: What infrastructure from Spec 1 can be reused?

**Finding** (from `backend/main.py`):
- Environment variables: `QDRANT_URL`, `QDRANT_API_KEY`, `COHERE_API_KEY`
- Collection name: `rag-embedding`
- Embedding model: `embed-english-v3.0`
- Embedding dimension: 1024
- Payload structure: `{"text", "url", "title", "chunk_index"}`

**Decision**: Reuse all configuration constants and client initialization patterns.

---

### 5. CLI Best Practices

**Question**: How to structure CLI output for both human and machine consumption?

**Finding**: Best practices for CLI tools:
- Default: Human-readable format with colors/formatting
- `--json` flag: Machine-parseable JSON output
- Errors to stderr, results to stdout
- Exit codes: 0 = success, 1 = error, 2 = no results

**Decision**: Implement dual output modes with `--json` flag.

---

## Summary of Decisions

| Area | Decision | Rationale |
|------|----------|-----------|
| Query embedding type | `search_query` | Asymmetric embedding for optimal retrieval |
| Score threshold | Optional, default 0.0 | Allow all results for debugging |
| Score normalization | `(score + 1) / 2` | Map [-1,1] to intuitive [0,1] |
| Output format | Human + JSON modes | Support both use cases |
| Error handling | Structured responses | CLI-friendly, automatable |
