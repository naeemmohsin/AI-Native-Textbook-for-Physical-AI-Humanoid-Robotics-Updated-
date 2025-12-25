# Implementation Plan: RAG Retrieval Pipeline Validation

**Branch**: `001-rag-retrieval-validation` | **Date**: 2025-12-25 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/001-rag-retrieval-validation/spec.md`

## Summary

Build a retrieval validation module that connects to the existing Qdrant collection (`rag-embedding`), performs vector similarity searches using Cohere embeddings for queries, and returns ranked results with metadata. This extends the existing embedding pipeline (Spec 1) to enable query-time retrieval and validation testing.

## Technical Context

**Language/Version**: Python 3.11+
**Primary Dependencies**: qdrant-client, cohere, python-dotenv
**Storage**: Qdrant Cloud (existing collection: `rag-embedding`)
**Testing**: pytest with integration tests against live Qdrant
**Target Platform**: CLI script (local development)
**Project Type**: Single module extending existing backend
**Performance Goals**: Query response < 5 seconds, handle 100 sequential queries
**Constraints**: Same Cohere model (embed-english-v3.0) as indexing, cosine similarity
**Scale/Scope**: 654 indexed chunks, 17 documents, validation testing only

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Principle | Status | Notes |
|-----------|--------|-------|
| Library-First | PASS | Retrieval module is self-contained, independently testable |
| CLI Interface | PASS | Script outputs to stdout, errors to stderr, supports JSON format |
| Test-First | PASS | Test scenarios defined in spec before implementation |
| Observability | PASS | Logging required per FR-008 |
| Simplicity | PASS | Minimal implementation, no unnecessary abstractions |

**Gate Status**: PASS - No violations detected.

## Project Structure

### Documentation (this feature)

```text
specs/001-rag-retrieval-validation/
├── plan.md              # This file
├── research.md          # Phase 0 output
├── data-model.md        # Phase 1 output
├── quickstart.md        # Phase 1 output
├── contracts/           # Phase 1 output (N/A - CLI only)
└── tasks.md             # Phase 2 output (/sp.tasks command)
```

### Source Code (repository root)

```text
backend/
├── main.py              # Existing embedding pipeline (Spec 1)
├── retrieve.py          # NEW: Retrieval module
├── .env                 # Existing credentials
└── pyproject.toml       # Existing dependencies

tests/
└── test_retrieve.py     # NEW: Retrieval validation tests
```

**Structure Decision**: Extend existing `backend/` directory with new `retrieve.py` module. Reuses existing Qdrant/Cohere client initialization patterns from `main.py`. Tests in `tests/` directory at project root.

## Complexity Tracking

> No constitution violations - section not required.

---

## Phase 0: Research Findings

### Decision 1: Query Embedding Input Type

**Decision**: Use `input_type="search_query"` for query embeddings
**Rationale**: Cohere's embed-english-v3.0 model uses asymmetric embeddings. Documents indexed with `input_type="search_document"` should be queried with `input_type="search_query"` for optimal retrieval performance.
**Alternatives considered**: Using same `search_document` type - rejected because Cohere documentation recommends asymmetric embedding for retrieval tasks.

### Decision 2: Similarity Score Normalization

**Decision**: Qdrant returns cosine similarity scores in range [-1, 1]; normalize to [0, 1] for user output
**Rationale**: Cosine similarity can be negative for very dissimilar vectors. Normalizing to [0, 1] provides more intuitive scores where 1 = most similar.
**Alternatives considered**: Raw scores - rejected for user confusion with negative values.

### Decision 3: Default Parameters

**Decision**: Default top_k=5, score_threshold=0.0 (return all results)
**Rationale**: Reasonable default for validation testing. Threshold of 0 allows seeing all results for debugging; users can filter if needed.
**Alternatives considered**: Higher threshold (0.5) - rejected as it may hide relevant results during validation.

### Decision 4: Error Handling Strategy

**Decision**: Return structured error responses with error type and message
**Rationale**: CLI output should be parseable; structured errors enable automation.
**Alternatives considered**: Exceptions only - rejected for poor CLI experience.

---

## Phase 1: Design Artifacts

See companion files:
- [data-model.md](./data-model.md) - Entity definitions
- [quickstart.md](./quickstart.md) - Usage guide

### API Contracts

N/A - This is a CLI tool, not a REST API. See quickstart.md for CLI interface specification.

### Key Implementation Components

1. **RetrievalClient class**
   - Initializes Qdrant and Cohere clients from environment
   - Provides `search(query, top_k, score_threshold)` method
   - Returns `RetrievalResponse` with results and metadata

2. **CLI Entry Point**
   - Accepts query as argument or stdin
   - Supports `--top-k`, `--threshold`, `--json` flags
   - Outputs human-readable or JSON format

3. **Validation Test Suite**
   - Test queries covering all book modules
   - Metadata verification tests
   - Edge case tests (empty query, long query, no matches)
   - Consistency tests (repeated queries)
