# Data Model: RAG Retrieval Pipeline Validation

**Feature**: 001-rag-retrieval-validation
**Date**: 2025-12-25

## Entities

### Query

Represents a search request from the user.

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| text | string | Yes | Natural language query text |
| top_k | integer | No | Maximum results to return (default: 5) |
| score_threshold | float | No | Minimum similarity score 0-1 (default: 0.0) |

**Validation Rules**:
- `text` must be non-empty after stripping whitespace
- `text` must be ≤ 1000 characters
- `top_k` must be between 1 and 100
- `score_threshold` must be between 0.0 and 1.0

---

### RetrievalResult

Represents a single retrieved document chunk with metadata.

| Field | Type | Description |
|-------|------|-------------|
| text | string | The text content of the chunk |
| url | string | Source URL of the original document |
| title | string | Title of the source document |
| chunk_index | integer | Position of this chunk within the document |
| score | float | Similarity score (0-1, higher = more relevant) |

**Relationships**:
- Originates from Qdrant point payload
- Score derived from cosine similarity, normalized

---

### RetrievalResponse

Collection of results plus query metadata.

| Field | Type | Description |
|-------|------|-------------|
| query | string | The original query text |
| results | list[RetrievalResult] | Ordered list of matching chunks |
| total_results | integer | Number of results returned |
| execution_time_ms | float | Time taken for retrieval |
| error | string or null | Error message if retrieval failed |

**State Transitions**:
- Initial → Searching → Success/Error
- Success: results populated, error is null
- Error: results empty, error contains message

---

## Qdrant Payload Schema

The existing Qdrant collection uses this payload structure (from Spec 1):

```json
{
  "text": "string - chunk text content",
  "url": "string - source document URL",
  "title": "string - document title",
  "chunk_index": "integer - position in document"
}
```

This maps directly to `RetrievalResult` fields (minus `score` which comes from search).

---

## Entity Relationships

```
Query (1) --performs--> Search --returns--> RetrievalResponse (1)
                                                    |
                                                    v
                                           RetrievalResult (0..n)
                                                    |
                                                    v
                                           Qdrant Point Payload
```

---

## Error Types

| Error Code | Description | Example |
|------------|-------------|---------|
| VALIDATION_ERROR | Invalid query input | Empty query, too long |
| CONNECTION_ERROR | Cannot reach Qdrant | Network timeout |
| AUTH_ERROR | Invalid credentials | Bad API key |
| COLLECTION_NOT_FOUND | Collection missing | Not indexed yet |
| EMBEDDING_ERROR | Cohere API failure | Rate limit, bad key |
