# Data Model: Embedding Pipeline

**Feature**: 001-embedding-pipeline
**Date**: 2025-12-17

## Entities

### Document

Represents a crawled web page from the Docusaurus site.

| Field | Type | Description |
|-------|------|-------------|
| url | string | Full URL of the page |
| title | string | Page title extracted from HTML |
| content | string | Clean text content (HTML stripped) |

### TextChunk

A segment of document text prepared for embedding.

| Field | Type | Description |
|-------|------|-------------|
| text | string | The chunk content |
| chunk_index | integer | Position within the parent document |
| source_url | string | URL of the parent document |
| source_title | string | Title of the parent document |

### Embedding (Qdrant Point)

Vector representation stored in Qdrant.

| Field | Type | Description |
|-------|------|-------------|
| id | string/uuid | Unique identifier for the point |
| vector | float[1024] | Cohere embedding vector |
| payload.text | string | Original chunk text |
| payload.url | string | Source document URL |
| payload.title | string | Source document title |
| payload.chunk_index | integer | Chunk position |

## Qdrant Collection Schema

**Collection Name**: `rag-embedding`

```python
{
    "vectors": {
        "size": 1024,  # Cohere embed-english-v3.0 dimensions
        "distance": "Cosine"
    }
}
```

### Payload Index Fields

| Field | Index Type | Purpose |
|-------|------------|---------|
| url | keyword | Filter by source URL |
| title | text | Full-text search on titles |

## Data Flow

```
Web Page (HTML)
    ↓ extract
Document (url, title, content)
    ↓ chunk
TextChunk[] (text, chunk_index, source_url, source_title)
    ↓ embed
Embedding[] (id, vector, payload)
    ↓ upsert
Qdrant Collection "rag-embedding"
```
