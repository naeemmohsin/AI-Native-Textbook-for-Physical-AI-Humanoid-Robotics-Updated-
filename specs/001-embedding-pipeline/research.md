# Research: Embedding Pipeline for RAG Retrieval

**Feature**: 001-embedding-pipeline
**Date**: 2025-12-17
**Status**: Complete

## Technology Decisions

### 1. Project Setup with UV Package Manager

**Decision**: Use `uv` for Python project initialization and dependency management

**Rationale**:
- Modern, fast Python package manager written in Rust
- Single tool for creating virtual environments and managing dependencies
- Faster than pip for dependency resolution
- Compatible with standard Python packaging (pyproject.toml)

**Alternatives Considered**:
- pip + venv: Standard but slower, requires multiple tools
- Poetry: Full-featured but heavier, slower dependency resolution
- pipenv: Less actively maintained, slower

### 2. Cohere Embedding Model

**Decision**: Use Cohere `embed-english-v3.0` or `embed-multilingual-v3.0` model

**Rationale**:
- embed-v3.0 models are production-ready with 1024 dimensions
- embed-v4.0 (newer) supports 256/512/1024/1536 dimensions
- `input_type` parameter is required for v3+:
  - `search_document`: For documents being indexed
  - `search_query`: For query embeddings during retrieval
- Batch limit: 96 texts per API call
- Good balance of quality and cost for RAG applications

**Key Parameters**:
- `model`: "embed-english-v3.0"
- `input_type`: "search_document" (for indexing)
- `texts`: List of strings (max 96 per call)
- Returns: List of float vectors (1024 dimensions)

### 3. Qdrant Vector Database

**Decision**: Use Qdrant Cloud or local instance with Python client

**Rationale**:
- Purpose-built for vector similarity search
- Rich payload filtering capabilities
- Simple Python SDK with batch upsert support
- Collection name: `rag-embedding` (per user requirement)

**Key Operations**:
- `create_collection`: Initialize with vector size matching embedding dimensions
- `upsert`: Batch insert vectors with metadata payloads
- `search`: Similarity search with optional payload filters

### 4. URL Crawling Strategy

**Decision**: Use `requests` + `BeautifulSoup` for HTML fetching and parsing

**Rationale**:
- Simple, well-documented libraries
- BeautifulSoup handles malformed HTML gracefully
- No JavaScript rendering needed for static Docusaurus sites
- Easy to extract content from specific HTML elements

**Target Site Analysis**:
- URL: https://ai-native-textbook-for-physical-ai-phi.vercel.app/
- Type: Docusaurus documentation site
- Content: ROS 2 and AI robotics documentation
- Structure: `/docs/` path for documentation pages

### 5. Text Chunking Strategy

**Decision**: Simple character-based chunking with overlap

**Rationale**:
- Chunk size: ~500-1000 characters per chunk
- Overlap: ~100 characters to maintain context across boundaries
- Simple implementation suitable for documentation content
- No external chunking library required

**Alternatives Considered**:
- LangChain text splitters: Adds dependency, overkill for this use case
- Sentence-based: More complex, inconsistent chunk sizes
- Semantic chunking: Requires additional model, too complex

## Architecture Decision

### Single-File Design (per user requirement)

**Decision**: Implement entire pipeline in single `main.py` file

**Functions**:
1. `get_all_urls(base_url)` - Crawl and discover all page URLs
2. `extract_text_from_urls(urls)` - Fetch and clean text from HTML
3. `chunk_text(text, metadata)` - Split text into chunks with overlap
4. `embed(chunks)` - Generate Cohere embeddings
5. `create_collection(client)` - Initialize Qdrant collection `rag-embedding`
6. `save_chunk_to_qdrant(client, chunks, embeddings)` - Upsert to Qdrant
7. `main()` - Orchestrate the pipeline

**Project Structure**:
```
backend/
├── pyproject.toml
├── .env
└── main.py
```

## Dependencies

| Package | Version | Purpose |
|---------|---------|---------|
| cohere | ^5.0 | Embedding generation |
| qdrant-client | ^1.7 | Vector database client |
| requests | ^2.31 | HTTP requests |
| beautifulsoup4 | ^4.12 | HTML parsing |
| python-dotenv | ^1.0 | Environment variable loading |

## Environment Variables

```
COHERE_API_KEY=your-cohere-api-key
QDRANT_URL=http://localhost:6333 or https://your-qdrant-cloud-url
QDRANT_API_KEY=your-qdrant-api-key (if using cloud)
```

## Sources

- [Cohere Embeddings Documentation](https://docs.cohere.com/docs/embeddings)
- [Cohere Embed API Reference](https://docs.cohere.com/reference/embed)
- [Qdrant Python Client Documentation](https://qdrant.tech/documentation/)
- [UV Package Manager](https://github.com/astral-sh/uv)
