# Implementation Plan: Embedding Pipeline for RAG Retrieval

**Branch**: `001-embedding-pipeline` | **Date**: 2025-12-17 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/001-embedding-pipeline/spec.md`

## Summary

Build a Python-based embedding pipeline that crawls the deployed Docusaurus site at "https://ai-native-textbook-for-physical-ai.vercel.app", extracts and cleans text content, generates embeddings using Cohere's API, and stores them in Qdrant for RAG-based retrieval. The entire implementation will be contained in a single `main.py` file within a `backend/` folder, following the user's specified function design.

## Technical Context

**Language/Version**: Python 3.11+
**Primary Dependencies**: cohere, qdrant-client, requests, beautifulsoup4, python-dotenv
**Storage**: Qdrant vector database (collection: `rag-embedding`)
**Testing**: Manual testing via script execution
**Target Platform**: Local/server Python runtime
**Project Type**: Single backend script
**Performance Goals**: Process 100+ pages, batch embedding generation
**Constraints**: Cohere API rate limits (96 texts per batch), Qdrant connection requirements
**Scale/Scope**: Single Docusaurus site, ~50-200 document pages

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Principle | Status | Notes |
|-----------|--------|-------|
| Simplicity | PASS | Single-file design, minimal dependencies |
| Library-First | PASS | Uses established libraries (cohere, qdrant-client) |
| Text I/O | PASS | CLI script with stdout logging |
| Test-First | N/A | User specified manual testing approach |

## Project Structure

### Documentation (this feature)

```text
specs/001-embedding-pipeline/
├── plan.md              # This file
├── research.md          # Technology decisions and rationale
├── data-model.md        # Entity definitions
├── quickstart.md        # Setup and usage guide
└── tasks.md             # Implementation tasks (created by /sp.tasks)
```

### Source Code (repository root)

```text
backend/
├── pyproject.toml       # UV/Python project configuration
├── .env.example         # Environment variable template
├── .env                  # Actual credentials (gitignored)
└── main.py              # Complete pipeline implementation
```

**Structure Decision**: Single backend folder with one Python file (`main.py`) containing all pipeline functions as specified by user requirements.

## Function Design

### main.py Functions

| Function | Purpose | Input | Output |
|----------|---------|-------|--------|
| `get_all_urls(base_url)` | Crawl sitemap/links to discover all page URLs | Base URL string | List of URL strings |
| `extract_text_from_urls(urls)` | Fetch HTML and extract clean text | List of URLs | List of (url, title, text) tuples |
| `chunk_text(text, chunk_size, overlap)` | Split text into overlapping chunks | Text string, sizes | List of chunk strings |
| `embed(chunks)` | Generate Cohere embeddings | List of text chunks | List of embedding vectors |
| `create_collection(client)` | Initialize Qdrant collection `rag-embedding` | Qdrant client | None (creates collection) |
| `save_chunk_to_qdrant(client, chunks, embeddings, metadata)` | Upsert vectors with metadata | Client, data | None (upserts points) |
| `main()` | Orchestrate entire pipeline | None | None (executes pipeline) |

## Pipeline Flow

```
1. main() starts
   │
2. get_all_urls(BASE_URL)
   │ → Returns: ["url1", "url2", ...]
   │
3. extract_text_from_urls(urls)
   │ → Returns: [(url, title, text), ...]
   │
4. For each document:
   │   chunk_text(text, 500, 100)
   │   → Returns: ["chunk1", "chunk2", ...]
   │
5. embed(all_chunks)  # Batched (max 96 per call)
   │ → Returns: [[0.1, 0.2, ...], ...]
   │
6. create_collection(client)
   │ → Creates "rag-embedding" collection
   │
7. save_chunk_to_qdrant(client, chunks, embeddings, metadata)
   │ → Upserts all points
   │
8. main() completes
```

## Complexity Tracking

> No violations identified. Design follows simplicity principles.

| Aspect | Decision | Justification |
|--------|----------|---------------|
| Single file | main.py only | User requirement, sufficient for pipeline scope |
| No async | Synchronous requests | Simpler implementation, acceptable for batch processing |
| No retry logic | Basic error handling | Can be added later if needed |
