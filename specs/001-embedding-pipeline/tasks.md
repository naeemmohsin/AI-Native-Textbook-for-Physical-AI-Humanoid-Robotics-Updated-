# Tasks: Embedding Pipeline for RAG Retrieval

**Input**: Design documents from `/specs/001-embedding-pipeline/`
**Prerequisites**: plan.md (required), spec.md (required), research.md, data-model.md, contracts/

**Tests**: Manual testing via script execution (no automated tests requested)

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Project type**: Single backend script
- **Root**: `backend/`
- **Main file**: `backend/main.py`

---

## Phase 1: Setup (Project Infrastructure)

**Purpose**: Initialize backend folder and Python project with UV

- [x] T001 Create `backend/` directory at repository root
- [x] T002 Initialize Python project with `uv init` in backend/
- [x] T003 Add dependencies with `uv add cohere qdrant-client requests beautifulsoup4 python-dotenv`
- [x] T004 [P] Create `backend/.env.example` with COHERE_API_KEY, QDRANT_URL, QDRANT_API_KEY placeholders
- [x] T005 [P] Add `backend/.env` to `.gitignore`
- [x] T006 Create empty `backend/main.py` with imports and docstring

**Checkpoint**: Project structure ready for implementation

---

## Phase 2: Foundational (Environment & Client Setup)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**CRITICAL**: No user story work can begin until this phase is complete

- [x] T007 Implement environment loading with `load_dotenv()` in backend/main.py
- [x] T008 Add Cohere client initialization in backend/main.py
- [x] T009 Add Qdrant client initialization in backend/main.py
- [x] T010 Add logging configuration (console output with timestamps) in backend/main.py
- [x] T011 Define BASE_URL constant for target Docusaurus site in backend/main.py

**Checkpoint**: Foundation ready - clients initialized, logging configured

---

## Phase 3: User Story 1 - Crawl URLs and Extract Text (Priority: P1)

**Goal**: Crawl deployed Docusaurus documentation URLs and extract clean text content

**Independent Test**: Run script with only US1 functions implemented, verify URLs discovered and text extracted to console output

### Implementation for User Story 1

- [x] T012 [US1] Implement `get_all_urls(base_url)` function in backend/main.py
  - Fetch sitemap.xml or crawl homepage links
  - Filter to /docs/ paths only
  - Return list of unique URLs
  - Log count of discovered URLs

- [x] T013 [US1] Implement `extract_text_from_urls(urls)` function in backend/main.py
  - Fetch HTML with requests
  - Parse with BeautifulSoup
  - Extract main content (article/main/div.content)
  - Remove nav, footer, sidebar, script, style elements
  - Extract title from `<title>` or `<h1>`
  - Return list of (url, title, text) tuples
  - Log progress for each URL processed

- [x] T014 [US1] Implement `chunk_text(text, chunk_size=500, overlap=100)` function in backend/main.py
  - Split text into overlapping chunks
  - Handle edge case where text < chunk_size
  - Return empty list for empty/whitespace text
  - Return list of chunk strings

- [x] T015 [US1] Add error handling for HTTP errors in get_all_urls and extract_text_from_urls
  - Catch RequestException
  - Log warnings for failed URLs
  - Continue processing remaining URLs

**Checkpoint**: US1 complete - can crawl site and extract clean chunked text

---

## Phase 4: User Story 2 - Generate Embeddings with Cohere (Priority: P2)

**Goal**: Generate vector embeddings from extracted text using Cohere's embedding API

**Independent Test**: Provide sample text chunks, verify embeddings returned with 1024 dimensions

### Implementation for User Story 2

- [x] T016 [US2] Implement `embed(chunks)` function in backend/main.py
  - Use Cohere embed-english-v3.0 model
  - Set input_type="search_document"
  - Batch requests (max 96 texts per API call)
  - Return list of embedding vectors
  - Log progress for each batch

- [x] T017 [US2] Add batching logic to handle more than 96 chunks
  - Split chunks into batches of 96
  - Process each batch sequentially
  - Combine all embedding results

- [x] T018 [US2] Add error handling for Cohere API errors in embed function
  - Catch CohereError/APIError
  - Log error details
  - Raise exception with clear message

**Checkpoint**: US2 complete - can generate embeddings for any text chunks

---

## Phase 5: User Story 3 - Store Embeddings in Qdrant (Priority: P3)

**Goal**: Store generated embeddings in Qdrant vector database for RAG retrieval

**Independent Test**: Store sample embeddings, perform similarity search, verify results returned

### Implementation for User Story 3

- [x] T019 [US3] Implement `create_collection(client)` function in backend/main.py
  - Create collection named "rag-embedding"
  - Set vector size to 1024 (Cohere embed-v3.0)
  - Set distance metric to Cosine
  - Delete existing collection if present (recreate)
  - Log success/failure

- [x] T020 [US3] Implement `save_chunk_to_qdrant(client, chunks, embeddings)` function in backend/main.py
  - Create PointStruct for each chunk/embedding pair
  - Generate UUID for each point
  - Include payload: text, url, title, chunk_index
  - Upsert all points in single batch
  - Log number of points upserted

- [x] T021 [US3] Add error handling for Qdrant operations
  - Catch QdrantException
  - Log error details
  - Raise exception with clear message

**Checkpoint**: US3 complete - can store and retrieve embeddings from Qdrant

---

## Phase 6: Pipeline Orchestration

**Purpose**: Wire all functions together in main() function

- [x] T022 Implement `main()` function in backend/main.py orchestrating full pipeline:
  1. Log pipeline start
  2. Call get_all_urls(BASE_URL)
  3. Call extract_text_from_urls(urls)
  4. Chunk all extracted text with metadata
  5. Call embed(all_chunks)
  6. Call create_collection(client)
  7. Call save_chunk_to_qdrant(client, chunks, embeddings)
  8. Log completion summary (URLs processed, chunks created, points stored)

- [x] T023 Add `if __name__ == "__main__": main()` entry point in backend/main.py

- [x] T024 Add sys.exit codes (0=success, 1=config error, 2=network error) to main()

**Checkpoint**: Full pipeline executable from command line

---

## Phase 7: Polish & Validation

**Purpose**: Final validation and cleanup

- [x] T025 [P] Validate .env.example has all required variables documented
- [x] T026 [P] Run full pipeline against target URL and verify Qdrant collection created
- [x] T027 Test similarity search query against stored embeddings (manual verification)
- [x] T028 Review and clean up any debug code or comments

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3-5)**: All depend on Foundational phase completion
  - US1 (Phase 3): Can start after Phase 2
  - US2 (Phase 4): Can start after Phase 2 (parallel to US1)
  - US3 (Phase 5): Can start after Phase 2 (parallel to US1, US2)
- **Orchestration (Phase 6)**: Depends on US1, US2, US3 completion
- **Polish (Phase 7)**: Depends on Phase 6 completion

### User Story Dependencies

- **User Story 1 (P1)**: Independent - no dependencies on other stories
- **User Story 2 (P2)**: Independent - uses output format from US1 but can be tested separately
- **User Story 3 (P3)**: Independent - uses output format from US2 but can be tested separately

### Within Each User Story

- All tasks in a story execute sequentially (same file: main.py)
- Core function → Error handling

### Parallel Opportunities

- **Phase 1**: T004, T005 can run in parallel (different files)
- **Phase 3-5**: All user story phases can run in parallel since all edit main.py but different functions
- **Phase 7**: T025, T026 can run in parallel

---

## Parallel Example: Setup Phase

```bash
# After T003 completes, these can run in parallel:
Task: "Create backend/.env.example with placeholders"
Task: "Add backend/.env to .gitignore"
```

---

## Implementation Strategy

### MVP First (User Story 1 + 2 + 3 = Complete Pipeline)

1. Complete Phase 1: Setup (5 minutes)
2. Complete Phase 2: Foundational (10 minutes)
3. Complete Phase 3: User Story 1 - URL Crawling (core feature)
4. Complete Phase 4: User Story 2 - Embedding Generation
5. Complete Phase 5: User Story 3 - Qdrant Storage
6. Complete Phase 6: Orchestration - Wire it all together
7. **VALIDATE**: Run `uv run python main.py` and verify embeddings in Qdrant
8. Complete Phase 7: Polish

### Single-File Constraint

All implementation happens in `backend/main.py`. Tasks are organized by logical function but target the same file. Execute tasks sequentially within each user story phase.

---

## Summary

| Phase | Tasks | Purpose |
|-------|-------|---------|
| 1. Setup | T001-T006 | Project initialization |
| 2. Foundational | T007-T011 | Client setup, logging |
| 3. US1: Crawl/Extract | T012-T015 | URL discovery, text extraction |
| 4. US2: Embeddings | T016-T018 | Cohere embedding generation |
| 5. US3: Storage | T019-T021 | Qdrant collection and upsert |
| 6. Orchestration | T022-T024 | main() function |
| 7. Polish | T025-T028 | Validation, cleanup |

**Total Tasks**: 28
**Tasks per User Story**: US1=4, US2=3, US3=3
**Parallel Opportunities**: 4 (T004/T005, T025/T026, and US phases can run in parallel)
**MVP Scope**: All phases required for functional pipeline

---

## Notes

- All functions implemented in single `backend/main.py` file per user requirement
- Manual testing via `uv run python main.py`
- Target URL: TARGET_URL = "https://ai-native-textbook-for-physical-ai.vercel.app/sitemap.xml"
- Collection name: `rag-embedding`
- Embedding dimensions: 1024 (Cohere embed-english-v3.0)
