# Tasks: RAG Retrieval Pipeline Validation

**Input**: Design documents from `/specs/001-rag-retrieval-validation/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, quickstart.md

**Tests**: Integration tests included for validation per spec requirements.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3, US4)
- Include exact file paths in descriptions

## Path Conventions

Based on plan.md:
- **Backend**: `backend/` (extends existing structure)
- **Tests**: `tests/` at repository root

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and retrieval module structure

- [x] T001 Create data model classes (Query, RetrievalResult, RetrievalResponse) in backend/models.py
- [x] T002 [P] Create error types (ValidationError, ConnectionError, etc.) in backend/errors.py
- [x] T003 [P] Add pytest to project dependencies in backend/pyproject.toml

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core retrieval client that ALL user stories depend on

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [x] T004 Create RetrievalClient class with Qdrant/Cohere initialization in backend/retrieve.py
- [x] T005 Implement query embedding generation using Cohere (input_type="search_query") in backend/retrieve.py
- [x] T006 Implement vector similarity search with Qdrant client.search() in backend/retrieve.py
- [x] T007 Add score normalization (cosine [-1,1] to [0,1]) in backend/retrieve.py
- [x] T008 Implement logging for retrieval operations (query, results, timing) in backend/retrieve.py
- [x] T009 Create CLI entry point with argparse (--top-k, --threshold, --json, --verbose) in backend/retrieve.py

**Checkpoint**: Foundation ready - retrieval client functional with CLI

---

## Phase 3: User Story 1 - Query Book Content (Priority: P1) 🎯 MVP

**Goal**: Submit natural language queries and receive relevant book chunks with scores

**Independent Test**: Run `uv run retrieve.py "What is ROS 2?"` and verify results contain ROS 2 content

### Tests for User Story 1

- [x] T010 [P] [US1] Create test file structure in tests/test_retrieve.py
- [x] T011 [P] [US1] Integration test: basic query returns results with required fields in tests/test_retrieve.py
- [x] T012 [P] [US1] Integration test: results ranked by score descending in tests/test_retrieve.py
- [x] T013 [P] [US1] Integration test: response time under 5 seconds in tests/test_retrieve.py

### Implementation for User Story 1

- [x] T014 [US1] Implement search() method returning RetrievalResponse in backend/retrieve.py
- [x] T015 [US1] Add configurable top_k parameter (default 5) in backend/retrieve.py
- [x] T016 [US1] Add configurable score_threshold parameter (default 0.0) in backend/retrieve.py
- [x] T017 [US1] Implement human-readable output format in backend/retrieve.py
- [x] T018 [US1] Implement JSON output format (--json flag) in backend/retrieve.py

**Checkpoint**: User Story 1 complete - can query and receive ranked results

---

## Phase 4: User Story 2 - Validate Metadata Accuracy (Priority: P2)

**Goal**: Each result includes accurate metadata (URL, title, chunk_index) traceable to source

**Independent Test**: Query for Module 1 content and verify URLs contain "module-1-ros2"

### Tests for User Story 2

- [x] T019 [P] [US2] Integration test: results contain valid URL metadata in tests/test_retrieve.py
- [x] T020 [P] [US2] Integration test: results contain title and chunk_index in tests/test_retrieve.py
- [x] T021 [P] [US2] Integration test: URL paths match expected module patterns in tests/test_retrieve.py

### Implementation for User Story 2

- [x] T022 [US2] Ensure RetrievalResult includes all payload fields (text, url, title, chunk_index) in backend/retrieve.py
- [x] T023 [US2] Add metadata display to human-readable output in backend/retrieve.py
- [x] T024 [US2] Add metadata fields to JSON output in backend/retrieve.py

**Checkpoint**: User Story 2 complete - metadata verified and displayed

---

## Phase 5: User Story 3 - Verify Query Scope Restriction (Priority: P3)

**Goal**: Retrieval only returns content from indexed book, not external sources

**Independent Test**: Query "What is quantum computing?" and verify low/no scores

### Tests for User Story 3

- [x] T025 [P] [US3] Integration test: out-of-scope query returns low scores (<0.5) in tests/test_retrieve.py
- [x] T026 [P] [US3] Integration test: all result URLs belong to book domain in tests/test_retrieve.py

### Implementation for User Story 3

- [x] T027 [US3] Add score threshold filtering to exclude low-relevance results in backend/retrieve.py
- [x] T028 [US3] Display "No relevant results" message when results empty or below threshold in backend/retrieve.py

**Checkpoint**: User Story 3 complete - scope validation working

---

## Phase 6: User Story 4 - Consistent Retrieval Behavior (Priority: P3)

**Goal**: Repeated identical queries return identical results (deterministic)

**Independent Test**: Run same query 3 times and compare results

### Tests for User Story 4

- [x] T029 [P] [US4] Integration test: identical queries return identical results in tests/test_retrieve.py
- [x] T030 [P] [US4] Integration test: result ordering consistent across runs in tests/test_retrieve.py

### Implementation for User Story 4

- [x] T031 [US4] Verify Qdrant search returns deterministic results (no random sampling) in backend/retrieve.py
- [x] T032 [US4] Add execution time logging to RetrievalResponse in backend/retrieve.py

**Checkpoint**: User Story 4 complete - consistent, deterministic retrieval

---

## Phase 7: Edge Cases & Error Handling

**Purpose**: Handle edge cases from spec.md

- [x] T033 [P] Integration test: empty query returns validation error in tests/test_retrieve.py
- [x] T034 [P] Integration test: long query (>1000 chars) handled gracefully in tests/test_retrieve.py
- [x] T035 [P] Integration test: special characters in query handled in tests/test_retrieve.py
- [x] T036 Implement query validation (empty, length, sanitization) in backend/retrieve.py
- [x] T037 Implement connection error handling with descriptive messages in backend/retrieve.py
- [x] T038 Implement graceful handling for collection not found in backend/retrieve.py

---

## Phase 8: Polish & Cross-Cutting Concerns

**Purpose**: Documentation and final validation

- [x] T039 [P] Add docstrings to all public functions in backend/retrieve.py
- [x] T040 [P] Run full test suite and verify all tests pass
- [x] T041 Run quickstart.md validation scenarios manually
- [x] T042 Update pyproject.toml with any missing dependencies

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phases 3-6)**: All depend on Foundational phase completion
  - User stories can proceed in parallel OR sequentially (P1 → P2 → P3)
- **Edge Cases (Phase 7)**: Depends on User Story 1 completion minimum
- **Polish (Phase 8)**: Depends on all user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational - No dependencies on other stories
- **User Story 2 (P2)**: Can start after Foundational - Builds on US1 output format
- **User Story 3 (P3)**: Can start after Foundational - Independent of US1/US2
- **User Story 4 (P3)**: Can start after Foundational - Independent of other stories

### Within Each Phase

- Tests MUST be written before implementation tasks
- Data models before service methods
- Core implementation before output formatting
- Commit after each task or logical group

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel
- All tests within a user story marked [P] can run in parallel
- Edge case tests (Phase 7) marked [P] can run in parallel
- Polish tasks marked [P] can run in parallel

---

## Parallel Example: User Story 1

```bash
# Launch all tests for User Story 1 together:
Task: T010 [P] [US1] Create test file structure in tests/test_retrieve.py
Task: T011 [P] [US1] Integration test: basic query returns results
Task: T012 [P] [US1] Integration test: results ranked by score
Task: T013 [P] [US1] Integration test: response time under 5 seconds
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup (T001-T003)
2. Complete Phase 2: Foundational (T004-T009) - CRITICAL
3. Complete Phase 3: User Story 1 (T010-T018)
4. **STOP and VALIDATE**: Run `uv run retrieve.py "What is ROS 2?"`
5. Deploy/demo if ready

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 → Test → MVP complete! ✅
3. Add User Story 2 → Test → Metadata validation added
4. Add User Story 3 → Test → Scope validation added
5. Add User Story 4 → Test → Consistency verified
6. Add Edge Cases → Full robustness
7. Polish → Production ready

---

## Summary

| Phase | Tasks | Description |
|-------|-------|-------------|
| Setup | 3 | Data models and errors |
| Foundational | 6 | RetrievalClient core |
| US1 (P1) | 9 | Query and results |
| US2 (P2) | 6 | Metadata validation |
| US3 (P3) | 4 | Scope restriction |
| US4 (P3) | 4 | Consistency |
| Edge Cases | 6 | Error handling |
| Polish | 4 | Documentation |
| **Total** | **42** | |

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Stop at any checkpoint to validate story independently
- Commit after each task or logical group
