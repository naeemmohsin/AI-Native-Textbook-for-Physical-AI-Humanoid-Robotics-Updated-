# Tasks: FastAPI RAG Frontend Integration

**Input**: Design documents from `/specs/006-fastapi-rag-frontend/`
**Prerequisites**: plan.md, spec.md, data-model.md, contracts/openapi.yaml

## Format: `[ID] [P?] [Story?] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: User story label (US1, US2, US3, US4)

---

## Phase 1: Setup

- [x] T001 Add FastAPI and uvicorn dependencies to backend/pyproject.toml
- [x] T002 Run `uv sync` to install new dependencies in backend/

---

## Phase 2: Foundational (Backend API)

- [x] T003 Create Pydantic models (QueryRequest, QueryResponse, Citation, ErrorResponse, HealthResponse) in backend/api.py
- [x] T004 Refactor backend/agent.py to export `run_query_async()` function for API use
- [x] T005 Create FastAPI app with CORS middleware in backend/api.py
- [x] T006 Implement GET /api/health endpoint in backend/api.py
- [x] T007 Implement POST /api/query endpoint in backend/api.py
- [x] T008 Add citation extraction from agent response in backend/api.py
- [x] T009 Add uvicorn runner to backend/api.py main block

**Checkpoint**: Backend API complete - test with `uv run api.py` and curl

---

## Phase 3: User Story 1 - Ask a Question via Chatbot (P1) MVP

**Goal**: Reader can open chatbot, ask question, receive cited response

**Test**: Open book page, click chatbot, submit "What is ROS 2?", verify response with citation

### Implementation

- [x] T010 [P] [US1] Create ChatWidget component in frontend_book/src/components/ChatWidget/index.tsx
- [x] T011 [P] [US1] Create ChatWidget styles in frontend_book/src/components/ChatWidget/styles.module.css
- [x] T012 [US1] Implement chat state management (messages, loading, error) in ChatWidget
- [x] T013 [US1] Implement API call to POST /api/query in ChatWidget
- [x] T014 [US1] Display response with citations in ChatWidget
- [x] T015 [US1] Create Root.tsx wrapper in frontend_book/src/theme/Root.tsx
- [x] T016 [US1] Add empty query validation in ChatWidget

**Checkpoint**: US1 complete - chatbot visible on all pages, queries work

---

## Phase 4: User Story 2 - Ask About Selected Text (P2)

**Goal**: Reader can select text and query about it

**Test**: Select text on page, click "Ask about this", verify context passed to chatbot

### Implementation

- [x] T017 [P] [US2] Add text selection listener in ChatWidget
- [x] T018 [US2] Create "Ask about this" floating button for selected text
- [x] T019 [US2] Pass selected text as context to API query
- [x] T020 [US2] Display context preview in ChatWidget

**Checkpoint**: US2 complete - selected text context works

---

## Phase 5: User Story 3 - Continue Conversation (P3)

**Goal**: Multi-turn conversations with session persistence

**Test**: Ask question, then ask follow-up, verify context maintained

### Implementation

- [x] T021 [US3] Generate and store session_id in localStorage
- [x] T022 [US3] Pass session_id in API requests
- [x] T023 [US3] Verify backend SQLiteSession maintains conversation history

**Checkpoint**: US3 complete - follow-up questions work

---

## Phase 6: User Story 4 - Mobile Support (P4)

**Goal**: Responsive chatbot for mobile devices

**Test**: Open on mobile, verify usable interface

### Implementation

- [x] T024 [P] [US4] Add responsive CSS for mobile in styles.module.css
- [x] T025 [US4] Adjust chat panel size for small screens
- [x] T026 [US4] Test touch interactions

**Checkpoint**: US4 complete - mobile experience usable

---

## Phase 7: Polish

- [x] T027 Add loading spinner animation in ChatWidget
- [x] T028 Add error retry button in ChatWidget
- [x] T029 Handle timeout (30s) with user message
- [x] T030 Validate end-to-end with quickstart.md steps

---

## Dependencies

```
Phase 1 (Setup) → Phase 2 (Backend API) → Phase 3-6 (User Stories) → Phase 7 (Polish)

User Stories can run in parallel after Phase 2:
- US1: No dependencies (MVP)
- US2: Independent (can be done after US1 for UX flow)
- US3: Independent (session management)
- US4: Independent (CSS only)
```

## Parallel Opportunities

```bash
# Phase 2 - All endpoints can be built sequentially in api.py

# Phase 3 - Frontend components in parallel:
T010 + T011 (ChatWidget component + styles)

# Phase 4-6 - User stories can run in parallel after US1
```

## Summary

| Phase | Tasks | Stories |
|-------|-------|---------|
| Setup | 2 | - |
| Foundational | 7 | - |
| US1 (MVP) | 7 | P1 |
| US2 | 4 | P2 |
| US3 | 3 | P3 |
| US4 | 3 | P4 |
| Polish | 4 | - |
| **Total** | **30** | **4** |

**MVP Scope**: Complete Phase 1-3 (16 tasks) for working chatbot.
