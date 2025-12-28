# Implementation Plan: FastAPI RAG Frontend Integration

**Branch**: `006-fastapi-rag-frontend` | **Date**: 2024-12-28 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/006-fastapi-rag-frontend/spec.md`

## Summary

Integrate the existing RAG agent (`backend/agent.py`) with the Docusaurus frontend (`frontend_book/`) via a FastAPI server. The FastAPI server (`backend/api.py`) exposes a query endpoint that calls the agent and returns JSON responses. A chatbot widget is added to the frontend that displays across all book pages, enabling readers to ask questions and receive AI-generated answers grounded in the book content.

## Technical Context

**Language/Version**: Python 3.13 (backend), TypeScript/React 19 (frontend)
**Primary Dependencies**: FastAPI, uvicorn, openai-agents, Docusaurus 3.9.2, React 19
**Storage**: SQLite for session memory (existing `conversations.db`)
**Testing**: pytest (backend), manual testing (frontend MVP)
**Target Platform**: Local development (Windows/WSL), deployment-ready
**Project Type**: Web application (frontend + backend)
**Performance Goals**: <10s response time (per SC-001), 10 concurrent users (per SC-004)
**Constraints**: CORS enabled for local dev, no authentication required
**Scale/Scope**: Single backend instance, single Docusaurus frontend

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Principle | Status | Notes |
|-----------|--------|-------|
| Library-First | N/A | Integration feature, not new library |
| CLI Interface | PASS | Existing agent.py CLI preserved |
| Test-First | PASS | API endpoint tests planned |
| Integration Testing | PASS | Frontend-backend integration tests planned |
| Observability | PASS | Logging already configured in agent.py |
| Simplicity | PASS | Minimal changes to existing code |

## Project Structure

### Documentation (this feature)

```text
specs/006-fastapi-rag-frontend/
├── plan.md              # This file
├── research.md          # Phase 0 output
├── data-model.md        # Phase 1 output
├── quickstart.md        # Phase 1 output
├── contracts/           # Phase 1 output (OpenAPI spec)
└── tasks.md             # Phase 2 output (/sp.tasks command)
```

### Source Code (repository root)

```text
backend/
├── agent.py             # Existing RAG agent (reused)
├── api.py               # NEW: FastAPI server
├── tools.py             # Existing agent tools (reused)
├── retrieve.py          # Existing retrieval client (reused)
├── models.py            # Existing data models (extended)
└── pyproject.toml       # Dependencies (FastAPI added)

frontend_book/
├── src/
│   ├── components/
│   │   └── ChatWidget/  # NEW: Chatbot UI component
│   │       ├── index.tsx
│   │       └── styles.module.css
│   ├── theme/
│   │   └── Root.tsx     # NEW: Wraps app with ChatWidget
│   └── css/
│       └── custom.css   # Updated for chatbot styles
├── docusaurus.config.ts # No changes needed
└── package.json         # No new dependencies needed
```

**Structure Decision**: Web application structure with `backend/` containing the FastAPI server and `frontend_book/` containing the Docusaurus site. The chatbot widget is a standalone React component injected at the Root level to appear on all pages.

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────┐
│                    Docusaurus Frontend                       │
│  ┌─────────────────────────────────────────────────────────┐│
│  │                   ChatWidget Component                   ││
│  │  - Floating button (bottom-right)                       ││
│  │  - Expandable chat panel                                ││
│  │  - Message history display                              ││
│  │  - Input form with submit                               ││
│  └─────────────────────────────────────────────────────────┘│
│                            │                                 │
│                   fetch() to backend                         │
└────────────────────────────┼────────────────────────────────┘
                             │
                             ▼
┌─────────────────────────────────────────────────────────────┐
│                    FastAPI Backend (api.py)                  │
│  ┌─────────────────────────────────────────────────────────┐│
│  │  POST /api/query                                        ││
│  │  - Accepts: { query, session_id?, context? }            ││
│  │  - Returns: { response, citations[], session_id }       ││
│  └─────────────────────────────────────────────────────────┘│
│  ┌─────────────────────────────────────────────────────────┐│
│  │  GET /api/health                                        ││
│  │  - Returns: { status: "ok" }                            ││
│  └─────────────────────────────────────────────────────────┘│
│                            │                                 │
│                   Calls agent.py functions                   │
└────────────────────────────┼────────────────────────────────┘
                             │
                             ▼
┌─────────────────────────────────────────────────────────────┐
│                 Existing RAG Agent (agent.py)                │
│  - OpenAI Agents SDK with search_book_content tool          │
│  - SQLiteSession for conversation memory                     │
│  - Qdrant + Cohere retrieval pipeline                       │
└─────────────────────────────────────────────────────────────┘
```

## Implementation Phases

### Phase 1: Backend API (api.py)

**Goal**: Create FastAPI server in `backend/api.py` that wraps the existing agent.

**Tasks**:
1. Add FastAPI and uvicorn to `backend/pyproject.toml`
2. Create `backend/api.py` with:
   - FastAPI app instance with CORS middleware
   - `POST /api/query` endpoint calling `run_single_query()` from agent.py
   - `GET /api/health` endpoint for monitoring
   - Pydantic request/response models
3. Refactor agent.py to expose `run_query_async()` function for API use
4. Add session management using SQLiteSession for multi-turn conversations

**Acceptance Criteria**:
- `uv run api.py` starts server on localhost:8000
- `curl -X POST http://localhost:8000/api/query -d '{"query":"What is ROS 2?"}'` returns JSON response
- Response includes `response`, `citations`, and `session_id` fields

### Phase 2: Frontend Chatbot Widget

**Goal**: Create a chatbot UI component that displays on all book pages.

**Tasks**:
1. Create `frontend_book/src/components/ChatWidget/index.tsx`:
   - Floating action button (bottom-right corner)
   - Expandable chat panel with message history
   - Input form for user questions
   - Loading state during API calls
   - Error display for failed requests
2. Create `frontend_book/src/components/ChatWidget/styles.module.css`:
   - Responsive styles for desktop and mobile
   - Theme-aware colors (light/dark mode)
3. Create `frontend_book/src/theme/Root.tsx` to wrap app with ChatWidget
4. Update `frontend_book/src/css/custom.css` for global chatbot styles

**Acceptance Criteria**:
- Chatbot button visible on all pages
- Clicking button opens chat panel
- User can type and submit questions
- Responses display with loading indicator
- Panel is usable on mobile devices

### Phase 3: Integration & Testing

**Goal**: Verify end-to-end functionality.

**Tasks**:
1. Test local development workflow:
   - Backend: `cd backend && uv run api.py`
   - Frontend: `cd frontend_book && npm start`
2. Verify CORS works between frontend (localhost:3000) and backend (localhost:8000)
3. Test conversation continuity (session persistence)
4. Test error handling (backend unavailable, timeout)
5. Create integration test for API endpoint

**Acceptance Criteria**:
- End-to-end query works: frontend → backend → agent → response displayed
- Session ID persists across multiple queries
- Error messages display correctly when backend fails

## API Contract Summary

### POST /api/query

**Request**:
```json
{
  "query": "What is ROS 2?",
  "session_id": "optional-session-id",
  "context": "optional selected text"
}
```

**Response (200 OK)**:
```json
{
  "response": "ROS 2 is a middleware framework...",
  "citations": [
    {"title": "ROS 2 Fundamentals", "url": "https://..."}
  ],
  "session_id": "session_abc123"
}
```

**Response (500 Error)**:
```json
{
  "error": "Failed to process query",
  "detail": "Connection to OpenAI failed"
}
```

### GET /api/health

**Response (200 OK)**:
```json
{
  "status": "ok",
  "version": "1.0.0"
}
```

## Dependencies to Add

### Backend (pyproject.toml)
```toml
dependencies = [
    # ... existing ...
    "fastapi>=0.115.0",
    "uvicorn[standard]>=0.32.0",
]
```

### Frontend
No new npm dependencies required. Uses native `fetch()` API.

## Risk Mitigation

| Risk | Mitigation |
|------|------------|
| CORS issues in development | Configure FastAPI CORSMiddleware with localhost origins |
| Agent import errors | Refactor agent.py exports, test imports |
| Session memory conflicts | Use unique session IDs per browser session |
| Slow response times | Add timeout handling, loading UI feedback |

## Complexity Tracking

No constitution violations. Implementation uses minimal dependencies and reuses existing code.
