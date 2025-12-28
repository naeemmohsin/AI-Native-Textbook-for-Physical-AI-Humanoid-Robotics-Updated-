# Research: FastAPI RAG Frontend Integration

**Date**: 2024-12-28
**Feature**: 006-fastapi-rag-frontend

## Research Questions Resolved

### 1. FastAPI + OpenAI Agents SDK Integration

**Decision**: Use async FastAPI endpoints with direct agent function calls

**Rationale**:
- The existing `agent.py` uses async functions (`run_single_query`, `run_interactive`)
- FastAPI natively supports async endpoints, making integration seamless
- No need to wrap sync code or use thread pools

**Alternatives Considered**:
- Background tasks with polling: Rejected - adds complexity, not needed for <10s responses
- WebSocket streaming: Out of scope per spec (full response at once)
- Celery worker queue: Overkill for single-instance deployment

### 2. Docusaurus Chatbot Integration Pattern

**Decision**: Use Docusaurus Root component swizzling with standalone React component

**Rationale**:
- Docusaurus supports `src/theme/Root.tsx` to wrap the entire app
- ChatWidget can be injected at Root level to appear on all pages
- No need to modify individual page components or layouts
- Works with both docs pages and custom pages

**Alternatives Considered**:
- Docusaurus plugin: Rejected - more complex, requires plugin API knowledge
- Layout component override: Rejected - would need to swizzle full Layout
- Global script injection: Rejected - less React-native, harder to manage state

### 3. Session Management Approach

**Decision**: Browser-generated UUID stored in localStorage, passed to backend

**Rationale**:
- Simple client-side session ID generation
- Backend uses existing SQLiteSession for conversation memory
- No authentication complexity
- Session persists across page navigations but not browser sessions (per spec)

**Alternatives Considered**:
- Server-generated session IDs: Rejected - requires extra round-trip
- Cookie-based sessions: Rejected - adds CORS cookie complexity
- JWT tokens: Rejected - out of scope (no auth required)

### 4. CORS Configuration

**Decision**: Allow localhost origins in development, configurable for production

**Rationale**:
- Docusaurus dev server runs on localhost:3000
- FastAPI backend runs on localhost:8000
- Simple CORSMiddleware configuration with explicit origins

**Implementation**:
```python
from fastapi.middleware.cors import CORSMiddleware

app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:3000", "http://127.0.0.1:3000"],
    allow_methods=["GET", "POST"],
    allow_headers=["*"],
)
```

### 5. Citation Extraction from Agent Response

**Decision**: Parse agent response for source citations, return as structured data

**Rationale**:
- Current agent responses include "Source:" markers with title and URL
- Can extract citations using regex or string parsing
- Frontend can render citations as clickable links

**Implementation Approach**:
- Agent response already contains formatted citations
- API layer extracts URLs and titles from response text
- Returns both raw response and structured citations array

## Technology Decisions

| Component | Choice | Version |
|-----------|--------|---------|
| Backend Framework | FastAPI | >=0.115.0 |
| ASGI Server | uvicorn | >=0.32.0 |
| Frontend Framework | React (via Docusaurus) | 19.x |
| HTTP Client (frontend) | Native fetch API | - |
| Session Storage (frontend) | localStorage | - |
| Session Storage (backend) | SQLiteSession | existing |

## Best Practices Applied

1. **API Design**: RESTful endpoints with clear request/response contracts
2. **Error Handling**: Structured error responses with status codes
3. **CORS Security**: Explicit origin allowlist, not wildcard
4. **Async/Await**: Full async chain from API to agent
5. **Component Isolation**: ChatWidget as self-contained component
6. **Theme Compatibility**: CSS variables for light/dark mode support
