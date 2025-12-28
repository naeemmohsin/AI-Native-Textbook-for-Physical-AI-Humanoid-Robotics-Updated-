# Data Model: FastAPI RAG Frontend Integration

**Date**: 2024-12-28
**Feature**: 006-fastapi-rag-frontend

## Entities

### QueryRequest

Represents an incoming query from the frontend.

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| query | string | Yes | The user's question text |
| session_id | string | No | Session identifier for conversation continuity |
| context | string | No | Selected text context for contextual queries |

**Validation Rules**:
- `query` must be non-empty, max 5000 characters
- `session_id` if provided, must be valid UUID format
- `context` if provided, max 5000 characters (truncated if longer)

### QueryResponse

Represents the API response to a query.

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| response | string | Yes | The AI-generated answer text |
| citations | Citation[] | Yes | List of source citations (may be empty) |
| session_id | string | Yes | Session identifier for follow-up queries |

### Citation

Represents a source reference from the book.

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| title | string | Yes | Title of the source section |
| url | string | Yes | URL to the book page |

### ErrorResponse

Represents an error response.

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| error | string | Yes | Error message |
| detail | string | No | Additional error details |

### HealthResponse

Represents a health check response.

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| status | string | Yes | Health status ("ok") |
| version | string | Yes | API version |

## Frontend State

### ChatMessage

Represents a message in the chat UI.

| Field | Type | Description |
|-------|------|-------------|
| id | string | Unique message identifier |
| role | "user" \| "assistant" | Message sender |
| content | string | Message text |
| citations | Citation[] | Citations (assistant messages only) |
| timestamp | Date | When the message was created |

### ChatState

Represents the chatbot component state.

| Field | Type | Description |
|-------|------|-------------|
| isOpen | boolean | Whether chat panel is open |
| messages | ChatMessage[] | Conversation history |
| sessionId | string | Current session identifier |
| isLoading | boolean | Whether a query is in progress |
| error | string \| null | Current error message |

## Pydantic Models (Backend)

```python
from pydantic import BaseModel, Field
from typing import Optional

class Citation(BaseModel):
    title: str
    url: str

class QueryRequest(BaseModel):
    query: str = Field(..., min_length=1, max_length=5000)
    session_id: Optional[str] = None
    context: Optional[str] = Field(None, max_length=5000)

class QueryResponse(BaseModel):
    response: str
    citations: list[Citation]
    session_id: str

class ErrorResponse(BaseModel):
    error: str
    detail: Optional[str] = None

class HealthResponse(BaseModel):
    status: str
    version: str
```

## TypeScript Types (Frontend)

```typescript
interface Citation {
  title: string;
  url: string;
}

interface QueryRequest {
  query: string;
  session_id?: string;
  context?: string;
}

interface QueryResponse {
  response: string;
  citations: Citation[];
  session_id: string;
}

interface ErrorResponse {
  error: string;
  detail?: string;
}

interface ChatMessage {
  id: string;
  role: 'user' | 'assistant';
  content: string;
  citations?: Citation[];
  timestamp: Date;
}

interface ChatState {
  isOpen: boolean;
  messages: ChatMessage[];
  sessionId: string;
  isLoading: boolean;
  error: string | null;
}
```

## State Transitions

### Chat Session Lifecycle

```
[Closed] --click button--> [Open, Empty]
[Open, Empty] --submit query--> [Open, Loading]
[Open, Loading] --success--> [Open, With Messages]
[Open, Loading] --error--> [Open, Error Displayed]
[Open, *] --click close--> [Closed]
```

### Session ID Management

1. On first load: Check localStorage for existing session_id
2. If none exists: Generate new UUID, store in localStorage
3. On each query: Include session_id in request
4. Backend uses session_id with SQLiteSession for conversation memory
