# Quickstart: FastAPI RAG Frontend Integration

**Feature**: 006-fastapi-rag-frontend
**Date**: 2024-12-28

## Prerequisites

- Python 3.13+ with `uv` package manager
- Node.js 20+ with npm
- Environment variables configured in `.env`:
  - `OPENAI_API_KEY` - OpenAI API key
  - `COHERE_API_KEY` - Cohere API key for embeddings
  - `QDRANT_URL` - Qdrant database URL
  - `QDRANT_API_KEY` - Qdrant API key

## Quick Start

### 1. Start the Backend API

```bash
# Navigate to backend directory
cd backend

# Install dependencies (if not already done)
uv sync

# Start the FastAPI server
uv run api.py
```

The API will be available at `http://localhost:8000`

**Verify it's running**:
```bash
curl http://localhost:8000/api/health
# Expected: {"status":"ok","version":"1.0.0"}
```

### 2. Start the Frontend

```bash
# In a new terminal, navigate to frontend
cd frontend_book

# Install dependencies (if not already done)
npm install

# Start the development server
npm start
```

The frontend will be available at `http://localhost:3000`

### 3. Test the Integration

1. Open `http://localhost:3000` in your browser
2. Click the chat button in the bottom-right corner
3. Type a question like "What is ROS 2?"
4. Verify you receive a response with citations

## API Endpoints

### POST /api/query

Submit a question to the RAG agent.

**Request**:
```bash
curl -X POST http://localhost:8000/api/query \
  -H "Content-Type: application/json" \
  -d '{"query": "What is ROS 2?"}'
```

**Response**:
```json
{
  "response": "ROS 2 is a middleware framework...",
  "citations": [
    {"title": "ROS 2 Fundamentals", "url": "https://..."}
  ],
  "session_id": "session_abc123"
}
```

### GET /api/health

Check API health status.

```bash
curl http://localhost:8000/api/health
```

## Development Workflow

### Running Both Services

**Terminal 1 (Backend)**:
```bash
cd backend && uv run api.py
```

**Terminal 2 (Frontend)**:
```bash
cd frontend_book && npm start
```

### Testing Changes

**Backend changes**: The FastAPI server auto-reloads on file changes.

**Frontend changes**: The Docusaurus dev server hot-reloads on file changes.

## Troubleshooting

### CORS Errors

If you see CORS errors in the browser console:
1. Verify backend is running on port 8000
2. Check that CORS middleware is configured in `api.py`
3. Ensure frontend is accessing `http://localhost:8000` (not `127.0.0.1`)

### Agent Errors

If queries fail with agent errors:
1. Check `.env` file has valid API keys
2. Verify Qdrant database is accessible
3. Check backend logs for detailed error messages

### Empty Responses

If responses have no citations:
1. Verify the Qdrant collection has indexed content
2. Check the query matches topics covered in the book
3. Try a more specific query

## Project Structure

```
textbook/
├── backend/
│   ├── api.py           # FastAPI server (this feature)
│   ├── agent.py         # RAG agent (existing)
│   ├── tools.py         # Agent tools (existing)
│   └── pyproject.toml   # Python dependencies
│
├── frontend_book/
│   ├── src/
│   │   ├── components/
│   │   │   └── ChatWidget/  # Chatbot UI (this feature)
│   │   └── theme/
│   │       └── Root.tsx     # App wrapper (this feature)
│   └── package.json
│
└── .env                 # Environment variables
```
