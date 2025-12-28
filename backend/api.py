"""
FastAPI server for RAG Agent.

Exposes the RAG agent via REST API for frontend integration.

Usage:
    uv run api.py
"""

import logging
import re
import uuid
from pathlib import Path
from typing import Optional

from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel, Field
import uvicorn

from agent import create_agent, AGENT_INSTRUCTIONS
from agents import Runner, SQLiteSession, trace

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S"
)
logger = logging.getLogger(__name__)

# =============================================================================
# T003: Pydantic Models
# =============================================================================

class Citation(BaseModel):
    """Source citation from book content."""
    title: str
    url: str


class QueryRequest(BaseModel):
    """Request body for query endpoint."""
    query: str = Field(..., min_length=1, max_length=5000)
    session_id: Optional[str] = None
    context: Optional[str] = Field(None, max_length=5000)


class QueryResponse(BaseModel):
    """Response body for query endpoint."""
    response: str
    citations: list[Citation]
    session_id: str


class ErrorResponse(BaseModel):
    """Error response body."""
    error: str
    detail: Optional[str] = None


class HealthResponse(BaseModel):
    """Health check response body."""
    status: str
    version: str


# =============================================================================
# T005: FastAPI App with CORS
# =============================================================================

app = FastAPI(
    title="RAG Book Agent API",
    description="API for querying the ROS 2 Robotics book using RAG-based AI agent",
    version="1.0.0"
)

# CORS middleware for frontend integration
app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        "http://localhost:3000",
        "http://127.0.0.1:3000",
        "http://localhost:3001",
    ],
    allow_credentials=True,
    allow_methods=["GET", "POST"],
    allow_headers=["*"],
)

# Create agent instance
agent = create_agent()

# Session storage path
DB_PATH = Path(__file__).parent / "conversations.db"


# =============================================================================
# T008: Citation Extraction
# =============================================================================

def extract_citations(response_text: str) -> list[Citation]:
    """
    Extract citations from agent response text.

    Looks for patterns like:
    - [Title](URL)
    - Title: ... URL: ...
    - Source: Title - URL
    """
    citations = []

    # Pattern 1: Markdown links [Title](URL)
    markdown_pattern = r'\[([^\]]+)\]\((https?://[^\)]+)\)'
    for match in re.finditer(markdown_pattern, response_text):
        title, url = match.groups()
        if title and url:
            citations.append(Citation(title=title.strip(), url=url.strip()))

    # Pattern 2: URL patterns with book domain
    url_pattern = r'(https://ai-native-textbook[^\s\)]+)'
    for match in re.finditer(url_pattern, response_text):
        url = match.group(1)
        # Extract title from URL path
        path_parts = url.rstrip('/').split('/')
        if path_parts:
            title = path_parts[-1].replace('-', ' ').title()
            # Avoid duplicates
            if not any(c.url == url for c in citations):
                citations.append(Citation(title=title, url=url))

    return citations


# =============================================================================
# T006: Health Endpoint
# =============================================================================

@app.get("/api/health", response_model=HealthResponse)
async def health_check():
    """Health check endpoint for monitoring."""
    return HealthResponse(status="ok", version="1.0.0")


# =============================================================================
# T007: Query Endpoint
# =============================================================================

@app.post("/api/query", response_model=QueryResponse)
async def submit_query(request: QueryRequest):
    """
    Submit a query to the RAG agent.

    Returns AI-generated response grounded in book content with citations.
    """
    logger.info(f"Received query: {request.query[:50]}...")

    # Generate or use provided session ID
    session_id = request.session_id or f"session_{uuid.uuid4().hex[:8]}"

    # Build query with optional context
    full_query = request.query
    if request.context:
        full_query = f"Context from the book:\n{request.context}\n\nQuestion: {request.query}"

    try:
        # Create session for conversation history
        session = SQLiteSession(session_id, str(DB_PATH))

        # Run agent with tracing
        with trace("API Query", group_id=session_id):
            result = await Runner.run(agent, full_query, session=session)

        response_text = result.final_output
        logger.info(f"Query complete, response length: {len(response_text)}")

        # Extract citations from response
        citations = extract_citations(response_text)

        return QueryResponse(
            response=response_text,
            citations=citations,
            session_id=session_id
        )

    except Exception as e:
        logger.error(f"Query failed: {e}")
        raise HTTPException(
            status_code=500,
            detail=ErrorResponse(
                error="Failed to process query",
                detail=str(e)
            ).model_dump()
        )


# =============================================================================
# T009: Uvicorn Runner
# =============================================================================

if __name__ == "__main__":
    uvicorn.run(
        "api:app",
        host="0.0.0.0",
        port=8000,
        reload=True,
        log_level="info"
    )
