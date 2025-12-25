"""
Data models for RAG Retrieval Pipeline.

Defines the core entities: Query, RetrievalResult, and RetrievalResponse.
"""

from dataclasses import dataclass, field
from typing import Optional


@dataclass
class Query:
    """
    Represents a search request from the user.

    Attributes:
        text: Natural language query text (required, non-empty, <= 1000 chars)
        top_k: Maximum results to return (1-100, default 5)
        score_threshold: Minimum similarity score 0-1 (default 0.0)
    """
    text: str
    top_k: int = 5
    score_threshold: float = 0.0

    def __post_init__(self):
        """Validate query parameters after initialization."""
        # Validation happens in RetrievalClient.validate_query()
        pass


@dataclass
class RetrievalResult:
    """
    Represents a single retrieved document chunk with metadata.

    Attributes:
        text: The text content of the chunk
        url: Source URL of the original document
        title: Title of the source document
        chunk_index: Position of this chunk within the document
        score: Similarity score (0-1, higher = more relevant)
    """
    text: str
    url: str
    title: str
    chunk_index: int
    score: float


@dataclass
class RetrievalResponse:
    """
    Collection of results plus query metadata.

    Attributes:
        query: The original query text
        results: Ordered list of matching chunks
        total_results: Number of results returned
        execution_time_ms: Time taken for retrieval in milliseconds
        error: Error message if retrieval failed, None otherwise
    """
    query: str
    results: list[RetrievalResult] = field(default_factory=list)
    total_results: int = 0
    execution_time_ms: float = 0.0
    error: Optional[str] = None

    @property
    def is_success(self) -> bool:
        """Check if the retrieval was successful (no error)."""
        return self.error is None

    def to_dict(self) -> dict:
        """Convert response to dictionary for JSON serialization."""
        return {
            "query": self.query,
            "total_results": self.total_results,
            "execution_time_ms": self.execution_time_ms,
            "results": [
                {
                    "text": r.text,
                    "url": r.url,
                    "title": r.title,
                    "chunk_index": r.chunk_index,
                    "score": r.score
                }
                for r in self.results
            ],
            "error": self.error
        }
