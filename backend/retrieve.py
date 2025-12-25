"""
RAG Retrieval Module for Book Content Search.

This module provides vector similarity search against the indexed book content
stored in Qdrant. It uses Cohere embeddings for queries and returns ranked
results with metadata.

Usage:
    uv run retrieve.py "What is ROS 2?"
    uv run retrieve.py --json --top-k 10 "URDF tutorial"
    uv run retrieve.py --threshold 0.7 "Gazebo simulation"
"""

import argparse
import json
import logging
import os
import sys
import time
from typing import Optional

import cohere
from dotenv import load_dotenv
from qdrant_client import QdrantClient
from qdrant_client.http.exceptions import UnexpectedResponse

from models import Query, RetrievalResult, RetrievalResponse
from errors import (
    ValidationError,
    ConnectionError,
    AuthenticationError,
    CollectionNotFoundError,
    EmbeddingError,
)

# Load environment variables
load_dotenv()

# Configuration
COLLECTION_NAME = "rag-embedding"
EMBEDDING_MODEL = "embed-english-v3.0"
EMBEDDING_DIMENSION = 1024
MAX_QUERY_LENGTH = 1000
DEFAULT_TOP_K = 5
DEFAULT_THRESHOLD = 0.0

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S"
)
logger = logging.getLogger(__name__)


class RetrievalClient:
    """
    Client for retrieving relevant book content from Qdrant vector database.

    Initializes connections to Qdrant and Cohere, and provides search functionality
    with configurable parameters.
    """

    def __init__(self):
        """Initialize the retrieval client with Qdrant and Cohere connections."""
        self.cohere_client: Optional[cohere.Client] = None
        self.qdrant_client: Optional[QdrantClient] = None
        self._initialize_clients()

    def _initialize_clients(self) -> None:
        """Initialize Qdrant and Cohere clients from environment variables."""
        # Get credentials from environment
        cohere_api_key = os.getenv("COHERE_API_KEY")
        qdrant_url = os.getenv("QDRANT_URL")
        qdrant_api_key = os.getenv("QDRANT_API_KEY")

        if not cohere_api_key:
            raise AuthenticationError("COHERE_API_KEY environment variable not set")

        if not qdrant_url:
            raise ConnectionError("QDRANT_URL environment variable not set")

        # Initialize Cohere client
        try:
            self.cohere_client = cohere.Client(api_key=cohere_api_key)
            logger.debug("Cohere client initialized")
        except Exception as e:
            raise AuthenticationError(f"Failed to initialize Cohere client: {e}")

        # Initialize Qdrant client
        try:
            self.qdrant_client = QdrantClient(
                url=qdrant_url,
                api_key=qdrant_api_key if qdrant_api_key else None
            )
            logger.debug("Qdrant client initialized")
        except Exception as e:
            raise ConnectionError(f"Failed to initialize Qdrant client: {e}")

    def validate_query(self, query: Query) -> None:
        """
        Validate query parameters.

        Args:
            query: The Query object to validate

        Raises:
            ValidationError: If query is invalid
        """
        # Check for empty query
        text = query.text.strip() if query.text else ""
        if not text:
            raise ValidationError("Query text cannot be empty")

        # Check query length
        if len(text) > MAX_QUERY_LENGTH:
            raise ValidationError(
                f"Query too long ({len(text)} chars). Maximum is {MAX_QUERY_LENGTH} characters."
            )

        # Validate top_k
        if query.top_k < 1 or query.top_k > 100:
            raise ValidationError("top_k must be between 1 and 100")

        # Validate score_threshold
        if query.score_threshold < 0.0 or query.score_threshold > 1.0:
            raise ValidationError("score_threshold must be between 0.0 and 1.0")

    def _generate_query_embedding(self, text: str) -> list[float]:
        """
        Generate embedding for query text using Cohere.

        Uses input_type="search_query" for asymmetric embedding (optimal for retrieval).

        Args:
            text: The query text to embed

        Returns:
            List of floats representing the embedding vector

        Raises:
            EmbeddingError: If embedding generation fails
        """
        try:
            response = self.cohere_client.embed(
                texts=[text],
                model=EMBEDDING_MODEL,
                input_type="search_query"  # Asymmetric embedding for queries
            )
            return response.embeddings[0]
        except Exception as e:
            raise EmbeddingError(f"Failed to generate query embedding: {e}")

    def _normalize_score(self, cosine_score: float) -> float:
        """
        Normalize cosine similarity score from [-1, 1] to [0, 1].

        Args:
            cosine_score: Raw cosine similarity score

        Returns:
            Normalized score in [0, 1] range
        """
        return (cosine_score + 1) / 2

    def _check_collection_exists(self) -> bool:
        """
        Check if the Qdrant collection exists.

        Returns:
            True if collection exists, False otherwise
        """
        try:
            collections = self.qdrant_client.get_collections().collections
            return any(c.name == COLLECTION_NAME for c in collections)
        except Exception:
            return False

    def search(
        self,
        query: Query
    ) -> RetrievalResponse:
        """
        Search for relevant book content matching the query.

        Args:
            query: Query object with text and search parameters

        Returns:
            RetrievalResponse with results and metadata
        """
        start_time = time.time()
        query_text = query.text.strip()

        try:
            # Validate query
            self.validate_query(query)

            # Check collection exists
            if not self._check_collection_exists():
                raise CollectionNotFoundError(COLLECTION_NAME)

            # Generate query embedding
            logger.info(f"Generating embedding for query: {query_text[:50]}...")
            query_embedding = self._generate_query_embedding(query_text)

            # Perform vector search
            logger.info(f"Searching Qdrant (top_k={query.top_k}, threshold={query.score_threshold})")

            # Perform vector search using query_points (new API)
            search_results = self.qdrant_client.query_points(
                collection_name=COLLECTION_NAME,
                query=query_embedding,
                limit=query.top_k,
            ).points

            # Convert to RetrievalResult objects with normalized scores
            results = []
            for point in search_results:
                normalized_score = self._normalize_score(point.score)

                # Apply score threshold filter
                if normalized_score >= query.score_threshold:
                    results.append(RetrievalResult(
                        text=point.payload.get("text", ""),
                        url=point.payload.get("url", ""),
                        title=point.payload.get("title", ""),
                        chunk_index=point.payload.get("chunk_index", 0),
                        score=round(normalized_score, 4)
                    ))

            # Calculate execution time
            execution_time_ms = (time.time() - start_time) * 1000

            logger.info(f"Found {len(results)} results in {execution_time_ms:.1f}ms")

            return RetrievalResponse(
                query=query_text,
                results=results,
                total_results=len(results),
                execution_time_ms=round(execution_time_ms, 2),
                error=None
            )

        except (ValidationError, ConnectionError, AuthenticationError,
                CollectionNotFoundError, EmbeddingError) as e:
            execution_time_ms = (time.time() - start_time) * 1000
            logger.error(f"Retrieval error: {e.message}")
            return RetrievalResponse(
                query=query_text,
                results=[],
                total_results=0,
                execution_time_ms=round(execution_time_ms, 2),
                error=e.message
            )
        except UnexpectedResponse as e:
            execution_time_ms = (time.time() - start_time) * 1000
            logger.error(f"Qdrant error: {e}")
            return RetrievalResponse(
                query=query_text,
                results=[],
                total_results=0,
                execution_time_ms=round(execution_time_ms, 2),
                error=f"Qdrant error: {e}"
            )
        except Exception as e:
            execution_time_ms = (time.time() - start_time) * 1000
            logger.error(f"Unexpected error: {e}")
            return RetrievalResponse(
                query=query_text,
                results=[],
                total_results=0,
                execution_time_ms=round(execution_time_ms, 2),
                error=f"Unexpected error: {e}"
            )


def format_human_readable(response: RetrievalResponse) -> str:
    """
    Format retrieval response for human-readable output.

    Args:
        response: The RetrievalResponse to format

    Returns:
        Formatted string for terminal display
    """
    lines = []

    if response.error:
        lines.append(f"Error: {response.error}")
        return "\n".join(lines)

    lines.append(f"Query: {response.query}")
    lines.append(f"Results: {response.total_results} found in {response.execution_time_ms:.0f}ms")
    lines.append("")

    if not response.results:
        lines.append("No relevant results found.")
        return "\n".join(lines)

    for i, result in enumerate(response.results, 1):
        lines.append(f"[{i}] Score: {result.score:.2f}")
        lines.append(f"    Title: {result.title}")
        lines.append(f"    URL: {result.url}")
        lines.append(f"    Chunk: {result.chunk_index}")
        # Truncate text for display
        text_preview = result.text[:200] + "..." if len(result.text) > 200 else result.text
        lines.append(f"    Text: {text_preview}")
        lines.append("")

    return "\n".join(lines)


def format_json(response: RetrievalResponse) -> str:
    """
    Format retrieval response as JSON.

    Args:
        response: The RetrievalResponse to format

    Returns:
        JSON string
    """
    return json.dumps(response.to_dict(), indent=2)


def main():
    """CLI entry point for retrieval."""
    parser = argparse.ArgumentParser(
        description="Search indexed book content using vector similarity",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
    uv run retrieve.py "What is ROS 2?"
    uv run retrieve.py --json "URDF tutorial"
    uv run retrieve.py --top-k 10 --threshold 0.7 "Gazebo simulation"
        """
    )
    parser.add_argument(
        "query",
        type=str,
        help="Natural language search query"
    )
    parser.add_argument(
        "--top-k",
        type=int,
        default=DEFAULT_TOP_K,
        help=f"Number of results to return (default: {DEFAULT_TOP_K})"
    )
    parser.add_argument(
        "--threshold",
        type=float,
        default=DEFAULT_THRESHOLD,
        help=f"Minimum similarity score 0-1 (default: {DEFAULT_THRESHOLD})"
    )
    parser.add_argument(
        "--json",
        action="store_true",
        help="Output results in JSON format"
    )
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="Enable verbose logging"
    )

    args = parser.parse_args()

    # Set logging level
    if args.verbose:
        logging.getLogger().setLevel(logging.DEBUG)
    else:
        logging.getLogger().setLevel(logging.WARNING)

    try:
        # Initialize client
        client = RetrievalClient()

        # Create query
        query = Query(
            text=args.query,
            top_k=args.top_k,
            score_threshold=args.threshold
        )

        # Perform search
        response = client.search(query)

        # Output results
        if args.json:
            print(format_json(response))
        else:
            print(format_human_readable(response))

        # Exit code based on results
        if response.error:
            sys.exit(1)
        elif response.total_results == 0:
            sys.exit(2)
        else:
            sys.exit(0)

    except Exception as e:
        if args.json:
            error_response = {
                "query": args.query,
                "total_results": 0,
                "execution_time_ms": 0,
                "results": [],
                "error": str(e)
            }
            print(json.dumps(error_response, indent=2))
        else:
            print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
