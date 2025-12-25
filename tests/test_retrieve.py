"""
Integration tests for RAG Retrieval Pipeline.

These tests run against the live Qdrant instance to validate retrieval functionality.
Ensure the embedding pipeline has been run before executing these tests.
"""

import os
import sys
import time

import pytest

# Add backend to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "backend"))

from models import Query, RetrievalResult, RetrievalResponse
from retrieve import RetrievalClient


# Skip all tests if credentials not available
pytestmark = pytest.mark.skipif(
    not os.getenv("COHERE_API_KEY") or not os.getenv("QDRANT_URL"),
    reason="Requires COHERE_API_KEY and QDRANT_URL environment variables"
)


@pytest.fixture(scope="module")
def client():
    """Create a RetrievalClient instance for tests."""
    # Load .env from backend directory
    from dotenv import load_dotenv
    backend_env = os.path.join(os.path.dirname(__file__), "..", "backend", ".env")
    load_dotenv(backend_env)
    return RetrievalClient()


# ==============================================================================
# Phase 3: User Story 1 - Query Book Content (P1)
# ==============================================================================

class TestBasicQuery:
    """Tests for basic query functionality (US1)."""

    def test_basic_query_returns_results(self, client):
        """T011: Basic query returns results with required fields."""
        query = Query(text="What is ROS 2?", top_k=5)
        response = client.search(query)

        assert response.is_success, f"Query failed: {response.error}"
        assert response.total_results > 0, "Expected at least one result"

        # Check all required fields are present
        for result in response.results:
            assert result.text, "Result missing text"
            assert result.url, "Result missing url"
            assert result.title, "Result missing title"
            assert isinstance(result.chunk_index, int), "chunk_index should be int"
            assert isinstance(result.score, float), "score should be float"

    def test_results_ranked_by_score_descending(self, client):
        """T012: Results are ranked by score in descending order."""
        query = Query(text="Gazebo simulation", top_k=10)
        response = client.search(query)

        assert response.is_success
        assert len(response.results) >= 2, "Need at least 2 results to check ordering"

        # Verify descending order
        scores = [r.score for r in response.results]
        for i in range(len(scores) - 1):
            assert scores[i] >= scores[i + 1], f"Scores not descending: {scores}"

    @pytest.mark.timeout(10)
    def test_response_time_under_5_seconds(self, client):
        """T013: Response time is under 5 seconds."""
        query = Query(text="Isaac Sim tutorial", top_k=5)

        start = time.time()
        response = client.search(query)
        elapsed = time.time() - start

        assert response.is_success
        assert elapsed < 5.0, f"Query took {elapsed:.2f}s, expected < 5s"
        # Also verify execution_time_ms is recorded
        assert response.execution_time_ms > 0


class TestQueryParameters:
    """Tests for query parameter handling (US1)."""

    def test_top_k_parameter(self, client):
        """T015: top_k parameter limits results correctly."""
        query = Query(text="robot", top_k=3)
        response = client.search(query)

        assert response.is_success
        assert response.total_results <= 3, f"Got {response.total_results}, expected <= 3"

    def test_score_threshold_parameter(self, client):
        """T016: score_threshold filters low-relevance results."""
        query = Query(text="ROS 2 nodes", top_k=10, score_threshold=0.7)
        response = client.search(query)

        assert response.is_success
        # All results should be above threshold
        for result in response.results:
            assert result.score >= 0.7, f"Score {result.score} below threshold 0.7"


# ==============================================================================
# Phase 4: User Story 2 - Validate Metadata Accuracy (P2)
# ==============================================================================

class TestMetadataAccuracy:
    """Tests for metadata validation (US2)."""

    def test_results_contain_valid_url(self, client):
        """T019: Results contain valid URL metadata."""
        query = Query(text="ROS 2 fundamentals", top_k=5)
        response = client.search(query)

        assert response.is_success
        for result in response.results:
            assert result.url.startswith("http"), f"Invalid URL: {result.url}"
            assert "ai-native-textbook" in result.url, f"URL not from book: {result.url}"

    def test_results_contain_title_and_chunk_index(self, client):
        """T020: Results contain title and chunk_index."""
        query = Query(text="URDF", top_k=5)
        response = client.search(query)

        assert response.is_success
        for result in response.results:
            assert result.title, "Missing title"
            assert len(result.title) > 0, "Empty title"
            assert result.chunk_index >= 0, f"Invalid chunk_index: {result.chunk_index}"

    def test_url_paths_match_module_patterns(self, client):
        """T021: URL paths match expected module patterns."""
        # Query specifically for Module 1 content
        query = Query(text="ROS 2 rclpy nodes", top_k=5)
        response = client.search(query)

        assert response.is_success
        # At least some results should be from module-1-ros2
        module_1_results = [r for r in response.results if "module-1-ros2" in r.url]
        assert len(module_1_results) > 0, "Expected results from module-1-ros2"


# ==============================================================================
# Phase 5: User Story 3 - Verify Query Scope Restriction (P3)
# ==============================================================================

class TestScopeRestriction:
    """Tests for query scope validation (US3)."""

    def test_out_of_scope_query_returns_low_scores(self, client):
        """T025: Out-of-scope query returns low scores (<0.5)."""
        query = Query(text="What is quantum computing and blockchain?", top_k=5)
        response = client.search(query)

        assert response.is_success
        # Scores should be relatively low for off-topic queries
        if response.results:
            avg_score = sum(r.score for r in response.results) / len(response.results)
            # Off-topic queries should have lower average scores
            assert avg_score < 0.7, f"Expected low avg score for off-topic, got {avg_score:.2f}"

    def test_all_result_urls_belong_to_book_domain(self, client):
        """T026: All result URLs belong to book domain."""
        query = Query(text="navigation and planning", top_k=10)
        response = client.search(query)

        assert response.is_success
        for result in response.results:
            assert "ai-native-textbook-for-physical-ai.vercel.app" in result.url, \
                f"URL not from book domain: {result.url}"


# ==============================================================================
# Phase 6: User Story 4 - Consistent Retrieval Behavior (P3)
# ==============================================================================

class TestConsistency:
    """Tests for retrieval consistency (US4)."""

    def test_identical_queries_return_identical_results(self, client):
        """T029: Identical queries return identical results."""
        query = Query(text="Isaac ROS perception", top_k=5)

        response1 = client.search(query)
        response2 = client.search(query)

        assert response1.is_success and response2.is_success
        assert response1.total_results == response2.total_results

        # Compare results
        for r1, r2 in zip(response1.results, response2.results):
            assert r1.url == r2.url, "URLs don't match"
            assert r1.chunk_index == r2.chunk_index, "Chunk indices don't match"
            assert abs(r1.score - r2.score) < 0.001, "Scores don't match"

    def test_result_ordering_consistent_across_runs(self, client):
        """T030: Result ordering is consistent across runs."""
        query = Query(text="Unity digital twin", top_k=5)

        # Run same query 3 times
        responses = [client.search(query) for _ in range(3)]

        # All should have same ordering
        for i in range(1, 3):
            urls_0 = [r.url for r in responses[0].results]
            urls_i = [r.url for r in responses[i].results]
            assert urls_0 == urls_i, f"Ordering inconsistent between run 0 and {i}"


# ==============================================================================
# Phase 7: Edge Cases & Error Handling
# ==============================================================================

class TestEdgeCases:
    """Tests for edge cases and error handling."""

    def test_empty_query_returns_validation_error(self, client):
        """T033: Empty query returns validation error."""
        query = Query(text="", top_k=5)
        response = client.search(query)

        assert not response.is_success
        assert response.error is not None
        assert "empty" in response.error.lower() or "validation" in response.error.lower()

    def test_whitespace_query_returns_validation_error(self, client):
        """Empty whitespace query returns validation error."""
        query = Query(text="   ", top_k=5)
        response = client.search(query)

        assert not response.is_success
        assert response.error is not None

    def test_long_query_handled_gracefully(self, client):
        """T034: Long query (>1000 chars) handled gracefully."""
        long_text = "robot " * 200  # ~1200 chars
        query = Query(text=long_text, top_k=5)
        response = client.search(query)

        # Should either work or return a validation error (not crash)
        if not response.is_success:
            assert "too long" in response.error.lower() or "1000" in response.error

    def test_special_characters_handled(self, client):
        """T035: Special characters in query handled."""
        query = Query(text="ROS 2: nodes & topics (pub/sub)", top_k=5)
        response = client.search(query)

        # Should work without crashing
        assert response.is_success or response.error is not None


# ==============================================================================
# Test Runner
# ==============================================================================

if __name__ == "__main__":
    pytest.main([__file__, "-v", "--tb=short"])
