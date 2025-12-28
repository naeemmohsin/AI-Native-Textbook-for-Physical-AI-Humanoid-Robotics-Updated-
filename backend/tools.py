"""
RAG Agent Tool Definitions.

Provides the retrieval tool for the RAG agent using the OpenAI Agents SDK.
Wraps the existing RetrievalClient to expose book content search as an agent tool.
"""

import logging
from agents import function_tool

from retrieve import RetrievalClient
from models import Query, RetrievalResponse

logger = logging.getLogger(__name__)

# Score threshold for relevant content
RELEVANCE_THRESHOLD = 0.5


def format_results_for_agent(response: RetrievalResponse) -> str:
    """
    Format retrieval results for the agent to use as context.

    Args:
        response: RetrievalResponse from the retrieval client

    Returns:
        Formatted string with sources or NO_RELEVANT_CONTENT marker
    """
    if response.error:
        return f"SEARCH_ERROR: {response.error}"

    if not response.results:
        return "NO_RELEVANT_CONTENT_FOUND"

    # Check if any results meet the relevance threshold
    relevant_results = [r for r in response.results if r.score >= RELEVANCE_THRESHOLD]

    if not relevant_results:
        return "NO_RELEVANT_CONTENT_FOUND"

    formatted = []
    for i, result in enumerate(relevant_results, 1):
        formatted.append(f"[Source {i}]")
        formatted.append(f"Title: {result.title}")
        formatted.append(f"URL: {result.url}")
        formatted.append(f"Relevance: {result.score:.2f}")
        formatted.append(f"Content: {result.text}")
        formatted.append("")

    return "\n".join(formatted)


@function_tool
def search_book_content(query: str, top_k: int = 5) -> str:
    """
    Search the robotics book for relevant content.

    Use this tool to find information from the ROS 2 Robotics book.
    The book covers topics including ROS 2, Gazebo simulation, URDF,
    Digital Twins, NVIDIA Isaac Sim, and Vision-Language-Action models.

    Args:
        query: Natural language search query describing what you want to find
        top_k: Number of results to retrieve (default 5)

    Returns:
        Formatted search results with source attribution, or indication that
        no relevant content was found
    """
    logger.info(f"Tool invoked: search_book_content(query='{query[:50]}...', top_k={top_k})")

    try:
        client = RetrievalClient()
        search_query = Query(text=query, top_k=top_k, score_threshold=0.0)
        response = client.search(search_query)

        result = format_results_for_agent(response)
        logger.info(f"Tool returned: {len(response.results)} results, relevant: {result != 'NO_RELEVANT_CONTENT_FOUND'}")

        return result

    except Exception as e:
        error_msg = f"SEARCH_ERROR: Unable to search - {str(e)}"
        logger.error(error_msg)
        return error_msg
