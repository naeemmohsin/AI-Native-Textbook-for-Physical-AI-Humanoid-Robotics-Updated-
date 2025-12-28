"""
RAG Agent CLI for Book Content Search.

An AI agent that answers questions about the ROS 2 Robotics book using
retrieval-augmented generation. Built with the OpenAI Agents SDK.

Usage:
    uv run agent.py --query "What is ROS 2?"
    uv run agent.py  # Interactive mode
"""

import argparse
import asyncio
import json
import logging
import os
import sys
import uuid
from pathlib import Path
from dotenv import load_dotenv
from agents import Agent, Runner, SQLiteSession, trace
from agents import OpenAIChatCompletionsModel
from openai import AsyncOpenAI
from tools import search_book_content

# Load environment variables from backend directory
env_path = Path(__file__).parent / ".env"
load_dotenv(env_path)

# Initialize OpenAI client with API key from .env
client = AsyncOpenAI(
    api_key=os.getenv("OPENAI_API_KEY"),
)

# Use OpenAI model
openai_model = OpenAIChatCompletionsModel(
    openai_client=client,
    model="gpt-4o-mini"
)

# Configure logging
logging.basicConfig(
    level=logging.WARNING,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S"
)
logger = logging.getLogger(__name__)

# Agent instructions with grounding rules
AGENT_INSTRUCTIONS = """You are a helpful assistant that answers questions about the ROS 2 Robotics book.

IMPORTANT RULES:
1. ALWAYS use the search_book_content tool to find information before answering
2. ONLY use information from the tool results - never use external knowledge
3. If the tool returns NO_RELEVANT_CONTENT_FOUND, say "I couldn't find information about that in the book"
4. If the tool returns SEARCH_ERROR, apologize and ask the user to try again
5. NEVER make up or assume information not in the retrieved content
6. Always cite sources by mentioning the title and URL from the search results
7. If asked about topics not covered, politely explain what topics ARE covered

The book covers these topics:
- ROS 2 (Robot Operating System 2) fundamentals
- Gazebo simulation environment
- URDF (Unified Robot Description Format)
- Digital Twins for robotics
- NVIDIA Isaac Sim
- Vision-Language-Action (VLA) models

When you find relevant content, synthesize a clear answer and cite your sources."""


def create_agent() -> Agent:
    """Create the RAG agent with retrieval tool."""
    return Agent(
        name="BookRAGAgent",
        instructions=AGENT_INSTRUCTIONS,
        model=openai_model,
        tools=[search_book_content],
    )

async def run_single_query(
    agent: Agent, query: str, json_output: bool = False, verbose: bool = False
) -> str:
    """
    Run a single query and return the response.

    Args:
        agent: The configured agent
        query: User's question
        json_output: Whether to format output as JSON
        verbose: Whether to log usage statistics

    Returns:
        Agent's response string
    """
    logger.info(f"Processing query: {query[:50]}...")

    try:
        with trace("Single Query", group_id=str(uuid.uuid4().hex[:16])):
            result = await Runner.run(agent, query)

        logger.info("Query complete")

        # Log usage statistics in verbose mode
        if verbose:
            usage = result.context_wrapper.usage
            logger.info(
                f"Usage - Requests: {usage.requests}, "
                f"Input tokens: {usage.input_tokens}, "
                f"Output tokens: {usage.output_tokens}"
            )

        if json_output:
            response_data = {
                "query": query,
                "response": result.final_output,
            }
            if verbose:
                usage = result.context_wrapper.usage
                response_data["usage"] = {
                    "requests": usage.requests,
                    "input_tokens": usage.input_tokens,
                    "output_tokens": usage.output_tokens,
                    "total_tokens": usage.total_tokens,
                }
            return json.dumps(response_data, indent=2)

        return result.final_output

    except Exception as e:
        error_msg = str(e)
        if "401" in error_msg or "invalid_api_key" in error_msg:
            error_response = "Error: Invalid OpenAI API key. Please check your OPENAI_API_KEY in .env file."
        elif "connection" in error_msg.lower():
            error_response = "Error: Unable to connect to OpenAI API. Please check your network connection."
        else:
            error_response = f"Error: {error_msg}"

        logger.error(f"Query failed: {error_msg}")

        if json_output:
            return json.dumps({
                "query": query,
                "response": None,
                "error": error_response,
            }, indent=2)

        return error_response


async def run_interactive(agent: Agent, verbose: bool = False) -> None:
    """
    Run interactive REPL mode with SQLiteSession for conversation memory.

    Args:
        agent: The configured agent
        verbose: Whether to log usage statistics
    """
    print("RAG Agent ready. Ask questions about the ROS 2 Robotics book.")
    print("Type 'quit' or 'exit' to end the session.\n")

    # Create a unique session ID for this conversation
    session_id = f"session_{uuid.uuid4().hex[:8]}"
    db_path = Path(__file__).parent / "conversations.db"
    session = SQLiteSession(session_id, str(db_path))

    # Use trace for the entire interactive session
    with trace("Interactive Session", group_id=session_id):
        while True:
            try:
                user_input = input("You: ").strip()
            except (EOFError, KeyboardInterrupt):
                print("\nGoodbye!")
                break

            # Handle exit commands
            if user_input.lower() in ["quit", "exit", "q"]:
                print("Goodbye!")
                break

            # Handle empty input
            if not user_input:
                print("Please enter a question.\n")
                continue

            logger.info(f"Processing: {user_input[:50]}...")

            try:
                # Session automatically manages conversation history
                result = await Runner.run(agent, user_input, session=session)

                # Log usage in verbose mode
                if verbose:
                    usage = result.context_wrapper.usage
                    logger.info(
                        f"Usage - Requests: {usage.requests}, "
                        f"Input tokens: {usage.input_tokens}, "
                        f"Output tokens: {usage.output_tokens}"
                    )

                print(f"\nAgent: {result.final_output}\n")

            except Exception as e:
                logger.error(f"Error processing query: {e}")
                print(f"\nError: Unable to process your question. Please try again.\n")


def main():
    """CLI entry point."""
    parser = argparse.ArgumentParser(
        description="RAG Agent for ROS 2 Robotics Book",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
    uv run agent.py --query "What is ROS 2?"
    uv run agent.py --query "How do I use Gazebo?" --json
    uv run agent.py  # Interactive mode
        """
    )
    parser.add_argument(
        "--query", "-q",
        type=str,
        help="Single query to process (non-interactive mode)"
    )
    parser.add_argument(
        "--json",
        action="store_true",
        help="Output response in JSON format"
    )
    parser.add_argument(
        "--verbose", "-v",
        action="store_true",
        help="Enable verbose logging"
    )

    args = parser.parse_args()

    # Set logging level
    if args.verbose:
        logging.getLogger().setLevel(logging.INFO)
        logging.getLogger("agents").setLevel(logging.DEBUG)

    # Check for OpenAI API key
    if not os.getenv("OPENAI_API_KEY"):
        print("Error: OPENAI_API_KEY environment variable not set", file=sys.stderr)
        sys.exit(1)

    # Create agent
    agent = create_agent()

    # Run in appropriate mode
    if args.query:
        # Single query mode
        response = asyncio.run(
            run_single_query(agent, args.query, args.json, args.verbose)
        )
        print(response)
    else:
        # Interactive mode
        asyncio.run(run_interactive(agent, args.verbose))


if __name__ == "__main__":
    main()
