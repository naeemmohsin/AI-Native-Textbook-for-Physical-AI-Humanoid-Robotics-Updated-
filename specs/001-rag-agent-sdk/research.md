# Research: RAG Agent with OpenAI Agents SDK

**Feature**: 001-rag-agent-sdk
**Date**: 2025-12-26

## Research Tasks Resolved

### 1. OpenAI Agents SDK Integration Pattern

**Decision**: Use `@function_tool` decorator to wrap existing RetrievalClient.search() as an agent tool

**Rationale**:
- Decorator approach provides automatic schema generation from type annotations
- Existing RetrievalClient already handles Qdrant/Cohere integration
- Minimal code changes required - wrap existing function rather than rewrite

**Alternatives Considered**:
- Manual FunctionTool creation: More verbose, unnecessary given clean existing API
- Hosted tools (FileSearchTool): Requires OpenAI Vector Stores, not compatible with Qdrant

**Implementation Pattern**:
```python
from agents import Agent, Runner, function_tool

@function_tool
def search_book_content(query: str, top_k: int = 5) -> str:
    """Search the robotics book for relevant content.

    Args:
        query: Natural language search query
        top_k: Number of results to return
    """
    client = RetrievalClient()
    response = client.search(Query(text=query, top_k=top_k))
    return format_results_for_agent(response)
```

**Source**: [OpenAI Agents SDK Tools](https://openai.github.io/openai-agents-python/tools/)

---

### 2. Conversation Context Management

**Decision**: Use in-memory list with `to_input_list()` for conversation history

**Rationale**:
- CLI-only interface doesn't need persistence
- Simplest approach for multi-turn conversations
- No external dependencies (SQLite, Redis) required
- Matches "Out of Scope: Persistent conversation storage" requirement

**Alternatives Considered**:
- SQLiteSession: Overkill for CLI, adds unnecessary dependency
- OpenAI Conversations API: Requires additional API calls
- Redis: Enterprise feature, not needed for CLI

**Implementation Pattern**:
```python
conversation_history = []

while True:
    user_input = input("You: ")
    result = await Runner.run(
        agent,
        conversation_history + [{"role": "user", "content": user_input}]
    )
    conversation_history = result.to_input_list()
    print(f"Agent: {result.final_output}")
```

**Source**: [OpenAI Agents SDK Running Agents](https://openai.github.io/openai-agents-python/running_agents/)

---

### 3. Grounding Responses in Retrieved Content Only

**Decision**: Use strict system prompt with explicit instructions to only use provided context

**Rationale**:
- Agent instructions control LLM behavior
- Combined with score threshold (< 0.5 = out of scope), ensures grounded responses
- No hallucination when retrieval returns empty/low scores

**Implementation Pattern**:
```python
AGENT_INSTRUCTIONS = """You are a helpful assistant that answers questions about the ROS 2 Robotics book.

IMPORTANT RULES:
1. ONLY use information from the search_book_content tool results
2. If the tool returns no results or low-relevance results, say "I couldn't find information about that in the book"
3. NEVER make up or assume information not in the retrieved content
4. Always cite the source (title, URL) when providing information
5. If asked about topics not covered in the book, politely explain the topic is out of scope

The book covers: ROS 2, Gazebo simulation, URDF, Digital Twins, NVIDIA Isaac, and Vision-Language-Action models."""
```

---

### 4. Agent Response Format with Source Attribution

**Decision**: Format retrieval results as structured text with clear source markers

**Rationale**:
- Agent can extract and cite sources from formatted results
- Consistent format helps LLM understand context boundaries
- URL and title included for each chunk

**Implementation Pattern**:
```python
def format_results_for_agent(response: RetrievalResponse) -> str:
    if not response.results or response.error:
        return "NO_RELEVANT_CONTENT_FOUND"

    formatted = []
    for i, result in enumerate(response.results, 1):
        formatted.append(f"[Source {i}]")
        formatted.append(f"Title: {result.title}")
        formatted.append(f"URL: {result.url}")
        formatted.append(f"Relevance: {result.score:.2f}")
        formatted.append(f"Content: {result.text}")
        formatted.append("")

    return "\n".join(formatted)
```

---

### 5. Error Handling Strategy

**Decision**: Wrap tool errors and return user-friendly messages through agent

**Rationale**:
- Agent can provide context-aware error messages
- Maintains conversation flow even on failures
- Errors become part of agent response, not exceptions

**Implementation Pattern**:
```python
@function_tool
def search_book_content(query: str, top_k: int = 5) -> str:
    try:
        client = RetrievalClient()
        response = client.search(Query(text=query, top_k=top_k))
        if response.error:
            return f"SEARCH_ERROR: {response.error}"
        return format_results_for_agent(response)
    except Exception as e:
        return f"SEARCH_ERROR: Unable to search - {str(e)}"
```

---

### 6. CLI Interface Design

**Decision**: Interactive REPL with conversation loop

**Rationale**:
- Matches User Story 4 (conversational context)
- Simple input() loop for multi-turn
- Exit commands for clean termination

**Implementation Pattern**:
```python
async def main():
    agent = create_agent()
    print("RAG Agent ready. Type 'quit' to exit.\n")

    conversation_history = []
    while True:
        user_input = input("You: ").strip()
        if user_input.lower() in ["quit", "exit", "q"]:
            break
        if not user_input:
            continue

        result = await Runner.run(agent, ...)
        print(f"\nAgent: {result.final_output}\n")
```

---

## Dependencies

| Package | Version | Purpose |
|---------|---------|---------|
| openai-agents | >=0.6.4 | Agent SDK framework |
| openai | >=1.0.0 | Underlying OpenAI client |
| cohere | >=5.0.0 | Query embeddings (existing) |
| qdrant-client | >=1.12.0 | Vector search (existing) |
| python-dotenv | >=1.0.0 | Environment variables (existing) |

---

## Environment Variables Required

| Variable | Purpose |
|----------|---------|
| OPENAI_API_KEY | OpenAI API authentication for Agent SDK |
| COHERE_API_KEY | Cohere embeddings (existing) |
| QDRANT_URL | Qdrant connection (existing) |
| QDRANT_API_KEY | Qdrant authentication (existing) |
