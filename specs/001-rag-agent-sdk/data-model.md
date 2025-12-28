# Data Model: RAG Agent with OpenAI Agents SDK

**Feature**: 001-rag-agent-sdk
**Date**: 2025-12-26

## Entities

### Agent Configuration

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| name | string | Yes | Agent identifier ("BookRAGAgent") |
| instructions | string | Yes | System prompt with grounding rules |
| model | string | Yes | LLM model identifier (e.g., "gpt-4o-mini") |
| tools | list[FunctionTool] | Yes | List of callable tools |

**Validation Rules**:
- Instructions must include grounding constraints
- At least one retrieval tool must be configured

---

### RetrievalToolInput

Input parameters for the retrieval tool function.

| Field | Type | Required | Default | Description |
|-------|------|----------|---------|-------------|
| query | string | Yes | - | Natural language search query |
| top_k | int | No | 5 | Number of results to retrieve |

**Validation Rules**:
- query cannot be empty or whitespace-only
- query max length: 1000 characters
- top_k range: 1-100

---

### RetrievalToolOutput

Formatted string output returned to the agent.

| Format | Description |
|--------|-------------|
| Success | Formatted sources with title, URL, relevance, content |
| No Results | "NO_RELEVANT_CONTENT_FOUND" |
| Error | "SEARCH_ERROR: {message}" |

**Output Format (Success)**:
```text
[Source 1]
Title: Module 1 - ROS 2 Fundamentals
URL: https://example.com/module-1
Relevance: 0.87
Content: ROS 2 is the second generation of the Robot Operating System...

[Source 2]
...
```

---

### ConversationMessage

Message in the conversation history.

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| role | string | Yes | "user" or "assistant" |
| content | string | Yes | Message text |

---

### AgentResponse

Final response from the agent after processing.

| Field | Type | Description |
|-------|------|-------------|
| final_output | string | Agent's generated response |
| sources_cited | list[string] | URLs referenced in response |
| is_grounded | bool | True if response based on retrieved content |
| is_out_of_scope | bool | True if query was out of scope |

---

## State Transitions

### Conversation Flow

```
[Idle] --user_input--> [Processing]
[Processing] --tool_call--> [Retrieving]
[Retrieving] --results--> [Generating]
[Generating] --response--> [Idle]

[Processing] --no_tool_needed--> [Generating]
[Retrieving] --error--> [Error_Handling]
[Error_Handling] --graceful_message--> [Idle]
```

### Query Classification

```
[Query Received]
    |
    v
[Retrieval Executed]
    |
    +-- results.length > 0 AND max_score >= 0.5 --> [In Scope - Generate Grounded Response]
    |
    +-- results.length == 0 OR max_score < 0.5 --> [Out of Scope - Generate "Not Found" Response]
    |
    +-- error occurred --> [Error - Generate Error Message]
```

---

## Relationships

```
Agent (1) --uses--> (1..n) Tools
Agent (1) --maintains--> (1) ConversationHistory
ConversationHistory (1) --contains--> (0..n) ConversationMessage
RetrievalTool (1) --wraps--> (1) RetrievalClient
RetrievalClient (1) --queries--> (1) QdrantCollection
```

---

## Reused Entities (from Spec 2)

The following entities are reused from the existing retrieval module:

| Entity | Location | Usage |
|--------|----------|-------|
| Query | backend/models.py | Passed to RetrievalClient |
| RetrievalResult | backend/models.py | Individual search result |
| RetrievalResponse | backend/models.py | Search response container |
| RetrievalClient | backend/retrieve.py | Qdrant/Cohere integration |
