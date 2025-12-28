# Implementation Plan: RAG Agent with OpenAI Agents SDK

**Branch**: `001-rag-agent-sdk` | **Date**: 2025-12-26 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/001-rag-agent-sdk/spec.md`

## Summary

Build an AI agent using the OpenAI Agents SDK that provides retrieval-augmented generation over the indexed robotics book content. The agent uses a tool-based architecture where retrieval is exposed as a callable function, enabling grounded responses with source attribution and explicit handling of out-of-scope queries.

## Technical Context

**Language/Version**: Python 3.11+
**Primary Dependencies**: openai-agents >=0.6.4, cohere, qdrant-client, python-dotenv
**Storage**: N/A (in-memory conversation history only)
**Testing**: pytest with integration tests
**Target Platform**: CLI (Linux/Windows/macOS)
**Project Type**: Single project extending existing backend
**Performance Goals**: End-to-end response within 10 seconds
**Constraints**: Responses must be grounded in retrieved content only
**Scale/Scope**: Single-user CLI, no concurrent access required

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Principle | Status | Notes |
|-----------|--------|-------|
| Library-First | PASS | Agent module is self-contained, reusable |
| CLI Interface | PASS | Interactive REPL with single-query mode |
| Test-First | PASS | Integration tests defined for all user stories |
| Integration Testing | PASS | Tests verify Qdrant + Cohere + OpenAI integration |
| Observability | PASS | Structured logging for retrieval and generation |
| Simplicity | PASS | Wraps existing RetrievalClient, minimal new code |

## Project Structure

### Documentation (this feature)

```text
specs/001-rag-agent-sdk/
├── plan.md              # This file
├── research.md          # Phase 0 output - OpenAI Agents SDK patterns
├── data-model.md        # Phase 1 output - Entity definitions
├── quickstart.md        # Phase 1 output - Usage guide
└── tasks.md             # Phase 2 output (created by /sp.tasks)
```

### Source Code (repository root)

```text
backend/
├── agent.py             # NEW: Agent CLI entry point
├── tools.py             # NEW: Tool definitions for agent
├── retrieve.py          # EXISTING: RetrievalClient (reused)
├── models.py            # EXISTING: Data models (reused)
├── errors.py            # EXISTING: Error types (reused)
└── pyproject.toml       # MODIFIED: Add openai-agents dependency

tests/
├── test_retrieve.py     # EXISTING: Retrieval tests
└── test_agent.py        # NEW: Agent integration tests
```

**Structure Decision**: Single project structure extending the existing backend. New files `agent.py` and `tools.py` integrate with existing `retrieve.py` module.

## Complexity Tracking

No constitution violations - no tracking required.

## Key Design Decisions

### 1. Tool Architecture

The retrieval functionality is exposed as a `@function_tool` decorated function that wraps the existing `RetrievalClient`. This provides:
- Automatic schema generation from type annotations
- Clean separation between agent orchestration and retrieval logic
- Reuse of proven Qdrant/Cohere integration

### 2. Grounding Strategy

Agent instructions explicitly constrain responses to retrieved content only:
- System prompt includes strict grounding rules
- Score threshold (< 0.5) triggers "not found" response
- Tool output format includes clear source markers

### 3. Conversation Management

In-memory conversation history using `to_input_list()`:
- No persistence needed for CLI use case
- Maintains context across multiple turns
- Clean exit on "quit" command

## Integration Points

| Component | Interface | Notes |
|-----------|-----------|-------|
| RetrievalClient | backend/retrieve.py:56 | Existing class, used as-is |
| Query model | backend/models.py:8 | Existing dataclass |
| Qdrant | QDRANT_URL env var | Existing collection "rag-embedding" |
| Cohere | COHERE_API_KEY env var | embed-english-v3.0 model |
| OpenAI | OPENAI_API_KEY env var | New dependency for agent |

## Next Steps

1. Run `/sp.tasks` to generate implementation tasks
2. Implement tasks following TDD approach
3. Verify all success criteria from spec

## Sources

- [OpenAI Agents SDK Documentation](https://openai.github.io/openai-agents-python/)
- [OpenAI Agents SDK Tools](https://openai.github.io/openai-agents-python/tools/)
- [OpenAI Agents SDK Running Agents](https://openai.github.io/openai-agents-python/running_agents/)
