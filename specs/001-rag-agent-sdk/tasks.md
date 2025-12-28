# Tasks: RAG Agent with OpenAI Agents SDK

**Input**: Design documents from `/specs/001-rag-agent-sdk/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, quickstart.md

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: US1, US2, US3, US4

---

## Phase 1: Setup

**Purpose**: Add openai-agents dependency

- [x] T001 Add openai-agents dependency in backend/pyproject.toml

---

## Phase 2: Foundational

**Purpose**: Create retrieval tool wrapper

- [x] T002 Create search_book_content function with @function_tool in backend/tools.py
- [x] T003 Create format_results_for_agent helper in backend/tools.py
- [x] T004 Define AGENT_INSTRUCTIONS constant with grounding rules in backend/agent.py

**Checkpoint**: Tool and agent config ready

---

## Phase 3: User Story 1 - Query Book Content (P1) MVP

**Goal**: Ask questions and get grounded answers with source attribution

**Independent Test**: `uv run agent.py --query "What is ROS 2?"`

- [x] T005 [US1] Create Agent with instructions and retrieval tool in backend/agent.py
- [x] T006 [US1] Implement single-query mode (--query flag) in backend/agent.py
- [x] T007 [US1] Add source attribution formatting in agent response in backend/agent.py
- [x] T008 [US1] Add logging for retrieval and generation in backend/agent.py

**Checkpoint**: Single query works with grounded response

---

## Phase 4: User Story 2 - Out-of-Scope Handling (P1)

**Goal**: Explicitly indicate when topic not covered

**Independent Test**: `uv run agent.py --query "What is quantum computing?"`

- [x] T009 [US2] Add NO_RELEVANT_CONTENT detection in backend/tools.py
- [x] T010 [US2] Update instructions for "not found" response in backend/agent.py
- [x] T011 [US2] Add topic suggestions when out of scope in backend/agent.py

**Checkpoint**: Out-of-scope queries handled gracefully

---

## Phase 5: User Story 3 - Tool Integration (P2)

**Goal**: Agent invokes retrieval as tool in workflow

**Independent Test**: Run with --verbose and verify tool call logs

- [x] T012 [US3] Add verbose mode (--verbose flag) for tool tracing in backend/agent.py
- [x] T013 [US3] Log tool invocation details in backend/tools.py

**Checkpoint**: Tool calls visible in logs

---

## Phase 6: User Story 4 - Conversational Context (P3)

**Goal**: Multi-turn conversation with context

**Independent Test**: Interactive mode with follow-up questions

- [x] T014 [US4] Implement interactive REPL loop in backend/agent.py
- [x] T015 [US4] Add conversation history with to_input_list() in backend/agent.py
- [x] T016 [US4] Handle quit/exit commands in backend/agent.py

**Checkpoint**: Multi-turn conversation works

---

## Phase 7: Edge Cases & Polish

**Purpose**: Error handling and cleanup

- [x] T017 [P] Add connection error handling in backend/agent.py
- [x] T018 [P] Add empty query validation in backend/agent.py
- [x] T019 Add --json output format in backend/agent.py
- [ ] T020 Run quickstart.md validation scenarios (BLOCKED: requires valid OPENAI_API_KEY)

---

## Dependencies

- **Phase 1**: No dependencies
- **Phase 2**: Depends on Phase 1
- **US1 (Phase 3)**: Depends on Phase 2
- **US2 (Phase 4)**: Depends on US1
- **US3 (Phase 5)**: Depends on US1
- **US4 (Phase 6)**: Depends on US1
- **Phase 7**: Depends on all user stories

## Parallel Opportunities

```bash
# Phase 2 - all parallel:
T002, T003, T004

# Phase 7 - partial parallel:
T017, T018
```

## MVP Scope

Complete Phase 1-4 (Setup + Foundational + US1 + US2) for minimum viable agent.

---

## Summary

| Phase | Tasks | User Story |
|-------|-------|------------|
| Setup | 1 | - |
| Foundational | 3 | - |
| US1 Query | 4 | P1 |
| US2 Out-of-Scope | 3 | P1 |
| US3 Tool Tracing | 2 | P2 |
| US4 Conversation | 3 | P3 |
| Polish | 4 | - |
| **Total** | **20** | |
