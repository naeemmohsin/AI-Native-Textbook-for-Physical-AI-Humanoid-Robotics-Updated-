# Quickstart: RAG Agent with OpenAI Agents SDK

**Feature**: 001-rag-agent-sdk
**Date**: 2025-12-26

## Prerequisites

1. **Existing Setup from Specs 1-2**:
   - Qdrant collection "rag-embedding" populated with book content
   - Cohere API key configured
   - Qdrant credentials configured

2. **New Requirements**:
   - OpenAI API key for the Agents SDK

## Installation

```bash
cd backend
uv add openai-agents
```

## Environment Variables

Add to `.env`:
```bash
# Existing (from Specs 1-2)
COHERE_API_KEY=your_cohere_key
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your_qdrant_key

# New for Spec 3
OPENAI_API_KEY=your_openai_key
```

## Usage

### Start the Agent CLI

```bash
cd backend
uv run agent.py
```

### Example Session

```
RAG Agent ready. Type 'quit' to exit.

You: What is ROS 2?

Agent: Based on the book content, ROS 2 (Robot Operating System 2) is the second
generation of the Robot Operating System, designed for production robotics
applications. Key features include:

- Real-time support for deterministic control loops
- Multi-platform compatibility (Linux, Windows, macOS)
- DDS-based communication for reliable messaging
- Improved security features

Sources:
- Module 1 - ROS 2 Fundamentals: https://example.com/module-1

You: What about quantum computing?

Agent: I couldn't find information about quantum computing in the book. The book
covers robotics topics including ROS 2, Gazebo simulation, URDF, Digital Twins,
NVIDIA Isaac Sim, and Vision-Language-Action models. Would you like to ask about
any of these topics instead?

You: quit
Goodbye!
```

## CLI Options

```bash
# Run with verbose logging
uv run agent.py --verbose

# Run single query (non-interactive)
uv run agent.py --query "What is Gazebo?"

# Output in JSON format
uv run agent.py --query "URDF tutorial" --json
```

## Verification Tests

### Test 1: Basic Query (US1)
```bash
uv run agent.py --query "What is ROS 2?"
```
Expected: Response contains ROS 2 information with source attribution.

### Test 2: Out-of-Scope Query (US2)
```bash
uv run agent.py --query "What is quantum computing?"
```
Expected: Response indicates topic not covered in the book.

### Test 3: Multi-turn Conversation (US4)
```
You: Tell me about Gazebo
Agent: [Response about Gazebo simulation...]

You: How do I install it?
Agent: [Response about Gazebo installation, understanding "it" refers to Gazebo...]
```
Expected: Agent maintains context across turns.

## Troubleshooting

| Issue | Solution |
|-------|----------|
| "OPENAI_API_KEY not set" | Add OPENAI_API_KEY to .env file |
| "Collection not found" | Run embedding pipeline from Spec 1 first |
| Slow responses | Check network connectivity to OpenAI API |
| Low relevance results | Adjust score_threshold in retrieval |
