# Quickstart: RAG Retrieval Validation

**Feature**: 001-rag-retrieval-validation
**Date**: 2025-12-25

## Prerequisites

1. Embedding pipeline completed (Spec 1) - collection `rag-embedding` populated
2. Environment variables configured in `backend/.env`:
   - `QDRANT_URL`
   - `QDRANT_API_KEY`
   - `COHERE_API_KEY`

## Installation

```bash
cd backend
uv sync  # or pip install -r requirements.txt
```

## CLI Usage

### Basic Query

```bash
# Human-readable output
uv run retrieve.py "What is ROS 2?"

# JSON output
uv run retrieve.py --json "What is ROS 2?"
```

### Options

| Flag | Description | Default |
|------|-------------|---------|
| `--top-k N` | Return top N results | 5 |
| `--threshold T` | Minimum similarity score (0-1) | 0.0 |
| `--json` | Output in JSON format | False |
| `--verbose` | Enable debug logging | False |

### Examples

```bash
# Get top 3 results
uv run retrieve.py --top-k 3 "How do I create a URDF file?"

# Filter low-relevance results
uv run retrieve.py --threshold 0.7 "Gazebo simulation"

# JSON output for automation
uv run retrieve.py --json --top-k 10 "Isaac Sim" > results.json
```

## Expected Output

### Human-Readable Format

```
Query: What is ROS 2?
Results: 5 found in 234ms

[1] Score: 0.89
    Title: Chapter 1: ROS 2 Fundamentals
    URL: https://ai-native-textbook.../docs/module-1-ros2/chapter-1-fundamentals
    Text: ROS 2 (Robot Operating System 2) is a set of software libraries
          and tools for building robot applications...

[2] Score: 0.82
    Title: Module Overview
    ...
```

### JSON Format

```json
{
  "query": "What is ROS 2?",
  "total_results": 5,
  "execution_time_ms": 234.5,
  "results": [
    {
      "text": "ROS 2 (Robot Operating System 2) is a set of...",
      "url": "https://ai-native-textbook.../docs/module-1-ros2/chapter-1-fundamentals",
      "title": "Chapter 1: ROS 2 Fundamentals",
      "chunk_index": 0,
      "score": 0.89
    }
  ],
  "error": null
}
```

## Validation Tests

Run the validation test suite:

```bash
# From project root
pytest tests/test_retrieve.py -v

# Run specific test categories
pytest tests/test_retrieve.py -k "test_basic_query" -v
pytest tests/test_retrieve.py -k "test_metadata" -v
pytest tests/test_retrieve.py -k "test_edge_cases" -v
```

## Troubleshooting

| Issue | Solution |
|-------|----------|
| "Collection not found" | Run embedding pipeline first (`uv run main.py`) |
| "Connection timeout" | Check QDRANT_URL and network connectivity |
| "Invalid API key" | Verify COHERE_API_KEY and QDRANT_API_KEY in .env |
| "No results" | Try lower threshold or broader query |

## Exit Codes

| Code | Meaning |
|------|---------|
| 0 | Success - results returned |
| 1 | Error - connection/API failure |
| 2 | No results found |
