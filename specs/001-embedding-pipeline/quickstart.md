# Quickstart: Embedding Pipeline

**Feature**: 001-embedding-pipeline
**Date**: 2025-12-17

## Prerequisites

- Python 3.11 or higher
- UV package manager installed (`pip install uv` or `curl -LsSf https://astral.sh/uv/install.sh | sh`)
- Cohere API key (sign up at https://cohere.com)
- Qdrant instance (local or cloud)

## Setup

### 1. Create Backend Folder

```bash
mkdir backend
cd backend
```

### 2. Initialize Project with UV

```bash
uv init
uv add cohere qdrant-client requests beautifulsoup4 python-dotenv
```

### 3. Configure Environment Variables

Create `.env` file:

```bash
COHERE_API_KEY=your-cohere-api-key
QDRANT_URL=http://localhost:6333
QDRANT_API_KEY=  # Leave empty for local, set for cloud
```

### 4. Start Qdrant (Local)

Using Docker:

```bash
docker run -p 6333:6333 qdrant/qdrant
```

Or use Qdrant Cloud (https://cloud.qdrant.io).

## Usage

### Run the Pipeline

```bash
cd backend
uv run python main.py
```

### Expected Output

```
[INFO] Starting embedding pipeline...
[INFO] Crawling URLs from https://ai-native-textbook-for-physical-ai-phi.vercel.app/
[INFO] Found 25 pages
[INFO] Extracting text from 25 URLs...
[INFO] Generated 150 chunks
[INFO] Creating embeddings (batch 1/2)...
[INFO] Creating embeddings (batch 2/2)...
[INFO] Creating Qdrant collection 'rag-embedding'...
[INFO] Upserting 150 points to Qdrant...
[INFO] Pipeline complete!
```

## Verification

### Check Qdrant Collection

```bash
curl http://localhost:6333/collections/rag-embedding
```

### Test Similarity Search

```python
from qdrant_client import QdrantClient
import cohere

# Initialize clients
qdrant = QdrantClient(url="http://localhost:6333")
co = cohere.Client(api_key="your-key")

# Embed query
query = "How to install ROS 2?"
query_embedding = co.embed(
    texts=[query],
    model="embed-english-v3.0",
    input_type="search_query"
).embeddings[0]

# Search
results = qdrant.search(
    collection_name="rag-embedding",
    query_vector=query_embedding,
    limit=5
)

for result in results:
    print(f"Score: {result.score:.3f}")
    print(f"Title: {result.payload['title']}")
    print(f"Text: {result.payload['text'][:200]}...")
    print("---")
```

## Troubleshooting

| Issue | Solution |
|-------|----------|
| Cohere rate limit | Add delay between batches or reduce batch size |
| Qdrant connection refused | Ensure Qdrant is running on specified port |
| Empty text extraction | Check if target URL is accessible and has content |
| SSL errors | Set `verify=False` in requests (development only) |
