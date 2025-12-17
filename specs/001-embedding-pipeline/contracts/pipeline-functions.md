# Pipeline Function Contracts

**Feature**: 001-embedding-pipeline
**Date**: 2025-12-17

## Function Signatures

### get_all_urls

```python
def get_all_urls(base_url: str) -> list[str]:
    """
    Crawl the Docusaurus site and discover all documentation page URLs.

    Args:
        base_url: The root URL of the Docusaurus site
                  e.g., "https://ai-native-textbook-for-physical-ai-phi.vercel.app/"

    Returns:
        List of unique URLs found on the site
        e.g., ["https://example.com/docs/intro", "https://example.com/docs/module1"]

    Behavior:
        - Fetches the sitemap.xml if available
        - Falls back to crawling links from the homepage
        - Filters to only include /docs/ paths
        - Removes duplicates
        - Handles HTTP errors gracefully
    """
```

### extract_text_from_urls

```python
def extract_text_from_urls(urls: list[str]) -> list[tuple[str, str, str]]:
    """
    Fetch HTML pages and extract clean text content.

    Args:
        urls: List of URLs to process

    Returns:
        List of tuples: (url, title, text)
        - url: The source URL
        - title: Page title from <title> or <h1>
        - text: Clean text with HTML tags removed

    Behavior:
        - Uses requests to fetch each URL
        - Parses HTML with BeautifulSoup
        - Extracts main content (article, main, or div.content)
        - Removes nav, footer, sidebar, script, style elements
        - Logs progress for each URL
        - Skips URLs that fail to fetch
    """
```

### chunk_text

```python
def chunk_text(
    text: str,
    chunk_size: int = 500,
    overlap: int = 100
) -> list[str]:
    """
    Split text into overlapping chunks for embedding.

    Args:
        text: The full text to chunk
        chunk_size: Target size of each chunk in characters (default: 500)
        overlap: Number of overlapping characters between chunks (default: 100)

    Returns:
        List of text chunks

    Behavior:
        - Splits text at sentence boundaries when possible
        - Ensures overlap between consecutive chunks
        - Returns empty list for empty/whitespace-only text
        - Handles edge case where text < chunk_size
    """
```

### embed

```python
def embed(chunks: list[str]) -> list[list[float]]:
    """
    Generate Cohere embeddings for text chunks.

    Args:
        chunks: List of text strings to embed

    Returns:
        List of embedding vectors (1024 floats each)

    Behavior:
        - Uses Cohere embed-english-v3.0 model
        - Sets input_type="search_document"
        - Batches requests (max 96 texts per API call)
        - Logs progress for each batch
        - Raises exception on API errors

    Environment:
        Requires COHERE_API_KEY environment variable
    """
```

### create_collection

```python
def create_collection(client: QdrantClient) -> None:
    """
    Initialize the Qdrant collection for storing embeddings.

    Args:
        client: Initialized QdrantClient instance

    Returns:
        None

    Behavior:
        - Creates collection named "rag-embedding"
        - Vector size: 1024 (matching Cohere embed-v3.0)
        - Distance metric: Cosine similarity
        - Recreates collection if it already exists
        - Logs success/failure
    """
```

### save_chunk_to_qdrant

```python
def save_chunk_to_qdrant(
    client: QdrantClient,
    chunks: list[dict],
    embeddings: list[list[float]]
) -> None:
    """
    Upsert embeddings with metadata to Qdrant.

    Args:
        client: Initialized QdrantClient instance
        chunks: List of chunk metadata dicts containing:
                - text: The chunk text
                - url: Source URL
                - title: Source title
                - chunk_index: Position in document
        embeddings: List of embedding vectors (aligned with chunks)

    Returns:
        None

    Behavior:
        - Creates PointStruct for each chunk/embedding pair
        - Generates UUID for each point
        - Upserts all points in a single batch
        - Logs number of points upserted
    """
```

### main

```python
def main() -> None:
    """
    Orchestrate the complete embedding pipeline.

    Behavior:
        1. Load environment variables from .env
        2. Initialize Cohere and Qdrant clients
        3. Call get_all_urls() with base URL
        4. Call extract_text_from_urls() with discovered URLs
        5. Chunk all extracted text
        6. Generate embeddings for all chunks
        7. Create Qdrant collection
        8. Save all embeddings to Qdrant
        9. Log completion summary

    Environment:
        - COHERE_API_KEY: Cohere API key
        - QDRANT_URL: Qdrant instance URL
        - QDRANT_API_KEY: Qdrant API key (optional for local)

    Exit Codes:
        - 0: Success
        - 1: Configuration error (missing env vars)
        - 2: Network/API error
    """
```

## Error Handling

| Function | Error Type | Handling |
|----------|------------|----------|
| get_all_urls | RequestException | Log warning, return empty list |
| extract_text_from_urls | RequestException | Log warning, skip URL |
| embed | CohereError | Raise with message |
| create_collection | QdrantException | Log error, raise |
| save_chunk_to_qdrant | QdrantException | Log error, raise |
| main | Any | Log error, exit with code |
