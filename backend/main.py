"""
Embedding Pipeline for RAG Retrieval

This script crawls a deployed Docusaurus documentation site, extracts text content,
generates embeddings using Cohere, and stores them in Qdrant for RAG-based retrieval.

TARGET_URL = "https://ai-native-textbook-for-physical-ai.vercel.app/sitemap.xml"
Collection: rag-embedding
"""

import os
import sys
import uuid
import logging
from pathlib import Path
from urllib.parse import urljoin, urlparse
from typing import Optional

import requests
from bs4 import BeautifulSoup
from dotenv import load_dotenv
import cohere
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams, PointStruct

# Load environment variables early
load_dotenv()

# Configuration from .env
BASE_URL = os.getenv("DEPLOYED_VERCEL_URL", "https://ai-native-textbook-for-physical-ai.vercel.app")
TARGET_URL = os.getenv("TARGET_URL", f"{BASE_URL}/sitemap.xml")
LOCAL_DOCS_PATH = os.getenv("LOCAL_DOCS_PATH", "../frontend_book/docs")
COLLECTION_NAME = "rag-embedding"
EMBEDDING_MODEL = "embed-english-v3.0"
EMBEDDING_DIMENSION = 1024
CHUNK_SIZE = 500
CHUNK_OVERLAP = 100
BATCH_SIZE = 96
MAX_URLS = 100  # Limit URLs to crawl

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S"
)
logger = logging.getLogger(__name__)

# Get all URLs from the Docusaurus site

def get_urls_from_sitemap(sitemap_url: str, base_url: str, max_urls: int = MAX_URLS) -> list[str]:
    """
    Fetch URLs from sitemap.xml and fix domain mismatches.

    Args:
        sitemap_url: URL of the sitemap.xml
        base_url: The actual base URL to use (fixes domain mismatches)
        max_urls: Maximum number of URLs to collect

    Returns:
        List of URLs from the sitemap with corrected domains
    """
    logger.info(f"Fetching sitemap from {sitemap_url}")

    try:
        response = requests.get(sitemap_url, timeout=10)
        response.raise_for_status()

        soup = BeautifulSoup(response.text, "xml")
        loc_tags = soup.find_all("loc")

        urls = []
        base_parsed = urlparse(base_url)

        for loc in loc_tags:
            if len(urls) >= max_urls:
                break

            original_url = loc.get_text(strip=True)
            parsed = urlparse(original_url)

            # Replace domain with the actual base URL domain
            fixed_url = f"{base_parsed.scheme}://{base_parsed.netloc}{parsed.path}"
            urls.append(fixed_url)

        logger.info(f"Found {len(urls)} URLs in sitemap")
        return urls

    except requests.RequestException as e:
        logger.warning(f"Failed to fetch sitemap: {e}")
        return []


def get_all_urls(base_url: str, max_urls: int = MAX_URLS) -> list[str]:
    """
    Get URLs from sitemap first, then fall back to crawling.

    Args:
        base_url: The root URL of the Docusaurus site
        max_urls: Maximum number of URLs to collect

    Returns:
        List of unique URLs found on the site
    """
    # Try sitemap first
    sitemap_url = f"{base_url.rstrip('/')}/sitemap.xml"
    urls = get_urls_from_sitemap(sitemap_url, base_url, max_urls)

    if urls:
        return urls

    # Fall back to crawling
    logger.info(f"Falling back to crawling from {base_url}")
    urls = set()
    visited = set()
    to_visit = [base_url]

    logger.info(f"Starting URL crawl from {base_url} (max: {max_urls} URLs)")

    while to_visit and len(urls) < max_urls:
        current_url = to_visit.pop(0)

        if current_url in visited:
            continue

        visited.add(current_url)

        try:
            response = requests.get(current_url, timeout=10)
            response.raise_for_status()

            soup = BeautifulSoup(response.text, "html.parser")

            # Find all links
            for link in soup.find_all("a", href=True):
                if len(urls) >= max_urls:
                    break

                href = link["href"]
                full_url = urljoin(current_url, href)

                # Parse the URL to check domain
                parsed = urlparse(full_url)
                base_parsed = urlparse(base_url)

                # Only include URLs from the same domain
                if parsed.netloc == base_parsed.netloc:
                    # Remove fragment
                    clean_url = f"{parsed.scheme}://{parsed.netloc}{parsed.path}"

                    # Include docs pages and main content
                    if clean_url not in visited and clean_url not in to_visit:
                        to_visit.append(clean_url)
                        urls.add(clean_url)

        except requests.RequestException as e:
            logger.warning(f"Failed to fetch {current_url}: {e}")
            continue

    url_list = list(urls)[:max_urls]
    logger.info(f"Discovered {len(url_list)} unique URLs")
    return url_list

#Extract text from URLs

def extract_text_from_urls(urls: list[str]) -> list[tuple[str, str, str]]:
    """
    Fetch HTML pages and extract clean text content.

    Args:
        urls: List of URLs to process

    Returns:
        List of tuples: (url, title, text)
    """
    documents = []

    logger.info(f"Extracting text from {len(urls)} URLs")

    for i, url in enumerate(urls):
        try:
            response = requests.get(url, timeout=10)
            response.raise_for_status()

            soup = BeautifulSoup(response.text, "html.parser")

            # Remove unwanted elements
            for element in soup.find_all(["nav", "footer", "sidebar", "script", "style", "header"]):
                element.decompose()

            # Try to find main content
            main_content = (
                soup.find("article") or
                soup.find("main") or
                soup.find("div", class_="content") or
                soup.find("div", class_="markdown") or
                soup.body
            )

            if main_content:
                # Extract title
                title_tag = soup.find("title")
                h1_tag = soup.find("h1")
                title = ""
                if title_tag:
                    title = title_tag.get_text(strip=True)
                elif h1_tag:
                    title = h1_tag.get_text(strip=True)
                else:
                    title = url.split("/")[-1] or "Untitled"

                # Extract text
                text = main_content.get_text(separator=" ", strip=True)

                # Clean up whitespace
                text = " ".join(text.split())

                if text and len(text) > 50:  # Skip very short content
                    documents.append((url, title, text))
                    logger.info(f"[{i+1}/{len(urls)}] Extracted: {title[:50]}...")
                else:
                    logger.warning(f"[{i+1}/{len(urls)}] Skipped (too short): {url}")
            else:
                logger.warning(f"[{i+1}/{len(urls)}] No main content found: {url}")

        except requests.RequestException as e:
            logger.warning(f"[{i+1}/{len(urls)}] Failed to fetch {url}: {e}")
            continue

    logger.info(f"Successfully extracted {len(documents)} documents")
    return documents


def extract_text_from_local_files(docs_path: str, base_url: str) -> list[tuple[str, str, str]]:
    """
    Read local markdown files and extract text content.

    Args:
        docs_path: Path to the local docs directory
        base_url: Base URL to construct URLs for the documents

    Returns:
        List of tuples: (url, title, text)
    """
    documents = []
    docs_dir = Path(docs_path)

    if not docs_dir.exists():
        logger.warning(f"Local docs path does not exist: {docs_path}")
        return documents

    # Find all markdown files
    md_files = list(docs_dir.rglob("*.md")) + list(docs_dir.rglob("*.mdx"))
    logger.info(f"Found {len(md_files)} local markdown files")

    for md_file in md_files:
        try:
            content = md_file.read_text(encoding="utf-8")

            # Extract title from frontmatter or first heading
            title = md_file.stem
            lines = content.split("\n")

            # Check for frontmatter title
            in_frontmatter = False
            for line in lines:
                if line.strip() == "---":
                    in_frontmatter = not in_frontmatter
                    continue
                if in_frontmatter and line.startswith("title:"):
                    title = line.replace("title:", "").strip().strip('"').strip("'")
                    break
                if not in_frontmatter and line.startswith("# "):
                    title = line.replace("# ", "").strip()
                    break

            # Remove frontmatter and clean text
            text_lines = []
            in_frontmatter = False
            frontmatter_count = 0
            for line in lines:
                if line.strip() == "---":
                    frontmatter_count += 1
                    if frontmatter_count <= 2:
                        continue
                if frontmatter_count < 2:
                    continue
                # Skip code blocks for cleaner text
                text_lines.append(line)

            text = " ".join(text_lines)
            text = " ".join(text.split())  # Clean whitespace

            # Construct URL from file path
            relative_path = md_file.relative_to(docs_dir)
            url_path = str(relative_path).replace("\\", "/").replace(".mdx", "").replace(".md", "")
            if url_path.endswith("/index"):
                url_path = url_path[:-6]
            url = f"{base_url.rstrip('/')}/docs/{url_path}"

            if text and len(text) > 50:
                documents.append((url, title, text))
                logger.info(f"Extracted local: {title[:50]}...")

        except Exception as e:
            logger.warning(f"Failed to read {md_file}: {e}")
            continue

    logger.info(f"Successfully extracted {len(documents)} documents from local files")
    return documents


# Chunk text for embedding (text got from above function)

def chunk_text(text: str, chunk_size: int = CHUNK_SIZE, overlap: int = CHUNK_OVERLAP) -> list[str]:
    """
    Split text into overlapping chunks for embedding.

    Args:
        text: The full text to chunk
        chunk_size: Target size of each chunk in characters
        overlap: Number of overlapping characters between chunks

    Returns:
        List of text chunks
    """
    if not text or not text.strip():
        return []

    text = text.strip()

    if len(text) <= chunk_size:
        return [text]

    chunks = []
    start = 0

    while start < len(text):
        end = start + chunk_size

        # Try to break at sentence boundary
        if end < len(text):
            # Look for sentence ending punctuation
            for punct in [". ", "! ", "? ", "\n"]:
                last_punct = text[start:end].rfind(punct)
                if last_punct > chunk_size // 2:
                    end = start + last_punct + 1
                    break

        chunk = text[start:end].strip()
        if chunk:
            chunks.append(chunk)

        # Move start with overlap
        start = end - overlap if end < len(text) else len(text)

    return chunks

#embed text chunks using Cohere

def embed(chunks: list[str], cohere_client: cohere.Client) -> list[list[float]]:
    """
    Generate Cohere embeddings for text chunks.

    Args:
        chunks: List of text strings to embed
        cohere_client: Initialized Cohere client

    Returns:
        List of embedding vectors (1024 floats each)
    """
    if not chunks:
        return []

    all_embeddings = []
    total_batches = (len(chunks) + BATCH_SIZE - 1) // BATCH_SIZE

    logger.info(f"Generating embeddings for {len(chunks)} chunks in {total_batches} batches")

    for batch_idx in range(0, len(chunks), BATCH_SIZE):
        batch = chunks[batch_idx:batch_idx + BATCH_SIZE]
        batch_num = batch_idx // BATCH_SIZE + 1

        try:
            response = cohere_client.embed(
                texts=batch,
                model=EMBEDDING_MODEL,
                input_type="search_document"
            )

            all_embeddings.extend(response.embeddings)
            logger.info(f"Batch {batch_num}/{total_batches}: Generated {len(batch)} embeddings")

        except Exception as e:
            logger.error(f"Batch {batch_num}/{total_batches}: Cohere API error: {e}")
            raise

    return all_embeddings

# Creating Qdrant collection and saving embeddings

def create_collection(client: QdrantClient) -> None:
    """
    Initialize the Qdrant collection for storing embeddings.

    Args:
        client: Initialized QdrantClient instance
    """
    logger.info(f"Creating Qdrant collection '{COLLECTION_NAME}'")

    try:
        # Delete existing collection if present
        collections = client.get_collections().collections
        if any(c.name == COLLECTION_NAME for c in collections):
            logger.info(f"Deleting existing collection '{COLLECTION_NAME}'")
            client.delete_collection(COLLECTION_NAME)

        # Create new collection
        client.create_collection(
            collection_name=COLLECTION_NAME,
            vectors_config=VectorParams(
                size=EMBEDDING_DIMENSION,
                distance=Distance.COSINE
            )
        )

        logger.info(f"Collection '{COLLECTION_NAME}' created successfully")

    except Exception as e:
        logger.error(f"Failed to create collection: {e}")
        raise

# Save chunks and embeddings to Qdrant

def save_chunk_to_qdrant(
    client: QdrantClient,
    chunks: list[dict],
    embeddings: list[list[float]],
    batch_size: int = 100
) -> None:
    """
    Upsert embeddings with metadata to Qdrant in batches.

    Args:
        client: Initialized QdrantClient instance
        chunks: List of chunk metadata dicts
        embeddings: List of embedding vectors
        batch_size: Number of points per batch (default 100)
    """
    if not chunks or not embeddings:
        logger.warning("No chunks or embeddings to save")
        return

    if len(chunks) != len(embeddings):
        raise ValueError(f"Chunks ({len(chunks)}) and embeddings ({len(embeddings)}) count mismatch")

    logger.info(f"Upserting {len(chunks)} points to Qdrant in batches of {batch_size}")

    try:
        points = [
            PointStruct(
                id=str(uuid.uuid4()),
                vector=embedding,
                payload={
                    "text": chunk["text"],
                    "url": chunk["url"],
                    "title": chunk["title"],
                    "chunk_index": chunk["chunk_index"]
                }
            )
            for chunk, embedding in zip(chunks, embeddings)
        ]

        # Upsert in batches to avoid timeout
        total_batches = (len(points) + batch_size - 1) // batch_size
        for i in range(0, len(points), batch_size):
            batch = points[i:i + batch_size]
            batch_num = i // batch_size + 1
            client.upsert(
                collection_name=COLLECTION_NAME,
                points=batch
            )
            logger.info(f"Batch {batch_num}/{total_batches}: Upserted {len(batch)} points")

        logger.info(f"Successfully upserted {len(points)} points to '{COLLECTION_NAME}'")

    except Exception as e:
        logger.error(f"Failed to upsert to Qdrant: {e}")
        raise


def main() -> None:
    """
    Orchestrate the complete embedding pipeline.
    """
    logger.info("=" * 60)
    logger.info("Starting Embedding Pipeline for RAG Retrieval")
    logger.info("=" * 60)

    # Validate configuration
    cohere_api_key = os.getenv("COHERE_API_KEY")
    qdrant_url = os.getenv("QDRANT_URL")
    qdrant_api_key = os.getenv("QDRANT_API_KEY")

    if not cohere_api_key:
        logger.error("COHERE_API_KEY environment variable is not set")
        sys.exit(1)

    logger.info(f"Target URL: {BASE_URL}")
    logger.info(f"Qdrant URL: {qdrant_url}")
    logger.info(f"Collection: {COLLECTION_NAME}")

    try:
        # Initialize clients
        logger.info("Initializing Cohere client...")
        cohere_client = cohere.Client(api_key=cohere_api_key)

        logger.info("Initializing Qdrant client...")
        qdrant_client = QdrantClient(
            url=qdrant_url,
            api_key=qdrant_api_key if qdrant_api_key else None
        )

        # Step 1: Crawl URLs
        logger.info("-" * 40)
        logger.info("Step 1: Crawling URLs")
        urls = get_all_urls(BASE_URL)

        if not urls:
            logger.error("No URLs found to process")
            sys.exit(2)

        # Step 2: Extract text
        logger.info("-" * 40)
        logger.info("Step 2: Extracting text from URLs")
        documents = extract_text_from_urls(urls)

        # Fallback to local files if web extraction fails
        if len(documents) < 3:
            logger.warning(f"Only {len(documents)} documents from web. Trying local files...")
            logger.info("-" * 40)
            logger.info("Step 2b: Extracting text from local markdown files")
            local_docs = extract_text_from_local_files(LOCAL_DOCS_PATH, BASE_URL)
            if local_docs:
                documents = local_docs

        if not documents:
            logger.error("No documents extracted from web or local files")
            sys.exit(2)

        # Step 3: Chunk text
        logger.info("-" * 40)
        logger.info("Step 3: Chunking text")
        all_chunks = []
        for url, title, text in documents:
            text_chunks = chunk_text(text)
            for idx, chunk in enumerate(text_chunks):
                all_chunks.append({
                    "text": chunk,
                    "url": url,
                    "title": title,
                    "chunk_index": idx
                })

        logger.info(f"Created {len(all_chunks)} chunks from {len(documents)} documents")

        if not all_chunks:
            logger.error("No chunks created")
            sys.exit(2)

        # Step 4: Generate embeddings
        logger.info("-" * 40)
        logger.info("Step 4: Generating embeddings")
        chunk_texts = [c["text"] for c in all_chunks]
        embeddings = embed(chunk_texts, cohere_client)

        # Step 5: Create Qdrant collection
        logger.info("-" * 40)
        logger.info("Step 5: Creating Qdrant collection")
        create_collection(qdrant_client)

        # Step 6: Save to Qdrant
        logger.info("-" * 40)
        logger.info("Step 6: Saving to Qdrant")
        save_chunk_to_qdrant(qdrant_client, all_chunks, embeddings)

        # Summary
        logger.info("=" * 60)
        logger.info("Pipeline completed successfully!")
        logger.info(f"  URLs processed: {len(urls)}")
        logger.info(f"  Documents extracted: {len(documents)}")
        logger.info(f"  Chunks created: {len(all_chunks)}")
        logger.info(f"  Embeddings generated: {len(embeddings)}")
        logger.info(f"  Points stored in '{COLLECTION_NAME}': {len(all_chunks)}")
        logger.info("=" * 60)

        sys.exit(0)

    except requests.RequestException as e:
        logger.error(f"Network error: {e}")
        sys.exit(2)
    except Exception as e:
        logger.error(f"Pipeline error: {e}")
        sys.exit(2)


if __name__ == "__main__":
    main()
