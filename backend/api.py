import os
import sys

sys.path.append(os.path.dirname(os.path.abspath(__file__)))

import argparse

# FastAPI imports for the RAG agent service
import asyncio
import logging
import re
import time
import time as time_module
from typing import Dict, List, Tuple
from urllib.parse import urljoin, urlparse

import cohere
import requests
from bs4 import BeautifulSoup
from dotenv import load_dotenv
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from qdrant_client import QdrantClient
from qdrant_client.http import models

# Import agent modules
from agent.rag_agent import RAGAgent, get_rag_agent
from agent.system_prompt import RAG_AGENT_SYSTEM_PROMPT
from api.models import ErrorResponse, QueryRequest, QueryResponse, RetrievedDocument
from api.router import router
from config.settings import settings
from services.context_injector import ContextInjector
from services.qdrant_service import QdrantService
from validation.error_handling import validate_api_keys
from validation.metadata_validator import validate_metadata_integrity
from validation.models import QueryConfiguration, ValidationTest, ValidationType
from validation.pipeline_validator import validate_end_to_end_pipeline
from validation.qdrant_utils import get_qdrant_client, search_in_qdrant

# Import validation modules
from validation.query_processor import generate_query_embedding
from validation.report_generator import generate_validation_report
from validation.retrieval import validate_semantic_search

# Load environment variables
load_dotenv()


def is_valid_url(url: str) -> bool:
    """
    Validate if a string is a properly formatted URL

    Args:
        url: String to validate as URL

    Returns:
        Boolean indicating if URL is valid
    """
    try:
        result = urlparse(url)
        return all([result.scheme, result.netloc])
    except Exception:
        return False


def normalize_url(url: str) -> str:
    """
    Normalize URL by ensuring proper formatting

    Args:
        url: URL string to normalize

    Returns:
        Normalized URL string
    """
    # Ensure URL has a scheme
    if not url.startswith(("http://", "https://")):
        url = "https://" + url

    # Remove trailing slashes
    url = url.rstrip("/")

    return url


def sanitize_content(content: str) -> str:
    """
    Sanitize content by removing potentially harmful elements

    Args:
        content: Raw content string to sanitize

    Returns:
        Sanitized content string
    """
    if not content:
        return content

    # Remove any script-like content that might have been missed
    content = re.sub(
        r"<script[^>]*>.*?</script>", "", content, flags=re.DOTALL | re.IGNORECASE
    )
    content = re.sub(
        r"<style[^>]*>.*?</style>", "", content, flags=re.DOTALL | re.IGNORECASE
    )

    # Remove potential JavaScript event handlers
    content = re.sub(r'on\w+\s*=\s*["\'][^"\']*["\']', "", content, flags=re.IGNORECASE)

    # Remove any remaining HTML tags that might be harmful
    # This is a simplified approach - for production, consider using a proper HTML sanitizer
    content = re.sub(
        r"<(script|iframe|object|embed|form|input|button)[^>]*>.*?</\1>",
        "",
        content,
        flags=re.DOTALL | re.IGNORECASE,
    )
    content = re.sub(
        r"<(script|iframe|object|embed|form|input|button)[^>]*/?>",
        "",
        content,
        flags=re.IGNORECASE,
    )

    # Clean up excessive whitespace
    content = " ".join(content.split())

    return content


# Configure logging
logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger(__name__)


def get_sitemap_urls(base_url: str, max_retries: int = 3) -> set:
    """
    Extract URLs from the sitemap.xml file if available

    Args:
        base_url: The base URL of the Docusaurus site
        max_retries: Maximum number of retries for failed requests

    Returns:
        Set of URLs found in the sitemap
    """
    sitemap_url = f"{base_url.rstrip('/')}/sitemap.xml"
    sitemap_urls = set()

    retry_count = 0
    success = False

    while retry_count < max_retries and not success:
        try:
            response = requests.get(sitemap_url, timeout=10)
            response.raise_for_status()

            # Parse the sitemap XML
            sitemap_soup = BeautifulSoup(response.content, "xml")

            # Find all <url><loc> elements in the sitemap
            for loc in sitemap_soup.find_all("loc"):
                url = loc.get_text().strip()
                if url and url.startswith(base_url):
                    sitemap_urls.add(url)

            success = True
            logger.info(f"Found {len(sitemap_urls)} URLs from sitemap.xml")

        except requests.Timeout as e:
            retry_count += 1
            logger.warning(
                f"Timeout fetching sitemap {sitemap_url} (attempt {retry_count}/{max_retries}): {str(e)}"
            )
            if retry_count < max_retries:
                time.sleep(2**retry_count)  # Exponential backoff
        except requests.RequestException as e:
            retry_count += 1
            logger.warning(
                f"Request error fetching sitemap {sitemap_url} (attempt {retry_count}/{max_retries}): {str(e)}"
            )
            if retry_count < max_retries:
                time.sleep(2**retry_count)  # Exponential backoff
        except Exception as e:
            logger.error(f"Error parsing sitemap {sitemap_url}: {str(e)}")
            break  # Don't retry for parsing errors

    return sitemap_urls


def get_all_urls(
    base_url: str, max_pages: int = 500, max_depth: int = 3, max_retries: int = 3
) -> List[str]:
    """
    Crawl the Docusaurus site and extract all accessible URLs

    Args:
        base_url: The base URL of the Docusaurus site
        max_pages: Maximum number of pages to crawl
        max_depth: Maximum depth for crawling
        max_retries: Maximum number of retries for failed requests

    Returns:
        List of URLs found on the site
    """
    visited_urls = set()
    urls_to_visit = [base_url]
    all_urls = set()

    # Add URLs from sitemap if available
    sitemap_urls = get_sitemap_urls(base_url, max_retries)
    all_urls.update(sitemap_urls)

    # Add the base URL to all URLs if not already in sitemap
    if base_url not in all_urls:
        all_urls.add(base_url)

    while urls_to_visit and len(all_urls) < max_pages:
        current_url = urls_to_visit.pop(0)

        if current_url in visited_urls:
            continue

        visited_urls.add(current_url)
        logger.info(f"Crawling: {current_url} (Progress: {len(all_urls)}/{max_pages})")

        # Retry mechanism for network requests
        retry_count = 0
        success = False

        while retry_count < max_retries and not success:
            try:
                # Add a small delay to be respectful to the server
                time.sleep(0.1)

                response = requests.get(current_url, timeout=10)
                response.raise_for_status()

                soup = BeautifulSoup(response.content, "html.parser")

                # Find all links on the page
                for link in soup.find_all("a", href=True):
                    href = link["href"]

                    # Convert relative URLs to absolute URLs
                    absolute_url = urljoin(current_url, href)

                    # Only add URLs from the same domain
                    if urlparse(absolute_url).netloc == urlparse(base_url).netloc:
                        # Filter for likely documentation pages
                        if (
                            absolute_url.startswith(base_url)
                            and not absolute_url.endswith(
                                (
                                    ".pdf",
                                    ".jpg",
                                    ".jpeg",
                                    ".png",
                                    ".gif",
                                    ".zip",
                                    ".exe",
                                )
                            )
                            and "#" not in absolute_url
                        ):  # Avoid anchor links
                            all_urls.add(absolute_url)

                            # Add to visit queue if within depth limit
                            if (
                                len(urlparse(absolute_url).path.split("/"))
                                - len(urlparse(base_url).path.split("/"))
                                <= max_depth
                            ):
                                if absolute_url not in visited_urls:
                                    urls_to_visit.append(absolute_url)

                success = True  # Mark as successful if no exception occurred

            except requests.Timeout as e:
                retry_count += 1
                logger.warning(
                    f"Timeout crawling {current_url} (attempt {retry_count}/{max_retries}): {str(e)}"
                )
                if retry_count < max_retries:
                    time.sleep(2**retry_count)  # Exponential backoff
            except requests.RequestException as e:
                retry_count += 1
                logger.warning(
                    f"Request error crawling {current_url} (attempt {retry_count}/{max_retries}): {str(e)}"
                )
                if retry_count < max_retries:
                    time.sleep(2**retry_count)  # Exponential backoff
            except Exception as e:
                logger.error(f"Unexpected error crawling {current_url}: {str(e)}")
                break  # Don't retry for unexpected errors

    return list(all_urls)


def extract_text_from_url(url: str, max_retries: int = 3) -> Tuple[str, str]:
    """
    Extract clean text content from a URL

    Args:
        url: The URL to extract content from
        max_retries: Maximum number of retries for failed requests

    Returns:
        Tuple of (title, content) extracted from the page
    """
    retry_count = 0
    success = False

    while retry_count < max_retries and not success:
        try:
            response = requests.get(url, timeout=10)
            response.raise_for_status()

            soup = BeautifulSoup(response.content, "html.parser")

            # Remove script and style elements
            for script in soup(["script", "style"]):
                script.decompose()

            # Extract title
            title_tag = soup.find("title")
            title = (
                title_tag.get_text().strip()
                if title_tag
                else urlparse(url).path.split("/")[-1] or "Untitled"
            )

            # Find main content - Docusaurus typically uses main or article tags
            main_content = (
                soup.find("main")
                or soup.find("article")
                or soup.find("div", class_="container")
                or soup
            )

            # Extract text content
            content = main_content.get_text(separator=" ", strip=True)

            # Clean up excessive whitespace
            content = " ".join(content.split())

            # Sanitize content to remove any potential harmful elements
            content = sanitize_content(content)

            success = True
            return title, content

        except requests.Timeout as e:
            retry_count += 1
            logger.warning(
                f"Timeout extracting content from {url} (attempt {retry_count}/{max_retries}): {str(e)}"
            )
            if retry_count < max_retries:
                time.sleep(2**retry_count)  # Exponential backoff
        except requests.RequestException as e:
            retry_count += 1
            logger.warning(
                f"Request error extracting content from {url} (attempt {retry_count}/{max_retries}): {str(e)}"
            )
            if retry_count < max_retries:
                time.sleep(2**retry_count)  # Exponential backoff
        except Exception as e:
            logger.error(f"Failed to extract content from {url}: {str(e)}")
            break  # Don't retry for unexpected errors

    return "", ""


def chunk_text(content: str, chunk_size: int = 1000, overlap: int = 100) -> List[Dict]:
    """
    Split content into chunks of specified size

    Args:
        content: The text content to chunk
        chunk_size: Size of each chunk in characters
        overlap: Overlap between chunks in characters

    Returns:
        List of chunk dictionaries with content and metadata
    """
    if not content:
        return []

    chunks = []
    start = 0

    while start < len(content):
        end = start + chunk_size

        # If we're near the end, just take the remaining content
        if end > len(content):
            end = len(content)

        chunk_text = content[start:end]

        chunk = {"content": chunk_text, "start_pos": start, "end_pos": end}

        chunks.append(chunk)

        # Move start position with overlap
        start = end - overlap

        # If the remaining content is less than overlap, break
        if len(content) - start <= overlap:
            break

    return chunks


def process_large_document_in_chunks(
    content: str, chunk_size: int = 1000, overlap: int = 100
):
    """
    Generator that yields document chunks one at a time to optimize memory usage

    Args:
        content: The text content to process
        chunk_size: Size of each chunk in characters
        overlap: Overlap between chunks in characters

    Yields:
        Dictionary containing chunk data
    """
    if not content:
        return

    start = 0

    while start < len(content):
        end = start + chunk_size

        # If we're near the end, just take the remaining content
        if end > len(content):
            end = len(content)

        chunk_text = content[start:end]

        chunk = {"content": chunk_text, "start_pos": start, "end_pos": end}

        yield chunk

        # Move start position with overlap
        start = end - overlap

        # If the remaining content is less than overlap, break
        if len(content) - start <= overlap:
            break


def validate_embeddings(
    embeddings: List[List[float]], expected_dimension: int = 768
) -> bool:
    """
    Validate that all embeddings have the expected dimension

    Args:
        embeddings: List of embedding vectors to validate
        expected_dimension: Expected dimension of each embedding vector

    Returns:
        Boolean indicating if all embeddings have the correct dimension
    """
    if not embeddings:
        return True

    for i, embedding in enumerate(embeddings):
        if len(embedding) != expected_dimension:
            logger.error(
                f"Embedding {i} has dimension {len(embedding)}, expected {expected_dimension}"
            )
            return False

    return True


def embed(
    texts: List[str], max_retries: int = 3, batch_size: int = 96
) -> List[List[float]]:
    """
    Generate embeddings for a list of texts using Cohere API

    Args:
        texts: List of text strings to embed
        max_retries: Maximum number of retries for failed requests
        batch_size: Maximum number of texts to process in a single API call

    Returns:
        List of embedding vectors
    """
    cohere_api_key = os.getenv("COHERE_API_KEY")
    if not cohere_api_key:
        raise ValueError("COHERE_API_KEY environment variable not set")

    all_embeddings = []
    co = cohere.Client(cohere_api_key)

    # Process texts in batches
    for i in range(0, len(texts), batch_size):
        batch = texts[i : i + batch_size]

        retry_count = 0
        success = False

        while retry_count < max_retries and not success:
            try:
                response = co.embed(
                    texts=batch,
                    model="embed-multilingual-v2.0",  # Using a multilingual model for broader compatibility
                )
                batch_embeddings = response.embeddings
                all_embeddings.extend(batch_embeddings)
                success = True
            except requests.Timeout as e:
                retry_count += 1
                logger.warning(
                    f"Timeout generating embeddings for batch {i // batch_size + 1} (attempt {retry_count}/{max_retries}): {str(e)}"
                )
                if retry_count < max_retries:
                    time.sleep(2**retry_count)  # Exponential backoff
            except requests.RequestException as e:
                retry_count += 1
                logger.warning(
                    f"Request error generating embeddings for batch {i // batch_size + 1} (attempt {retry_count}/{max_retries}): {str(e)}"
                )
                if retry_count < max_retries:
                    time.sleep(2**retry_count)  # Exponential backoff
            except Exception as e:
                retry_count += 1
                logger.warning(
                    f"Error generating embeddings for batch {i // batch_size + 1} (attempt {retry_count}/{max_retries}): {str(e)}"
                )
                if retry_count < max_retries:
                    time.sleep(2**retry_count)  # Exponential backoff

        if not success:
            raise Exception(
                f"Failed to generate embeddings for batch {i // batch_size + 1} after {max_retries} attempts"
            )

    # Validate embeddings dimensions
    if not validate_embeddings(all_embeddings):
        raise ValueError(f"Embeddings validation failed: incorrect dimensions found")

    logger.info(
        f"Successfully generated {len(all_embeddings)} embeddings with {len(all_embeddings[0]) if all_embeddings else 0} dimensions each"
    )
    return all_embeddings


def create_collection(client: QdrantClient, collection_name: str = "rag_embedding"):
    """
    Create a Qdrant collection for storing embeddings

    Args:
        client: Qdrant client instance
        collection_name: Name of the collection to create
    """
    try:
        # Check if collection already exists
        client.get_collection(collection_name)
        logger.info(f"Collection '{collection_name}' already exists")
    except:
        # Create collection if it doesn't exist
        client.create_collection(
            collection_name=collection_name,
            vectors_config=models.VectorParams(
                size=768,  # Cohere multilingual-v2.0 model outputs 768-dimensional vectors
                distance=models.Distance.COSINE,
            ),
        )
        logger.info(f"Created collection '{collection_name}'")


def save_chunks_to_qdrant_batch(
    client: QdrantClient,
    chunks: List[Dict],
    embeddings: List[List[float]],
    collection_name: str = "rag_embedding",
    max_retries: int = 3,
    batch_size: int = 64,
):
    """
    Save multiple chunks with their embeddings to Qdrant in batches

    Args:
        client: Qdrant client instance
        chunks: List of dictionaries containing chunk data
        embeddings: List of embedding vectors corresponding to the chunks
        collection_name: Name of the collection to save to
        max_retries: Maximum number of retries for failed requests
        batch_size: Maximum number of points to save in a single API call
    """
    if len(chunks) != len(embeddings):
        raise ValueError(
            f"Number of chunks ({len(chunks)}) must match number of embeddings ({len(embeddings)})"
        )

    # Process in batches
    for i in range(0, len(chunks), batch_size):
        batch_chunks = chunks[i : i + batch_size]
        batch_embeddings = embeddings[i : i + batch_size]

        retry_count = 0
        success = False

        while retry_count < max_retries and not success:
            try:
                points = []
                for chunk, embedding in zip(batch_chunks, batch_embeddings):
                    points.append(
                        models.PointStruct(
                            id=str(
                                abs(hash(chunk["content"] + str(i)))
                            ),  # Hash-based ID with batch index to reduce collisions
                            vector=embedding,
                            payload={
                                "content": chunk["content"],
                                "source_url": chunk.get("source_url", ""),
                                "title": chunk.get("title", ""),
                                "start_pos": chunk.get("start_pos", 0),
                                "end_pos": chunk.get("end_pos", 0),
                            },
                        )
                    )

                client.upsert(collection_name=collection_name, points=points)
                success = True
            except requests.Timeout as e:
                retry_count += 1
                logger.warning(
                    f"Timeout saving batch {i // batch_size + 1} to Qdrant (attempt {retry_count}/{max_retries}): {str(e)}"
                )
                if retry_count < max_retries:
                    time.sleep(2**retry_count)  # Exponential backoff
            except requests.RequestException as e:
                retry_count += 1
                logger.warning(
                    f"Request error saving batch {i // batch_size + 1} to Qdrant (attempt {retry_count}/{max_retries}): {str(e)}"
                )
                if retry_count < max_retries:
                    time.sleep(2**retry_count)  # Exponential backoff
            except Exception as e:
                retry_count += 1
                logger.warning(
                    f"Error saving batch {i // batch_size + 1} to Qdrant (attempt {retry_count}/{max_retries}): {str(e)}"
                )
                if retry_count < max_retries:
                    time.sleep(2**retry_count)  # Exponential backoff

        if not success:
            raise Exception(
                f"Failed to save batch {i // batch_size + 1} to Qdrant after {max_retries} attempts"
            )

    logger.info(
        f"Successfully saved {len(chunks)} chunks to Qdrant in {len(chunks) // batch_size + 1} batches"
    )


def check_duplicate(
    client: QdrantClient, content: str, collection_name: str = "rag_embedding"
) -> bool:
    """
    Check if content already exists in the Qdrant collection

    Args:
        client: Qdrant client instance
        content: Content to check for duplicates
        collection_name: Name of the collection to check

    Returns:
        Boolean indicating if the content already exists
    """
    try:
        # Create a simple hash of the content to use as a search term
        content_hash = str(abs(hash(content)))

        # Search for points with the same content hash in payload
        search_results = client.scroll(
            collection_name=collection_name,
            scroll_filter=models.Filter(
                must=[
                    models.FieldCondition(
                        key="content",
                        match=models.MatchText(
                            text=content[:100] if len(content) > 100 else content
                        ),  # Use first 100 chars for matching
                    )
                ]
            ),
            limit=1,
        )

        # If we found any results, check if content matches exactly
        for result in search_results[
            0
        ]:  # results is a tuple (records, next_page_offset)
            if result.payload.get("content") == content:
                return True

        return False
    except Exception as e:
        logger.warning(f"Error checking for duplicates: {str(e)}")
        # If we can't check for duplicates, assume it's not a duplicate to avoid losing data
        return False


def save_chunk_to_qdrant(
    client: QdrantClient,
    chunk: Dict,
    embedding: List[float],
    collection_name: str = "rag_embedding",
    max_retries: int = 3,
):
    """
    Save a chunk with its embedding to Qdrant

    Args:
        client: Qdrant client instance
        chunk: Dictionary containing chunk data
        embedding: Embedding vector for the chunk
        collection_name: Name of the collection to save to
        max_retries: Maximum number of retries for failed requests
    """
    # Check for duplicates before saving (skip this for now due to Qdrant index issues)
    # if check_duplicate(client, chunk["content"], collection_name):
    #     logger.info(f"Duplicate content detected, skipping: {chunk['content'][:100]}...")
    #     return  # Skip saving if duplicate

    retry_count = 0
    success = False

    while retry_count < max_retries and not success:
        try:
            # Use a simple integer ID instead of hash to avoid validation issues
            import random

            point_id = random.randint(1, 1000000000)  # Generate random integer ID

            client.upsert(
                collection_name=collection_name,
                points=[
                    models.PointStruct(
                        id=point_id,  # Using integer ID to avoid validation issues
                        vector=embedding,
                        payload={
                            "content": chunk["content"],
                            "source_url": chunk.get("source_url", ""),
                            "title": chunk.get("title", ""),
                            "start_pos": chunk.get("start_pos", 0),
                            "end_pos": chunk.get("end_pos", 0),
                        },
                    )
                ],
            )
            success = True
        except requests.Timeout as e:
            retry_count += 1
            logger.warning(
                f"Timeout saving chunk to Qdrant (attempt {retry_count}/{max_retries}): {str(e)}"
            )
            if retry_count < max_retries:
                time.sleep(2**retry_count)  # Exponential backoff
        except requests.RequestException as e:
            retry_count += 1
            logger.warning(
                f"Request error saving chunk to Qdrant (attempt {retry_count}/{max_retries}): {str(e)}"
            )
            if retry_count < max_retries:
                time.sleep(2**retry_count)  # Exponential backoff
        except Exception as e:
            retry_count += 1
            logger.warning(
                f"Error saving chunk to Qdrant (attempt {retry_count}/{max_retries}): {str(e)}"
            )
            if retry_count < max_retries:
                time.sleep(2**retry_count)  # Exponential backoff

    if not success:
        raise Exception(f"Failed to save chunk to Qdrant after {max_retries} attempts")


def parse_arguments():
    """
    Parse command line arguments
    """
    parser = argparse.ArgumentParser(
        description="Website Ingestion & Vectorization for RAG and Validation"
    )

    # Validation-related arguments
    validation_group = parser.add_argument_group("validation options")
    validation_group.add_argument(
        "--validate-search", action="store_true", help="Run semantic search validation"
    )
    validation_group.add_argument(
        "--validate-metadata",
        action="store_true",
        help="Run metadata integrity validation",
    )
    validation_group.add_argument(
        "--validate-pipeline",
        action="store_true",
        help="Run end-to-end pipeline validation",
    )
    validation_group.add_argument(
        "--query", type=str, help="Query string for validation"
    )
    validation_group.add_argument(
        "--expected", type=str, help="Expected results for validation (comma-separated)"
    )
    validation_group.add_argument(
        "--top-k",
        type=int,
        default=5,
        help="Number of results to retrieve for validation (default: 5)",
    )
    validation_group.add_argument(
        "--threshold",
        type=float,
        default=0.7,
        help="Minimum similarity score threshold (default: 0.7)",
    )
    validation_group.add_argument(
        "--max-concurrent",
        type=int,
        default=10,
        help="Maximum concurrent validation tests (default: 10)",
    )

    # Original ingestion arguments
    ingestion_group = parser.add_argument_group("ingestion options")
    ingestion_group.add_argument(
        "--url", type=str, help="Base URL of the Docusaurus site to crawl"
    )
    ingestion_group.add_argument(
        "--max-pages",
        type=int,
        default=int(os.getenv("MAX_PAGES", "500")),
        help="Maximum number of pages to crawl (default from env or 500)",
    )
    ingestion_group.add_argument(
        "--max-depth",
        type=int,
        default=int(os.getenv("CRAWL_DEPTH", "3")),
        help="Maximum depth for crawling (default from env or 3)",
    )
    ingestion_group.add_argument(
        "--chunk-size",
        type=int,
        default=int(os.getenv("CHUNK_SIZE", "1000")),
        help="Size of text chunks in characters (default from env or 1000)",
    )
    ingestion_group.add_argument(
        "--chunk-overlap",
        type=int,
        default=int(os.getenv("CHUNK_OVERLAP", "100")),
        help="Overlap between chunks in characters (default from env or 100)",
    )
    ingestion_group.add_argument(
        "--max-retries",
        type=int,
        default=int(os.getenv("MAX_RETRIES", "3")),
        help="Maximum number of retries for failed requests (default from env or 3)",
    )

    parser.add_argument(
        "--log-level",
        type=str,
        default="INFO",
        choices=["DEBUG", "INFO", "WARNING", "ERROR"],
        help="Logging level (default: INFO)",
    )

    return parser.parse_args()


def validate_configuration():
    """
    Validate all required configuration parameters

    Raises:
        ValueError: If any required configuration is missing or invalid
    """
    # Check environment variables
    required_env_vars = ["COHERE_API_KEY", "QDRANT_URL", "QDRANT_API_KEY"]
    missing_vars = [var for var in required_env_vars if not os.getenv(var)]

    if missing_vars:
        raise ValueError(
            f"Missing required environment variables: {', '.join(missing_vars)}"
        )

    # Validate Qdrant connection
    qdrant_url = os.getenv("QDRANT_URL")
    qdrant_api_key = os.getenv("QDRANT_API_KEY")

    try:
        test_client = QdrantClient(
            url=qdrant_url,
            api_key=qdrant_api_key,
        )
        # Try to list collections to verify connection
        test_client.get_collections()
    except Exception as e:
        raise ValueError(f"Could not connect to Qdrant: {str(e)}")


def run_ingestion_pipeline(args):
    """
    Run the original RAG ingestion pipeline
    """
    start_time = time.time()

    # Configuration from arguments with environment variable fallbacks
    base_url = args.url
    max_pages = args.max_pages
    max_depth = args.max_depth
    chunk_size = args.chunk_size
    chunk_overlap = args.chunk_overlap
    max_retries = args.max_retries

    # Validate base URL
    if not base_url:
        raise ValueError("Base URL is required for ingestion. Use --url to specify it.")
    if not is_valid_url(base_url):
        raise ValueError(f"Invalid base URL: {base_url}")

    base_url = normalize_url(base_url)

    # Validate configuration values
    if max_pages <= 0:
        raise ValueError(f"max_pages must be positive, got {max_pages}")
    if max_depth <= 0:
        raise ValueError(f"max_depth must be positive, got {max_depth}")
    if chunk_size <= 0:
        raise ValueError(f"chunk_size must be positive, got {chunk_size}")
    if chunk_overlap < 0:
        raise ValueError(f"chunk_overlap must be non-negative, got {chunk_overlap}")
    if max_retries <= 0:
        raise ValueError(f"max_retries must be positive, got {max_retries}")

    # Initialize Qdrant client
    qdrant_url = os.getenv("QDRANT_URL")
    qdrant_api_key = os.getenv("QDRANT_API_KEY")

    qdrant_client = QdrantClient(
        url=qdrant_url,
        api_key=qdrant_api_key,
    )

    # Create collection
    create_collection(qdrant_client, "rag_embedding")

    logger.info(f"Starting to crawl: {base_url}")

    # Step 1: Get all URLs
    urls_start_time = time.time()
    urls = get_all_urls(base_url, max_pages, max_depth, max_retries)
    urls_time = time.time() - urls_start_time
    logger.info(f"Found {len(urls)} URLs to process in {urls_time:.2f} seconds")

    processed_count = 0
    total_chunks = 0
    content_extraction_time = 0
    embedding_time = 0
    storage_time = 0

    # Step 2-5: Process each URL
    for url in urls:
        logger.info(f"Processing: {url}")

        # Extract content
        extract_start_time = time.time()
        title, content = extract_text_from_url(url, max_retries)
        content_extraction_time += time.time() - extract_start_time

        if not content:
            logger.warning(f"No content extracted from {url}, skipping")
            continue

        # Chunk content
        chunks = chunk_text(content, chunk_size, chunk_overlap)

        # Process each chunk
        for i, chunk in enumerate(chunks):
            chunk_with_metadata = {**chunk, "source_url": url, "title": title}

            try:
                # Generate embedding
                embed_start_time = time.time()
                embeddings = embed([chunk["content"]], max_retries)
                embedding_time += time.time() - embed_start_time
                embedding = embeddings[0]  # Get the first (and only) embedding

                # Save to Qdrant
                storage_start_time = time.time()
                save_chunk_to_qdrant(
                    qdrant_client,
                    chunk_with_metadata,
                    embedding,
                    max_retries=max_retries,
                )
                storage_time += time.time() - storage_start_time

                total_chunks += 1

                if total_chunks % 10 == 0:
                    logger.info(f"Processed {total_chunks} chunks so far...")

            except Exception as e:
                logger.error(f"Error processing chunk from {url}: {str(e)}")
                continue

        processed_count += 1

        # Log progress
        if processed_count % 10 == 0:
            logger.info(f"Processed {processed_count}/{len(urls)} URLs")

    total_time = time.time() - start_time
    logger.info(
        f"Pipeline completed! Processed {processed_count} URLs and {total_chunks} content chunks."
    )
    logger.info(f"Performance metrics:")
    logger.info(f"  Total execution time: {total_time:.2f} seconds")
    logger.info(f"  URL discovery time: {urls_time:.2f} seconds")
    logger.info(f"  Content extraction time: {content_extraction_time:.2f} seconds")
    logger.info(f"  Embedding generation time: {embedding_time:.2f} seconds")
    logger.info(f"  Storage time: {storage_time:.2f} seconds")
    if total_chunks > 0:
        logger.info(
            f"  Average time per chunk: {(content_extraction_time + embedding_time + storage_time) / total_chunks:.2f} seconds"
        )
    logger.info(
        "Embeddings have been stored in Qdrant Cloud collection 'rag_embedding'."
    )


def run_validation(args):
    """
    Run validation based on command line arguments
    """
    try:
        validate_api_keys()

        # Initialize Qdrant client
        qdrant_client = get_qdrant_client()

        if args.validate_search:
            logger.info("Starting semantic search validation...")

            if not args.query:
                raise ValueError("--query is required for semantic search validation")

            # Parse expected results if provided
            expected_results = []
            if args.expected:
                expected_results = [item.strip() for item in args.expected.split(",")]

            # Create query configuration
            config = QueryConfiguration(
                top_k=args.top_k,
                threshold=args.threshold,
                max_concurrent_tests=args.max_concurrent,
            )

            # Run semantic search validation
            report = validate_semantic_search(
                args.query, expected_results, config.__dict__
            )
            logger.info(f"Semantic search validation completed: {report}")

        elif args.validate_metadata:
            logger.info("Starting metadata integrity validation...")

            # For now, we'll validate metadata from a sample query
            # In a real scenario, we would get results from a query
            from validation.query_processor import process_query

            # If a query is provided, process it and validate metadata
            if args.query:
                config = {
                    "top_k": args.top_k,
                    "threshold": args.threshold,
                    "qdrant_collection": "rag_embedding",
                }

                # Process the query to get results
                results = process_query(args.query, config)

                # Validate metadata integrity
                from validation.metadata_validator import validate_metadata_integrity

                validation_result = validate_metadata_integrity(results)

                logger.info(f"Metadata validation completed: {validation_result}")
            else:
                logger.warning(
                    "--query is required for metadata validation. Using a sample approach for now."
                )
                # For now, show a sample validation with mock data
                mock_results = [
                    {
                        "content": "Sample content for testing",
                        "source_url": "https://example.com/test",
                        "title": "Sample Title",
                        "start_pos": 0,
                        "end_pos": 100,
                        "embedding_id": "test_id_1",
                    }
                ]

                from validation.metadata_validator import validate_metadata_integrity

                validation_result = validate_metadata_integrity(mock_results)

                logger.info(
                    f"Sample metadata validation completed: {validation_result}"
                )

        elif args.validate_pipeline:
            logger.info("Starting end-to-end pipeline validation...")

            # Create test configuration
            test_config = {
                "queries": [args.query]
                if args.query
                else [
                    "What is RAG?",
                    "How does semantic search work?",
                    "Explain vector databases",
                ],
                "max_concurrent_tests": args.max_concurrent,
                "timeout_seconds": 30,  # Default timeout
                "expected_accuracy_threshold": 0.85,  # Default threshold
                "top_k": args.top_k,
                "threshold": args.threshold,
                "qdrant_collection": "rag_embedding",
            }

            # Run pipeline validation
            from validation.pipeline_validator import validate_end_to_end_pipeline

            validation_result = validate_end_to_end_pipeline(test_config)

            logger.info(f"Pipeline validation completed: {validation_result}")

    except ValueError as e:
        logger.error(f"Validation configuration error: {str(e)}")
        raise
    except Exception as e:
        logger.error(f"Validation pipeline failed with error: {str(e)}")
        raise


def cli_main():
    """
    Main function to execute the complete RAG ingestion pipeline or validation
    """
    # Parse command line arguments
    args = parse_arguments()

    # Set logging level based on arguments
    logging.getLogger().setLevel(getattr(logging, args.log_level.upper()))

    try:
        # Validate configuration before proceeding
        validate_configuration()

        # Check if any validation flags are set
        if args.validate_search or args.validate_metadata or args.validate_pipeline:
            logger.info("Starting RAG validation pipeline...")
            run_validation(args)
        else:
            logger.info("Starting RAG ingestion pipeline...")
            run_ingestion_pipeline(args)

    except KeyboardInterrupt:
        logger.info("Pipeline interrupted by user")
        raise
    except Exception as e:
        logger.error(f"Pipeline failed with error: {str(e)}")
        raise


rag_agent = get_rag_agent()

# FastAPI app for the RAG agent service
app = FastAPI(
    title="RAG Agent API",
    description="API for RAG-enabled agent using OpenAI SDK with Google Gemini",
    version="1.0.0",
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, replace with specific origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include API router
app.include_router(router, prefix="", tags=["query"])


# Direct query endpoint for RAG agent
@app.post("/query")
async def query_agent(request: QueryRequest):
    try:
        # Correct way: await the async method
        response = await rag_agent.query(request.query)
        return QueryResponse(answer=response)
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Agent error: {str(e)}")


if __name__ == "__main__" and "--url" in os.sys.argv:
    cli_main()
