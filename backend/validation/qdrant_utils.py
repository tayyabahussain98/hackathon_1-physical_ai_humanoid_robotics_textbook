"""
Qdrant utility functions for RAG validation system.

This module provides utilities for Qdrant client initialization and connection.
"""
import os
from typing import List, Dict, Any, Optional
from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.http.models import PointStruct
import logging
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

logger = logging.getLogger(__name__)


def get_qdrant_client() -> QdrantClient:
    """
    Initialize and return a Qdrant client using environment variables.

    Returns:
        QdrantClient instance configured with environment settings
    """
    qdrant_url = os.getenv("QDRANT_URL")
    qdrant_api_key = os.getenv("QDRANT_API_KEY")

    if not qdrant_url:
        raise ValueError("QDRANT_URL environment variable not set")
    if not qdrant_api_key:
        raise ValueError("QDRANT_API_KEY environment variable not set")

    client = QdrantClient(
        url=qdrant_url,
        api_key=qdrant_api_key,
    )

    # Test connection
    try:
        client.get_collections()
        logger.info("Successfully connected to Qdrant")
    except Exception as e:
        logger.error(f"Failed to connect to Qdrant: {str(e)}")
        raise

    return client


def search_in_qdrant(
    client: QdrantClient,
    collection_name: str,
    query_vector: List[float],
    top_k: int = 5,
    threshold: float = 0.0
) -> List[Dict[str, Any]]:
    """
    Perform a similarity search in Qdrant collection.

    Args:
        client: Qdrant client instance
        collection_name: Name of the collection to search in
        query_vector: Query embedding vector
        top_k: Number of results to retrieve
        threshold: Minimum similarity score threshold

    Returns:
        List of search results with content and metadata
    """
    try:
        # Perform search with the specified parameters
        search_results = client.query_points(
            collection_name=collection_name,
            query=query_vector,
            limit=top_k,
            score_threshold=threshold
        )

        results = []
        # query_points returns a named tuple-like object with 'points' attribute
        search_points = search_results.points if hasattr(search_results, 'points') else search_results
        for hit in search_points:
            result = {
                "content": hit.payload.get("content", "") if hit.payload else "",
                "similarity_score": hit.score,
                "metadata": hit.payload if hit.payload else {},
                "source_url": hit.payload.get("source_url", "") if hit.payload else "",
                "title": hit.payload.get("title", "") if hit.payload else "",
                "start_pos": hit.payload.get("start_pos", 0) if hit.payload else 0,
                "end_pos": hit.payload.get("end_pos", 0) if hit.payload else 0,
                "embedding_id": str(hit.id)
            }
            results.append(result)

        return results

    except Exception as e:
        logger.error(f"Error performing search in Qdrant: {str(e)}")
        raise


def validate_qdrant_connection() -> bool:
    """
    Validate that the Qdrant connection is working properly.

    Returns:
        Boolean indicating if connection is valid
    """
    try:
        client = get_qdrant_client()
        # Try to list collections to verify connection
        client.get_collections()
        return True
    except Exception as e:
        logger.error(f"Qdrant connection validation failed: {str(e)}")
        return False


def collection_exists(client: QdrantClient, collection_name: str) -> bool:
    """
    Check if a collection exists in Qdrant.

    Args:
        client: Qdrant client instance
        collection_name: Name of the collection to check

    Returns:
        Boolean indicating if collection exists
    """
    try:
        client.get_collection(collection_name)
        return True
    except:
        return False