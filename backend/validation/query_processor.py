"""
Query processing module for RAG validation system.

This module handles query embedding generation and processing for validation.
"""
import os
import cohere
from typing import List
import logging
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

logger = logging.getLogger(__name__)

def generate_query_embedding(query: str) -> List[float]:
    """
    Generates an embedding for the input query using Cohere.

    Args:
        query: The input query string

    Returns:
        Embedding vector as a list of floats

    Raises:
        EmbeddingGenerationError: If embedding generation fails
    """
    cohere_api_key = os.getenv("COHERE_API_KEY")
    if not cohere_api_key:
        raise ValueError("COHERE_API_KEY environment variable not set")

    try:
        co = cohere.Client(cohere_api_key)
        response = co.embed(
            texts=[query],
            model="embed-multilingual-v2.0"  # Using the same model as RAG ingestion
        )
        return response.embeddings[0]  # Return the first (and only) embedding
    except Exception as e:
        logger.error(f"Error generating query embedding: {str(e)}")
        raise EmbeddingGenerationError(f"Failed to generate embedding: {str(e)}")


def process_query(query: str, config: dict) -> List[dict]:
    """
    Processes a query by generating embeddings and searching in Qdrant.

    Args:
        query: The input query string
        config: Configuration parameters for query processing

    Returns:
        List of search results from the vector database
    """
    from .qdrant_utils import get_qdrant_client, search_in_qdrant

    # Generate query embedding
    query_embedding = generate_query_embedding(query)

    # Get Qdrant client
    client = get_qdrant_client()

    # Extract config parameters with defaults
    top_k = config.get('top_k', 5)
    threshold = config.get('threshold', 0.7)
    collection_name = config.get('qdrant_collection', 'rag_embedding')

    # Perform search in Qdrant
    results = search_in_qdrant(
        client=client,
        collection_name=collection_name,
        query_vector=query_embedding,
        top_k=top_k,
        threshold=threshold
    )

    return results


class EmbeddingGenerationError(Exception):
    """Exception raised when embedding generation fails."""
    pass