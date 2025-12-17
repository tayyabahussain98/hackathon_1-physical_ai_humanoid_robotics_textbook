"""
Qdrant service for RAG Agent Backend with OpenAI Agents SDK using Gemini.

This module handles Qdrant client initialization and semantic search functionality.
"""
import os
from typing import List, Dict, Any, Optional
from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.http.models import PointStruct
from config.settings import settings


class QdrantService:
    """
    Service class for interacting with Qdrant vector database.
    """

    def __init__(self, url: Optional[str] = None, api_key: Optional[str] = None, collection_name: str = "rag_embedding"):
        """
        Initialize the Qdrant service.

        Args:
            url: Qdrant URL (defaults to settings value)
            api_key: Qdrant API key (defaults to settings value)
            collection_name: Name of the collection to use (default: rag_embedding)
        """
        self.qdrant_url = url or settings.qdrant_url
        self.qdrant_api_key = api_key or settings.qdrant_api_key
        self.collection_name = collection_name

        # Initialize Qdrant client
        self.client = QdrantClient(
            url=self.qdrant_url,
            api_key=self.qdrant_api_key,
        )

        # Verify connection
        try:
            self.client.get_collections()
        except Exception as e:
            raise ConnectionError(f"Failed to connect to Qdrant: {str(e)}")

    def search(self, query_vector: List[float], top_k: int = 5, threshold: float = 0.0) -> List[Dict[str, Any]]:
        """
        Perform semantic search in Qdrant collection.

        Args:
            query_vector: Query embedding vector
            top_k: Number of results to retrieve
            threshold: Minimum similarity score threshold

        Returns:
            List of search results with content and metadata
        """
        try:
            # Perform search with the specified parameters
            search_results = self.client.query_points(
                collection_name=self.collection_name,
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
            raise Exception(f"Error performing search in Qdrant: {str(e)}")

    def health_check(self) -> bool:
        """
        Perform a health check by verifying the connection to Qdrant.

        Returns:
            True if Qdrant is accessible, False otherwise
        """
        try:
            # Try to list collections to verify connection
            self.client.get_collections()
            return True
        except Exception:
            return False


# Global instance if needed
qdrant_service = None


def get_qdrant_service() -> QdrantService:
    """
    Get or create a singleton instance of the Qdrant service.

    Returns:
        QdrantService instance
    """
    global qdrant_service
    if qdrant_service is None:
        qdrant_service = QdrantService()
    return qdrant_service