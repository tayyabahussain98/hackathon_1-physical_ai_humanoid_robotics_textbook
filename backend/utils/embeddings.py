"""
Embedding utilities for RAG Agent Backend with OpenAI Agents SDK using Gemini.

This module handles text embedding generation for semantic search queries.
"""

import os
from typing import List
import cohere
from config.settings import settings


class EmbeddingService:
    """
    Service class for generating embeddings using Cohere's embedding models.
    """

    def __init__(self):
        """
        Initialize the embedding service with Cohere's API.
        """
        api_key = os.getenv("COHERE_API_KEY")
        if not api_key:
            raise ValueError("COHERE_API_KEY must be provided as environment variable")

        self.client = cohere.Client(api_key)
        # Use a Cohere embedding model - multilingual-22-12 is a good general purpose model
        self.model_name = "embed-multilingual-v2.0"

    def generate_embedding(self, text: str) -> List[float]:
        """
        Generate embedding for a single text string.

        Args:
            text: Input text to embed

        Returns:
            List of floats representing the embedding vector
        """
        try:
            response = self.client.embed(
                texts=[text],
                model=self.model_name
            )
            return response.embeddings[0]  # Return the first (and only) embedding
        except Exception as e:
            raise Exception(f"Error generating embedding: {str(e)}")

    def generate_embeddings_batch(self, texts: List[str]) -> List[List[float]]:
        """
        Generate embeddings for a batch of text strings.

        Args:
            texts: List of input texts to embed

        Returns:
            List of embedding vectors (each vector is a list of floats)
        """
        try:
            response = self.client.embed(
                texts=texts,
                model=self.model_name
            )
            return response.embeddings
        except Exception as e:
            raise Exception(f"Error generating embeddings batch: {str(e)}")


# Global instance if needed
embedding_service = None


def get_embedding_service() -> EmbeddingService:
    """
    Get or create a singleton instance of the embedding service.

    Returns:
        EmbeddingService instance
    """
    global embedding_service
    if embedding_service is None:
        embedding_service = EmbeddingService()
    return embedding_service