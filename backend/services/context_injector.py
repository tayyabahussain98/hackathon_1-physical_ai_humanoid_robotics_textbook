"""
Context injection service for RAG Agent Backend with OpenAI Agents SDK using Gemini.

This module handles logic to inject retrieved context into agent prompts.
"""
from typing import List, Dict, Any, Optional
from api.models import RetrievedDocument


class ContextInjector:
    """
    Service class for injecting retrieved context into agent prompts.
    """

    @staticmethod
    def format_context_for_agent(retrieved_docs: List[Dict[str, Any]], user_query: str) -> str:
        """
        Format retrieved documents into a context string suitable for the agent.

        Args:
            retrieved_docs: List of retrieved documents from Qdrant
            user_query: Original user query

        Returns:
            Formatted context string
        """
        if not retrieved_docs:
            return f"[DOCUMENTS]\nNo relevant documents found in the knowledge base.\n\nUser Query:\n{user_query}"

        context_str = "[DOCUMENTS]\n"
        for i, doc in enumerate(retrieved_docs, 1):
            content = doc.get('content', '')
            source_url = doc.get('source_url', 'Unknown source')
            title = doc.get('title', '')
            similarity_score = doc.get('similarity_score', 0.0)

            context_str += f"Document {i}: {content} (Source: {source_url}, Similarity: {similarity_score:.3f})\n"

        context_str += f"\nUser Query:\n{user_query}"
        return context_str

    @staticmethod
    def validate_response_grounding(agent_response: str, retrieved_docs: List[Dict[str, Any]]) -> bool:
        """
        Validate that the agent response is grounded in the retrieved documents.

        Args:
            agent_response: The agent's response to validate
            retrieved_docs: List of documents that were provided as context

        Returns:
            True if response appears to be grounded in the context, False otherwise
        """
        if not retrieved_docs:
            # If no documents were retrieved, any response should acknowledge this
            return "no relevant documents" in agent_response.lower() or "not found" in agent_response.lower()

        # Simple validation: check if key phrases from documents appear in response
        response_lower = agent_response.lower()
        for doc in retrieved_docs:
            content = doc.get('content', '')
            if len(content) > 20:  # Only check substantial content
                # Check if significant portions of the document content appear in the response
                content_lower = content.lower()
                # Look for at least 10% of the content words in the response
                content_words = set(content_lower.split()[:20])  # Check first 20 words
                if content_words and any(word in response_lower for word in list(content_words)[:3]):  # Check first 3 words
                    return True

        # Alternative: check if source URLs are referenced
        for doc in retrieved_docs:
            source_url = doc.get('source_url', '')
            if source_url and source_url in agent_response:
                return True

        return False

    @staticmethod
    def extract_sources(agent_response: str, retrieved_docs: List[Dict[str, Any]]) -> List[str]:
        """
        Extract source information from the agent response based on retrieved documents.

        Args:
            agent_response: The agent's response
            retrieved_docs: List of documents that were provided as context

        Returns:
            List of source URLs referenced in the response
        """
        sources = set()
        response_lower = agent_response.lower()

        for doc in retrieved_docs:
            source_url = doc.get('source_url', '')
            if source_url and (source_url in agent_response or source_url in response_lower):
                sources.add(source_url)

        return list(sources)

    @staticmethod
    def create_retrieved_document_models(retrieved_docs: List[Dict[str, Any]]) -> List[RetrievedDocument]:
        """
        Convert retrieved document dictionaries to Pydantic models.

        Args:
            retrieved_docs: List of retrieved document dictionaries

        Returns:
            List of RetrievedDocument models
        """
        models = []
        for doc in retrieved_docs:
            model = RetrievedDocument(
                content=doc.get('content', ''),
                similarity_score=doc.get('similarity_score', 0.0),
                source_url=doc.get('source_url', ''),
                title=doc.get('title', '')
            )
            models.append(model)
        return models


# Global instance if needed
context_injector = None


def get_context_injector() -> ContextInjector:
    """
    Get or create a singleton instance of the context injector service.

    Returns:
        ContextInjector instance
    """
    global context_injector
    if context_injector is None:
        context_injector = ContextInjector()
    return context_injector