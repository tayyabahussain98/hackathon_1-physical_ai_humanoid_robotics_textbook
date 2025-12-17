"""
RAG agent implementation using OpenAI Agents SDK with Google Gemini via OpenAI-compatible API.

This module contains the RAG agent class and OpenAI-compatible client configuration.
"""
import os
from typing import List, Dict, Any, Optional
from agents import Agent, Runner, OpenAIChatCompletionsModel
from openai import AsyncOpenAI
from dotenv import load_dotenv, find_dotenv
from .system_prompt import RAG_AGENT_SYSTEM_PROMPT

# Load environment variables
load_dotenv(find_dotenv())


class RAGAgent:
    """
    RAG agent that uses OpenAI Agents SDK configured to work with Google Gemini via OpenAI-compatible API.
    """

    def __init__(self, api_key: Optional[str] = None, base_url: Optional[str] = None, model: Optional[str] = None):
        """
        Initialize the RAG agent with OpenAI-compatible API configuration.

        Args:
            api_key: Gemini API key (defaults to GEMINI_API_KEY environment variable)
            base_url: OpenAI-compatible API base URL for Gemini (defaults to Google's endpoint)
            model: Model name to use (defaults to GEMINI_MODEL_NAME environment variable)
        """
        self.api_key = api_key or os.getenv("GEMINI_API_KEY")
        if not self.api_key:
            raise ValueError("GEMINI_API_KEY must be provided either as parameter or environment variable")

        self.base_url = base_url or "https://generativelanguage.googleapis.com/v1beta/openai/"
        self.model = model or os.getenv("GEMINI_MODEL_NAME", "gemini-2.5-flash")

        # Initialize OpenAI client with Google's OpenAI-compatible API
        self.external_client: AsyncOpenAI = AsyncOpenAI(
            api_key=self.api_key,
            base_url=self.base_url
        )

        # Initialize the LLM model
        self.llm_model: OpenAIChatCompletionsModel = OpenAIChatCompletionsModel(
            model=self.model,
            openai_client=self.external_client
        )

        # Initialize the agent with system instructions
        self.agent: Agent = Agent(
            name="RAG-Assistant",
            model=self.llm_model,
            instructions=RAG_AGENT_SYSTEM_PROMPT
        )

    async def query(self, user_query: str, context_documents: Optional[List[Dict[str, Any]]] = None) -> str:
        """
        Process a user query with optional context documents.

        Args:
            user_query: The user's query string
            context_documents: Optional list of context documents to ground the response

        Returns:
            The agent's response string
        """
        # Format the context if provided
        context_str = ""
        sources = []

        if context_documents:
            context_str = "[DOCUMENTS]\n"
            for i, doc in enumerate(context_documents, 1):
                content = doc.get('content', '')
                source_url = doc.get('source_url', 'Unknown source')
                title = doc.get('title', '')

                # Add document to context
                context_str += f"Document {i}: {content} (Source: {source_url})\n"
                sources.append(source_url)
        else:
            context_str = "[DOCUMENTS]\nNo relevant documents found in the knowledge base.\n"
            sources.append("No sources available")

        # Create the full prompt with context
        full_prompt = f"Context:\n{context_str}\n\nUser Query:\n{user_query}"

        try:
            # Run the agent with the formatted prompt
            result = await Runner.run(  # await must be here
                    starting_agent=self.agent,
                    input=full_prompt
                )
            return result.final_output

        except Exception as e:
            # Handle API errors gracefully
            error_msg = f"Error calling Gemini API: {str(e)}"
            print(error_msg)  # In production, use proper logging
            return f"Sorry, I encountered an error processing your request: {str(e)}"

    def health_check(self) -> bool:
        """
        Perform a health check by verifying API configuration without making an expensive API call.

        Returns:
            True if the API is properly configured, False otherwise
        """
        try:
            # Just verify that the API key and client are properly configured
            # Don't make an actual API call that would consume quota
            if not self.api_key or len(self.api_key.strip()) == 0:
                return False

            # Verify the client was initialized properly
            if not hasattr(self, 'external_client') or self.external_client is None:
                return False

            # Verify the model is configured
            if not self.model or len(self.model.strip()) == 0:
                return False

            return True
        except Exception:
            return False


# Global instance if needed
rag_agent = None


def get_rag_agent() -> RAGAgent:
    """
    Get or create a singleton instance of the RAG agent.

    Returns:
        RAGAgent instance
    """
    global rag_agent
    if rag_agent is None:
        rag_agent = RAGAgent()
    return rag_agent
