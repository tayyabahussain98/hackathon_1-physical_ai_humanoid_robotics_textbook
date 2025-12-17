"""
API router for RAG Agent Backend with OpenAI Agents SDK using Gemini.

This module implements the API endpoints and connects them to the agent.
"""
from fastapi import APIRouter, HTTPException, Depends
from pydantic import BaseModel
import time
from typing import Optional

from agent.rag_agent import get_rag_agent
from api.models import QueryRequest, QueryResponse, ErrorResponse
from config.settings import settings
from services.qdrant_service import get_qdrant_service
from services.context_injector import get_context_injector
from utils.embeddings import get_embedding_service
from validation.query_processor import process_query  # Import existing validation functionality


router = APIRouter()


@router.post("/query", response_model=QueryResponse)
async def query_endpoint(request: QueryRequest):
    """
    Query endpoint that processes user queries through the RAG agent.

    Args:
        request: QueryRequest containing the user query and optional parameters

    Returns:
        QueryResponse with the agent's answer and metadata
    """
    start_time = time.time()

    try:
        # Use default values if not provided in request
        top_k = request.top_k or settings.default_top_k
        threshold = request.threshold or settings.default_threshold

        # Get services
        rag_agent = get_rag_agent()
        qdrant_service = get_qdrant_service()
        context_injector = get_context_injector()
        embedding_service = get_embedding_service()

        # Generate embedding for the user query
        query_embedding = embedding_service.generate_embedding(request.query)

        # Retrieve documents from Qdrant based on the query embedding
        retrieved_docs = qdrant_service.search(
            query_vector=query_embedding,
            top_k=top_k,
            threshold=threshold
        )

        # Format context for the agent
        formatted_context = context_injector.format_context_for_agent(retrieved_docs, request.query)

        # Get response from the agent
        agent_response = await rag_agent.query(request.query, retrieved_docs)

        # Calculate processing time
        processing_time = time.time() - start_time

        # Extract sources from the response
        sources = context_injector.extract_sources(agent_response, retrieved_docs)

        # Create RetrievedDocument models
        retrieved_document_models = context_injector.create_retrieved_document_models(retrieved_docs)

        # Create and return response
        response = QueryResponse(
            answer=agent_response,
            sources=sources,
            retrieved_documents=retrieved_document_models if retrieved_docs else None,
            processing_time=processing_time
        )

        return response

    except Exception as e:
        processing_time = time.time() - start_time
        error_response = QueryResponse(
            answer="",
            sources=[],
            retrieved_documents=None,
            processing_time=processing_time,
            error=str(e)
        )
        # Log the error in a real implementation
        raise HTTPException(status_code=500, detail=str(e))


@router.get("/health")
async def health_check():
    """
    Health check endpoint to verify the system is running.

    Returns:
        Health status information
    """
    try:
        # Check agent health
        rag_agent = get_rag_agent()
        agent_healthy = rag_agent.health_check()

        # Check Qdrant health
        qdrant_service = get_qdrant_service()
        qdrant_healthy = qdrant_service.health_check()

        overall_healthy = agent_healthy and qdrant_healthy

        return {
            "status": "healthy" if overall_healthy else "unhealthy",
            "details": {
                "agent": "healthy" if agent_healthy else "unhealthy",
                "qdrant": "healthy" if qdrant_healthy else "unhealthy"
            }
        }
    except Exception as e:
        return {
            "status": "unhealthy",
            "details": {
                "error": str(e)
            }
        }