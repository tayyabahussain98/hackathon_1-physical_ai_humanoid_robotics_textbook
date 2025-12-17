"""
API request/response models for RAG Agent Backend with OpenAI Agents SDK using Gemini.

This module defines Pydantic models for API requests and responses.
"""
from typing import List, Optional
from pydantic import BaseModel, Field


class QueryRequest(BaseModel):
    """
    Request model for the query endpoint.
    """
    query: str = Field(..., description="The user's query text", min_length=1)
    top_k: Optional[int] = Field(
        default=None,
        description="Number of documents to retrieve (default: 5)",
        ge=1,
        le=20
    )
    threshold: Optional[float] = Field(
        default=None,
        description="Minimum similarity score threshold (default: 0.7)",
        ge=0.0,
        le=1.0
    )
    include_sources: Optional[bool] = Field(
        default=True,
        description="Whether to include source information in response (default: true)"
    )


class RetrievedDocument(BaseModel):
    """
    Model representing a retrieved document from Qdrant.
    """
    content: str = Field(..., description="The content of the retrieved document")
    similarity_score: float = Field(
        ...,
        description="Similarity score of the document to the query",
        ge=0.0,
        le=1.0
    )
    source_url: str = Field(..., description="URL of the source document")
    title: Optional[str] = Field(default=None, description="Title of the source document")


class QueryResponse(BaseModel):
    """
    Response model for the query endpoint.
    """
    answer: str = Field(..., description="The agent's answer to the query")
    sources: List[str] = Field(..., description="Source documents referenced in the response")
    retrieved_documents: Optional[List[RetrievedDocument]] = Field(
        default=None,
        description="Documents retrieved during the process"
    )
    processing_time: float = Field(..., description="Time taken to process the request in seconds")
    error: Optional[str] = Field(default=None, description="Error message if processing failed")


class ErrorResponse(BaseModel):
    """
    Error response model for API errors.
    """
    error: str = Field(..., description="Error message")