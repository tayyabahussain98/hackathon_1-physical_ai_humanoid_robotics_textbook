"""
Configuration settings for RAG Agent Backend with OpenAI Agents SDK using Gemini.

This module handles environment variable loading and validation.
"""
import os
from typing import Optional
from pydantic import BaseModel, field_validator


class Settings(BaseModel):
    """
    Application settings loaded from environment variables.
    """
    # Qdrant configuration
    qdrant_url: str = os.getenv("QDRANT_URL", "")
    qdrant_api_key: str = os.getenv("QDRANT_API_KEY", "")

    # Gemini configuration
    gemini_api_key: str = os.getenv("GEMINI_API_KEY", "")
    gemini_model_name: str = os.getenv("GEMINI_MODEL_NAME", "gemini-2.5-flash")

    # Application configuration
    default_top_k: int = int(os.getenv("DEFAULT_TOP_K", "5"))
    default_threshold: float = float(os.getenv("DEFAULT_THRESHOLD", "0.7"))
    api_host: str = os.getenv("API_HOST", "0.0.0.0")
    api_port: int = int(os.getenv("API_PORT", "8000"))

    # Validation
    @field_validator('qdrant_url')
    @classmethod
    def qdrant_url_must_not_be_empty(cls, v):
        if not v:
            raise ValueError('QDRANT_URL must be provided')
        return v

    @field_validator('qdrant_api_key')
    @classmethod
    def qdrant_api_key_must_not_be_empty(cls, v):
        if not v:
            raise ValueError('QDRANT_API_KEY must be provided')
        return v

    @field_validator('gemini_api_key')
    @classmethod
    def gemini_api_key_must_not_be_empty(cls, v):
        if not v:
            raise ValueError('GEMINI_API_KEY must be provided')
        return v

    @field_validator('default_top_k')
    @classmethod
    def default_top_k_must_be_valid(cls, v):
        if v <= 0 or v > 20:
            raise ValueError('DEFAULT_TOP_K must be between 1 and 20')
        return v

    @field_validator('default_threshold')
    @classmethod
    def default_threshold_must_be_valid(cls, v):
        if v < 0.0 or v > 1.0:
            raise ValueError('DEFAULT_THRESHOLD must be between 0.0 and 1.0')
        return v


# Create a singleton instance of settings
settings = Settings()