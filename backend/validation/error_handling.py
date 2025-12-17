"""
Error handling and retry mechanisms for RAG validation system.

This module provides utilities for error handling and retry logic.
"""
import time
import logging
from functools import wraps
from typing import Callable, Any, Optional, Tuple
from requests.exceptions import RequestException
import cohere
from qdrant_client import QdrantClient

logger = logging.getLogger(__name__)


def retry_with_exponential_backoff(
    max_retries: int = 3,
    base_delay: float = 1.0,
    max_delay: float = 60.0,
    exceptions: Tuple = (Exception,)
):
    """
    Decorator to retry a function with exponential backoff.

    Args:
        max_retries: Maximum number of retry attempts
        base_delay: Initial delay between retries in seconds
        max_delay: Maximum delay between retries in seconds
        exceptions: Tuple of exceptions to catch and retry on
    """
    def decorator(func: Callable) -> Callable:
        @wraps(func)
        def wrapper(*args, **kwargs) -> Any:
            last_exception = None

            for attempt in range(max_retries + 1):
                try:
                    return func(*args, **kwargs)
                except exceptions as e:
                    last_exception = e
                    if attempt == max_retries:
                        # No more retries left
                        break

                    # Calculate delay with exponential backoff and jitter
                    delay = min(base_delay * (2 ** attempt), max_delay)
                    jitter = delay * 0.1  # 10% jitter
                    actual_delay = delay + (jitter * (attempt % 2 - 0.5)) * 2  # Random jitter

                    logger.warning(
                        f"Attempt {attempt + 1}/{max_retries} failed for {func.__name__}: {str(e)}. "
                        f"Retrying in {actual_delay:.2f} seconds..."
                    )
                    time.sleep(actual_delay)

            # If we get here, all retries failed
            logger.error(f"All {max_retries} retry attempts failed for {func.__name__}")
            raise last_exception

        return wrapper
    return decorator


def handle_validation_errors(func: Callable) -> Callable:
    """
    Decorator to handle common validation errors with appropriate logging.

    Args:
        func: Function to wrap with error handling
    """
    @wraps(func)
    def wrapper(*args, **kwargs) -> Any:
        try:
            return func(*args, **kwargs)
        except cohere.CohereError as e:
            logger.error(f"Cohere API error in {func.__name__}: {str(e)}")
            raise
        except Exception as e:
            logger.error(f"Unexpected error in {func.__name__}: {str(e)}")
            raise

    return wrapper


class ValidationError(Exception):
    """Base exception for validation errors."""
    pass


class RetrievalError(Exception):
    """Exception for retrieval-related errors."""
    pass


class MetadataValidationError(Exception):
    """Exception for metadata validation errors."""
    pass


class PipelineValidationError(Exception):
    """Exception for pipeline validation errors."""
    pass


class ReportGenerationError(Exception):
    """Exception for report generation errors."""
    pass


class EmbeddingGenerationError(Exception):
    """Exception for embedding generation errors."""
    pass


def validate_api_keys():
    """
    Validate that required API keys are present in environment variables.

    Raises:
        ValueError: If required API keys are missing
    """
    import os
    required_keys = ["COHERE_API_KEY", "QDRANT_URL", "QDRANT_API_KEY"]

    missing_keys = [key for key in required_keys if not os.getenv(key)]
    if missing_keys:
        raise ValueError(f"Missing required environment variables: {', '.join(missing_keys)}")


def validate_configuration(config):
    """
    Validate configuration objects.

    Args:
        config: Configuration object to validate

    Raises:
        ValidationError: If configuration is invalid
    """
    if hasattr(config, 'validate') and callable(getattr(config, 'validate')):
        try:
            config.validate()
        except Exception as e:
            raise ValidationError(f"Configuration validation failed: {str(e)}")