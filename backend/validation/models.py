"""
Data models for RAG validation system.

This module defines the core data structures for validation entities.
"""
from dataclasses import dataclass
from typing import List, Dict, Any, Optional
from datetime import datetime
from enum import Enum
import re


class ValidationType(Enum):
    """Type of validation being performed."""
    SEMANTIC_SEARCH = "SEMANTIC_SEARCH"
    METADATA_INTEGRITY = "METADATA_INTEGRITY"
    END_TO_END = "END_TO_END"


class ValidationStatus(Enum):
    """Status of a validation test."""
    PENDING = "PENDING"
    RUNNING = "RUNNING"
    COMPLETED = "COMPLETED"
    FAILED = "FAILED"


@dataclass
class QueryConfiguration:
    """Configuration parameters for query processing and validation."""
    top_k: int = 5
    threshold: float = 0.7
    max_concurrent_tests: int = 10
    timeout_seconds: int = 30
    validation_thresholds: Dict[str, float] = None

    def __post_init__(self):
        if self.validation_thresholds is None:
            self.validation_thresholds = {
                "semantic_search_precision": 0.85,
                "metadata_integrity": 0.95
            }

    def validate(self):
        """Validate the configuration parameters."""
        if self.top_k <= 0:
            raise ValueError("top_k must be a positive integer")
        if not 0.0 <= self.threshold <= 1.0:
            raise ValueError("threshold must be between 0.0 and 1.0")
        if not 1 <= self.max_concurrent_tests <= 100:
            raise ValueError("max_concurrent_tests must be between 1 and 100")


@dataclass
class ValidationConfiguration:
    """Overall configuration for the validation system."""
    cohere_model: str = "embed-multilingual-v2.0"
    qdrant_collection: str = "rag_embedding"
    default_query_config: QueryConfiguration = None
    validation_rules: Dict[str, Any] = None
    report_format: str = "detailed"

    def __post_init__(self):
        if self.default_query_config is None:
            self.default_query_config = QueryConfiguration()
        if self.validation_rules is None:
            self.validation_rules = {}

    def validate(self):
        """Validate the configuration parameters."""
        if not self.cohere_model:
            raise ValueError("cohere_model must be specified")
        if not self.qdrant_collection:
            raise ValueError("qdrant_collection must be specified")


@dataclass
class SearchResult:
    """Contains retrieved content chunks with similarity scores, metadata, and provenance information."""
    content: str
    similarity_score: float
    metadata: Dict[str, Any]
    source_url: str
    title: str
    start_pos: int
    end_pos: int
    embedding_id: str

    def validate(self):
        """Validate the search result data."""
        if not self.content:
            raise ValueError("Content must not be empty")
        if not 0.0 <= self.similarity_score <= 1.0:
            raise ValueError("Similarity score must be between 0.0 and 1.0")
        if self.start_pos < 0 or self.end_pos < 0:
            raise ValueError("Position values must be non-negative")
        if self.start_pos > self.end_pos:
            raise ValueError("Start position cannot be greater than end position")


@dataclass
class ValidationTest:
    """Represents a validation run that includes query inputs, expected outcomes, and actual results."""
    query: str
    expected_results: List[str]
    validation_type: ValidationType
    configuration: QueryConfiguration

    # Runtime fields (set during execution)
    id: Optional[str] = None
    actual_results: List[SearchResult] = None
    created_at: Optional[datetime] = None
    completed_at: Optional[datetime] = None
    status: ValidationStatus = ValidationStatus.PENDING

    def __post_init__(self):
        if self.actual_results is None:
            self.actual_results = []
        if self.created_at is None:
            self.created_at = datetime.now()

    def validate(self):
        """Validate the validation test data."""
        if not self.query:
            raise ValueError("Query must not be empty")
        if not isinstance(self.validation_type, ValidationType):
            raise ValueError("validation_type must be a ValidationType enum")
        if self.configuration:
            self.configuration.validate()


@dataclass
class ValidationReport:
    """Aggregated results of validation tests including accuracy metrics, reliability measures, and issue identification."""
    test_id: str
    created_at: datetime

    # Metrics and results
    accuracy_metrics: Dict[str, Any] = None
    reliability_metrics: Dict[str, Any] = None
    metadata_integrity_results: Dict[str, Any] = None
    semantic_search_results: Dict[str, Any] = None
    pipeline_reliability_results: Dict[str, Any] = None
    issues_found: List[str] = None
    recommendations: List[str] = None

    # Optional fields
    id: Optional[str] = None

    def __post_init__(self):
        if self.accuracy_metrics is None:
            self.accuracy_metrics = {}
        if self.reliability_metrics is None:
            self.reliability_metrics = {}
        if self.metadata_integrity_results is None:
            self.metadata_integrity_results = {}
        if self.semantic_search_results is None:
            self.semantic_search_results = {}
        if self.pipeline_reliability_results is None:
            self.pipeline_reliability_results = {}
        if self.issues_found is None:
            self.issues_found = []
        if self.recommendations is None:
            self.recommendations = []

    def validate(self):
        """Validate the validation report data."""
        if not self.test_id:
            raise ValueError("test_id must be specified")
        if not self.created_at:
            raise ValueError("created_at must be specified")
        if not self.accuracy_metrics and not self.reliability_metrics:
            raise ValueError("At least one type of validation results must be present")


def is_valid_url(url: str) -> bool:
    """
    Validate if a string is a properly formatted URL.

    Args:
        url: String to validate as URL

    Returns:
        Boolean indicating if URL is valid
    """
    try:
        import re
        result = re.match(
            r'^https?://'  # http:// or https://
            r'(?:(?:[A-Z0-9](?:[A-Z0-9-]{0,61}[A-Z0-9])?\.)+[A-Z]{2,6}\.?|'  # domain...
            r'localhost|'  # localhost...
            r'\d{1,3}\.\d{1,3}\.\d{1,3}\.\d{1,3})'  # ...or ip
            r'(?::\d+)?'  # optional port
            r'(?:/?|[/?]\S+)$', url, re.IGNORECASE)
        return result is not None
    except Exception:
        return False


def validate_search_result_content(content: str) -> bool:
    """
    Validate search result content.

    Args:
        content: Content string to validate

    Returns:
        Boolean indicating if content is valid
    """
    if not content or not isinstance(content, str):
        return False
    if len(content.strip()) == 0:
        return False
    return True


def validate_similarity_score(score: float) -> bool:
    """
    Validate similarity score is within valid range.

    Args:
        score: Similarity score to validate

    Returns:
        Boolean indicating if score is valid
    """
    if not isinstance(score, (int, float)):
        return False
    return 0.0 <= score <= 1.0


def validate_position_values(start: int, end: int) -> bool:
    """
    Validate position values are non-negative and in correct order.

    Args:
        start: Start position
        end: End position

    Returns:
        Boolean indicating if positions are valid
    """
    if not isinstance(start, int) or not isinstance(end, int):
        return False
    if start < 0 or end < 0:
        return False
    if start > end:
        return False
    return True