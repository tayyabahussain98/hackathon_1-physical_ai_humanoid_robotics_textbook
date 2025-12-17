"""
Retrieval validation module for RAG validation system.

This module handles semantic search validation functionality.
"""
import os
from typing import List, Dict, Any
import logging
from dotenv import load_dotenv
from datetime import datetime

# Load environment variables
load_dotenv()

logger = logging.getLogger(__name__)

def validate_semantic_search(query: str, expected_results: List[str], config: dict) -> Dict[str, Any]:
    """
    Validates semantic search accuracy by comparing retrieved results with expected outcomes.

    Args:
        query: The search query string to validate
        expected_results: List of expected relevant results for comparison
        config: Configuration parameters for the search operation

    Returns:
        ValidationReport with accuracy metrics and results
    """
    from .query_processor import process_query
    from .models import ValidationTest, ValidationReport, ValidationType, ValidationStatus
    from .report_generator import generate_validation_report
    import time

    # Create a validation test object
    validation_test = ValidationTest(
        query=query,
        expected_results=expected_results,
        validation_type=ValidationType.SEMANTIC_SEARCH,
        configuration=config
    )

    # Process the query to get actual results
    actual_results = process_query(query, config)
    validation_test.actual_results = actual_results
    validation_test.status = ValidationStatus.RUNNING

    # Calculate accuracy metrics
    accuracy_metrics = calculate_accuracy_metrics(actual_results, expected_results, config)

    # Create validation report
    validation_report = ValidationReport(
        test_id=f"semantic_search_{int(time.time())}",
        created_at=datetime.now(),
        accuracy_metrics=accuracy_metrics,
        semantic_search_results={
            "query": query,
            "expected_results": expected_results,
            "actual_results": actual_results,
            "config_used": config
        }
    )

    validation_test.status = ValidationStatus.COMPLETED

    # Generate and return the report
    return generate_validation_report(validation_report)


def calculate_accuracy_metrics(actual_results: List[Dict], expected_results: List[str], config: dict) -> Dict[str, float]:
    """
    Calculate accuracy metrics for semantic search validation.

    Args:
        actual_results: List of actual search results
        expected_results: List of expected results
        config: Configuration parameters

    Returns:
        Dictionary containing accuracy metrics
    """
    if not expected_results:
        return {"precision": 0.0, "recall": 0.0, "f1_score": 0.0}

    # Extract content from actual results for comparison
    actual_contents = [result.get('content', '') for result in actual_results if result.get('content')]

    # Calculate precision: fraction of retrieved documents that are relevant
    relevant_retrieved = 0
    for content in actual_contents:
        for expected in expected_results:
            if expected.lower() in content.lower():
                relevant_retrieved += 1
                break

    precision = relevant_retrieved / len(actual_contents) if actual_contents else 0.0

    # Calculate recall: fraction of relevant documents that are retrieved
    relevant_total = len(expected_results)
    retrieved_relevant = 0
    for expected in expected_results:
        for content in actual_contents:
            if expected.lower() in content.lower():
                retrieved_relevant += 1
                break

    recall = retrieved_relevant / relevant_total if relevant_total > 0 else 0.0

    # Calculate F1 score
    f1_score = 2 * (precision * recall) / (precision + recall) if (precision + recall) > 0 else 0.0

    # Calculate threshold-based accuracy
    threshold = config.get('threshold', 0.7)
    above_threshold_count = sum(1 for result in actual_results
                                if result.get('similarity_score', 0) >= threshold)
    threshold_accuracy = above_threshold_count / len(actual_results) if actual_results else 0.0

    return {
        "precision": precision,
        "recall": recall,
        "f1_score": f1_score,
        "threshold_accuracy": threshold_accuracy,
        "total_retrieved": len(actual_results),
        "relevant_retrieved": relevant_retrieved,
        "total_expected": len(expected_results)
    }


class ValidationError(Exception):
    """Exception raised when validation fails."""
    pass


class RetrievalError(Exception):
    """Exception raised when there are issues connecting to Qdrant."""
    pass