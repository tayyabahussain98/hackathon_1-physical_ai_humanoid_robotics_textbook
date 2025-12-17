"""
Metadata validation module for RAG validation system.

This module handles metadata integrity validation functionality.
"""
import os
from typing import List, Dict, Any
import logging
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

logger = logging.getLogger(__name__)

def validate_metadata_integrity(results: List[Dict[str, Any]]) -> Dict[str, Any]:
    """
    Validates that all retrieved results maintain correct metadata integrity.

    Args:
        results: List of search results to validate for metadata integrity

    Returns:
        Dictionary with validation results and any issues found
    """
    issues_found = []
    total_results = len(results)
    valid_results = 0
    invalid_results = 0

    # Check each result for metadata integrity
    for i, result in enumerate(results):
        result_issues = []

        # Check if required metadata fields exist
        required_fields = ['content', 'source_url', 'title', 'start_pos', 'end_pos', 'embedding_id']
        for field in required_fields:
            if field not in result:
                result_issues.append(f"Missing required field: {field}")

        # Validate content field
        if 'content' in result:
            content = result['content']
            if not isinstance(content, str) or len(content.strip()) == 0:
                result_issues.append(f"Content is invalid or empty")

        # Validate source_url
        if 'source_url' in result:
            from urllib.parse import urlparse
            source_url = result['source_url']
            if not isinstance(source_url, str) or len(source_url.strip()) == 0:
                result_issues.append(f"Source URL is invalid or empty")
            else:
                try:
                    parsed = urlparse(source_url)
                    if not all([parsed.scheme, parsed.netloc]):
                        result_issues.append(f"Source URL is not properly formatted: {source_url}")
                except Exception:
                    result_issues.append(f"Source URL is not properly formatted: {source_url}")

        # Validate title
        if 'title' in result:
            title = result['title']
            if not isinstance(title, str):
                result_issues.append(f"Title is not a string")

        # Validate position values
        if 'start_pos' in result and 'end_pos' in result:
            start_pos = result['start_pos']
            end_pos = result['end_pos']
            if not isinstance(start_pos, int) or not isinstance(end_pos, int):
                result_issues.append(f"Position values must be integers")
            elif start_pos < 0 or end_pos < 0:
                result_issues.append(f"Position values must be non-negative")
            elif start_pos > end_pos:
                result_issues.append(f"Start position cannot be greater than end position")

        # Validate embedding_id
        if 'embedding_id' in result:
            embedding_id = result['embedding_id']
            if not isinstance(embedding_id, str) or len(embedding_id.strip()) == 0:
                result_issues.append(f"Embedding ID is invalid or empty")

        # Add result-specific issues to the overall list
        if result_issues:
            issues_found.append({
                'result_index': i,
                'result_id': result.get('embedding_id', f'result_{i}'),
                'issues': result_issues
            })
            invalid_results += 1
        else:
            valid_results += 1

    # Calculate integrity metrics
    integrity_percentage = (valid_results / total_results * 100) if total_results > 0 else 100.0

    # Create metadata integrity results
    metadata_integrity_results = {
        "total_results": total_results,
        "valid_results": valid_results,
        "invalid_results": invalid_results,
        "integrity_percentage": integrity_percentage,
        "issues_found": issues_found
    }

    # Overall validation status
    validation_passed = len(issues_found) == 0
    if validation_passed:
        status = "PASS"
    else:
        status = "FAIL"

    return {
        "status": status,
        "validation_passed": validation_passed,
        "metadata_integrity_results": metadata_integrity_results
    }


class MetadataValidationError(Exception):
    """Exception raised when metadata integrity issues are detected."""
    pass