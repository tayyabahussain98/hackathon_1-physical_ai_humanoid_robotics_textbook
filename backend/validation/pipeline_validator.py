"""
Pipeline validation module for RAG validation system.

This module handles end-to-end pipeline validation functionality.
"""
import os
from typing import Dict, Any
import logging
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

logger = logging.getLogger(__name__)

def validate_end_to_end_pipeline(test_config: dict) -> Dict[str, Any]:
    """
    Validates the complete retrieval pipeline from query input to result delivery.

    Args:
        test_config: Configuration for the end-to-end validation test

    Returns:
        ValidationReport with reliability metrics and results
    """
    import time
    import threading
    from concurrent.futures import ThreadPoolExecutor, as_completed
    from typing import Callable

    from .query_processor import process_query
    from .models import ValidationTest, ValidationReport, ValidationType, ValidationStatus
    from .report_generator import generate_validation_report

    # Extract configuration parameters
    queries = test_config.get('queries', [])
    max_concurrent = test_config.get('max_concurrent_tests', 10)
    timeout_seconds = test_config.get('timeout_seconds', 30)
    expected_accuracy_threshold = test_config.get('expected_accuracy_threshold', 0.85)

    # Initialize metrics
    total_tests = len(queries)
    successful_tests = 0
    failed_tests = 0
    total_time = 0
    response_times = []
    errors = []

    def run_single_test(query: str) -> dict:
        """Run a single end-to-end test and return results."""
        start_time = time.time()
        test_result = {
            'query': query,
            'success': False,
            'error': None,
            'response_time': 0,
            'results_count': 0
        }

        try:
            # Process the query
            config = {
                'top_k': test_config.get('top_k', 5),
                'threshold': test_config.get('threshold', 0.7),
                'qdrant_collection': test_config.get('qdrant_collection', 'rag_embedding')
            }

            results = process_query(query, config)
            response_time = time.time() - start_time

            test_result['success'] = True
            test_result['response_time'] = response_time
            test_result['results_count'] = len(results)
            test_result['results'] = results

            return test_result

        except Exception as e:
            response_time = time.time() - start_time
            test_result['error'] = str(e)
            test_result['response_time'] = response_time
            return test_result

    # Run tests concurrently based on max_concurrent setting
    with ThreadPoolExecutor(max_workers=max_concurrent) as executor:
        # Submit all test tasks
        future_to_query = {executor.submit(run_single_test, query): query for query in queries}

        # Collect results as they complete
        for future in as_completed(future_to_query):
            test_result = future.result()
            response_times.append(test_result['response_time'])

            if test_result['success']:
                successful_tests += 1
            else:
                failed_tests += 1
                errors.append({
                    'query': test_result['query'],
                    'error': test_result['error']
                })

    # Calculate reliability metrics
    success_rate = successful_tests / total_tests if total_tests > 0 else 0
    avg_response_time = sum(response_times) / len(response_times) if response_times else 0
    p95_response_time = calculate_percentile(response_times, 0.95) if response_times else 0
    total_execution_time = sum(response_times)

    # Determine if pipeline meets reliability criteria
    meets_threshold = success_rate >= expected_accuracy_threshold

    # Create pipeline reliability results
    pipeline_reliability_results = {
        "total_tests": total_tests,
        "successful_tests": successful_tests,
        "failed_tests": failed_tests,
        "success_rate": success_rate,
        "meets_threshold": meets_threshold,
        "expected_threshold": expected_accuracy_threshold,
        "avg_response_time": avg_response_time,
        "p95_response_time": p95_response_time,
        "total_execution_time": total_execution_time,
        "response_times": response_times,
        "errors": errors
    }

    # Create a validation test object
    validation_test = ValidationTest(
        query="End-to-End Pipeline Test",
        expected_results=[f"Success rate >= {expected_accuracy_threshold}"],
        validation_type=ValidationType.END_TO_END,
        configuration=test_config
    )

    # Create validation report
    from datetime import datetime
    validation_report = ValidationReport(
        test_id=f"pipeline_validation_{int(time.time())}",
        created_at=datetime.now(),
        reliability_metrics={
            "success_rate": success_rate,
            "avg_response_time": avg_response_time,
            "p95_response_time": p95_response_time,
            "total_execution_time": total_execution_time
        },
        pipeline_reliability_results=pipeline_reliability_results
    )

    validation_test.status = ValidationStatus.COMPLETED

    # Generate and return the report
    return generate_validation_report(validation_report)


def calculate_percentile(data: list, percentile: float) -> float:
    """
    Calculate the specified percentile of a list of values.

    Args:
        data: List of numeric values
        percentile: Percentile to calculate (e.g., 0.95 for 95th percentile)

    Returns:
        The calculated percentile value
    """
    if not data:
        return 0

    sorted_data = sorted(data)
    index = int(percentile * len(sorted_data))
    if index >= len(sorted_data):
        index = len(sorted_data) - 1

    return sorted_data[index]


class PipelineValidationError(Exception):
    """Exception raised when pipeline validation fails."""
    pass