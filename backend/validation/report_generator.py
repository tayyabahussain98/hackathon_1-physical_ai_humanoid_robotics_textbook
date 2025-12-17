"""
Report generation module for RAG validation system.

This module handles validation report generation functionality.
"""
import os
from typing import Dict, Any
import logging
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

logger = logging.getLogger(__name__)

def generate_validation_report(validation_test: Dict[str, Any]) -> Dict[str, Any]:
    """
    Generates a comprehensive validation report from a validation test.

    Args:
        validation_test: The completed validation test to generate a report for

    Returns:
        ValidationReport with all metrics and results
    """
    from datetime import datetime
    from .models import ValidationReport, ValidationType

    # Create a validation report based on the test results
    # If validation_test is already a ValidationReport object, use its attributes
    if hasattr(validation_test, 'test_id'):
        test_id = validation_test.test_id
        accuracy_metrics = getattr(validation_test, 'accuracy_metrics', {})
        reliability_metrics = getattr(validation_test, 'reliability_metrics', {})
        metadata_integrity_results = getattr(validation_test, 'metadata_integrity_results', {})
        semantic_search_results = getattr(validation_test, 'semantic_search_results', {})
        pipeline_reliability_results = getattr(validation_test, 'pipeline_reliability_results', {})
    else:
        # If it's a ValidationTest object, extract values differently
        test_id = getattr(validation_test, 'id', f"test_{int(datetime.now().timestamp())}")
        accuracy_metrics = getattr(validation_test, 'accuracy_metrics', {})
        reliability_metrics = getattr(validation_test, 'reliability_metrics', {})
        metadata_integrity_results = getattr(validation_test, 'metadata_integrity_results', {})
        semantic_search_results = getattr(validation_test, 'semantic_search_results', {})
        pipeline_reliability_results = getattr(validation_test, 'pipeline_reliability_results', {})

    # Generate recommendations based on results
    recommendations = generate_recommendations(
        accuracy_metrics,
        reliability_metrics,
        metadata_integrity_results,
        semantic_search_results,
        pipeline_reliability_results
    )

    # Identify issues based on results
    issues_found = identify_issues(
        accuracy_metrics,
        reliability_metrics,
        metadata_integrity_results,
        semantic_search_results,
        pipeline_reliability_results
    )

    # Create the report object
    report = ValidationReport(
        test_id=test_id,
        created_at=datetime.now(),
        accuracy_metrics=accuracy_metrics,
        reliability_metrics=reliability_metrics,
        metadata_integrity_results=metadata_integrity_results,
        semantic_search_results=semantic_search_results,
        pipeline_reliability_results=pipeline_reliability_results,
        issues_found=issues_found,
        recommendations=recommendations
    )

    # Validate the report before returning
    report.validate()

    # Convert to dictionary for return
    return {
        'test_id': report.test_id,
        'created_at': report.created_at.isoformat(),
        'accuracy_metrics': report.accuracy_metrics,
        'reliability_metrics': report.reliability_metrics,
        'metadata_integrity_results': report.metadata_integrity_results,
        'semantic_search_results': report.semantic_search_results,
        'pipeline_reliability_results': report.pipeline_reliability_results,
        'issues_found': report.issues_found,
        'recommendations': report.recommendations,
        'summary': generate_report_summary(report)
    }


def generate_recommendations(accuracy_metrics, reliability_metrics, metadata_integrity_results, semantic_search_results, pipeline_reliability_results):
    """
    Generate recommendations based on validation results.

    Args:
        accuracy_metrics: Accuracy metrics from validation
        reliability_metrics: Reliability metrics from validation
        metadata_integrity_results: Metadata integrity results
        semantic_search_results: Semantic search results
        pipeline_reliability_results: Pipeline reliability results

    Returns:
        List of recommendations
    """
    recommendations = []

    # Accuracy-based recommendations
    if 'precision' in accuracy_metrics:
        precision = accuracy_metrics['precision']
        if precision < 0.7:
            recommendations.append("Precision is below 70%, consider improving query formulation or adjusting similarity thresholds")
        elif precision > 0.95:
            recommendations.append("High precision achieved, consider if recall could be improved by adjusting thresholds")

    if 'recall' in accuracy_metrics:
        recall = accuracy_metrics['recall']
        if recall < 0.6:
            recommendations.append("Recall is below 60%, consider lowering similarity thresholds to retrieve more relevant results")

    # Reliability-based recommendations
    if 'success_rate' in reliability_metrics:
        success_rate = reliability_metrics['success_rate']
        if success_rate < 0.9:
            recommendations.append("Success rate is below 90%, investigate API limits, network connectivity, or system resources")

    if 'avg_response_time' in reliability_metrics:
        avg_time = reliability_metrics['avg_response_time']
        if avg_time > 5.0:  # More than 5 seconds
            recommendations.append("Average response time exceeds 5 seconds, consider optimizing queries or upgrading infrastructure")

    # Metadata integrity recommendations
    if 'integrity_percentage' in metadata_integrity_results:
        integrity_pct = metadata_integrity_results['integrity_percentage']
        if integrity_pct < 95:
            recommendations.append("Metadata integrity below 95%, check data ingestion process for missing or malformed metadata")

    if not recommendations:
        recommendations.append("All validation metrics are within acceptable ranges")

    return recommendations


def identify_issues(accuracy_metrics, reliability_metrics, metadata_integrity_results, semantic_search_results, pipeline_reliability_results):
    """
    Identify issues based on validation results.

    Args:
        accuracy_metrics: Accuracy metrics from validation
        reliability_metrics: Reliability metrics from validation
        metadata_integrity_results: Metadata integrity results
        semantic_search_results: Semantic search results
        pipeline_reliability_results: Pipeline reliability results

    Returns:
        List of identified issues
    """
    issues = []

    # Check for accuracy issues
    if 'precision' in accuracy_metrics and accuracy_metrics['precision'] < 0.5:
        issues.append(f"Low precision: {accuracy_metrics['precision']:.2f} - search results may not be relevant")

    if 'recall' in accuracy_metrics and accuracy_metrics['recall'] < 0.5:
        issues.append(f"Low recall: {accuracy_metrics['recall']:.2f} - relevant results may be missed")

    # Check for reliability issues
    if 'success_rate' in reliability_metrics and reliability_metrics['success_rate'] < 0.8:
        issues.append(f"Low success rate: {reliability_metrics['success_rate']:.2f} - pipeline is unreliable")

    # Check for performance issues
    if 'avg_response_time' in reliability_metrics and reliability_metrics['avg_response_time'] > 10.0:
        issues.append(f"High average response time: {reliability_metrics['avg_response_time']:.2f}s - performance issue")

    # Check for metadata issues
    if 'integrity_percentage' in metadata_integrity_results and metadata_integrity_results['integrity_percentage'] < 90:
        issues.append(f"Low metadata integrity: {metadata_integrity_results['integrity_percentage']:.2f}% - data quality issue")

    return issues


def generate_report_summary(report: 'ValidationReport') -> dict:
    """
    Generate a summary of the validation report.

    Args:
        report: The validation report to summarize

    Returns:
        Dictionary containing the report summary
    """
    summary = {
        'status': 'PASS' if len(report.issues_found) == 0 else 'FAIL',
        'total_issues': len(report.issues_found),
        'total_recommendations': len(report.recommendations),
        'validation_type': 'unknown'  # This would be determined based on which results are populated
    }

    # Determine validation type based on populated results
    if report.semantic_search_results:
        summary['validation_type'] = 'semantic_search'
    elif report.metadata_integrity_results:
        summary['validation_type'] = 'metadata_integrity'
    elif report.pipeline_reliability_results:
        summary['validation_type'] = 'pipeline_reliability'

    # Add key metrics to summary
    if report.accuracy_metrics:
        summary['key_accuracy_metrics'] = {
            'precision': report.accuracy_metrics.get('precision'),
            'recall': report.accuracy_metrics.get('recall'),
            'f1_score': report.accuracy_metrics.get('f1_score')
        }

    if report.reliability_metrics:
        summary['key_reliability_metrics'] = {
            'success_rate': report.reliability_metrics.get('success_rate'),
            'avg_response_time': report.reliability_metrics.get('avg_response_time')
        }

    return summary


class ReportGenerationError(Exception):
    """Exception raised when report generation fails."""
    pass