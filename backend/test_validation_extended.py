"""
Extended unit and integration tests for RAG validation features.
"""
import unittest
from unittest.mock import Mock, patch, MagicMock, MagicMock
import os
from validation.query_processor import generate_query_embedding, process_query
from validation.retrieval import validate_semantic_search, calculate_accuracy_metrics
from validation.metadata_validator import validate_metadata_integrity
from validation.pipeline_validator import validate_end_to_end_pipeline, calculate_percentile
from validation.report_generator import generate_validation_report, generate_recommendations, identify_issues
from validation.models import ValidationTest, ValidationReport, ValidationType, QueryConfiguration
from validation.error_handling import validate_api_keys, retry_with_exponential_backoff, handle_validation_errors


class TestQueryProcessor(unittest.TestCase):
    """Test cases for query processing functionality."""

    @patch.dict(os.environ, {'COHERE_API_KEY': 'test-key'})
    @patch('cohere.Client')
    def test_generate_query_embedding_success(self, mock_cohere_client):
        """Test successful query embedding generation."""
        # Setup mock
        mock_response = Mock()
        mock_response.embeddings = [[0.1, 0.2, 0.3]]
        mock_cohere_client.return_value.embed.return_value = mock_response

        # Test
        result = generate_query_embedding("test query")

        # Verify
        self.assertEqual(result, [0.1, 0.2, 0.3])
        mock_cohere_client.return_value.embed.assert_called_once()

    @patch.dict(os.environ, {}, clear=True)
    def test_generate_query_embedding_no_api_key(self):
        """Test query embedding generation without API key."""
        with self.assertRaises(ValueError):
            generate_query_embedding("test query")

    @patch.dict(os.environ, {'COHERE_API_KEY': 'test-key'})
    @patch('validation.query_processor.generate_query_embedding')
    @patch('validation.qdrant_utils.get_qdrant_client')
    @patch('validation.qdrant_utils.search_in_qdrant')
    def test_process_query(self, mock_search_in_qdrant, mock_get_qdrant_client, mock_generate_embedding):
        """Test processing a query with mocked dependencies."""
        # Setup mocks
        mock_generate_embedding.return_value = [0.1, 0.2, 0.3]
        mock_client = Mock()
        mock_get_qdrant_client.return_value = mock_client
        mock_search_in_qdrant.return_value = [
            {
                'content': 'test content',
                'similarity_score': 0.9,
                'metadata': {},
                'source_url': 'https://example.com',
                'title': 'Test Title',
                'start_pos': 0,
                'end_pos': 100,
                'embedding_id': 'test-id'
            }
        ]

        # Test
        config = {
            'top_k': 5,
            'threshold': 0.7,
            'qdrant_collection': 'rag_embedding'
        }
        result = process_query("test query", config)

        # Verify
        self.assertEqual(len(result), 1)
        self.assertEqual(result[0]['content'], 'test content')
        self.assertEqual(result[0]['similarity_score'], 0.9)


class TestRetrievalValidation(unittest.TestCase):
    """Test cases for retrieval validation functionality."""

    def test_calculate_accuracy_metrics_basic(self):
        """Test basic accuracy metrics calculation."""
        actual_results = [
            {'content': 'relevant content', 'similarity_score': 0.9},
            {'content': 'somewhat relevant', 'similarity_score': 0.7}
        ]
        expected_results = ['relevant content', 'somewhat relevant']

        config = {'threshold': 0.8}
        metrics = calculate_accuracy_metrics(actual_results, expected_results, config)

        self.assertGreaterEqual(metrics['precision'], 0.0)
        self.assertGreaterEqual(metrics['recall'], 0.0)
        self.assertGreaterEqual(metrics['f1_score'], 0.0)

    def test_calculate_accuracy_metrics_empty_expected(self):
        """Test accuracy metrics calculation with empty expected results."""
        actual_results = [{'content': 'test', 'similarity_score': 0.9}]
        expected_results = []

        metrics = calculate_accuracy_metrics(actual_results, expected_results, {})

        self.assertEqual(metrics['precision'], 0.0)
        self.assertEqual(metrics['recall'], 0.0)
        self.assertEqual(metrics['f1_score'], 0.0)

    @patch('validation.retrieval.process_query')
    def test_validate_semantic_search(self, mock_process_query):
        """Test semantic search validation with mocked query processing."""
        # Setup mock
        mock_process_query.return_value = [
            {
                'content': 'relevant content',
                'similarity_score': 0.9,
                'metadata': {},
                'source_url': 'https://example.com',
                'title': 'Test Title',
                'start_pos': 0,
                'end_pos': 100,
                'embedding_id': 'test-id'
            }
        ]

        # Test
        config = {'top_k': 5, 'threshold': 0.7, 'qdrant_collection': 'rag_embedding'}
        result = validate_semantic_search("test query", ["relevant content"], config)

        # Verify
        self.assertIsNotNone(result)
        self.assertIn('accuracy_metrics', result)


class TestMetadataValidation(unittest.TestCase):
    """Test cases for metadata validation functionality."""

    def test_validate_metadata_integrity_complete(self):
        """Test metadata integrity validation with complete data."""
        results = [
            {
                'content': 'test content',
                'source_url': 'https://example.com/test',
                'title': 'Test Title',
                'start_pos': 0,
                'end_pos': 100,
                'embedding_id': 'test-id'
            }
        ]

        result = validate_metadata_integrity(results)

        self.assertTrue(result['validation_passed'])
        self.assertEqual(result['status'], 'PASS')
        self.assertEqual(result['metadata_integrity_results']['valid_results'], 1)

    def test_validate_metadata_integrity_missing_fields(self):
        """Test metadata integrity validation with missing fields."""
        results = [
            {
                'content': 'test content',
                # Missing required fields
            }
        ]

        result = validate_metadata_integrity(results)

        self.assertFalse(result['validation_passed'])
        self.assertEqual(result['status'], 'FAIL')
        self.assertEqual(result['metadata_integrity_results']['invalid_results'], 1)
        self.assertGreater(len(result['metadata_integrity_results']['issues_found']), 0)

    def test_validate_metadata_integrity_invalid_url(self):
        """Test metadata integrity validation with invalid URL."""
        results = [
            {
                'content': 'test content',
                'source_url': 'not-a-valid-url',
                'title': 'Test Title',
                'start_pos': 0,
                'end_pos': 100,
                'embedding_id': 'test-id'
            }
        ]

        result = validate_metadata_integrity(results)

        self.assertFalse(result['validation_passed'])
        self.assertIn('issues_found', result['metadata_integrity_results'])


class TestPipelineValidation(unittest.TestCase):
    """Test cases for pipeline validation functionality."""

    def test_calculate_percentile(self):
        """Test percentile calculation."""
        data = [1, 2, 3, 4, 5]

        p50 = calculate_percentile(data, 0.5)
        self.assertEqual(p50, 3)  # Median of 1-5 is 3

        p90 = calculate_percentile(data, 0.9)
        self.assertEqual(p90, 5)  # 90th percentile of 1-5 is 5

        empty_data = []
        p50_empty = calculate_percentile(empty_data, 0.5)
        self.assertEqual(p50_empty, 0)  # Should return 0 for empty data

    @patch('validation.pipeline_validator.process_query')
    def test_validate_end_to_end_pipeline(self, mock_process_query):
        """Test end-to-end pipeline validation."""
        # Setup mock
        mock_process_query.return_value = [
            {
                'content': 'test content',
                'similarity_score': 0.9,
                'metadata': {},
                'source_url': 'https://example.com',
                'title': 'Test Title',
                'start_pos': 0,
                'end_pos': 100,
                'embedding_id': 'test-id'
            }
        ]

        # Test
        test_config = {
            'queries': ['test query'],
            'max_concurrent_tests': 1,
            'timeout_seconds': 30,
            'expected_accuracy_threshold': 0.85,
            'top_k': 5,
            'threshold': 0.7,
            'qdrant_collection': 'rag_embedding'
        }

        result = validate_end_to_end_pipeline(test_config)

        # Verify
        self.assertIsNotNone(result)
        self.assertIn('pipeline_reliability_results', result)


class TestReportGeneration(unittest.TestCase):
    """Test cases for report generation functionality."""

    def test_generate_recommendations(self):
        """Test recommendation generation based on metrics."""
        accuracy_metrics = {'precision': 0.5}  # Low precision
        reliability_metrics = {}
        metadata_integrity_results = {}
        semantic_search_results = {}
        pipeline_reliability_results = {}

        recommendations = generate_recommendations(
            accuracy_metrics, reliability_metrics, metadata_integrity_results,
            semantic_search_results, pipeline_reliability_results
        )

        # Should have recommendations for low precision
        self.assertTrue(any('precision' in rec.lower() for rec in recommendations))

    def test_identify_issues(self):
        """Test issue identification based on metrics."""
        accuracy_metrics = {'precision': 0.3, 'recall': 0.2}  # Very low
        reliability_metrics = {}
        metadata_integrity_results = {}
        semantic_search_results = {}
        pipeline_reliability_results = {}

        issues = identify_issues(
            accuracy_metrics, reliability_metrics, metadata_integrity_results,
            semantic_search_results, pipeline_reliability_results
        )

        # Should identify issues with low precision and recall
        self.assertTrue(len(issues) > 0)
        self.assertTrue(any('precision' in issue.lower() for issue in issues))
        self.assertTrue(any('recall' in issue.lower() for issue in issues))

    def test_generate_validation_report(self):
        """Test validation report generation."""
        validation_test = {
            'id': 'test-report-1',
            'validation_type': 'SEMANTIC_SEARCH',
            'accuracy_metrics': {'precision': 0.8, 'recall': 0.75},
            'reliability_metrics': {'success_rate': 0.95}
        }

        report = generate_validation_report(validation_test)

        # Verify report structure
        self.assertIn('test_id', report)
        self.assertIn('accuracy_metrics', report)
        self.assertIn('reliability_metrics', report)
        self.assertIn('summary', report)


class TestErrorHandling(unittest.TestCase):
    """Test cases for error handling functionality."""

    @patch.dict(os.environ, {'COHERE_API_KEY': 'test-key', 'QDRANT_URL': 'https://test.com', 'QDRANT_API_KEY': 'test-key'})
    def test_validate_api_keys_success(self):
        """Test successful API key validation."""
        # This should not raise an exception
        validate_api_keys()

    @patch.dict(os.environ, {}, clear=True)
    def test_validate_api_keys_missing(self):
        """Test API key validation with missing keys."""
        with self.assertRaises(ValueError):
            validate_api_keys()


if __name__ == '__main__':
    unittest.main()