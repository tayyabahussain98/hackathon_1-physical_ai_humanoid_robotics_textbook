# Data Model: RAG Retrieval & Pipeline Validation

## Entity: Validation Test
**Description**: Represents a validation run that includes query inputs, expected outcomes, and actual results

**Fields**:
- `id`: Unique identifier for the validation test
- `query`: The input query string to validate against the RAG system
- `expected_results`: List of expected relevant content or topics
- `actual_results`: List of actual retrieved results from the RAG system
- `validation_type`: Type of validation (SEMANTIC_SEARCH, METADATA_INTEGRITY, END_TO_END)
- `created_at`: Timestamp when the test was created
- `completed_at`: Timestamp when the test was completed
- `status`: Current status (PENDING, RUNNING, COMPLETED, FAILED)
- `configuration`: Configuration parameters used for the validation

**Validation Rules**:
- Query must not be empty
- Validation type must be one of the defined enum values
- Status must follow valid state transitions

## Entity: Search Result
**Description**: Contains retrieved content chunks with similarity scores, metadata, and provenance information

**Fields**:
- `content`: The actual content text retrieved from the vector database
- `similarity_score`: Float value representing the similarity to the query (0.0 to 1.0)
- `metadata`: Dictionary containing source information (URL, title, etc.)
- `source_url`: URL where the original content was found
- `title`: Title of the source document
- `start_pos`: Starting position of the chunk in the original document
- `end_pos`: Ending position of the chunk in the original document
- `embedding_id`: ID of the vector in the Qdrant collection

**Validation Rules**:
- Content must not be empty
- Similarity score must be between 0.0 and 1.0
- Source URL must be a valid URL format

## Entity: Validation Report
**Description**: Aggregated results of validation tests including accuracy metrics, reliability measures, and issue identification

**Fields**:
- `id`: Unique identifier for the validation report
- `test_id`: Reference to the validation test that generated this report
- `accuracy_metrics`: Dictionary of accuracy measurements (precision, recall, etc.)
- `reliability_metrics`: Dictionary of reliability measurements (success rate, response times, etc.)
- `metadata_integrity_results`: Results of metadata validation checks
- `semantic_search_results`: Results of semantic search validation
- `pipeline_reliability_results`: Results of end-to-end pipeline validation
- `issues_found`: List of identified issues during validation
- `recommendations`: List of recommendations for improvement
- `created_at`: Timestamp when the report was generated

**Validation Rules**:
- Test ID must reference an existing validation test
- Accuracy metrics must contain valid numerical values
- At least one type of validation results must be present

## Entity: Query Configuration
**Description**: Configuration parameters for query processing and validation

**Fields**:
- `top_k`: Number of results to retrieve from the vector database
- `threshold`: Minimum similarity score for considering results relevant
- `max_concurrent_tests`: Maximum number of concurrent validation tests
- `timeout_seconds`: Timeout for individual validation operations
- `validation_thresholds`: Dictionary of thresholds for different validation metrics

**Validation Rules**:
- Top-k must be a positive integer
- Threshold must be between 0.0 and 1.0
- Max concurrent tests must be between 1 and 100

## Entity: Validation Configuration
**Description**: Overall configuration for the validation system

**Fields**:
- `cohere_model`: Name of the Cohere model to use for query embeddings
- `qdrant_collection`: Name of the Qdrant collection to validate against
- `default_query_config`: Default query configuration parameters
- `validation_rules`: Rules defining how validation should be performed
- `report_format`: Format for generating validation reports

**Validation Rules**:
- Cohere model must be a valid model name
- Qdrant collection must exist
- Default query config must be a valid Query Configuration object