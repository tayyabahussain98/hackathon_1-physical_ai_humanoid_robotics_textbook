# API Contract: RAG Validation System

## Module: validation.retrieval
**Purpose**: Core retrieval and search validation logic

### Function: validate_semantic_search(query: str, expected_results: List[str], config: QueryConfiguration) -> ValidationReport
**Description**: Validates semantic search accuracy by comparing retrieved results with expected outcomes

**Parameters**:
- `query`: The search query string to validate
- `expected_results`: List of expected relevant results for comparison
- `config`: Configuration parameters for the search operation

**Returns**: ValidationReport with accuracy metrics and results

**Errors**:
- `ValidationError`: If query is invalid or required configuration is missing
- `RetrievalError`: If there are issues connecting to Qdrant

## Module: validation.metadata_validator
**Purpose**: Metadata integrity validation

### Function: validate_metadata_integrity(results: List[SearchResult]) -> Dict[str, Any]
**Description**: Validates that all retrieved results maintain correct metadata integrity

**Parameters**:
- `results`: List of search results to validate for metadata integrity

**Returns**: Dictionary with validation results and any issues found

**Errors**:
- `MetadataValidationError`: If metadata integrity issues are detected

## Module: validation.pipeline_validator
**Purpose**: End-to-end pipeline validation

### Function: validate_end_to_end_pipeline(test_config: ValidationConfiguration) -> ValidationReport
**Description**: Validates the complete retrieval pipeline from query input to result delivery

**Parameters**:
- `test_config`: Configuration for the end-to-end validation test

**Returns**: ValidationReport with reliability metrics and results

**Errors**:
- `PipelineValidationError`: If pipeline validation fails

## Module: validation.query_processor
**Purpose**: Query embedding and processing

### Function: process_query(query: str, config: QueryConfiguration) -> List[SearchResult]
**Description**: Processes a query by generating embeddings and searching in Qdrant

**Parameters**:
- `query`: The input query string
- `config`: Configuration parameters for query processing

**Returns**: List of search results from the vector database

**Errors**:
- `QueryProcessingError`: If query processing fails

### Function: generate_query_embedding(query: str) -> List[float]
**Description**: Generates an embedding for the input query using Cohere

**Parameters**:
- `query`: The input query string

**Returns**: Embedding vector as a list of floats

**Errors**:
- `EmbeddingGenerationError`: If embedding generation fails

## Module: validation.report_generator
**Purpose**: Validation report generation

### Function: generate_validation_report(validation_test: ValidationTest) -> ValidationReport
**Description**: Generates a comprehensive validation report from a validation test

**Parameters**:
- `validation_test`: The completed validation test to generate a report for

**Returns**: ValidationReport with all metrics and results

**Errors**:
- `ReportGenerationError`: If report generation fails