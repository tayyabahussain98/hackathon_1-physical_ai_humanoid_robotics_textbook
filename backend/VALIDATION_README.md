# RAG Validation System

This document explains how to use the RAG (Retrieval-Augmented Generation) validation system to validate semantic search accuracy, metadata integrity, and end-to-end pipeline reliability.

## Overview

The validation system provides three main types of validation:
1. **Semantic Search Validation**: Validates the accuracy of semantic search results
2. **Metadata Integrity Validation**: Validates that metadata remains intact and accurate
3. **End-to-End Pipeline Validation**: Validates the complete retrieval pipeline from query to results

## Prerequisites

- Python 3.10+
- Valid Cohere API key in `.env` file
- Valid Qdrant Cloud URL and API key in `.env` file
- Previously ingested content in Qdrant collection named `rag_embedding`

## Setup

1. Ensure your environment variables are set in `.env`:
   ```
   COHERE_API_KEY=your_cohere_api_key
   QDRANT_URL=your_qdrant_cloud_url
   QDRANT_API_KEY=your_qdrant_api_key
   ```

2. Install dependencies:
   ```bash
   pip install -r requirements.txt
   ```

## Usage

### Semantic Search Validation

Validate the accuracy of semantic search results against expected outcomes:

```bash
python main.py --validate-search --query "What is RAG?" --expected "Retrieval Augmented Generation, technique that combines search with generation"
```

Additional options:
- `--top-k 5`: Number of results to retrieve (default: 5)
- `--threshold 0.7`: Minimum similarity score threshold (default: 0.7)
- `--expected`: Expected results for comparison (comma-separated)

### Metadata Integrity Validation

Validate that all retrieved content maintains correct metadata integrity:

```bash
python main.py --validate-metadata --query "What is vector database?"
```

This validates source URLs, titles, and content boundaries for retrieved results.

### End-to-End Pipeline Validation

Validate the complete retrieval pipeline reliability:

```bash
python main.py --validate-pipeline --query "How does semantic search work?"
```

Additional options:
- `--max-concurrent 10`: Maximum concurrent validation tests (default: 10)
- `--top-k 5`: Number of results to retrieve (default: 5)
- `--threshold 0.7`: Minimum similarity score threshold (default: 0.7)

### Combined Validation

You can run multiple validation types in sequence:

```bash
# Run semantic search validation first
python main.py --validate-search --query "What is machine learning?" --top-k 10

# Then run metadata validation
python main.py --validate-metadata --query "Explain neural networks"

# Finally run end-to-end validation
python main.py --validate-pipeline --query "What are transformer models?"
```

## Configuration Parameters

- `--top-k`: Number of results to retrieve (default: 5)
- `--threshold`: Minimum similarity score threshold (default: 0.7)
- `--max-concurrent`: Maximum concurrent validation tests (default: 10)
- `--log-level`: Logging level (DEBUG, INFO, WARNING, ERROR) (default: INFO)

## Output

The validation system generates comprehensive reports including:

- **Accuracy Metrics**: Precision, recall, F1-score for semantic search
- **Reliability Metrics**: Success rate, response times, error rates
- **Metadata Integrity Results**: Percentage of valid metadata fields
- **Issues Found**: Specific problems identified during validation
- **Recommendations**: Suggestions for improving validation performance

## Examples

### Basic Semantic Search Validation
```bash
python main.py --validate-search --query "Explain quantum computing" --expected "quantum mechanics, qubits, superposition, entanglement"
```

### Metadata Validation with Custom Threshold
```bash
python main.py --validate-metadata --query "What is blockchain?" --threshold 0.6
```

### Pipeline Validation with Multiple Queries
```bash
python main.py --validate-pipeline --max-concurrent 5 --top-k 8
```

## Troubleshooting

- **API Key Issues**: Ensure your Cohere and Qdrant API keys are valid and properly set in the environment
- **Connection Issues**: Verify your Qdrant URL is accessible
- **Empty Results**: Check that you have previously ingested content in the `rag_embedding` collection
- **Low Accuracy**: Consider adjusting the similarity threshold or reviewing your content quality

## Performance Considerations

- Large validation sets may take significant time to complete
- Concurrent validation tests are limited to 10 by default to prevent overloading APIs
- Monitor API usage to stay within rate limits