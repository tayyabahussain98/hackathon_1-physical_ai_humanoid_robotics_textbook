# Quickstart Guide: RAG Validation System

## Prerequisites

- Python 3.10+
- Existing RAG ingestion pipeline with embedded content in Qdrant
- Cohere API key
- Qdrant Cloud credentials

## Setup

1. **Install dependencies** (if not already installed from RAG ingestion):
   ```bash
   pip install -r backend/requirements.txt
   ```

2. **Configure environment variables** (if not already configured):
   ```bash
   # Copy from existing .env.example if needed
   export COHERE_API_KEY="your_cohere_api_key"
   export QDRANT_URL="your_qdrant_cloud_url"
   export QDRANT_API_KEY="your_qdrant_api_key"
   ```

## Running Validation Tests

### 1. Semantic Search Validation
Validate the accuracy of semantic search results:

```bash
python backend/main.py --validate-search --query "your search query" --expected "expected topic"
```

### 2. Metadata Integrity Validation
Validate that all embedded content maintains correct metadata:

```bash
python backend/main.py --validate-metadata
```

### 3. End-to-End Pipeline Validation
Run comprehensive pipeline validation:

```bash
python backend/main.py --validate-pipeline --test-set "default"
```

## Configuration Options

- `--top-k`: Number of results to retrieve (default: 5)
- `--threshold`: Minimum similarity score threshold (default: 0.7)
- `--max-concurrent`: Maximum concurrent validation tests (default: 10)
- `--timeout`: Timeout for validation operations in seconds (default: 30)

## Example Usage

```bash
# Validate semantic search with custom parameters
python backend/main.py --validate-search --query "What is humanoid robotics?" --top-k 10 --threshold 0.8

# Run full pipeline validation
python backend/main.py --validate-pipeline --max-concurrent 5
```

## Output

Validation results will be displayed in the console and saved to validation reports in the backend directory. Reports include:
- Accuracy metrics (precision, recall)
- Reliability measures
- Metadata integrity checks
- Identified issues and recommendations