# Quickstart Guide: Website Ingestion & Vectorization for RAG

**Feature**: 1-website-ingestion-rag
**Created**: 2025-12-14
**Guide Version**: 1.0

## Prerequisites

- Python 3.10 or higher
- UV package manager installed
- Cohere API key
- Qdrant Cloud account and API key

## Setup Instructions

### 1. Clone or Create Project Directory
```bash
mkdir backend
cd backend
```

### 2. Initialize Project with UV
```bash
uv venv  # Create virtual environment
source .venv/bin/activate  # Activate environment (Linux/Mac)
# On Windows: source .venv/Scripts/activate
```

### 3. Install Dependencies
```bash
uv pip install requests beautifulsoup4 cohere qdrant-client python-dotenv
```

### 4. Create Environment File
Create a `.env` file in the backend directory with the following content:
```env
COHERE_API_KEY=your_cohere_api_key_here
QDRANT_URL=your_qdrant_cloud_url_here
QDRANT_API_KEY=your_qdrant_api_key_here
```

## Running the Ingestion Pipeline

### 1. Place main.py in the backend directory

### 2. Execute the pipeline
```bash
python main.py
```

## Configuration Options

The system can be configured through environment variables:

- `COHERE_API_KEY`: API key for Cohere embeddings service
- `QDRANT_URL`: URL for Qdrant Cloud instance
- `QDRANT_API_KEY`: API key for Qdrant Cloud
- `CRAWL_DEPTH`: Maximum depth for crawling (default: 3)
- `MAX_PAGES`: Maximum number of pages to crawl (default: 500)
- `CHUNK_SIZE`: Size of text chunks in characters (default: 1000)
- `CHUNK_OVERLAP`: Overlap between chunks in characters (default: 100)

## Expected Output

When running the pipeline:
1. The system will crawl the specified Docusaurus site
2. Extract and clean text content from all pages
3. Chunk the content into manageable pieces
4. Generate embeddings using Cohere
5. Store the embeddings in Qdrant Cloud
6. Display progress and final statistics

## Troubleshooting

### Common Issues:

1. **API Authentication Errors**:
   - Verify API keys in the `.env` file
   - Check for typos in the key values

2. **Crawling Rate Limits**:
   - The system implements rate limiting automatically
   - Check if the target site has specific crawling policies

3. **Memory Issues with Large Sites**:
   - Reduce the `MAX_PAGES` configuration
   - Process sites in smaller batches

4. **Qdrant Connection Issues**:
   - Verify the QDRANT_URL is correct
   - Check that the API key has proper permissions