# RAG System Backend with OpenAI Agents SDK using Gemini

This backend service provides two main functionalities:
1. **Website Ingestion & Vectorization**: Crawls Docusaurus GitHub Pages sites, extracts documentation content, generates semantic embeddings using Cohere, and stores vectors with metadata in Qdrant Cloud
2. **RAG Agent Service**: Provides a RAG-enabled agent using the OpenAI Agents SDK configured to work with Google's Gemini model via OpenAI-compatible API. The system uses FastAPI to expose a clean API endpoint that retrieves relevant book content from Qdrant vector database and injects it into the agent's reasoning flow to ensure responses are grounded only in retrieved content.

## Prerequisites

- Python 3.11+ (required for OpenAI Agents SDK compatibility)
- UV package manager
- Cohere API key (for ingestion functionality)
- Google Gemini API key (for agent functionality)
- Qdrant Cloud account and API key

## Setup

1. **Install dependencies using UV:**
   ```bash
   uv venv  # Create virtual environment
   source .venv/bin/activate  # Activate environment (Linux/Mac)
   # On Windows: source .venv/Scripts/activate
   uv pip install -r requirements.txt
   ```

2. **Configure environment variables:**
   Copy the example environment file and update with your credentials:
   ```bash
   cp .env.example .env
   # Edit .env with your Cohere and Qdrant credentials
   ```

## Usage

### Website Ingestion & Vectorization

Run the ingestion tool with required parameters:

```bash
python main.py --url "https://example.github.io/docs" [options]
```

#### Ingestion Command Line Options

- `--url` (required): Base URL of the Docusaurus site to crawl
- `--max-pages`: Maximum number of pages to crawl (default: 500)
- `--max-depth`: Maximum depth for crawling (default: 3)
- `--chunk-size`: Size of text chunks in characters (default: 1000)
- `--chunk-overlap`: Overlap between chunks in characters (default: 100)
- `--max-retries`: Maximum number of retries for failed requests (default: 3)
- `--log-level`: Logging level (DEBUG, INFO, WARNING, ERROR) (default: INFO)

#### Ingestion Example Usage

```bash
# Basic usage
python main.py --url "https://tayyabahussain98.github.io/hackathon_1-physical_ai_humanoid_robotics_textbook/"

# With custom parameters
python main.py --url "https://example.github.io/docs" --max-pages 100 --chunk-size 2000 --log-level DEBUG
```

### RAG Agent Service

Start the FastAPI server for the RAG agent:

```bash
uvicorn main:app --host 0.0.0.0 --port 8000 --reload
```

#### Agent API Usage

```bash
# Query the RAG agent
curl -X POST http://localhost:8000/query \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What is ROS 2?",
    "top_k": 3,
    "threshold": 0.7
  }'
```

## Configuration

The system uses the following environment variables:

### Ingestion Variables
- `COHERE_API_KEY`: Your Cohere API key (used for both content embedding during ingestion and query embedding for RAG)
- `QDRANT_URL`: Your Qdrant Cloud URL
- `QDRANT_API_KEY`: Your Qdrant API key
- `MAX_PAGES`: Default maximum pages to crawl (default: 500)
- `CRAWL_DEPTH`: Default maximum crawl depth (default: 3)
- `CHUNK_SIZE`: Default chunk size in characters (default: 1000)
- `CHUNK_OVERLAP`: Default chunk overlap in characters (default: 100)
- `MAX_RETRIES`: Default maximum retries for failed requests (default: 3)

### Agent Variables
- `GEMINI_API_KEY`: Your Google Gemini API key
- `GEMINI_MODEL_NAME`: Name of the Gemini model to use (default: gemini-2.5-flash)
- `DEFAULT_TOP_K`: Default number of documents to retrieve (default: 5)
- `DEFAULT_THRESHOLD`: Default similarity threshold (default: 0.7)
- `API_HOST`: Host for the API server (default: 0.0.0.0)
- `API_PORT`: Port for the API server (default: 8000)

## Architecture

The tool follows these steps:

1. **URL Discovery**: Extracts all accessible URLs from the target Docusaurus site using both crawling and sitemap parsing
2. **Content Extraction**: Extracts clean text content from each page, preserving titles and structure
3. **Text Chunking**: Splits content into manageable chunks with configurable size and overlap
4. **Embedding Generation**: Creates semantic embeddings using Cohere's API
5. **Vector Storage**: Stores embeddings with metadata in Qdrant Cloud

## Features

- **Robust Error Handling**: Implements retry mechanisms with exponential backoff for network requests
- **Duplicate Detection**: Prevents storing duplicate content in Qdrant
- **Performance Monitoring**: Tracks execution time for each pipeline stage
- **Configurable Parameters**: Allows customization of crawl depth, chunk size, and other parameters
- **Progress Tracking**: Provides detailed logging of pipeline progress
- **Content Sanitization**: Removes potentially harmful HTML elements from extracted content

## Output

The tool stores vectors in a Qdrant collection named `rag_embedding` with the following metadata:
- `content`: The text content of the chunk
- `source_url`: The URL where the content was found
- `title`: The page title
- `start_pos`: Starting position of the chunk in the original document
- `end_pos`: Ending position of the chunk in the original document