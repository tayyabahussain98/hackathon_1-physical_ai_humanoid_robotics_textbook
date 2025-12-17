# Research Document: Website Ingestion & Vectorization for RAG

**Feature**: 1-website-ingestion-rag
**Created**: 2025-12-14
**Researcher**: Claude Code

## Research Summary

This document consolidates research findings for implementing the Website Ingestion & Vectorization for RAG system.

### Decision: UV Package Manager Setup
**Rationale**: UV is a fast Python package manager written in Rust that offers superior performance compared to pip. For this project, UV will be used to initialize the project, manage dependencies, and create virtual environments.
**Alternatives considered**: pip + venv, Poetry, Pipenv
**Chosen approach**: Using UV for both virtual environment creation and dependency management

### Decision: Docusaurus Crawling Strategy
**Rationale**: Docusaurus sites have predictable navigation structures with sidebar links and category organization. The most effective approach is to start with the base URL, extract all internal links, and recursively crawl the site structure while respecting robots.txt and implementing rate limiting.
**Alternatives considered**: Single-page crawling, JavaScript rendering (Selenium), sitemap parsing
**Chosen approach**: Recursive link extraction with BeautifulSoup, with fallback to requests-html for dynamic content

### Decision: Cohere Embedding API Integration
**Rationale**: Cohere provides high-quality semantic embeddings through a straightforward API. The implementation will use Cohere's Python SDK to generate embeddings, handling authentication via API key and implementing proper error handling for rate limits.
**Alternatives considered**: OpenAI embeddings, Hugging Face models, Sentence Transformers
**Chosen approach**: Cohere API with proper authentication and error handling

### Decision: Qdrant Cloud Vector Storage
**Rationale**: Qdrant Cloud provides managed vector database services with good performance for similarity search operations. The implementation will create a collection named "rag_embedding" to store vectors with associated metadata.
**Alternatives considered**: Pinecone, Weaviate, ChromaDB, FAISS
**Chosen approach**: Qdrant Cloud with dedicated collection for RAG embeddings

## Technical Details

### UV Setup Commands
```bash
uv venv  # Create virtual environment
source .venv/bin/activate  # Activate environment (Linux/Mac)
# or
source .venv/Scripts/activate  # Windows
uv pip install [package-name]  # Install packages
```

### Docusaurus Site Structure
- Navigation typically found in `<nav>` elements or sidebar components
- Links follow patterns like `/docs/category/page` or `/category/page`
- Sitemap may be available at `/sitemap.xml`
- Content usually within `<main>` or `<article>` tags

### Cohere API Requirements
- API key required in environment variable
- Embedding models: multilingual-22-12, embed-multilingual-v2.0, etc.
- Input text must be in batches (max 96 texts per request)
- Output dimensions vary by model (typically 1024 or higher)

### Qdrant Cloud Setup
- Requires URL and API key
- Collections need to be created before storing vectors
- Each point needs an ID, vector, and optional payload (metadata)
- Supports filtering and similarity search operations

## Implementation Considerations

### Error Handling
- Network timeouts and retries
- API rate limits and backoff strategies
- Invalid content or malformed HTML
- Authentication failures

### Performance Optimization
- Parallel processing for embedding generation
- Batch operations for vector storage
- Efficient memory usage during crawling
- Rate limiting to respect target servers

### Security
- API keys stored in environment variables
- Input validation for URLs
- Content sanitization before processing
- Secure connection handling