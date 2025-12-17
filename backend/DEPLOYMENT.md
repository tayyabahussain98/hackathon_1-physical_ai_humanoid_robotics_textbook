# Deployment Guide: Website Ingestion & Vectorization for RAG

## Prerequisites

### Infrastructure Requirements
- Access to Qdrant Cloud (or self-hosted Qdrant instance)
- Cohere API access
- Python 3.10+ runtime environment
- Sufficient memory for processing large documents (recommended: 4GB+)

### Environment Setup
- Create a dedicated environment for the RAG ingestion pipeline
- Ensure outbound network access to target Docusaurus sites, Cohere API, and Qdrant Cloud

## Deployment Steps

### 1. Clone and Setup Repository
```bash
# Clone your repository
git clone <your-repo-url>
cd backend

# Create virtual environment
uv venv
source .venv/bin/activate  # On Windows: source .venv/Scripts/activate

# Install dependencies
uv pip install -r requirements.txt
```

### 2. Environment Configuration
Create a `.env` file with your credentials:
```bash
COHERE_API_KEY=your_cohere_api_key
QDRANT_URL=your_qdrant_cloud_url
QDRANT_API_KEY=your_qdrant_api_key
```

### 3. Security Configuration
- Store API keys securely using environment variables or a secrets manager
- Ensure the environment has appropriate network security rules
- Use read-only API keys where possible

### 4. Resource Allocation
- **CPU**: 2-4 cores recommended for optimal performance
- **Memory**: 4GB minimum, 8GB recommended for large documents
- **Storage**: Minimal local storage needed, primarily for temporary processing
- **Network**: Ensure stable connectivity to target sites and APIs

## Running the Pipeline

### Single Execution
```bash
python main.py --url "https://example.github.io/docs" --max-pages 500
```

### Scheduled Execution
For regular updates, set up a cron job or scheduled task:
```bash
# Example cron job to run weekly
0 2 * * 0 cd /path/to/backend && source .venv/bin/activate && python main.py --url "https://example.github.io/docs"
```

### Monitoring and Logging
- Monitor application logs for errors and performance metrics
- Set up alerts for failed API calls or connectivity issues
- Track processing metrics (pages processed, chunks created, storage usage)

## Scaling Considerations

### Horizontal Scaling
- Process multiple sites in parallel using separate processes
- Distribute load across multiple instances for large sites
- Use queue-based processing for high-volume scenarios

### Vertical Scaling
- Increase memory allocation for processing large documents
- Allocate more CPU cores for faster embedding generation
- Use faster network connections for better crawling performance

### Rate Limiting
- Adjust `--max-retries` and request timing to respect target servers
- Implement appropriate delays between requests
- Monitor for and handle API rate limits gracefully

## Health Checks

### Pre-flight Checks
- Verify connectivity to Qdrant Cloud
- Confirm Cohere API key validity
- Check target site accessibility

### Runtime Monitoring
- Monitor embedding generation success rate
- Track storage success rate in Qdrant
- Watch for memory usage during processing

## Troubleshooting

### Common Issues
1. **API Rate Limits**: Implement exponential backoff and retry logic
2. **Memory Issues**: Process large documents in streaming fashion
3. **Connectivity Problems**: Verify network access and API credentials
4. **Content Extraction Issues**: Handle dynamic content with appropriate tools

### Error Recovery
- The pipeline includes comprehensive retry mechanisms
- Failed chunks can be reprocessed independently
- Monitor logs for persistent failures requiring manual intervention

## Maintenance

### Regular Tasks
- Rotate API keys periodically
- Monitor storage costs in Qdrant Cloud
- Update dependencies regularly
- Review and clean up old collections if needed

### Backup Strategy
- Qdrant Cloud handles storage redundancy
- Consider backing up configuration files
- Document the deployment process for recovery scenarios