# Quickstart: RAG Agent Backend with OpenAI Agents SDK using Gemini

## Prerequisites
- Python 3.11+
- UV package manager
- Access to Google's Gemini API
- Qdrant Cloud account with API credentials

## Setup

1. **Clone the repository**
   ```bash
   git clone <repository-url>
   cd <repository-name>
   ```

2. **Navigate to backend directory**
   ```bash
   cd backend
   ```

3. **Install dependencies using UV**
   ```bash
   uv init
   uv pip install fastapi openai-agents python-dotenv qdrant-client pydantic pytest
   ```

4. **Set up environment variables**
   Create a `.env` file based on `.env.example`:
   ```env
   GEMINI_API_KEY=your_gemini_api_key
   QDRANT_URL=your_qdrant_cluster_url
   QDRANT_API_KEY=your_qdrant_api_key
   GEMINI_MODEL_NAME=gemini-2.5-flash
   ```

## Running the Application

1. **Start the FastAPI server**
   ```bash
   uv run python main.py
   ```
   The server will start on `http://localhost:8000`

2. **Test the API**
   ```bash
   curl -X POST http://localhost:8000/query \
     -H "Content-Type: application/json" \
     -d '{
       "query": "What is ROS 2?",
       "top_k": 3,
       "threshold": 0.7
     }'
   ```

## Key Components

- **Agent**: Configured OpenAI Agent using Gemini via OpenAI-compatible API
- **Qdrant Service**: Handles vector retrieval from Qdrant database
- **Context Injector**: Injects retrieved context into agent prompts
- **API Router**: Exposes the `/query` endpoint

## Configuration

The system can be configured via environment variables:
- `GEMINI_API_KEY`: Your Google Gemini API key
- `QDRANT_URL`: URL of your Qdrant cluster
- `QDRANT_API_KEY`: API key for Qdrant access
- `GEMINI_MODEL_NAME`: Name of the Gemini model to use (default: gemini-2.5-flash)
- `DEFAULT_TOP_K`: Default number of documents to retrieve (default: 5)
- `DEFAULT_THRESHOLD`: Default similarity threshold (default: 0.7)

## Testing

Run the test suite:
```bash
pytest tests/
```

## Architecture

The system follows a clean architecture:
1. API layer receives queries
2. Qdrant service retrieves relevant documents
3. Context injector prepares the prompt with retrieved context
4. Agent generates grounded response
5. Response is returned via API