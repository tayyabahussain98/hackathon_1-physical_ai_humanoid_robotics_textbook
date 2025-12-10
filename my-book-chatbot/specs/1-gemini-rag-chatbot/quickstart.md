# Quickstart Guide: Gemini RAG Chatbot

## Prerequisites

- Python 3.9+ installed
- Node.js 18+ installed
- uv package manager installed (`pip install uv`)
- Access to Google Gemini API
- Qdrant Cloud account
- Neon Postgres database

## Setup

### 1. Initialize the Project

```bash
# Create project directory
mkdir gemini-rag-chatbot
cd gemini-rag-chatbot

# Initialize Python project with uv
uv init
```

### 2. Create Directory Structure

```
gemini-rag-chatbot/
├── backend/
│   └── main.py
├── frontend/
│   └── src/
│       └── app/
│           └── page.js
├── .env
└── pyproject.toml
```

### 3. Environment Variables

Create a `.env` file with the following:

```env
# Gemini API Configuration
GEMINI_API_KEY=your_gemini_api_key_here
LITELLM_API_BASE=https://generativelanguage.googleapis.com/v1beta

# Qdrant Configuration
QDRANT_URL=your_qdrant_cloud_url
QDRANT_API_KEY=your_qdrant_api_key

# Neon Postgres Configuration
DATABASE_URL=your_neon_postgres_connection_string

# Application Settings
UV_ENV=development
```

### 4. Install Dependencies

Backend dependencies in `pyproject.toml`:

```toml
[project]
name = "gemini-rag-chatbot"
version = "0.1.0"
dependencies = [
    "fastapi>=0.104.0",
    "uvicorn>=0.24.0",
    "python-dotenv>=1.0.0",
    "qdrant-client>=1.8.0",
    "openai>=1.3.5",
    "litellm>=1.35.0",
    "pydantic>=2.5.0",
    "sqlalchemy>=2.0.0",
    "psycopg2-binary>=2.9.9",
    "python-multipart>=0.0.6",
    "aiofiles>=23.2.1"
]
```

Install with uv:

```bash
uv pip install -e .
```

Frontend dependencies:

```bash
cd frontend
npm create next-app@latest .
npm install @openai/chatkit-react
```

## Running the Application

### 1. Start the Backend

```bash
cd backend
uvicorn main:app --reload --port 8000
```

### 2. Start the Frontend

```bash
cd frontend
npm run dev
```

### 3. Ingest Documentation

To ingest documentation content into the vector database:

```bash
cd backend
python main.py --ingest
```

## API Endpoints

### Chat Endpoint
- **POST** `/api/chat`
- Process user messages with RAG context
- Example:
```bash
curl -X POST http://localhost:8000/api/chat \
  -H "Content-Type: application/json" \
  -d '{
    "message": "What does this function do?",
    "conversation_id": "new",
    "selected_text": "This is the function that handles authentication"
  }'
```

### Health Check
- **GET** `/api/health`
- Check API status

### Documentation Ingestion
- **POST** `/api/ingest`
- Ingest documentation content

## Development Workflow

1. Make changes to `backend/main.py` for backend functionality
2. Make changes to `frontend/src/app/page.js` for frontend functionality
3. Test with `uvicorn main:app --reload` for backend hot-reload
4. Test with `npm run dev` for frontend hot-reload

## Testing

### Backend Tests

```bash
# Run backend tests
python -m pytest tests/
```

### Frontend Tests

```bash
# Run frontend tests
npm test
```

## Deployment

### Environment Variables for Production

Update your environment variables for production:

```env
# Production settings
UV_ENV=production
DEBUG=false

# Production URLs
QDRANT_URL=your_production_qdrant_url
DATABASE_URL=your_production_database_url

# Security
SECRET_KEY=your_strong_secret_key
ALLOWED_ORIGINS=https://yourdomain.com
```

### Build and Deploy

1. Build the frontend: `npm run build`
2. Deploy the backend with uv
3. Configure your domain and SSL
4. Set up monitoring and logging