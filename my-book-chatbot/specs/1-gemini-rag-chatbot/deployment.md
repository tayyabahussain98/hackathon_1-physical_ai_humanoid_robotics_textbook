# Deployment Guide: Gemini RAG Chatbot

## Overview
This document outlines the deployment process for the Gemini RAG Chatbot system, including both backend and frontend components.

## Architecture
- **Backend**: FastAPI application deployed on Railway
- **Frontend**: Next.js application deployed on Vercel
- **Documentation**: Docusaurus site with integrated chatbot, deployed on GitHub Pages
- **Vector Database**: Qdrant Cloud
- **Database**: Neon Postgres

## Backend Deployment (Railway)

### Prerequisites
- Railway account
- Qdrant Cloud account and credentials
- Neon Postgres database
- Google Gemini API key

### Steps
1. Create a new Railway project
2. Connect your GitHub repository
3. Set the following environment variables:
   - `GEMINI_API_KEY`: Your Google Gemini API key
   - `QDRANT_URL`: Your Qdrant Cloud URL
   - `QDRANT_API_KEY`: Your Qdrant Cloud API key
   - `NEON_DATABASE_URL`: Your Neon Postgres connection string
4. Set the start command to: `uvicorn backend.main:app --host 0.0.0.0 --port $PORT`
5. Deploy the application

## Frontend Deployment (Vercel)

### Prerequisites
- Vercel account
- Frontend code ready

### Steps
1. Create a new Vercel project
2. Connect your GitHub repository
3. Set the project root to `frontend/`
4. No special environment variables needed for frontend
5. Build command: `npm run build`
6. Output directory: `out` (default for Next.js)
7. Deploy the application

## Documentation Integration (GitHub Pages)

### Prerequisites
- GitHub repository with Pages enabled
- Docusaurus site configured

### Steps
1. The docusaurus.config.ts file has been updated to include both development and production chatbot scripts
2. The scripts will be loaded on all pages of the documentation site
3. Ensure that GitHub Pages is enabled for your repository

## Environment Variables

### Backend (.env)
```
GEMINI_API_KEY=your_gemini_api_key_here
QDRANT_URL=your_qdrant_cloud_url
QDRANT_API_KEY=your_qdrant_api_key
NEON_DATABASE_URL=your_neon_postgres_connection_string
```

### Frontend
The frontend does not require specific environment variables as it communicates with the backend API.

## Testing the Deployment

### Pre-deployment Testing
1. Test locally with both development and production configurations
2. Verify that the chat functionality works in the documentation
3. Test text selection and response generation
4. Verify dark mode functionality

### Post-deployment Testing
1. Access the deployed documentation site
2. Verify that the chat button appears on all pages
3. Test text selection functionality
4. Verify that chat responses are generated correctly
5. Test the end-to-end flow: highlight text → open chat → get Gemini response

## Rollback Plan
- Maintain previous deployment versions on Railway and Vercel
- In case of issues, revert to the previous stable version
- Monitor logs and error reports after deployment

## Monitoring
- Set up error tracking for both backend and frontend
- Monitor response times for the chat functionality
- Track usage statistics for the chatbot feature