from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from .router import router

# Create FastAPI app
app = FastAPI(
    title="RAG Agent API",
    description="API for RAG-enabled agent using OpenAI SDK with Google Gemini",
    version="1.0.0"
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, replace with specific origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include API router
app.include_router(router, prefix="", tags=["query"])

__all__ = ["app"]