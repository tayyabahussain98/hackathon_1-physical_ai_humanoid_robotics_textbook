"""
Vercel entry point for FastAPI application.

This module wraps the FastAPI app with Mangum to make it compatible with 
Vercel's serverless functions (which use AWS Lambda under the hood).
"""

from mangum import Mangum
from api import app

# Wrap FastAPI app with Mangum adapter for AWS Lambda/Vercel
# lifespan="off" prevents issues with startup/shutdown events in serverless
handler = Mangum(app, lifespan="off")
