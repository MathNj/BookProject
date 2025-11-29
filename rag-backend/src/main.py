"""
FastAPI backend for Physical AI & Humanoid Robotics Textbook RAG system.

Main application entry point with CORS configuration and health check endpoint.
"""

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse
import os
from typing import Dict, Any

app = FastAPI(
    title="Physical AI & Humanoid Robotics - RAG Backend",
    description="Context-aware RAG chatbot backend for interactive textbook",
    version="0.1.0",
)

# CORS Configuration - Allow GitHub Pages frontend
origins = [
    "http://localhost:3000",  # Local development
    "http://localhost:5173",  # Vite dev server
    os.getenv("FRONTEND_URL", "https://example.github.io"),  # Production frontend URL
]

app.add_middleware(
    CORSMiddleware,
    allow_origins=origins,
    allow_credentials=True,
    allow_methods=["GET", "POST", "PUT", "DELETE", "OPTIONS"],
    allow_headers=["*"],
    expose_headers=["Content-Length", "X-Total-Count"],
    max_age=3600,
)


@app.get("/health", tags=["Health"])
async def health_check() -> Dict[str, Any]:
    """
    Health check endpoint for monitoring and deployment verification.

    Returns:
        dict: Status information including service health
    """
    return {
        "status": "ok",
        "service": "rag-backend",
        "version": "0.1.0",
    }


@app.get("/", tags=["Root"])
async def root() -> Dict[str, str]:
    """
    Root endpoint with service information.

    Returns:
        dict: Service metadata
    """
    return {
        "message": "Physical AI & Humanoid Robotics RAG Backend",
        "docs": "/docs",
        "health": "/health",
    }


@app.on_event("startup")
async def startup_event():
    """Startup event handler for initializing services."""
    print("🚀 RAG Backend starting up...")
    # TODO: Initialize database connections, Qdrant client, OpenAI client


@app.on_event("shutdown")
async def shutdown_event():
    """Shutdown event handler for cleanup."""
    print("🛑 RAG Backend shutting down...")
    # TODO: Close database connections, cleanup resources


if __name__ == "__main__":
    import uvicorn

    uvicorn.run(
        "src.main:app",
        host="0.0.0.0",
        port=8000,
        reload=True,  # Enable auto-reload during development
        log_level="info",
    )
