"""
FastAPI backend for Physical AI & Humanoid Robotics Textbook RAG system.

Main application entry point with CORS configuration, health check endpoint, and RAG chat endpoint.
"""

from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse
import os
import logging
from typing import Dict, Any

from src.schemas import ChatRequest, ChatResponse, SearchResponse, ErrorResponse, SearchResult
from src.agent import get_agent
from src.services.vector_store import get_vector_store

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

app = FastAPI(
    title="Physical AI & Humanoid Robotics - RAG Backend",
    description="Context-aware RAG chatbot backend for interactive textbook",
    version="0.1.0",
)

# CORS Configuration - Allow frontend and development servers
origins = [
    "http://localhost:3000",  # Local development - PRIMARY
    "http://localhost:3001",  # Docusaurus on secondary port (fallback)
    "http://localhost:5173",  # Vite dev server (alternative)
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


@app.post("/chat", response_model=ChatResponse, tags=["Chat"], status_code=200)
async def chat(request: ChatRequest) -> ChatResponse:
    """
    Chat endpoint for RAG-powered question answering.

    Accepts a list of messages and returns an AI-generated response based on the textbook context.

    Args:
        request: ChatRequest with messages list

    Returns:
        ChatResponse: Assistant's response

    Raises:
        HTTPException: If there's an error processing the request
    """
    try:
        if not request.messages:
            raise HTTPException(status_code=400, detail="At least one message is required")

        logger.info(f"Processing chat request with {len(request.messages)} message(s)")

        # Convert schema messages to dict format for agent
        messages = [msg.dict() for msg in request.messages]

        # Get agent and run it
        agent = get_agent()
        response = await agent.run_sync(messages)

        logger.info("Chat request processed successfully")
        return ChatResponse(
            message=response,
            model=request.model,
            success=True
        )

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error processing chat request: {e}")
        raise HTTPException(
            status_code=500,
            detail=f"Error processing chat request: {str(e)}"
        )


@app.post("/search", response_model=SearchResponse, tags=["Search"], status_code=200)
async def search(query: str, limit: int = 3) -> SearchResponse:
    """
    Search endpoint for semantic search in the textbook.

    Finds relevant passages from the textbook based on semantic similarity.

    Args:
        query: Search query string
        limit: Maximum number of results (default: 3, max: 10)

    Returns:
        SearchResponse: List of relevant passages with metadata

    Raises:
        HTTPException: If there's an error processing the request
    """
    try:
        if not query or not query.strip():
            raise HTTPException(status_code=400, detail="Query cannot be empty")

        limit = min(max(limit, 1), 10)  # Clamp between 1 and 10
        logger.info(f"Search query: '{query}' (limit: {limit})")

        # Get vector store and perform search
        vector_store = get_vector_store()
        results = vector_store.search(query, limit=limit, min_similarity=0.85)

        # Convert results to response model
        search_results = [
            SearchResult(
                text=result.get("text", ""),
                source=result.get("source", "unknown"),
                chapter=result.get("chapter", "unknown"),
                section=result.get("section", "unknown"),
                relevance_score=round(result.get("score", 0), 3)
            )
            for result in results
        ]

        logger.info(f"Search found {len(search_results)} results")
        return SearchResponse(
            results=search_results,
            count=len(search_results),
            query=query
        )

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error processing search request: {e}")
        raise HTTPException(
            status_code=500,
            detail=f"Error processing search request: {str(e)}"
        )


@app.on_event("startup")
async def startup_event():
    """Startup event handler for initializing services."""
    logger.info("RAG Backend starting up...")
    try:
        # Initialize vector store connection (lazy loaded)
        vector_store = get_vector_store()
        logger.info("Vector store initialized")

        # Initialize agent (lazy loaded)
        agent = get_agent()
        logger.info("RAG agent initialized")

    except Exception as e:
        logger.error(f"Failed to initialize services: {e}")
        raise


@app.on_event("shutdown")
async def shutdown_event():
    """Shutdown event handler for cleanup."""
    logger.info("RAG Backend shutting down...")
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
