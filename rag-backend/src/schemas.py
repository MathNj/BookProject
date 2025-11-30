"""
Request and response schemas for RAG Backend API.

Defines Pydantic models for type validation and API documentation.
"""

from pydantic import BaseModel, Field
from typing import Optional, List


class Message(BaseModel):
    """Message object for chat history."""

    role: str = Field(..., description="Role of the message sender (user, assistant, system)")
    content: str = Field(..., description="Content of the message")


class ChatRequest(BaseModel):
    """Request body for chat endpoint."""

    messages: List[Message] = Field(..., description="Conversation history with at least one message")
    model: Optional[str] = Field(default="gpt-4o-mini", description="OpenAI model to use (default: gpt-4o-mini)")

    class Config:
        example = {
            "messages": [
                {"role": "user", "content": "What is ROS 2?"}
            ],
            "model": "gpt-4o-mini"
        }


class ChatResponse(BaseModel):
    """Response body for chat endpoint."""

    message: str = Field(..., description="Assistant's response to the user's question")
    model: str = Field(..., description="Model used for the response")
    success: bool = Field(default=True, description="Whether the request was successful")

    class Config:
        example = {
            "message": "ROS 2 (Robot Operating System 2) is...",
            "model": "gpt-4o-mini",
            "success": True
        }


class SearchResult(BaseModel):
    """A single search result from the vector store."""

    text: str = Field(..., description="Text excerpt from the textbook")
    source: str = Field(..., description="Source file name")
    chapter: str = Field(..., description="Chapter title")
    section: str = Field(..., description="Section identifier")
    relevance_score: float = Field(..., description="Cosine similarity score (0-1)")


class SearchResponse(BaseModel):
    """Response body for search endpoint."""

    results: List[SearchResult] = Field(..., description="Search results")
    count: int = Field(..., description="Number of results returned")
    query: str = Field(..., description="The search query used")

    class Config:
        example = {
            "results": [
                {
                    "text": "ROS 2 is a middleware...",
                    "source": "01-nervous-system/intro.md",
                    "chapter": "The Nervous System",
                    "section": "Section 1",
                    "relevance_score": 0.95
                }
            ],
            "count": 1,
            "query": "What is ROS 2?"
        }


class ErrorResponse(BaseModel):
    """Standard error response."""

    error: str = Field(..., description="Error message")
    detail: Optional[str] = Field(None, description="Additional error details")
    success: bool = Field(default=False, description="Always False for errors")
