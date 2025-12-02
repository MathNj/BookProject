"""
Configuration module for RAG Backend.

Loads environment variables for:
- Qdrant vector database connection
- Google Gemini API access
- Database credentials
- Authentication secrets
"""

from pydantic_settings import BaseSettings
from typing import Optional


class Settings(BaseSettings):
    """Application configuration from environment variables."""

    # Qdrant Vector Store Configuration
    qdrant_url: str = "http://localhost:6333"
    qdrant_api_key: Optional[str] = None
    qdrant_collection_name: str = "textbook_content"

    # API Configuration
    openai_api_key: str = ""  # OpenAI API key for embeddings and chat
    api_key: str = ""  # Backwards compatibility alias
    chat_model: str = "gemini-2.5-flash"
    embedding_model: str = "text-embedding-3-small"  # OpenAI embedding model
    openai_model: str = "text-embedding-3-small"  # For backwards compatibility
    openai_embedding_dimensions: int = 1536  # text-embedding-3-small produces 1536-dim vectors

    # Database Configuration
    database_url: Optional[str] = None
    database_echo: bool = False

    # Authentication Configuration
    jwt_secret_key: str = "your-secret-key-change-in-production"
    jwt_algorithm: str = "HS256"
    jwt_expiration_hours: int = 24

    # Frontend Configuration
    frontend_url: str = "http://localhost:3000"
    cors_origins: list = [
        "http://localhost:3000",
        "http://localhost:3001",
    ]
    api_port: int = 8000

    # Feature Flags
    enable_rag_indexing: bool = True
    enable_auth: bool = True

    # Logging
    log_level: str = "INFO"

    class Config:
        env_file = ".env"
        env_file_encoding = "utf-8"
        case_sensitive = False
        extra = "ignore"  # Ignore extra fields from .env file


# Create a singleton instance
settings = Settings()
