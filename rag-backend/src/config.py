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

    # API Configuration (Gemini key used with OpenAI SDK)
    api_key: str = ""  # Google Gemini API key
    openai_api_key: str = ""  # Alias for api_key, for backwards compatibility
    chat_model: str = "gpt-4o-mini"
    embedding_model: str = "text-embedding-3-small"
    openai_model: str = "text-embedding-3-small"  # For backwards compatibility
    openai_embedding_dimensions: int = 1536  # OpenAI embedding vector size

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
        "http://localhost:5173",
        "http://localhost:8000",
    ]

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
