# =============================================================================
# RAG Chatbot Backend - Configuration
# =============================================================================
# Environment configuration using Pydantic Settings
# T013: Configuration loader
# =============================================================================

from functools import lru_cache
from typing import Optional

from pydantic import Field, field_validator
from pydantic_settings import BaseSettings, SettingsConfigDict


class Settings(BaseSettings):
    """Application configuration loaded from environment variables."""

    model_config = SettingsConfigDict(
        env_file=".env",
        env_file_encoding="utf-8",
        case_sensitive=False,
        extra="ignore",
    )

    # -------------------------------------------------------------------------
    # OpenAI Configuration
    # -------------------------------------------------------------------------
    openai_api_key: str = Field(..., description="OpenAI API key")
    openai_model: str = Field(default="gpt-4o-mini", description="OpenAI model to use")

    # -------------------------------------------------------------------------
    # OpenRouter Fallback
    # -------------------------------------------------------------------------
    openrouter_api_key: Optional[str] = Field(
        default=None, description="OpenRouter API key for fallback"
    )
    openrouter_model: str = Field(
        default="openai/gpt-4o-mini", description="OpenRouter model to use"
    )

    # -------------------------------------------------------------------------
    # Embedding Services
    # -------------------------------------------------------------------------
    cohere_api_key: str = Field(..., description="Cohere API key for embeddings")

    # -------------------------------------------------------------------------
    # Qdrant Configuration
    # -------------------------------------------------------------------------
    qdrant_url: str = Field(
        default="http://localhost:6333", description="Qdrant server URL"
    )
    qdrant_api_key: Optional[str] = Field(
        default=None, description="Qdrant API key (optional for local)"
    )
    qdrant_collection: str = Field(
        default="book_chunks", description="Qdrant collection name"
    )

    # -------------------------------------------------------------------------
    # PostgreSQL Configuration
    # -------------------------------------------------------------------------
    database_url: str = Field(..., description="PostgreSQL connection URL")

    # -------------------------------------------------------------------------
    # API Configuration
    # -------------------------------------------------------------------------
    api_host: str = Field(default="0.0.0.0", description="API server host")
    api_port: int = Field(default=8000, description="API server port")
    api_admin_key: str = Field(..., description="Admin API key for protected endpoints")

    # -------------------------------------------------------------------------
    # CORS Configuration
    # -------------------------------------------------------------------------
    cors_origins: str = Field(
        default="http://localhost:3000",
        description="Comma-separated list of allowed origins",
    )

    @property
    def cors_origins_list(self) -> list[str]:
        """Parse CORS origins into a list."""
        return [origin.strip() for origin in self.cors_origins.split(",")]

    # -------------------------------------------------------------------------
    # Logging Configuration
    # -------------------------------------------------------------------------
    log_level: str = Field(default="INFO", description="Logging level")
    log_format: str = Field(default="json", description="Log format (json or text)")

    @field_validator("log_level")
    @classmethod
    def validate_log_level(cls, v: str) -> str:
        """Validate log level."""
        valid_levels = {"DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"}
        if v.upper() not in valid_levels:
            raise ValueError(f"Invalid log level: {v}. Must be one of {valid_levels}")
        return v.upper()

    # -------------------------------------------------------------------------
    # RAG Configuration
    # -------------------------------------------------------------------------
    relevance_threshold: float = Field(
        default=0.7, description="Minimum relevance score for confident responses"
    )
    top_k_chunks: int = Field(
        default=5, description="Number of chunks to retrieve"
    )
    chunk_size: int = Field(
        default=512, description="Chunk size in tokens"
    )
    chunk_overlap: int = Field(
        default=50, description="Chunk overlap in tokens"
    )

    @field_validator("relevance_threshold")
    @classmethod
    def validate_relevance_threshold(cls, v: float) -> float:
        """Validate relevance threshold is between 0 and 1."""
        if not 0.0 <= v <= 1.0:
            raise ValueError("Relevance threshold must be between 0.0 and 1.0")
        return v

    # -------------------------------------------------------------------------
    # Rate Limiting Configuration
    # -------------------------------------------------------------------------
    rate_limit_requests_per_minute: int = Field(
        default=10, description="Maximum requests per minute per IP"
    )
    rate_limit_queue_size: int = Field(
        default=50, description="Maximum queue depth"
    )
    rate_limit_queue_timeout: int = Field(
        default=30, description="Queue timeout in seconds"
    )


@lru_cache
def get_settings() -> Settings:
    """Get cached settings instance."""
    return Settings()
