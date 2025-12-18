# =============================================================================
# RAG Chatbot Backend - Request Models
# =============================================================================
# Pydantic models for API request validation
# T014: Request models per api-contracts.md
# =============================================================================

from typing import Optional

from pydantic import BaseModel, Field


class QueryRequest(BaseModel):
    """Request body for POST /v1/query endpoint."""

    question: str = Field(
        ...,
        min_length=1,
        max_length=1000,
        description="User's question about book content",
    )
    session_id: Optional[str] = Field(
        default=None,
        description="Optional session ID for context tracking",
    )
    stream: bool = Field(
        default=False,
        description="Enable SSE streaming response",
    )

    model_config = {
        "json_schema_extra": {
            "examples": [
                {
                    "question": "What is ROS2 and how does it differ from ROS1?",
                    "session_id": "550e8400-e29b-41d4-a716-446655440000",
                    "stream": False,
                }
            ]
        }
    }


class IngestRequest(BaseModel):
    """Request body for POST /v1/ingest endpoint."""

    source_directory: str = Field(
        default="docs/",
        description="Path to directory containing book content (MD/MDX files)",
    )
    force_reindex: bool = Field(
        default=False,
        description="Re-embed all content regardless of changes",
    )

    model_config = {
        "json_schema_extra": {
            "examples": [
                {
                    "source_directory": "docs/",
                    "force_reindex": False,
                }
            ]
        }
    }
