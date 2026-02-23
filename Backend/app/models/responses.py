# =============================================================================
# RAG Chatbot Backend - Response Models
# =============================================================================
# Pydantic models for API response serialization
# T015: Response models per api-contracts.md
# =============================================================================

from datetime import datetime
from typing import Any, Optional
from uuid import UUID

from pydantic import BaseModel, Field


# -----------------------------------------------------------------------------
# Query Response Models
# -----------------------------------------------------------------------------


class QueryResponse(BaseModel):
    """Response body for POST /v1/query endpoint (non-streaming)."""

    answer: str = Field(..., description="Generated response text")
    sources: list[str] = Field(
        default_factory=list,
        description="Chapter/section references used in response",
    )
    confidence: str = Field(
        ...,
        pattern="^(high|medium|low)$",
        description="Confidence level based on retrieval score",
    )
    retrieval_latency_ms: int = Field(
        ..., ge=0, description="Time for vector search in milliseconds"
    )
    generation_latency_ms: int = Field(
        ..., ge=0, description="Time for LLM response in milliseconds"
    )

    model_config = {
        "json_schema_extra": {
            "examples": [
                {
                    "answer": "ROS2 (Robot Operating System 2) is the next generation...",
                    "sources": [
                        "Chapter 1: Introduction to ROS2",
                        "Chapter 2: ROS2 vs ROS1 Comparison",
                    ],
                    "confidence": "high",
                    "retrieval_latency_ms": 87,
                    "generation_latency_ms": 1245,
                }
            ]
        }
    }


class QueuedResponse(BaseModel):
    """Response when query is queued due to rate limiting."""

    status: str = Field(default="queued", description="Queue status")
    queue_position: int = Field(..., ge=1, description="Position in queue")
    estimated_wait_seconds: int = Field(..., ge=0, description="Estimated wait time")
    message: str = Field(..., description="User-friendly status message")


# -----------------------------------------------------------------------------
# Ingest Response Models
# -----------------------------------------------------------------------------


class IngestStartResponse(BaseModel):
    """Response for POST /v1/ingest - ingestion started."""

    run_id: UUID = Field(..., description="Unique identifier for this ingestion run")
    status: str = Field(default="running", description="Ingestion status")
    message: str = Field(..., description="Status message")


class IngestStatusResponse(BaseModel):
    """Response for GET /v1/ingest/{run_id} - ingestion progress."""

    run_id: UUID = Field(..., description="Unique identifier for this ingestion run")
    status: str = Field(
        ...,
        pattern="^(running|completed|failed)$",
        description="Current status",
    )
    started_at: datetime = Field(..., description="When ingestion started")
    completed_at: Optional[datetime] = Field(
        default=None, description="When ingestion completed"
    )
    files_processed: int = Field(default=0, ge=0, description="Number of files processed")
    chunks_created: int = Field(default=0, ge=0, description="New chunks created")
    chunks_updated: int = Field(default=0, ge=0, description="Existing chunks updated")
    chunks_deleted: int = Field(default=0, ge=0, description="Orphaned chunks deleted")
    duration_seconds: Optional[float] = Field(
        default=None, ge=0, description="Total duration in seconds"
    )
    error_message: Optional[str] = Field(
        default=None, description="Error message if status is failed"
    )


# -----------------------------------------------------------------------------
# Health Response Models
# -----------------------------------------------------------------------------


class ComponentStatus(BaseModel):
    """Status of an individual system component."""

    status: str = Field(..., description="Component status")
    latency_ms: Optional[int] = Field(default=None, ge=0, description="Latency in ms")
    model: Optional[str] = Field(default=None, description="Model name if applicable")
    fallback_available: Optional[bool] = Field(
        default=None, description="Whether fallback is available"
    )
    fallback_reason: Optional[str] = Field(
        default=None, description="Reason for fallback if active"
    )


class HealthResponse(BaseModel):
    """Response for GET /v1/health endpoint."""

    status: str = Field(
        ...,
        pattern="^(healthy|degraded|unhealthy)$",
        description="Overall system health",
    )
    components: dict[str, ComponentStatus] = Field(
        ..., description="Individual component statuses"
    )
    timestamp: datetime = Field(
        default_factory=datetime.utcnow, description="Health check timestamp"
    )


# -----------------------------------------------------------------------------
# Metrics Response Models
# -----------------------------------------------------------------------------


class IndexingMetrics(BaseModel):
    """Indexing-related metrics."""

    total_chunks: int = Field(..., ge=0, description="Total indexed chunks")
    total_files: int = Field(..., ge=0, description="Total indexed files")
    last_ingestion: Optional[datetime] = Field(
        default=None, description="Last ingestion timestamp"
    )
    index_size_mb: float = Field(..., ge=0, description="Index size in MB")


class QueryMetrics(BaseModel):
    """Query performance metrics."""

    total_24h: int = Field(..., ge=0, description="Total queries in last 24 hours")
    avg_latency_ms: float = Field(..., ge=0, description="Average query latency")
    p95_latency_ms: float = Field(..., ge=0, description="95th percentile latency")
    fallback_rate_percent: float = Field(
        ..., ge=0, le=100, description="Percentage of fallback responses"
    )


class RetrievalMetrics(BaseModel):
    """Retrieval quality metrics."""

    avg_top_score: float = Field(..., ge=0, le=1, description="Average top relevance score")
    avg_precision_at_5: float = Field(
        ..., ge=0, le=1, description="Average precision at 5 chunks"
    )
    out_of_scope_rate_percent: float = Field(
        ..., ge=0, le=100, description="Percentage of out-of-scope queries"
    )


class SystemStatusMetrics(BaseModel):
    """System status metrics."""

    uptime_hours: float = Field(..., ge=0, description="Uptime in hours")
    error_rate_percent: float = Field(
        ..., ge=0, le=100, description="Error rate percentage"
    )


class MetricsResponse(BaseModel):
    """Response for GET /v1/metrics endpoint."""

    indexing: IndexingMetrics = Field(..., description="Indexing metrics")
    queries: QueryMetrics = Field(..., description="Query performance metrics")
    retrieval: RetrievalMetrics = Field(..., description="Retrieval quality metrics")
    system: SystemStatusMetrics = Field(..., description="System status metrics")
    timestamp: datetime = Field(
        default_factory=datetime.utcnow, description="Metrics timestamp"
    )


# -----------------------------------------------------------------------------
# Error Response Models
# -----------------------------------------------------------------------------


class ErrorDetail(BaseModel):
    """Error response detail."""

    code: str = Field(..., description="Error code")
    message: str = Field(..., description="Human-readable error message")
    details: Optional[dict[str, Any]] = Field(
        default=None, description="Additional error details"
    )


class ErrorResponse(BaseModel):
    """Standard error response format."""

    error: ErrorDetail = Field(..., description="Error information")
