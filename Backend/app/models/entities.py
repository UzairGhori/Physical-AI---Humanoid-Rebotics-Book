# =============================================================================
# RAG Chatbot Backend - Domain Entities
# =============================================================================
# Core domain models used internally by services
# T016: Entity models per data-model.md
# =============================================================================

from datetime import datetime
from typing import Optional
from uuid import UUID, uuid4

from pydantic import BaseModel, Field


# -----------------------------------------------------------------------------
# Ingestion Entities
# -----------------------------------------------------------------------------


class BookChunk(BaseModel):
    """A segment of book content prepared for embedding."""

    chunk_id: str = Field(..., description="Unique identifier for the chunk")
    content: str = Field(..., max_length=4000, description="Chunk text content")
    source_file: str = Field(..., description="Path to source MD/MDX file")
    chapter: str = Field(..., description="Chapter title")
    section: Optional[str] = Field(default=None, description="Section title")
    position: int = Field(..., ge=0, description="Position within source file")
    char_count: int = Field(..., gt=0, description="Character count")
    token_count: int = Field(..., gt=0, description="Token count")
    content_hash: Optional[str] = Field(
        default=None, description="SHA-256 hash for deduplication"
    )


class EmbeddingResult(BaseModel):
    """Result of embedding generation."""

    chunk_id: str = Field(..., description="ID of the embedded chunk")
    vector: list[float] = Field(
        ..., min_length=1024, max_length=1024, description="1024-dimensional vector"
    )
    model_used: str = Field(
        ..., pattern="^cohere$", description="Embedding model used"
    )
    latency_ms: int = Field(..., ge=0, description="Embedding generation latency")


class ChunkMetadata(BaseModel):
    """Metadata stored in PostgreSQL for each chunk."""

    id: UUID = Field(default_factory=uuid4, description="Primary key")
    vector_id: UUID = Field(..., description="Corresponding Qdrant point ID")
    chunk_id: str = Field(..., description="Chunk identifier")
    source_file: str = Field(..., description="Source file path")
    chapter: str = Field(..., description="Chapter title")
    section: Optional[str] = Field(default=None, description="Section title")
    position: int = Field(..., ge=0, description="Position in file")
    content_hash: str = Field(..., description="SHA-256 content hash")
    char_count: int = Field(..., gt=0, description="Character count")
    token_count: int = Field(..., gt=0, description="Token count")
    embedding_model: str = Field(default="qwen", description="Model used for embedding")
    created_at: datetime = Field(
        default_factory=datetime.utcnow, description="Creation timestamp"
    )
    updated_at: datetime = Field(
        default_factory=datetime.utcnow, description="Last update timestamp"
    )


# -----------------------------------------------------------------------------
# Query Entities
# -----------------------------------------------------------------------------


class RetrievedChunk(BaseModel):
    """A chunk returned from vector search."""

    chunk_id: str = Field(..., description="Chunk identifier")
    content: str = Field(..., description="Chunk text content")
    chapter: str = Field(..., description="Chapter title")
    section: Optional[str] = Field(default=None, description="Section title")
    relevance_score: float = Field(
        ..., ge=0.0, le=1.0, description="Cosine similarity score"
    )
    source_file: Optional[str] = Field(default=None, description="Source file path")


class RetrievalResult(BaseModel):
    """Complete retrieval response from vector search."""

    query: str = Field(..., description="Original query text")
    chunks: list[RetrievedChunk] = Field(
        default_factory=list, max_length=10, description="Retrieved chunks"
    )
    top_score: float = Field(..., ge=0.0, le=1.0, description="Highest relevance score")
    is_relevant: bool = Field(
        ..., description="Whether top score meets relevance threshold"
    )
    confidence: str = Field(
        ..., pattern="^(high|medium|low)$", description="Confidence level"
    )
    model_used: str = Field(..., description="Embedding model used for query")
    latency_ms: int = Field(..., ge=0, description="Search latency in milliseconds")


class ChatMessage(BaseModel):
    """A single message in conversation."""

    role: str = Field(..., pattern="^(user|assistant)$", description="Message role")
    content: str = Field(..., description="Message content")
    timestamp: datetime = Field(
        default_factory=datetime.utcnow, description="Message timestamp"
    )
    sources: Optional[list[str]] = Field(
        default=None, description="Source references (for assistant messages)"
    )


class ChatSession(BaseModel):
    """A chat session containing conversation history."""

    session_id: str = Field(..., description="Unique session identifier")
    messages: list[ChatMessage] = Field(
        default_factory=list, description="Conversation history"
    )
    created_at: datetime = Field(
        default_factory=datetime.utcnow, description="Session creation time"
    )
    last_activity_at: datetime = Field(
        default_factory=datetime.utcnow, description="Last activity timestamp"
    )


# -----------------------------------------------------------------------------
# Ingestion Run Tracking
# -----------------------------------------------------------------------------


class IngestionRun(BaseModel):
    """Tracks an ingestion execution."""

    id: UUID = Field(default_factory=uuid4, description="Run identifier")
    started_at: datetime = Field(
        default_factory=datetime.utcnow, description="Start timestamp"
    )
    completed_at: Optional[datetime] = Field(
        default=None, description="Completion timestamp"
    )
    status: str = Field(
        default="running",
        pattern="^(running|completed|failed)$",
        description="Run status",
    )
    source_directory: str = Field(..., description="Source directory path")
    files_processed: int = Field(default=0, ge=0, description="Files processed count")
    chunks_created: int = Field(default=0, ge=0, description="New chunks created")
    chunks_updated: int = Field(default=0, ge=0, description="Chunks updated")
    chunks_deleted: int = Field(default=0, ge=0, description="Chunks deleted")
    error_message: Optional[str] = Field(default=None, description="Error message if failed")


# -----------------------------------------------------------------------------
# Logging Entities
# -----------------------------------------------------------------------------


class RetrievalLog(BaseModel):
    """Audit log for a retrieval operation."""

    id: UUID = Field(default_factory=uuid4, description="Log entry ID")
    query_text: str = Field(..., description="Original query text")
    query_embedding_model: str = Field(..., description="Model used for query embedding")
    retrieved_chunk_ids: list[UUID] = Field(..., description="Retrieved chunk IDs")
    relevance_scores: list[float] = Field(..., description="Relevance scores")
    top_score: float = Field(..., description="Highest score")
    response_generated: bool = Field(
        default=True, description="Whether response was generated"
    )
    fallback_triggered: bool = Field(
        default=False, description="Whether fallback message was used"
    )
    latency_ms: int = Field(..., ge=0, description="Total latency")
    created_at: datetime = Field(
        default_factory=datetime.utcnow, description="Log timestamp"
    )
