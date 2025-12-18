# Data Model: RAG Chatbot

**Feature**: 002-rag-chatbot
**Date**: 2025-12-17
**Status**: Complete

## Overview

This document defines the data structures for the RAG Chatbot system, spanning three storage layers:
1. **Qdrant** - Vector embeddings and payload data
2. **Neon Postgres** - Relational metadata and logs
3. **Browser Session** - Client-side chat state

## Entity Relationship Diagram

```
┌─────────────────────────────────────────────────────────────────────┐
│                           INGESTION FLOW                            │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│  ┌──────────────┐     ┌──────────────┐     ┌──────────────┐        │
│  │  BookSource  │────▶│  BookChunk   │────▶│  Embedding   │        │
│  │  (MD/MDX)    │  1:N│              │  1:1│  (Qdrant)    │        │
│  └──────────────┘     └──────────────┘     └──────────────┘        │
│                              │                    │                 │
│                              │ 1:1                │ 1:1             │
│                              ▼                    │                 │
│                       ┌──────────────┐            │                 │
│                       │ChunkMetadata │◀───────────┘                 │
│                       │ (Postgres)   │                              │
│                       └──────────────┘                              │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────────┐
│                            QUERY FLOW                               │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│  ┌──────────────┐     ┌──────────────┐     ┌──────────────┐        │
│  │  UserQuery   │────▶│RetrievalResult────▶│ ChatMessage  │        │
│  │              │     │              │     │              │        │
│  └──────────────┘     └──────────────┘     └──────────────┘        │
│         │                    │                    │                 │
│         │                    │                    │ N:1             │
│         │                    ▼                    ▼                 │
│         │             ┌──────────────┐     ┌──────────────┐        │
│         └────────────▶│RetrievalLog  │     │ ChatSession  │        │
│                       │ (Postgres)   │     │ (Browser)    │        │
│                       └──────────────┘     └──────────────┘        │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

## Qdrant Vector Store Schema

### Collection: `book_chunks`

**Vector Configuration**:
```json
{
  "vectors": {
    "size": 1024,
    "distance": "Cosine",
    "on_disk": true
  },
  "optimizers_config": {
    "indexing_threshold": 10000
  },
  "hnsw_config": {
    "m": 16,
    "ef_construct": 100
  }
}
```

**Point Structure**:
```json
{
  "id": "uuid-v4",
  "vector": [0.123, -0.456, ...],  // 1024 dimensions
  "payload": {
    "chunk_id": "string",
    "content": "string (max 4000 chars)",
    "source_file": "string",
    "chapter": "string",
    "section": "string | null",
    "position": "integer",
    "char_count": "integer",
    "token_count": "integer",
    "created_at": "ISO8601 timestamp",
    "embedding_model": "cohere"
  }
}
```

**Payload Indexes** (for filtered search):
```json
{
  "chapter": { "type": "keyword", "is_tenant": false },
  "section": { "type": "keyword", "is_tenant": false },
  "source_file": { "type": "keyword", "is_tenant": false }
}
```

## Neon Postgres Schema

### Table: `chunk_metadata`

Stores detailed metadata for each indexed chunk, linked by vector ID.

```sql
CREATE TABLE chunk_metadata (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    vector_id UUID NOT NULL UNIQUE,
    chunk_id VARCHAR(255) NOT NULL UNIQUE,
    source_file VARCHAR(500) NOT NULL,
    chapter VARCHAR(255) NOT NULL,
    section VARCHAR(255),
    position INTEGER NOT NULL,
    content_hash VARCHAR(64) NOT NULL,  -- SHA-256 for dedup
    char_count INTEGER NOT NULL,
    token_count INTEGER NOT NULL,
    embedding_model VARCHAR(50) NOT NULL DEFAULT 'qwen',
    created_at TIMESTAMPTZ NOT NULL DEFAULT NOW(),
    updated_at TIMESTAMPTZ NOT NULL DEFAULT NOW(),

    CONSTRAINT valid_position CHECK (position >= 0),
    CONSTRAINT valid_counts CHECK (char_count > 0 AND token_count > 0)
);

CREATE INDEX idx_chunk_metadata_chapter ON chunk_metadata(chapter);
CREATE INDEX idx_chunk_metadata_source ON chunk_metadata(source_file);
CREATE INDEX idx_chunk_metadata_hash ON chunk_metadata(content_hash);
```

### Table: `ingestion_runs`

Tracks each ingestion execution for auditability.

```sql
CREATE TABLE ingestion_runs (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    started_at TIMESTAMPTZ NOT NULL DEFAULT NOW(),
    completed_at TIMESTAMPTZ,
    status VARCHAR(20) NOT NULL DEFAULT 'running',  -- running, completed, failed
    source_directory VARCHAR(500) NOT NULL,
    files_processed INTEGER DEFAULT 0,
    chunks_created INTEGER DEFAULT 0,
    chunks_updated INTEGER DEFAULT 0,
    chunks_deleted INTEGER DEFAULT 0,
    error_message TEXT,

    CONSTRAINT valid_status CHECK (status IN ('running', 'completed', 'failed'))
);

CREATE INDEX idx_ingestion_runs_status ON ingestion_runs(status);
CREATE INDEX idx_ingestion_runs_started ON ingestion_runs(started_at DESC);
```

### Table: `retrieval_logs`

Audit trail for debugging retrieval quality.

```sql
CREATE TABLE retrieval_logs (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    query_text TEXT NOT NULL,
    query_embedding_model VARCHAR(50) NOT NULL,
    retrieved_chunk_ids UUID[] NOT NULL,
    relevance_scores FLOAT[] NOT NULL,
    top_score FLOAT NOT NULL,
    response_generated BOOLEAN NOT NULL DEFAULT true,
    fallback_triggered BOOLEAN NOT NULL DEFAULT false,
    latency_ms INTEGER NOT NULL,
    created_at TIMESTAMPTZ NOT NULL DEFAULT NOW(),

    CONSTRAINT scores_match_chunks CHECK (
        array_length(retrieved_chunk_ids, 1) = array_length(relevance_scores, 1)
    )
);

CREATE INDEX idx_retrieval_logs_created ON retrieval_logs(created_at DESC);
CREATE INDEX idx_retrieval_logs_fallback ON retrieval_logs(fallback_triggered)
    WHERE fallback_triggered = true;
```

### Table: `system_metrics`

Rolling metrics for the `/metrics` endpoint.

```sql
CREATE TABLE system_metrics (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    metric_name VARCHAR(100) NOT NULL,
    metric_value FLOAT NOT NULL,
    recorded_at TIMESTAMPTZ NOT NULL DEFAULT NOW()
);

CREATE INDEX idx_metrics_name_time ON system_metrics(metric_name, recorded_at DESC);

-- Retention: Auto-delete metrics older than 7 days
-- (Implement via pg_cron or application-level cleanup)
```

## Application-Level Data Structures

### Python Pydantic Models

```python
from pydantic import BaseModel, Field
from datetime import datetime
from uuid import UUID
from typing import Optional

# ─────────────────────────────────────────────────────────────
# Ingestion Models
# ─────────────────────────────────────────────────────────────

class BookChunk(BaseModel):
    """A segment of book content prepared for embedding."""
    chunk_id: str = Field(..., description="Unique identifier for the chunk")
    content: str = Field(..., max_length=4000)
    source_file: str = Field(..., description="Path to source MD/MDX file")
    chapter: str
    section: Optional[str] = None
    position: int = Field(..., ge=0)
    char_count: int = Field(..., gt=0)
    token_count: int = Field(..., gt=0)

class EmbeddingResult(BaseModel):
    """Result of embedding generation."""
    chunk_id: str
    vector: list[float] = Field(..., min_length=1024, max_length=1024)
    model_used: str = Field(..., pattern="^cohere$")
    latency_ms: int

# ─────────────────────────────────────────────────────────────
# Query Models
# ─────────────────────────────────────────────────────────────

class QueryRequest(BaseModel):
    """User query input."""
    question: str = Field(..., min_length=1, max_length=1000)
    session_id: Optional[str] = None

class RetrievedChunk(BaseModel):
    """A chunk returned from vector search."""
    chunk_id: str
    content: str
    chapter: str
    section: Optional[str]
    relevance_score: float = Field(..., ge=0.0, le=1.0)

class RetrievalResult(BaseModel):
    """Complete retrieval response."""
    query: str
    chunks: list[RetrievedChunk] = Field(..., max_length=5)
    top_score: float
    model_used: str
    latency_ms: int

class ChatMessage(BaseModel):
    """A single message in conversation."""
    role: str = Field(..., pattern="^(user|assistant)$")
    content: str
    timestamp: datetime = Field(default_factory=datetime.utcnow)
    sources: Optional[list[str]] = None  # Chapter/section refs for assistant

class QueryResponse(BaseModel):
    """Complete response to user query."""
    answer: str
    sources: list[str]  # ["Chapter 1: Introduction", "Chapter 3: ROS2"]
    confidence: str = Field(..., pattern="^(high|medium|low)$")
    retrieval_latency_ms: int
    generation_latency_ms: int

# ─────────────────────────────────────────────────────────────
# Ingestion API Models
# ─────────────────────────────────────────────────────────────

class IngestRequest(BaseModel):
    """Request to ingest book content."""
    source_directory: str = Field(default="docs/")
    force_reindex: bool = Field(default=False)

class IngestResponse(BaseModel):
    """Response from ingestion process."""
    run_id: UUID
    status: str
    files_processed: int
    chunks_created: int
    chunks_updated: int
    duration_seconds: float

# ─────────────────────────────────────────────────────────────
# Health & Metrics Models
# ─────────────────────────────────────────────────────────────

class HealthStatus(BaseModel):
    """System health check response."""
    status: str = Field(..., pattern="^(healthy|degraded|unhealthy)$")
    qdrant_connected: bool
    postgres_connected: bool
    openai_available: bool
    embedding_service: str  # "qwen" | "bonsai" | "unavailable"
    timestamp: datetime

class SystemMetrics(BaseModel):
    """System performance metrics."""
    total_chunks_indexed: int
    queries_last_24h: int
    avg_query_latency_ms: float
    avg_retrieval_precision: float
    fallback_rate_percent: float
    uptime_hours: float
```

## Browser Session Storage

Chat session state stored in `sessionStorage` (cleared on tab close).

### Key: `rag_chat_session`

```typescript
interface ChatSession {
  sessionId: string;        // UUID generated on first message
  messages: ChatMessage[];  // Conversation history
  createdAt: string;        // ISO8601
  lastActivityAt: string;   // ISO8601
}

interface ChatMessage {
  role: 'user' | 'assistant';
  content: string;
  timestamp: string;
  sources?: string[];       // Only for assistant messages
}
```

**Storage Limits**:
- Max messages per session: 50 (oldest trimmed on overflow)
- Max content length per message: 10,000 characters
- Session timeout: None (cleared only on tab/browser close)

## Data Flow Sequences

### Ingestion Sequence

```
1. CLI triggers ingestion → POST /ingest
2. Scan source_directory for .md/.mdx files
3. For each file:
   a. Parse markdown, extract chapter/section headings
   b. Split into chunks (~512 tokens, 50 overlap)
   c. Calculate content_hash (SHA-256)
   d. Check existing hash in chunk_metadata
   e. If new/changed:
      - Generate embedding (Qwen primary, Bonsai fallback)
      - Upsert point to Qdrant
      - Upsert metadata to Postgres
4. Delete orphaned chunks (removed from source)
5. Update ingestion_runs record
6. Return IngestResponse
```

### Query Sequence

```
1. User submits question → POST /query
2. Validate input (length, sanitization)
3. Generate query embedding (Qwen/Bonsai)
4. Search Qdrant for top-5 similar chunks
5. Check top_score against threshold (0.7)
   - Below threshold: Return fallback message
   - Above threshold: Continue
6. Build prompt with retrieved context
7. Call OpenAI API (stream response)
8. Log to retrieval_logs
9. Return QueryResponse (or stream tokens)
```

## Indexing Strategy

### Qdrant Indexes

| Field | Index Type | Purpose |
|-------|------------|---------|
| `chapter` | Keyword | Filter by chapter |
| `section` | Keyword | Filter by section |
| `source_file` | Keyword | Re-index single file |

### Postgres Indexes

| Table | Index | Purpose |
|-------|-------|---------|
| `chunk_metadata` | `chapter` | Join with Qdrant results |
| `chunk_metadata` | `content_hash` | Deduplication check |
| `retrieval_logs` | `created_at DESC` | Recent logs query |
| `ingestion_runs` | `started_at DESC` | Latest run lookup |

## Data Retention Policy

| Data Type | Retention | Cleanup Method |
|-----------|-----------|----------------|
| Vector embeddings | Indefinite | Manual re-index |
| Chunk metadata | Indefinite | Cascade from vectors |
| Retrieval logs | 30 days | Scheduled DELETE |
| System metrics | 7 days | Scheduled DELETE |
| Browser sessions | Session duration | Browser clears |

## Migration Notes

1. **Initial Setup**: Run `alembic upgrade head` for Postgres schema
2. **Qdrant Collection**: Created on first ingestion if not exists
3. **Version Tracking**: Alembic migrations in `Backend/migrations/`
