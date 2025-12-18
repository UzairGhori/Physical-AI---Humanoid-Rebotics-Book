# Implementation Plan: RAG Chatbot

**Branch**: `002-rag-chatbot` | **Date**: 2025-12-17 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/002-rag-chatbot/spec.md`

## Summary

Build an integrated RAG chatbot for the Physical AI & Humanoid Robotics book using:
- **FastAPI** backend with OpenAI Python SDK for agent/chat logic
- **Cohere** embedding model
- **Qdrant** vector database for semantic search
- **Neon Postgres** for metadata storage
- **Docusaurus** frontend with embedded chat UI

The chatbot answers questions exclusively from book content, with source attribution and graceful out-of-scope handling.

## Technical Context

**Language/Version**: Python 3.11+
**Primary Dependencies**: FastAPI, OpenAI SDK, qdrant-client, asyncpg, pydantic
**Storage**: Qdrant (vectors), Neon Postgres (metadata)
**Testing**: pytest, pytest-asyncio, httpx (contract tests)
**Target Platform**: Linux server (Docker), Docusaurus static site
**Project Type**: Web application (backend + frontend integration)
**Performance Goals**: <3s query latency, 100 concurrent users
**Constraints**: <100ms vector search, 99% availability
**Scale/Scope**: 50+ chapters, ~3000 chunks, 1000+ daily queries

## Constitution Check

*GATE: Must pass before implementation. All items verified.*

| Principle | Status | Implementation |
|-----------|--------|----------------|
| Spec-First Development | ✅ Pass | Spec completed and clarified |
| Clarity & Consistency | ✅ Pass | API contracts defined, data model documented |
| Accuracy Over Speed | ✅ Pass | Grounding check before response generation |
| Iterative Improvement | ✅ Pass | Re-ingestion CLI for content updates |
| Transparency & Grounding | ✅ Pass | Out-of-scope fallback message defined |
| RAG-Grounded Responses | ✅ Pass | Relevance threshold (0.7), source attribution |
| SDK-First Architecture | ✅ Pass | OpenAI Python SDK for all LLM interactions |

## Project Structure

### Documentation (this feature)

```text
specs/002-rag-chatbot/
├── spec.md              # Feature specification
├── plan.md              # This file
├── research.md          # Technical research findings
├── data-model.md        # Database schema and entities
├── quickstart.md        # Developer setup guide
├── contracts/
│   └── api-contracts.md # REST API definitions
└── tasks.md             # Implementation tasks (via /sp.tasks)
```

### Source Code (repository root)

```text
Backend/
├── app/
│   ├── __init__.py
│   ├── main.py              # FastAPI application entry
│   ├── config.py            # Environment configuration
│   ├── models/
│   │   ├── __init__.py
│   │   ├── requests.py      # Pydantic request models
│   │   ├── responses.py     # Pydantic response models
│   │   └── entities.py      # Domain entities (BookChunk, etc.)
│   ├── services/
│   │   ├── __init__.py
│   │   ├── embedding.py     # Cohere embedding service
│   │   ├── retrieval.py     # Qdrant search service
│   │   ├── generation.py    # OpenAI response generation
│   │   └── ingestion.py     # Content ingestion pipeline
│   ├── api/
│   │   ├── __init__.py
│   │   ├── routes.py        # API route definitions
│   │   ├── query.py         # /query endpoint handler
│   │   ├── ingest.py        # /ingest endpoint handler
│   │   ├── health.py        # /health endpoint handler
│   │   └── metrics.py       # /metrics endpoint handler
│   ├── db/
│   │   ├── __init__.py
│   │   ├── postgres.py      # Asyncpg connection pool
│   │   ├── qdrant.py        # Qdrant client wrapper
│   │   └── migrations/      # Alembic migrations
│   └── utils/
│       ├── __init__.py
│       ├── chunking.py      # Markdown chunking logic
│       ├── sanitizer.py     # Input sanitization
│       └── logging.py       # Structured logging
├── cli/
│   ├── __init__.py
│   └── ingest.py            # CLI for manual ingestion
├── tests/
│   ├── __init__.py
│   ├── conftest.py          # Pytest fixtures
│   ├── unit/
│   │   ├── test_chunking.py
│   │   ├── test_embedding.py
│   │   └── test_sanitizer.py
│   ├── integration/
│   │   ├── test_retrieval.py
│   │   ├── test_ingestion.py
│   │   └── test_generation.py
│   └── contract/
│       ├── test_query_api.py
│       ├── test_ingest_api.py
│       └── test_health_api.py
├── pyproject.toml           # Project dependencies
├── Dockerfile               # Container build
├── docker-compose.yml       # Local development stack
└── .env.example             # Environment template

src/components/
└── ChatWidget/
    ├── index.tsx            # Chat widget component
    ├── ChatWidget.module.css
    └── types.ts             # TypeScript interfaces

static/
└── js/
    └── chat-embed.js        # Standalone embed script
```

**Structure Decision**: Web application structure with separated Backend (FastAPI) and Frontend integration (Docusaurus components). Backend follows clean architecture with distinct layers: API → Services → Database.

## Architecture Overview

### System Architecture Diagram

```
┌─────────────────────────────────────────────────────────────────────────┐
│                           DOCUSAURUS SITE                               │
│  ┌─────────────────────────────────────────────────────────────────┐   │
│  │                        Chat Widget (React)                       │   │
│  │  ┌───────────┐    ┌───────────┐    ┌───────────────────────┐   │   │
│  │  │  Input    │───▶│  Submit   │───▶│  Display Response     │   │   │
│  │  │  Field    │    │  Button   │    │  + Sources            │   │   │
│  │  └───────────┘    └───────────┘    └───────────────────────┘   │   │
│  │                         │                                       │   │
│  │                         │ HTTP POST /query                      │   │
│  │                         ▼                                       │   │
│  └─────────────────────────┼───────────────────────────────────────┘   │
└─────────────────────────────┼───────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                         FASTAPI BACKEND                                 │
│  ┌─────────────────────────────────────────────────────────────────┐   │
│  │                        API Layer                                 │   │
│  │  /query    /ingest    /health    /metrics                       │   │
│  └─────────────────────────┬───────────────────────────────────────┘   │
│                             │                                           │
│  ┌─────────────────────────┼───────────────────────────────────────┐   │
│  │                    Service Layer                                 │   │
│  │  ┌──────────┐   ┌──────────┐   ┌──────────┐   ┌──────────┐     │   │
│  │  │Embedding │   │Retrieval │   │Generation│   │Ingestion │     │   │
│  │  │ Service  │   │ Service  │   │ Service  │   │ Service  │     │   │
│  │  └────┬─────┘   └────┬─────┘   └────┬─────┘   └────┬─────┘     │   │
│  └───────┼──────────────┼──────────────┼──────────────┼───────────┘   │
│          │              │              │              │                 │
│          ▼              ▼              ▼              │                 │
│  ┌───────────────┐ ┌───────────┐ ┌───────────┐      │                 │
│  │ Cohere        │ │  Qdrant   │ │  OpenAI   │      │                 │
│  │ (Embeddings)  │ │ (Vectors) │ │  (LLM)    │      │                 │
│  └───────────────┘ └───────────┘ └───────────┘      │                 │
│                           │                          │                 │
│                           ▼                          │                 │
│                    ┌───────────┐                     │                 │
│                    │   Neon    │◀────────────────────┘                 │
│                    │ Postgres  │                                       │
│                    └───────────┘                                       │
└─────────────────────────────────────────────────────────────────────────┘
```

### Sequence Diagrams

#### Query Flow

```
User        ChatWidget      FastAPI       EmbeddingSvc    Qdrant      OpenAI
 │              │              │              │             │            │
 │─ Ask Q ─────▶│              │              │             │            │
 │              │─ POST /query▶│              │             │            │
 │              │              │─ embed(Q) ──▶│             │            │
 │              │              │◀─ vector ────│             │            │
 │              │              │─ search(vec)────────────────▶│          │
 │              │              │◀─ top 5 chunks ──────────────│          │
 │              │              │                              │            │
 │              │              │── Check relevance ─────────────────────▶│
 │              │              │   (score ≥ 0.7?)             │            │
 │              │              │                              │            │
 │              │              │─ generate(context + Q) ──────────────────▶│
 │              │              │◀─ stream tokens ─────────────────────────│
 │              │◀─ SSE tokens─│              │             │            │
 │◀─ Display ───│              │              │             │            │
 │              │              │              │             │            │
```

#### Ingestion Flow

```
Admin       CLI         FastAPI      ChunkingSvc    EmbeddingSvc    Qdrant    Postgres
  │          │             │              │              │            │           │
  │─ ingest ▶│             │              │              │            │           │
  │          │─POST /ingest▶│             │              │            │           │
  │          │             │─ scan files ▶│              │            │           │
  │          │             │◀─ file list ─│              │            │           │
  │          │             │              │              │            │           │
  │          │             │── For each file: ──────────────────────────────────▶│
  │          │             │   │                         │            │           │
  │          │             │   │─ chunk(content) ───────▶│            │           │
  │          │             │   │◀─ chunks ───────────────│            │           │
  │          │             │   │                         │            │           │
  │          │             │   │─ embed(chunk) ──────────▶│           │           │
  │          │             │   │◀─ vector ───────────────│            │           │
  │          │             │   │                         │            │           │
  │          │             │   │─ upsert(vector) ─────────────────────▶│          │
  │          │             │   │─ upsert(metadata) ───────────────────────────────▶│
  │          │             │   │                         │            │           │
  │          │◀─ 202 ──────│◀──┘                         │            │           │
  │◀─ done ──│             │              │              │            │           │
```

## Technical Decisions

### 1. Embedding Strategy

**Decision**: Use Cohere for embeddings.

**Implementation**:
```python
import cohere
from app.models.entities import EmbeddingResult

class EmbeddingService:
    def __init__(self, api_key: str, model: str = "embed-english-v3.0"):
        self._client = cohere.AsyncClient(api_key)
        self._model = model

    async def embed(self, text: str) -> EmbeddingResult:
        response = await self._client.embed(
            texts=[text],
            model=self._model,
            input_type="search_document"
        )
        return EmbeddingResult(
            vector=response.embeddings[0],
            model_used="cohere",
            latency_ms=...
        )
```

**Rationale**: Simplifies the architecture by using a single, high-quality embedding service.

### 2. Streaming Responses

**Decision**: Enable SSE streaming by default for `/query`

**Implementation**: FastAPI StreamingResponse with async generator

**Rationale**: Reduces perceived latency; first token visible in <500ms vs 2-3s for full response.

### 3. Chunking Parameters

**Decision**: 512 tokens with 50-token overlap

| Parameter | Value | Rationale |
|-----------|-------|-----------|
| Chunk size | ~512 tokens (~2000 chars) | Fits context window, semantically coherent |
| Overlap | ~50 tokens (~200 chars) | Prevents boundary context loss |
| Splitter | Markdown-aware recursive | Respects document structure |

### 4. Relevance Threshold

**Decision**: 0.7 cosine similarity for "confident" responses

| Score Range | Confidence | Behavior |
|-------------|------------|----------|
| ≥ 0.8 | High | Full answer with sources |
| 0.7 - 0.8 | Medium | Answer with uncertainty note |
| < 0.7 | Low | Fallback: "I don't know based on the book content" |

### 5. Rate Limiting Strategy

**Decision**: Queue with feedback (per spec clarification)

**Implementation**:
- In-memory queue (asyncio.Queue) for MVP
- Max queue depth: 50 requests
- Display queue position to user
- Timeout: 30 seconds

### 6. Session Management

**Decision**: Browser sessionStorage only (no server-side persistence)

**Rationale**: Simplifies architecture; out-of-scope for MVP per spec.

## API Endpoints Summary

| Endpoint | Method | Auth | Purpose |
|----------|--------|------|---------|
| `/v1/query` | POST | None | Submit question, get answer |
| `/v1/ingest` | POST | API Key | Trigger content ingestion |
| `/v1/ingest/{run_id}` | GET | API Key | Check ingestion progress |
| `/v1/health` | GET | None | System health check |
| `/v1/metrics` | GET | API Key | Performance metrics |

Full contracts: [contracts/api-contracts.md](./contracts/api-contracts.md)

## Data Model Summary

### Qdrant Collection: `book_chunks`

- Vector size: 1024 dimensions
- Distance: Cosine
- Payload: chunk_id, content, chapter, section, position

### Postgres Tables

| Table | Purpose |
|-------|---------|
| `chunk_metadata` | Links vector IDs to source files |
| `ingestion_runs` | Tracks ingestion history |
| `retrieval_logs` | Query audit trail |
| `system_metrics` | Rolling performance data |

Full schema: [data-model.md](./data-model.md)

## Testing Strategy

### Test Categories

| Category | Scope | Tools |
|----------|-------|-------|
| Unit | Individual functions | pytest |
| Integration | Service interactions | pytest-asyncio, testcontainers |
| Contract | API compliance | httpx, pytest |
| Load | Performance targets | locust |
| Security | Injection, auth | manual + automated |

### Key Test Cases

1. **Retrieval Accuracy**: Known questions return expected chunks (≥80% precision)
2. **Grounding Compliance**: Out-of-scope queries return fallback (100%)
3. **Latency**: End-to-end <3s under normal load
4. **Concurrent Users**: 100 simultaneous queries without degradation
5. **Input Sanitization**: Script injection is neutralized

### Test Data

- Sample book content: 10 chapters for integration tests
- Known Q&A pairs: 50 curated for accuracy testing
- Out-of-scope queries: 20 for fallback testing

## Deployment Architecture

### Local Development

```yaml
# docker-compose.yml
services:
  api:
    build: ./Backend
    ports: ["8000:8000"]
    env_file: .env
    depends_on: [qdrant, postgres]

  qdrant:
    image: qdrant/qdrant:latest
    ports: ["6333:6333"]
    volumes: [qdrant_data:/qdrant/storage]

  postgres:
    image: postgres:15
    environment:
      POSTGRES_DB: ragchat
      POSTGRES_USER: dev
      POSTGRES_PASSWORD: dev
    ports: ["5432:5432"]
```

### Production

| Component | Service | Configuration |
|-----------|---------|---------------|
| FastAPI | Railway/Render | 2+ workers, auto-scale |
| Qdrant | Qdrant Cloud | Managed, replicated |
| Postgres | Neon | Serverless, auto-scale |
| Docusaurus | GitHub Pages | Static hosting |

### Environment Variables

```bash
# Required
OPENAI_API_KEY=sk-...
COHERE_API_KEY=...
QDRANT_URL=https://xxx.qdrant.io
QDRANT_API_KEY=...
DATABASE_URL=postgres://...

# Optional
OPENROUTER_API_KEY=sk-or-...  # Fallback LLM
LOG_LEVEL=INFO
CORS_ORIGINS=https://your-site.github.io
```

## Performance Targets

| Metric | Target | Measurement |
|--------|--------|-------------|
| Query latency (p50) | <2s | End-to-end |
| Query latency (p95) | <3s | End-to-end |
| Vector search | <100ms | Qdrant query |
| First token | <500ms | Streaming start |
| Concurrent users | 100 | Without degradation |
| Availability | 99% | Uptime monitoring |
| Ingestion speed | 100 chunks/min | Batch processing |

## Risk Mitigation

| Risk | Mitigation |
|------|------------|
| OpenAI rate limits | OpenRouter fallback + request queue |
| High latency | Streaming responses, caching |
| Data inconsistency | Transactional ingestion, content hashing |
| Prompt injection | System prompt hardening, input sanitization |

## Dependencies

### Python Packages

```txt
fastapi>=0.109.0
uvicorn[standard]>=0.27.0
openai>=1.12.0
qdrant-client>=1.7.0
asyncpg>=0.29.0
pydantic>=2.6.0
pydantic-settings>=2.1.0
python-dotenv>=1.0.0
httpx>=0.26.0
tiktoken>=0.6.0
alembic>=1.13.0
structlog>=24.1.0
```

### External Services

| Service | Purpose | Fallback |
|---------|---------|----------|
| OpenAI API | LLM generation | OpenRouter |
| Cohere API | Embeddings | None |
| Qdrant Cloud | Vector storage | Self-hosted |
| Neon Postgres | Metadata | Standard Postgres |

## Next Steps

1. Run `/sp.tasks` to generate implementation task list
2. Set up development environment per [quickstart.md](./quickstart.md)
3. Implement Phase 1: Setup (project structure, dependencies)
4. Implement Phase 2: Foundational (database connections, base models)
5. Implement User Stories in priority order (P1 → P2 → P3 → P4)

---

📋 **Architectural decision detected**: Technology stack selection (FastAPI + Qdrant + Neon + OpenAI SDK) represents a significant architectural choice.
Document reasoning and tradeoffs? Run `/sp.adr "RAG Chatbot Technology Stack Selection"`
