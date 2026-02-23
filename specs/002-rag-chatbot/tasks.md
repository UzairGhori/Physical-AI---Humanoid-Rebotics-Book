# Tasks: Integrated RAG Chatbot

**Input**: Design documents from `/specs/002-rag-chatbot/`
**Prerequisites**: plan.md (required), spec.md (required), research.md, data-model.md, contracts/

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (US1, US2, US3, US4, INFRA)
- Include exact file paths in descriptions

## Path Conventions

- **Backend**: `Backend/app/`, `Backend/cli/`, `Backend/tests/`
- **Frontend**: `src/components/`, `static/js/`
- **Docs**: `docs/`

---

## Phase 1: Setup (Project Infrastructure)

**Purpose**: Initialize project structure, dependencies, and development environment

- [x] T001 [INFRA] Create Backend project structure per plan.md in `Backend/app/`
- [x] T002 [INFRA] Initialize pyproject.toml with dependencies: fastapi, uvicorn, openai, qdrant-client, asyncpg, pydantic
- [x] T003 [P] [INFRA] Create `.env.example` with all required environment variables in `Backend/`
- [x] T004 [P] [INFRA] Create `docker-compose.yml` for local development (Qdrant, Postgres) in repo root
- [x] T005 [P] [INFRA] Create `Dockerfile` for Backend service in `Backend/`
- [x] T006 [INFRA] Configure pytest and create `Backend/tests/conftest.py` with async fixtures

**Checkpoint**: Development environment ready; `docker-compose up` starts infrastructure

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

### Database Connections

- [x] T007 [INFRA] Implement Postgres connection pool in `Backend/app/db/postgres.py`
  - **Inputs**: DATABASE_URL from environment
  - **Outputs**: AsyncPG connection pool with health check
  - **Acceptance**: Pool connects, runs test query, handles reconnection

- [x] T008 [INFRA] Implement Qdrant client wrapper in `Backend/app/db/qdrant.py`
  - **Inputs**: QDRANT_URL, QDRANT_API_KEY from environment
  - **Outputs**: Configured QdrantClient with collection management
  - **Acceptance**: Client connects, creates collection if not exists

- [x] T009 [P] [INFRA] Create Alembic migration for `chunk_metadata` table in `Backend/app/db/migrations/`
  - **Inputs**: Schema from data-model.md
  - **Outputs**: Migration file creating chunk_metadata table
  - **Acceptance**: `alembic upgrade head` succeeds

- [x] T010 [P] [INFRA] Create Alembic migration for `ingestion_runs` table in `Backend/app/db/migrations/`

- [x] T011 [P] [INFRA] Create Alembic migration for `retrieval_logs` table in `Backend/app/db/migrations/`

- [x] T012 [P] [INFRA] Create Alembic migration for `system_metrics` table in `Backend/app/db/migrations/`

### Configuration & Models

- [x] T013 [INFRA] Implement configuration loader in `Backend/app/config.py`
  - **Inputs**: Environment variables, .env file
  - **Outputs**: Pydantic Settings class with all config
  - **Acceptance**: All required vars loaded, validation errors on missing

- [x] T014 [P] [INFRA] Create Pydantic request models in `Backend/app/models/requests.py`
  - **Inputs**: API contracts from contracts/api-contracts.md
  - **Outputs**: QueryRequest, IngestRequest models
  - **Acceptance**: Models validate per contract constraints

- [x] T015 [P] [INFRA] Create Pydantic response models in `Backend/app/models/responses.py`
  - **Inputs**: API contracts from contracts/api-contracts.md
  - **Outputs**: QueryResponse, IngestResponse, HealthStatus, SystemMetrics models
  - **Acceptance**: Models serialize to expected JSON structure

- [x] T016 [P] [INFRA] Create domain entity models in `Backend/app/models/entities.py`
  - **Inputs**: Data model from data-model.md
  - **Outputs**: BookChunk, EmbeddingResult, RetrievedChunk, ChatMessage classes
  - **Acceptance**: Entities match spec; validation works

### Utilities

- [x] T017 [P] [INFRA] Implement input sanitizer in `Backend/app/utils/sanitizer.py`
  - **Inputs**: Raw user query string
  - **Outputs**: Sanitized string (HTML stripped, length enforced)
  - **Acceptance**: XSS/injection attempts neutralized, 1000 char limit enforced

- [x] T018 [P] [INFRA] Implement structured logging in `Backend/app/utils/logging.py`
  - **Inputs**: Log level from config
  - **Outputs**: Configured structlog logger
  - **Acceptance**: JSON logs with request_id, timestamp, level

### FastAPI Application

- [x] T019 [INFRA] Create FastAPI application entry in `Backend/app/main.py`
  - **Inputs**: Config, route definitions
  - **Outputs**: FastAPI app with CORS, lifespan events
  - **Acceptance**: Server starts, /docs accessible, health endpoint responds

- [x] T020 [INFRA] Implement `/v1/health` endpoint in `Backend/app/api/health.py`
  - **Inputs**: Database connections
  - **Outputs**: HealthStatus response
  - **Acceptance**: Returns healthy/degraded/unhealthy based on component status

**Checkpoint**: Foundation ready - `uvicorn app.main:app` starts, /health returns status

---

## Phase 3: User Story 1 - Ask Question About Book Content (Priority: P1) 🎯 MVP

**Goal**: Users can ask questions and receive grounded answers from book content

**Independent Test**: Ask "What is ROS2?" and verify response within 3 seconds with source attribution

### Embedding Service

- [x] T021 [US1] Implement Cohere embedding client in `Backend/app/services/embedding.py`
  - **Inputs**: Text to embed, COHERE_API_KEY
  - **Outputs**: 1024-dim vector, latency_ms
  - **Acceptance**: Returns valid vector for sample text
- [c] T022 [US1] Implement Bonsai fallback embedding in `Backend/app/services/embedding.py`
- [c] T023 [US1] Implement primary-fallback embedding logic in `Backend/app/services/embedding.py`

### Retrieval Service

- [x] T024 [US1] Implement vector search in `Backend/app/services/retrieval.py`
  - **Inputs**: Query vector, top_k=5
  - **Outputs**: List of RetrievedChunk with relevance_score
  - **Acceptance**: Returns top-5 chunks sorted by relevance

- [x] T025 [US1] Implement relevance threshold check in `Backend/app/services/retrieval.py`
  - **Inputs**: RetrievalResult, threshold=0.7
  - **Outputs**: Boolean (is_relevant), confidence level (high/medium/low)
  - **Acceptance**: Correctly classifies per score ranges in plan.md

### Generation Service

- [x] T026 [US1] Implement OpenAI client setup in `Backend/app/services/generation.py`
  - **Inputs**: OPENAI_API_KEY, model="gpt-4o-mini"
  - **Outputs**: Configured OpenAI client
  - **Acceptance**: Client initializes, can call completions API

- [x] T027 [US1] Implement RAG prompt builder in `Backend/app/services/generation.py`
  - **Inputs**: User question, retrieved chunks
  - **Outputs**: System prompt + context-stuffed user message
  - **Acceptance**: Prompt includes grounding instructions, context, question

- [x] T028 [US1] Implement streaming response generator in `Backend/app/services/generation.py`
  - **Inputs**: Prompt, OpenAI client
  - **Outputs**: Async generator yielding tokens
  - **Acceptance**: First token in <500ms, full response streams

- [x] T029 [US1] Implement source attribution extraction in `Backend/app/services/generation.py`
  - **Inputs**: Retrieved chunks
  - **Outputs**: List of "Chapter X: Section Y" strings
  - **Acceptance**: Sources match retrieved chunk metadata

### Query Endpoint

- [x] T030 [US1] Implement `/v1/query` endpoint (non-streaming) in `Backend/app/api/query.py`
  - **Inputs**: QueryRequest body
  - **Outputs**: QueryResponse with answer, sources, confidence
  - **Acceptance**: Returns grounded answer within 3s

- [x] T031 [US1] Implement `/v1/query` endpoint (streaming) in `Backend/app/api/query.py`
  - **Inputs**: QueryRequest with stream=true
  - **Outputs**: SSE stream with tokens, sources, done event
  - **Acceptance**: Streams tokens via text/event-stream

- [x] T032 [US1] Implement retrieval logging in `Backend/app/api/query.py`
  - **Inputs**: Query, chunks, scores, latency
  - **Outputs**: Record in retrieval_logs table
  - **Acceptance**: Every query logged with full metadata

### Unit Tests for US1

- [x] T033 [P] [US1] Unit test for Cohere embedding service in `Backend/tests/unit/test_embedding.py`
  - **Acceptance**: Tests Cohere embedding success and failure

- [x] T034 [P] [US1] Unit test for sanitizer in `Backend/tests/unit/test_sanitizer.py`
  - **Acceptance**: Tests XSS stripping, length truncation, edge cases

**Checkpoint**: User Story 1 complete - can ask questions via API and receive grounded answers

---

## Phase 4: User Story 2 - Handle Out-of-Scope Questions (Priority: P2)

**Goal**: System gracefully declines questions not covered in book content

**Independent Test**: Ask "What's the best restaurant in NYC?" and verify fallback message

### Fallback Logic

- [x] T035 [US2] Implement fallback response in `Backend/app/services/generation.py`
  - **Inputs**: is_relevant=False from threshold check
  - **Outputs**: "I don't know based on the book content." message
  - **Acceptance**: Returns fallback when top_score < 0.7

- [x] T036 [US2] Implement uncertainty response in `Backend/app/services/generation.py`
  - **Inputs**: is_relevant=True but confidence=medium
  - **Outputs**: Answer with uncertainty note + suggested sections
  - **Acceptance**: Medium confidence answers include disclaimer

- [x] T037 [US2] Update query endpoint to handle fallback in `Backend/app/api/query.py`
  - **Inputs**: Low relevance retrieval result
  - **Outputs**: QueryResponse with confidence=low, empty sources
  - **Acceptance**: Fallback responses have generation_latency_ms=0

### Integration Tests for US2

- [x] T038 [US2] Integration test for out-of-scope handling in `Backend/tests/integration/test_generation.py`
  - **Acceptance**: 20 out-of-scope queries all return fallback message

**Checkpoint**: User Story 2 complete - out-of-scope questions handled gracefully

---

## Phase 5: User Story 3 - Book Content Ingestion (Priority: P3)

**Goal**: Administrators can ingest book content via CLI command

**Independent Test**: Run ingestion CLI on sample chapter, verify it appears in search results

### Chunking Service

- [ ] T039 [US3] Implement markdown file scanner in `Backend/app/services/ingestion.py`
  - **Inputs**: Source directory path
  - **Outputs**: List of .md/.mdx file paths
  - **Acceptance**: Finds all markdown files recursively

- [ ] T040 [US3] Implement markdown parser in `Backend/app/utils/chunking.py`
  - **Inputs**: Markdown file content
  - **Outputs**: Extracted chapter, sections, content
  - **Acceptance**: Extracts H1 as chapter, H2/H3 as sections

- [ ] T041 [US3] Implement text chunking in `Backend/app/utils/chunking.py`
  - **Inputs**: Parsed content, chunk_size=512 tokens, overlap=50 tokens
  - **Outputs**: List of BookChunk with position, token_count
  - **Acceptance**: Chunks respect markdown boundaries, overlap correct

- [ ] T042 [US3] Implement content hash calculation in `Backend/app/utils/chunking.py`
  - **Inputs**: Chunk content
  - **Outputs**: SHA-256 hash string
  - **Acceptance**: Same content produces same hash

### Ingestion Pipeline

- [ ] T043 [US3] Implement chunk deduplication in `Backend/app/services/ingestion.py`
  - **Inputs**: New chunks, existing chunk_metadata
  - **Outputs**: Lists of new, updated, unchanged chunks
  - **Acceptance**: Only changed content re-embedded

- [ ] T044 [US3] Implement batch embedding in `Backend/app/services/ingestion.py`
  - **Inputs**: List of BookChunk
  - **Outputs**: List of EmbeddingResult
  - **Acceptance**: Processes 100 chunks/min minimum

- [ ] T045 [US3] Implement Qdrant upsert in `Backend/app/services/ingestion.py`
  - **Inputs**: Chunks with vectors
  - **Outputs**: Upserted point IDs
  - **Acceptance**: Points created with correct payload

- [ ] T046 [US3] Implement Postgres metadata upsert in `Backend/app/services/ingestion.py`
  - **Inputs**: Chunks, vector IDs
  - **Outputs**: Updated chunk_metadata records
  - **Acceptance**: Metadata linked to vector IDs

- [ ] T047 [US3] Implement orphan cleanup in `Backend/app/services/ingestion.py`
  - **Inputs**: Current source files, existing metadata
  - **Outputs**: Deleted orphaned chunks
  - **Acceptance**: Chunks from removed files deleted

- [ ] T048 [US3] Implement ingestion run tracking in `Backend/app/services/ingestion.py`
  - **Inputs**: Ingestion stats
  - **Outputs**: ingestion_runs record
  - **Acceptance**: Run recorded with files_processed, chunks_created, etc.

### Ingestion API & CLI

- [ ] T049 [US3] Implement `/v1/ingest` POST endpoint in `Backend/app/api/ingest.py`
  - **Inputs**: IngestRequest (source_directory, force_reindex)
  - **Outputs**: 202 Accepted with run_id
  - **Acceptance**: Starts async ingestion, returns immediately

- [ ] T050 [US3] Implement `/v1/ingest/{run_id}` GET endpoint in `Backend/app/api/ingest.py`
  - **Inputs**: run_id path parameter
  - **Outputs**: Ingestion status and progress
  - **Acceptance**: Returns running/completed/failed with stats

- [ ] T051 [US3] Implement ingestion CLI in `Backend/cli/ingest.py`
  - **Inputs**: --source, --force flags
  - **Outputs**: Progress output, completion message
  - **Acceptance**: `python -m cli.ingest --source docs/` works

### Unit Tests for US3

- [ ] T052 [P] [US3] Unit test for chunking in `Backend/tests/unit/test_chunking.py`
  - **Acceptance**: Tests token counting, overlap, markdown boundaries

### Integration Tests for US3

- [ ] T053 [US3] Integration test for ingestion pipeline in `Backend/tests/integration/test_ingestion.py`
  - **Acceptance**: Ingest sample doc, verify in Qdrant and Postgres

**Checkpoint**: User Story 3 complete - full ingestion pipeline working

---

## Phase 6: User Story 4 - View Chat History (Priority: P4)

**Goal**: Users can see session chat history in the UI

**Independent Test**: Ask 5 questions, verify all visible; refresh page, verify cleared

### Frontend Chat Widget

- [ ] T054 [US4] Create ChatWidget React component in `src/components/ChatWidget/index.tsx`
  - **Inputs**: API base URL
  - **Outputs**: Floating chat widget with input, messages
  - **Acceptance**: Widget renders, accepts input, displays messages

- [ ] T055 [US4] Implement session storage logic in `src/components/ChatWidget/index.tsx`
  - **Inputs**: ChatSession interface
  - **Outputs**: Messages persisted in sessionStorage
  - **Acceptance**: History survives widget toggle, clears on page refresh

- [ ] T056 [US4] Implement streaming response display in `src/components/ChatWidget/index.tsx`
  - **Inputs**: SSE stream from /query
  - **Outputs**: Tokens displayed as they arrive
  - **Acceptance**: Smooth streaming display, sources shown after

- [ ] T057 [US4] Implement error and loading states in `src/components/ChatWidget/index.tsx`
  - **Inputs**: API errors, loading status
  - **Outputs**: User-friendly error messages, loading indicator
  - **Acceptance**: Network errors shown gracefully

- [ ] T058 [US4] Implement responsive styling in `src/components/ChatWidget/ChatWidget.module.css`
  - **Inputs**: Viewport width
  - **Outputs**: Mobile (320px) to desktop (2560px) layouts
  - **Acceptance**: Widget usable on all viewports per SC-008

- [ ] T059 [US4] Create TypeScript interfaces in `src/components/ChatWidget/types.ts`
  - **Inputs**: API response types
  - **Outputs**: ChatSession, ChatMessage, QueryResponse interfaces
  - **Acceptance**: Types match backend response models

### Docusaurus Integration

- [ ] T060 [US4] Create standalone embed script in `static/js/chat-embed.js`
  - **Inputs**: API_URL configuration
  - **Outputs**: Self-contained script that injects chat widget
  - **Acceptance**: Single script tag adds working chatbot

- [ ] T061 [US4] Integrate ChatWidget into Docusaurus in `docusaurus.config.js`
  - **Inputs**: ChatWidget component
  - **Outputs**: Widget appears on all book pages
  - **Acceptance**: Widget loads on every page

**Checkpoint**: User Story 4 complete - full frontend chat experience working

---

## Phase 7: Cross-Cutting Concerns & Polish

**Purpose**: Rate limiting, metrics, security hardening

### Rate Limiting

- [ ] T062 [INFRA] Implement request queue in `Backend/app/utils/queue.py`
  - **Inputs**: Max queue depth=50, timeout=30s
  - **Outputs**: Queued request handler
  - **Acceptance**: Returns queue position when at capacity

- [ ] T063 [INFRA] Integrate rate limiting with /query endpoint in `Backend/app/api/query.py`
  - **Inputs**: Queue, concurrent request count
  - **Outputs**: 202 Accepted with queue status when busy
  - **Acceptance**: Per-IP throttling (10 req/min)

### Metrics Endpoint

- [ ] T064 [INFRA] Implement `/v1/metrics` endpoint in `Backend/app/api/metrics.py`
  - **Inputs**: API key auth, system_metrics table
  - **Outputs**: SystemMetrics response
  - **Acceptance**: Returns indexing, query, retrieval, system stats

- [ ] T065 [INFRA] Implement metrics collection in `Backend/app/services/metrics.py`
  - **Inputs**: Query events, ingestion events
  - **Outputs**: Rolling metrics in system_metrics table
  - **Acceptance**: Metrics updated on each operation

### Security

- [ ] T066 [INFRA] Implement API key authentication for admin endpoints in `Backend/app/api/routes.py`
  - **Inputs**: X-API-Key header
  - **Outputs**: 401 on invalid/missing key
  - **Acceptance**: /ingest, /metrics protected; /query, /health public

- [ ] T067 [INFRA] Implement OpenRouter fallback for LLM in `Backend/app/services/generation.py`
  - **Inputs**: OPENROUTER_API_KEY, OpenAI failure
  - **Outputs**: Response from OpenRouter
  - **Acceptance**: Fallback triggers on OpenAI error

### Final Integration

- [ ] T068 [INFRA] Configure CORS for production domains in `Backend/app/main.py`
  - **Inputs**: CORS_ORIGINS from config
  - **Outputs**: Proper Access-Control headers
  - **Acceptance**: Docusaurus site can call API

- [ ] T069 [INFRA] Create production deployment config in `Backend/`
  - **Inputs**: Railway/Render requirements
  - **Outputs**: Procfile or railway.json
  - **Acceptance**: Deploys successfully to production

**Checkpoint**: All cross-cutting concerns addressed

---

## Phase 8: Testing & Validation

**Purpose**: Comprehensive testing against success criteria

### Contract Tests

- [ ] T070 [P] [TEST] Contract test for /query API in `Backend/tests/contract/test_query_api.py`
  - **Acceptance**: All request/response schemas validated per contracts

- [ ] T071 [P] [TEST] Contract test for /ingest API in `Backend/tests/contract/test_ingest_api.py`
  - **Acceptance**: All request/response schemas validated per contracts

- [ ] T072 [P] [TEST] Contract test for /health API in `Backend/tests/contract/test_health_api.py`
  - **Acceptance**: All response schemas validated per contracts

### Integration Tests

- [ ] T073 [TEST] Integration test for retrieval accuracy in `Backend/tests/integration/test_retrieval.py`
  - **Acceptance**: ≥80% precision on 50 curated Q&A pairs (SC-007)

- [ ] T074 [TEST] Integration test for grounding compliance in `Backend/tests/integration/test_generation.py`
  - **Acceptance**: 100% fallback rate on 20 out-of-scope queries (SC-003)

- [ ] T075 [TEST] End-to-end latency test in `Backend/tests/integration/test_latency.py`
  - **Acceptance**: p95 latency <3s on 100 queries (SC-001)

### Load Tests

- [ ] T076 [TEST] Load test for concurrent users in `Backend/tests/load/test_concurrent.py`
  - **Acceptance**: 100 concurrent users, no degradation (SC-005)

### Security Tests

- [ ] T077 [TEST] Security test for injection prevention in `Backend/tests/security/test_injection.py`
  - **Acceptance**: XSS, SQL injection attempts neutralized

- [ ] T078 [TEST] Security test for API key auth in `Backend/tests/security/test_auth.py`
  - **Acceptance**: Admin endpoints reject invalid keys

**Checkpoint**: All success criteria validated

---

## Phase 9: Documentation & Deployment

**Purpose**: Final documentation and production deployment

- [ ] T079 [INFRA] Update Backend README with setup instructions in `Backend/README.md`
- [ ] T080 [INFRA] Ingest full book content using CLI
- [ ] T081 [INFRA] Deploy Backend to production (Railway/Render)
- [ ] T082 [INFRA] Deploy Docusaurus site with chat widget to GitHub Pages
- [ ] T083 [INFRA] Verify production health and run smoke tests

**Checkpoint**: System live in production

---

## Dependencies & Execution Order

### Phase Dependencies

```
Phase 1: Setup ────────────────────────────────────────────────▶ No dependencies
         │
         ▼
Phase 2: Foundational ─────────────────────────────────────────▶ Blocks all stories
         │
         ├──────────────────────────────────────────────────────┐
         ▼                                                      ▼
Phase 3: US1 (P1) ─────▶ Phase 4: US2 (P2) ──────▶ Phase 7: Cross-cutting
         │                        │                            │
         ▼                        ▼                            ▼
Phase 5: US3 (P3) ─────▶ Phase 6: US4 (P4) ──────▶ Phase 8: Testing
                                                               │
                                                               ▼
                                                  Phase 9: Documentation
```

### Parallel Opportunities

- **Phase 1**: T003, T004, T005 can run in parallel
- **Phase 2**: T009-T012 (migrations) can run in parallel; T014-T018 can run in parallel
- **Phase 3**: T033, T034 (unit tests) can run in parallel with implementation
- **Phase 8**: T070-T072 (contract tests) can run in parallel

### MVP Definition

**Minimum Viable Product** = Phase 1 + Phase 2 + Phase 3 (User Story 1)

After MVP, add stories incrementally:
1. US2 (out-of-scope handling) - Low effort, high trust impact
2. US3 (ingestion pipeline) - Required for content updates
3. US4 (frontend UI) - Full user experience

---

## Task Summary

| Phase | Tasks | Effort Estimate |
|-------|-------|-----------------|
| Phase 1: Setup | 6 | 4 hours |
| Phase 2: Foundational | 14 | 8 hours |
| Phase 3: US1 (P1) | 14 | 12 hours |
| Phase 4: US2 (P2) | 4 | 3 hours |
| Phase 5: US3 (P3) | 15 | 10 hours |
| Phase 6: US4 (P4) | 8 | 8 hours |
| Phase 7: Cross-cutting | 8 | 6 hours |
| Phase 8: Testing | 9 | 6 hours |
| Phase 9: Documentation | 5 | 3 hours |
| **Total** | **83** | **~60 hours** |

---

## Notes

- [P] tasks = different files, no dependencies, can parallelize
- [Story] label maps task to specific user story for traceability
- Each checkpoint validates phase completion before proceeding
- Commit after each task or logical group
- Run tests frequently to catch regressions early
