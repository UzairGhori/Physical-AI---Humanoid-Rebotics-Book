# ADR-003: RAG Data Persistence Stack

> **Scope**: This ADR documents the integrated data persistence stack for vector storage and relational metadata, including the choice of managed services vs self-hosted options.

- **Status:** Accepted
- **Date:** 2025-12-17
- **Feature:** 002-rag-chatbot
- **Context:** The RAG chatbot requires two types of data storage: (1) vector embeddings for semantic search of book content, and (2) relational metadata for chunk attribution, ingestion tracking, and query logging. The constitution mandates Qdrant for vectors and Neon Postgres for metadata. We need to handle ~3000 vectors with <100ms search latency and support 1000+ daily queries with audit trails.

## Decision

We will use the following integrated data persistence stack:

- **Vector Storage**: Qdrant Cloud (managed)
  - 1536-dimension vectors (matching embedding models)
  - Cosine similarity distance metric
  - HNSW index with m=16, ef_construct=100
  - Payload indexing on chapter, section for filtered search
  - Collection: `book_chunks`

- **Relational Metadata**: Neon Serverless Postgres
  - Connection pooling via built-in PgBouncer
  - Async driver: asyncpg for FastAPI integration
  - SSL required for all connections
  - Tables: chunk_metadata, ingestion_runs, retrieval_logs, system_metrics

- **Data Sync Strategy**:
  - Vector ID (UUID) links Qdrant points to Postgres metadata
  - Content hash (SHA-256) for deduplication during re-ingestion
  - Transactional ingestion: Postgres first, then Qdrant upsert

## Consequences

### Positive

- **Specialized Performance**: Purpose-built vector DB outperforms pgvector for semantic search
- **Serverless Scaling**: Both services auto-scale; no capacity planning needed
- **Operational Simplicity**: Managed services reduce DevOps burden
- **Cost Efficiency**: Pay-per-use model fits variable query volume
- **Separation of Concerns**: Vectors for search, Postgres for structured queries/joins
- **Audit Trail**: Full query logging in relational DB enables debugging and analytics

### Negative

- **Service Dependency**: Relies on two external managed services
- **Network Latency**: Cross-service queries add ~10-20ms overhead
- **Data Consistency**: Two-phase write (Postgres + Qdrant) requires careful error handling
- **Cost at Scale**: Managed services more expensive than self-hosted at high volume
- **Vendor Lock-in**: Qdrant Cloud API differs from alternatives; migration non-trivial

## Alternatives Considered

### Alternative A: pgvector (Postgres Only)
- **Components**: Neon Postgres with pgvector extension
- **Why Rejected**:
  - Constitution mandates Qdrant as primary vector store
  - pgvector performance degrades >100k vectors
  - Limited payload filtering compared to Qdrant
  - Single database is simpler but less optimized

### Alternative B: Pinecone + Supabase
- **Components**: Pinecone for vectors, Supabase Postgres
- **Why Rejected**:
  - Constitution specifically requires Qdrant
  - Pinecone pricing less favorable for small-medium scale
  - Supabase adds unnecessary auth layer complexity
  - Would require different client libraries

### Alternative C: Self-Hosted Qdrant + Local Postgres
- **Components**: Docker Qdrant, standard PostgreSQL
- **Why Rejected**:
  - Increased operational burden (backups, scaling, monitoring)
  - Higher initial setup complexity
  - Less suitable for team without dedicated DevOps
  - Can migrate later if scale demands

### Alternative D: Weaviate (All-in-One)
- **Components**: Weaviate with built-in vector + object storage
- **Why Rejected**:
  - Constitution requires separate Qdrant + Neon stack
  - Less mature ecosystem
  - Different query language (GraphQL-based)

## References

- Feature Spec: [specs/002-rag-chatbot/spec.md](../../specs/002-rag-chatbot/spec.md)
- Implementation Plan: [specs/002-rag-chatbot/plan.md](../../specs/002-rag-chatbot/plan.md)
- Data Model: [specs/002-rag-chatbot/data-model.md](../../specs/002-rag-chatbot/data-model.md)
- Related ADRs: ADR-002 (Backend Stack)
- Constitution: [.specify/memory/constitution.md](../../.specify/memory/constitution.md) (Technology Stack section)
