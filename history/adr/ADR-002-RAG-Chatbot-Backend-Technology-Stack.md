# ADR-002: RAG Chatbot Backend Technology Stack

> **Scope**: This ADR documents the integrated backend technology stack for the RAG chatbot, including API framework, LLM integration, and runtime environment.

- **Status:** Accepted
- **Date:** 2025-12-17
- **Feature:** 002-rag-chatbot
- **Context:** The RAG chatbot requires a Python backend to handle chat queries, integrate with LLM providers, and serve responses to the Docusaurus frontend. The constitution mandates using the official OpenAI Python SDK for all agent/chat logic. We need a stack that supports async operations, streaming responses, and can handle 100 concurrent users with <3s latency.

## Decision

We will use the following integrated backend stack:

- **API Framework**: FastAPI 0.109+
  - Native async/await support for non-blocking I/O
  - Built-in OpenAPI documentation
  - Pydantic v2 for request/response validation
  - Server-Sent Events (SSE) support for streaming

- **LLM Integration**: OpenAI Python SDK 1.12+
  - Official SDK per constitution requirement
  - `gpt-4o-mini` model for cost-effective RAG responses
  - Streaming support via `stream=True`
  - OpenRouter as fallback via base_url swap

- **ASGI Server**: Uvicorn with Gunicorn
  - Production deployment with multiple workers
  - Auto-reload for development
  - HTTP/2 support

- **Runtime**: Python 3.11+
  - Performance improvements (faster startup, better error messages)
  - Native asyncio enhancements
  - Type hint improvements for tooling

## Consequences

### Positive

- **High Performance**: FastAPI is one of the fastest Python frameworks, comparable to Node.js/Go for async workloads
- **Type Safety**: Pydantic integration ensures runtime validation and IDE support
- **Streaming UX**: SSE support enables <500ms first-token latency vs 2-3s for full responses
- **SDK Compliance**: Meets constitution's SDK-First Architecture principle
- **Developer Experience**: Auto-generated OpenAPI docs, hot reload, clear error messages
- **Ecosystem**: Rich async library ecosystem (asyncpg, httpx, qdrant-client)

### Negative

- **Python GIL**: CPU-bound operations still single-threaded; mitigated by async I/O focus
- **Cold Start**: Slower startup than compiled languages; acceptable for this use case
- **Dependency on OpenAI**: Primary LLM provider; mitigated by OpenRouter fallback
- **Memory Footprint**: Higher than Go/Rust; acceptable for expected scale

## Alternatives Considered

### Alternative A: Django + DRF + Celery
- **Components**: Django 5.0, Django REST Framework, Celery for async
- **Why Rejected**:
  - Heavier framework overhead
  - Async support less native (ASGI still evolving in Django)
  - Celery adds operational complexity for real-time streaming
  - Overkill for API-focused service without admin UI needs

### Alternative B: Flask + Quart + Background Workers
- **Components**: Flask/Quart, manual async handling, custom streaming
- **Why Rejected**:
  - Less built-in validation (no Pydantic integration)
  - Manual OpenAPI spec maintenance
  - More boilerplate for SSE streaming
  - Smaller async ecosystem

### Alternative C: Node.js + Express/Fastify
- **Components**: Node.js, Fastify, OpenAI JS SDK
- **Why Rejected**:
  - Constitution mandates OpenAI Python SDK
  - Python ecosystem preferred for ML/embedding tooling
  - Team expertise in Python

## References

- Feature Spec: [specs/002-rag-chatbot/spec.md](../../specs/002-rag-chatbot/spec.md)
- Implementation Plan: [specs/002-rag-chatbot/plan.md](../../specs/002-rag-chatbot/plan.md)
- Research: [specs/002-rag-chatbot/research.md](../../specs/002-rag-chatbot/research.md)
- Related ADRs: ADR-001 (Task Structure)
- Constitution: [.specify/memory/constitution.md](../../.specify/memory/constitution.md) (SDK-First Architecture principle)
