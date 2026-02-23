# ADR-004: Embedding Strategy with Primary-Fallback Pattern

> **Scope**: This ADR documents the embedding model selection and the resilience pattern for handling embedding service failures.

- **Status:** Accepted
- **Date:** 2025-12-17
- **Feature:** 002-rag-chatbot
- **Context:** The RAG chatbot requires vector embeddings for both content ingestion and query processing. The constitution mandates using both Qwen and Bonsai embedding models. During spec clarification, we determined a Primary-Fallback strategy rather than ensemble averaging or dual indexing. We need consistent 1536-dimension vectors with <200ms embedding latency and high availability.

## Decision

We will use the following embedding strategy:

- **Primary Model**: Qwen Embeddings
  - Dimensions: 1536 (compatible with OpenAI text-embedding-3-small)
  - Strong semantic understanding for technical content
  - Timeout: 5 seconds before fallback trigger
  - Used for both ingestion and query embedding

- **Fallback Model**: Bonsai Embeddings
  - Dimensions: 1536 (must match primary for index compatibility)
  - Automatic activation on Qwen timeout or error
  - Logged as degraded mode for monitoring

- **Fallback Pattern**:
  ```python
  async def get_embedding(text: str) -> EmbeddingResult:
      try:
          vector = await qwen_embed(text, timeout=5.0)
          return EmbeddingResult(vector=vector, model="qwen")
      except (TimeoutError, ServiceError):
          logger.warning("Qwen fallback triggered")
          vector = await bonsai_embed(text)
          return EmbeddingResult(vector=vector, model="bonsai")
  ```

- **Consistency Guarantee**: Same model used for query embedding as the target vectors (stored in metadata)

## Consequences

### Positive

- **High Availability**: Service continues even if primary embedding provider fails
- **Simple Architecture**: Single vector index, no merging complexity
- **Predictable Latency**: 5s timeout prevents long waits
- **Quality Preference**: Qwen's higher quality used by default when available
- **Transparent Degradation**: Fallback usage logged for monitoring/alerting
- **Cost Control**: Primary model used when available; fallback as insurance

### Negative

- **Dimension Lock-in**: Both models must produce identical dimensions (1536)
- **Potential Quality Variance**: Fallback may have slightly different semantic properties
- **Single Index Limitation**: Cannot leverage multiple embedding perspectives
- **Fallback Testing**: Requires deliberate chaos testing to verify fallback path
- **Cold Start Risk**: If Qwen fails during ingestion, some vectors use Bonsai

## Alternatives Considered

### Alternative A: Ensemble Averaging
- **Approach**: Generate both embeddings, average vector values
- **Why Rejected**:
  - Doubles embedding cost and latency
  - Unclear if averaging improves retrieval quality
  - More complex implementation
  - Both services must succeed for each operation

### Alternative B: Dual Index with Rank Fusion
- **Approach**: Store separate Qwen and Bonsai indexes, merge search results
- **Why Rejected**:
  - Doubles storage cost
  - Complex rank fusion logic
  - Query latency increases (two searches + merge)
  - Overkill for current scale (~3000 vectors)

### Alternative C: Sequential Reranking
- **Approach**: Qwen for initial retrieval, Bonsai for reranking top-N
- **Why Rejected**:
  - Increases latency (two model calls per query)
  - Reranking benefit unclear for small result sets (top-5)
  - More complex pipeline

### Alternative D: OpenAI text-embedding-3-small Only
- **Approach**: Use single OpenAI embedding model, no fallback
- **Why Rejected**:
  - Constitution requires Qwen + Bonsai support
  - Single point of failure
  - Less flexibility for future model evaluation

## References

- Feature Spec: [specs/002-rag-chatbot/spec.md](../../specs/002-rag-chatbot/spec.md) (Clarifications section)
- Implementation Plan: [specs/002-rag-chatbot/plan.md](../../specs/002-rag-chatbot/plan.md) (Technical Decisions)
- Research: [specs/002-rag-chatbot/research.md](../../specs/002-rag-chatbot/research.md) (Embedding Models section)
- Related ADRs: ADR-003 (Data Persistence Stack)
- Constitution: [.specify/memory/constitution.md](../../.specify/memory/constitution.md) (Technology Stack - Embeddings)
