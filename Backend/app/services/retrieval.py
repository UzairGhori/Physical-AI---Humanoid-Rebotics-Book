# =============================================================================
# RAG Chatbot Backend - Retrieval Service
# =============================================================================
# Vector search and relevance checking
# T024-T025: Retrieval service with relevance threshold
# =============================================================================

import time
from typing import Any, Optional

from app.config import get_settings
from app.db.qdrant import get_qdrant_client
from app.models.entities import RetrievalResult, RetrievedChunk
from app.services.embedding import embed_text
from app.utils.logging import service_logger

logger = service_logger


class RetrievalService:
    """
    Service for retrieving relevant chunks from vector database.

    Handles vector search, relevance scoring, and confidence classification.
    """

    def __init__(
        self,
        relevance_threshold: float = 0.7,
        top_k: int = 5,
    ):
        """
        Initialize retrieval service.

        Args:
            relevance_threshold: Minimum score for confident responses
            top_k: Number of chunks to retrieve
        """
        self._relevance_threshold = relevance_threshold
        self._top_k = top_k

    async def retrieve(
        self,
        query: str,
        filter_conditions: Optional[dict[str, Any]] = None,
    ) -> RetrievalResult:
        """
        Retrieve relevant chunks for a query.

        Args:
            query: User query text
            filter_conditions: Optional filters (chapter, section)

        Returns:
            RetrievalResult with chunks, scores, and confidence
        """
        start_time = time.perf_counter()

        # Generate query embedding
        embedding_result = await embed_text(query, chunk_id="query")
        query_vector = embedding_result.vector
        model_used = embedding_result.model_used

        # Search Qdrant
        qdrant = get_qdrant_client()
        search_results = await qdrant.search(
            vector=query_vector,
            limit=self._top_k,
            filter_conditions=filter_conditions,
        )

        # Convert to RetrievedChunk entities
        chunks = []
        for result in search_results:
            payload = result.get("payload", {})
            chunks.append(
                RetrievedChunk(
                    chunk_id=payload.get("chunk_id", result["id"]),
                    content=payload.get("content", ""),
                    chapter=payload.get("chapter", "Unknown"),
                    section=payload.get("section"),
                    relevance_score=result["score"],
                    source_file=payload.get("source_file"),
                )
            )

        # Calculate top score and confidence
        top_score = chunks[0].relevance_score if chunks else 0.0
        is_relevant, confidence = self._check_relevance(top_score)

        latency_ms = int((time.perf_counter() - start_time) * 1000)

        logger.info(
            "Retrieval completed",
            query_length=len(query),
            chunks_found=len(chunks),
            top_score=top_score,
            confidence=confidence,
            latency_ms=latency_ms,
        )

        return RetrievalResult(
            query=query,
            chunks=chunks,
            top_score=top_score,
            is_relevant=is_relevant,
            confidence=confidence,
            model_used=model_used,
            latency_ms=latency_ms,
        )

    def _check_relevance(self, top_score: float) -> tuple[bool, str]:
        """
        Check if retrieval result meets relevance threshold.

        Per plan.md:
        - ≥ 0.8: High confidence
        - 0.7 - 0.8: Medium confidence
        - < 0.7: Low confidence (fallback)

        Args:
            top_score: Highest relevance score from search

        Returns:
            Tuple of (is_relevant, confidence_level)
        """
        if top_score >= 0.8:
            return True, "high"
        elif top_score >= self._relevance_threshold:
            return True, "medium"
        else:
            return False, "low"

    def get_source_attribution(self, chunks: list[RetrievedChunk]) -> list[str]:
        """
        Extract source attribution strings from chunks.

        Args:
            chunks: Retrieved chunks

        Returns:
            List of "Chapter X: Section Y" strings
        """
        sources = []
        seen = set()

        for chunk in chunks:
            if chunk.section:
                source = f"{chunk.chapter}: {chunk.section}"
            else:
                source = chunk.chapter

            if source not in seen:
                sources.append(source)
                seen.add(source)

        return sources


# Global service instance
_retrieval_service: Optional[RetrievalService] = None


def get_retrieval_service() -> RetrievalService:
    """Get the global retrieval service instance."""
    global _retrieval_service
    if _retrieval_service is None:
        settings = get_settings()
        _retrieval_service = RetrievalService(
            relevance_threshold=settings.relevance_threshold,
            top_k=settings.top_k_chunks,
        )
    return _retrieval_service


async def retrieve_chunks(
    query: str,
    filter_conditions: Optional[dict[str, Any]] = None,
) -> RetrievalResult:
    """
    Convenience function to retrieve chunks using global service.

    Args:
        query: User query text
        filter_conditions: Optional filters

    Returns:
        RetrievalResult
    """
    service = get_retrieval_service()
    return await service.retrieve(query, filter_conditions)
