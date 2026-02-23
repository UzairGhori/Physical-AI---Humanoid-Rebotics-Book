# =============================================================================
# RAG Chatbot Backend - Embedding Service
# =============================================================================
# Embedding generation with Cohere
# =============================================================================

import asyncio
import time
from typing import Optional

import cohere
from app.config import get_settings
from app.models.entities import EmbeddingResult
from app.utils.logging import service_logger

logger = service_logger


class EmbeddingServiceError(Exception):
    """Custom exception for embedding service errors."""
    pass


class EmbeddingService:
    """
    Embedding service using Cohere.
    """

    def __init__(self, api_key: str, model: str = "embed-english-v3.0"):
        """
        Initialize embedding service.

        Args:
            api_key: API key for Cohere
            model: Cohere model to use for embeddings
        """
        self._api_key = api_key
        self._model = model
        self._client: Optional[cohere.AsyncClient] = None

    @property
    def client(self) -> cohere.AsyncClient:
        """Get Cohere client."""
        if self._client is None:
            self._client = cohere.AsyncClient(self._api_key)
        return self._client

    async def embed(self, text: str, chunk_id: str = "") -> EmbeddingResult:
        """
        Generate embedding for text using Cohere.

        Args:
            text: Text to embed
            chunk_id: Optional chunk identifier

        Returns:
            EmbeddingResult with vector and model information

        Raises:
            EmbeddingServiceError: If embedding generation fails
        """
        start_time = time.perf_counter()
        try:
            response = await self.client.embed(
                texts=[text],
                model=self._model,
                input_type="search_document"
            )
            vector = response.embeddings[0]
            latency_ms = int((time.perf_counter() - start_time) * 1000)

            logger.debug(
                "Cohere embedding generated",
                chunk_id=chunk_id,
                latency_ms=latency_ms,
            )

            return EmbeddingResult(
                chunk_id=chunk_id,
                vector=vector,
                model_used="cohere",
                latency_ms=latency_ms,
            )
        except Exception as e:
            logger.error(
                "Cohere embedding failed",
                chunk_id=chunk_id,
                error=str(e),
            )
            raise EmbeddingServiceError(f"Cohere embedding failed: {e}")

    async def embed_batch(
        self,
        texts: list[tuple[str, str]],
        batch_size: int = 96,  # Cohere's API limit
    ) -> list[EmbeddingResult]:
        """
        Generate embeddings for multiple texts.

        Args:
            texts: List of (text, chunk_id) tuples
            batch_size: Number of texts to process in one API call

        Returns:
            List of EmbeddingResults
        """
        results = []
        for i in range(0, len(texts), batch_size):
            batch = texts[i : i + batch_size]
            batch_texts = [text for text, chunk_id in batch]
            
            start_time = time.perf_counter()
            try:
                response = await self.client.embed(
                    texts=batch_texts,
                    model=self._model,
                    input_type="search_document"
                )
                
                latency_ms = int((time.perf_counter() - start_time) * 1000)

                for j, embedding in enumerate(response.embeddings):
                    text, chunk_id = batch[j]
                    results.append(EmbeddingResult(
                        chunk_id=chunk_id,
                        vector=embedding,
                        model_used="cohere",
                        latency_ms=latency_ms // len(batch) # Approximate latency per item
                    ))

            except Exception as e:
                logger.error("Batch embedding failed", error=str(e))
                raise EmbeddingServiceError(f"Batch embedding failed: {e}")

        return results


# Global service instance
_embedding_service: Optional[EmbeddingService] = None


def get_embedding_service() -> EmbeddingService:
    """Get the global embedding service instance."""
    global _embedding_service
    if _embedding_service is None:
        settings = get_settings()
        _embedding_service = EmbeddingService(
            api_key=settings.cohere_api_key,
        )
    return _embedding_service


async def embed_text(text: str, chunk_id: str = "") -> EmbeddingResult:
    """
    Convenience function to embed text using global service.

    Args:
        text: Text to embed
        chunk_id: Optional chunk identifier

    Returns:
        EmbeddingResult
    """
    service = get_embedding_service()
    return await service.embed(text, chunk_id)