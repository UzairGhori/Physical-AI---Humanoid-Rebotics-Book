# =============================================================================
# RAG Chatbot Backend - Qdrant Client
# =============================================================================
# Qdrant vector database client wrapper
# T008: Qdrant client with collection management
# =============================================================================

import time
from typing import Any, Optional
from uuid import UUID

from qdrant_client import QdrantClient
from qdrant_client.http import models as qmodels
from qdrant_client.http.exceptions import UnexpectedResponse

from app.utils.logging import db_logger

logger = db_logger

# Vector configuration per data-model.md
VECTOR_SIZE = 1536
DISTANCE_METRIC = qmodels.Distance.COSINE


class QdrantWrapper:
    """Wrapper for Qdrant vector database operations."""

    def __init__(
        self,
        url: str,
        api_key: Optional[str] = None,
        collection_name: str = "book_chunks",
    ):
        """
        Initialize Qdrant client wrapper.

        Args:
            url: Qdrant server URL
            api_key: API key (optional for local)
            collection_name: Name of the collection to use
        """
        self._url = url
        self._api_key = api_key
        self._collection_name = collection_name
        self._client: Optional[QdrantClient] = None

    async def connect(self) -> None:
        """Initialize the Qdrant client and ensure collection exists."""
        try:
            self._client = QdrantClient(
                url=self._url,
                api_key=self._api_key if self._api_key else None,
                timeout=30,
            )

            # Ensure collection exists
            await self._ensure_collection()
            logger.info(
                "Qdrant client connected",
                url=self._url,
                collection=self._collection_name,
            )
        except Exception as e:
            logger.error("Failed to connect to Qdrant", error=str(e))
            raise

    async def disconnect(self) -> None:
        """Close the Qdrant client."""
        if self._client:
            self._client.close()
            self._client = None
            logger.info("Qdrant client disconnected")

    @property
    def client(self) -> QdrantClient:
        """Get the Qdrant client."""
        if self._client is None:
            raise RuntimeError("Qdrant client not initialized. Call connect() first.")
        return self._client

    async def _ensure_collection(self) -> None:
        """Create collection if it doesn't exist."""
        try:
            collections = self.client.get_collections()
            collection_names = [c.name for c in collections.collections]

            if self._collection_name not in collection_names:
                self.client.create_collection(
                    collection_name=self._collection_name,
                    vectors_config=qmodels.VectorParams(
                        size=VECTOR_SIZE,
                        distance=DISTANCE_METRIC,
                        on_disk=True,
                    ),
                    hnsw_config=qmodels.HnswConfigDiff(
                        m=16,
                        ef_construct=100,
                    ),
                    optimizers_config=qmodels.OptimizersConfigDiff(
                        indexing_threshold=10000,
                    ),
                )

                # Create payload indexes
                self.client.create_payload_index(
                    collection_name=self._collection_name,
                    field_name="chapter",
                    field_schema=qmodels.PayloadSchemaType.KEYWORD,
                )
                self.client.create_payload_index(
                    collection_name=self._collection_name,
                    field_name="section",
                    field_schema=qmodels.PayloadSchemaType.KEYWORD,
                )
                self.client.create_payload_index(
                    collection_name=self._collection_name,
                    field_name="source_file",
                    field_schema=qmodels.PayloadSchemaType.KEYWORD,
                )

                logger.info(
                    "Created Qdrant collection",
                    collection=self._collection_name,
                    vector_size=VECTOR_SIZE,
                )
            else:
                logger.info(
                    "Qdrant collection exists",
                    collection=self._collection_name,
                )
        except UnexpectedResponse as e:
            logger.error("Qdrant collection setup failed", error=str(e))
            raise

    async def health_check(self) -> tuple[bool, int]:
        """
        Check Qdrant connectivity and measure latency.

        Returns:
            Tuple of (is_healthy, latency_ms)
        """
        if self._client is None:
            return False, 0

        try:
            start = time.perf_counter()
            self.client.get_collection(self._collection_name)
            latency_ms = int((time.perf_counter() - start) * 1000)
            return True, latency_ms
        except Exception as e:
            logger.error("Qdrant health check failed", error=str(e))
            return False, 0

    async def search(
        self,
        vector: list[float],
        limit: int = 5,
        score_threshold: Optional[float] = None,
        filter_conditions: Optional[dict[str, Any]] = None,
    ) -> list[dict[str, Any]]:
        """
        Search for similar vectors.

        Args:
            vector: Query vector (1536 dimensions)
            limit: Maximum results to return
            score_threshold: Minimum similarity score
            filter_conditions: Optional filter by payload fields

        Returns:
            List of search results with payload and score
        """
        query_filter = None
        if filter_conditions:
            must_conditions = []
            for field, value in filter_conditions.items():
                must_conditions.append(
                    qmodels.FieldCondition(
                        key=field,
                        match=qmodels.MatchValue(value=value),
                    )
                )
            query_filter = qmodels.Filter(must=must_conditions)

        results = self.client.search(
            collection_name=self._collection_name,
            query_vector=vector,
            limit=limit,
            score_threshold=score_threshold,
            query_filter=query_filter,
            with_payload=True,
        )

        return [
            {
                "id": str(hit.id),
                "score": hit.score,
                "payload": hit.payload,
            }
            for hit in results
        ]

    async def upsert(
        self,
        point_id: UUID,
        vector: list[float],
        payload: dict[str, Any],
    ) -> None:
        """
        Insert or update a vector point.

        Args:
            point_id: Unique point identifier
            vector: Vector embedding
            payload: Associated metadata
        """
        self.client.upsert(
            collection_name=self._collection_name,
            points=[
                qmodels.PointStruct(
                    id=str(point_id),
                    vector=vector,
                    payload=payload,
                )
            ],
        )

    async def upsert_batch(
        self,
        points: list[tuple[UUID, list[float], dict[str, Any]]],
    ) -> None:
        """
        Batch upsert multiple vector points.

        Args:
            points: List of (point_id, vector, payload) tuples
        """
        point_structs = [
            qmodels.PointStruct(
                id=str(point_id),
                vector=vector,
                payload=payload,
            )
            for point_id, vector, payload in points
        ]

        self.client.upsert(
            collection_name=self._collection_name,
            points=point_structs,
        )

    async def delete(self, point_ids: list[UUID]) -> None:
        """
        Delete points by IDs.

        Args:
            point_ids: List of point IDs to delete
        """
        self.client.delete(
            collection_name=self._collection_name,
            points_selector=qmodels.PointIdsList(
                points=[str(pid) for pid in point_ids],
            ),
        )

    async def get_collection_info(self) -> dict[str, Any]:
        """Get collection statistics."""
        info = self.client.get_collection(self._collection_name)
        return {
            "points_count": info.points_count,
            "vectors_count": info.vectors_count,
            "indexed_vectors_count": info.indexed_vectors_count,
            "status": info.status.value,
        }


# Global instance (initialized in app lifespan)
_qdrant_wrapper: Optional[QdrantWrapper] = None


def get_qdrant_client() -> QdrantWrapper:
    """Get the global Qdrant client instance."""
    if _qdrant_wrapper is None:
        raise RuntimeError("Qdrant client not initialized")
    return _qdrant_wrapper


async def init_qdrant_client(
    url: str,
    api_key: Optional[str] = None,
    collection_name: str = "book_chunks",
) -> QdrantWrapper:
    """
    Initialize the global Qdrant client.

    Args:
        url: Qdrant server URL
        api_key: API key (optional)
        collection_name: Collection name

    Returns:
        Initialized client wrapper
    """
    global _qdrant_wrapper
    _qdrant_wrapper = QdrantWrapper(url, api_key, collection_name)
    await _qdrant_wrapper.connect()
    return _qdrant_wrapper


async def close_qdrant_client() -> None:
    """Close the global Qdrant client."""
    global _qdrant_wrapper
    if _qdrant_wrapper:
        await _qdrant_wrapper.disconnect()
        _qdrant_wrapper = None
