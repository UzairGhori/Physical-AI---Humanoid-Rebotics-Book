# =============================================================================
# RAG Chatbot Backend - Embedding Service Unit Tests
# =============================================================================
from unittest.mock import AsyncMock, MagicMock, patch

import pytest
from app.services.embedding import EmbeddingService, EmbeddingServiceError


class TestEmbeddingService:
    """Tests for EmbeddingService class."""

    @pytest.fixture
    def service(self):
        """Create a test embedding service."""
        return EmbeddingService(api_key="test-cohere-key")

    @pytest.fixture
    def mock_cohere_response(self):
        """Mock successful Cohere API response."""
        mock = MagicMock()
        mock.embeddings = [[0.1] * 1024]
        return mock

    @pytest.mark.asyncio
    async def test_embed_success(self, service, mock_cohere_response):
        """Test successful embedding with Cohere."""
        with patch.object(service, "_client", new_callable=AsyncMock) as mock_client:
            mock_client.embed.return_value = mock_cohere_response

            result = await service.embed("test text", "chunk-1")

            assert result.model_used == "cohere"
            assert len(result.vector) == 1024
            assert result.chunk_id == "chunk-1"
            assert result.latency_ms >= 0
            mock_client.embed.assert_awaited_once_with(
                texts=["test text"],
                model="embed-english-v3.0",
                input_type="search_document",
            )

    @pytest.mark.asyncio
    async def test_embed_failure(self, service):
        """Test embedding failure."""
        with patch.object(service, "_client", new_callable=AsyncMock) as mock_client:
            mock_client.embed.side_effect = Exception("Cohere API error")

            with pytest.raises(EmbeddingServiceError):
                await service.embed("test text", "chunk-1")

    @pytest.mark.asyncio
    async def test_embed_batch_success(self, service):
        """Test successful batch embedding."""
        texts = [("text1", "chunk-1"), ("text2", "chunk-2")]
        
        mock_response = MagicMock()
        mock_response.embeddings = [[0.1] * 1024, [0.2] * 1024]

        with patch.object(service, "_client", new_callable=AsyncMock) as mock_client:
            mock_client.embed.return_value = mock_response

            results = await service.embed_batch(texts)

            assert len(results) == 2
            assert results[0].chunk_id == "chunk-1"
            assert results[1].chunk_id == "chunk-2"
            assert len(results[0].vector) == 1024
            mock_client.embed.assert_awaited_once_with(
                texts=["text1", "text2"],
                model="embed-english-v3.0",
                input_type="search_document",
            )

    @pytest.mark.asyncio
    async def test_embed_batch_failure(self, service):
        """Test batch embedding failure."""
        texts = [("text1", "chunk-1"), ("text2", "chunk-2")]
        
        with patch.object(service, "_client", new_callable=AsyncMock) as mock_client:
            mock_client.embed.side_effect = Exception("Cohere API error")

            with pytest.raises(EmbeddingServiceError):
                await service.embed_batch(texts)

    @pytest.mark.asyncio
    async def test_client_property(self, service):
        """Test that the client property initializes the cohere client."""
        assert service._client is None
        client1 = service.client
        assert client1 is not None
        client2 = service.client
        assert client1 is client2 # Check that the client is cached