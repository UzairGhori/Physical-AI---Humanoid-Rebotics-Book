# =============================================================================
# RAG Chatbot Backend - Pytest Configuration
# =============================================================================
# Shared fixtures for all tests
# =============================================================================

import asyncio
from collections.abc import AsyncGenerator, Generator
from typing import Any
from unittest.mock import AsyncMock, MagicMock

import pytest
from httpx import ASGITransport, AsyncClient


# -----------------------------------------------------------------------------
# Event Loop Configuration
# -----------------------------------------------------------------------------
@pytest.fixture(scope="session")
def event_loop() -> Generator[asyncio.AbstractEventLoop, None, None]:
    """Create an event loop for the test session."""
    loop = asyncio.new_event_loop()
    yield loop
    loop.close()


# -----------------------------------------------------------------------------
# Application Fixtures
# -----------------------------------------------------------------------------
@pytest.fixture
def app():
    """Create a test FastAPI application instance."""
    # Import here to avoid circular imports during test collection
    from app.main import create_app

    return create_app()


@pytest.fixture
async def async_client(app) -> AsyncGenerator[AsyncClient, None]:
    """Create an async HTTP client for testing API endpoints."""
    transport = ASGITransport(app=app)
    async with AsyncClient(transport=transport, base_url="http://test") as client:
        yield client


# -----------------------------------------------------------------------------
# Database Fixtures
# -----------------------------------------------------------------------------
@pytest.fixture
def mock_postgres_pool() -> MagicMock:
    """Create a mock PostgreSQL connection pool."""
    pool = MagicMock()
    pool.acquire = AsyncMock()
    pool.release = AsyncMock()
    pool.close = AsyncMock()

    # Mock connection
    conn = MagicMock()
    conn.execute = AsyncMock(return_value="SELECT 1")
    conn.fetch = AsyncMock(return_value=[])
    conn.fetchrow = AsyncMock(return_value=None)
    conn.fetchval = AsyncMock(return_value=None)

    pool.acquire.return_value.__aenter__ = AsyncMock(return_value=conn)
    pool.acquire.return_value.__aexit__ = AsyncMock(return_value=None)

    return pool


@pytest.fixture
def mock_qdrant_client() -> MagicMock:
    """Create a mock Qdrant client."""
    client = MagicMock()

    # Mock search response
    client.search = AsyncMock(return_value=[])
    client.upsert = AsyncMock(return_value=None)
    client.delete = AsyncMock(return_value=None)
    client.get_collection = AsyncMock(return_value=MagicMock(
        points_count=0,
        vectors_count=0,
    ))
    client.create_collection = AsyncMock(return_value=None)
    client.collection_exists = AsyncMock(return_value=True)

    return client


# -----------------------------------------------------------------------------
# Service Fixtures
# -----------------------------------------------------------------------------
@pytest.fixture
def mock_embedding_service() -> MagicMock:
    """Create a mock embedding service."""
    service = MagicMock()
    service.embed = AsyncMock(return_value={
        "vector": [0.1] * 1536,
        "model": "qwen",
        "latency_ms": 100,
    })
    return service


@pytest.fixture
def mock_openai_client() -> MagicMock:
    """Create a mock OpenAI client."""
    client = MagicMock()

    # Mock completion response
    completion = MagicMock()
    completion.choices = [MagicMock(
        message=MagicMock(content="This is a test response."),
        finish_reason="stop",
    )]
    completion.usage = MagicMock(
        prompt_tokens=100,
        completion_tokens=50,
        total_tokens=150,
    )

    client.chat.completions.create = AsyncMock(return_value=completion)

    return client


# -----------------------------------------------------------------------------
# Sample Data Fixtures
# -----------------------------------------------------------------------------
@pytest.fixture
def sample_query_request() -> dict[str, Any]:
    """Create a sample query request."""
    return {
        "question": "What is ROS2?",
        "session_id": "test-session-123",
        "stream": False,
    }


@pytest.fixture
def sample_chunk() -> dict[str, Any]:
    """Create a sample book chunk."""
    return {
        "chunk_id": "chunk-001",
        "content": "ROS2 (Robot Operating System 2) is a flexible framework for writing robot software.",
        "chapter": "Introduction to ROS2",
        "section": "Overview",
        "position": 1,
        "token_count": 50,
        "source_file": "docs/01-introduction.md",
        "content_hash": "abc123",
    }


@pytest.fixture
def sample_chunks(sample_chunk: dict[str, Any]) -> list[dict[str, Any]]:
    """Create a list of sample chunks."""
    return [
        sample_chunk,
        {
            **sample_chunk,
            "chunk_id": "chunk-002",
            "content": "ROS2 uses DDS for communication.",
            "position": 2,
        },
        {
            **sample_chunk,
            "chunk_id": "chunk-003",
            "content": "Nodes are the basic building blocks in ROS2.",
            "position": 3,
        },
    ]


@pytest.fixture
def sample_retrieval_result(sample_chunks: list[dict[str, Any]]) -> dict[str, Any]:
    """Create a sample retrieval result."""
    return {
        "chunks": [
            {**chunk, "relevance_score": 0.85 - i * 0.05}
            for i, chunk in enumerate(sample_chunks)
        ],
        "top_score": 0.85,
        "is_relevant": True,
        "confidence": "high",
    }


# -----------------------------------------------------------------------------
# Environment Fixtures
# -----------------------------------------------------------------------------
@pytest.fixture
def mock_env(monkeypatch) -> None:
    """Set up mock environment variables for testing."""
    env_vars = {
        "OPENAI_API_KEY": "sk-test-key",
        "OPENAI_MODEL": "gpt-4o-mini",
        "QWEN_API_KEY": "test-qwen-key",
        "QWEN_API_URL": "https://api.qwen.test/v1",
        "BONSAI_API_KEY": "test-bonsai-key",
        "BONSAI_API_URL": "https://api.bonsai.test/v1",
        "QDRANT_URL": "http://localhost:6333",
        "QDRANT_API_KEY": "",
        "QDRANT_COLLECTION": "test_chunks",
        "DATABASE_URL": "postgresql://test:test@localhost:5432/testdb",
        "API_HOST": "0.0.0.0",
        "API_PORT": "8000",
        "API_ADMIN_KEY": "test-admin-key",
        "CORS_ORIGINS": "http://localhost:3000",
        "LOG_LEVEL": "DEBUG",
        "RELEVANCE_THRESHOLD": "0.7",
        "TOP_K_CHUNKS": "5",
        "CHUNK_SIZE": "512",
        "CHUNK_OVERLAP": "50",
        "EMBEDDING_TIMEOUT": "5.0",
    }

    for key, value in env_vars.items():
        monkeypatch.setenv(key, value)


# -----------------------------------------------------------------------------
# Utility Functions
# -----------------------------------------------------------------------------
def create_mock_sse_response(tokens: list[str]) -> AsyncMock:
    """Create a mock SSE streaming response."""
    async def mock_stream():
        for token in tokens:
            yield MagicMock(
                choices=[MagicMock(
                    delta=MagicMock(content=token),
                    finish_reason=None,
                )]
            )
        yield MagicMock(
            choices=[MagicMock(
                delta=MagicMock(content=None),
                finish_reason="stop",
            )]
        )

    return mock_stream()
