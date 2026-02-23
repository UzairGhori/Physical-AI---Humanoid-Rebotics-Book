import json
import pytest
from unittest.mock import AsyncMock, patch, MagicMock

from app.main import app
from app.models.entities import RetrievedChunk
from httpx import AsyncClient, ASGITransport

# Custom AsyncIterator for mocking
class MockAsyncIterator:
    def __init__(self, seq):
        self.seq = iter(seq)

    async def __aiter__(self):
        return self

    async def __anext__(self):
        try:
            return next(self.seq)
        except StopIteration:
            raise StopAsyncIteration

# Mock data for retrieval results
MOCK_CHUNKS = [
    RetrievedChunk(
        chunk_id="chunk1",
        content="This is relevant content about physical AI from Chapter 1.",
        vector=[0.1] * 1536,
        chapter="Chapter 1",
        section="Introduction",
        relevance_score=0.9,
    ),
    RetrievedChunk(
        chunk_id="chunk2",
        content="Humanoid robotics are discussed in Chapter 2, Section 1.",
        vector=[0.2] * 1536,
        chapter="Chapter 2",
        section="Humanoid Robotics",
        relevance_score=0.8,
    ),
]

FALLBACK_RESPONSE_PREFIX = "I don't know based on the book content."
UNCERTAINTY_DISCLAIMER = "I'm not confident about this answer based on the provided book content. It might be out of scope or require more context."


@pytest.fixture(name="client")
async def client_fixture():
    """Test client for API."""
    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as client:
        yield client


@pytest.fixture(autouse=True)
def mock_retrieval_service():
    """
    Mocks the retrieval service to control retrieval results and confidence.
    """
    with patch("app.api.query.get_retrieval_service") as mock_get_retrieval_service:
        mock_service = AsyncMock()
        mock_get_retrieval_service.return_value = mock_service
        yield mock_service


@pytest.fixture(autouse=True)
def mock_generation_service():
    """
    Mocks the generation service to control generated answers.
    """
    with patch("app.api.query.get_generation_service") as mock_get_generation_service:
        mock_service = AsyncMock()
        mock_get_generation_service.return_value = mock_service
        # Default mock behavior for generate
        mock_service.generate.return_value = ("Mocked answer.", 100)
        # Default mock behavior for get_source_attribution
        mock_service.get_source_attribution.return_value = ["Chapter 1: Introduction"]

        async def _mock_stream(question: str, chunks: list, confidence: str):
            if confidence == "low":
                yield FALLBACK_RESPONSE_PREFIX
                return

            yield "Streamed "
            yield "answer."

        mock_service.generate_stream = _mock_stream
        yield mock_service


@pytest.mark.asyncio
async def test_query_high_confidence(
    client: AsyncClient, mock_retrieval_service, mock_generation_service
):
    """Test query with high confidence retrieval."""
    mock_retrieval_service.retrieve.return_value = AsyncMock(
        chunks=MOCK_CHUNKS,
        confidence="high",
        is_relevant=True,
        latency_ms=50,
        model_used="test-model",
        top_score=0.9,
    )
    mock_generation_service.generate.return_value = ("A confident answer.", 100)

    response = await client.post("/v1/query", json={"question": "What is physical AI?"})
    assert response.status_code == 200
    data = response.json()
    assert data["answer"] == "A confident answer."
    assert data["confidence"] == "high"
    assert "sources" in data
    assert UNCERTAINTY_DISCLAIMER not in data["answer"]


@pytest.mark.asyncio
async def test_query_medium_confidence(
    client: AsyncClient, mock_retrieval_service, mock_generation_service
):
    """Test query with medium confidence retrieval, expecting disclaimer."""
    mock_retrieval_service.retrieve.return_value = AsyncMock(
        chunks=MOCK_CHUNKS,
        confidence="medium",
        is_relevant=True,  # is_relevant is now deprecated but still present in retrieval_result
        latency_ms=50,
        model_used="test-model",
        top_score=0.6,
    )
    mock_generation_service.generate.return_value = (
        f"A somewhat confident answer. {UNCERTAINTY_DISCLAIMER}",
        100,
    )  # Generation service now adds the disclaimer

    response = await client.post("/v1/query", json={"question": "Tell me more about robotics."})
    assert response.status_code == 200
    data = response.json()
    assert data["confidence"] == "medium"
    assert UNCERTAINTY_DISCLAIMER in data["answer"]
    assert "sources" in data


@pytest.mark.asyncio
async def test_query_low_confidence(
    client: AsyncClient, mock_retrieval_service, mock_generation_service
):
    """Test query with low confidence retrieval, expecting fallback."""
    mock_retrieval_service.retrieve.return_value = AsyncMock(
        chunks=[],
        confidence="low",
        is_relevant=False,
        latency_ms=50,
        model_used="test-model",
        top_score=0.1,
    )
    # The generation service should return the fallback response directly
    mock_generation_service.generate.return_value = (FALLBACK_RESPONSE_PREFIX, 0)

    response = await client.post("/v1/query", json={"question": "What is the meaning of life?"})
    assert response.status_code == 200
    data = response.json()
    assert data["answer"].startswith(FALLBACK_RESPONSE_PREFIX)
    assert data["confidence"] == "low"
    assert data["sources"] == []
    assert UNCERTAINTY_DISCLAIMER not in data["answer"]


@pytest.mark.asyncio
async def test_query_stream_high_confidence(
    client: AsyncClient, mock_retrieval_service, mock_generation_service
):
    """Test streaming query with high confidence retrieval."""
    mock_retrieval_service.retrieve.return_value = AsyncMock(
        chunks=MOCK_CHUNKS,
        confidence="high",
        is_relevant=True,
        latency_ms=50,
        model_used="test-model",
        top_score=0.9,
    )

    async def _mock_high_confidence_stream(*args, **kwargs):
        for token in ["Streamed ", "confident ", "answer."]:
            yield token
    mock_generation_service.generate_stream = _mock_high_confidence_stream
    mock_generation_service.get_source_attribution.return_value = ["Chapter 1"]

    response = await client.post("/v1/query", json={"question": "What is ROS2?", "stream": True})
    assert response.status_code == 200
    full_response = ""
    sources_data = []
    async for line in response.aiter_bytes():
        line_str = line.decode("utf-8")
        if line_str.startswith("data:"):
            try:
                event_data = json.loads(line_str[len("data:"):].strip())
                if "token" in event_data:
                    full_response += event_data["token"]
                if "sources" in event_data:
                    sources_data = event_data["sources"]
            except json.JSONDecodeError:
                pass
    assert "Streamed confident answer." in full_response
    assert "Chapter 1" in sources_data
    assert UNCERTAINTY_DISCLAIMER not in full_response


@pytest.mark.asyncio
async def test_query_stream_medium_confidence(
    client: AsyncClient, mock_retrieval_service, mock_generation_service
):
    """Test streaming query with medium confidence retrieval, expecting disclaimer."""
    mock_retrieval_service.retrieve.return_value = AsyncMock(
        chunks=MOCK_CHUNKS,
        confidence="medium",
        is_relevant=True,
        latency_ms=50,
        model_used="test-model",
        top_score=0.6,
    )

    async def _mock_medium_confidence_stream(*args, **kwargs):
        for token in ["Streamed ", "uncertain ", f"answer. {UNCERTAINTY_DISCLAIMER}"]:
            yield token
    mock_generation_service.generate_stream = _mock_medium_confidence_stream
    mock_generation_service.get_source_attribution.return_value = ["Chapter 2"]

    response = await client.post("/v1/query", json={"question": "ROS2 topics?", "stream": True})
    assert response.status_code == 200
    full_response = ""
    sources_data = []
    async for line in response.aiter_bytes():
        line_str = line.decode("utf-8")
        if line_str.startswith("data:"):
            try:
                event_data = json.loads(line_str[len("data:"):].strip())
                if "token" in event_data:
                    full_response += event_data["token"]
                if "sources" in event_data:
                    sources_data = event_data["sources"]
            except json.JSONDecodeError:
                pass
    assert UNCERTAINTY_DISCLAIMER in full_response
    assert "Chapter 2" in sources_data


@pytest.mark.asyncio
async def test_query_stream_low_confidence(
    client: AsyncClient, mock_retrieval_service, mock_generation_service
):
    """Test streaming query with low confidence retrieval, expecting fallback."""
    mock_retrieval_service.retrieve.return_value = AsyncMock(
        chunks=[],
        confidence="low",
        is_relevant=False,
        latency_ms=50,
        model_used="test-model",
        top_score=0.1,
    )

    async def _mock_low_confidence_stream(*args, **kwargs):
        yield FALLBACK_RESPONSE_PREFIX
    mock_generation_service.generate_stream = _mock_low_confidence_stream
    mock_generation_service.get_source_attribution.return_value = []

    response = await client.post("/v1/query", json={"question": "Completely irrelevant?", "stream": True})
    assert response.status_code == 200
    full_response = ""
    sources_data = []
    async for line in response.aiter_bytes():
        line_str = line.decode("utf-8")
        if line_str.startswith("data:"):
            try:
                event_data = json.loads(line_str[len("data:"):].strip())
                if "token" in event_data:
                    full_response += event_data["token"]
                if "sources" in event_data:
                    sources_data = event_data["sources"]
            except json.JSONDecodeError:
                pass
    assert full_response.startswith(FALLBACK_RESPONSE_PREFIX)
    assert sources_data == []
    assert UNCERTAINTY_DISCLAIMER not in full_response

OUT_OF_SCOPE_QUERIES = [
    "What is the weather like in London?",
    "Who won the last world cup?",
    "What is the best recipe for lasagna?",
    "What is the meaning of life?",
    "Can you tell me a joke?",
    "What is the capital of France?",
    "Who is the president of the United States?",
    "What is the stock price of Google?",
    "What is the best movie of all time?",
    "How to bake a cake?",
    "What is the best programming language?",
    "What is the best way to learn Python?",
    "What is the best IDE for Python?",
    "What is the best framework for Python?",
    "What is the best library for Python?",
    "What is the best course for Python?",
    "What is the best book for Python?",
    "What is the best tutorial for Python?",
    "What is the best video for Python?",
    "What is the best website for Python?",
]
@pytest.mark.asyncio
@pytest.mark.parametrize("query", OUT_OF_SCOPE_QUERIES)
async def test_query_out_of_scope(
    client: AsyncClient, mock_retrieval_service, mock_generation_service, query: str
):
    """Test out-of-scope queries all return fallback message."""
    mock_retrieval_service.retrieve.return_value = AsyncMock(
        chunks=[],
        confidence="low",
        is_relevant=False,
        latency_ms=50,
        model_used="test-model",
        top_score=0.1,
    )
    mock_generation_service.generate.return_value = (FALLBACK_RESPONSE_PREFIX, 0)

    response = await client.post("/v1/query", json={"question": query})
    assert response.status_code == 200
    data = response.json()
    assert data["answer"].startswith(FALLBACK_RESPONSE_PREFIX)
    assert data["confidence"] == "low"
    assert data["sources"] == []
    assert UNCERTAINTY_DISCLAIMER not in data["answer"]
