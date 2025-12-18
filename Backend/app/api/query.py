# =============================================================================
# RAG Chatbot Backend - Query Endpoint
# =============================================================================
# /v1/query endpoint for question answering
# T030-T032: Query endpoint with streaming and logging
# =============================================================================

import json
import time
from typing import Any
from uuid import UUID, uuid4

from fastapi import APIRouter, HTTPException
from fastapi.responses import StreamingResponse

from app.config import get_settings
from app.db.postgres import get_postgres_pool
from app.models.requests import QueryRequest
from app.models.responses import ErrorDetail, ErrorResponse, QueryResponse
from app.services.generation import get_generation_service
from app.services.retrieval import get_retrieval_service
from app.utils.logging import api_logger
from app.utils.sanitizer import sanitize_query, validate_session_id

logger = api_logger
router = APIRouter(tags=["query"])


@router.post(
    "/query",
    response_model=QueryResponse,
    responses={
        400: {"model": ErrorResponse, "description": "Invalid query"},
        429: {"model": ErrorResponse, "description": "Rate limited"},
        503: {"model": ErrorResponse, "description": "Service unavailable"},
    },
)
async def query(request: QueryRequest) -> QueryResponse | StreamingResponse:
    """
    Submit a question and receive a grounded answer based on book content.

    - **question**: The question to ask (1-1000 characters)
    - **session_id**: Optional session ID for context tracking
    - **stream**: Enable SSE streaming response (default: false)
    """
    start_time = time.perf_counter()

    # Sanitize input
    sanitized_question = sanitize_query(request.question)
    if not sanitized_question:
        raise HTTPException(
            status_code=400,
            detail=ErrorDetail(
                code="INVALID_QUERY",
                message="Question cannot be empty after sanitization",
            ).model_dump(),
        )

    # Validate session ID if provided
    session_id = validate_session_id(request.session_id)

    logger.info(
        "Query received",
        question_length=len(sanitized_question),
        session_id=session_id,
        stream=request.stream,
    )

    # Handle streaming response
    if request.stream:
        return StreamingResponse(
            _stream_response(sanitized_question, session_id, start_time),
            media_type="text/event-stream",
            headers={
                "Cache-Control": "no-cache",
                "Connection": "keep-alive",
                "X-Accel-Buffering": "no",
            },
        )

    # Non-streaming response
    return await _generate_response(sanitized_question, session_id, start_time)


async def _generate_response(
    question: str,
    session_id: str | None,
    start_time: float,
) -> QueryResponse:
    """Generate non-streaming response."""
    retrieval_service = get_retrieval_service()
    generation_service = get_generation_service()

    # Retrieve relevant chunks
    retrieval_result = await retrieval_service.retrieve(question)

    # Generate response
    answer, generation_latency = await generation_service.generate(
        question=question,
        chunks=retrieval_result.chunks,
        confidence=retrieval_result.confidence,
    )

    # Get source attribution
    sources = await generation_service.get_source_attribution(retrieval_result.chunks)
    if retrieval_result.confidence == "low":
        sources = []

    # Log retrieval (async, don't await)
    _log_retrieval_async(
        query_text=question,
        retrieval_result=retrieval_result,
        fallback_triggered=retrieval_result.confidence == "low",
        latency_ms=int((time.perf_counter() - start_time) * 1000),
    )

    logger.info(
        "Query completed",
        confidence=retrieval_result.confidence,
        sources_count=len(sources),
        retrieval_latency_ms=retrieval_result.latency_ms,
        generation_latency_ms=generation_latency,
    )

    return QueryResponse(
        answer=answer,
        sources=sources,
        confidence=retrieval_result.confidence,
        retrieval_latency_ms=retrieval_result.latency_ms,
        generation_latency_ms=generation_latency,
    )


async def _stream_response(
    question: str,
    session_id: str | None,
    start_time: float,
):
    """Generate streaming response via SSE."""
    retrieval_service = get_retrieval_service()
    generation_service = get_generation_service()

    # Retrieve relevant chunks
    retrieval_result = await retrieval_service.retrieve(question)

    # Stream tokens
    async for token in generation_service.generate_stream(
        question=question,
        chunks=retrieval_result.chunks,
        confidence=retrieval_result.confidence,
    ):
        yield f"event: token\ndata: {json.dumps({'token': token})}\n\n"

    # Send sources
    sources = generation_service.get_source_attribution(retrieval_result.chunks)
    if retrieval_result.confidence == "low":
        sources = []
    yield f"event: sources\ndata: {json.dumps({'sources': sources})}\n\n"

    # Send done event
    generation_latency = int((time.perf_counter() - start_time) * 1000) - retrieval_result.latency_ms
    yield f"event: done\ndata: {json.dumps({'retrieval_latency_ms': retrieval_result.latency_ms, 'generation_latency_ms': generation_latency})}\n\n"

    # Log retrieval
    _log_retrieval_async(
        query_text=question,
        retrieval_result=retrieval_result,
        fallback_triggered=retrieval_result.confidence == "low",
        latency_ms=int((time.perf_counter() - start_time) * 1000),
    )


def _log_retrieval_async(
    query_text: str,
    retrieval_result: Any,
    fallback_triggered: bool,
    latency_ms: int,
) -> None:
    """
    Log retrieval to database asynchronously.

    This runs in background to not block response.
    """
    import asyncio

    async def _do_log():
        try:
            pool = get_postgres_pool()

            chunk_ids = [
                uuid4()  # Generate UUIDs for now; in production, use actual vector IDs
                for _ in retrieval_result.chunks
            ]
            scores = [c.relevance_score for c in retrieval_result.chunks]

            await pool.execute(
                """
                INSERT INTO retrieval_logs (
                    query_text,
                    query_embedding_model,
                    retrieved_chunk_ids,
                    relevance_scores,
                    top_score,
                    response_generated,
                    fallback_triggered,
                    latency_ms
                ) VALUES ($1, $2, $3, $4, $5, $6, $7, $8)
                """,
                query_text[:1000],  # Truncate if too long
                retrieval_result.model_used,
                chunk_ids,
                scores,
                retrieval_result.top_score,
                True,
                fallback_triggered,
                latency_ms,
            )
        except Exception as e:
            logger.error("Failed to log retrieval", error=str(e))

    # Run in background
    try:
        asyncio.create_task(_do_log())
    except RuntimeError:
        # No event loop running (e.g., in tests)
        pass
