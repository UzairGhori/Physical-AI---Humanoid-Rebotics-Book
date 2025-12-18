# =============================================================================
# RAG Chatbot Backend - Health Endpoint
# =============================================================================
# System health check endpoint
# T020: /v1/health endpoint
# =============================================================================

from datetime import datetime

from fastapi import APIRouter

from app.db.postgres import get_postgres_pool
from app.db.qdrant import get_qdrant_client
from app.models.responses import ComponentStatus, HealthResponse

router = APIRouter(tags=["health"])


@router.get("/health", response_model=HealthResponse)
async def health_check() -> HealthResponse:
    """
    Check system health and component status.

    Returns overall system health status and individual component statuses.
    - healthy: All components operational
    - degraded: Some components impaired but functional
    - unhealthy: Critical components down
    """
    components: dict[str, ComponentStatus] = {}
    overall_status = "healthy"

    # Check PostgreSQL
    try:
        postgres_pool = get_postgres_pool()
        is_healthy, latency = await postgres_pool.health_check()
        components["postgres"] = ComponentStatus(
            status="connected" if is_healthy else "disconnected",
            latency_ms=latency if is_healthy else None,
        )
        if not is_healthy:
            overall_status = "unhealthy"
    except RuntimeError:
        components["postgres"] = ComponentStatus(status="not_initialized")
        overall_status = "unhealthy"
    except Exception as e:
        components["postgres"] = ComponentStatus(status=f"error: {str(e)[:50]}")
        overall_status = "unhealthy"

    # Check Qdrant
    try:
        qdrant_client = get_qdrant_client()
        is_healthy, latency = await qdrant_client.health_check()
        components["qdrant"] = ComponentStatus(
            status="connected" if is_healthy else "disconnected",
            latency_ms=latency if is_healthy else None,
        )
        if not is_healthy:
            overall_status = "unhealthy"
    except RuntimeError:
        components["qdrant"] = ComponentStatus(status="not_initialized")
        overall_status = "unhealthy"
    except Exception as e:
        components["qdrant"] = ComponentStatus(status=f"error: {str(e)[:50]}")
        overall_status = "unhealthy"

    # OpenAI status (basic check - actual availability verified on first call)
    # For now, mark as available since we can't test without API call cost
    components["openai"] = ComponentStatus(
        status="available",
        model="gpt-4o-mini",
    )

    # Embedding service status
    # Default to qwen available; actual status determined during embedding calls
    components["embedding"] = ComponentStatus(
        status="available",
        model="qwen",
        fallback_available=True,
    )

    # Determine overall status
    component_statuses = [c.status for c in components.values()]
    if any("error" in s or s == "disconnected" or s == "not_initialized" for s in component_statuses):
        if all(s in ("connected", "available") for s in component_statuses):
            overall_status = "healthy"
        elif any(s in ("connected", "available") for s in component_statuses):
            overall_status = "degraded"
        else:
            overall_status = "unhealthy"

    return HealthResponse(
        status=overall_status,
        components=components,
        timestamp=datetime.utcnow(),
    )
