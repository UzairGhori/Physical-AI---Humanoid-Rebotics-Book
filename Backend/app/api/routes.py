# =============================================================================
# RAG Chatbot Backend - API Routes
# =============================================================================
# Central route registration
# =============================================================================

from fastapi import APIRouter

from app.api.health import router as health_router
from app.api.query import router as query_router

# Create main API router
api_router = APIRouter(prefix="/v1")

# Include sub-routers
api_router.include_router(health_router)
api_router.include_router(query_router)
