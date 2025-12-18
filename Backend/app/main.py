# =============================================================================
# RAG Chatbot Backend - FastAPI Application
# =============================================================================
# Main application entry point
# T019: FastAPI application with CORS and lifespan events
# =============================================================================

from contextlib import asynccontextmanager
from typing import AsyncGenerator

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

from app.api.health import router as health_router
from app.api.query import router as query_router
from app.config import get_settings
from app.db.postgres import close_postgres_pool, init_postgres_pool
from app.db.qdrant import close_qdrant_client, init_qdrant_client
from app.utils.logging import app_logger, setup_logging

logger = app_logger


@asynccontextmanager
async def lifespan(app: FastAPI) -> AsyncGenerator[None, None]:
    """
    Application lifespan context manager.

    Handles startup and shutdown events for database connections.
    """
    settings = get_settings()

    # Setup logging
    setup_logging(settings.log_level, settings.log_format)
    logger.info("Starting RAG Chatbot Backend")

    # Initialize database connections
    try:
        await init_postgres_pool(settings.database_url)
        logger.info("PostgreSQL pool initialized")
    except Exception as e:
        logger.error("Failed to initialize PostgreSQL", error=str(e))
        # Continue startup - health endpoint will report unhealthy

    try:
        await init_qdrant_client(
            url=settings.qdrant_url,
            api_key=settings.qdrant_api_key,
            collection_name=settings.qdrant_collection,
        )
        logger.info("Qdrant client initialized")
    except Exception as e:
        logger.error("Failed to initialize Qdrant", error=str(e))
        # Continue startup - health endpoint will report unhealthy

    logger.info(
        "Application started",
        host=settings.api_host,
        port=settings.api_port,
    )

    yield

    # Shutdown: close connections
    logger.info("Shutting down RAG Chatbot Backend")
    await close_postgres_pool()
    await close_qdrant_client()
    logger.info("Application shutdown complete")


def create_app() -> FastAPI:
    """
    Create and configure the FastAPI application.

    Returns:
        Configured FastAPI application instance
    """
    settings = get_settings()

    app = FastAPI(
        title="RAG Chatbot API",
        description="REST API for the Physical AI & Humanoid Robotics Book RAG Chatbot",
        version="0.1.0",
        docs_url="/docs",
        redoc_url="/redoc",
        openapi_url="/v1/openapi.json",
        lifespan=lifespan,
    )

    # Configure CORS
    app.add_middleware(
        CORSMiddleware,
        allow_origins=settings.cors_origins_list,
        allow_credentials=True,
        allow_methods=["GET", "POST", "OPTIONS"],
        allow_headers=["Content-Type", "X-API-Key"],
        max_age=86400,  # Cache preflight requests for 24 hours
    )

    # Register routers
    app.include_router(health_router, prefix="/v1")
    app.include_router(query_router, prefix="/v1")

    # Root endpoint
    @app.get("/")
    async def root():
        """Root endpoint with API information."""
        return {
            "name": "RAG Chatbot API",
            "version": "0.1.0",
            "docs": "/docs",
            "health": "/v1/health",
        }

    return app


# Application instance for uvicorn
app = create_app()


if __name__ == "__main__":
    import uvicorn

    settings = get_settings()
    uvicorn.run(
        "app.main:app",
        host=settings.api_host,
        port=settings.api_port,
        reload=True,
    )
