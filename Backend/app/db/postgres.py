# =============================================================================
# RAG Chatbot Backend - PostgreSQL Connection
# =============================================================================
# Async PostgreSQL connection pool using asyncpg
# T007: Postgres connection pool with health check
# =============================================================================

from contextlib import asynccontextmanager
from typing import Any, AsyncGenerator, Optional

import asyncpg
from asyncpg import Connection, Pool

from app.utils.logging import db_logger

logger = db_logger


class PostgresPool:
    """Async PostgreSQL connection pool manager."""

    def __init__(self, database_url: str):
        """
        Initialize PostgreSQL pool manager.

        Args:
            database_url: PostgreSQL connection URL
        """
        self._database_url = database_url
        self._pool: Optional[Pool] = None

    async def connect(self) -> None:
        """Create the connection pool."""
        if self._pool is not None:
            logger.warning("Pool already initialized")
            return

        try:
            self._pool = await asyncpg.create_pool(
                self._database_url,
                min_size=2,
                max_size=10,
                command_timeout=60,
                statement_cache_size=100,
            )
            logger.info("PostgreSQL pool created successfully")
        except Exception as e:
            logger.error("Failed to create PostgreSQL pool", error=str(e))
            raise

    async def disconnect(self) -> None:
        """Close the connection pool."""
        if self._pool:
            await self._pool.close()
            self._pool = None
            logger.info("PostgreSQL pool closed")

    @property
    def pool(self) -> Pool:
        """Get the connection pool."""
        if self._pool is None:
            raise RuntimeError("PostgreSQL pool not initialized. Call connect() first.")
        return self._pool

    @asynccontextmanager
    async def acquire(self) -> AsyncGenerator[Connection, None]:
        """
        Acquire a connection from the pool.

        Yields:
            Database connection

        Example:
            async with postgres_pool.acquire() as conn:
                result = await conn.fetch("SELECT * FROM users")
        """
        async with self.pool.acquire() as connection:
            yield connection

    async def health_check(self) -> tuple[bool, int]:
        """
        Check database connectivity and measure latency.

        Returns:
            Tuple of (is_healthy, latency_ms)
        """
        import time

        if self._pool is None:
            return False, 0

        try:
            start = time.perf_counter()
            async with self.acquire() as conn:
                await conn.fetchval("SELECT 1")
            latency_ms = int((time.perf_counter() - start) * 1000)
            return True, latency_ms
        except Exception as e:
            logger.error("PostgreSQL health check failed", error=str(e))
            return False, 0

    async def execute(self, query: str, *args: Any) -> str:
        """
        Execute a query without returning results.

        Args:
            query: SQL query
            *args: Query parameters

        Returns:
            Status string
        """
        async with self.acquire() as conn:
            return await conn.execute(query, *args)

    async def fetch(self, query: str, *args: Any) -> list[asyncpg.Record]:
        """
        Execute a query and return all results.

        Args:
            query: SQL query
            *args: Query parameters

        Returns:
            List of records
        """
        async with self.acquire() as conn:
            return await conn.fetch(query, *args)

    async def fetchrow(self, query: str, *args: Any) -> Optional[asyncpg.Record]:
        """
        Execute a query and return first row.

        Args:
            query: SQL query
            *args: Query parameters

        Returns:
            Single record or None
        """
        async with self.acquire() as conn:
            return await conn.fetchrow(query, *args)

    async def fetchval(self, query: str, *args: Any) -> Any:
        """
        Execute a query and return first column of first row.

        Args:
            query: SQL query
            *args: Query parameters

        Returns:
            Single value
        """
        async with self.acquire() as conn:
            return await conn.fetchval(query, *args)


# Global pool instance (initialized in app lifespan)
_postgres_pool: Optional[PostgresPool] = None


def get_postgres_pool() -> PostgresPool:
    """Get the global PostgreSQL pool instance."""
    if _postgres_pool is None:
        raise RuntimeError("PostgreSQL pool not initialized")
    return _postgres_pool


async def init_postgres_pool(database_url: str) -> PostgresPool:
    """
    Initialize the global PostgreSQL pool.

    Args:
        database_url: PostgreSQL connection URL

    Returns:
        Initialized pool instance
    """
    global _postgres_pool
    _postgres_pool = PostgresPool(database_url)
    await _postgres_pool.connect()
    return _postgres_pool


async def close_postgres_pool() -> None:
    """Close the global PostgreSQL pool."""
    global _postgres_pool
    if _postgres_pool:
        await _postgres_pool.disconnect()
        _postgres_pool = None
