# =============================================================================
# RAG Chatbot Backend - Structured Logging
# =============================================================================
# Logging configuration using structlog
# T018: Structured logging with JSON output
# =============================================================================

import logging
import sys
from typing import Any

import structlog
from structlog.types import Processor


def setup_logging(log_level: str = "INFO", log_format: str = "json") -> None:
    """
    Configure structured logging for the application.

    Args:
        log_level: Logging level (DEBUG, INFO, WARNING, ERROR, CRITICAL)
        log_format: Output format ('json' or 'text')
    """
    # Convert string level to logging constant
    numeric_level = getattr(logging, log_level.upper(), logging.INFO)

    # Configure standard library logging
    logging.basicConfig(
        format="%(message)s",
        stream=sys.stdout,
        level=numeric_level,
    )

    # Define shared processors
    shared_processors: list[Processor] = [
        structlog.contextvars.merge_contextvars,
        structlog.processors.add_log_level,
        structlog.processors.TimeStamper(fmt="iso"),
        structlog.processors.StackInfoRenderer(),
        structlog.processors.UnicodeDecoder(),
    ]

    if log_format == "json":
        # JSON format for production
        processors: list[Processor] = [
            *shared_processors,
            structlog.processors.format_exc_info,
            structlog.processors.JSONRenderer(),
        ]
    else:
        # Human-readable format for development
        processors = [
            *shared_processors,
            structlog.dev.ConsoleRenderer(colors=True),
        ]

    structlog.configure(
        processors=processors,
        wrapper_class=structlog.make_filtering_bound_logger(numeric_level),
        context_class=dict,
        logger_factory=structlog.PrintLoggerFactory(),
        cache_logger_on_first_use=True,
    )


def get_logger(name: str | None = None) -> structlog.stdlib.BoundLogger:
    """
    Get a configured logger instance.

    Args:
        name: Logger name (optional)

    Returns:
        Configured structlog logger
    """
    return structlog.get_logger(name)


def bind_request_context(
    request_id: str,
    method: str | None = None,
    path: str | None = None,
    **kwargs: Any,
) -> None:
    """
    Bind request context for correlation.

    Args:
        request_id: Unique request identifier
        method: HTTP method
        path: Request path
        **kwargs: Additional context to bind
    """
    context = {"request_id": request_id}
    if method:
        context["method"] = method
    if path:
        context["path"] = path
    context.update(kwargs)
    structlog.contextvars.bind_contextvars(**context)


def clear_request_context() -> None:
    """Clear all bound request context."""
    structlog.contextvars.clear_contextvars()


class LoggerMixin:
    """Mixin class to add logging capability to any class."""

    @property
    def logger(self) -> structlog.stdlib.BoundLogger:
        """Get logger bound to this class."""
        return get_logger(self.__class__.__name__)


# Pre-configured loggers for common use cases
app_logger = get_logger("app")
api_logger = get_logger("api")
db_logger = get_logger("db")
service_logger = get_logger("service")
