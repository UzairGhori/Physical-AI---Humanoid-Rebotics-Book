# =============================================================================
# RAG Chatbot Backend - Input Sanitizer
# =============================================================================
# Input sanitization utilities for security
# T017: Input sanitizer with XSS/injection prevention
# =============================================================================

import html
import re
from typing import Optional


# Maximum allowed query length
MAX_QUERY_LENGTH = 1000

# Patterns for potentially dangerous content
SCRIPT_PATTERN = re.compile(r"<script[^>]*>.*?</script>", re.IGNORECASE | re.DOTALL)
EVENT_HANDLER_PATTERN = re.compile(r"\bon\w+\s*=", re.IGNORECASE)
JAVASCRIPT_PATTERN = re.compile(r"javascript:", re.IGNORECASE)
DATA_PATTERN = re.compile(r"data:", re.IGNORECASE)
SQL_INJECTION_PATTERN = re.compile(
    r"(\b(SELECT|INSERT|UPDATE|DELETE|DROP|UNION|ALTER|CREATE)\b)",
    re.IGNORECASE,
)


def sanitize_query(query: str, max_length: int = MAX_QUERY_LENGTH) -> str:
    """
    Sanitize user query input for safety.

    Args:
        query: Raw user input
        max_length: Maximum allowed length (default 1000)

    Returns:
        Sanitized query string

    Examples:
        >>> sanitize_query("<script>alert('xss')</script>Hello")
        'Hello'
        >>> sanitize_query("What is ROS2?")
        'What is ROS2?'
    """
    if not query:
        return ""

    # Step 1: Strip leading/trailing whitespace
    result = query.strip()

    # Step 2: Remove script tags
    result = SCRIPT_PATTERN.sub("", result)

    # Step 3: Remove HTML tags
    result = strip_html_tags(result)

    # Step 4: HTML-escape special characters
    result = html.escape(result)

    # Step 5: Unescape safe characters for readability
    result = result.replace("&amp;", "&")
    result = result.replace("&#x27;", "'")
    result = result.replace("&quot;", '"')

    # Step 6: Remove javascript: and data: URLs
    result = JAVASCRIPT_PATTERN.sub("", result)
    result = DATA_PATTERN.sub("", result)

    # Step 7: Remove event handlers
    result = EVENT_HANDLER_PATTERN.sub("", result)

    # Step 8: Normalize whitespace
    result = " ".join(result.split())

    # Step 9: Enforce length limit
    if len(result) > max_length:
        result = result[:max_length]

    return result


def strip_html_tags(text: str) -> str:
    """
    Remove all HTML tags from text.

    Args:
        text: Input text with potential HTML tags

    Returns:
        Text with HTML tags removed
    """
    # Pattern matches any HTML tag
    tag_pattern = re.compile(r"<[^>]+>")
    return tag_pattern.sub("", text)


def is_potentially_malicious(text: str) -> bool:
    """
    Check if text contains potentially malicious patterns.

    This is a heuristic check - it may have false positives for
    legitimate technical questions about SQL or JavaScript.

    Args:
        text: Text to check

    Returns:
        True if potentially malicious patterns detected
    """
    # Check for script tags
    if SCRIPT_PATTERN.search(text):
        return True

    # Check for event handlers
    if EVENT_HANDLER_PATTERN.search(text):
        return True

    # Check for javascript: URLs
    if JAVASCRIPT_PATTERN.search(text):
        return True

    return False


def sanitize_for_logging(text: str, max_length: int = 500) -> str:
    """
    Sanitize text for safe logging.

    Args:
        text: Text to sanitize
        max_length: Maximum length for log output

    Returns:
        Sanitized text safe for logging
    """
    if not text:
        return ""

    # Remove control characters except newlines and tabs
    result = "".join(
        char for char in text if char.isprintable() or char in "\n\t"
    )

    # Truncate if too long
    if len(result) > max_length:
        result = result[:max_length] + "..."

    return result


def validate_session_id(session_id: Optional[str]) -> Optional[str]:
    """
    Validate and sanitize session ID.

    Args:
        session_id: Session ID to validate

    Returns:
        Validated session ID or None if invalid
    """
    if not session_id:
        return None

    # Session ID should be alphanumeric with hyphens (UUID format)
    session_id = session_id.strip()

    # Allow UUID format: 8-4-4-4-12 hexadecimal characters
    uuid_pattern = re.compile(
        r"^[0-9a-f]{8}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{12}$",
        re.IGNORECASE,
    )

    if uuid_pattern.match(session_id):
        return session_id.lower()

    # Also allow simple alphanumeric IDs (max 64 chars)
    simple_pattern = re.compile(r"^[a-zA-Z0-9_-]{1,64}$")
    if simple_pattern.match(session_id):
        return session_id

    return None


def sanitize_source_path(path: str) -> str:
    """
    Sanitize file/directory path input.

    Args:
        path: Path to sanitize

    Returns:
        Sanitized path

    Raises:
        ValueError: If path contains dangerous patterns
    """
    if not path:
        return ""

    # Normalize path
    path = path.strip()

    # Check for path traversal attempts
    if ".." in path:
        raise ValueError("Path traversal not allowed")

    # Remove leading slashes for relative paths
    path = path.lstrip("/\\")

    # Only allow alphanumeric, underscore, hyphen, forward slash, dot
    safe_pattern = re.compile(r"^[a-zA-Z0-9_\-./]+$")
    if not safe_pattern.match(path):
        raise ValueError("Invalid characters in path")

    return path
