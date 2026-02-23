# =============================================================================
# RAG Chatbot Backend - Sanitizer Unit Tests
# =============================================================================
# T034: Unit tests for input sanitizer
# =============================================================================

import pytest

from app.utils.sanitizer import (
    is_potentially_malicious,
    sanitize_for_logging,
    sanitize_query,
    sanitize_source_path,
    strip_html_tags,
    validate_session_id,
)


class TestSanitizeQuery:
    """Tests for sanitize_query function."""

    def test_normal_query(self):
        """Normal queries should pass through unchanged."""
        query = "What is ROS2?"
        result = sanitize_query(query)
        assert result == "What is ROS2?"

    def test_strips_whitespace(self):
        """Leading/trailing whitespace should be stripped."""
        query = "  What is ROS2?  "
        result = sanitize_query(query)
        assert result == "What is ROS2?"

    def test_removes_script_tags(self):
        """Script tags should be removed."""
        query = "<script>alert('xss')</script>What is ROS2?"
        result = sanitize_query(query)
        assert "<script>" not in result
        assert "alert" not in result
        assert "ROS2" in result

    def test_removes_html_tags(self):
        """HTML tags should be removed."""
        query = "<b>What</b> is <i>ROS2</i>?"
        result = sanitize_query(query)
        assert "<b>" not in result
        assert "</b>" not in result
        assert "What is ROS2?" in result

    def test_removes_event_handlers(self):
        """Event handlers should be removed."""
        query = "onclick=alert('xss') What is ROS2?"
        result = sanitize_query(query)
        assert "onclick" not in result

    def test_removes_javascript_urls(self):
        """JavaScript URLs should be removed."""
        query = "javascript:alert('xss') What is ROS2?"
        result = sanitize_query(query)
        assert "javascript:" not in result

    def test_removes_data_urls(self):
        """Data URLs should be removed."""
        query = "data:text/html,<script>alert('xss')</script>"
        result = sanitize_query(query)
        assert "data:" not in result

    def test_enforces_length_limit(self):
        """Long queries should be truncated."""
        query = "x" * 2000
        result = sanitize_query(query, max_length=1000)
        assert len(result) == 1000

    def test_normalizes_whitespace(self):
        """Multiple spaces should be normalized."""
        query = "What    is   ROS2?"
        result = sanitize_query(query)
        assert result == "What is ROS2?"

    def test_empty_query(self):
        """Empty query should return empty string."""
        assert sanitize_query("") == ""
        assert sanitize_query("   ") == ""

    def test_preserves_safe_characters(self):
        """Safe characters should be preserved."""
        query = "What is ROS2? It's great!"
        result = sanitize_query(query)
        assert "ROS2" in result
        assert "?" in result

    def test_complex_xss_attempt(self):
        """Complex XSS attempts should be neutralized."""
        query = '<img src="x" onerror="alert(1)">What is ROS2?'
        result = sanitize_query(query)
        assert "onerror" not in result
        assert "<img" not in result


class TestStripHtmlTags:
    """Tests for strip_html_tags function."""

    def test_removes_simple_tags(self):
        """Simple HTML tags should be removed."""
        assert strip_html_tags("<b>bold</b>") == "bold"
        assert strip_html_tags("<p>paragraph</p>") == "paragraph"

    def test_removes_nested_tags(self):
        """Nested tags should be removed."""
        assert strip_html_tags("<div><p>text</p></div>") == "text"

    def test_removes_self_closing_tags(self):
        """Self-closing tags should be removed."""
        assert strip_html_tags("text<br/>more") == "textmore"

    def test_preserves_text_without_tags(self):
        """Text without tags should be unchanged."""
        assert strip_html_tags("plain text") == "plain text"


class TestIsPotentiallyMalicious:
    """Tests for is_potentially_malicious function."""

    def test_safe_query(self):
        """Safe queries should return False."""
        assert not is_potentially_malicious("What is ROS2?")

    def test_detects_script_tags(self):
        """Script tags should be detected."""
        assert is_potentially_malicious("<script>alert('xss')</script>")

    def test_detects_event_handlers(self):
        """Event handlers should be detected."""
        assert is_potentially_malicious("onclick=alert('xss')")
        assert is_potentially_malicious("onerror=alert('xss')")

    def test_detects_javascript_urls(self):
        """JavaScript URLs should be detected."""
        assert is_potentially_malicious("javascript:alert('xss')")


class TestSanitizeForLogging:
    """Tests for sanitize_for_logging function."""

    def test_normal_text(self):
        """Normal text should pass through."""
        assert sanitize_for_logging("normal text") == "normal text"

    def test_truncates_long_text(self):
        """Long text should be truncated."""
        text = "x" * 1000
        result = sanitize_for_logging(text, max_length=500)
        assert len(result) == 503  # 500 + "..."
        assert result.endswith("...")

    def test_removes_control_characters(self):
        """Control characters should be removed."""
        text = "text\x00with\x01control"
        result = sanitize_for_logging(text)
        assert "\x00" not in result
        assert "\x01" not in result

    def test_preserves_newlines(self):
        """Newlines should be preserved."""
        text = "line1\nline2"
        assert sanitize_for_logging(text) == "line1\nline2"

    def test_empty_text(self):
        """Empty text should return empty string."""
        assert sanitize_for_logging("") == ""


class TestValidateSessionId:
    """Tests for validate_session_id function."""

    def test_valid_uuid(self):
        """Valid UUIDs should pass."""
        uuid = "550e8400-e29b-41d4-a716-446655440000"
        assert validate_session_id(uuid) == uuid.lower()

    def test_valid_uuid_uppercase(self):
        """Uppercase UUIDs should be lowercased."""
        uuid = "550E8400-E29B-41D4-A716-446655440000"
        assert validate_session_id(uuid) == uuid.lower()

    def test_valid_simple_id(self):
        """Simple alphanumeric IDs should pass."""
        assert validate_session_id("session123") == "session123"
        assert validate_session_id("user_session-1") == "user_session-1"

    def test_invalid_format(self):
        """Invalid formats should return None."""
        assert validate_session_id("<script>") is None
        assert validate_session_id("../../../etc/passwd") is None

    def test_none_input(self):
        """None input should return None."""
        assert validate_session_id(None) is None

    def test_empty_string(self):
        """Empty string should return None."""
        assert validate_session_id("") is None


class TestSanitizeSourcePath:
    """Tests for sanitize_source_path function."""

    def test_valid_path(self):
        """Valid paths should pass."""
        assert sanitize_source_path("docs/") == "docs/"
        assert sanitize_source_path("docs/chapter1.md") == "docs/chapter1.md"

    def test_rejects_path_traversal(self):
        """Path traversal attempts should raise error."""
        with pytest.raises(ValueError, match="traversal"):
            sanitize_source_path("../../../etc/passwd")

    def test_rejects_invalid_characters(self):
        """Invalid characters should raise error."""
        with pytest.raises(ValueError, match="Invalid"):
            sanitize_source_path("docs/<script>")

    def test_strips_leading_slash(self):
        """Leading slashes should be stripped."""
        assert sanitize_source_path("/docs/") == "docs/"

    def test_empty_path(self):
        """Empty path should return empty string."""
        assert sanitize_source_path("") == ""
