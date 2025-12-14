"""
Input validation and sanitization utilities for the RAG Chatbot system
"""
import re
from typing import Optional, Union
from bleach import clean
import html


def sanitize_input(text: str) -> str:
    """
    Sanitize user input to prevent XSS and other injection attacks
    """
    if not text:
        return text

    # Remove potentially dangerous HTML tags and attributes
    sanitized = clean(text, strip=True, strip_comments=True)

    # Unescape any HTML entities that might be malicious
    sanitized = html.unescape(sanitized)

    # Remove any remaining script tags (double check)
    sanitized = re.sub(r'<script[^>]*>.*?</script>', '', sanitized, flags=re.IGNORECASE | re.DOTALL)
    sanitized = re.sub(r'javascript:', '', sanitized, flags=re.IGNORECASE)
    sanitized = re.sub(r'on\w+\s*=', '', sanitized, flags=re.IGNORECASE)

    return sanitized.strip()


def validate_session_id(session_id: str) -> bool:
    """
    Validate session ID format to prevent session fixation and other attacks
    """
    if not session_id:
        return False

    # Check if it's a valid UUID format (or at least looks like one)
    # UUID format: 8-4-4-4-12 hex characters
    uuid_pattern = r'^[0-9a-f]{8}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{12}$'

    # Or if it's a hex string of appropriate length
    hex_pattern = r'^[0-9a-f]{32,}$'

    return bool(re.match(uuid_pattern, session_id) or re.match(hex_pattern, session_id))


def validate_message_content(message: str) -> tuple[bool, str]:
    """
    Validate message content for length and potentially dangerous content

    Returns:
        tuple[bool, str]: (is_valid, error_message)
    """
    if not message or not message.strip():
        return False, "Message cannot be empty"

    # Check length
    if len(message) > 2000:  # Reasonable limit to prevent abuse
        return False, "Message is too long. Please keep your question under 2000 characters."

    # Check for potential prompt injection attempts
    injection_patterns = [
        r'ignore\s+previous',
        r'forget\s+previous',
        r'system\s+prompt',
        r'role\s+play',
        r'<\|.*?\|>',  # Special tokens
        r'\{\{.*?\}\}',  # Template-like patterns
    ]

    for pattern in injection_patterns:
        if re.search(pattern, message, re.IGNORECASE):
            return False, "Message contains potentially unsafe content"

    return True, ""


def validate_retrieval_mode(mode: str) -> tuple[bool, str]:
    """
    Validate retrieval mode parameter

    Returns:
        tuple[bool, str]: (is_valid, error_message)
    """
    if not mode:
        return False, "Retrieval mode cannot be empty"

    valid_modes = ["global", "selected-text"]
    if mode not in valid_modes:
        return False, f"Invalid mode: {mode}. Must be one of {valid_modes}"

    return True, ""


def validate_selected_text(selected_text: Optional[str]) -> tuple[bool, str]:
    """
    Validate selected text parameter

    Returns:
        tuple[bool, str]: (is_valid, error_message)
    """
    if not selected_text:
        return True, ""  # Selected text is optional

    if len(selected_text) > 5000:  # Reasonable limit for selected text
        return False, "Selected text is too long. Please select a smaller portion of text."

    return True, ""


def validate_context_window_size(size: Union[int, str]) -> tuple[bool, str]:
    """
    Validate context window size parameter

    Returns:
        tuple[bool, str]: (is_valid, error_message)
    """
    try:
        size_int = int(size)
        if size_int <= 0 or size_int > 50:
            return False, "Context window size must be between 1 and 50"
        return True, ""
    except (ValueError, TypeError):
        return False, "Context window size must be a valid integer"


def validate_query_params(query: str, mode: str, selected_text: Optional[str] = None) -> tuple[bool, str]:
    """
    Validate all query parameters together

    Returns:
        tuple[bool, str]: (is_valid, error_message)
    """
    # Validate message content
    is_valid, error_msg = validate_message_content(query)
    if not is_valid:
        return False, error_msg

    # Validate mode
    is_valid, error_msg = validate_retrieval_mode(mode)
    if not is_valid:
        return False, error_msg

    # Validate selected text if in selected-text mode
    if mode == "selected-text" and selected_text:
        is_valid, error_msg = validate_selected_text(selected_text)
        if not is_valid:
            return False, error_msg

    return True, ""


def escape_special_characters(text: str) -> str:
    """
    Escape special characters that could be used in injection attacks
    """
    if not text:
        return text

    # Escape special regex characters
    special_chars = ['\\', '.', '^', '$', '*', '+', '?', '{', '}', '[', ']', '|', '(', ')']
    escaped_text = text
    for char in special_chars:
        escaped_text = escaped_text.replace(char, '\\' + char)

    return escaped_text


def is_safe_content(content: str) -> bool:
    """
    Check if content is safe from common injection patterns
    """
    if not content:
        return True

    dangerous_patterns = [
        r'<script',  # Script tags
        r'javascript:',  # JavaScript URLs
        r'vbscript:',  # VBScript URLs
        r'on\w+\s*=',  # Event handlers
        r'<iframe',  # Iframe tags
        r'<object',  # Object tags
        r'<embed',  # Embed tags
        r'eval\s*\(',  # Eval function calls
        r'expression\s*\(',  # CSS expressions
    ]

    for pattern in dangerous_patterns:
        if re.search(pattern, content, re.IGNORECASE):
            return False

    return True