"""
Custom exceptions for the RAG Chatbot system
"""

from fastapi import HTTPException, status
from typing import Optional


class BaseRAGException(Exception):
    """Base exception class for RAG Chatbot system"""
    def __init__(self, message: str, error_code: Optional[str] = None):
        self.message = message
        self.error_code = error_code
        super().__init__(self.message)


class ChatSessionNotFoundException(BaseRAGException):
    """Raised when a chat session is not found"""
    pass


class InvalidRetrievalModeException(BaseRAGException):
    """Raised when an invalid retrieval mode is specified"""
    pass


class ContentRetrievalException(BaseRAGException):
    """Raised when content retrieval fails"""
    pass


class VectorStoreException(BaseRAGException):
    """Raised when vector store operations fail"""
    pass


class DatabaseException(BaseRAGException):
    """Raised when database operations fail"""
    pass


class CohereException(BaseRAGException):
    """Raised when Cohere API operations fail"""
    pass


class ValidationError(BaseRAGException):
    """Raised when input validation fails"""
    pass


# HTTP Exception helpers
def raise_session_not_found(session_id: str) -> HTTPException:
    """Raise HTTP 404 for missing session"""
    return HTTPException(
        status_code=status.HTTP_404_NOT_FOUND,
        detail=f"Chat session with ID {session_id} not found"
    )


def raise_invalid_mode(mode: str) -> HTTPException:
    """Raise HTTP 400 for invalid retrieval mode"""
    return HTTPException(
        status_code=status.HTTP_400_BAD_REQUEST,
        detail=f"Invalid retrieval mode: {mode}. Must be 'global' or 'selected-text'"
    )


def raise_content_not_found(query: str) -> HTTPException:
    """Raise HTTP 404 when no content is found for query"""
    return HTTPException(
        status_code=status.HTTP_404_NOT_FOUND,
        detail=f"No relevant content found for query: {query}"
    )


def raise_bad_request(message: str) -> HTTPException:
    """Raise HTTP 400 for bad requests"""
    return HTTPException(
        status_code=status.HTTP_400_BAD_REQUEST,
        detail=message
    )


def raise_internal_error(message: str) -> HTTPException:
    """Raise HTTP 500 for internal errors"""
    return HTTPException(
        status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
        detail=message
    )