"""
Pydantic models for chat-related data structures
"""

from pydantic import BaseModel, Field, field_validator
from typing import List, Optional, Dict, Any, Union
from uuid import UUID
from datetime import datetime


class ChatSessionBase(BaseModel):
    mode: str = Field(..., description="Retrieval mode: 'global' or 'selected-text'")
    metadata_json: Optional[Dict[str, Any]] = Field(default=None, description="Additional session metadata")


class ChatSessionCreate(ChatSessionBase):
    user_id: Optional[UUID] = None


class ChatSession(ChatSessionBase):
    session_id: UUID
    user_id: UUID
    created_at: datetime
    last_active: datetime

    class Config:
        from_attributes = True


class ChatMessageBase(BaseModel):
    role: str = Field(..., description="Role of the message sender: 'user' or 'assistant'")
    content: str = Field(..., description="The actual message content")
    retrieved_context: Optional[Dict[str, Any]] = Field(default=None, description="Context fragments used to generate response")
    source_chunks: Optional[List[UUID]] = Field(default=None, description="IDs of source content chunks used")


class ChatMessageCreate(ChatMessageBase):
    session_id: UUID


class ChatMessage(ChatMessageBase):
    message_id: UUID
    session_id: UUID
    timestamp: datetime

    class Config:
        from_attributes = True


class ChatRequest(BaseModel):
    session_id: UUID
    message: str = Field(
        ...,
        min_length=1,
        max_length=2000,  # Increased from the 1000 char limit in the service
        description="User's question about book content"
    )
    mode: str = Field(
        default="global",
        pattern=r"^(global|selected-text)$",  # Only allow specific values
        description="Retrieval mode: 'global' or 'selected-text'"
    )
    selected_text: Optional[str] = Field(
        default=None,
        max_length=5000,  # Reasonable limit for selected text
        description="Optional text for selected-text mode"
    )

    @field_validator('message')
    @classmethod
    def validate_message_content(cls, v):
        # Remove potentially harmful characters/sequences
        # Basic sanitization - in production, use a proper HTML sanitizer if needed
        if v and ('<script' in v.lower() or 'javascript:' in v.lower() or 'vbscript:' in v.lower()):
            raise ValueError('Message contains potentially harmful content')
        return v.strip()  # Remove leading/trailing whitespace

    @field_validator('selected_text')
    @classmethod
    def validate_selected_text_content(cls, v):
        if v:
            # Remove potentially harmful characters/sequences
            if ('<script' in v.lower() or 'javascript:' in v.lower() or 'vbscript:' in v.lower()):
                raise ValueError('Selected text contains potentially harmful content')
            return v.strip()
        return v


class ChatResponse(BaseModel):
    session_id: UUID
    response: str = Field(..., description="AI-generated response based on book content")
    retrieved_context: Optional[List[Dict[str, Any]]] = Field(default=None, description="Retrieved context used to generate response")
    timestamp: datetime


class RetrieveRequest(BaseModel):
    query: str = Field(
        ...,
        min_length=1,
        max_length=2000,
        description="Search query"
    )
    mode: str = Field(
        default="global",
        pattern=r"^(global|selected-text)$",  # Only allow specific values
        description="Retrieval mode: 'global' or 'selected-text'"
    )
    selected_text: Optional[str] = Field(
        default=None,
        max_length=5000,
        description="Optional text for selected-text mode"
    )
    document_id: Optional[str] = Field(
        default=None,
        max_length=255,  # Reasonable limit for document ID
        description="Optional document ID for selected-text mode"
    )

    @field_validator('query')
    @classmethod
    def validate_query_content(cls, v):
        # Remove potentially harmful characters/sequences
        if v and ('<script' in v.lower() or 'javascript:' in v.lower() or 'vbscript:' in v.lower()):
            raise ValueError('Query contains potentially harmful content')
        return v.strip()

    @field_validator('selected_text')
    @classmethod
    def validate_retrieve_selected_text_content(cls, v):
        if v:
            # Remove potentially harmful characters/sequences
            if ('<script' in v.lower() or 'javascript:' in v.lower() or 'vbscript:' in v.lower()):
                raise ValueError('Selected text contains potentially harmful content')
            return v.strip()
        return v


class RetrieveResponse(BaseModel):
    results: List[Dict[str, Any]] = Field(..., description="List of retrieved results")


class ChatSessionCreateRequest(BaseModel):
    mode: str = Field(default="global", description="Retrieval mode: 'global' or 'selected-text'")
    user_id: Optional[UUID] = None