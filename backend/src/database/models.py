"""
Database models for the RAG Chatbot system
Defines the SQLAlchemy models for chat sessions and messages
"""

from sqlalchemy import Column, Integer, String, Text, DateTime, ForeignKey, JSON
from sqlalchemy.dialects.postgresql import UUID as PostgresUUID, JSONB
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.sql import func
import uuid

Base = declarative_base()


class ChatSession(Base):
    __tablename__ = "chat_sessions"

    session_id = Column(
        PostgresUUID(as_uuid=True),
        primary_key=True,
        default=uuid.uuid4
    )
    user_id = Column(
        PostgresUUID(as_uuid=True),
        default=uuid.uuid4
    )
    created_at = Column(DateTime, default=func.now(), nullable=False)
    last_active = Column(DateTime, default=func.now(), nullable=False)
    mode = Column(String(20), nullable=False)  # 'global' or 'selected-text'
    metadata_json = Column(JSONB)  # Additional session metadata


class ChatMessage(Base):
    __tablename__ = "chat_messages"

    message_id = Column(
        PostgresUUID(as_uuid=True),
        primary_key=True,
        default=uuid.uuid4
    )
    session_id = Column(
        PostgresUUID(as_uuid=True),
        ForeignKey("chat_sessions.session_id", ondelete="CASCADE"),
        nullable=False
    )
    role = Column(String(10), nullable=False)  # 'user' or 'assistant'
    content = Column(Text, nullable=False)
    timestamp = Column(DateTime, default=func.now(), nullable=False)
    retrieved_context = Column(JSONB)  # Context fragments used to generate response
    # For simplicity in this version, we'll store as text - in a real app you might need a different approach
    source_chunks = Column(String(2000))  # Store as JSON string