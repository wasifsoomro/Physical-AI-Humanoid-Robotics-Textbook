"""
Authentication utilities for the RAG Chatbot system
"""

from fastapi import Depends, HTTPException, status
from typing import Optional, Union
from uuid import UUID
from ..utils.exceptions import raise_bad_request
import uuid
from datetime import datetime, timedelta
from ..database.models import ChatSession
from sqlalchemy.orm import Session
from ..database.database import get_db


def generate_session_id() -> str:
    """Generate a unique session ID"""
    return str(uuid.uuid4())


def validate_session_id(session_id: Union[str, UUID]) -> bool:
    """Validate that a session ID is a proper UUID"""
    try:
        if isinstance(session_id, UUID):
            # If it's already a UUID object, it's valid
            return True
        else:
            # If it's a string, try to parse it
            uuid.UUID(session_id)
            return True
    except (ValueError, TypeError):
        return False


def create_session_if_not_exists(db: Session, session_id: str, mode: str = "global") -> ChatSession:
    """Create a session if it doesn't exist, otherwise return existing session"""
    from sqlalchemy import and_
    from uuid import UUID

    # Convert string session_id to UUID for database query
    try:
        uuid_session_id = UUID(session_id)
    except ValueError:
        raise raise_bad_request(f"Invalid session ID format: {session_id}")

    # Check if session exists
    existing_session = db.query(ChatSession).filter(ChatSession.session_id == uuid_session_id).first()

    if existing_session:
        # Update last active time
        existing_session.last_active = datetime.utcnow()
        db.commit()
        db.refresh(existing_session)
        return existing_session
    else:
        # Convert string session_id to UUID for creating new session
        try:
            uuid_session_id = UUID(session_id)
        except ValueError:
            raise raise_bad_request(f"Invalid session ID format: {session_id}")

        # Create new session
        new_session = ChatSession(
            session_id=uuid_session_id,
            mode=mode,
            created_at=datetime.utcnow(),
            last_active=datetime.utcnow()
        )
        db.add(new_session)
        db.commit()
        db.refresh(new_session)
        return new_session