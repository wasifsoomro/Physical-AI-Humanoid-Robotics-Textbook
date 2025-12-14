#!/usr/bin/env python3
"""
Script to create database tables in Neon database - fresh approach
"""

import os
from sqlalchemy import create_engine, Column, Integer, String, Text, DateTime, ForeignKey
from sqlalchemy.dialects.postgresql import UUID as PostgresUUID, JSONB
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.sql import func
from sqlalchemy.orm import sessionmaker
from dotenv import load_dotenv
import uuid

# Load environment variables
load_dotenv(dotenv_path="D:/Web 3 Metavers and GenAI/Wasif/My Hackathon/humanoid-robotics-book/.backend.env")

def main():
    # Get the database URL directly
    database_url = os.getenv("DATABASE_URL")
    print(f"Using database URL: {database_url}")

    if not database_url:
        print("DATABASE_URL not found in environment")
        return 1

    # Create engine directly with the database URL
    engine = create_engine(database_url)

    # Define models locally to avoid import issues
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
        source_chunks = Column(String(2000))  # Store as JSON string

    try:
        # Create all tables
        Base.metadata.create_all(bind=engine)
        print("Database tables created successfully in Neon database!")

        # Check what tables were created by reflecting them
        from sqlalchemy import inspect
        inspector = inspect(engine)
        tables = inspector.get_table_names()
        print(f"Tables in database: {tables}")

        return 0
    except Exception as e:
        print(f"Error creating tables: {e}")
        import traceback
        traceback.print_exc()
        return 1

if __name__ == "__main__":
    exit(main())