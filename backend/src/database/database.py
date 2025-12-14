"""
Database connection and session management
"""

from sqlalchemy import create_engine
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import sessionmaker
from sqlalchemy.pool import QueuePool
import os
from dotenv import load_dotenv
from pathlib import Path

# Load the backend environment file
env_path = Path(__file__).parent.parent.parent.parent / ".backend.env"
if env_path.exists():
    load_dotenv(env_path)
else:
    load_dotenv()  # Fallback to default .env

# Use the database URL from environment, fallback to SQLite
DATABASE_URL = os.getenv("DATABASE_URL", "sqlite:///./rag_chatbot.db")

# Check if DATABASE_URL uses PostgreSQL
if DATABASE_URL.startswith("postgresql://") or DATABASE_URL.startswith("postgres://"):
    # Use PostgreSQL
    engine = create_engine(
        DATABASE_URL,
        poolclass=QueuePool,
        pool_size=10,
        max_overflow=20,
        pool_pre_ping=True,
        pool_recycle=300,
    )
else:
    # Use SQLite
    engine = create_engine(
        DATABASE_URL,
        poolclass=QueuePool,
        pool_size=10,
        max_overflow=20,
        pool_pre_ping=True,
        pool_recycle=300,
        connect_args={"check_same_thread": False}  # Required for SQLite with threading
    )

SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)

Base = declarative_base()


def get_db():
    """Dependency for getting database session"""
    db = SessionLocal()
    try:
        return db
    finally:
        db.close()


def create_tables():
    """Create all database tables"""
    from .models import Base
    Base.metadata.create_all(bind=engine)