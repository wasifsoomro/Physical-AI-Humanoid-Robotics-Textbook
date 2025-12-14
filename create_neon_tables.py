#!/usr/bin/env python3
"""
Script to create database tables in Neon database
"""

import os
import sys
from pathlib import Path

# Add the backend src directory to the path
sys.path.insert(0, str(Path(__file__).parent / "backend" / "src"))

from dotenv import load_dotenv
from database.database import engine
from database.models import Base

# Load environment variables
load_dotenv(dotenv_path=Path(__file__).parent / ".backend.env")

def main():
    print("Creating tables in Neon database...")

    # Get the database URL to confirm we're using the right one
    database_url = os.getenv("DATABASE_URL")
    print(f"Using database URL: {database_url}")

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
    sys.exit(main())