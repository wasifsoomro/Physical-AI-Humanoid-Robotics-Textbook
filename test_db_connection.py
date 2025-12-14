#!/usr/bin/env python3
"""
Test script to check Neon database connection
"""

import os
import sys
from pathlib import Path

# Add the backend src directory to the path
sys.path.insert(0, str(Path(__file__).parent / "backend" / "src"))

from dotenv import load_dotenv
from sqlalchemy import create_engine, text

# Load environment variables
load_dotenv(dotenv_path=Path(__file__).parent / ".backend.env")

def main():
    # Get the database URL from environment
    database_url = os.getenv("DATABASE_URL")
    print(f"Database URL: {database_url}")

    if not database_url:
        print("DATABASE_URL not found in environment")
        return 1

    try:
        # Create engine
        engine = create_engine(database_url)

        # Test the connection
        with engine.connect() as connection:
            result = connection.execute(text("SELECT 1"))
            print("Database connection successful!")
            print(f"Test query result: {result.fetchone()}")

        return 0
    except Exception as e:
        print(f"Database connection failed: {e}")
        return 1

if __name__ == "__main__":
    sys.exit(main())