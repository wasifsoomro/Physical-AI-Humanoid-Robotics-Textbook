#!/usr/bin/env python3
"""
Script to create database tables
"""

import os
import sys
from pathlib import Path

# Add the backend src directory to the path
sys.path.insert(0, str(Path(__file__).parent / "backend" / "src"))

from dotenv import load_dotenv
from database.database import create_tables, engine
from database.models import Base

# Load environment variables
load_dotenv(dotenv_path=Path(__file__).parent / ".backend.env")

def main():
    print("Creating database tables...")

    try:
        # Create all tables
        Base.metadata.create_all(bind=engine)
        print("Database tables created successfully!")

        # Check what tables were created
        tables = Base.metadata.tables.keys()
        print(f"Created tables: {list(tables)}")

        return 0
    except Exception as e:
        print(f"Error creating tables: {e}")
        return 1

if __name__ == "__main__":
    sys.exit(main())