#!/usr/bin/env python3
"""
Test script to check Qdrant connection
"""

import os
import sys
from pathlib import Path

# Add the backend src directory to the path
sys.path.insert(0, str(Path(__file__).parent / "backend" / "src"))

from dotenv import load_dotenv

# Load environment variables
load_dotenv(dotenv_path=Path(__file__).parent / ".backend.env")

print("Environment variables:")
print(f"QDRANT_URL: {os.getenv('QDRANT_URL')}")
print(f"QDRANT_HOST: {os.getenv('QDRANT_HOST')}")
print(f"QDRANT_PORT: {os.getenv('QDRANT_PORT')}")
print(f"QDRANT_API_KEY: {'***' if os.getenv('QDRANT_API_KEY') else 'Not set'}")

try:
    from vector_store.qdrant_client import QdrantManager

    print("\nCreating QdrantManager...")
    qdrant_manager = QdrantManager()

    print(f"Client host: {getattr(qdrant_manager.client._client, 'host', 'Unknown')}")
    print(f"Client port: {getattr(qdrant_manager.client._client, 'port', 'Unknown')}")

    print("\nTesting connection...")
    collections = qdrant_manager.client.get_collections()
    print(f"Connected successfully! Available collections: {[c.name for c in collections.collections]}")

except Exception as e:
    print(f"Error connecting to Qdrant: {e}")
    print("This is expected if Qdrant is not running or credentials are incorrect.")