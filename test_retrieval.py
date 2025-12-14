#!/usr/bin/env python3
"""
Test script to check the retrieval functionality directly
"""

import os
import sys
from pathlib import Path

# Add the project root and backend src to the path
project_root = Path(__file__).parent
sys.path.insert(0, str(project_root))
sys.path.insert(0, str(project_root / "backend" / "src"))

from dotenv import load_dotenv

# Import using the path we added
from services.retrieval_service import RetrievalService

# Load environment variables
load_dotenv(dotenv_path=Path(__file__).parent / ".backend.env")

print("Testing retrieval service directly...")

try:
    # Initialize the retrieval service
    retrieval_service = RetrievalService()

    print("Retrieval service initialized successfully")

    # Test a query
    test_query = "What is this book about?"
    print(f"Testing query: {test_query}")

    results = retrieval_service.retrieve_content(
        query=test_query,
        mode="global",
        limit=3
    )

    print(f"Retrieved {len(results)} results")

    for i, result in enumerate(results):
        print(f"\nResult {i+1}:")
        print(f"  Content: {result.get('content', '')[:200]}...")
        print(f"  Source: {result.get('source', '')}")
        print(f"  Score: {result.get('score', '')}")
        print(f"  Section: {result.get('section_title', '')}")

    if len(results) == 0:
        print("\nNo results found - this could indicate:")
        print("1. No content has been ingested into Qdrant yet")
        print("2. The ingestion process failed")
        print("3. There's an issue with the embedding generation")

except Exception as e:
    print(f"Error testing retrieval service: {e}")
    import traceback
    traceback.print_exc()