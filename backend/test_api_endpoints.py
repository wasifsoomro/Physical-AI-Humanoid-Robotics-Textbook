#!/usr/bin/env python3
"""
Test script to verify the API endpoints are working properly
"""
import sys
from pathlib import Path
import os
from dotenv import load_dotenv
import time
import uuid

# Add src to path
sys.path.insert(0, str(Path('src').resolve()))

# Load environment variables
load_dotenv('../.backend.env')

def test_api_endpoints():
    print("Testing API endpoints...")

    # Import required modules
    from src.services.retrieval_service import RetrievalService
    from src.api.chat_routes import retrieve_endpoint
    from fastapi import Request
    from sqlalchemy.orm import Session
    from unittest.mock import Mock

    # Create a mock request object
    mock_request = Mock(spec=Request)
    mock_request.url.path = "/api/retrieve"

    # Create a mock response object
    from starlette.responses import Response
    mock_response = Response()

    # Create a mock database session
    mock_db = Mock(spec=Session)

    # Create a test request
    from src.models.chat import ChatRequest
    test_request = ChatRequest(
        session_id=uuid.UUID("123e4567-e89b-12d3-a456-426614174000"),
        message="humanoid robotics",
        mode="global"
    )

    print("\nTesting the retrieve_endpoint function directly...")
    try:
        result = retrieve_endpoint(
            request=test_request,
            response=mock_response,
            db=mock_db
        )
        print(f"   Success! Retrieved {len(result['results'])} results")
        for i, result_item in enumerate(result['results'][:2]):  # Show first 2
            print(f"     Result {i+1}: {result_item.get('section_title', 'N/A')} - Score: {result_item.get('score', 0):.3f}")
    except Exception as e:
        print(f"   Error calling retrieve_endpoint: {e}")
        import traceback
        traceback.print_exc()

    # Test the health check functionality too
    print("\nTesting health endpoint...")
    try:
        from src.api.main import health_check
        health_result = health_check()
        print(f"   Health check: {health_result}")
    except Exception as e:
        print(f"   Error in health check: {e}")

    # Test the root endpoint
    print("\nTesting root endpoint...")
    try:
        from src.api.main import read_root
        root_result = read_root()
        print(f"   Root endpoint: {root_result}")
    except Exception as e:
        print(f"   Error in root endpoint: {e}")

if __name__ == "__main__":
    test_api_endpoints()