#!/usr/bin/env python3
"""
Test script to verify the retrieval functionality works properly
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

def test_retrieval():
    print("Testing retrieval functionality...")

    # Import required modules
    from src.services.retrieval_service import RetrievalService
    from src.services.conversation_service import ConversationService

    # Test 1: Direct retrieval service
    print("\n1. Testing RetrievalService directly...")
    retrieval_service = RetrievalService()

    test_query = "humanoid robotics"
    print(f"   Query: '{test_query}'")

    try:
        results = retrieval_service.retrieve_content(test_query, mode='global', limit=2)
        print(f"   Found {len(results)} results")
        for i, result in enumerate(results):
            print(f"     Result {i+1}: {result['section_title']} (score: {result['score']:.3f})")
    except Exception as e:
        print(f"   Error in retrieval: {e}")

    # Test 2: Conversation service (which also uses retrieval)
    print("\n2. Testing ConversationService...")
    conversation_service = ConversationService()

    try:
        # Create a mock session ID
        session_id = str(uuid.uuid4())
        user_message = "What is humanoid robotics?"

        print(f"   Session ID: {session_id}")
        print(f"   Message: '{user_message}'")

        result = conversation_service.process_message(
            session_id=session_id,
            user_message=user_message,
            mode="global",
            selected_text=None,
            db=None  # This might cause issues, but let's see
        )

        print(f"   Response: {result['response'][:100]}...")
        print(f"   Retrieved context: {len(result.get('retrieved_context', []))} items")

    except Exception as e:
        print(f"   Error in conversation service: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    test_retrieval()