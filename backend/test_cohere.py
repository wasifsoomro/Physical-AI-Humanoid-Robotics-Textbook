#!/usr/bin/env python3
"""
Test script to check the Cohere client functionality
"""

import os
import sys
from pathlib import Path

# Add the project root to the path so .env can be loaded
project_root = Path(__file__).parent
sys.path.insert(0, str(project_root))

from dotenv import load_dotenv

# Load environment variables from project root
load_dotenv(dotenv_path=Path(__file__).parent.parent / ".backend.env")

from src.ai.cohere_client import CohereClient

print("Testing Cohere client...")

try:
    # Initialize the Cohere client
    cohere_client = CohereClient()

    print("Cohere client initialized successfully")

    # Test embedding generation
    test_text = "What is this book about?"
    print(f"Testing embedding generation for: {test_text}")

    embedding = cohere_client.generate_embedding(test_text)
    print(f"Generated embedding with {len(embedding)} dimensions")

    # Test chat completion
    print(f"Testing chat completion...")

    messages = [{"role": "user", "content": "What is this book about?"}]
    try:
        response = cohere_client.generate_chat_completion(
            messages=messages,
            context="This is a book about humanoid robotics and AI."
        )
        print(f"Chat completion response: {response[:200]}...")
    except Exception as chat_error:
        print(f"Chat completion failed with error: {chat_error}")
        print("This may be due to an invalid model, which should trigger fallback mechanism")
        print("Testing if fallback mechanism works...")

        # Test if the client properly falls back
        if hasattr(cohere_client, '_use_fallback') and cohere_client._use_fallback:
            print("Client is using fallback mechanism")
            # Try again with a known fallback scenario
            fallback_response = cohere_client.generate_chat_completion(
                messages=messages,
                context="This is a book about humanoid robotics and AI."
            )
            print(f"Fallback response: {fallback_response[:200]}...")
        else:
            print("Client is not using fallback, but let's try to set it manually")
            # Force using fallback for testing
            original_use_fallback = cohere_client._use_fallback
            cohere_client._use_fallback = True
            fallback_response = cohere_client.generate_chat_completion(
                messages=messages,
                context="This is a book about humanoid robotics and AI."
            )
            print(f"Forced fallback response: {fallback_response[:200]}...")
            cohere_client._use_fallback = original_use_fallback

except Exception as e:
    print(f"Error testing Cohere client: {e}")
    import traceback
    traceback.print_exc()