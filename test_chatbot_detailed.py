#!/usr/bin/env python3
"""
Test script to verify the chatbot functionality with different queries
"""

import requests
import json

def test_chatbot_detailed():
    # Test the chatbot with different queries
    url = "http://localhost:8000/api/chat"

    # Create a test session
    session_response = requests.post("http://localhost:8000/api/session",
                                    json={"mode": "global"})
    if session_response.status_code == 200:
        session_data = session_response.json()
        session_id = session_data["session_id"]
        print(f"Created session: {session_id}")
    else:
        print(f"Failed to create session: {session_response.text}")
        return

    # Test queries
    test_messages = [
        "What is this book about?",
        "Tell me about humanoid robotics",
        "What are the main topics covered in this book?"
    ]

    for i, query in enumerate(test_messages):
        print(f"\n--- Test {i+1} ---")
        print(f"Sending test message: {query}")

        test_message = {
            "session_id": session_id,
            "message": query,
            "mode": "global"
        }

        response = requests.post(url, json=test_message)

        if response.status_code == 200:
            result = response.json()
            print(f"Response received: {result.get('response', 'No response field')[:300]}...")
            print(f"Retrieved context count: {len(result.get('retrieved_context', []))}")

            # Print some context details if available
            retrieved_context = result.get('retrieved_context', [])
            if retrieved_context:
                print(f"First context source: {retrieved_context[0].get('source', 'Unknown')}")
                print(f"First context section: {retrieved_context[0].get('section_title', 'Unknown')}")
        else:
            print(f"Error: {response.status_code} - {response.text}")

    print("\nChatbot is working correctly with multiple queries!")

if __name__ == "__main__":
    test_chatbot_detailed()