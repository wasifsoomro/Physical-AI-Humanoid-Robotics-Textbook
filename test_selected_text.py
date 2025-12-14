#!/usr/bin/env python3
"""
Test script to verify the selected text functionality
"""

import requests
import json

def test_selected_text_functionality():
    # Test the chatbot with selected text mode
    url = "http://localhost:8002/api/chat"

    # Create a test session
    session_response = requests.post("http://localhost:8002/api/session",
                                    json={"mode": "selected-text"})
    if session_response.status_code == 200:
        session_data = session_response.json()
        session_id = session_data["session_id"]
        print(f"Created session: {session_id}")
    else:
        print(f"Failed to create session: {session_response.text}")
        return

    # Test query with selected text
    selected_text = "Humanoid robotics is a specialized field of robotics that focuses on designing and building robots with a human-like structure and behavior. These robots are engineered to mimic human movements, interactions, and even communication to facilitate easier collaboration and interaction with humans in various environments."

    test_message = {
        "session_id": session_id,
        "message": "Explain this text in simple terms",
        "mode": "selected-text",
        "selected_text": selected_text
    }

    print(f"Sending test message with selected text: {test_message['message']}")
    print(f"Selected text preview: {selected_text[:100]}...")

    response = requests.post(url, json=test_message)

    if response.status_code == 200:
        result = response.json()
        print(f"Response received: {result.get('response', 'No response field')[:300]}...")
        print(f"Retrieved context count: {len(result.get('retrieved_context', []))}")

        if result.get('retrieved_context'):
            print(f"First context content preview: {result['retrieved_context'][0].get('content', '')[:100]}...")
    else:
        print(f"Error: {response.status_code} - {response.text}")

    print("\nSelected text functionality test completed!")

if __name__ == "__main__":
    test_selected_text_functionality()