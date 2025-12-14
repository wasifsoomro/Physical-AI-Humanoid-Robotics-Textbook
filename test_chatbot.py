#!/usr/bin/env python3
"""
Test script to verify the chatbot functionality
"""

import requests
import json

def test_chatbot():
    # Test the chatbot with a simple query
    url = "http://localhost:8002/api/chat"

    # Create a test session
    session_response = requests.post("http://localhost:8002/api/session",
                                    json={"mode": "global"})
    if session_response.status_code == 200:
        session_data = session_response.json()
        session_id = session_data["session_id"]
        print(f"Created session: {session_id}")
    else:
        print(f"Failed to create session: {session_response.text}")
        return

    # Test query
    test_message = {
        "session_id": session_id,
        "message": "What is this book about?",
        "mode": "global"
    }

    print(f"Sending test message: {test_message['message']}")

    response = requests.post(url, json=test_message)

    if response.status_code == 200:
        result = response.json()
        print(f"Response received: {result.get('response', 'No response field')[:200]}...")
        print("Chatbot is working correctly!")
    else:
        print(f"Error: {response.status_code} - {response.text}")

if __name__ == "__main__":
    test_chatbot()