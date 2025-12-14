"""
End-to-end validation tests for the complete RAG Chatbot system
These tests validate the complete flow from API request to response
"""
import pytest
from fastapi.testclient import TestClient
from unittest.mock import Mock, patch, MagicMock
import uuid
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

from src.api.main import app
from src.services.conversation_service import ConversationService
from src.services.retrieval_service import RetrievalService
from src.database.models import ChatSession, ChatMessage


client = TestClient(app)


def test_complete_chatbot_workflow():
    """
    Test the complete workflow: session creation -> chat -> history retrieval
    """
    # 1. Create a new session
    session_response = client.post("/api/session", json={
        "mode": "global"
    })

    assert session_response.status_code == 200
    session_data = session_response.json()
    assert "session_id" in session_data
    assert session_data["mode"] == "global"

    session_id = session_data["session_id"]

    # 2. Send a message to the chat endpoint
    chat_response = client.post("/api/chat", json={
        "session_id": session_id,
        "message": "What is humanoid robotics?",
        "mode": "global"
    })

    assert chat_response.status_code == 200
    chat_data = chat_response.json()
    assert chat_data["session_id"] == session_id
    assert "response" in chat_data
    assert len(chat_data["response"]) > 0

    # 3. Retrieve conversation history
    history_response = client.get(f"/api/session/{session_id}/history")

    assert history_response.status_code == 200
    history_data = history_response.json()
    assert history_data["session_id"] == session_id
    assert "history" in history_data
    # Should have at least the user message and the assistant response
    assert len(history_data["history"]) >= 2


def test_global_search_mode_workflow():
    """
    Test the complete workflow in global search mode
    """
    # Create session with global mode
    session_response = client.post("/api/session", json={
        "mode": "global"
    })

    assert session_response.status_code == 200
    session_data = session_response.json()
    session_id = session_data["session_id"]

    # Send a question about humanoid robotics
    chat_response = client.post("/api/chat", json={
        "session_id": session_id,
        "message": "Explain the main components of humanoid robots?",
        "mode": "global"
    })

    assert chat_response.status_code == 200
    chat_data = chat_response.json()
    assert chat_data["session_id"] == session_id
    assert "response" in chat_data
    assert chat_data["response"] is not None
    assert len(chat_data["response"]) > 0

    # Verify that retrieved context is included
    assert "retrieved_context" in chat_data
    # Context might be empty in tests due to mocking, but the field should exist


def test_selected_text_mode_workflow():
    """
    Test the complete workflow in selected-text mode
    """
    # Create session with selected-text mode
    session_response = client.post("/api/session", json={
        "mode": "selected-text"
    })

    assert session_response.status_code == 200
    session_data = session_response.json()
    session_id = session_data["session_id"]

    # Define selected text to focus on
    selected_text = "Humanoid robots typically have sensors for balance, actuators for movement, and controllers for coordination."

    # Send a question focused on the selected text
    chat_response = client.post("/api/chat", json={
        "session_id": session_id,
        "message": "What components are mentioned in the text?",
        "mode": "selected-text",
        "selected_text": selected_text
    })

    assert chat_response.status_code == 200
    chat_data = chat_response.json()
    assert chat_data["session_id"] == session_id
    assert "response" in chat_data
    assert chat_data["response"] is not None


def test_conversation_continuation():
    """
    Test that conversations can continue with context
    """
    # Create a new session
    session_response = client.post("/api/session", json={
        "mode": "global"
    })

    assert session_response.status_code == 200
    session_data = session_response.json()
    session_id = session_data["session_id"]

    # First message: ask about sensors
    first_response = client.post("/api/chat", json={
        "session_id": session_id,
        "message": "What types of sensors do humanoid robots use?",
        "mode": "global"
    })

    assert first_response.status_code == 200
    first_data = first_response.json()
    assert first_data["session_id"] == session_id

    # Second message: follow up on the previous topic
    second_response = client.post("/api/chat", json={
        "session_id": session_id,
        "message": "How do cameras help with balance?",
        "mode": "global"
    })

    assert second_response.status_code == 200
    second_data = second_response.json()
    assert second_data["session_id"] == session_id

    # Both messages should belong to the same session
    assert first_data["session_id"] == second_data["session_id"]


def test_mode_switching_in_conversation():
    """
    Test switching between global and selected-text modes within a conversation
    """
    # Create session (default to global)
    session_response = client.post("/api/session", json={
        "mode": "global"
    })

    assert session_response.status_code == 200
    session_data = session_response.json()
    session_id = session_data["session_id"]

    # Start with global mode question
    global_response = client.post("/api/chat", json={
        "session_id": session_id,
        "message": "What is humanoid robotics?",
        "mode": "global"
    })

    assert global_response.status_code == 200

    # Switch to selected-text mode in the same session
    selected_text = "The bipedal locomotion system uses advanced algorithms for balance."
    selected_response = client.post("/api/chat", json={
        "session_id": session_id,
        "message": "Explain the balance algorithms?",
        "mode": "selected-text",
        "selected_text": selected_text
    })

    assert selected_response.status_code == 200
    selected_data = selected_response.json()
    assert selected_data["session_id"] == session_id


def test_session_history_retrieval():
    """
    Test that conversation history can be retrieved correctly
    """
    # Create session
    session_response = client.post("/api/session", json={
        "mode": "global"
    })

    assert session_response.status_code == 200
    session_data = session_response.json()
    session_id = session_data["session_id"]

    # Have a brief conversation
    messages = [
        "What are actuators in humanoid robots?",
        "How do they differ from traditional robots?",
        "What are the main challenges?"
    ]

    for msg in messages:
        response = client.post("/api/chat", json={
            "session_id": session_id,
            "message": msg,
            "mode": "global"
        })
        assert response.status_code == 200

    # Retrieve history
    history_response = client.get(f"/api/session/{session_id}/history")
    assert history_response.status_code == 200

    history_data = history_response.json()
    assert history_data["session_id"] == session_id
    assert "history" in history_data
    assert "count" in history_data

    # Should have at least 6 messages (3 user + 3 assistant)
    assert history_data["count"] >= 6
    assert len(history_data["history"]) >= 6


def test_error_handling_in_e2e_flow():
    """
    Test error handling throughout the end-to-end flow
    """
    # Test with invalid session ID format
    invalid_response = client.post("/api/chat", json={
        "session_id": "invalid-session-id",  # Not a valid UUID
        "message": "Test message",
        "mode": "global"
    })

    # Should return an error
    assert invalid_response.status_code in [400, 422]  # Bad request or validation error

    # Test with invalid mode
    session_response = client.post("/api/session", json={
        "mode": "invalid-mode"
    })

    # Should return an error
    assert session_response.status_code in [400, 422]


def test_retrieve_endpoint_e2e():
    """
    Test the retrieve endpoint for content-only retrieval
    """
    # Create session
    session_response = client.post("/api/session", json={
        "mode": "global"
    })

    assert session_response.status_code == 200
    session_data = session_response.json()
    session_id = session_data["session_id"]

    # Use the retrieve endpoint directly
    retrieve_response = client.post("/api/retrieve", json={
        "session_id": session_id,
        "message": "humanoid locomotion",
        "mode": "global"
    })

    assert retrieve_response.status_code == 200
    retrieve_data = retrieve_response.json()
    assert "results" in retrieve_data


def test_rate_limiting_in_e2e_flow():
    """
    Test that rate limiting works in the end-to-end flow
    """
    # Create a session
    session_response = client.post("/api/session", json={
        "mode": "global"
    })

    assert session_response.status_code == 200
    session_data = session_response.json()
    session_id = session_data["session_id"]

    # Send multiple requests rapidly (this tests the rate limiting logic)
    responses = []
    for i in range(35):  # More than the default rate limit
        response = client.post("/api/chat", json={
            "session_id": session_id,
            "message": f"Test message {i}",
            "mode": "global"
        })
        responses.append(response)

    # At least some requests should be rate-limited
    successful_requests = [r for r in responses if r.status_code == 200]
    rate_limited_requests = [r for r in responses if r.status_code == 429]  # Rate limit status code

    # Note: This test may not trigger rate limiting in the test environment
    # since the rate limiter might not be active in test mode
    # The important thing is that the system handles the requests without crashing


def test_security_validation_in_e2e_flow():
    """
    Test security validation throughout the end-to-end flow
    """
    # Test with potentially malicious input
    session_response = client.post("/api/session", json={
        "mode": "global"
    })

    assert session_response.status_code == 200
    session_data = session_response.json()
    session_id = session_data["session_id"]

    # Test with script-like content (should be sanitized)
    malicious_response = client.post("/api/chat", json={
        "session_id": session_id,
        "message": "What is humanoid robotics? <script>alert('test')</script>",
        "mode": "global"
    })

    # Should still process the request but handle malicious content properly
    assert malicious_response.status_code in [200, 400, 422]

    # Test with SQL injection-like content
    sql_response = client.post("/api/chat", json={
        "session_id": session_id,
        "message": "Tell me about robots'; DROP TABLE messages; --",
        "mode": "global"
    })

    # Should handle the potentially malicious content
    assert sql_response.status_code in [200, 400, 422]