"""
Contract tests for the chat API endpoint
"""

import pytest
from fastapi.testclient import TestClient
from unittest.mock import Mock, patch
import uuid
from backend.src.api.main import app

client = TestClient(app)


def test_chat_endpoint_contract():
    """Test the contract of the POST /api/chat endpoint"""
    # Mock data
    session_id = str(uuid.uuid4())
    test_request = {
        "session_id": session_id,
        "message": "What is humanoid robotics?",
        "mode": "global"
    }

    # Since we're doing contract testing, we'll mock the dependencies
    with patch('backend.src.api.chat_routes.ChatService') as mock_chat_service:
        # Mock the chat service response
        mock_response = {
            "session_id": session_id,
            "response": "Humanoid robotics is a branch of robotics that focuses on creating robots with human-like characteristics.",
            "retrieved_context": [{"content": "Sample context", "source": "source1", "score": 0.95}],
            "timestamp": "2025-12-09T10:00:00"
        }

        mock_chat_service_instance = Mock()
        mock_chat_service_instance.process_message.return_value = mock_response
        mock_chat_service.return_value = mock_chat_service_instance

        response = client.post("/api/chat", json=test_request)

        # Verify response structure and status
        assert response.status_code == 200
        data = response.json()

        # Check required fields in response
        assert "session_id" in data
        assert "response" in data
        assert "timestamp" in data

        # Verify types
        assert isinstance(data["session_id"], str)
        assert isinstance(data["response"], str)
        assert isinstance(data["timestamp"], str)  # ISO format string

        # Check that session_id matches
        assert data["session_id"] == session_id


def test_chat_endpoint_missing_fields():
    """Test that endpoint handles missing required fields appropriately"""
    incomplete_request = {
        "session_id": str(uuid.uuid4())
        # Missing 'message' field
    }

    response = client.post("/api/chat", json=incomplete_request)
    assert response.status_code in [400, 422]  # Either validation error or bad request


def test_chat_endpoint_invalid_session_id():
    """Test that endpoint handles invalid session ID appropriately"""
    invalid_request = {
        "session_id": "invalid-session-id",
        "message": "What is humanoid robotics?",
        "mode": "global"
    }

    response = client.post("/api/chat", json=invalid_request)
    # Should either accept it (as it will be created) or return an error
    assert response.status_code in [200, 400, 422]


def test_chat_endpoint_different_modes():
    """Test that endpoint accepts different modes"""
    session_id = str(uuid.uuid4())

    for mode in ["global", "selected-text"]:
        test_request = {
            "session_id": session_id,
            "message": "What is humanoid robotics?",
            "mode": mode
        }

        with patch('backend.src.api.chat_routes.ChatService') as mock_chat_service:
            mock_response = {
                "session_id": session_id,
                "response": "Test response",
                "timestamp": "2025-12-09T10:00:00"
            }

            mock_chat_service_instance = Mock()
            mock_chat_service_instance.process_message.return_value = mock_response
            mock_chat_service.return_value = mock_chat_service_instance

            response = client.post("/api/chat", json=test_request)
            assert response.status_code == 200