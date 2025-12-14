"""
Contract tests for retrieval mode switching functionality
"""

import pytest
from fastapi.testclient import TestClient
from unittest.mock import Mock, patch
import uuid
from backend.src.api.main import app

client = TestClient(app)


def test_retrieve_endpoint_contract():
    """Test the contract of the POST /api/retrieve endpoint with different modes"""
    session_id = str(uuid.uuid4())

    # Test global mode
    global_request = {
        "session_id": session_id,
        "message": "What is humanoid robotics?",
        "mode": "global"
    }

    with patch('backend.src.api.chat_routes.RetrievalService') as mock_retrieval_service:
        mock_retrieval_service_instance = Mock()
        mock_retrieval_service_instance.retrieve_content.return_value = [
            {"content": "Global search result", "source": "source1", "score": 0.95}
        ]
        mock_retrieval_service.return_value = mock_retrieval_service_instance

        response = client.post("/api/retrieve", json=global_request)

        assert response.status_code == 200
        data = response.json()
        assert "results" in data
        assert len(data["results"]) == 1


def test_retrieve_endpoint_selected_text_mode():
    """Test the contract of the POST /api/retrieve endpoint with selected-text mode"""
    session_id = str(uuid.uuid4())

    selected_text_request = {
        "session_id": session_id,
        "message": "What does this section say about actuators?",
        "mode": "selected-text",
        "selected_text": "This section discusses various actuator types used in humanoid robotics..."
    }

    with patch('backend.src.api.chat_routes.RetrievalService') as mock_retrieval_service:
        mock_retrieval_service_instance = Mock()
        mock_retrieval_service_instance.retrieve_content_from_selected_text_only.return_value = [
            {"content": "Selected text result", "source": "selected_text", "score": 1.0}
        ]
        mock_retrieval_service.return_value = mock_retrieval_service_instance

        response = client.post("/api/retrieve", json=selected_text_request)

        assert response.status_code == 200
        data = response.json()
        assert "results" in data
        assert len(data["results"]) == 1


def test_chat_endpoint_mode_switching():
    """Test that the chat endpoint properly handles different modes"""
    session_id = str(uuid.uuid4())

    # Test global mode
    global_request = {
        "session_id": session_id,
        "message": "What is humanoid robotics?",
        "mode": "global"
    }

    with patch('backend.src.api.chat_routes.ConversationService') as mock_conv_service, \
         patch('backend.src.api.chat_routes.RetrievalService') as mock_retrieval_service:

        mock_retrieval_service_instance = Mock()
        mock_retrieval_service_instance.retrieve_content.return_value = [
            {"content": "Global search result", "source": "source1", "score": 0.95}
        ]
        mock_retrieval_service.return_value = mock_retrieval_service_instance

        mock_conv_service_instance = Mock()
        mock_conv_service_instance.process_message.return_value = {
            "session_id": session_id,
            "response": "Global mode response",
            "retrieved_context": [{"content": "Global search result", "source": "source1", "score": 0.95}],
            "timestamp": "2025-12-09T10:00:00"
        }
        mock_conv_service.return_value = mock_conv_service_instance

        response = client.post("/api/chat", json=global_request)
        assert response.status_code == 200


    # Test selected-text mode
    selected_text_request = {
        "session_id": session_id,
        "message": "What does this say about actuators?",
        "mode": "selected-text",
        "selected_text": "This section discusses various actuator types used in humanoid robotics..."
    }

    with patch('backend.src.api.chat_routes.ConversationService') as mock_conv_service, \
         patch('backend.src.api.chat_routes.RetrievalService') as mock_retrieval_service:

        mock_retrieval_service_instance = Mock()
        mock_retrieval_service_instance.retrieve_content_from_selected_text_only.return_value = [
            {"content": "Selected text result", "source": "selected_text", "score": 1.0}
        ]
        mock_retrieval_service.return_value = mock_retrieval_service_instance

        mock_conv_service_instance = Mock()
        mock_conv_service_instance.process_message.return_value = {
            "session_id": session_id,
            "response": "Selected text mode response",
            "retrieved_context": [{"content": "Selected text result", "source": "selected_text", "score": 1.0}],
            "timestamp": "2025-12-09T10:00:00"
        }
        mock_conv_service.return_value = mock_conv_service_instance

        response = client.post("/api/chat", json=selected_text_request)
        assert response.status_code == 200


def test_invalid_mode_handling():
    """Test that invalid modes are handled properly"""
    session_id = str(uuid.uuid4())

    invalid_request = {
        "session_id": session_id,
        "message": "What is humanoid robotics?",
        "mode": "invalid-mode"  # Invalid mode
    }

    response = client.post("/api/chat", json=invalid_request)
    assert response.status_code in [400, 422]  # Should return error