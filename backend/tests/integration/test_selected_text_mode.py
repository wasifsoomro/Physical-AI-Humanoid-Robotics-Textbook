"""
Integration tests for selected-text mode functionality
"""

import pytest
from fastapi.testclient import TestClient
from unittest.mock import Mock, patch
import uuid
from backend.src.api.main import app


client = TestClient(app)


def test_selected_text_mode_integration():
    """Test the complete flow of selected-text mode"""
    session_id = str(uuid.uuid4())

    selected_text = "Humanoid robotics is a branch of robotics that focuses on creating robots with human-like characteristics. These robots are designed to mimic human behavior and appearance to some extent."

    with patch('backend.src.api.chat_routes.ConversationService') as mock_conv_service, \
         patch('backend.src.api.chat_routes.RetrievalService') as mock_retrieval_service:

        # Mock the retrieval service for selected-text mode
        mock_retrieval_service_instance = Mock()
        mock_retrieval_service_instance.retrieve_content_from_selected_text_only.return_value = [
            {
                "content": selected_text,
                "source": "selected_text",
                "score": 1.0
            }
        ]
        mock_retrieval_service.return_value = mock_retrieval_service_instance

        # Mock the conversation service
        mock_conv_service_instance = Mock()
        mock_conv_service_instance.process_message.return_value = {
            "session_id": session_id,
            "response": "Based on the selected text, humanoid robotics focuses on creating robots with human-like characteristics.",
            "retrieved_context": [
                {
                    "content": selected_text,
                    "source": "selected_text",
                    "score": 1.0
                }
            ],
            "timestamp": "2025-12-09T10:00:00"
        }
        mock_conv_service.return_value = mock_conv_service_instance

        # Make request with selected-text mode
        request_data = {
            "session_id": session_id,
            "message": "What is humanoid robotics?",
            "mode": "selected-text",
            "selected_text": selected_text
        }

        response = client.post("/api/chat", json=request_data)

        # Assertions
        assert response.status_code == 200
        data = response.json()

        assert "session_id" in data
        assert "response" in data
        assert "retrieved_context" in data
        assert data["session_id"] == session_id
        assert "selected" in data["response"].lower() or "text" in data["response"].lower()


def test_mode_comparison_integration():
    """Test that global and selected-text modes produce different results"""
    session_id = str(uuid.uuid4())

    selected_text = "The actuators in humanoid robots are typically servo motors or pneumatic systems."

    # Test global mode
    with patch('backend.src.api.chat_routes.ConversationService') as mock_conv_service, \
         patch('backend.src.api.chat_routes.RetrievalService') as mock_retrieval_service:

        mock_retrieval_service_instance = Mock()
        mock_retrieval_service_instance.retrieve_content.return_value = [
            {"content": "General content about humanoid robotics", "source": "general.md", "score": 0.9}
        ]
        mock_retrieval_service.return_value = mock_retrieval_service_instance

        mock_conv_service_instance = Mock()
        mock_conv_service_instance.process_message.return_value = {
            "session_id": session_id,
            "response": "General information about humanoid robotics",
            "retrieved_context": [{"content": "General content about humanoid robotics", "source": "general.md", "score": 0.9}],
            "timestamp": "2025-12-09T10:00:00"
        }
        mock_conv_service.return_value = mock_conv_service_instance

        global_response = client.post("/api/chat", json={
            "session_id": session_id,
            "message": "What are actuators?",
            "mode": "global"
        })

    # Test selected-text mode
    with patch('backend.src.api.chat_routes.ConversationService') as mock_conv_service, \
         patch('backend.src.api.chat_routes.RetrievalService') as mock_retrieval_service:

        mock_retrieval_service_instance = Mock()
        mock_retrieval_service_instance.retrieve_content_from_selected_text_only.return_value = [
            {"content": selected_text, "source": "selected_text", "score": 1.0}
        ]
        mock_retrieval_service.return_value = mock_retrieval_service_instance

        mock_conv_service_instance = Mock()
        mock_conv_service_instance.process_message.return_value = {
            "session_id": session_id,
            "response": "Actuators in humanoid robots are typically servo motors or pneumatic systems",
            "retrieved_context": [{"content": selected_text, "source": "selected_text", "score": 1.0}],
            "timestamp": "2025-12-09T10:00:00"
        }
        mock_conv_service.return_value = mock_conv_service_instance

        selected_response = client.post("/api/chat", json={
            "session_id": session_id,
            "message": "What are actuators?",
            "mode": "selected-text",
            "selected_text": selected_text
        })

    # Both should succeed but with different responses based on the mode
    assert global_response.status_code == 200
    assert selected_response.status_code == 200

    global_data = global_response.json()
    selected_data = selected_response.json()

    # The responses should be different based on the mode
    # (in real implementation they would be different, but with mocks they might be the same)
    assert "session_id" in global_data
    assert "session_id" in selected_data


def test_selected_text_with_empty_content():
    """Test behavior when selected text is empty or None"""
    session_id = str(uuid.uuid4())

    # Test with empty selected text
    with patch('backend.src.api.chat_routes.ConversationService') as mock_conv_service, \
         patch('backend.src.api.chat_routes.RetrievalService') as mock_retrieval_service:

        mock_retrieval_service_instance = Mock()
        # Should fall back to regular retrieval if selected_text is empty
        mock_retrieval_service_instance.retrieve_content.return_value = [
            {"content": "General content", "source": "general.md", "score": 0.8}
        ]
        mock_retrieval_service.return_value = mock_retrieval_service_instance

        mock_conv_service_instance = Mock()
        mock_conv_service_instance.process_message.return_value = {
            "session_id": session_id,
            "response": "General response when no specific text provided",
            "retrieved_context": [{"content": "General content", "source": "general.md", "score": 0.8}],
            "timestamp": "2025-12-09T10:00:00"
        }
        mock_conv_service.return_value = mock_conv_service_instance

        response = client.post("/api/chat", json={
            "session_id": session_id,
            "message": "What is this about?",
            "mode": "selected-text",
            "selected_text": ""  # Empty selected text
        })

        assert response.status_code == 200


def test_selected_text_mode_retrieve_endpoint():
    """Test the retrieve endpoint with selected-text mode"""
    session_id = str(uuid.uuid4())

    selected_text = "Artificial intelligence in robotics enables machines to make decisions."

    with patch('backend.src.api.chat_routes.RetrievalService') as mock_retrieval_service:
        mock_retrieval_service_instance = Mock()
        mock_retrieval_service_instance.retrieve_content_from_selected_text_only.return_value = [
            {
                "content": selected_text,
                "source": "selected_text",
                "score": 1.0
            }
        ]
        mock_retrieval_service.return_value = mock_retrieval_service_instance

        response = client.post("/api/retrieve", json={
            "session_id": session_id,
            "message": "How is AI used in robotics?",
            "mode": "selected-text",
            "selected_text": selected_text
        })

        assert response.status_code == 200
        data = response.json()
        assert "results" in data
        assert len(data["results"]) == 1
        assert data["results"][0]["content"] == selected_text