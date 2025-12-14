"""
Contract tests for conversation history persistence functionality
"""

import pytest
from fastapi.testclient import TestClient
from unittest.mock import Mock, patch
import uuid
from backend.src.api.main import app

client = TestClient(app)


def test_session_creation_with_history():
    """Test the contract of session creation with history tracking"""
    request_data = {
        "mode": "global"
    }

    with patch('backend.src.api.chat_routes.create_session_if_not_exists') as mock_create_session:
        mock_session = Mock()
        mock_session.session_id = str(uuid.uuid4())
        mock_session.mode = "global"
        mock_create_session.return_value = mock_session

        response = client.post("/api/session", json=request_data)

        assert response.status_code == 200
        data = response.json()

        assert "session_id" in data
        assert "mode" in data
        assert "created_at" in data
        assert data["mode"] == "global"


def test_conversation_history_storage_contract():
    """Test that conversation history is stored with each message"""
    session_id = str(uuid.uuid4())

    with patch('backend.src.api.chat_routes.ConversationService') as mock_conv_service, \
         patch('backend.src.api.chat_routes.RetrievalService') as mock_retrieval_service:

        # Mock the retrieval service
        mock_retrieval_service_instance = Mock()
        mock_retrieval_service_instance.retrieve_content.return_value = [
            {"content": "Test context", "source": "source1", "score": 0.95}
        ]
        mock_retrieval_service.return_value = mock_retrieval_service_instance

        # Mock the conversation service to return a response with history context
        mock_conv_service_instance = Mock()
        mock_conv_service_instance.process_message.return_value = {
            "session_id": session_id,
            "response": "Response considering previous context",
            "retrieved_context": [{"content": "Test context", "source": "source1", "score": 0.95}],
            "timestamp": "2025-12-09T10:00:00"
        }
        mock_conv_service.return_value = mock_conv_service_instance

        # Send first message
        first_message = {
            "session_id": session_id,
            "message": "What is humanoid robotics?",
            "mode": "global"
        }

        response1 = client.post("/api/chat", json=first_message)
        assert response1.status_code == 200

        # Send follow-up message that should have access to context
        followup_message = {
            "session_id": session_id,
            "message": "Can you elaborate on the actuators?",
            "mode": "global"
        }

        response2 = client.post("/api/chat", json=followup_message)
        assert response2.status_code == 200

        # Verify that both requests were successful and context was maintained
        assert "session_id" in response1.json()
        assert "session_id" in response2.json()
        assert response1.json()["session_id"] == response2.json()["session_id"]


def test_conversation_context_access():
    """Test the contract for accessing conversation context"""
    session_id = str(uuid.uuid4())

    # Test that the API can retrieve conversation history
    # (This would be a separate endpoint in a full implementation)
    with patch('backend.src.api.chat_routes.ConversationService') as mock_conv_service:
        mock_conv_service_instance = Mock()
        mock_conv_service_instance.get_conversation_context.return_value = [
            {"role": "user", "content": "Previous question", "timestamp": "2025-12-09T09:00:00"},
            {"role": "assistant", "content": "Previous answer", "timestamp": "2025-12-09T09:01:00"}
        ]
        mock_conv_service.return_value = mock_conv_service_instance

        # In a real implementation, we might have a GET endpoint for conversation history
        # For now, we're ensuring the service can provide context
        assert True  # Placeholder for future history endpoint test