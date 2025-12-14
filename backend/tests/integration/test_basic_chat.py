"""
Integration tests for basic chat functionality
"""

import pytest
from fastapi.testclient import TestClient
from unittest.mock import Mock, patch, MagicMock
import uuid
from backend.src.api.main import app
from backend.src.services.conversation_service import ConversationService
from backend.src.services.retrieval_service import RetrievalService


client = TestClient(app)


def test_basic_chat_flow():
    """Test the complete flow of a basic chat interaction"""
    session_id = str(uuid.uuid4())

    # Mock the services that would be used in the actual implementation
    with patch('backend.src.api.chat_routes.ConversationService') as mock_conv_service, \
         patch('backend.src.api.chat_routes.RetrievalService') as mock_retrieval_service:

        # Mock the retrieval service to return some context
        mock_retrieval_service_instance = Mock()
        mock_retrieval_service_instance.retrieve_content.return_value = [
            {
                "content": "Humanoid robotics is a branch of robotics that focuses on creating robots with human-like characteristics.",
                "source": "module1/introduction.md",
                "score": 0.95
            }
        ]
        mock_retrieval_service.return_value = mock_retrieval_service_instance

        # Mock the conversation service to return a response
        mock_conv_service_instance = Mock()
        mock_conv_service_instance.process_message.return_value = {
            "session_id": session_id,
            "response": "Humanoid robotics is a branch of robotics that focuses on creating robots with human-like characteristics.",
            "retrieved_context": [
                {
                    "content": "Humanoid robotics is a branch of robotics that focuses on creating robots with human-like characteristics.",
                    "source": "module1/introduction.md",
                    "score": 0.95
                }
            ]
        }
        mock_conv_service.return_value = mock_conv_service_instance

        # Make the request
        request_data = {
            "session_id": session_id,
            "message": "What is humanoid robotics?",
            "mode": "global"
        }

        response = client.post("/api/chat", json=request_data)

        # Assertions
        assert response.status_code == 200
        data = response.json()

        assert "session_id" in data
        assert "response" in data
        assert "retrieved_context" in data
        assert data["session_id"] == session_id
        assert len(data["response"]) > 0
        assert len(data["retrieved_context"]) > 0


def test_chat_with_new_session():
    """Test chat functionality with a new session ID"""
    new_session_id = str(uuid.uuid4())

    with patch('backend.src.api.chat_routes.ConversationService') as mock_conv_service, \
         patch('backend.src.api.chat_routes.RetrievalService') as mock_retrieval_service:

        # Mock services
        mock_retrieval_service_instance = Mock()
        mock_retrieval_service_instance.retrieve_content.return_value = [
            {"content": "Test content", "source": "test.md", "score": 0.8}
        ]
        mock_retrieval_service.return_value = mock_retrieval_service_instance

        mock_conv_service_instance = Mock()
        mock_conv_service_instance.process_message.return_value = {
            "session_id": new_session_id,
            "response": "Test response based on content",
            "retrieved_context": [{"content": "Test content", "source": "test.md", "score": 0.8}]
        }
        mock_conv_service.return_value = mock_conv_service_instance

        request_data = {
            "session_id": new_session_id,
            "message": "Tell me about this topic",
            "mode": "global"
        }

        response = client.post("/api/chat", json=request_data)

        assert response.status_code == 200
        data = response.json()
        assert data["session_id"] == new_session_id


def test_chat_with_empty_message():
    """Test behavior when an empty message is sent"""
    session_id = str(uuid.uuid4())

    with patch('backend.src.api.chat_routes.ConversationService') as mock_conv_service, \
         patch('backend.src.api.chat_routes.RetrievalService') as mock_retrieval_service:

        mock_retrieval_service_instance = Mock()
        mock_retrieval_service_instance.retrieve_content.return_value = []
        mock_retrieval_service.return_value = mock_retrieval_service_instance

        mock_conv_service_instance = Mock()
        mock_conv_service_instance.process_message.return_value = {
            "session_id": session_id,
            "response": "Please provide a more specific question about the book content.",
            "retrieved_context": []
        }
        mock_conv_service.return_value = mock_conv_service_instance

        request_data = {
            "session_id": session_id,
            "message": "",  # Empty message
            "mode": "global"
        }

        response = client.post("/api/chat", json=request_data)

        # Should still return a valid response even with empty message
        assert response.status_code == 200


def test_chat_session_persistence():
    """Test that session information is properly handled"""
    session_id = str(uuid.uuid4())

    with patch('backend.src.api.chat_routes.ConversationService') as mock_conv_service, \
         patch('backend.src.api.chat_routes.RetrievalService') as mock_retrieval_service:

        mock_retrieval_service_instance = Mock()
        mock_retrieval_service_instance.retrieve_content.return_value = [
            {"content": "Sample content", "source": "sample.md", "score": 0.9}
        ]
        mock_retrieval_service.return_value = mock_retrieval_service_instance

        mock_conv_service_instance = Mock()
        mock_conv_service_instance.process_message.return_value = {
            "session_id": session_id,
            "response": "Sample response",
            "retrieved_context": [{"content": "Sample content", "source": "sample.md", "score": 0.9}]
        }
        mock_conv_service.return_value = mock_conv_service_instance

        # First message
        response1 = client.post("/api/chat", json={
            "session_id": session_id,
            "message": "First question?",
            "mode": "global"
        })

        # Second message in same session
        response2 = client.post("/api/chat", json={
            "session_id": session_id,
            "message": "Follow up question?",
            "mode": "global"
        })

        # Both should succeed
        assert response1.status_code == 200
        assert response2.status_code == 200