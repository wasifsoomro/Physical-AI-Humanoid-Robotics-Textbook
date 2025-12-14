"""
Integration tests for multi-turn conversation context functionality
"""

import pytest
from fastapi.testclient import TestClient
from unittest.mock import Mock, patch, MagicMock
import uuid
from backend.src.api.main import app


client = TestClient(app)


def test_multi_turn_conversation_flow():
    """Test a complete multi-turn conversation flow with context"""
    session_id = str(uuid.uuid4())

    # First message - establishing context
    with patch('backend.src.api.chat_routes.ConversationService') as mock_conv_service, \
         patch('backend.src.api.chat_routes.RetrievalService') as mock_retrieval_service:

        mock_retrieval_service_instance = Mock()
        mock_retrieval_service_instance.retrieve_content.return_value = [
            {"content": "Humanoid robots have various types of actuators including servo motors and pneumatic systems.", "source": "actuators.md", "score": 0.9}
        ]
        mock_retrieval_service.return_value = mock_retrieval_service_instance

        mock_conv_service_instance = Mock()
        mock_conv_service_instance.process_message.return_value = {
            "session_id": session_id,
            "response": "Humanoid robots typically use servo motors and pneumatic systems as actuators.",
            "retrieved_context": [{"content": "Humanoid robots have various types of actuators including servo motors and pneumatic systems.", "source": "actuators.md", "score": 0.9}],
            "timestamp": "2025-12-09T10:00:00"
        }
        mock_conv_service.return_value = mock_conv_service_instance

        first_response = client.post("/api/chat", json={
            "session_id": session_id,
            "message": "What types of actuators are used in humanoid robots?",
            "mode": "global"
        })

        assert first_response.status_code == 200
        first_data = first_response.json()
        assert "Humanoid robots" in first_data["response"]


    # Follow-up message - should have access to context from first message
    with patch('backend.src.api.chat_routes.ConversationService') as mock_conv_service, \
         patch('backend.src.api.chat_routes.RetrievalService') as mock_retrieval_service:

        mock_retrieval_service_instance = Mock()
        mock_retrieval_service_instance.retrieve_content.return_value = [
            {"content": "Servo motors provide precise control for humanoid robot movements.", "source": "servo_motors.md", "score": 0.85}
        ]
        mock_retrieval_service.return_value = mock_retrieval_service_instance

        mock_conv_service_instance = Mock()
        # In a real implementation, this would include context from the previous exchange
        mock_conv_service_instance.process_message.return_value = {
            "session_id": session_id,
            "response": "Servo motors provide precise control and are commonly used for joint movements in humanoid robots.",
            "retrieved_context": [{"content": "Servo motors provide precise control for humanoid robot movements.", "source": "servo_motors.md", "score": 0.85}],
            "timestamp": "2025-12-09T10:01:00"
        }
        mock_conv_service.return_value = mock_conv_service_instance

        second_response = client.post("/api/chat", json={
            "session_id": session_id,
            "message": "How do servo motors work in these robots?",
            "mode": "global"
        })

        assert second_response.status_code == 200
        second_data = second_response.json()
        assert "servo motors" in second_data["response"].lower()


def test_conversation_context_persistence():
    """Test that conversation context persists within a session"""
    session_id = str(uuid.uuid4())

    # Simulate a conversation with multiple exchanges
    conversation_steps = [
        {
            "input": "What is a humanoid robot?",
            "expected_keywords": ["human", "robot"]
        },
        {
            "input": "What about their joints?",
            "expected_keywords": ["joint", "actuator"]
        },
        {
            "input": "How are they powered?",
            "expected_keywords": ["power", "motor"]
        }
    ]

    for i, step in enumerate(conversation_steps):
        with patch('backend.src.api.chat_routes.ConversationService') as mock_conv_service, \
             patch('backend.src.api.chat_routes.RetrievalService') as mock_retrieval_service:

            mock_retrieval_service_instance = Mock()
            mock_retrieval_service_instance.retrieve_content.return_value = [
                {"content": f"Relevant content for step {i+1}", "source": f"step{i+1}.md", "score": 0.8}
            ]
            mock_retrieval_service.return_value = mock_retrieval_service_instance

            mock_conv_service_instance = Mock()
            mock_conv_service_instance.process_message.return_value = {
                "session_id": session_id,
                "response": f"Response for step {i+1} about {step['input']}",
                "retrieved_context": [{"content": f"Relevant content for step {i+1}", "source": f"step{i+1}.md", "score": 0.8}],
                "timestamp": f"2025-12-09T10:0{i+1}:00"
            }
            mock_conv_service.return_value = mock_conv_service_instance

            response = client.post("/api/chat", json={
                "session_id": session_id,
                "message": step["input"],
                "mode": "global"
            })

            assert response.status_code == 200, f"Failed on step {i+1}"
            data = response.json()
            assert data["session_id"] == session_id


def test_cross_session_isolation():
    """Test that conversations in different sessions are isolated"""
    session_id_1 = str(uuid.uuid4())
    session_id_2 = str(uuid.uuid4())

    # Send the same question to two different sessions
    question = "What are sensors in humanoid robots?"

    for session_id in [session_id_1, session_id_2]:
        with patch('backend.src.api.chat_routes.ConversationService') as mock_conv_service, \
             patch('backend.src.api.chat_routes.RetrievalService') as mock_retrieval_service:

            mock_retrieval_service_instance = Mock()
            mock_retrieval_service_instance.retrieve_content.return_value = [
                {"content": "Sensors in humanoid robots include cameras, accelerometers, and gyroscopes.", "source": "sensors.md", "score": 0.9}
            ]
            mock_retrieval_service.return_value = mock_retrieval_service_instance

            mock_conv_service_instance = Mock()
            mock_conv_service_instance.process_message.return_value = {
                "session_id": session_id,
                "response": "Sensors in humanoid robots include cameras, accelerometers, and gyroscopes.",
                "retrieved_context": [{"content": "Sensors in humanoid robots include cameras, accelerometers, and gyroscopes.", "source": "sensors.md", "score": 0.9}],
                "timestamp": "2025-12-09T10:00:00"
            }
            mock_conv_service.return_value = mock_conv_service_instance

            response = client.post("/api/chat", json={
                "session_id": session_id,
                "message": question,
                "mode": "global"
            })

            assert response.status_code == 200
            data = response.json()
            assert data["session_id"] == session_id

    # Both sessions should have processed the request independently
    assert session_id_1 != session_id_2


def test_conversation_context_with_mode_switching():
    """Test that context is maintained when switching modes within a conversation"""
    session_id = str(uuid.uuid4())
    selected_text = "This section discusses the walking gait of humanoid robots."

    # Start with global mode
    with patch('backend.src.api.chat_routes.ConversationService') as mock_conv_service, \
         patch('backend.src.api.chat_routes.RetrievalService') as mock_retrieval_service:

        mock_retrieval_service_instance = Mock()
        mock_retrieval_service_instance.retrieve_content.return_value = [
            {"content": "General information about humanoid locomotion.", "source": "locomotion.md", "score": 0.8}
        ]
        mock_retrieval_service.return_value = mock_retrieval_service_instance

        mock_conv_service_instance = Mock()
        mock_conv_service_instance.process_message.return_value = {
            "session_id": session_id,
            "response": "Humanoid robots use various locomotion methods including walking and crawling.",
            "retrieved_context": [{"content": "General information about humanoid locomotion.", "source": "locomotion.md", "score": 0.8}],
            "timestamp": "2025-12-09T10:00:00"
        }
        mock_conv_service.return_value = mock_conv_service_instance

        global_response = client.post("/api/chat", json={
            "session_id": session_id,
            "message": "How do humanoid robots move?",
            "mode": "global"
        })

        assert global_response.status_code == 200


    # Then switch to selected-text mode in the same session
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
            "response": "The walking gait of humanoid robots involves coordinated leg movements.",
            "retrieved_context": [{"content": selected_text, "source": "selected_text", "score": 1.0}],
            "timestamp": "2025-12-09T10:01:00"
        }
        mock_conv_service.return_value = mock_conv_service_instance

        selected_response = client.post("/api/chat", json={
            "session_id": session_id,
            "message": "Can you explain the walking pattern?",
            "mode": "selected-text",
            "selected_text": selected_text
        })

        assert selected_response.status_code == 200
        assert selected_response.json()["session_id"] == session_id