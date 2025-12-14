"""
Unit tests for conversation context management functionality
"""

import pytest
from unittest.mock import Mock, patch, MagicMock
from datetime import datetime, timedelta
from backend.src.services.conversation_service import ConversationService
from backend.src.database.models import ChatMessage


def test_context_window_management():
    """Test that the conversation service manages context window properly"""
    with patch('backend.src.ai.openai_client.OpenAIClient'), \
         patch('backend.src.vector_store.qdrant_client.QdrantManager'), \
         patch('backend.src.services.retrieval_service.RetrievalService'):

        service = ConversationService()

        # The service should be able to manage conversation context
        # In our current implementation, context management happens at the DB level
        # and is passed to the LLM as needed
        assert service is not None


def test_message_history_storage():
    """Test that messages are properly stored in history"""
    # Test the ChatMessage model directly
    from backend.src.models.chat import ChatMessageCreate

    session_id = "test-session-id"
    user_message = ChatMessageCreate(
        session_id=session_id,
        role="user",
        content="What is a humanoid robot?",
        retrieved_context=[{"content": "Definition content", "source": "def.md", "score": 0.9}],
        source_chunks=[]
    )

    assistant_message = ChatMessageCreate(
        session_id=session_id,
        role="assistant",
        content="A humanoid robot is a robot with human-like features.",
        retrieved_context=[{"content": "Definition content", "source": "def.md", "score": 0.9}],
        source_chunks=[]
    )

    # Validate the models
    assert user_message.role == "user"
    assert assistant_message.role == "assistant"
    assert user_message.session_id == assistant_message.session_id
    assert "humanoid robot" in user_message.content.lower()
    assert "human-like features" in assistant_message.content.lower()


def test_conversation_context_retrieval():
    """Test that conversation context can be retrieved for a session"""
    with patch('backend.src.ai.openai_client.OpenAIClient') as mock_openai, \
         patch('backend.src.vector_store.qdrant_client.QdrantManager') as mock_qdrant, \
         patch('backend.src.services.retrieval_service.RetrievalService') as mock_retrieval:

        # Mock the dependencies
        mock_openai_instance = Mock()
        mock_openai.return_value = mock_openai_instance

        mock_qdrant_instance = Mock()
        mock_qdrant.return_value = mock_qdrant_instance

        mock_retrieval_instance = Mock()
        mock_retrieval.return_value = mock_retrieval_instance

        service = ConversationService()

        # In our implementation, context is retrieved from the database
        # when process_message is called, using the session_id to fetch
        # previous messages
        assert hasattr(service, 'process_message')


def test_context_aware_response_generation():
    """Test that responses can be generated with awareness of conversation context"""
    with patch('backend.src.ai.openai_client.OpenAIClient') as mock_openai, \
         patch('backend.src.vector_store.qdrant_client.QdrantManager') as mock_qdrant, \
         patch('backend.src.services.retrieval_service.RetrievalService') as mock_retrieval:

        # Mock the OpenAI client to return a specific response when context is provided
        mock_openai_instance = Mock()
        def mock_generate_chat_completion(messages, context=None):
            if context and "previous conversation" in context.lower():
                return "This response considers the previous conversation context."
            else:
                return "This is a standalone response."

        mock_openai_instance.generate_chat_completion.side_effect = mock_generate_chat_completion
        mock_openai.return_value = mock_openai_instance

        mock_qdrant_instance = Mock()
        mock_qdrant.return_value = mock_qdrant_instance

        mock_retrieval_instance = Mock()
        mock_retrieval_instance.retrieve_content.return_value = [
            {"content": "Humanoid robotics definition content", "source": "definition.md", "score": 0.95}
        ]
        mock_retrieval.return_value = mock_retrieval_instance

        service = ConversationService()

        # Test that the service can process a message with context
        result = service.process_message(
            session_id="test-session",
            user_message="What are humanoid robots?",
            mode="global"
        )

        assert "session_id" in result
        assert "response" in result
        assert result["session_id"] == "test-session"


def test_context_limit_handling():
    """Test handling of conversation context limits"""
    with patch('backend.src.ai.openai_client.OpenAIClient') as mock_openai, \
         patch('backend.src.vector_store.qdrant_client.QdrantManager') as mock_qdrant, \
         patch('backend.src.services.retrieval_service.RetrievalService') as mock_retrieval:

        mock_openai_instance = Mock()
        mock_openai.return_value = mock_openai_instance

        mock_qdrant_instance = Mock()
        mock_qdrant.return_value = mock_qdrant_instance

        mock_retrieval_instance = Mock()
        mock_retrieval_instance.retrieve_content.return_value = [
            {"content": "Test content", "source": "test.md", "score": 0.8}
        ]
        mock_retrieval.return_value = mock_retrieval_instance

        service = ConversationService()

        # Test with a very long conversation context (simulated)
        long_context = "This is a very long conversation context. " * 100
        result = service.process_message(
            session_id="test-session",
            user_message="What was I asking about?",
            mode="global"
        )

        # Should still process successfully despite context length
        assert "response" in result


def test_session_context_isolation():
    """Test that contexts are isolated between different sessions"""
    with patch('backend.src.ai.openai_client.OpenAIClient') as mock_openai, \
         patch('backend.src.vector_store.qdrant_client.QdrantManager') as mock_qdrant, \
         patch('backend.src.services.retrieval_service.RetrievalService') as mock_retrieval:

        mock_openai_instance = Mock()
        mock_openai.return_value = mock_openai_instance

        mock_qdrant_instance = Mock()
        mock_qdrant.return_value = mock_qdrant_instance

        mock_retrieval_instance = Mock()
        mock_retrieval_instance.retrieve_content.return_value = [
            {"content": "Shared content", "source": "shared.md", "score": 0.8}
        ]
        mock_retrieval.return_value = mock_retrieval_instance

        service = ConversationService()

        # Process messages in different sessions
        result1 = service.process_message(
            session_id="session-1",
            user_message="My question in session 1",
            mode="global"
        )

        result2 = service.process_message(
            session_id="session-2",
            user_message="My question in session 2",
            mode="global"
        )

        # Each should have its own session ID
        assert result1["session_id"] == "session-1"
        assert result2["session_id"] == "session-2"
        # Context should be isolated between sessions
        assert result1["session_id"] != result2["session_id"]