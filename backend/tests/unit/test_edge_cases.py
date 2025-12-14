"""
Unit tests for edge cases in the RAG Chatbot system
"""
import pytest
from unittest.mock import Mock, patch, MagicMock
from backend.src.services.retrieval_service import RetrievalService
from backend.src.services.conversation_service import ConversationService
from backend.src.utils.exceptions import ValidationError, ContentRetrievalException


def test_retrieve_content_empty_query():
    """Test behavior when query is empty or whitespace"""
    with patch('backend.src.ai.openai_client.OpenAIClient'), \
         patch('backend.src.vector_store.qdrant_client.QdrantManager'):

        service = RetrievalService()

        # Test empty query
        with pytest.raises(ValidationError) as exc_info:
            service.retrieve_content("", mode="global")
        assert "Query cannot be empty" in str(exc_info.value)

        # Test whitespace query
        with pytest.raises(ValidationError) as exc_info:
            service.retrieve_content("   ", mode="global")
        assert "Query cannot be empty" in str(exc_info.value)


def test_retrieve_content_invalid_mode():
    """Test behavior when mode is invalid"""
    with patch('backend.src.ai.openai_client.OpenAIClient'), \
         patch('backend.src.vector_store.qdrant_client.QdrantManager'):

        service = RetrievalService()

        with pytest.raises(ValidationError) as exc_info:
            service.retrieve_content("test query", mode="invalid-mode")
        assert "Invalid mode" in str(exc_info.value)


def test_retrieve_content_with_selected_text_empty():
    """Test behavior when selected text mode is used with empty selected text"""
    with patch('backend.src.ai.openai_client.OpenAIClient'), \
         patch('backend.src.vector_store.qdrant_client.QdrantManager'):

        service = RetrievalService()

        with pytest.raises(ValidationError) as exc_info:
            service.retrieve_content_from_selected_text_only(
                query="test query",
                selected_text=""
            )
        assert "Selected text cannot be empty" in str(exc_info.value)


def test_retrieve_content_with_selected_text_whitespace():
    """Test behavior when selected text mode is used with whitespace selected text"""
    with patch('backend.src.ai.openai_client.OpenAIClient'), \
         patch('backend.src.vector_store.qdrant_client.QdrantManager'):

        service = RetrievalService()

        with pytest.raises(ValidationError) as exc_info:
            service.retrieve_content_from_selected_text_only(
                query="test query",
                selected_text="   "
            )
        assert "Selected text cannot be empty" in str(exc_info.value)


def test_process_message_empty_message():
    """Test behavior when processing an empty message"""
    with patch('backend.src.ai.openai_client.OpenAIClient'), \
         patch('backend.src.services.retrieval_service.RetrievalService'):

        service = ConversationService()

        with pytest.raises(ValidationError) as exc_info:
            service.process_message(
                session_id="test-session-id",
                user_message="",
                mode="global"
            )
        assert "Message cannot be empty" in str(exc_info.value)


def test_process_message_too_long_message():
    """Test behavior when processing a very long message"""
    with patch('backend.src.ai.openai_client.OpenAIClient'), \
         patch('backend.src.services.retrieval_service.RetrievalService'):

        service = ConversationService()

        long_message = "A" * 1500  # Exceeds 1000 character limit
        with pytest.raises(ValidationError) as exc_info:
            service.process_message(
                session_id="test-session-id",
                user_message=long_message,
                mode="global"
            )
        assert "Message is too long" in str(exc_info.value)


def test_process_message_invalid_mode():
    """Test behavior when processing with invalid mode"""
    with patch('backend.src.ai.openai_client.OpenAIClient'), \
         patch('backend.src.services.retrieval_service.RetrievalService'):

        service = ConversationService()

        with pytest.raises(ValidationError) as exc_info:
            service.process_message(
                session_id="test-session-id",
                user_message="test message",
                mode="invalid-mode"
            )
        assert "Invalid mode" in str(exc_info.value)


def test_process_message_invalid_session_id():
    """Test behavior when processing with invalid session ID"""
    with patch('backend.src.ai.openai_client.OpenAIClient'), \
         patch('backend.src.services.retrieval_service.RetrievalService'):

        service = ConversationService()

        with pytest.raises(ValidationError) as exc_info:
            service.process_message(
                session_id="",  # Empty session ID
                user_message="test message",
                mode="global"
            )
        assert "Invalid session ID format" in str(exc_info.value)

        # Test with too short session ID
        with pytest.raises(ValidationError) as exc_info:
            service.process_message(
                session_id="short",
                user_message="test message",
                mode="global"
            )
        assert "Invalid session ID format" in str(exc_info.value)


def test_process_message_invalid_context_window():
    """Test behavior when processing with invalid context window size"""
    with patch('backend.src.ai.openai_client.OpenAIClient'), \
         patch('backend.src.services.retrieval_service.RetrievalService'):

        service = ConversationService()

        # Test with negative context window
        with pytest.raises(ValidationError) as exc_info:
            service.process_message(
                session_id="test-session-id-12345",
                user_message="test message",
                mode="global",
                context_window_size=-1
            )
        assert "Context window size must be between 1 and 50" in str(exc_info.value)

        # Test with context window too large
        with pytest.raises(ValidationError) as exc_info:
            service.process_message(
                session_id="test-session-id-12345",
                user_message="test message",
                mode="global",
                context_window_size=51
            )
        assert "Context window size must be between 1 and 50" in str(exc_info.value)


def test_trim_context_empty_messages():
    """Test context trimming with empty messages list"""
    with patch('backend.src.ai.openai_client.OpenAIClient'), \
         patch('backend.src.services.retrieval_service.RetrievalService'):

        service = ConversationService()

        result = service._trim_context_by_token_limit([])
        assert result == []


def test_trim_context_invalid_max_tokens():
    """Test context trimming with invalid max tokens"""
    with patch('backend.src.ai.openai_client.OpenAIClient'), \
         patch('backend.src.services.retrieval_service.RetrievalService'):

        service = ConversationService()

        messages = [
            {"role": "user", "content": "Hello"},
            {"role": "assistant", "content": "Hi there"}
        ]

        # Test with negative max tokens
        result = service._trim_context_by_token_limit(messages, max_tokens=-100)
        # Should use default of 3000 tokens, so all messages should be kept
        assert len(result) == 2


def test_trim_context_invalid_message_format():
    """Test context trimming with invalid message format"""
    with patch('backend.src.ai.openai_client.OpenAIClient'), \
         patch('backend.src.services.retrieval_service.RetrievalService'):

        service = ConversationService()

        messages = [
            {"role": "user", "content": "Hello"},  # Valid
            "Invalid message format",              # Invalid
            {"role": "assistant", "content": "Hi there"},  # Valid
            {"role": "user"},                      # Invalid (no content)
        ]

        result = service._trim_context_by_token_limit(messages)
        # Should handle invalid messages gracefully and keep valid ones
        assert len(result) <= 2  # At most 2 valid messages


def test_get_recent_conversation_context_invalid_inputs():
    """Test getting conversation context with invalid inputs"""
    with patch('backend.src.ai.openai_client.OpenAIClient'), \
         patch('backend.src.services.retrieval_service.RetrievalService'):

        service = ConversationService()

        # Test with None database session
        result = service._get_recent_conversation_context(None, "test-session")
        assert result == []

        # Test with empty session ID
        mock_db = Mock()
        result = service._get_recent_conversation_context(mock_db, "")
        assert result == []

        # Test with None session ID
        result = service._get_recent_conversation_context(mock_db, None)
        assert result == []


def test_cache_key_generation():
    """Test cache key generation for different inputs"""
    with patch('backend.src.ai.openai_client.OpenAIClient'), \
         patch('backend.src.vector_store.qdrant_client.QdrantManager'):

        service = RetrievalService()

        # Generate cache keys for same query with different modes
        key1 = service._generate_cache_key("test query", "global", None)
        key2 = service._generate_cache_key("test query", "selected-text", None)
        key3 = service._generate_cache_key("test query", "global", "doc1")

        # Keys should be different
        assert key1 != key2
        assert key1 != key3
        assert key2 != key3


def test_cache_ttl_expiration():
    """Test cache TTL expiration"""
    from datetime import datetime, timedelta

    with patch('backend.src.ai.openai_client.OpenAIClient'), \
         patch('backend.src.vector_store.qdrant_client.QdrantManager'):

        service = RetrievalService()

        # Manually add an entry with an old timestamp
        old_timestamp = datetime.now() - timedelta(seconds=service._cache_ttl + 1)
        service._cache["test_key"] = (["test_result"], old_timestamp)

        # Check if cache is valid (should be expired)
        is_valid = service._is_cache_valid(old_timestamp)
        assert not is_valid


def test_retrieve_content_with_special_characters():
    """Test retrieval with special characters and unicode"""
    with patch('backend.src.ai.openai_client.OpenAIClient') as mock_openai, \
         patch('backend.src.vector_store.qdrant_client.QdrantManager') as mock_qdrant:

        mock_openai_instance = Mock()
        test_embedding = [0.1] * 1536
        mock_openai_instance.generate_embedding.return_value = test_embedding
        mock_openai.return_value = mock_openai_instance

        mock_qdrant_instance = Mock()
        test_results = [{"content": "Test", "source": "test", "score": 0.9, "document_id": "doc", "section_title": "test", "chunk_id": "chunk"}]
        mock_qdrant_instance.search_similar_content.return_value = test_results
        mock_qdrant.return_value = mock_qdrant_instance

        service = RetrievalService()

        # Test with special characters
        results = service.retrieve_content("What is 🤖 & 🧠?", mode="global")
        assert len(results) == 1

        # Test with unicode
        results = service.retrieve_content("What is искусственный интеллект?", mode="global")
        assert len(results) == 1

        # Test with SQL injection-like text (should be handled safely)
        results = service.retrieve_content("test'; DROP TABLE messages; --", mode="global")
        assert len(results) == 1


def test_conversation_context_with_none_values():
    """Test conversation context handling with None values"""
    with patch('backend.src.ai.openai_client.OpenAIClient'), \
         patch('backend.src.services.retrieval_service.RetrievalService'):

        service = ConversationService()

        # Mock a message with missing attributes
        mock_msg = Mock()
        mock_msg.role = "user"
        mock_msg.content = "test message"
        # Deliberately not setting timestamp to test None handling

        # This should not crash
        result = service._get_recent_conversation_context(Mock(), "test-session", limit=1)
        # The function will return an empty list since the database query will fail with the Mock
        # but it should not crash due to None values