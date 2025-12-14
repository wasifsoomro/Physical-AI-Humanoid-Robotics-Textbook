"""
Unit tests for mode-specific retrieval functionality
"""

import pytest
from unittest.mock import Mock, patch
from backend.src.services.retrieval_service import RetrievalService


def test_retrieve_content_global_mode():
    """Test content retrieval in global mode"""
    with patch('backend.src.ai.openai_client.OpenAIClient') as mock_openai, \
         patch('backend.src.vector_store.qdrant_client.QdrantManager') as mock_qdrant:

        mock_openai_instance = Mock()
        test_embedding = [0.1] * 1536
        mock_openai_instance.generate_embedding.return_value = test_embedding
        mock_openai.return_value = mock_openai_instance

        mock_qdrant_instance = Mock()
        test_results = [
            {"content": "Global result 1", "source": "doc1", "score": 0.95}
        ]
        mock_qdrant_instance.search_similar_content.return_value = test_results
        mock_qdrant.return_value = mock_qdrant_instance

        service = RetrievalService()
        results = service.retrieve_content("Test query", mode="global")

        assert len(results) == 1
        assert results[0]["content"] == "Global result 1"

        # Verify that search was called without document_id (for global search)
        mock_qdrant_instance.search_similar_content.assert_called_once_with(
            test_embedding,
            limit=5,
            document_id=None
        )


def test_retrieve_content_selected_text_mode():
    """Test content retrieval in selected-text mode"""
    with patch('backend.src.ai.openai_client.OpenAIClient') as mock_openai, \
         patch('backend.src.vector_store.qdrant_client.QdrantManager') as mock_qdrant:

        mock_openai_instance = Mock()
        # The combined query embedding should be generated
        test_embedding = [0.2] * 1536
        mock_openai_instance.generate_embedding.return_value = test_embedding
        mock_openai.return_value = mock_openai_instance

        mock_qdrant_instance = Mock()
        test_results = [
            {"content": "Selected text result", "source": "selected", "score": 0.88}
        ]
        mock_qdrant_instance.search_similar_content.return_value = test_results
        mock_qdrant.return_value = mock_qdrant_instance

        service = RetrievalService()
        results = service.retrieve_content(
            "Test query",
            mode="selected-text",
            document_id="test_doc_id"
        )

        assert len(results) == 1
        assert results[0]["content"] == "Selected text result"

        # Verify that search was called with document_id (for selected-text search)
        mock_qdrant_instance.search_similar_content.assert_called_once_with(
            test_embedding,
            limit=5,
            document_id="test_doc_id"
        )


def test_retrieve_content_from_selected_text_only():
    """Test the direct selected text retrieval method"""
    service = RetrievalService()

    query = "What does this say about actuators?"
    selected_text = "This section discusses various actuator types used in humanoid robotics..."

    results = service.retrieve_content_from_selected_text_only(
        query=query,
        selected_text=selected_text
    )

    assert len(results) == 1
    result = results[0]
    assert result["content"] == selected_text
    assert result["source"] == "selected_text"
    assert result["score"] == 1.0
    assert result["document_id"] == "selected_text"


def test_retrieve_content_with_selected_text_method():
    """Test the retrieve_content_with_selected_text method"""
    with patch('backend.src.ai.openai_client.OpenAIClient') as mock_openai, \
         patch('backend.src.vector_store.qdrant_client.QdrantManager') as mock_qdrant:

        mock_openai_instance = Mock()
        test_embedding = [0.3] * 1536
        mock_openai_instance.generate_embedding.return_value = test_embedding
        mock_openai.return_value = mock_openai_instance

        mock_qdrant_instance = Mock()
        test_results = [
            {"content": "Similar content", "source": "similar_doc", "score": 0.85}
        ]
        mock_qdrant_instance.search_similar_content.return_value = test_results
        mock_qdrant.return_value = mock_qdrant_instance

        service = RetrievalService()
        query = "What about actuators?"
        selected_text = "Information about actuators in robots..."

        results = service.retrieve_content_with_selected_text(
            query=query,
            selected_text=selected_text
        )

        assert len(results) == 1
        assert results[0]["content"] == "Similar content"

        # Verify that the combined query was created
        mock_openai_instance.generate_embedding.assert_called_once()
        # The call should have been made with the combined query
        call_args = mock_openai_instance.generate_embedding.call_args[0][0]
        assert "What about actuators?" in call_args
        assert "Information about actuators in robots..." in call_args


def test_validate_retrieval_mode():
    """Test the mode validation function"""
    service = RetrievalService()

    # Valid modes
    assert service.validate_retrieval_mode("global") is True
    assert service.validate_retrieval_mode("selected-text") is True

    # Invalid mode
    assert service.validate_retrieval_mode("invalid-mode") is False
    assert service.validate_retrieval_mode("GLOBAL") is False  # Case sensitive
    assert service.validate_retrieval_mode("") is False