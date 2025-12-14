"""
Unit tests for content retrieval functionality
"""

import pytest
from unittest.mock import Mock, patch, MagicMock
from backend.src.services.retrieval_service import RetrievalService
from backend.src.ai.openai_client import OpenAIClient
from backend.src.vector_store.qdrant_client import QdrantManager


def test_retrieval_service_initialization():
    """Test that the retrieval service initializes properly"""
    with patch('backend.src.ai.openai_client.OpenAIClient'), \
         patch('backend.src.vector_store.qdrant_client.QdrantManager'):

        service = RetrievalService()
        assert service is not None


def test_retrieve_content_with_query():
    """Test that content retrieval works with a query"""
    with patch('backend.src.ai.openai_client.OpenAIClient') as mock_openai, \
         patch('backend.src.vector_store.qdrant_client.QdrantManager') as mock_qdrant:

        # Mock the OpenAI client to return a test embedding
        mock_openai_instance = Mock()
        test_embedding = [0.1] * 1536  # 1536 dimensions for text-embedding-3-large
        mock_openai_instance.generate_embedding.return_value = test_embedding
        mock_openai.return_value = mock_openai_instance

        # Mock the Qdrant client to return test results
        mock_qdrant_instance = Mock()
        test_results = [
            {
                "content": "Test content for humanoid robotics",
                "source": "module1/introduction.md",
                "score": 0.95,
                "document_id": "doc1",
                "section_title": "Introduction",
                "chunk_id": "chunk1"
            }
        ]
        mock_qdrant_instance.search_similar_content.return_value = test_results
        mock_qdrant.return_value = mock_qdrant_instance

        service = RetrievalService()
        results = service.retrieve_content("What is humanoid robotics?", mode="global")

        # Assertions
        assert len(results) == 1
        assert results[0]["content"] == "Test content for humanoid robotics"
        assert results[0]["source"] == "module1/introduction.md"
        assert results[0]["score"] == 0.95

        # Verify that the embedding was generated
        mock_openai_instance.generate_embedding.assert_called_once_with("What is humanoid robotics?")

        # Verify that the search was called with the embedding
        mock_qdrant_instance.search_similar_content.assert_called_once_with(
            test_embedding,
            limit=5,
            document_id=None
        )


def test_retrieve_content_with_selected_text_mode():
    """Test that content retrieval works in selected-text mode"""
    with patch('backend.src.ai.openai_client.OpenAIClient') as mock_openai, \
         patch('backend.src.vector_store.qdrant_client.QdrantManager') as mock_qdrant:

        mock_openai_instance = Mock()
        test_embedding = [0.1] * 1536
        mock_openai_instance.generate_embedding.return_value = test_embedding
        mock_openai.return_value = mock_openai_instance

        mock_qdrant_instance = Mock()
        test_results = [
            {
                "content": "Selected text content",
                "source": "module2/specific_section.md",
                "score": 0.88,
                "document_id": "doc2",
                "section_title": "Specific Section",
                "chunk_id": "chunk2"
            }
        ]
        mock_qdrant_instance.search_similar_content.return_value = test_results
        mock_qdrant.return_value = mock_qdrant_instance

        service = RetrievalService()
        results = service.retrieve_content(
            "Question about selected text",
            mode="selected-text",
            document_id="doc2"
        )

        # Assertions
        assert len(results) == 1
        assert results[0]["content"] == "Selected text content"

        # Verify that search was called with document_id for selected-text mode
        mock_qdrant_instance.search_similar_content.assert_called_once_with(
            test_embedding,
            limit=5,
            document_id="doc2"
        )


def test_retrieve_content_no_results():
    """Test behavior when no content is found"""
    with patch('backend.src.ai.openai_client.OpenAIClient') as mock_openai, \
         patch('backend.src.vector_store.qdrant_client.QdrantManager') as mock_qdrant:

        mock_openai_instance = Mock()
        test_embedding = [0.1] * 1536
        mock_openai_instance.generate_embedding.return_value = test_embedding
        mock_openai.return_value = mock_openai_instance

        mock_qdrant_instance = Mock()
        mock_qdrant_instance.search_similar_content.return_value = []  # No results
        mock_qdrant.return_value = mock_qdrant_instance

        service = RetrievalService()
        results = service.retrieve_content("Very specific question with no matches", mode="global")

        # Should return empty list
        assert len(results) == 0


def test_retrieve_content_embedding_generation_failure():
    """Test behavior when embedding generation fails"""
    with patch('backend.src.ai.openai_client.OpenAIClient') as mock_openai, \
         patch('backend.src.vector_store.qdrant_client.QdrantManager'):

        mock_openai_instance = Mock()
        mock_openai_instance.generate_embedding.side_effect = Exception("API Error")
        mock_openai.return_value = mock_openai_instance

        service = RetrievalService()

        # Should raise an exception
        with pytest.raises(Exception):
            service.retrieve_content("Question", mode="global")


def test_retrieve_content_search_failure():
    """Test behavior when vector search fails"""
    with patch('backend.src.ai.openai_client.OpenAIClient') as mock_openai, \
         patch('backend.src.vector_store.qdrant_client.QdrantManager') as mock_qdrant:

        mock_openai_instance = Mock()
        test_embedding = [0.1] * 1536
        mock_openai_instance.generate_embedding.return_value = test_embedding
        mock_openai.return_value = mock_openai_instance

        mock_qdrant_instance = Mock()
        mock_qdrant_instance.search_similar_content.side_effect = Exception("Search Error")
        mock_qdrant.return_value = mock_qdrant_instance

        service = RetrievalService()

        # Should raise an exception
        with pytest.raises(Exception):
            service.retrieve_content("Question", mode="global")