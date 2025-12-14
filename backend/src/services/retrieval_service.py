"""
Retrieval service for the RAG Chatbot system
Handles the logic for retrieving relevant content based on user queries
"""

from typing import List, Dict, Any, Optional
from ..ai.cohere_client import CohereClient
from ..vector_store.qdrant_client import QdrantManager
from ..utils.logging_config import get_logger
from ..utils.exceptions import ContentRetrievalException, ValidationError
from ..utils.cache import cache, get_cached_retrieval_results, cache_retrieval_results
from ..utils.validation import sanitize_input, validate_retrieval_mode
import hashlib
from datetime import datetime, timedelta


logger = get_logger(__name__)


class RetrievalService:
    def __init__(self):
        self.cohere_client = CohereClient()
        self.qdrant_manager = QdrantManager()

    def _get_from_cache(self, query: str, mode: str, document_id: Optional[str] = None) -> Optional[List[Dict[str, Any]]]:
        """Get results from cache if available and valid"""
        results = get_cached_retrieval_results(query, mode, document_id)
        if results is not None:
            logger.info(f"Cache hit for query: {query[:50]}...")
        return results

    def _put_in_cache(self, query: str, mode: str, results: List[Dict[str, Any]], document_id: Optional[str] = None):
        """Put results in cache"""
        cache_retrieval_results(query, mode, results, document_id, ttl=300)  # 5 minutes TTL

    def retrieve_content(
        self,
        query: str,
        mode: str = "global",
        document_id: Optional[str] = None,
        limit: int = 5
    ) -> List[Dict[str, Any]]:
        """
        Retrieve relevant content based on the query
        """
        try:
            # Sanitize inputs
            query = sanitize_input(query) if query else query

            # Validate inputs
            if not query or not query.strip():
                raise ValidationError("Query cannot be empty")

            is_valid, error_msg = validate_retrieval_mode(mode)
            if not is_valid:
                raise ValidationError(error_msg)

            if mode == "selected-text" and not document_id:
                logger.warning(f"Selected-text mode requested but no document_id provided for query: {query[:50]}...")

            # Check cache first
            cached_results = self._get_from_cache(query, mode, document_id)
            if cached_results is not None:
                return cached_results

            # Generate embedding for the query
            query_embedding = self.cohere_client.generate_embedding(query)

            # Search for similar content in the vector store
            results = self.qdrant_manager.search_similar_content(
                query_embedding=query_embedding,
                limit=limit,
                document_id=document_id if mode == "selected-text" else None
            )

            logger.info(f"Retrieved {len(results)} results for query: {query[:50]}...")

            # Cache the results
            self._put_in_cache(query, mode, results, document_id)

            return results

        except Exception as e:
            logger.error(f"Error retrieving content: {e}")
            raise ContentRetrievalException(f"Failed to retrieve content: {str(e)}")

    def retrieve_content_with_selected_text(
        self,
        query: str,
        selected_text: str,
        limit: int = 5
    ) -> List[Dict[str, Any]]:
        """
        Retrieve relevant content specifically from the selected text
        In this mode, we create an embedding from the selected text and search for similar content
        """
        try:
            # For selected-text mode, we'll create an embedding of the combined query and selected text
            # to find content similar to both
            combined_query = f"{query} {selected_text}"

            # Generate a document ID based on the selected text content for caching
            document_id = hashlib.md5(selected_text.encode()).hexdigest()

            # Check cache first using the combined query and document_id
            cached_results = self._get_from_cache(combined_query, "selected-text", document_id)
            if cached_results is not None:
                return cached_results

            query_embedding = self.cohere_client.generate_embedding(combined_query)

            # Search in Qdrant, limiting to content related to the selected text
            results = self.qdrant_manager.search_similar_content(
                query_embedding=query_embedding,
                limit=limit,
                document_id=document_id
            )

            logger.info(f"Retrieved {len(results)} results for selected text query: {query[:50]}...")

            # Cache the results
            self._put_in_cache(combined_query, "selected-text", results, document_id)

            return results

        except Exception as e:
            logger.error(f"Error retrieving content with selected text: {e}")
            raise ContentRetrievalException(f"Failed to retrieve content with selected text: {str(e)}")

    def retrieve_content_from_selected_text_only(
        self,
        query: str,
        selected_text: str,
        limit: int = 5
    ) -> List[Dict[str, Any]]:
        """
        Alternative approach: Use the selected text directly as context for the LLM
        This approach doesn't require pre-stored embeddings of the selected text
        """
        try:
            # Generate a document ID based on the selected text content for caching
            document_id = hashlib.md5(selected_text.encode()).hexdigest()

            # Check cache first using the query and document_id
            cached_results = self._get_from_cache(query, "selected-text-direct", document_id)
            if cached_results is not None:
                return cached_results

            # In this approach, we return the selected text itself as the context
            # The actual filtering happens at the LLM level in the conversation service
            result = [{
                "content": selected_text,
                "source": "selected_text",
                "score": 1.0,  # Perfect match since it's the exact selected text
                "document_id": "selected_text",
                "section_title": "Selected Text",
                "chunk_id": "selected_text_chunk"
            }]

            logger.info(f"Using selected text directly as context for query: {query[:50]}...")

            # Cache the results
            self._put_in_cache(query, "selected-text-direct", result, document_id)

            return result

        except Exception as e:
            logger.error(f"Error retrieving content from selected text: {e}")
            raise ContentRetrievalException(f"Failed to retrieve content from selected text: {str(e)}")

    def validate_retrieval_mode(self, mode: str) -> bool:
        """
        Validate that the retrieval mode is supported
        """
        return mode in ["global", "selected-text"]