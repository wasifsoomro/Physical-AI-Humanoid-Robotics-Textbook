"""
Qdrant vector store client for managing book content embeddings
"""

from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.http.models import Distance, VectorParams
from typing import List, Dict, Any, Optional
import os
from dotenv import load_dotenv
from uuid import UUID
import logging

load_dotenv()

logger = logging.getLogger(__name__)

class QdrantManager:
    def __init__(self):
        # Initialize Qdrant client
        self.host = os.getenv("QDRANT_HOST", "localhost")
        self.port = int(os.getenv("QDRANT_PORT", "6333"))
        self.url = os.getenv("QDRANT_URL")
        self.api_key = os.getenv("QDRANT_API_KEY")

        if self.url:
            # Use cloud instance
            self.client = QdrantClient(
                url=self.url,
                api_key=self.api_key,
                timeout=10
            )
        else:
            # Use local instance
            self.client = QdrantClient(
                host=self.host,
                port=self.port,
                timeout=10
            )

        # Collection name for book content
        self.collection_name = "book_content_chunks"

        # Vector size for Cohere embed-english-v3.0 (1024 dimensions)
        self.vector_size = 1024

    def create_collection(self):
        """Create the collection for storing book content chunks"""
        try:
            # Check if collection already exists
            collections = self.client.get_collections()
            collection_names = [c.name for c in collections.collections]

            if self.collection_name not in collection_names:
                # Create collection with vector configuration
                self.client.create_collection(
                    collection_name=self.collection_name,
                    vectors_config=VectorParams(
                        size=self.vector_size,
                        distance=Distance.COSINE
                    )
                )

                # Create payload indexes for efficient querying
                self.client.create_payload_index(
                    collection_name=self.collection_name,
                    field_name="document_id",
                    field_schema=models.PayloadSchemaType.KEYWORD
                )

                self.client.create_payload_index(
                    collection_name=self.collection_name,
                    field_name="section_title",
                    field_schema=models.PayloadSchemaType.KEYWORD
                )

                logger.info(f"Created Qdrant collection: {self.collection_name}")
            else:
                logger.info(f"Qdrant collection {self.collection_name} already exists")

        except Exception as e:
            logger.error(f"Error creating Qdrant collection: {e}")
            raise

    def store_content_chunk(self, chunk_id: str, content: str, embedding: List[float],
                           document_id: str, section_title: str, page_reference: str,
                           token_count: int, chunk_index: int) -> bool:
        """Store a content chunk with its embedding in Qdrant"""
        try:
            self.client.upsert(
                collection_name=self.collection_name,
                points=[
                    models.PointStruct(
                        id=chunk_id,
                        vector=embedding,
                        payload={
                            "chunk_id": chunk_id,
                            "document_id": document_id,
                            "content": content,
                            "section_title": section_title,
                            "page_reference": page_reference,
                            "token_count": token_count,
                            "chunk_index": chunk_index
                        }
                    )
                ]
            )
            return True
        except Exception as e:
            logger.error(f"Error storing content chunk {chunk_id}: {e}")
            return False

    def search_similar_content(self, query_embedding: List[float], limit: int = 5,
                              document_id: Optional[str] = None) -> List[Dict[str, Any]]:
        """Search for similar content chunks based on embedding"""
        try:
            # Prepare filters
            filters = None
            if document_id:
                # Filter to specific document for selected-text mode
                filters = models.Filter(
                    must=[
                        models.FieldCondition(
                            key="document_id",
                            match=models.MatchValue(value=document_id)
                        )
                    ]
                )

            # Perform search - using the correct method for the Qdrant client version
            results = self.client.search(
                collection_name=self.collection_name,
                query_vector=query_embedding,
                query_filter=filters,
                limit=limit,
                with_payload=True
            )

            # Format results
            formatted_results = []
            for result in results:
                formatted_results.append({
                    "content": result.payload["content"],
                    "source": result.payload["page_reference"],
                    "score": result.score,
                    "document_id": result.payload["document_id"],
                    "section_title": result.payload["section_title"],
                    "chunk_id": result.payload["chunk_id"]
                })

            return formatted_results

        except Exception as e:
            logger.error(f"Error searching similar content: {e}")
            return []

    def delete_document_chunks(self, document_id: str) -> bool:
        """Delete all chunks associated with a specific document"""
        try:
            # Create filter to find points with specific document_id
            filter_condition = models.Filter(
                must=[
                    models.FieldCondition(
                        key="document_id",
                        match=models.MatchValue(value=document_id)
                    )
                ]
            )

            # Delete points matching the filter
            self.client.delete(
                collection_name=self.collection_name,
                points_selector=models.FilterSelector(
                    filter=filter_condition
                )
            )

            return True
        except Exception as e:
            logger.error(f"Error deleting document chunks for {document_id}: {e}")
            return False

    def get_total_chunks(self) -> int:
        """Get the total number of chunks in the collection"""
        try:
            collection_info = self.client.get_collection(self.collection_name)
            return collection_info.points_count
        except Exception as e:
            logger.error(f"Error getting total chunks count: {e}")
            return 0