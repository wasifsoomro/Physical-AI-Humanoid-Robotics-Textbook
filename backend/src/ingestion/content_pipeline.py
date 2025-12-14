"""
Content ingestion pipeline for the RAG Chatbot system
Handles processing of book content into vector embeddings
"""

import os
from typing import List, Dict, Any, Optional
from pathlib import Path
import uuid
from ..utils.logging_config import get_logger
from ..utils.exceptions import ValidationError
from ..ai.cohere_client import CohereClient
from ..vector_store.qdrant_client import QdrantManager
from ..utils.text_processing import chunk_text
import logging

logger = get_logger(__name__)

class ContentIngestionPipeline:
    def __init__(self):
        self.cohere_client = CohereClient()
        self.qdrant_manager = QdrantManager()
        self.qdrant_manager.create_collection()  # Ensure collection exists

    def ingest_book_content(self, content_dir: str, document_id: str) -> Dict[str, Any]:
        """Ingest all content from a directory into the vector store"""
        try:
            content_dir_path = Path(content_dir)
            if not content_dir_path.exists():
                raise ValidationError(f"Content directory does not exist: {content_dir}")

            total_chunks = 0
            total_documents = 0

            # Process all markdown files in the directory
            for file_path in content_dir_path.rglob("*.md"):
                logger.info(f"Processing file: {file_path}")
                file_content = self._read_file_content(file_path)

                # Extract section title from filename or content
                section_title = file_path.stem
                page_reference = str(file_path.relative_to(content_dir_path))

                # Chunk the content
                chunks = chunk_text(file_content)

                # Process each chunk
                for i, chunk in enumerate(chunks):
                    chunk_id = str(uuid.uuid4())

                    # Generate embedding for the chunk
                    embedding = self.cohere_client.generate_embedding(chunk)

                    # Store in vector database
                    success = self.qdrant_manager.store_content_chunk(
                        chunk_id=chunk_id,
                        content=chunk,
                        embedding=embedding,
                        document_id=document_id,
                        section_title=section_title,
                        page_reference=page_reference,
                        token_count=len(chunk.split()),  # Approximate token count
                        chunk_index=i
                    )

                    if success:
                        total_chunks += 1
                    else:
                        logger.error(f"Failed to store chunk {chunk_id}")

                total_documents += 1

            result = {
                "status": "success",
                "total_chunks": total_chunks,
                "total_documents": total_documents,
                "document_id": document_id
            }

            logger.info(f"Ingestion completed: {result}")
            return result

        except Exception as e:
            logger.error(f"Error during content ingestion: {e}")
            raise ValidationError(f"Content ingestion failed: {str(e)}")

    def _read_file_content(self, file_path: Path) -> str:
        """Read content from a file"""
        try:
            with open(file_path, 'r', encoding='utf-8') as file:
                content = file.read()
                # Remove frontmatter if present (common in Docusaurus docs)
                if content.startswith('---'):
                    # Find the end of frontmatter
                    parts = content.split('---', 2)
                    if len(parts) > 2:
                        content = parts[2]
                return content
        except Exception as e:
            logger.error(f"Error reading file {file_path}: {e}")
            raise ValidationError(f"Failed to read file {file_path}: {str(e)}")

    def update_document_content(self, content_dir: str, document_id: str) -> Dict[str, Any]:
        """Update content for a specific document in the vector store"""
        try:
            # First, delete existing chunks for this document
            self.qdrant_manager.delete_document_chunks(document_id)
            logger.info(f"Deleted existing chunks for document: {document_id}")

            # Then, ingest the new content
            result = self.ingest_book_content(content_dir, document_id)
            result["update_status"] = "completed"

            logger.info(f"Document update completed: {result}")
            return result
        except Exception as e:
            logger.error(f"Error updating document content: {e}")
            raise ValidationError(f"Document update failed: {str(e)}")

    def validate_content_structure(self, content_dir: str) -> Dict[str, Any]:
        """Validate the structure of the content directory"""
        try:
            content_dir_path = Path(content_dir)
            if not content_dir_path.exists():
                raise ValidationError(f"Content directory does not exist: {content_dir}")

            # Count markdown files
            md_files = list(content_dir_path.rglob("*.md"))
            total_size = sum(f.stat().st_size for f in md_files)

            validation_result = {
                "valid": True,
                "total_files": len(md_files),
                "total_size_bytes": total_size,
                "average_file_size": total_size / len(md_files) if md_files else 0,
                "file_paths": [str(f.relative_to(content_dir_path)) for f in md_files]
            }

            logger.info(f"Content structure validation: {validation_result}")
            return validation_result
        except Exception as e:
            logger.error(f"Error validating content structure: {e}")
            raise ValidationError(f"Content structure validation failed: {str(e)}")

    def get_ingestion_stats(self) -> Dict[str, Any]:
        """Get statistics about the ingestion process"""
        try:
            total_chunks = self.qdrant_manager.get_total_chunks()

            stats = {
                "total_chunks_in_store": total_chunks,
                "vector_size": 1536,  # OpenAI text-embedding-3-large
                "embedding_model": "text-embedding-3-large"
            }

            logger.info(f"Ingestion stats: {stats}")
            return stats
        except Exception as e:
            logger.error(f"Error getting ingestion stats: {e}")
            raise ValidationError(f"Failed to get ingestion stats: {str(e)}")