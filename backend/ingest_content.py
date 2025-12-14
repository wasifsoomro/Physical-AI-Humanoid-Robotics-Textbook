#!/usr/bin/env python3
"""
Wrapper script to run the content ingestion
"""

import sys
import os
from pathlib import Path

# Add the backend src directory to the path
sys.path.insert(0, str(Path(__file__).parent / "src"))

from ingestion.content_pipeline import ContentIngestionPipeline
from utils.logging_config import get_logger

logger = get_logger(__name__)

def main():
    content_dir = str(Path(__file__).parent / "../docs")
    document_id = "humanoid-robotics-book"

    print(f"Starting content processing for directory: {content_dir}")
    print(f"Document ID: {document_id}")
    print("-" * 50)

    try:
        # Initialize the ingestion pipeline
        pipeline = ContentIngestionPipeline()

        # Perform the ingestion
        print("Starting content ingestion...")
        result = pipeline.ingest_book_content(content_dir, document_id)

        print("Ingestion completed successfully!")
        print(f"Status: {result['status']}")
        print(f"Total chunks ingested: {result['total_chunks']}")
        print(f"Total documents processed: {result['total_documents']}")
        print(f"Document ID: {result['document_id']}")

        # Show ingestion stats
        stats = pipeline.get_ingestion_stats()
        print(f"\nCurrent vector store stats:")
        print(f"Total chunks in store: {stats['total_chunks_in_store']}")
        print(f"Embedding model: {stats['embedding_model']}")

    except Exception as e:
        print(f"Error during content processing: {str(e)}")
        logger.error(f"Error during content processing: {str(e)}", exc_info=True)
        return 1

    return 0

if __name__ == "__main__":
    sys.exit(main())