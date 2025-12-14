#!/usr/bin/env python3
"""
Script to ingest book content into the vector store
"""

import argparse
import sys
import os
from pathlib import Path

# Add the backend src directory to the path so we can import our modules
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from ingestion.content_pipeline import ContentIngestionPipeline
from utils.logging_config import get_logger

logger = get_logger(__name__)

def main():
    parser = argparse.ArgumentParser(description="Ingest book content into the vector store")
    parser.add_argument(
        "--content-dir",
        required=True,
        help="Directory containing the book content (markdown files)"
    )
    parser.add_argument(
        "--document-id",
        default="humanoid-robotics-book",
        help="Document ID for the content (default: humanoid-robotics-book)"
    )
    parser.add_argument(
        "--validate-only",
        action="store_true",
        help="Only validate the content structure, don't ingest"
    )

    args = parser.parse_args()

    # Validate content directory exists
    content_dir = Path(args.content_dir)
    if not content_dir.exists():
        print(f"Error: Content directory does not exist: {args.content_dir}")
        sys.exit(1)

    if not content_dir.is_dir():
        print(f"Error: Path is not a directory: {args.content_dir}")
        sys.exit(1)

    print(f"Starting content processing for directory: {args.content_dir}")
    print(f"Document ID: {args.document_id}")
    print(f"Validate only: {args.validate_only}")
    print("-" * 50)

    try:
        # Initialize the ingestion pipeline
        pipeline = ContentIngestionPipeline()

        if args.validate_only:
            # Validate the content structure
            print("Validating content structure...")
            validation_result = pipeline.validate_content_structure(str(content_dir))
            print("Validation completed successfully!")
            print(f"Valid: {validation_result['valid']}")
            print(f"Total files: {validation_result['total_files']}")
            print(f"Total size: {validation_result['total_size_bytes']} bytes")
            print(f"Average file size: {validation_result['average_file_size']:.2f} bytes")
            print("\nFiles to be processed:")
            for file_path in validation_result['file_paths'][:10]:  # Show first 10 files
                print(f"  - {file_path}")
            if len(validation_result['file_paths']) > 10:
                print(f"  ... and {len(validation_result['file_paths']) - 10} more files")
        else:
            # Perform the ingestion
            print("Starting content ingestion...")
            result = pipeline.ingest_book_content(str(content_dir), args.document_id)

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

    except KeyboardInterrupt:
        print("\nOperation cancelled by user.")
        sys.exit(1)
    except Exception as e:
        print(f"Error during content processing: {str(e)}")
        logger.error(f"Error during content processing: {str(e)}", exc_info=True)
        sys.exit(1)

if __name__ == "__main__":
    main()