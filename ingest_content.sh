#!/bin/bash
# Script to ingest book content into the RAG system

echo "Ingesting book content into the RAG system..."

# Get the directory of this script
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

echo "Project root: $PROJECT_ROOT"
echo "Content directory: $PROJECT_ROOT/docs"

# Run the ingestion script
cd "$PROJECT_ROOT/backend" && python -m scripts.ingest_book_content --content-dir "$PROJECT_ROOT/docs" --document-id "humanoid-robotics-book"

if [ $? -eq 0 ]; then
    echo "✅ Content ingestion completed successfully!"
else
    echo "❌ Content ingestion failed!"
    exit 1
fi