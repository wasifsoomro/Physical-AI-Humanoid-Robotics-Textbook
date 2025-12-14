#!/bin/bash
# Script to start the backend server with Cohere integration

echo "Starting the RAG Chatbot backend server with Cohere..."

# Get the directory of this script
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

echo "Project root: $PROJECT_ROOT"
echo "Changing to backend directory: $PROJECT_ROOT/backend"

cd "$PROJECT_ROOT/backend"

# Set environment variables for Cohere
export $(grep -v '^#' "$PROJECT_ROOT/.backend.env" | xargs)

# Start the FastAPI server
echo "Starting FastAPI server on http://localhost:8000..."
python -m uvicorn src.api.main:app --host 0.0.0.0 --port 8000 --reload