#!/bin/bash
# Script to start the backend server

echo "Starting the RAG Chatbot backend server..."

# Get the directory of this script
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

echo "Project root: $PROJECT_ROOT"
echo "Changing to backend directory: $PROJECT_ROOT/backend"

cd "$PROJECT_ROOT/backend"

# Check if virtual environment exists, if not create it
if [ ! -d "venv" ]; then
    echo "Creating virtual environment..."
    python -m venv venv
fi

# Activate virtual environment
if [[ "$OSTYPE" == "msys" || "$OSTYPE" == "win32" ]]; then
    source venv/Scripts/activate
else
    source venv/bin/activate
fi

# Install dependencies
echo "Installing dependencies..."
pip install -r requirements.txt

# Set environment variables
export $(grep -v '^#' "$PROJECT_ROOT/.backend.env" | xargs)

# Run the server
echo "Starting FastAPI server..."
uvicorn src.api.main:app --host 0.0.0.0 --port 8000 --reload