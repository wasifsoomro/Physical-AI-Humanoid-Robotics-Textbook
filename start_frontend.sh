#!/bin/bash
# Script to start the frontend development server

echo "Starting the RAG Chatbot frontend development server..."

# Get the directory of this script
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

echo "Project root: $PROJECT_ROOT"
echo "Changing to frontend directory: $PROJECT_ROOT/frontend"

cd "$PROJECT_ROOT/frontend"

# Install dependencies
echo "Installing frontend dependencies..."
npm install

# Set environment variables
export REACT_APP_API_BASE_URL=http://localhost:8002

# Run the React development server
echo "Starting React development server..."
npm start