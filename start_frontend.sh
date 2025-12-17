#!/bin/bash
# Script to start the Docusaurus frontend development server

echo "Starting the RAG Chatbot Docusaurus frontend development server..."

# Get the directory of this script
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$SCRIPT_DIR"  # Already in project root

echo "Project root: $PROJECT_ROOT"
echo "Changing to project root directory: $PROJECT_ROOT"

cd "$PROJECT_ROOT"

# Install dependencies
echo "Installing Docusaurus dependencies..."
npm install

# Set environment variables for Docusaurus
export REACT_APP_API_BASE_URL=http://localhost:8000

# Run the Docusaurus development server
echo "Starting Docusaurus development server..."
npm start