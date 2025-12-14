#!/bin/bash
# Script to start the Docusaurus documentation site

echo "Starting the Docusaurus documentation site..."

# Get the directory of this script
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

echo "Project root: $PROJECT_ROOT"
echo "Changing to project root: $PROJECT_ROOT"

cd "$PROJECT_ROOT"

# Install dependencies
echo "Installing Docusaurus dependencies..."
npm install

# Run the Docusaurus development server
echo "Starting Docusaurus development server..."
npm start