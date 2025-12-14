# Running the Humanoid Robotics Book RAG Chatbot

This guide explains how to run the complete system with the RAG (Retrieval-Augmented Generation) chatbot integrated into the Docusaurus book site.

## Prerequisites

- Node.js (v18 or higher)
- Python (v3.8 or higher)
- npm
- An OpenAI API key

## Setup

### 1. Environment Configuration

First, set up your environment variables in the `.backend.env` file:

```bash
# OpenAI Configuration
OPENAI_API_KEY=your_openai_api_key_here

# Qdrant Configuration (Vector Database)
QDRANT_URL="your_qdrant_url"
QDRANT_API_KEY="your_qdrant_api_key"

# Neon Postgres Configuration (Session Storage)
DATABASE_URL="your_postgres_connection_string"
```

### 2. Install Dependencies

Install dependencies for all components:

```bash
# Install dependencies for the main Docusaurus site
npm install

# Install backend dependencies
cd backend
pip install -r requirements.txt
cd ..

# Install frontend dependencies
cd frontend
npm install
cd ..
```

## Running the System

The system consists of three main components that need to run simultaneously:

### 1. Start the Backend Server

```bash
./start_backend.sh
```

This starts the FastAPI server on `http://localhost:8000`.

### 2. Start the Docusaurus Site

In a separate terminal:

```bash
./start_docusaurus.sh
```

This starts the Docusaurus site on `http://localhost:3000`.

### 3. Ingest Book Content (One-time setup)

Before using the chatbot, you need to ingest the book content into the vector store:

```bash
./ingest_content.sh
```

This will process all markdown files in the `docs/` directory and store them in the vector database.

## Alternative: Running with Docker

If you prefer to use Docker, you can set up Qdrant and PostgreSQL containers:

### Start Supporting Services

```bash
# Start Qdrant (vector database)
docker run -d --name qdrant -p 6333:6333 qdrant/qdrant

# Start PostgreSQL (for session storage)
docker run -d --name postgres -p 5432:5432 -e POSTGRES_DB=rag_chatbot -e POSTGRES_USER=username -e POSTGRES_PASSWORD=password postgres:13
```

## Testing the System

1. Open your browser to `http://localhost:3000`
2. Navigate to any documentation page
3. The chat widget should appear in the bottom-right corner
4. Try asking questions about the book content
5. Toggle between "Global Search" and "Selected Text" modes
6. View conversation history by clicking "Show History"

## API Endpoints

The backend provides the following API endpoints:

- `POST /api/chat` - Send a message and receive a response
- `POST /api/session` - Create a new chat session
- `GET /api/session/{sessionId}/history` - Get conversation history
- `POST /api/retrieve` - Retrieve content without generating response
- `POST /api/ingest` - Ingest content into the vector store
- `GET /api/health` - Health check endpoint

## Troubleshooting

### Common Issues:

1. **Backend not connecting to frontend**: Make sure the backend is running on port 8000 and check CORS settings.

2. **OpenAI API errors**: Verify your API key is correct and you have sufficient quota.

3. **Content not being found**: Make sure you've ingested the book content using the ingestion script.

4. **Qdrant connection errors**: Check that your Qdrant URL and API key are correct.

### Logs:

- Backend logs are displayed in the terminal where you started the backend
- Frontend logs are displayed in the terminal where you started the frontend
- Docusaurus logs are displayed in the terminal where you started Docusaurus

## Development

For development, you can run each component separately using the provided scripts. The system is designed to allow independent development of each component.

### Hot Reloading

- Backend: uvicorn runs with `--reload` flag for automatic restart on code changes
- Frontend: React development server provides hot reloading
- Docusaurus: Development server provides hot reloading

## Architecture Overview

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Docusaurus    │    │   FastAPI       │    │   Vector DB     │
│   (Frontend)    │◄──►│   (Backend)     │◄──►│   (Qdrant)      │
│   Port 3000     │    │   Port 8000     │    │   Port 6333     │
└─────────────────┘    └─────────────────┘    └─────────────────┘
         │                       │                       │
         ▼                       ▼                       ▼
┌─────────────────────────────────────────────────────────────┐
│                    PostgreSQL (Sessions)                    │
│                    Port 5432 (via connection string)        │
└─────────────────────────────────────────────────────────────┘
```

The system is designed to be scalable and maintainable, with clear separation of concerns between components.