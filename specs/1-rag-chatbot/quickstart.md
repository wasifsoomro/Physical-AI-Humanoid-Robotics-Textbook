# Quickstart Guide: RAG Chatbot for Docusaurus Book

**Feature**: 1-rag-chatbot
**Created**: 2025-12-08

## Overview
This guide provides a quick path to get the RAG Chatbot running with your Docusaurus book. The system allows readers to ask questions about book content and receive accurate answers based on the book text.

## Prerequisites
- Node.js 18+ (for Docusaurus)
- Python 3.9+ (for FastAPI backend)
- OpenAI API key
- Qdrant Cloud account (Free Tier)
- Neon Postgres database
- Git

## Setup Steps

### Step 1: Clone and Initialize
```bash
git clone <your-repo-url>
cd humanoid-robotics-book
npm install
cd backend  # Navigate to backend directory (will be created during implementation)
pip install -r requirements.txt  # Backend dependencies will be defined later
```

### Step 2: Environment Configuration
Create a `.env` file in the backend directory:
```env
OPENAI_API_KEY=your_openai_api_key
QDRANT_API_KEY=your_qdrant_api_key
QDRANT_URL=your_qdrant_cluster_url
NEON_DATABASE_URL=your_neon_database_url
```

### Step 3: Database Setup
1. Set up Neon Postgres database
2. Create required tables using the schema defined in the data model
3. The application will create tables automatically on first run

### Step 4: Content Indexing
Run the content ingestion script to process your book content:
```bash
python scripts/ingest_content.py
```
This will:
- Extract text from Docusaurus docs/
- Chunk content (500 tokens with 75-token overlap)
- Generate embeddings using OpenAI
- Store in Qdrant vector database

### Step 5: Backend Service
Start the FastAPI backend service:
```bash
uvicorn main:app --reload
```

### Step 6: Frontend Integration
The chat widget will be automatically integrated into your Docusaurus pages. The component can be added to any page using:
```jsx
import ChatWidget from './components/ChatWidget';

<ChatWidget />
```

### Step 7: Testing
1. Visit your Docusaurus site
2. Interact with the chat widget
3. Ask questions about book content
4. Verify responses are grounded in book text

## API Endpoints
- `POST /api/chat` - Process user queries
- `POST /api/retrieve` - Retrieve relevant content
- `POST /api/session` - Create new session
- `GET /api/session/{session_id}` - Get session history

## Troubleshooting
- If content isn't being retrieved, verify the ingestion script completed successfully
- If responses are slow, check API key quotas and network connectivity
- If UI doesn't appear, ensure the component is properly imported in Docusaurus

## Next Steps
1. Implement mode toggle (global vs selected-text retrieval)
2. Add conversation history persistence
3. Optimize performance based on usage patterns
4. Deploy to production environment