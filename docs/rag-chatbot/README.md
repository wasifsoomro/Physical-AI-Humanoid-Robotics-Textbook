# RAG Chatbot for Humanoid Robotics Book

## Overview

The RAG (Retrieval-Augmented Generation) Chatbot is an interactive assistant integrated into the Humanoid Robotics Book Docusaurus site. It allows readers to ask questions about the book content and receive contextually relevant answers based on the book's material.

## Features

### 1. Basic Chatbot Interaction
- Users can ask questions about humanoid robotics content
- Answers are generated based on the book's content using RAG technology
- Real-time conversation interface

### 2. Dual Retrieval Modes
- **Global Search Mode**: Searches the entire book content for relevant information
- **Selected Text Mode**: Focuses on specific text that the user has selected on the page

### 3. Conversation History
- Maintains context across multiple turns in a conversation
- Users can view their conversation history
- Session persistence for continuity

## Architecture

### Frontend Components
- `ChatWidget.tsx`: Main chat interface component
- `ConversationHistory.tsx`: Component for displaying conversation history
- `TextSelector.tsx`: Handles text selection functionality
- `ModeSelector.tsx`: Allows switching between global and selected-text modes
- `chatService.ts`: Service layer for API communication

### Backend Components
- FastAPI backend server
- Neon Postgres database for session storage
- Qdrant vector store for content indexing
- OpenAI integration for response generation
- Content ingestion pipeline

## API Endpoints

### Chat Endpoints
- `POST /api/chat`: Send a message and receive a response
  - Request: `{ session_id, message, mode, selected_text? }`
  - Response: `{ session_id, response, retrieved_context?, timestamp }`

### Session Endpoints
- `POST /api/session`: Create a new chat session
  - Request: `{ mode }`
  - Response: `{ session_id, mode, created_at }`
- `GET /api/session/{sessionId}/history`: Get conversation history
  - Response: `{ session_id, history: [{message_id, role, content, timestamp}], count }`

## Implementation Details

### Context Awareness
The chatbot maintains conversation context by storing the session ID in localStorage and retrieving conversation history when a session is resumed.

### Retrieval Process
1. **Global Mode**: Searches the entire indexed book content
2. **Selected Text Mode**: Limits search to the specific text selection provided by the user

### Error Handling
- Network error detection and user-friendly messages
- Input validation for message length and content
- Graceful degradation when services are unavailable

## Getting Started

### Prerequisites
- Node.js and npm for frontend
- Python 3.8+ for backend
- Docker for running Qdrant vector store
- OpenAI API key

### Installation

1. Clone the repository
2. Install frontend dependencies:
   ```bash
   cd frontend
   npm install
   ```
3. Install backend dependencies:
   ```bash
   cd backend
   pip install -r requirements.txt
   ```
4. Set up environment variables in `.env` files for both frontend and backend

### Running the Application

1. Start the backend:
   ```bash
   cd backend
   python main.py
   ```
2. Start the frontend:
   ```bash
   cd frontend
   npm start
   ```
3. Start the Docusaurus site:
   ```bash
   cd docs
   npm start
   ```

## Configuration

### Environment Variables

#### Frontend (.env)
- `REACT_APP_API_BASE_URL`: Backend API URL (default: http://localhost:8000)

#### Backend (.env)
- `OPENAI_API_KEY`: OpenAI API key for language model
- `QDRANT_URL`: Qdrant vector database URL
- `DATABASE_URL`: Neon Postgres database URL
- `QDRANT_API_KEY`: Qdrant API key (if using cloud)

## Testing

The application includes multiple layers of testing:
- Unit tests for core functionality
- Integration tests for API endpoints
- Contract tests for API compliance
- End-to-end tests for user workflows

## Security Considerations

- Input sanitization for user messages
- Rate limiting to prevent abuse
- Secure API key storage
- HTTPS in production environments

## Performance Optimization

- Vector search optimization in Qdrant
- Multi-level caching system for query responses and retrieval results
- Efficient context window management
- Asynchronous processing for better responsiveness
- Rate limiting to prevent abuse

## Troubleshooting

### Common Issues
- **API Connection Errors**: Verify backend is running and API URL is correct
- **Empty Responses**: Check OpenAI API key and quota limits
- **Slow Performance**: Verify vector database indexing and network connectivity

### Logging
The application includes comprehensive logging for debugging:
- Frontend: Console logs for user interactions
- Backend: Structured logs for API requests and processing

## Future Enhancements

- Enhanced context window management
- Support for file uploads and analysis
- Multi-language support
- Advanced conversation memory
- Integration with additional content sources

## Contributing

See the main repository documentation for contribution guidelines.

## License

This project is licensed under the terms specified in the main repository.