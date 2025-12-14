# Data Model: RAG Chatbot for Docusaurus Book

**Feature**: 1-rag-chatbot
**Created**: 2025-12-08

## Overview
This document defines the data models for the RAG Chatbot system, including database schemas for Neon Postgres and vector store payload structures for Qdrant.

## Database Schema (Neon Postgres)

### 1. Chat Sessions Table
**Table**: `chat_sessions`

| Column | Type | Constraints | Description |
|--------|------|-------------|-------------|
| session_id | UUID | PRIMARY KEY, NOT NULL | Unique identifier for the session |
| user_id | UUID | DEFAULT gen_random_uuid() | Identifier for the user (anonymous if not authenticated) |
| created_at | TIMESTAMP | NOT NULL, DEFAULT NOW() | Timestamp when session was created |
| last_active | TIMESTAMP | NOT NULL, DEFAULT NOW() | Timestamp of last interaction |
| mode | VARCHAR(20) | NOT NULL, CHECK in ('global', 'selected-text') | Current retrieval mode |
| metadata | JSONB | | Additional session metadata |

**Indexes**:
- Primary: session_id
- Secondary: last_active (for cleanup queries)

### 2. Chat Messages Table
**Table**: `chat_messages`

| Column | Type | Constraints | Description |
|--------|------|-------------|-------------|
| message_id | UUID | PRIMARY KEY, NOT NULL | Unique identifier for the message |
| session_id | UUID | FOREIGN KEY, NOT NULL | References parent session |
| role | VARCHAR(10) | NOT NULL, CHECK in ('user', 'assistant') | Role of the message sender |
| content | TEXT | NOT NULL | The actual message content |
| timestamp | TIMESTAMP | NOT NULL, DEFAULT NOW() | When the message was created |
| retrieved_context | JSONB | | Context fragments used to generate response |
| source_chunks | UUID[] | | IDs of source content chunks used |

**Indexes**:
- Primary: message_id
- Foreign Key: session_id → chat_sessions(session_id)
- Secondary: session_id, timestamp (for chronological message retrieval)

### 3. User Sessions Table (Optional - for authenticated users)
**Table**: `user_sessions`

| Column | Type | Constraints | Description |
|--------|------|-------------|-------------|
| user_id | UUID | PRIMARY KEY, NOT NULL | Unique identifier for the user |
| email | VARCHAR(255) | UNIQUE | User's email (if authenticated) |
| created_at | TIMESTAMP | NOT NULL, DEFAULT NOW() | When user session was created |
| last_active | TIMESTAMP | NOT NULL, DEFAULT NOW() | Last activity timestamp |

## Vector Store Schema (Qdrant)

### Content Chunks Collection Payload
**Collection**: `book_content_chunks`

**Vector Configuration**:
- Size: 1536 (for OpenAI text-embedding-3-large)
- Distance: Cosine

**Payload Fields**:

| Field | Type | Description |
|-------|------|-------------|
| chunk_id | Keyword | Unique identifier for the chunk (UUID) |
| document_id | Keyword | Identifier for the source document |
| content | Text | The actual content chunk (text to be embedded) |
| section_title | Keyword | Title of the section this chunk belongs to |
| page_reference | Keyword | Reference to the original location (e.g., "module1/chapter1") |
| token_count | Integer | Number of tokens in the chunk |
| chunk_index | Integer | Sequential index of this chunk within the document |

**Indexes**:
- Point index on chunk_id (for direct retrieval)
- Field index on document_id (for document-scoped searches)
- Field index on section_title (for section-scoped searches)

## Data Relationships

### Session-Message Relationship
- One `chat_session` can have many `chat_messages`
- Foreign key constraint ensures referential integrity
- Cascading delete for session cleanup

### Content Retrieval Flow
1. User query → Vector search in Qdrant
2. Retrieved chunks → Context injection for OpenAI
3. OpenAI response → Store in chat_messages with source references

## Validation Rules

### Session Validation
- Session mode must be either 'global' or 'selected-text'
- Session cannot be older than 30 days (cleanup requirement)
- User ID format must be valid UUID

### Message Validation
- Message content must be between 1 and 10,000 characters
- Role must be either 'user' or 'assistant'
- Timestamp cannot be in the future
- Retrieved context must reference valid chunk IDs

### Content Chunk Validation
- Content length must correspond to 300-800 token range
- Document ID must reference an existing book document
- Chunk index must be sequential within document

## Performance Considerations

### Indexing Strategy
- Frequently queried fields are indexed in both Postgres and Qdrant
- Composite indexes for multi-field queries
- Proper partitioning for large datasets

### Storage Optimization
- JSONB for flexible metadata in Postgres
- Efficient vector storage in Qdrant
- Compression for large text fields where appropriate

## Security Considerations

### Data Protection
- Sensitive user data is encrypted at rest
- Session data is anonymized by default
- API logs don't store full message content

### Access Control
- Session-based access control for conversation history
- No direct access to vector store from frontend
- Rate limiting to prevent abuse