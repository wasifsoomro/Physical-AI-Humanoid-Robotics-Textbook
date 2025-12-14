"""
API routes for the chat functionality
"""

from fastapi import APIRouter, Depends, HTTPException, Request, Response
from sqlalchemy.orm import Session
from typing import Optional
import uuid
from datetime import datetime
from ..models.chat import ChatRequest, ChatResponse, ChatSessionCreateRequest
from ..services.conversation_service import ConversationService
from ..services.retrieval_service import RetrievalService
from ..database.database import get_db
from ..utils.exceptions import raise_session_not_found, raise_invalid_mode, raise_content_not_found, raise_bad_request, raise_internal_error
from ..utils.logging_config import log_api_call, log_performance, get_logger
from ..utils.rate_limiter import check_rate_limit
from ..api.auth import validate_session_id
from ..ingestion.content_pipeline import ContentIngestionPipeline
import time


router = APIRouter()


@router.post("/chat", response_model=ChatResponse)
async def chat_endpoint(
    request: ChatRequest,
    response: Response,
    db: Session = Depends(get_db)
):
    """
    Process a chat message and return an AI response
    """
    start_time = time.time()

    # Validate session ID format
    if not validate_session_id(request.session_id):
        raise raise_bad_request(f"Invalid session ID format: {request.session_id}")

    # Validate mode
    if request.mode not in ["global", "selected-text"]:
        raise raise_invalid_mode(request.mode)

    # Rate limiting: use session ID as client identifier
    is_allowed, rate_headers = check_rate_limit(str(request.session_id), "/api/chat")
    if not is_allowed:
        logger = get_logger("rate_limit")
        logger.warning(f"Rate limit exceeded for session {request.session_id}")
        raise raise_bad_request("Rate limit exceeded. Please try again later.")

    try:
        # Log mode switching if different from previous mode (if we were tracking it)
        logger = get_logger("chat")
        logger.info(f"Processing chat request with mode '{request.mode}' for session {request.session_id}")

        # Initialize services
        conversation_service = ConversationService()

        # Process the message
        result = conversation_service.process_message(
            session_id=str(request.session_id),
            user_message=request.message,
            mode=request.mode,
            selected_text=request.selected_text,
            db=db
        )

        # Log API call
        duration = time.time() - start_time
        log_api_call(
            endpoint="/api/chat",
            method="POST",
            session_id=str(request.session_id),
            duration=duration
        )
        log_performance("chat_processing", duration, session_id=str(request.session_id))

        # Add rate limit headers to the response object
        for header, value in rate_headers.items():
            response.headers[header] = str(value)

        # Return the response
        return ChatResponse(
            session_id=result["session_id"],
            response=result["response"],
            retrieved_context=result.get("retrieved_context"),
            timestamp=result["timestamp"]
        )

    except Exception as e:
        duration = time.time() - start_time
        log_api_call(
            endpoint="/api/chat",
            method="POST",
            session_id=str(request.session_id),
            duration=duration
        )

        # Log the full error for debugging
        logger = get_logger("chat")
        logger.error(f"Error processing chat request for session {request.session_id}: {str(e)}", exc_info=True)

        # Provide a graceful degradation response
        if "content" in str(e).lower() or "cohere" in str(e).lower() or "api" in str(e).lower() or "vector" in str(e).lower():
            # If it's an AI service error, provide a fallback response
            fallback_response = {
                "session_id": str(request.session_id),
                "response": "I'm currently experiencing technical difficulties. Please try again in a moment. In the meantime, I recommend checking the humanoid robotics book content directly for the information you need.",
                "retrieved_context": [],
                "timestamp": datetime.utcnow().isoformat()
            }

            # Add rate limit headers if they exist
            for header, value in rate_headers.items():
                response.headers[header] = str(value)

            return ChatResponse(**fallback_response)
        else:
            # For other errors, raise the original exception
            raise e


@router.post("/retrieve")
async def retrieve_endpoint(
    request: ChatRequest,  # Reusing ChatRequest for simplicity, though we could create a specific one
    response: Response,
    db: Session = Depends(get_db)
):
    """
    Retrieve relevant content based on query (without generating response)
    """
    start_time = time.time()

    # Validate session ID format
    if not validate_session_id(request.session_id):
        raise raise_bad_request(f"Invalid session ID format: {request.session_id}")

    # Validate mode
    if request.mode not in ["global", "selected-text"]:
        raise raise_invalid_mode(request.mode)

    # Rate limiting: use session ID as client identifier
    is_allowed, rate_headers = check_rate_limit(str(request.session_id), "/api/retrieve")
    if not is_allowed:
        logger = get_logger("rate_limit")
        logger.warning(f"Rate limit exceeded for session {request.session_id}")
        raise raise_bad_request("Rate limit exceeded. Please try again later.")

    try:
        # Log mode for retrieval
        logger = get_logger("retrieval")
        logger.info(f"Processing retrieval request with mode '{request.mode}' for session {request.session_id}")

        # Initialize retrieval service
        retrieval_service = RetrievalService()

        # Retrieve content based on mode
        if request.mode == "selected-text" and request.selected_text:
            results = retrieval_service.retrieve_content_with_selected_text(
                query=request.message,
                selected_text=request.selected_text
            )
        else:
            results = retrieval_service.retrieve_content(
                query=request.message,
                mode=request.mode,
                document_id=request.selected_text  # Using selected_text as document ID in selected-text mode
            )

        # Add rate limit headers to the response object
        for header, value in rate_headers.items():
            response.headers[header] = str(value)

        # Log API call
        duration = time.time() - start_time
        log_api_call(
            endpoint="/api/retrieve",
            method="POST",
            session_id=str(request.session_id),
            duration=duration
        )
        log_performance("content_retrieval", duration, session_id=str(request.session_id))

        return {"results": results}

    except Exception as e:
        duration = time.time() - start_time
        log_api_call(
            endpoint="/api/retrieve",
            method="POST",
            session_id=str(request.session_id),
            duration=duration
        )

        # Log the full error for debugging
        logger = get_logger("retrieval")
        logger.error(f"Error processing retrieval request for session {request.session_id}: {str(e)}", exc_info=True)

        # Provide a graceful degradation response
        if "content" in str(e).lower() or "cohere" in str(e).lower() or "api" in str(e).lower() or "vector" in str(e).lower():
            # If it's an AI service or vector store error, provide a fallback response
            fallback_response = {
                "results": []
            }

            # Add rate limit headers if they exist
            for header, value in rate_headers.items():
                response.headers[header] = str(value)

            return fallback_response
        else:
            # For other errors, raise the original exception
            raise e


# Ingestion endpoints
@router.post("/ingest")
async def ingest_content(
    content_dir: str,
    document_id: str,
    response: Response
):
    """
    Ingest book content from a directory into the vector store
    """
    start_time = time.time()
    logger = get_logger("ingestion")

    try:
        logger.info(f"Starting content ingestion for document: {document_id}, directory: {content_dir}")

        # Initialize the ingestion pipeline
        ingestion_pipeline = ContentIngestionPipeline()

        # Perform the ingestion
        result = ingestion_pipeline.ingest_book_content(content_dir, document_id)

        # Log API call
        duration = time.time() - start_time
        log_api_call(
            endpoint="/api/ingest",
            method="POST",
            session_id=document_id,  # Using document_id as session identifier for logging
            duration=duration
        )
        log_performance("content_ingestion", duration, session_id=document_id)

        logger.info(f"Content ingestion completed successfully: {result}")
        return result

    except Exception as e:
        duration = time.time() - start_time
        log_api_call(
            endpoint="/api/ingest",
            method="POST",
            session_id=document_id,
            duration=duration
        )

        logger.error(f"Error during content ingestion: {str(e)}", exc_info=True)
        raise raise_internal_error(f"Content ingestion failed: {str(e)}")


@router.post("/ingest/validate")
async def validate_content_structure(
    content_dir: str
):
    """
    Validate the structure of content directory before ingestion
    """
    logger = get_logger("ingestion")

    try:
        logger.info(f"Validating content structure for directory: {content_dir}")

        # Initialize the ingestion pipeline
        ingestion_pipeline = ContentIngestionPipeline()

        # Perform validation
        result = ingestion_pipeline.validate_content_structure(content_dir)

        logger.info(f"Content structure validation completed: {result}")
        return result

    except Exception as e:
        logger.error(f"Error during content structure validation: {str(e)}", exc_info=True)
        raise raise_internal_error(f"Content structure validation failed: {str(e)}")


@router.get("/ingest/stats")
async def get_ingestion_stats():
    """
    Get statistics about the ingestion process
    """
    logger = get_logger("ingestion")

    try:
        logger.info("Retrieving ingestion statistics")

        # Initialize the ingestion pipeline
        ingestion_pipeline = ContentIngestionPipeline()

        # Get stats
        stats = ingestion_pipeline.get_ingestion_stats()

        logger.info(f"Ingestion statistics retrieved: {stats}")
        return stats

    except Exception as e:
        logger.error(f"Error retrieving ingestion statistics: {str(e)}", exc_info=True)
        raise raise_internal_error(f"Failed to retrieve ingestion statistics: {str(e)}")


@router.post("/session")
async def create_session(
    request: ChatSessionCreateRequest,
    db: Session = Depends(get_db)
):
    """
    Create a new chat session
    """
    from ..api.auth import create_session_if_not_exists
    from ..database.models import ChatSession
    import uuid
    from datetime import datetime

    # Log session creation with mode
    logger = get_logger("session")
    logger.info(f"Creating new session with initial mode '{request.mode}'")

    # Generate a new session ID
    session_id = str(uuid.uuid4())

    # Create the session
    session = create_session_if_not_exists(db, session_id, request.mode)

    return {
        "session_id": session.session_id,
        "mode": session.mode,
        "created_at": session.created_at.isoformat()
    }


@router.get("/session/{session_id}/history")
async def get_conversation_history(
    session_id: str,
    response: Response,
    limit: int = 50,  # Default limit of 50 messages
    db: Session = Depends(get_db)
):
    """
    Retrieve conversation history for a specific session
    """
    from ..database.models import ChatMessage
    from sqlalchemy import desc

    # Validate session ID format
    if not validate_session_id(session_id):
        raise raise_bad_request(f"Invalid session ID format: {session_id}")

    # Rate limiting: use session ID as client identifier
    is_allowed, rate_headers = check_rate_limit(session_id, "/api/session/history")
    if not is_allowed:
        logger = get_logger("rate_limit")
        logger.warning(f"Rate limit exceeded for session {session_id}")
        raise raise_bad_request("Rate limit exceeded. Please try again later.")

    try:
        # Query the database for messages in this session
        messages = db.query(ChatMessage) \
            .filter(ChatMessage.session_id == session_id) \
            .order_by(desc(ChatMessage.timestamp)) \
            .limit(min(limit, 100)) \
            .all()  # Added limit to prevent too many results

        # Convert to response format (newest first)
        history = []
        for msg in messages:
            history.append({
                "message_id": str(msg.message_id),
                "role": msg.role,
                "content": msg.content,
                "timestamp": msg.timestamp.isoformat(),
                "retrieved_context": msg.retrieved_context
            })

        # Add rate limit headers to the response object
        for header, value in rate_headers.items():
            response.headers[header] = str(value)

        logger = get_logger("history")
        logger.info(f"Retrieved {len(history)} messages for session {session_id}")

        return {
            "session_id": session_id,
            "history": history,
            "count": len(history)
        }

    except Exception as e:
        logger = get_logger("history")
        logger.error(f"Error retrieving conversation history for session {session_id}: {str(e)}", exc_info=True)

        # Provide a graceful degradation response
        if "database" in str(e).lower() or "connection" in str(e).lower() or "sql" in str(e).lower():
            # If it's a database error, provide an empty history as fallback
            fallback_response = {
                "session_id": session_id,
                "history": [],
                "count": 0
            }

            # Add rate limit headers if they exist
            for header, value in rate_headers.items():
                response.headers[header] = str(value)

            return fallback_response
        else:
            # For other errors, raise the original exception
            raise raise_internal_error(f"Failed to retrieve conversation history: {str(e)}")