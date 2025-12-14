"""
Conversation service for the RAG Chatbot system
Handles the logic for processing user messages and generating responses
"""

from typing import Dict, Any, List, Optional
from datetime import datetime
import uuid
from sqlalchemy.orm import Session
from ..models.chat import ChatMessageCreate, ChatSession
from ..database.models import ChatMessage as ChatMessageModel
from ..database.database import get_db
from ..ai.cohere_client import CohereClient
from ..services.retrieval_service import RetrievalService
from ..utils.logging_config import get_logger
from ..utils.exceptions import ValidationError, ContentRetrievalException
from ..utils.cache import get_cached_query_response, cache_query_response
from ..utils.validation import sanitize_input, validate_query_params, validate_context_window_size
from ..api.auth import create_session_if_not_exists


logger = get_logger(__name__)


class ConversationService:
    def __init__(self):
        self.cohere_client = CohereClient()
        self.retrieval_service = RetrievalService()

    def process_message(
        self,
        session_id: str,
        user_message: str,
        mode: str = "global",
        selected_text: Optional[str] = None,
        db: Session = None,
        context_window_size: int = 5
    ) -> Dict[str, Any]:
        """
        Process a user message and generate an AI response with conversation context
        """
        # Try to get cached response first (using a cache key that includes session context)
        cache_key_suffix = f"{session_id}_{context_window_size}"
        cached_result = get_cached_query_response(user_message, mode, cache_key_suffix)
        if cached_result is not None:
            logger.info(f"Cache hit for query: {user_message[:50]}...")
            return cached_result

        try:
            # Sanitize inputs
            user_message = sanitize_input(user_message) if user_message else user_message
            selected_text = sanitize_input(selected_text) if selected_text else selected_text

            # Validate all query parameters
            is_valid, error_msg = validate_query_params(user_message, mode, selected_text)
            if not is_valid:
                raise ValidationError(error_msg)

            # Validate context window size
            is_valid, error_msg = validate_context_window_size(context_window_size)
            if not is_valid:
                raise ValidationError(error_msg)

            # Validate session ID format
            if not session_id or len(session_id) < 10:
                raise ValidationError(f"Invalid session ID format: {session_id}")

            # Get or create session
            if db:
                session = create_session_if_not_exists(db, session_id, mode)
            else:
                # If no DB session provided, just continue without DB operations
                session = None

            # Retrieve conversation history for context (using configurable context window)
            conversation_context = []
            if db:
                conversation_context = self._get_recent_conversation_context(db, session_id, limit=context_window_size)

            # Validate retrieved context
            if conversation_context is None:
                conversation_context = []

            # Retrieve relevant content based on the query and mode
            retrieved_content = []
            context_str = ""

            try:
                if mode == "selected-text" and selected_text:
                    # Validate selected text
                    if not selected_text.strip():
                        raise ValidationError("Selected text cannot be empty in selected-text mode")

                    # In selected-text mode, use the provided text
                    retrieved_content = self.retrieval_service.retrieve_content_from_selected_text_only(
                        query=user_message,
                        selected_text=selected_text
                    )
                else:
                    # In global mode, search across all content
                    retrieved_content = self.retrieval_service.retrieve_content(
                        user_message,
                        mode=mode
                    )
            except Exception as retrieval_error:
                logger.error(f"Error during content retrieval: {retrieval_error}", exc_info=True)
                # Continue with empty context if retrieval fails, rather than failing completely
                retrieved_content = []

            # Validate retrieved content
            if retrieved_content is None:
                retrieved_content = []

            # Create context string from retrieved content
            if retrieved_content:
                context_parts = []
                for item in retrieved_content:
                    if isinstance(item, dict) and "content" in item:
                        context_parts.append(item["content"])
                    else:
                        logger.warning(f"Invalid retrieved content item: {item}")
                context_str = "\n\n".join(context_parts)

            # Prepare messages for OpenAI, including conversation history for context
            messages = []

            # Add conversation history if available (maintaining role consistency)
            if conversation_context:
                for msg in conversation_context:
                    if hasattr(msg, 'role') and hasattr(msg, 'content'):
                        messages.append({
                            "role": msg.role,
                            "content": msg.content
                        })
                    else:
                        logger.warning(f"Invalid message object in conversation context: {msg}")

            # Add the current user message
            messages.append({
                "role": "user",
                "content": user_message
            })

            # Trim messages to stay within token limits
            messages = self._trim_context_by_token_limit(messages)

            # Validate that we have messages to send
            if not messages:
                raise ValidationError("No valid messages to send to AI service")

            try:
                # Generate response using Cohere with both retrieved context and conversation history
                response = self.cohere_client.generate_chat_completion(
                    messages=messages,
                    context=context_str
                )
            except Exception as ai_error:
                logger.error(f"Error during AI response generation: {ai_error}", exc_info=True)
                # Provide a fallback response when AI service fails
                response = "I'm currently experiencing technical difficulties. Please try again in a moment. In the meantime, I recommend checking the humanoid robotics book content directly for the information you need."

            # Create response object
            result = {
                "session_id": session_id,
                "response": response,
                "retrieved_context": retrieved_content,
                "timestamp": datetime.utcnow().isoformat()
            }

            # Cache the result for future identical queries (with same session context)
            cache_query_response(user_message, mode, result, cache_key_suffix, ttl=300)  # 5 minutes TTL

            # Save the conversation to database if DB session is available
            if db and session:
                try:
                    self._save_conversation(db, session_id, user_message, response, retrieved_content)
                except Exception as db_error:
                    logger.error(f"Error saving conversation to database: {db_error}", exc_info=True)
                    # Don't fail the entire request if saving to DB fails

            logger.info(f"Processed message for session {session_id}, mode: {mode}, context_window: {context_window_size}")
            return result

        except ValidationError:
            # Re-raise validation errors as they are
            raise
        except Exception as e:
            logger.error(f"Error processing message: {e}", exc_info=True)
            # Provide a fallback response in case of unexpected errors
            return {
                "session_id": session_id,
                "response": "I'm currently experiencing technical difficulties. Please try again in a moment. In the meantime, I recommend checking the humanoid robotics book content directly for the information you need.",
                "retrieved_context": [],
                "timestamp": datetime.utcnow().isoformat()
            }

    def _get_recent_conversation_context(self, db: Session, session_id: str, limit: int = 5) -> List[ChatMessageModel]:
        """
        Retrieve recent conversation messages for context
        """
        try:
            # Validate inputs
            if not db:
                logger.warning("Database session is None, returning empty context")
                return []

            if not session_id:
                logger.warning("Session ID is None or empty, returning empty context")
                return []

            if limit <= 0:
                logger.warning(f"Invalid limit: {limit}, using default of 5")
                limit = 5

            # Query the database for recent messages in this session
            recent_messages = db.query(ChatMessageModel) \
                .filter(ChatMessageModel.session_id == session_id) \
                .order_by(ChatMessageModel.timestamp.desc()) \
                .limit(limit) \
                .all()

            # Validate the results
            if recent_messages is None:
                recent_messages = []

            # Reverse the order to have oldest first (chronological order)
            recent_messages.reverse()

            # Filter out any invalid messages
            valid_messages = []
            for msg in recent_messages:
                if msg and hasattr(msg, 'role') and hasattr(msg, 'content') and hasattr(msg, 'timestamp'):
                    valid_messages.append(msg)
                else:
                    logger.warning(f"Invalid message object in conversation context: {msg}")

            return valid_messages

        except Exception as e:
            logger.error(f"Error retrieving conversation context: {e}")
            return []

    def _trim_context_by_token_limit(self, messages: List[Dict[str, str]], max_tokens: int = 3000) -> List[Dict[str, str]]:
        """
        Trim conversation context to stay within token limits
        Uses a more accurate token counting approach with caching
        """
        try:
            # Validate inputs
            if not messages:
                return []

            if max_tokens <= 0:
                logger.warning(f"Invalid max_tokens: {max_tokens}, using default of 3000")
                max_tokens = 3000

            # Use a simple caching mechanism for token counts to avoid recomputing
            # This is still a rough estimation - in production, use proper token counting library
            token_cache = {}

            # Calculate total token count for all messages
            total_tokens = 0
            message_tokens = []

            for message in messages:
                if not isinstance(message, dict):
                    logger.warning(f"Invalid message format: {message}, skipping")
                    message_tokens.append(0)
                    continue

                message_content = message.get("content", "")
                if not isinstance(message_content, str):
                    logger.warning(f"Invalid message content: {message_content}, skipping")
                    message_tokens.append(0)
                    continue

                # Create a cache key for this content
                content_key = hash(message_content)
                if content_key in token_cache:
                    tokens = token_cache[content_key]
                else:
                    # More accurate estimation: 1 token is roughly 4 characters, but include role tokens
                    tokens = len(message_content) // 4
                    # Add tokens for role and structure (approximately 1-2 tokens per message)
                    tokens += 2
                    token_cache[content_key] = tokens

                message_tokens.append(tokens)
                total_tokens += tokens

            # If we're under the limit, return all messages
            if total_tokens <= max_tokens:
                return messages

            # Otherwise, remove messages starting from the oldest until we're under the limit
            trimmed_messages = []
            remaining_tokens = 0

            # Process in reverse order to keep the most recent messages
            for i in range(len(messages) - 1, -1, -1):
                message = messages[i]
                tokens = message_tokens[i]

                if remaining_tokens + tokens <= max_tokens:
                    trimmed_messages.insert(0, message)  # Insert at beginning to maintain order
                    remaining_tokens += tokens
                else:
                    # Stop when we would exceed the limit
                    break

            logger.info(f"Context trimming: reduced from {total_tokens} to {remaining_tokens} tokens out of {max_tokens}")
            return trimmed_messages

        except Exception as e:
            logger.error(f"Error trimming context by token limit: {e}")
            # Return original messages if trimming fails
            return messages

    def _save_conversation(
        self,
        db: Session,
        session_id: str,
        user_message: str,
        ai_response: str,
        retrieved_context: List[Dict[str, Any]]
    ) -> None:
        """
        Save the conversation to the database
        """
        try:
            # Validate inputs
            if not db:
                logger.warning("Database session is None, skipping conversation save")
                return

            if not session_id:
                logger.warning("Session ID is None or empty, skipping conversation save")
                return

            if not user_message:
                logger.warning("User message is None or empty, skipping conversation save")
                return

            if not ai_response:
                logger.warning("AI response is None or empty, skipping conversation save")
                return

            # Validate retrieved context
            if retrieved_context is None:
                retrieved_context = []

            # Validate and clean retrieved context
            validated_context = []
            for item in retrieved_context:
                if isinstance(item, dict):
                    validated_context.append(item)
                else:
                    logger.warning(f"Invalid retrieved context item: {item}, skipping")

            # Create user message record
            user_msg = ChatMessageCreate(
                session_id=session_id,
                role="user",
                content=user_message,
                retrieved_context={} if not validated_context else validated_context[0] if isinstance(validated_context, list) and len(validated_context) > 0 else validated_context,
                source_chunks=[]  # Will be populated if we track chunk IDs
            )

            user_db_msg = ChatMessageModel(
                session_id=user_msg.session_id,
                role=user_msg.role,
                content=user_msg.content,
                retrieved_context=user_msg.retrieved_context,
                source_chunks=user_msg.source_chunks
            )
            db.add(user_db_msg)

            # Create assistant message record
            assistant_msg = ChatMessageCreate(
                session_id=session_id,
                role="assistant",
                content=ai_response,
                retrieved_context={} if not validated_context else validated_context[0] if isinstance(validated_context, list) and len(validated_context) > 0 else validated_context,
                source_chunks=[]  # Will be populated if we track chunk IDs
            )

            assistant_db_msg = ChatMessageModel(
                session_id=assistant_msg.session_id,
                role=assistant_msg.role,
                content=assistant_msg.content,
                retrieved_context=assistant_msg.retrieved_context,
                source_chunks=assistant_msg.source_chunks
            )
            db.add(assistant_db_msg)

            db.commit()
            logger.info(f"Successfully saved conversation for session {session_id}")

        except Exception as e:
            logger.error(f"Error saving conversation to database: {e}")
            if db:
                db.rollback()
            # Don't raise exception as this shouldn't break the conversation flow