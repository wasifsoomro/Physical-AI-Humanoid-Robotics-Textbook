"""
Cohere client for the RAG Chatbot system
Handles embedding generation and chat completion
"""

import os
from typing import List, Dict, Any, Optional
from dotenv import load_dotenv
from ..utils.logging_config import get_logger
from ..utils.exceptions import CohereException
import time
import random

# Try to import cohere, but provide fallback if not available
try:
    import cohere
    COHERE_AVAILABLE = True
except ImportError:
    COHERE_AVAILABLE = False
    cohere = None  # type: ignore

load_dotenv()

logger = get_logger(__name__)

class CohereClient:
    def __init__(self):
        if COHERE_AVAILABLE:
            api_key = os.getenv("COHERE_API_KEY")
            if not api_key:
                logger.warning("COHERE_API_KEY environment variable is not set")
                self._use_fallback = True
            else:
                try:
                    self.client = cohere.Client(api_key)
                    # Use embed-english-v3.0 model which is Cohere's latest embedding model
                    self.embedding_model = "embed-english-v3.0"
                    # Try to get the chat model from environment, with fallback to command-r-plus
                    self.chat_model = os.getenv("COHERE_CHAT_MODEL", "command-r-plus")
                    self._use_fallback = False
                except Exception as e:
                    logger.error(f"Error initializing Cohere client: {e}")
                    self._use_fallback = True
        else:
            logger.warning("Cohere library not available, using fallback implementation")
            self._use_fallback = True

    def generate_embedding(self, text: str) -> List[float]:
        """Generate embedding for a given text"""
        try:
            start_time = time.time()

            if self._use_fallback:
                # Fallback implementation: create a simple embedding based on character hash
                # This is not a real embedding but allows the system to function
                import hashlib
                import struct

                # Create a simple deterministic "embedding" of fixed length
                text_hash = hashlib.md5(text.encode('utf-8')).digest()
                embedding = []
                for i in range(0, len(text_hash), 4):
                    # Convert 4 bytes to a float
                    chunk = text_hash[i:i+4]
                    if len(chunk) == 4:
                        # Unpack as unsigned int and normalize to range [-1, 1]
                        val = struct.unpack('I', chunk)[0]
                        normalized_val = (val % 200000) / 100000.0 - 1.0  # Normalize to [-1, 1]
                        embedding.append(normalized_val)

                # Ensure embedding has a consistent length (pad or truncate)
                embedding = embedding[:384]  # Common embedding dimension
                while len(embedding) < 384:
                    embedding.append(0.0)

                duration = time.time() - start_time
                logger.info(f"Generated fallback embedding in {duration:.2f}s for text length {len(text)}")
                return embedding
            else:
                # Add delay to respect rate limits (Cohere trial key has 100 calls/min limit)
                time.sleep(0.65)  # Sleep for 650ms to stay under the rate limit

                # Cohere's embed method - using the text type for search queries
                response = self.client.embed(
                    texts=[text],
                    model=self.embedding_model,
                    input_type="search_query"  # Using search_query for questions
                )

                embedding = response.embeddings[0]  # Get the first embedding
                duration = time.time() - start_time

                logger.info(f"Generated embedding in {duration:.2f}s for text length {len(text)}")
                return embedding
        except Exception as e:
            logger.error(f"Error generating embedding: {e}")
            raise CohereException(f"Failed to generate embedding: {str(e)}")

    def generate_embeddings_batch(self, texts: List[str]) -> List[List[float]]:
        """Generate embeddings for a batch of texts"""
        try:
            start_time = time.time()

            if self._use_fallback:
                # Fallback implementation: generate embeddings for each text
                embeddings = []
                for text in texts:
                    embedding = self.generate_embedding(text)
                    embeddings.append(embedding)

                duration = time.time() - start_time
                logger.info(f"Generated {len(embeddings)} fallback embeddings in {duration:.2f}s")
                return embeddings
            else:
                # Cohere's embed method for batch processing
                response = self.client.embed(
                    texts=texts,
                    model=self.embedding_model,
                    input_type="search_document"  # Using search_document for content chunks
                )

                embeddings = response.embeddings
                duration = time.time() - start_time

                logger.info(f"Generated {len(embeddings)} embeddings in {duration:.2f}s")
                return embeddings
        except Exception as e:
            logger.error(f"Error generating batch embeddings: {e}")
            raise CohereException(f"Failed to generate batch embeddings: {str(e)}")

    def generate_chat_completion(self, messages: List[Dict[str, str]],
                                context: Optional[str] = None) -> str:
        """Generate chat completion with optional context"""
        start_time = time.time()

        if self._use_fallback:
            # Fallback implementation: return a concise response based on context and messages
            if context:
                # If context is provided, create a concise response based on it
                current_query = messages[-1]["content"] if messages else "Hello"
                # Check if the query seems to need detailed explanation or can be answered concisely
                if any(word in current_query.lower() for word in ["explain", "how does", "why", "what is", "describe"]):
                    # For explanation-type queries, provide more detail
                    fallback_response = f"Based on the provided context: {current_query} - For detailed information, please refer to the specific sections in the humanoid robotics book content that discuss this topic."
                else:
                    # For factual/simple queries, be more concise
                    fallback_response = f"Based on the provided context: Relevant information found in the humanoid robotics book content. Please refer to the book for specific details."
            else:
                # If no context, provide a concise general response
                current_query = messages[-1]["content"] if messages else "Hello"
                fallback_response = f"Thank you for your question: '{current_query}'. I'm currently using a fallback system. For accurate information, please refer to the humanoid robotics book content directly."

            duration = time.time() - start_time
            logger.info(f"Generated fallback chat completion in {duration:.2f}s")
            return fallback_response
        else:
            # Prepare the chat history from the messages
            chat_history = []

            # Process messages, converting them to Cohere format
            for msg in messages:
                role = msg.get("role", "user")
                content = msg.get("content", "")

                # Convert role to Cohere format
                cohere_role = "USER" if role.lower() in ["user", "human"] else "CHATBOT"

                chat_history.append({
                    "role": cohere_role,
                    "message": content
                })

            # Prepare the final message (the current user query)
            # Get the last message as the current query
            current_query = messages[-1]["content"] if messages else "Hello"

            # Prepare the preamble with context if provided
            preamble = None
            if context:
                preamble = f"You are a helpful assistant for the Humanoid Robotics Book. Answer questions based on the following context:\n\n{context}\n\nProvide concise, to-the-point answers. For simple factual questions, give brief responses. For complex topics that need explanation, provide detailed responses. Always be accurate and reference the provided context. If the information is not available in the context, respond that you don't have that information in the provided content."

            try:
                # Call Cohere's chat endpoint
                response = self.client.chat(
                    model=self.chat_model,
                    message=current_query,
                    chat_history=chat_history[:-1] if len(chat_history) > 1 else [],  # Exclude the current query from history
                    preamble=preamble,
                    temperature=0.5,  # Slightly higher temperature for more natural, concise responses
                    max_tokens=500    # Reduced max tokens to encourage more concise responses
                )

                completion = response.text
                duration = time.time() - start_time

                logger.info(f"Generated chat completion in {duration:.2f}s")
                return completion
            except Exception as e:
                # Check if this is a model not found error - if so, switch to fallback mode
                error_str = str(e).lower()
                if "model" in error_str and ("not found" in error_str or "removed" in error_str or "404" in str(e)):
                    logger.warning(f"Chat model '{self.chat_model}' not found or invalid. Switching to fallback mode.")
                    self._use_fallback = True
                    # Now call the method again using fallback
                    return self.generate_chat_completion(messages, context)
                else:
                    logger.error(f"Error generating chat completion: {e}")
                    raise CohereException(f"Failed to generate chat completion: {str(e)}")

    def validate_api_key(self) -> bool:
        """Validate the Cohere API key by making a simple request"""
        try:
            if self._use_fallback:
                # If using fallback, we can't validate an API key
                return False
            else:
                # Test with a simple embedding request
                test_embedding = self.generate_embedding("test")
                return len(test_embedding) > 0
        except Exception:
            return False