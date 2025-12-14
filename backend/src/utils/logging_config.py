"""
Logging configuration for the RAG Chatbot system
"""

import logging
import sys
from datetime import datetime
from typing import Optional
import os
from dotenv import load_dotenv

load_dotenv()

def setup_logging():
    """Set up logging configuration"""
    log_level = os.getenv("LOG_LEVEL", "INFO").upper()

    # Create formatter
    formatter = logging.Formatter(
        '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )

    # Create console handler
    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setFormatter(formatter)

    # Configure root logger
    root_logger = logging.getLogger()
    root_logger.setLevel(getattr(logging, log_level))
    root_logger.addHandler(console_handler)

    # Configure specific loggers
    logging.getLogger('uvicorn').setLevel(getattr(logging, log_level))
    logging.getLogger('uvicorn.error').setLevel(getattr(logging, log_level))
    logging.getLogger('uvicorn.access').setLevel(getattr(logging, log_level))

    return root_logger


def get_logger(name: str) -> logging.Logger:
    """Get a logger with the specified name"""
    return logging.getLogger(name)


def log_api_call(
    endpoint: str,
    method: str,
    user_id: Optional[str] = None,
    session_id: Optional[str] = None,
    duration: Optional[float] = None
) -> None:
    """Log API call information"""
    logger = get_logger("api")
    logger.info(
        f"API_CALL - Method: {method}, Endpoint: {endpoint}, "
        f"User: {user_id}, Session: {session_id}, Duration: {duration}s"
    )


def log_error(
    error: Exception,
    context: str = "",
    user_id: Optional[str] = None,
    session_id: Optional[str] = None
) -> None:
    """Log error with context"""
    logger = get_logger("error")
    logger.error(
        f"ERROR - Context: {context}, User: {user_id}, Session: {session_id}, "
        f"Error: {str(error)}, Type: {type(error).__name__}"
    )


def log_performance(
    operation: str,
    duration: float,
    user_id: Optional[str] = None,
    session_id: Optional[str] = None
) -> None:
    """Log performance metrics"""
    logger = get_logger("performance")
    logger.info(
        f"PERFORMANCE - Operation: {operation}, Duration: {duration}s, "
        f"User: {user_id}, Session: {session_id}"
    )