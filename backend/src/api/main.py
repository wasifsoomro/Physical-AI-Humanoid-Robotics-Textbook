"""
Main FastAPI application with API router configuration
"""

from fastapi import FastAPI, Request
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse
from .chat_routes import router as chat_router
from ..utils.exceptions import BaseRAGException
from ..utils.logging_config import setup_logging, log_error
import logging

# Setup logging
setup_logging()

app = FastAPI(
    title="RAG Chatbot API",
    description="API for the RAG Chatbot system integrated with the Docusaurus Book",
    version="1.0.0"
)

# Add CORS middleware
import os
# Get allowed origins from environment variable, default to all for development
allowed_origins_legacy = os.getenv("ALLOWED_ORIGINS", "*").split(",")  # For backward compatibility
# Remove any empty strings and strip whitespace
allowed_origins_legacy = [origin.strip() for origin in allowed_origins_legacy if origin.strip()]

# Check the correct environment variable name as well
allowed_origins_new = os.getenv("ALLOWED_ORIGINS", "").split(",")  # Correct spelling
allowed_origins_new = [origin.strip() for origin in allowed_origins_new if origin.strip()]

# Use the correct variable if it's set, otherwise use the legacy one or default
if allowed_origins_new:
    # Use the new variable if it's set with values
    allowed_origins = allowed_origins_new
elif allowed_origins_legacy and allowed_origins_legacy != ["*"]:
    # Use legacy if new one isn't set but legacy has specific origins
    allowed_origins = allowed_origins_legacy
else:
    # Default to allowing all origins for development if neither is set properly
    allowed_origins = ["*"]

app.add_middleware(
    CORSMiddleware,
    allow_origins=allowed_origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Exception handlers
@app.exception_handler(BaseRAGException)
async def handle_rag_exception(request: Request, exc: BaseRAGException):
    log_error(exc, f"RAG Exception at {request.url.path}")
    return JSONResponse(
        status_code=400,
        content={"detail": exc.message, "error_code": exc.error_code}
    )

@app.exception_handler(Exception)
async def handle_general_exception(request: Request, exc: Exception):
    log_error(exc, f"General Exception at {request.url.path}")
    return JSONResponse(
        status_code=500,
        content={"detail": "An internal server error occurred"}
    )

# Include API routes
app.include_router(chat_router, prefix="/api", tags=["chat"])

@app.get("/")
def read_root():
    return {"message": "RAG Chatbot API is running"}

@app.get("/health")
def health_check():
    return {"status": "healthy"}