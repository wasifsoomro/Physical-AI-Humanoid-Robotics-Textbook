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
app.add_middleware(
    CORSMiddleware,
    allow_origins=["https://physical-ai-humanoid-robotics-textbook.vercel.app"],  # In production, replace with specific origins
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
