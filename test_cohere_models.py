#!/usr/bin/env python3
"""
Test script to identify valid Cohere models with your API key
"""

import os
import sys
from pathlib import Path

# Add the project root to the path so .env can be loaded
project_root = Path(__file__).parent
sys.path.insert(0, str(project_root))

from dotenv import load_dotenv

# Load environment variables from project root
load_dotenv(dotenv_path=Path(__file__).parent / ".backend.env")

# Try to import cohere
try:
    import cohere
    print("Cohere library is available")
except ImportError:
    print("Cohere library is not installed. Run: pip install cohere")
    exit(1)

# Get your API key
api_key = os.getenv("COHERE_API_KEY")
if not api_key:
    print("COHERE_API_KEY environment variable is not set")
    exit(1)

print(f"Using API key: {api_key[:10]}...")  # Only show first 10 chars for security

# Initialize the client
client = cohere.Client(api_key)

# List of potential models to test
potential_models = [
    "command-r-plus",    # Current high-performance model
    "command-r",         # Current standard model
    "command",           # Legacy model (may be deprecated)
    "command-light",     # Legacy model (may be deprecated)
    "c4ai-aya-expanse-8b",  # Another available model
    "c4ai-aya-expanse-32b"  # Another available model
]

print("\nTesting available models...")

for model in potential_models:
    try:
        # Try a simple call to see if the model exists
        response = client.chat(
            model=model,
            message="Hello, how are you?",
            max_tokens=5  # Very few tokens to minimize cost
        )
        print(f"[SUCCESS] Model '{model}' is available")
    except Exception as e:
        error_msg = str(e).lower()
        if "model" in error_msg and ("not found" in error_msg or "removed" in error_msg or "404" in str(e)):
            print(f"[FAILED] Model '{model}' is not available: {e}")
        else:
            # Model exists but there might be other issues (like rate limiting)
            print(f"[POTENTIAL] Model '{model}' may be available but has other issues: {e}")

print("\nYou should set COHERE_CHAT_MODEL in your .backend.env file to one of the available models above.")
print("The recommended model is usually 'command-r-plus' for best performance.")