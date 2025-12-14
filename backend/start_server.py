#!/usr/bin/env python3
"""
Proper startup script for the backend server with environment loading
"""

import os
import sys
from pathlib import Path
from dotenv import load_dotenv

# Load environment variables from the project root
env_path = Path(__file__).parent.parent / ".backend.env"
if env_path.exists():
    load_dotenv(env_path)
    print(f"Loaded environment variables from {env_path}")
else:
    print(f"Environment file not found at {env_path}")

# Add the backend src directory to the path
sys.path.insert(0, str(Path(__file__).parent / "src"))

# Now import and start the app
from src.api.main import app
import uvicorn

if __name__ == "__main__":
    print("Starting backend server...")
    print(f"QDRANT_URL: {os.getenv('QDRANT_URL')}")
    print(f"QDRANT_HOST: {os.getenv('QDRANT_HOST')}")
    print(f"DATABASE_URL: {os.getenv('DATABASE_URL')}")

    uvicorn.run(app, host="0.0.0.0", port=8000)