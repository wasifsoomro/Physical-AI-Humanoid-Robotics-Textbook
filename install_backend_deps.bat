#!/bin/bash
# Install backend dependencies with PostgreSQL support for Windows

echo "Installing backend dependencies..."

# Navigate to backend directory
cd backend

# Install dependencies one by one to handle issues better
pip install fastapi==0.104.1
pip install uvicorn[standard]==0.24.0
pip install pydantic==2.5.0
pip install pydantic-settings==2.1.0
pip install sqlalchemy==2.0.23
pip install "asyncpg==0.29.0"
pip install qdrant-client==1.8.0
pip install cohere==5.5.3
pip install python-dotenv==1.0.0
pip install pyjwt==2.8.0
pip install "passlib[bcrypt]==1.7.4"
pip install python-multipart==0.0.6
pip install alembic==1.13.1
pip install uuid7==0.1.0

echo "Dependencies installed successfully!"