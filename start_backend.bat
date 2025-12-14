@echo off
echo Starting the RAG Chatbot backend server...

echo Changing to backend directory...
cd /d "%~dp0backend"

echo Activating virtual environment...
call venv\Scripts\activate.bat

echo Installing dependencies if needed...
pip install -r requirements.txt

echo Setting environment variables...
set /p BACKEND_ENV=<"../.backend.env"
for /f "tokens=1,* delims==" %%a in ('findstr /r /c:"^[^#].*=" "../.backend.env"') do (
    set "%%a=%%b"
)

echo Starting FastAPI server...
uvicorn src.api.main:app --host 0.0.0.0 --port 8000 --reload