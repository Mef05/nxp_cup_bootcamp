#!/bin/bash
cd "$(dirname "$0")"
pip install -r requirements.txt -q
echo "Starting tuning server at http://localhost:8000"
uvicorn api_server:app --reload --port 8000
