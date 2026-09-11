# NXP Cup Tuning Application

This is a full-stack tuning system for the NXP Cup race car simulation.

## Components
1. **API Server (`api_server.py`)**: A FastAPI server providing a REST API and WebSocket connection for real-time simulation metrics.
2. **Dashboard (`static/index.html`)**: A single-page Vanilla HTML/JS/CSS web dashboard to visualize the track, monitor metrics, and interactively tune config parameters.
3. **AI Agent (`ai_agent.py`)**: An autonomous agent powered by Gemini 2.0 Flash that automatically discovers the best configuration parameters by running headless simulations and observing the metrics.

## Setup

Install the required Python packages:
```bash
pip install -r requirements.txt
```

Set your Gemini API key (optional, required only for the AI tuning agent):
```bash
export GEMINI_API_KEY="your-api-key-here"
```

## Running

Start the server using the provided shell script:
```bash
./start.sh
```

Then, open your web browser and navigate to:
[http://localhost:8000](http://localhost:8000)

## API Endpoints

- `GET /config`: Get the baseline `config.py` constants merged with the active overrides.
- `POST /config`: Accept a partial JSON dict, update the in-memory overrides, and return the newly merged config.
- `POST /config/reset`: Reset all active overrides to defaults.
- `POST /simulate`: Run N frames headlessly (default: 300) and return metrics JSON.
- `GET /metrics`: Return the latest metrics from the last simulate run.
- `GET /track`: Return the track layout points (left boundary, right boundary, centerline).
- `GET /ai/tune`: Start the autonomous AI tuning loop and receive Server-Sent Events (SSE) representing progress and iterations.
- `WS /ws`: Stream real-time frames containing car state and diagnostics.
