import os
import json
import asyncio
import urllib.request
import urllib.error
from fastapi import APIRouter
from fastapi.responses import StreamingResponse

try:
    from google import genai
    from google.genai import types
except ImportError:
    genai = None

router = APIRouter()

def run_simulation_tool(config_overrides: dict) -> dict:
    """Run simulation with specific config overrides and return metrics."""
    # First set the config overrides
    req = urllib.request.Request("http://127.0.0.1:8000/config", data=json.dumps(config_overrides).encode('utf-8'), headers={'Content-Type': 'application/json'})
    urllib.request.urlopen(req)
    # Then simulate
    req = urllib.request.Request("http://127.0.0.1:8000/simulate", data=json.dumps({"frames": 0}).encode('utf-8'), headers={'Content-Type': 'application/json'})
    with urllib.request.urlopen(req) as response:
        return json.loads(response.read().decode('utf-8'))

def get_current_config_tool() -> dict:
    """Get the current configuration parameters."""
    with urllib.request.urlopen("http://127.0.0.1:8000/config") as response:
        return json.loads(response.read().decode('utf-8'))

def set_config_tool(params: dict) -> dict:
    """Set the configuration parameters."""
    req = urllib.request.Request("http://127.0.0.1:8000/config", data=json.dumps(params).encode('utf-8'), headers={'Content-Type': 'application/json'})
    with urllib.request.urlopen(req) as response:
        return json.loads(response.read().decode('utf-8'))

def reset_config_tool() -> dict:
    """Reset configuration to default baseline."""
    req = urllib.request.Request("http://127.0.0.1:8000/config/reset", data=b'', method='POST')
    with urllib.request.urlopen(req) as response:
        return json.loads(response.read().decode('utf-8'))

# Provide descriptions to the AI
tools = [
    {
        "name": "run_simulation",
        "description": "Run the simulation for 300 frames with the provided config_overrides dict and return the metrics. Use this to test parameters.",
        "parameters": {
            "type": "OBJECT",
            "properties": {
                "config_overrides": {
                    "type": "OBJECT",
                    "description": "Dictionary of parameter names and their new values"
                }
            },
            "required": ["config_overrides"]
        }
    },
    {
        "name": "get_current_config",
        "description": "Get the current configuration parameters and their values.",
        "parameters": {
            "type": "OBJECT",
            "properties": {}
        }
    },
    {
        "name": "set_config",
        "description": "Apply configuration parameters globally.",
        "parameters": {
            "type": "OBJECT",
            "properties": {
                "params": {
                    "type": "OBJECT",
                    "description": "Dictionary of parameter names and their new values"
                }
            },
            "required": ["params"]
        }
    },
    {
        "name": "reset_config",
        "description": "Reset configuration to baseline defaults.",
        "parameters": {
            "type": "OBJECT",
            "properties": {}
        }
    }
]

system_prompt = """You are an autonomous AI tuning agent for the NXP Cup race car simulation.
Your goal is to maximize the simulation score.
Score Formula: 100 - 2*mean_cte - 3*max_cte - 5*time_reversed - 50*(off_track). Higher = better.
You have tools to get_current_config, set_config, reset_config, and run_simulation.
The parameters you can tune include:
- STEER_KP (0.0-15.0): proportional gain
- STEER_KP_Q (0.0-1.0): quadratic gain
- STEER_KD (0.0-2.0): derivative gain
- STEERING_ALPHA (0.0-1.0): steering smoothing
- LOOKAHEAD_FACTOR (0.0-1.0): lookahead distance weight
- SPEED_RIGHT, SPEED_LEFT (0-100): motor speeds
- DIFFERENTIAL_FACTOR (0.0-1.0): inner-wheel reduction when steering
Iteratively tune the parameters using run_simulation. Run up to 10-20 iterations. 
Try to find a setup that avoids going off_track (which carries a -50 penalty), 
keeps mean_cte and max_cte low, and maximizes the final score.
When you are done, describe your best config and end your response.
"""

async def tune_generator():
    api_key = os.environ.get("GEMINI_API_KEY")
    if not api_key:
        yield f"data: {json.dumps({'status': 'error', 'message': 'GEMINI_API_KEY environment variable is not set'})}\n\n"
        return
        
    if not genai:
        yield f"data: {json.dumps({'status': 'error', 'message': 'google-genai package is not installed'})}\n\n"
        return

    try:
        client = genai.Client(api_key=api_key)
        
        # Reset and get baseline
        reset_config_tool()
        baseline_metrics = run_simulation_tool({})
        best_score = baseline_metrics.get("score", 0)
        
        yield f"data: {json.dumps({'status': 'progress', 'message': f'Baseline score: {best_score:.2f}'})}\n\n"
        
        chat = client.chats.create(
            model="gemini-2.0-flash",
            config=types.GenerateContentConfig(
                system_instruction=system_prompt,
                temperature=0.7,
                tools=tools
            )
        )
        
        yield f"data: {json.dumps({'status': 'progress', 'message': 'Starting tuning iterations...'})}\n\n"
        
        response = chat.send_message("Please begin tuning the car to maximize the score. Return only when you have found the best possible parameters.")
        
        iterations = 0
        while iterations < 20:
            iterations += 1
            if response.function_calls:
                for fc in response.function_calls:
                    yield f"data: {json.dumps({'status': 'progress', 'message': f'Tool Call: {fc.name}'})}\n\n"
                    
                    try:
                        args = fc.args if hasattr(fc, 'args') else {}
                        
                        if fc.name == "run_simulation":
                            result = run_simulation_tool(args.get("config_overrides", {}))
                            score = result.get("score", 0)
                            if score > best_score:
                                best_score = score
                                yield f"data: {json.dumps({'status': 'progress', 'message': f'New best score: {best_score:.2f}'})}\n\n"
                        elif fc.name == "get_current_config":
                            result = get_current_config_tool()
                        elif fc.name == "set_config":
                            result = set_config_tool(args.get("params", {}))
                        elif fc.name == "reset_config":
                            result = reset_config_tool()
                        else:
                            result = {"error": f"Unknown function {fc.name}"}
                            
                        # yield progress back to LLM
                        response = chat.send_message(
                            types.Part.from_function_response(
                                name=fc.name,
                                response={"result": result}
                            )
                        )
                    except Exception as e:
                        yield f"data: {json.dumps({'status': 'progress', 'message': f'Tool error: {str(e)}'})}\n\n"
                        response = chat.send_message(
                            types.Part.from_function_response(
                                name=fc.name,
                                response={"error": str(e)}
                            )
                        )
            else:
                yield f"data: {json.dumps({'status': 'progress', 'message': 'AI finished tuning.'})}\n\n"
                break
                
        yield f"data: {json.dumps({'status': 'done', 'best_score': best_score})}\n\n"
        
    except Exception as e:
        yield f"data: {json.dumps({'status': 'error', 'message': str(e)})}\n\n"

@router.get("/ai/tune")
async def ai_tune():
    return StreamingResponse(tune_generator(), media_type="text/event-stream")
