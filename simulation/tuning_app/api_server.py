import sys
import os
import math
import asyncio
import json
from typing import Dict, Any, List

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel

# Add parent directory to path so we can import simulation modules
sys.path.insert(0, os.path.abspath('..'))
import config
from simulate import SimulationRunner

app = FastAPI()
app.mount("/static", StaticFiles(directory="static"), name="static")

try:
    import ai_agent
    app.include_router(ai_agent.router)
except Exception as e:
    print(f"Warning: Failed to load ai_agent: {e}")

@app.get("/")
def serve_index():
    return FileResponse("static/index.html")

CURRENT_OVERRIDES = {}
LATEST_METRICS = {}

def get_full_config():
    base_config = {k: v for k, v in vars(config).items() if k.isupper() and not k.startswith('_')}
    # Make sure we merge current overrides
    for k, v in CURRENT_OVERRIDES.items():
        base_config[k] = v
    return base_config

@app.get("/config")
def get_config():
    return get_full_config()

@app.post("/config")
def update_config(updates: Dict[str, Any]):
    CURRENT_OVERRIDES.update(updates)
    return get_full_config()

@app.post("/config/reset")
def reset_config():
    CURRENT_OVERRIDES.clear()
    return get_full_config()

@app.get("/metrics")
def get_metrics():
    return LATEST_METRICS

@app.get("/track")
def get_track():
    from track import get_track_boundaries, get_track_centerline
    lb, rb = get_track_boundaries()
    center = get_track_centerline()
    return {"left": lb, "right": rb, "center": center}

# Websocket manager
class ConnectionManager:
    def __init__(self):
        self.active_connections: List[WebSocket] = []

    async def connect(self, websocket: WebSocket):
        await websocket.accept()
        self.active_connections.append(websocket)

    def disconnect(self, websocket: WebSocket):
        if websocket in self.active_connections:
            self.active_connections.remove(websocket)

    async def broadcast(self, message: str):
        for connection in list(self.active_connections):
            try:
                await connection.send_text(message)
            except Exception:
                self.disconnect(connection)

manager = ConnectionManager()

@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await manager.connect(websocket)
    try:
        while True:
            await websocket.receive_text()
    except WebSocketDisconnect:
        manager.disconnect(websocket)

class SimulateRequest(BaseModel):
    frames: int = 300

@app.post("/simulate")
async def simulate(req: SimulateRequest = None):
    # Default to 1200 frames (20 seconds) if not specified or 0
    max_frames = req.frames if (req and req.frames > 0) else 1200
    
    # We must patch the config before creating runner to ensure everything matches
    full_config = get_full_config()
    sim = SimulationRunner(config_overrides=full_config)
    
    max_cte = 0.0
    sum_cte = 0.0
    time_reversed = 0
    off_track = False
    lap_completed = False
    
    # Track width is 0.55m -> half width is 0.275m
    TRACK_HALF_WIDTH = 0.275
    
    start_x = sim.car.x
    start_y = sim.car.y
    has_left_start_area = False
    
    actual_frames = 0
    for i in range(max_frames):
        actual_frames += 1
        sim.step()
        
        # calculate metrics
        cte = sim.last_debug.get('cte', 0.0)
        abs_cte = abs(cte)
        max_cte = max(max_cte, abs_cte)
        sum_cte += abs_cte
        
        world_cte = abs(sim.hist_cte[-1]) if len(sim.hist_cte) > 0 else 0
        if world_cte > TRACK_HALF_WIDTH:
            off_track = True
            
        if sim.last_debug.get('speed_L', 0) < 0 or sim.last_debug.get('speed_R', 0) < 0:
            time_reversed += 1
            
        # check lap completion
        dist_from_start = math.hypot(sim.car.x - start_x, sim.car.y - start_y)
        if dist_from_start > 1.5:
            has_left_start_area = True
        elif has_left_start_area and dist_from_start < 0.3:
            lap_completed = True
            break
            
        # check failure
        if off_track:
            break
            
        # broadcast via WS
        ws_data = {
            "frame": sim.frame,
            "x": sim.car.x,
            "y": sim.car.y,
            "theta": sim.car.theta,
            "steer": sim.controller.current_steer,
            "cte": cte,
            "prox": sim.last_debug.get('proximity', 0.0)
        }
        await manager.broadcast(json.dumps(ws_data))
        # Yield to event loop so WS can send
        await asyncio.sleep(0.001)
        
    mean_cte = sum_cte / actual_frames if actual_frames > 0 else 0
    score = 100.0 - (2.0 * mean_cte) - (3.0 * max_cte) - (5.0 * time_reversed) - (50.0 if off_track else 0.0) + (20.0 if lap_completed else 0.0)
    
    global LATEST_METRICS
    LATEST_METRICS = {
        "frames": actual_frames,
        "finished": True,
        "off_track": off_track,
        "max_cte_px": max_cte,
        "mean_cte_px": mean_cte,
        "oscillation_score": 0.0,
        "time_reversed": time_reversed,
        "lap_completed": lap_completed,
        "score": score
    }
    
    return LATEST_METRICS
