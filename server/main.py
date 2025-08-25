from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
import datetime
import random

app = FastAPI()

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # Allow all origins for development purposes
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

class StartRequest(BaseModel):
    components: list[str]
    model: str

class ConfigResponse(BaseModel):
    components: list[str]
    models: list[str]

@app.get("/bots")
async def get():
    return ConfigResponse(
        components=["Component1", "Component2", "Component3"],
        models=["ModelName", "AnotherModel", "YetAnotherModel"]
    )

@app.post("bot/start")
async def start(req: StartRequest):
    print(f"Received start request: {req}")
    return {"status": "success", "message": "Start request received", "components": req.components, "model": req.model}

@app.websocket("bot/plot")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    try:
        position = {"x": 0, "y": 0, "z": 0}
        while True:
            x_min, x_max = (-5184, 5184)
            y_min, y_max = (-4096, 4096)
            z_min, z_max = (0, 2044)

            deltax = random.uniform(-2, 2)
            deltay = random.uniform(-2, 2)
            deltaz = random.uniform(-2, 2)


            position["x"] = max(min(position["x"] + deltax, x_max), x_min)
            position["y"] = max(min(position["y"] + deltay, y_max), y_min)
            position["z"] = max(min(position["z"] + deltaz, z_max), z_min)
            position["timestamp"] = datetime.now().isoformat()

            await websocket.send_json(position)
    except WebSocketDisconnect:
        print("Client disconnected")
    except Exception as e:
        print(f"Error: {e}")

