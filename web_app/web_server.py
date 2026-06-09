from fastapi import FastAPI, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse
from fastapi.staticfiles import StaticFiles
from fastapi.templating import Jinja2Templates
from robot_controller import RobotController
from pathlib import Path
import json

app = FastAPI()
robot_controller = RobotController()

BASE_DIR = Path(__file__).resolve().parent
CONFIG_PATH = BASE_DIR / "web_config.json"
RESULTS_DIR = BASE_DIR.parent / "vision_workspace" / "operation" / "results"

app.mount("/static", StaticFiles(directory=BASE_DIR / "static"), name="static")
app.mount("/results", StaticFiles(directory=RESULTS_DIR), name="results")

templates = Jinja2Templates(directory=BASE_DIR / "templates")


@app.get("/", response_class=HTMLResponse)
def home(request: Request):
    return templates.TemplateResponse(
        request=request,
        name="index.html"
    )


@app.get("/api/config")
def get_config():
    with open(CONFIG_PATH, "r") as f:
        return json.load(f)


@app.post("/api/config")
def save_config(config: dict):
    print("Saving config to:", CONFIG_PATH)
    print("New config:", config)

    with open(CONFIG_PATH, "w") as f:
        json.dump(config, f, indent=4)

    return {"status": "saved"}


@app.get("/api/latest-image")
def latest_image():
    image_files = (
        list(RESULTS_DIR.glob("*.jpg")) +
        list(RESULTS_DIR.glob("*.jpeg")) +
        list(RESULTS_DIR.glob("*.png"))
    )

    if not image_files:
        return {"image": None}

    newest = max(image_files, key=lambda p: p.stat().st_mtime)
    return {"image": f"/results/{newest.name}"}


# ADD WEBSOCKET CODE BELOW HERE

connected_clients = []


async def broadcast(message: dict):
    disconnected = []

    for client in connected_clients:
        try:
            await client.send_json(message)
        except Exception:
            disconnected.append(client)

    for client in disconnected:
        connected_clients.remove(client)


def handle_robot_command(command: str):
    print("Robot command received:", command)

    if command == "connect_robot":
        return robot.connect()

    if command == "start_robot":
        return robot.start_robot()

    if command == "stop_robot":
        return robot.stop_robot()

    if command == "calibrate_robot":
        return robot.calibrate_robot()

    return {"ok": False, "message": f"Unknown command: {command}"}

@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    connected_clients.append(websocket)

    await websocket.send_json({
        "type": "connection",
        "message": "WebSocket connected"
    })

    try:
        while True:
            data = await websocket.receive_json()
            print("WebSocket data:", data)

            command = data.get("command")
            result = handle_robot_command(command)

            await websocket.send_json({
                "type": "command_result",
                "command": command,
                "result": result
            })

    except WebSocketDisconnect:
        connected_clients.remove(websocket)