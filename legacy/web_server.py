from fastapi import FastAPI, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse
from fastapi.staticfiles import StaticFiles
from fastapi.templating import Jinja2Templates
from pathlib import Path
import json

from python.cobot_application.shared_data import package_ID,vision_commands,cobot_commands,camera_commands
#from .webpage_controller import Webpage_Controller


#-------- maps for type conversions ----------

receiver_map = {
    "COBOT": package_ID.cobot_id,
    "VISION": package_ID.vision_id,
    "CAMERA": package_ID.camera_id,
}



BASE_DIR = Path(__file__).resolve().parent
PYTHON_DIR = BASE_DIR.parent
PROJECT_ROOT = PYTHON_DIR.parent


def find_results_dir():
    possible_paths = [
        PYTHON_DIR / "vision_workspace" / "operation" / "results",
        PROJECT_ROOT / "vision_workspace" / "operation" / "results",
        PYTHON_DIR / "cobot_application" / "vision_system" / "operation" / "results",
    ]

    for path in possible_paths:
        if path.exists():
            print("Using results folder:", path, flush=True)
            return path

    fallback = possible_paths[0]
    print("WARNING: No results folder found. Using fallback:", fallback, flush=True)
    return fallback


RESULTS_DIR = find_results_dir()


def create_web_app(network, configurations):
    app = FastAPI()

    #web_controller = Webpage_Controller(network)
    config_path = Path(configurations.path)

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
        with open(config_path, "r", encoding="utf-8") as f:
            return json.load(f)

    @app.post("/api/config")
    def save_config(config: dict):
        print("Saving config to:", config_path, flush=True)
        print("New config:", config, flush=True)

        with open(config_path, "w", encoding="utf-8") as f:
            json.dump(config, f, indent=4)

        configurations.update(config)

        return {
            "status": "saved",
            "saved_to": str(config_path),
            "config": config
        }

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

    async def handle_web_command(reciver: str, command: str, value = None):
        print("Web command received:", command, flush=True)



        return {
            "ok": False,
            "message": f"Unknown command: {command}"
        }

    @app.websocket("/ws")
    async def websocket_endpoint(websocket: WebSocket):
        await websocket.accept()

        print("WebSocket connected", flush=True)

        await websocket.send_json({
            "type": "connection",
            "message": "WebSocket connected"
        })

        try:
            while True:
                data = await websocket.receive_json()
                print("WebSocket data:", data, flush=True)

                reciver = data.get("reciver")
                command = data.get("command")
                payload = data.get("payload")

                result = await handle_web_command(reciver,command,payload)

                await websocket.send_json({
                    "type": "command_result",
                    "command": command,
                    "result": result
                })

        except WebSocketDisconnect:
            print("WebSocket disconnected", flush=True)

    return app