import asyncio
import json
from pathlib import Path

import uvicorn
from fastapi import FastAPI, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse
from fastapi.staticfiles import StaticFiles
from fastapi.templating import Jinja2Templates

from cobot_application.shared_data import P2P_command, sender_ID


class Webpage_Controller:
    def __init__(self, network, configurations):
        self.network = network
        self.configurations = configurations

        self.id = sender_ID.web_id

        self.base_dir = Path(__file__).resolve().parent
        self.python_dir = self.base_dir.parent
        self.project_root = self.python_dir.parent

        self.config_path = Path(self.configurations.path)
        self.results_dir = self.find_results_dir()
        self.results_dir.mkdir(parents=True, exist_ok=True)

        self.connected_clients = []

        self.app = FastAPI()
        self.templates = Jinja2Templates(directory=self.base_dir / "templates")

        self.setup_routes()

        print("Webpage controller awake", flush=True)
        print("Web config path:", self.config_path, flush=True)
        print("Web results path:", self.results_dir, flush=True)

    def find_results_dir(self):
        possible_paths = [
            Path("/app/vision_workspace/operation/results"),
            self.project_root.parent / "vision_workspace" / "operation" / "results",
            self.project_root / "vision_workspace" / "operation" / "results",
            self.python_dir / "vision_workspace" / "operation" / "results",
            self.python_dir / "cobot_application" / "vision_system" / "operation" / "results",
        ]

        for path in possible_paths:
            print("Checking results path:", path, flush=True)

            if path.exists():
                print("Using results path:", path, flush=True)
                return path

        print("No existing results path found. Using:", possible_paths[0], flush=True)
        return possible_paths[0]

    async def loop(self):
        print("Starting webpage server...", flush=True)

        config = uvicorn.Config(
            self.app,
            host="0.0.0.0",
            port=8000,
            log_level="info"
        )

        server = uvicorn.Server(config)

        await asyncio.gather(
            server.serve(),
            self.web_message_loop()
        )

    async def web_message_loop(self):
        while True:
            command = await self.network.web_commands.get()

            print(
                "Webpage received reverse message:",
                command.message,
                command.value,
                flush=True
            )

            await self.broadcast_to_clients({
                "type": command.message,
                "sender": str(command.sender_ID),
                "message": command.value
            })

    async def broadcast_to_clients(self, payload):
        disconnected_clients = []

        for websocket in self.connected_clients:
            try:
                await websocket.send_json(payload)
            except Exception:
                disconnected_clients.append(websocket)

        for websocket in disconnected_clients:
            if websocket in self.connected_clients:
                self.connected_clients.remove(websocket)

    async def send_command(self, receiver_ID, message, value=None):
        print("WebpageController received command request", flush=True)
        print("Receiver:", receiver_ID, flush=True)
        print("Message:", message, flush=True)
        print("Value:", value, flush=True)

        command = P2P_command(
            sender_ID=self.id,
            receiver_ID=receiver_ID,
            message=message,
            value=value
        )

        if receiver_ID == sender_ID.cobot_id:
            print("Putting command into cobot queue", flush=True)
            await self.network.cobot_commands.put(command)
            return {"ok": True, "message": f"Sent '{message}' to cobot"}

        if receiver_ID == sender_ID.vision_id:
            print("Putting command into vision queue", flush=True)
            await self.network.vision_commands.put(command)
            return {"ok": True, "message": f"Sent '{message}' to vision system"}

        if receiver_ID == sender_ID.camera_id:
            print("Putting command into camera queue", flush=True)
            await self.network.camera_commands.put(command)
            return {"ok": True, "message": f"Sent '{message}' to camera"}

        return {"ok": False, "message": "Unknown receiver ID"}

    async def handle_web_command(self, command, value=None):
        print("Web command received:", command, flush=True)

        if command == "stop_robot":
            return await self.send_command(
                receiver_ID=sender_ID.cobot_id,
                message="stop_robot",
                value=value
            )

        if command == "start_robot":
            return await self.send_command(
                receiver_ID=sender_ID.vision_id,
                message="start_vision",
                value=value
            )

        if command == "calibrate_robot":
            return await self.send_command(
                receiver_ID=sender_ID.cobot_id,
                message="calibrate_robot",
                value=value
            )

        if command == "set_cobot_speed":
            return await self.send_command(
                receiver_ID=sender_ID.cobot_id,
                message="set_cobot_speed",
                value=value
            )

        if command == "capture_image":
            return await self.send_command(
                receiver_ID=sender_ID.camera_id,
                message="capture_image",
                value=value
            )

        if command == "run_detection":
            return await self.send_command(
                receiver_ID=sender_ID.vision_id,
                message="run_detection",
                value=value
            )

        return {
            "ok": False,
            "message": f"Unknown command: {command}"
        }

    def setup_routes(self):
        self.app.mount(
            "/static",
            StaticFiles(directory=self.base_dir / "static"),
            name="static"
        )

        self.app.mount(
            "/results",
            StaticFiles(directory=self.results_dir),
            name="results"
        )

        @self.app.get("/", response_class=HTMLResponse)
        def home(request: Request):
            return self.templates.TemplateResponse(
                request=request,
                name="index.html"
            )

        @self.app.get("/api/config")
        def get_config():
            with open(self.config_path, "r", encoding="utf-8") as f:
                return json.load(f)

        @self.app.post("/api/config")
        def save_config(config: dict):
            print("Saving config to:", self.config_path, flush=True)
            print("New config:", config, flush=True)

            with open(self.config_path, "w", encoding="utf-8") as f:
                json.dump(config, f, indent=4)

            self.configurations.update(config)

            return {
                "status": "saved",
                "saved_to": str(self.config_path),
                "config": config
            }

        @self.app.get("/api/latest-image")
        def latest_image():
            image_files = (
                list(self.results_dir.glob("*.jpg")) +
                list(self.results_dir.glob("*.jpeg")) +
                list(self.results_dir.glob("*.png"))
            )

            print("Looking for images in:", self.results_dir, flush=True)
            print("Images found:", [p.name for p in image_files], flush=True)

            if not image_files:
                return {"image": None}

            newest = max(image_files, key=lambda p: p.stat().st_mtime)

            print("Newest image selected:", newest.name, flush=True)

            return {
                "image": f"/results/{newest.name}",
                "filename": newest.name
            }

        @self.app.websocket("/ws")
        async def websocket_endpoint(websocket: WebSocket):
            await websocket.accept()
            self.connected_clients.append(websocket)

            print("WebSocket connected", flush=True)

            await websocket.send_json({
                "type": "connection",
                "message": "WebSocket connected"
            })

            try:
                while True:
                    data = await websocket.receive_json()
                    print("WebSocket data:", data, flush=True)

                    command = data.get("command")
                    value = data.get("value")

                    result = await self.handle_web_command(command, value)

                    await websocket.send_json({
                        "type": "command_result",
                        "command": command,
                        "result": result
                    })

            except WebSocketDisconnect:
                print("WebSocket disconnected", flush=True)

            finally:
                if websocket in self.connected_clients:
                    self.connected_clients.remove(websocket)
