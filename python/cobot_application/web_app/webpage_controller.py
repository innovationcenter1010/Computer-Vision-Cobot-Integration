import asyncio
import json
from pathlib import Path
import base64

import uvicorn
from fastapi import FastAPI, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse
from fastapi.staticfiles import StaticFiles
from fastapi.templating import Jinja2Templates

from python.cobot_application.shared_data import P2P_command, package_ID, Py_network, COMMAND_REGISTRY, web_commands



#-------- maps for type conversions from js ----------

receiver_map = {
    "COBOT": package_ID.cobot_id,
    "VISION": package_ID.vision_id,
    "CAMERA": package_ID.camera_id,
}

COMMAND_TRANSLATOR = {
    package_ID.vision_id.value: {
        "detect": COMMAND_REGISTRY[package_ID.vision_id.value].detect,
    },

    package_ID.cobot_id.value: {
        "pickup": COMMAND_REGISTRY[package_ID.cobot_id.value].pick_layer,
        "start": COMMAND_REGISTRY[package_ID.cobot_id.value].start,
        "calibrate": COMMAND_REGISTRY[package_ID.cobot_id.value].calibrate,
        "stop": COMMAND_REGISTRY[package_ID.cobot_id.value].stop,
    },

    package_ID.camera_id.value: {
        "snapshot": COMMAND_REGISTRY[package_ID.camera_id.value].snapshot,
    },

}

#decodes strings from js into enum values in python 
def decode_command(receiver_id: str, command_str: str):

    #print("receiver.value =", receiver.value,flush=True)
    #print("registry keys =", COMMAND_TRANSLATOR.keys(),flush=True)
    #print("command =", command_str,flush=True)

    receiver = None
    command_enum =None

    try:
        receiver = receiver_map[receiver_id]
    except KeyError:
        raise ValueError(
            f"Invalid receiver '{receiver_id}'"
        )


    try:
        command_enum = COMMAND_TRANSLATOR[receiver.value][command_str]
    except KeyError:
        raise ValueError(
            f"Invalid command '{command_str}' for receiver '{receiver_id}'"
        )

    return receiver, command_enum

#parses the payload into a number is it matches
def parse_payload(value):
    if value is None:
        return None

    if isinstance(value, (int, float)):
        return value

    if isinstance(value, str):
        value = value.strip()

        # try int first
        try:
            return int(value)
        except ValueError:
            pass

        # then float
        try:
            return float(value)
        except ValueError:
            pass

    # fallback: return original if it can't be parsed
    return value

# -------------------- actual controller --------------------------


class Webpage_Controller:
    def __init__(self, network: Py_network, configurations):

        self.id = package_ID.web_id
        self.network = network
        self.configurations = configurations
        self.command_queue = network.command_queues[self.id.value]

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


        self.command_map = {
            web_commands.update_image: self.update_image
        }


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

        return results_path
    
    async def run_server(self):
        print("Starting webpage server...", flush=True)

        config = uvicorn.Config(
            self.app,
            host="0.0.0.0",
            port=8000,
            log_level="info"
        )

        server = uvicorn.Server(config)
        await server.serve() 

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

    async def loop(self):
        print("Vision waiting for command...", flush=True)

        #wait for a command to come in
        command: P2P_command = await self.command_queue.get()

        #map the command from the command map to run the actual function
        handle = self.command_map.get(command.message)
        
        #only run the command if it is properly mapped
        if handle: await handle(command)

        else: raise RuntimeError("Unknown command:", command.message)

    async def update_image(self, command: P2P_command):

        rgb_bytes = command.value["rgb"]
        depth_bytes = command.value["depth"]

        message = {
            "type": "camera_frame",
            "rgb": base64.b64encode(rgb_bytes).decode("ascii"),
            "depth": base64.b64encode(depth_bytes).decode("ascii"),
        }

        disconnected = []

        for websocket in self.websockets:

            try:
                await websocket.send_json(message)

            except Exception as e:

                print(
                    f"Failed to send image to websocket: {e}",
                    flush=True
                )

                disconnected.append(websocket)

        for websocket in disconnected:
            self.websockets.discard(websocket)



    async def handle_web_command(self, receiver: str,command: str,payload: str):

        receiver_id, command_id = decode_command(receiver,command)
        parsed_payload = parse_payload(payload)

        outgoing_command = P2P_command(
            sender_ID=self.id,
            receiver_ID=receiver_id,
            message=command_id,
            value=parsed_payload
        )

        await self.network.send(outgoing_command)


        return {"result": None,
                "message": command}
        

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

                    receiver = data.get("receiver")
                    command = data.get("command")
                    payload = data.get("payload")

                    print("handling", data,flush=True)

                    try:
                        result = await self.handle_web_command(receiver, command, payload)
                        print(data,"done", flush=True)
                    except Exception as e:
                        print("WEB COMMAND ERROR:", e)
                        result = {"result": str(e),
                      "message": "websocket command failed"}

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
