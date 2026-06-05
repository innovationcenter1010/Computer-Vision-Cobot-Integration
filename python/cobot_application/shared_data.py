import asyncio
import json
from pathlib import Path


class Config:
    def __init__(self):
        base = Path(__file__).resolve().parent
        self.path = base / "config.JSON"

        with self.path.open("r", encoding="utf-8") as f:
            self.data = json.load(f)

    def get(self, key, default=None):
        return self.data.get(key, default)

    def __getitem__(self, key):
        return self.data[key]

    def __contains__(self, key):
        return key in self.data

    def as_dict(self):
        return self.data

    def update(self, new_data: dict):
        """Optional: update config at runtime (in memory only)."""
        self.data.update(new_data)

class Py_network():

    def __init__(self):
        self.camera_commands = asyncio.Queue()
        self.cobot_commands = asyncio.Queue()
        self.web_commands = asyncio.Queue()
        self.vision_commands = asyncio.Queue()

        self.camera_state = "not initialized"
        self.cobot_state = "not initialized"
        self.web_state = "not initialized"
        self.vision_state =  "not initialized"




