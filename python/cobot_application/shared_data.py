import asyncio
import json
from pathlib import Path


class Config:
    def __init__(self):
        base = Path(__file__).resolve().parent
        self.path = base / "config.JSON"

        with self.path.open("r", encoding="utf-8") as f:
            self.data = json.load(f)

    def get(self, level, key, default=None):
        return self.data.get(level, {}).get(key, default)

    def as_dict(self):
        return self.data

    def update(self, new_data: dict):
        """Optional: update config at runtime (in memory only)."""
        self.data.update(new_data)

#message class for easy communication between packages
#only ever put this type of message to a queue
class P2P_command():

    def __init__(self,sender_ID,message,value):
        self.sender_ID = sender_ID
        self.message = message
        self.value = value
        
#network for easy communication between controll loops
class Py_network():

    def __init__(self):
        #command queues for each of the running loops
        #SHOULD ONLY TAKE P2P_MESSAGES
        self.camera_commands = asyncio.Queue()
        self.cobot_commands = asyncio.Queue()
        self.web_commands = asyncio.Queue()
        self.vision_commands = asyncio.Queue()

        #states for each module -- ENUMS?
        self.camera_state = "not initialized"
        self.cobot_state = "not initialized"
        self.web_state = "not initialized"
        self.vision_state =  "not initialized"




