import asyncio
import json
from pathlib import Path
from enum import Enum,auto



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

    def __init__(self, sender_ID, receiver_ID, message, value=None):
        self.sender_ID = sender_ID
        self.receiver_ID = receiver_ID
        self.message = message
        self.value = value

class sender_ID(Enum):
    cobot_id = auto()
    vision_id = auto()
    camera_id = auto()
    web_id = auto()
        
#network for easy communication between controll loops
class Py_network():

    def __init__(self):
        #command queues for each of the running loops
        #SHOULD ONLY TAKE P2P_MESSAGES

  


        self.command_queues = {
            sender_ID.cobot_id: asyncio.Queue(),
            sender_ID.vision_id: asyncio.Queue(),
            sender_ID.camera_id: asyncio.Queue(),
            sender_ID.web_id: asyncio.Queue(),
        }

        

        #states for each module -- ENUMS?
        self.camera_state = "not initialized"
        self.cobot_state = "not initialized"
        self.web_state = "not initialized"
        self.vision_state =  "not initialized"

    def send(self,command: P2P_command):

        if not command.receiver_ID: raise RuntimeError("no reciver id")

        self.command_queues[command.receiver_ID].put(command)





