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



class package_ID(Enum):
    cobot_id = auto()
    vision_id = auto()
    camera_id = auto()
    web_id = auto()

## library for commands that can be sent to each system
class vision_commands(Enum):
    detect = 1
    convert_to_jpeg = 2

class cobot_commands(Enum):
    pick_layer = auto()
    start = auto()
    calibrate = auto()
    stop = auto()

class camera_commands(Enum):
    snapshot = auto()
    get_stream_provider = auto()

class web_commands(Enum):
    refresh = auto()
    update_image = auto()

COMMAND_REGISTRY = {
    package_ID.vision_id.value: vision_commands,
    package_ID.cobot_id.value: cobot_commands,
    package_ID.camera_id.value: camera_commands,
    package_ID.web_id.value: web_commands,
}


#message class for sending commabds between modules
class P2P_command():

    def __init__(self, sender_ID: package_ID, 
                 receiver_ID: package_ID, 
                 message: vision_commands | cobot_commands | camera_commands | web_commands, 
                 value=None):
        

        self.sender_ID = sender_ID
        self.receiver_ID = receiver_ID
        self.message = message
        self.value = value

        
#network for easy communication between controll loops
class Py_network():

    def __init__(self):
        #command queues for each of the running loops
        #SHOULD ONLY TAKE P2P_MESSAGES


        self.command_queues = {
            package_ID.cobot_id.value: asyncio.Queue(),
            package_ID.vision_id.value: asyncio.Queue(),
            package_ID.camera_id.value: asyncio.Queue(),
            package_ID.web_id.value: asyncio.Queue(),
        }


    
        #states for each module -- ENUMS?
        self.camera_state = "not initialized"
        self.cobot_state = "not initialized"
        self.web_state = "not initialized"
        self.vision_state =  "not initialized"

        

    async def send(self,command: P2P_command):

        if not command.receiver_ID: raise RuntimeError("no reciver id")

        print("sending command",flush=True)
        await self.command_queues[command.receiver_ID.value].put(command)





