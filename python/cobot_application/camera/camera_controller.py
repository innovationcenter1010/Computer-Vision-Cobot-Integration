import asyncio
import astra_cpp

from python.cobot_application.shared_data import Py_network, Config, package_ID, P2P_command, camera_commands
from python.cobot_application.camera.camera import OrbecCamera, orbec_snapshot 

class Camera_Controller():
    def __init__(self,network: Py_network,configurations):
        self.network = network
        self.id = package_ID.camera_id
        self.command_queue = network.command_queues[self.id.value]
        self.configurations = configurations

        self.camera = OrbecCamera()
        self.camera.connect()

        # camera command map
        self.command_map = {

            camera_commands.snapshot: self.snapshot

        }

        print("camera awake",flush=True)

    async def loop(self):
        while True:

            #wait for a command to come in
            command: P2P_command = await self.command_queue.get()

            #map the command from the command map to run the actual function
            handle = self.command_map.get(command.message)
            
            #only run the command if it is properly mapped
            if handle: await handle(command)

            else: raise RuntimeError("Unknown command:", command.message)



    async def snapshot(self, command):

        new_snapshot = astra_cpp.RGBBFrame()
            


