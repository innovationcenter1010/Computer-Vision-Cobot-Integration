import asyncio
from cobot_application.camera import astra_cpp
import numpy as np

from python.cobot_application.shared_data import Py_network, Config, package_ID, P2P_command, camera_commands, vision_commands
from python.cobot_application.camera.preprocessing import rgbdframe_to_numpy

class Camera_Controller():

    def __init__(self,network: Py_network,configurations):
        
        self.network = network
        self.id = package_ID.camera_id
        self.command_queue = network.command_queues[self.id.value]
        self.configurations = configurations

        print("starting camera initialization",flush=True)
        self.camera = astra_cpp.AstraCamera()
        self.camera.start()


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

        print("taking snapshot",flush=True)

        new_snapshot = astra_cpp.RGBDFrame()

        new_snapshot_result = self.camera.get_frame()

        new_snapshot = new_snapshot_result.snapshot

        outgoing_snapshot = rgbdframe_to_numpy(new_snapshot)

        vision_command = P2P_command(
            receiver_ID=package_ID.vision_id,
            sender_ID=package_ID.camera_id,
            message=vision_commands.detect,
            value=outgoing_snapshot
        )

        await self.network.send(vision_command)


            


