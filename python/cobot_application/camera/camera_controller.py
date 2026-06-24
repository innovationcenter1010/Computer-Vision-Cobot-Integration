import asyncio
from cobot_application.camera import astra_cpp
import numpy as np

from python.cobot_application.shared_data import Py_network, Config, package_ID, P2P_command, camera_commands, vision_commands, web_commands
from python.cobot_application.camera.camera_streamer import CameraStream, DualFrame


class Camera_Controller():

    def __init__(self,network: Py_network,configurations):
        
        self.network = network
        self.id = package_ID.camera_id
        self.command_queue = network.command_queues[self.id.value]
        self.configurations = configurations

        self.camera_stream = CameraStream()



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


    async def stream(self):
        while True:
            await self.send_image_to_web()
            await asyncio.sleep(1)



    async def snapshot(self, command: P2P_command):

        outgoing_frame = DualFrame()
        
        outgoing_frame.depth_frame = self.camera_stream.latest_depth
        outgoing_frame.color_frame = self.camera_stream.latest_color


        snapshot_command = P2P_command(
            receiver_ID= command.sender_ID,
            sender_ID=package_ID.camera_id,
            message=vision_commands.convert_to_jpeg,
            value=outgoing_frame
        )

        await self.network.send(snapshot_command)




            


