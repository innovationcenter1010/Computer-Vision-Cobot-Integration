import asyncio
from python.cobot_application.shared_data import Py_network, Config, sender_ID, P2P_command

class Camera_Controller():
    def __init__(self,network,configurations):
        self.network = network
        self.command_queue = network.camera_commands
        self.configurations = configurations
        self.id = sender_ID.camera_id
        print("camera awake",flush=True)

    async def loop(self):
        while True:

            pickup = [create_fake_can(0,-925,500,0)]

            layer_command = P2P_command(self.id,"pickup",pickup)
            await self.network.cobot_commands.put(layer_command)
            #print("computer vision functions",flush=True)
            await asyncio.sleep(1000)


