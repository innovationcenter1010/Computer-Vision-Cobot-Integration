import asyncio
from cobot_application.shared_data import Py_network, Config, sender_ID, P2P_command


class Vision_Controller():
    def __init__(self,network,configurations):
        self.network = network
        self.command_queue = network.vision_commands
        self.configurations = configurations
        self.id = sender_ID.vision_id
        print("vision system awake",flush=True)

    async def loop(self):
        while True:

            layer_command = P2P_command(self.id,"pickup",5)
            await self.command_queue.put(layer_command)
            #print("computer vision functions",flush=True)
            await asyncio.sleep(1000)
