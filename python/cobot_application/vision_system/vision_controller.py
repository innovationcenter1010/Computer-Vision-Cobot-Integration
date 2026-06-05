import asyncio
from cobot_application.shared_data import Py_network, Config


class Vision_Controller():
    def __init__(self,network,configurations):
        self.network = network
        self.command_queue = network.vision_commands
        self.configurations = configurations
        print("vision system awake",flush=True)

    async def loop(self):
        while True:
            print(await self.command_queue.get(),flush= True)
            #print("computer vision functions",flush=True)
            await asyncio.sleep(1)