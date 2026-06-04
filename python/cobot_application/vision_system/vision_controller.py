import asyncio
from cobot_application.shared_data import Py_network, Config


class Vision_Controller():
    def __init__(self,network,configurations):
        self.network = network
        self.configurations = configurations
        print("vision system awake")

    async def loop():
        print("computer vision functions")
        await asyncio.sleep(0.2)