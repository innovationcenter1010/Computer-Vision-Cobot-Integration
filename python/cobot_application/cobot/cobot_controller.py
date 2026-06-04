from cobot import Cobot
import asyncio
from shared_data import Py_network, Config


class Cobot_Controller():
    def __init__(self,network,configurations):
        self.network = network
        self.configurations = configurations
        cobot = Cobot(configurations.get("cobot_ip"))
        print("hi")
        
    async def loop(self):
        while True: 
            print("cobot looping")





