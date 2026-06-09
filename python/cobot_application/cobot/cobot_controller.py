
import asyncio

from cobot_application.cobot.cobot import Cobot
from cobot_application.shared_data import Py_network, Config, sender_ID, P2P_command

#cobot controller class for initializing and looping cobot functionality
class Cobot_Controller():
    def __init__(self,network,configurations):
        self.network = network #accet to the application network
        self.command_queue = network.cobot_commands #access to its owncommand queue
        self.configurations = configurations # access to the global configurations
        self.cobot = Cobot((configurations.as_dict())["cobot_configs"]) #construct a cobot object with a the settings from the config
        print("Cobot awake",flush=True)
        
    async def loop(self):
        while True: 

            print("Waiting for command...", flush=True)

            command = await self.command_queue.get()

            print("command recived!",flush=True)

            for can in command.value:
                await asyncio.sleep(1)
                await self.cobot.pick_and_place(can)

            print("comand recived!!!",flush=True)
            await asyncio.sleep(1)





