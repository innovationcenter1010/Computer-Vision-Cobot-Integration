
import asyncio

from cobot_application.cobot.cobot import Cobot, Cobot_Operation_Report, Cobot_Operation_Status
from python.cobot_application.shared_data import Py_network, Config, package_ID, P2P_command, cobot_commands

#cobot controller class for initializing and looping cobot functionality
class Cobot_Controller():
    def __init__(self,network: Py_network,configurations):
        self.network = network #accet to the application network
        self.id = package_ID.cobot_id
        print("whats going on",self.id in network.command_queues,flush=True)
        self.command_queue = network.command_queues[self.id.value] #access to its owncommand queue
        self.configurations = configurations # access to the global configurations
        self.cobot = Cobot((configurations.as_dict())["cobot_configs"]) #construct a cobot object with a the settings from the config

        self.command_map = {
            cobot_commands.pick_layer: self.pick_layer
        }

        print("Cobot awake",flush=True)
        
    async def loop(self):
        while True:
            print("cobot waiting for a command")

            #wait for a command to come in
            command: P2P_command = await self.command_queue.get()

            #map the command from the command map to run the actual function
            handle = self.command_map.get(command.message)
            
            #only run the command if it is properly mapped
            if handle: await handle(command)

            else: raise RuntimeError("Unknown command:", command.message)

    async def pick_layer(self,command: P2P_command):
        
        layer_report = Layer_Report()

        for can in command.value:
            await asyncio.sleep(0.5)
            r = Cobot_Operation_Report()
            r = await self.cobot.pick_and_place(can)
            layer_report.add(r.overall_status)
            print(r)
        



class Layer_Report:
    def __init__(self):
        self.successes: int = 0
        self.failures: int = 0
        self.overall: Cobot_Operation_Status = Cobot_Operation_Status.job_success
    
    def add(self,status: Cobot_Operation_Status):
        if(status == Cobot_Operation_Status.job_success):
            self.successes +=1
        elif(status == Cobot_Operation_Status.job_failure):
            self.failures += 1
            self.overall = Cobot_Operation_Status.job_failure
