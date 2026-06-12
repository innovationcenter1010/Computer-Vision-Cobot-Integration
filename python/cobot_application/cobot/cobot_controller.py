
import asyncio

from cobot_application.cobot.cobot import Cobot
from cobot_application.shared_data import Py_network, Config, sender_ID, P2P_command
from python.cobot_application.cobot.calibration import run_cobot_calibration

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

            print("command received!", flush=True)
            print("Command message:", command.message, flush=True)
            print("Command value:", command.value, flush=True)

            if command.message == "calibrate_robot":
                print("Cobot calibration requested from webpage", flush=True)

                result = await run_cobot_calibration(
                    cobot=self.cobot,
                    configurations=self.configurations,
                    value=command.value
                )

                print("Calibration result:", result, flush=True)
                continue

            if command.message == "start_robot":
                print("Cobot start requested from webpage", flush=True)
                continue

            if command.message == "stop_robot":
                print("Cobot stop requested from webpage", flush=True)
                continue

            if command.message == "pickup":
                if command.value is None:
                    print("Pickup command received, but command.value is None", flush=True)
                    continue

                for can in command.value:
                    await self.cobot.pick_and_place(can)

                print("pickup command complete", flush=True)
                await asyncio.sleep(1)
                continue

            print("Unknown cobot command:", command.message, flush=True)
