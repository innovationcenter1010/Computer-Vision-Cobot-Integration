import asyncio
from python.cobot_application.shared_data import Py_network, Config, package_ID, P2P_command, vision_commands, cobot_commands
from cobot_application.vision_system.run_operation_scan import run_YOLO


class Vision_Controller():
    def __init__(self,network: Py_network,configurations):
        self.network = network
        self.id = package_ID.vision_id
        self.command_queue = network.command_queues[self.id.value]
        self.configurations = configurations

        self.command_map = {

            vision_commands.detect: self.scan

        }

        print("vision system awake",flush=True)

    async def loop(self):
        while True:
            print("Vision waiting for command...", flush=True)

            #wait for a command to come in
            command: P2P_command = await self.command_queue.get()

            #map the command from the command map to run the actual function
            handle = self.command_map.get(command.message)
            
            #only run the command if it is properly mapped
            if handle: await handle(command)

            else: raise RuntimeError("Unknown command:", command.message)



    async def scan(self, command: P2P_command):

        print("Starting YOLO scan...", flush=True)

        detections = await asyncio.to_thread(
            run_YOLO,
            None,
            self.configurations.as_dict()["vision_system"]
        )

        print("YOLO scan complete", flush=True)
        print("Detections:", detections, flush=True)


        layer_command = P2P_command(
            sender_ID=self.id,
            receiver_ID=package_ID.cobot_id,
            message=cobot_commands.pick_layer,
            value=detections
        )

        await self.network.send(layer_command)