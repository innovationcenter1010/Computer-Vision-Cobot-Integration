import asyncio
from cobot_application.shared_data import Py_network, Config, sender_ID, P2P_command
from cobot_application.vision_system.run_operation_scan import run_YOLO


class Vision_Controller():
    def __init__(self,network: Py_network,configurations):
        self.network = network
        self.command_queue = network.vision_commands
        self.configurations = configurations
        self.id = sender_ID.vision_id
        print("vision system awake",flush=True)

    async def loop(self):
        while True:
            print("Vision waiting for command...", flush=True)

            command = await self.command_queue.get()

            print("Vision command received", flush=True)

            try:
                if command.message=="start_vision":
                    print("Starting YOLO scan...", flush=True)

                    detections = await asyncio.to_thread(
                        run_YOLO,
                        self.configurations.as_dict()["vision_system"]
                    )

                    print("YOLO scan complete", flush=True)
                    print("Detections:", detections, flush=True)


                    layer_command = P2P_command(
                        sender_ID=self.id,
                        receiver_ID=sender_ID.cobot_id,
                        message="pickup",
                        value=detections
                    )

                    await self.network.cobot_commands.put(layer_command)

                else:
                    print("Unknown vision command:", command.message, flush=True)

            except Exception as e:
                print("Vision loop error:", e, flush=True)

            await asyncio.sleep(0.1)

            
