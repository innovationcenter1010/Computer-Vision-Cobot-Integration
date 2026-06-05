import asyncio
from cobot_application.shared_data import Py_network, Config, sender_ID, P2P_command
from cobot_application.cobot.layers import Can 

class Camera_Controller():
    def __init__(self,network,configurations):
        self.network = network
        self.command_queue = network.camera_commands
        self.configurations = configurations
        self.id = sender_ID.camera_id
        print("camera awake",flush=True)

    async def loop(self):
        while True:

            pickup = create_fake_can(0,-925,500,0)

            layer_command = P2P_command(self.id,"pickup",pickup)
            await self.network.cobot_commands.put(layer_command)
            #print("computer vision functions",flush=True)
            await asyncio.sleep(1000)


def create_fake_can(x, y,z,rz):
    can = Can.__new__(Can)  # bypass __init__

    # fake classification data
    can.can_class = "fake_can"
    can.class_id = -1
    can.confidence = 1.0

    # pixel / image space
    can.corner_x = x
    can.corner_y = y
    can.width = 0
    can.height = 0

    can.center_x = x + 2
    can.center_y = y + 2

    # REAL WORLD VALUES (directly mapped from inputs)
    can.real_x = x
    can.real_y = y
    can.real_z = z
    can.real_rz = rz

    return can