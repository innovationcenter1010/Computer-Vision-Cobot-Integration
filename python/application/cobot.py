import motoman_cpp
from layers import Can
import asyncio

GRIPPER_ON = True
GRIPPER_OFF = False

class Cobot:

    async def __init__(self, ip_address: str):
        self.robot = motoman_cpp.MotomanRobot(ip_address)
        
        for attempt in range(5):
            result = self.robot.connect()
            if result.status.is_ok():
                print("Connected to robot!!")
                break
            else:
                print(f"Connection attempt {attempt + 1} failed. Retrying...")
                asyncio.sleep(2)


        self.gripper_io_group = 1
        self.gripper_io_bit = 2
        self.targets = []


    async def pick_and_place(self, can):

        cobot_pickup_position = motoman_cpp.Position(can.real_x, can.real_y, can.real_z, 0, 0, can.real_rz)

        await self.goto(cobot_pickup_position)

        self._set_gripper(GRIPPER_ON)

        await self.goto(self.targets[0])

        self._set_gripper(GRIPPER_OFF)
    
    async def goto(self,position):
        self.robot.add_point_to_trajectory(position)
        self.robot.motion_start()
        while not self.robot.has_reached_target():
            await asyncio.sleep(0.2)

    def disconnect(self):
        self.robot.disconnect()

    def _set_gripper(self, state: bool):
        return self.robot.set_general_output_bit(self.gripper_io_group, self.gripper_io_bit, state)
