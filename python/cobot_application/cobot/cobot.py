from cobot_application.cobot import motoman_cpp
from cobot_application.cobot.layers import Can
import asyncio
from enum import Enum,auto

GRIPPER_ON = True
GRIPPER_OFF = False


class Cobot_Operation_Report(Enum):
    SUCCESS = auto()
    PICK_FAILURE = auto()
    PLACE_FAILURE = auto()


class Cobot:

    #cobot will be initialized with the dictionary cobot configs
    #one of the sub dictionaies for the configurations json
    def __init__(self, cobot_configs):
        self.cobot_configs = cobot_configs
        self.robot = motoman_cpp.MotomanRobot(cobot_configs["cobot_ip"])
        
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
        tooling_offset = self.cobot_configs["tool_head_length_mm"]
        compression_offset = self.cobot_configs["tool_head_compression_distance"]

        cobot_pickup_position = motoman_cpp.Position(can.real_x, can.real_y, can.real_z, 0, 0, can.real_rz)
        cobot_address_can_position = motoman_cpp.Position(can.real_x, can.real_y, can.real_z + 200, 0, 0, can.real_rz)
        pickup_trajectory = [cobot_address_can_position,cobot_pickup_position]




        for position in pickup_trajectory:
            await self.goto(position)

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
    



