from cobot_application.cobot import motoman_cpp
from cobot_application.vision_system.yolo_obb_detector import OBBDetection
import asyncio
from enum import Enum,auto

GRIPPER_ON = True
GRIPPER_OFF = False

#struct containing the the status of a large cobot operation
class Cobot_Operation_Report(Enum):
    SUCCESS = 0
    PICK_FAILURE = 1
    PLACE_FAILURE = 2


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

        #cast the position data from configs.JSO into the position type compatible with the cobot 
        self.target = motoman_cpp.Position(*(self.cobot_configs["cobot_place_positions"])["place_position_a"])

    


    async def pick_and_place(self, can: OBBDetection):
        tooling_offset = self.cobot_configs["tool_head_length_mm"]
        compression_offset = self.cobot_configs["tool_head_compression_distance_mm"]

        cobot_pickup_position = motoman_cpp.Position(can.real_x, can.real_y, 400, 180, 0, 0)
        cobot_address_can_position = motoman_cpp.Position(can.real_x, can.real_y, 400, 180, 0, 0)
        pickup_trajectory = [cobot_address_can_position,cobot_pickup_position]

        for position in pickup_trajectory:
            await self.goto(position)

        self._set_gripper(GRIPPER_ON)

        await self.goto(self.target)

        self._set_gripper(GRIPPER_OFF)
    
    async def goto(self,position):
        self.robot.add_point_to_trajectory(position)
        self.robot.motion_start()
        while not self.robot.has_reached_target():
            await asyncio.sleep(0.2)

    def disconnect(self):
        self.robot.disconnect()

    def _set_gripper(self, state: bool):
        return self.robot.write_general_output_bit(self.gripper_io_group, self.gripper_io_bit, state)
    



    



