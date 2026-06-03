import motoman_cpp
from layers import Can
import asyncio

class Cobot:

    def __init__(self, ip_address: str):
        self.robot = motoman_cpp.MotomanRobot(ip_address)
        self.speed = 50 # Default speed for robot movements

    def pickup_can(self, can):
        """
        Moves the robot to the specified can position and picks it up.
        can_position: a motoman_cpp.Position object representing the can's location.
        """

        cobot_pickup_position = motoman_cpp.Position(can.real_x, can.real_y, can.real_z, 0, 0, can.real_rz)

        # Move robot to the can's position
        self.robot.write_position(1, cobot_pickup_position)

        # Simulate picking up the can (this would involve more complex logic in a real implementation)
        print(f"Picking up can at position: {cobot_pickup_position}")

        return True
    
    def goto(self,position):
        self.robot.add_point_to_trajectory(position)
        while not self.robot.has_reached_target():
            asyncio.sleep(0.1)
