import time
import motoman_cpp
import asyncio

robot = motoman_cpp.MotomanRobot("192.168.1.31")


robot.connect()

# ---------------- Define 4 positions ----------------
p1 = motoman_cpp.Position(0, -925, 600, 180, 0, -90)
p2 = motoman_cpp.Position(0, -925, 500, 180, 0, -90)
p3 = motoman_cpp.Position(0, -925, 750, 180, 0, -90)
p4 = motoman_cpp.Position(900, 0, 750, 180, 0, 0)
p5 = motoman_cpp.Position(0, -925, 600, 180, 0, -90)

positions = [p1, p2, p3, p4, p5]

robot.set_motion_profile(speed=50.0, accel=50.0, decel=50.0)

"""
print("turning on")
robot.set_general_output_bit(1,2, True)  # Example: Set digital output bit 0 to True (this would depend on your specific robot's IO configuration)

time.sleep(5)  # Short delay to ensure the IO command is processed

print("turning off")
robot.set_general_output_bit(1,2, False)  # Example: Set digital output bit 0 to False
"""

# ---------------- Build trajectory ----------------
print("Clearing trajectory...")
robot.clear_trajectory()

print("Adding trajectory points...")


print("Starting motion...")
robot.motion_start()

async def execute_trajectory():

    await robot.goto_position(positions[0])
    await robot.goto_position(positions[1])
    await robot.goto_position(positions[2])
    await robot.goto_position(positions[3])
    await robot.goto_position(positions[4])

asyncio.run(execute_trajectory())

  # Move to the first position and wait for completion



# Optional: wait for motion (simple blocking delay)
# Replace with proper status polling if available
time.sleep(20)

print("Stopping (optional safety stop)...")
robot.motion_stop()

print("Clearing trajectory...")
robot.clear_trajectory()

print("Done")



