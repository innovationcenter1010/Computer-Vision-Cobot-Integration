import asyncio
import time
import motoman_cpp

async def run_cobot_calibration(cobot, configurations=None, value=None):

    print("CALIBRATION SCRIPT STARTED", flush=True)
    print("Calibration value:", value, flush=True)

robot = motoman_cpp.MotomanRobot("192.168.1.31")

robot.connect()

# ---------------- Define 4 positions ----------------
p1 = motoman_cpp.Position(0, -925, 800, 180, 0, -90)
p2 = motoman_cpp.Position(0, -925, 500, 180, 0, -90)
p3 = motoman_cpp.Position(0, -925, 750, 180, 0, -90)
p4 = motoman_cpp.Position(900, 0, 750, 180, 0, 0)
p5 = motoman_cpp.Position(0, -925, 600, 180, 0, -90)

positions = [p1, p2, p3, p4, p5]

robot.set_motion_profile(speed=50.0, accel=50.0, decel=50.0)


# ---------------- Build trajectory ----------------
print("Clearing trajectory...",flush = True)
robot.clear_trajectory()

print("Adding trajectory points...",flush = True)


print("Starting motion...")
robot.motion_start()


async def goto(position):
    robot.add_point_to_trajectory(position)
    robot.motion_start()  # Ensure motion is started after adding the point
    while not robot.has_reached_target():
        await asyncio.sleep(0.1)

async def main():
    for p in positions:
        await goto(p)

asyncio.run(main())


print("Stopping (optional safety stop)...")
robot.motion_stop()

print("Clearing trajectory...")
robot.clear_trajectory()

print("Done")