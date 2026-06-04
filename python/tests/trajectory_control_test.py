import time
import os
import traceback
import motoman_cpp
import asyncio

# ---------------- Environment settings ----------------
robot_ip = os.getenv("ROBOT_IP", "192.168.1.31")
retry_seconds = float(os.getenv("ROBOT_RETRY_SECONDS", "5"))
keep_alive = os.getenv("KEEP_ALIVE", "1")

print("========================================", flush=True)
print("Cobot container started", flush=True)
print(f"Using ROBOT_IP={robot_ip}", flush=True)
print(f"ROBOT_RETRY_SECONDS={retry_seconds}", flush=True)
print("========================================", flush=True)

# IMPORTANT: use robot_ip from Portainer environment variable
robot = motoman_cpp.MotomanRobot(robot_ip)

print("Connecting to robot...", flush=True)

while True:
    try:
        connected = robot.connect()

        if connected:
            print(f"Connected to robot at {robot_ip}", flush=True)
            break

        print(f"Failed to connect to robot at {robot_ip}. Retrying in {retry_seconds} seconds...", flush=True)

    except Exception as e:
        print(f"Exception while connecting to robot at {robot_ip}: {e}", flush=True)
        traceback.print_exc()

    time.sleep(retry_seconds)

# ---------------- Motion sequence ----------------
try:
    print("Robot connected. Starting motion sequence in 3 seconds...", flush=True)
    time.sleep(3)

    # ---------------- Define positions ----------------
    p1 = motoman_cpp.Position(0, -925, 600, 180, 0, -90)
    p2 = motoman_cpp.Position(0, -925, 255, 180, 0, -90)
    p3 = motoman_cpp.Position(0, -925, 750, 180, 0, -90)
    p4 = motoman_cpp.Position(900, 0, 750, 180, 0, 0)
    p5 = motoman_cpp.Position(0, -925, 600, 180, 0, -90)

    positions = [p1, p2, p3, p4, p5]

    print("Setting motion profile...", flush=True)
    robot.set_motion_profile(speed=50.0, accel=50.0, decel=50.0)

    print("Clearing trajectory...", flush=True)
    robot.clear_trajectory()

    print("Starting motion...", flush=True)
    robot.motion_start()

    print("Adding point 1: above can", flush=True)
    robot.add_point_to_trajectory(positions[0])

    print("Adding point 2: go to can", flush=True)
    robot.add_point_to_trajectory(positions[1])

    print("Waiting before gripper ON...", flush=True)
    time.sleep(5)

    print("Turning output ON", flush=True)
    robot.write_general_output_bit(1, 2, True)

    print("Waiting after gripper ON...", flush=True)
    time.sleep(5)

    print("Adding point 3: lift can", flush=True)
    robot.add_point_to_trajectory(positions[2])

    print("Adding point 4: drop position", flush=True)
    robot.add_point_to_trajectory(positions[3])

    print("Waiting before gripper OFF...", flush=True)
    time.sleep(15)

    print("Turning output OFF", flush=True)
    robot.write_general_output_bit(1, 2, False)

    time.sleep(1)

async def execute_trajectory():

    await robot.goto_position(positions[0])
    await robot.goto_position(positions[1])
    await robot.goto_position(positions[2])
    await robot.goto_position(positions[3])
    await robot.goto_position(positions[4])

asyncio.run(execute_trajectory())

  # Move to the first position and wait for completion

    print("Waiting for motion to complete...", flush=True)
    time.sleep(20)

    print("Stopping motion...", flush=True)
    robot.motion_stop()

    print("Clearing trajectory...", flush=True)
    robot.clear_trajectory()

    print("Motion sequence done.", flush=True)

except Exception as e:
    print(f"Exception during robot motion: {e}", flush=True)
    traceback.print_exc()

    try:
        print("Attempting safety motion_stop...", flush=True)
        robot.motion_stop()
    except Exception as stop_error:
        print(f"motion_stop failed: {stop_error}", flush=True)

    try:
        print("Attempting trajectory clear...", flush=True)
        robot.clear_trajectory()
    except Exception as clear_error:
        print(f"clear_trajectory failed: {clear_error}", flush=True)

# ---------------- Keep container alive ----------------
if keep_alive == "1":
    print("Keeping container alive after sequence...", flush=True)
    while True:
        print("Container still alive.", flush=True)
        time.sleep(10)

print("Done.", flush=True)