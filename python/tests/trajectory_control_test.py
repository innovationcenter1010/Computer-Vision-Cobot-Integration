import time
import motoman_cpp

robot = motoman_cpp.MotomanRobot("192.168.1.31")

robot.connect()

# ---------------- Define 4 positions ----------------
p1 = motoman_cpp.Position(0, -925, 600, 180, 0, -90)
p2 = motoman_cpp.Position(925, 0, 600, 180, 0, 0)
p3 = motoman_cpp.Position(925, 0, 900, 180, 0, 0)
p4 = motoman_cpp.Position(0, -925, 900, 180, 0, -90)
p5 = motoman_cpp.Position(0, -925, 600, 180, 0, -90)

positions = [p1, p2, p3, p4, p5]

robot.set_motion_profile(speed=50.0, accel=50.0, decel=50.0)

# ---------------- Build trajectory ----------------
print("Clearing trajectory...")
robot.clear_trajectory()

print("Adding trajectory points...")

for pos in positions:
    robot.add_point_to_trajectory(pos)


print("Starting motion...")
robot.motion_start()

# Optional: wait for motion (simple blocking delay)
# Replace with proper status polling if available
time.sleep(30)

print("Stopping (optional safety stop)...")
robot.motion_stop()

print("Clearing trajectory...")
robot.clear_trajectory()

print("Done")



