import time
import motoman_cpp


robot = motoman_cpp.MotomanRobot("192.168.1.31")


print("Connecting...")
#robot.connect()
print("Connected")

# ---------------- Define 4 positions ----------------
p1 = motoman_cpp.Position(300, 0, 400, 0, 0, 0)
p2 = motoman_cpp.Position(400, 100, 450, 0, 0, 0)
p3 = motoman_cpp.Position(500, -100, 450, 0, 0, 0)
p4 = motoman_cpp.Position(600, 0, 400, 0, 0, 0)

robot.set_motion_profile(speed=50.0, accel=50.0, decel=50.0)

# ---------------- Build trajectory ----------------
print("Clearing trajectory...")
robot.clear_trajectory()

print("Adding trajectory points...")

robot.add_point_to_trajectory(p1)
robot.add_point_to_trajectory(p2)
robot.add_point_to_trajectory(p3)
robot.add_point_to_trajectory(p4)

print("Starting motion...")
robot.motion_start()

# Optional: wait for motion (simple blocking delay)
# Replace with proper status polling if available
time.sleep(10)

print("Stopping (optional safety stop)...")
robot.motion_stop()

print("Clearing trajectory...")
robot.clear_trajectory()

print("Done")



