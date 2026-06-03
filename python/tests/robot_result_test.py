import time
import motoman_cpp


def print_result(name, result):
    print("\n==============================")
    print(f"{name}")

    # status (adjust depending on your StatusInfo binding)
    try:
        print("Status:", result.status)
    except Exception:
        print("Status (raw):", result.status)

    # payload (std::any → python object)
    try:
        print("Payload:", result.payload)
    except Exception:
        print("Payload: <unprintable>")


# ---------------- Robot init ----------------
robot = motoman_cpp.MotomanRobot("192.168.1.31")

print_result("CONNECT", robot.connect())

# ---------------- Positions ----------------
p1 = motoman_cpp.Position(0, -925, 600, 180, 0, -90)
p2 = motoman_cpp.Position(0, -925, 255, 180, 0, -90)
p3 = motoman_cpp.Position(0, -925, 750, 180, 0, -90)
p4 = motoman_cpp.Position(900, 0, 750, 180, 0, 0)
p5 = motoman_cpp.Position(0, -925, 600, 180, 0, -90)

positions = [p1, p2, p3, p4, p5]

# ---------------- Motion profile ----------------
print_result(
    "SET MOTION PROFILE",
    robot.set_motion_profile(speed=50.0, accel=50.0, decel=50.0)
)

# ---------------- Trajectory ----------------
print_result("CLEAR TRAJECTORY", robot.clear_trajectory())

print("Starting motion...")
print_result("MOTION START", robot.motion_start())

# ---------------- Add + execute moves ----------------
print("Adding trajectory points...")

for i, pos in enumerate(positions):
    print_result(
        f"ADD POINT {i}",
        robot.add_point_to_trajectory(pos)
    )

    time.sleep(10)  # small delay so logs are readable

# ---------------- Safety stop ----------------
time.sleep(2)

print_result("MOTION STOP", robot.motion_stop())

print_result("CLEAR TRAJECTORY (FINAL)", robot.clear_trajectory())

print("\nDone")