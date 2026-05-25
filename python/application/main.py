import motoman_cpp

robot = motoman_cpp.MotomanRobot("192.168.1.31")

robot.connect()

# Move robot position register P001
robot.write_position(1, 250000, 0, 120500)

pos = robot.read_position(1)

print("X:", pos.x)
print("Y:", pos.y)
print("Z:", pos.z)

robot.disconnect()