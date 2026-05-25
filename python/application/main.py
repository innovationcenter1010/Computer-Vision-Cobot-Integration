import motoman_cpp

robot = motoman_cpp.MotomanRobot("192.168.1.31")

#robot.connect()

position1 = motoman_cpp.Position(250000, 0, 120500, 0, 0, 0)

# Move robot position register P001
robot.write_position(1, position1)

pos = robot.read_position(1)

print("X:", pos.x)
print("Y:", pos.y)
print("Z:", pos.z)

#robot.disconnect()