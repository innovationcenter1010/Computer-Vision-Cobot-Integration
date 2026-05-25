import motoman_cpp
import time

robot = motoman_cpp.MotomanRobot("192.168.1.31")

robot.connect()


while True: 

    pos = robot.read_position(1)


    print("Robot Position --------------")
    print("X:", pos.x)
    print("Y:", pos.y)
    print("Z:", pos.z)
    print("RX:", pos.rx)
    print("RY:", pos.ry)
    print("RZ:", pos.rz)

    time.sleep(1)


robot.disconnect()