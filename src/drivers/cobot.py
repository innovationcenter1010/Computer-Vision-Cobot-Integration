import time
from underautomation.yaskawa.yaskawa_robot import YaskawaRobot
from underautomation.yaskawa.connect_parameters import ConnectParameters

# Initialize connection over CMTK LAN Port
robot = YaskawaRobot()
robot.connect(ConnectParameters("192.168.1.40"))
hses = robot.high_speed_e_server # Shorten the API handle

def send_target_coordinate(x, y, z, rx, ry, rz):
    print("Waiting for Cobot to be ready...")
    # 1. Wait until the Cobot signals it is ready (I002 == 1) and idle (I001 == 0)
    while hses.read_integer_variable(2) != 1 or hses.read_integer_variable(1) != 0:
        time.sleep(0.05) # Poll every 50ms

    # 2. Write the new CV coordinates to Position Variable 1 (P001)
    # Format: [X, Y, Z, Rx, Ry, Rz] (Yaskawa expects millimeters and degrees)
    hses.write_position_variable(1, [x, y, z, rx, ry, rz])
    print(f"Coordinates written to P001: X={x}, Y={y}, Z={z}")

    # 3. Raise the flag! Set I001 to 1 to tell the cobot data is ready
    hses.write_integer_variable(1, 1)
    print("Handshake flag I001 set to 1. Waiting for cobot execution...")

    # 4. Wait for the cobot to acknowledge, finish the move, and reset the flag to 0
    while hses.read_integer_variable(1) != 0:
        time.sleep(0.05)
        
    print("Move complete! Ready for next CV cycle.")