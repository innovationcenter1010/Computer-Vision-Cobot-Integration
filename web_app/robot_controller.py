class RobotController:
    def __init__(self):
        self.connected = False

    def connect(self):
        print("Connecting to robot...")
        self.connected = True
        return {"ok": True, "message": "Robot connected"}

    def start_robot(self):
        print("START ROBOT CALLED")
        return {"ok": True, "message": "Start robot function called"}

    def stop_robot(self):
        print("STOP ROBOT CALLED")
        return {"ok": True, "message": "Stop robot function called"}

    def calibrate_robot(self):
        print("CALIBRATE ROBOT CALLED")
        return {"ok": True, "message": "Calibrate robot function called"}