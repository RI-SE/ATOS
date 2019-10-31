from tools.MSCP import MSCP
from tools.Executable import Executable
from tools.TrajectoryFile import *
import time
import subprocess
import sys

if __name__ == "__main__":

    
    server = Executable("../build/TEServer",["-m","0"])
    time.sleep(0.05)
    
    # 1: Connect to the server
    userControl = MSCP("127.0.0.1")
    time.sleep(0.25)
    if server.poll():
        print("Connecting to the server caused a problem")
        server.stop()
        sys.exit(1)
    # 2: Load trajectory
    traj = ReadTrajectoryFile("resources/trajectories/",fileName="random")

    # 3: Start a test object
    # 4: Upload trajectory
    userControl.UploadFile("traj/127.0.0.1",traj)

    # 5: Send init
    userControl.Init()
    userControl.waitForObjectControlState("INITIALIZED")
    if server.poll():
        print("Sending init to the server caused a problem")
        server.stop()
        sys.exit(1)

    # 6: Send connect
    userControl.Connect()
    userControl.waitForObjectControlState("CONNECTED")
    if server.poll():
        print("Sending connect to the server caused a problem")
        server.stop()
        sys.exit(1)

    # 7: Send arm
    userControl.Arm()
    userControl.waitForObjectControlState("ARMED")
    if server.poll():
        print("Sending arm to the server caused a problem")
        server.stop()
        sys.exit(1)

    # 8: Send start
    userControl.Start(10)
    userControl.waitForObjectControlState("RUNNING")
    if server.poll():
        print("Sending start to the server caused a problem")
        server.stop()
        sys.exit(1)

    # 9: Send abort
    userControl.Abort()
    userControl.waitForObjectControlState("CONNECTED")
    if server.poll():
        print("Sending abort to the server caused a problem")
        server.stop()
        sys.exit(1)

    # 10: Send reset
    userControl.Disconnect()
    userControl.waitForObjectControlState("IDLE")
    if server.poll():
        print("Sending disconnect to the server caused a problem")
        server.stop()
        sys.exit(1)
    
    # 11: Done!
    userControl.shutdown()
    server.stop()
    sys.exit(0)
