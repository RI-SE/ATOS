from tools.MSCP import MSCP
from tools.Executable import Executable
from tools.TrajectoryFile import *
from tools.PortChecker import *
import time
import subprocess
import sys


userControl = None
server = None
obj = None


def checkProgramStatus(failurePrintout):
    if server != None:
        deadPIDs = server.poll()
        if deadPIDs:
            print(failurePrintout)
            print("Dead PIDs: " + str(deadPIDs))
            if userControl != None:
                userControl.shutdown()
            server.stop()
            if obj != None:
                obj.stop()
            sys.exit(1)

if __name__ == "__main__":

    # Note: server does not close sockets properly so this fails frequently (cross fingers for now):
    #WaitForPortAvailable(54241,"TCP",timeout=0)
    server = Executable("../build/bin/Core",["-m","0"])
    time.sleep(0.05)
    checkProgramStatus("=== Starting the server caused a problem")
    
    # 1: Connect to the server
    userControl = MSCP("127.0.0.1")
    time.sleep(0.25)
    checkProgramStatus("=== Connecting to the server caused a problem")

    # 2: Load trajectory
    traj = ReadTrajectoryFile("resources/trajectories/",fileName="random")
    
    # 3: Start a test object
    WaitForPortAvailable(53240,"UDP",timeout=0)
    WaitForPortAvailable(53241,"TCP",timeout=0)
    obj = Executable("VirtualObject",["-nogui"])
    
    # 4: Upload trajectory
    userControl.UploadFile("traj/127.0.0.1",traj)

    # 5: Send init
    userControl.Init()
    userControl.waitForObjectControlState("INITIALIZED")
    checkProgramStatus("Sending init to the server caused a problem")

    # 6: Send connect
    userControl.Connect()
    userControl.waitForObjectControlState("CONNECTED")
    checkProgramStatus("Sending connect to the server caused a problem")

    # 7: Send arm
    userControl.Arm()
    userControl.waitForObjectControlState("ARMED")
    checkProgramStatus("Sending arm to the server caused a problem")

    # 8: Send start
    userControl.Start(10)
    userControl.waitForObjectControlState("RUNNING")
    checkProgramStatus("Sending start to the server caused a problem")

    # 9: Send abort
    userControl.Abort()
    userControl.waitForObjectControlState("CONNECTED")
    checkProgramStatus("Sending abort to the server caused a problem")

    # 10: Send reset
    userControl.Disconnect()
    userControl.waitForObjectControlState("IDLE")
    checkProgramStatus("Sending disconnect to the server caused a problem")
    
    # 11: Done!
    userControl.shutdown()
    server.stop()
    obj.stop()
    sys.exit(0)



