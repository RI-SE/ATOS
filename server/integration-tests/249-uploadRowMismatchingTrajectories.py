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
        if server.poll():
            print(failurePrintout)
            if userControl != None:
                userControl.shutdown()
            server.stop()
            if obj != None:
                obj.stop()
            sys.exit(1)

if __name__ == "__main__":

    # 3: Start a test object
    WaitForPortAvailable(53240,"UDP",timeout=0)
    WaitForPortAvailable(53241,"TCP",timeout=0)
    obj = Executable("VirtualObject",["-nogui"])

    # Note: server does not close sockets properly so this fails frequently (cross fingers for now):
    #WaitForPortAvailable(54241,"TCP",timeout=0)
    server = Executable("../build/TEServer",["-m","0"])
    time.sleep(0.05)
    checkProgramStatus("=== Starting the server caused a problem")
    
    # 1: Connect to the server
    userControl = MSCP("127.0.0.1")
    time.sleep(0.25)
    checkProgramStatus("=== Connecting to the server caused a problem")

    # 2: Load trajectory
    fewRowTraj = ReadTrajectoryFile("resources/trajectories/faulty",fileName="GarageplanInnerring_lessRowsThanSpecified.traj")
    manyRowTraj = ReadTrajectoryFile("resources/trajectories/faulty",fileName="GarageplanInnerring_moreRowsThanSpecified.traj")
    normalTraj = ReadTrajectoryFile("resources/trajectories")

    # 4: Upload short trajectory
    userControl.UploadFile("traj/127.0.0.1", fewRowTraj)

    # 5: Send init
    try:
        userControl.Init()
        time.sleep(0.05)
        checkProgramStatus("=== Sending init to the server after uploading trajectory with less rows than specified caused a problem")
        userControl.waitForObjectControlState("INITIALIZED", timeout=0.5)
        raise AssertionError("Transitioned to initialized even though malformed trajectory was uploaded")
    except TimeoutError as e:
        # If there was a timeout while waiting for initialized that means everything went as intended
        print("=== Timed out while waiting for initialisation")
 
    # 6: Upload normal trajectory, to verify we can still initialise
    userControl.UploadFile("traj/127.0.0.1", normalTraj)

    userControl.Init()
    userControl.waitForObjectControlState("INITIALIZED")

    userControl.Disconnect()
    userControl.waitForObjectControlState("IDLE")

    # 7: Upload long trajectory
    userControl.UploadFile("traj/127.0.0.1", manyRowTraj)

    # 8: Send init
    try:
        userControl.Init()
        time.sleep(0.05)
        checkProgramStatus("=== Sending init to the server after uploading trajectory with more rows than specified caused a problem")
        userControl.waitForObjectControlState("INITIALIZED", timeout=0.5)
        raise AssertionError("Transitioned to initialized even though malformed trajectory was uploaded")
    except TimeoutError as e:
        # If there was a timeout while waiting for initialized that means everything went as intended
        print("=== Timed out while waiting for initialisation")

    # 9: Upload normal trajectory, to verify we can still initialise
    userControl.UploadFile("traj/127.0.0.1", normalTraj)

    userControl.Init()
    userControl.waitForObjectControlState("INITIALIZED")

    userControl.Disconnect()
    userControl.waitForObjectControlState("IDLE")

    # 10: Done!
    userControl.shutdown()
    server.stop()
    obj.stop()
    sys.exit(0)

