from tools.MSCP import MSCP
from tools.Executable import Executable
from tools.ConfigurationFiles import *
from tools.ObjectFile import *
from tools.PortChecker import *
import time
import subprocess
import sys
import random
import string

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

    try:
        # Note: server does not close sockets properly so this fails frequently (cross fingers for now):
        #WaitForPortAvailable(54241,"TCP",timeout=0)
        server = Executable("../../build/bin/Core",["-m","0"])
        time.sleep(0.05)
        checkProgramStatus("=== Starting the server caused a problem")
        
        # 1: Connect to the server
        userControl = MSCP("127.0.0.1")
        time.sleep(0.25)
        checkProgramStatus("=== Connecting to the server caused a problem")

        # 2: Load trajectory
        [fewRowTraj,frTrajName] = ReadTrajectoryFile("resources/trajectories/faulty",fileName="GarageplanInnerring_lessRowsThanSpecified.traj")
        [manyRowTraj,mrTrajName] = ReadTrajectoryFile("resources/trajectories/faulty",fileName="GarageplanInnerring_moreRowsThanSpecified.traj")
        [normalTraj,nTrajName] = ReadTrajectoryFile("resources/trajectories")

        # Clear old data
        userControl.ClearTrajectories()
        userControl.ClearGeofences()
        userControl.ClearObjects()

        # 4: Upload short trajectory
        userControl.UploadFile(frTrajName, fewRowTraj, "trajectory")
        userControl.UploadFile(mrTrajName, manyRowTraj, "trajectory")
        userControl.UploadFile(nTrajName, normalTraj, "trajectory")

        objData = ConstructObjectFileData("127.0.0.1",frTrajName,random.randint(1,100))
        userControl.UploadFile(''.join(random.choice(string.ascii_letters) for i in range(10)) + ".obj",objData,"object")

        # 5: Send init
        try:
            userControl.Init()
            time.sleep(0.05)
            checkProgramStatus("=== Sending init to the server after uploading trajectory with less rows than specified caused a problem")
            userControl.waitForObjectControlState("INITIALIZED", timeout=0.5)
            raise AssertionError("Transitioned to initialized even though malformed trajectory was uploaded")
        except TimeoutError as e:
            # If there was a timeout while waiting for initialized that means everything went as intended
            print("=== Timed out successfully while waiting for initialisation")
        
        time.sleep(0.05) 
        
        # 6: Upload normal trajectory, to verify we can still initialise
        userControl.ClearObjects()
        objData = ConstructObjectFileData("127.0.0.1",nTrajName,random.randint(1,100))
        userControl.UploadFile(''.join(random.choice(string.ascii_letters) for i in range(10)) + ".obj",objData,"object")
        userControl.Init()
        userControl.waitForObjectControlState("INITIALIZED")
        
        time.sleep(0.05)

        userControl.Disconnect()
        userControl.waitForObjectControlState("IDLE")

        # 7: Upload long trajectory
        userControl.ClearObjects()
        objData = ConstructObjectFileData("127.0.0.1",mrTrajName,random.randint(1,100))
        userControl.UploadFile(''.join(random.choice(string.ascii_letters) for i in range(10)) + ".obj",objData,"object")
        
        # 8: Send init
        try:
            userControl.Init()
            time.sleep(0.05)
            checkProgramStatus("=== Sending init to the server after uploading trajectory with more rows than specified caused a problem")
            userControl.waitForObjectControlState("INITIALIZED", timeout=0.5)
            raise AssertionError("Transitioned to initialized even though malformed trajectory was uploaded")
        except TimeoutError as e:
            # If there was a timeout while waiting for initialized that means everything went as intended
            print("=== Timed out successfully while waiting for initialisation")

        time.sleep(0.05)
        
        # 9: Upload normal trajectory, to verify we can still initialise
        userControl.ClearObjects()
        objData = ConstructObjectFileData("127.0.0.1",nTrajName,random.randint(1,100))
        userControl.UploadFile(''.join(random.choice(string.ascii_letters) for i in range(10)) + ".obj",objData,"object")
        
        userControl.Init()
        userControl.waitForObjectControlState("INITIALIZED")

        time.sleep(0.05)

        userControl.Disconnect()
        userControl.waitForObjectControlState("IDLE")

    finally:
        # 10: Done!
        userControl.shutdown()
        server.stop()
    
    sys.exit(0)

