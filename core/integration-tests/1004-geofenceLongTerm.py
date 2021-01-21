from tools.MSCP import MSCP
from tools.Executable import Executable
from tools.ConfigurationFiles import *
from tools.ISO import ISO, ISOObject
import time
from time import sleep
import random
import string
import subprocess
import sys

S_PI = 3.1415926535

core = Executable("Core",["-m","1"])
sup = Executable("Supervision")
time.sleep(0.05)
mscp = MSCP("127.0.0.1")
time.sleep(0.25)
obj = ISOObject()

runCounter = 0
failedRuns = 0
successfulRuns = 0

geofAllowedTrajs = [{0: [0, 1, 2]},
                    {1: [0, 1]},
                    {2: [1]}]

geofArr = [[(-100,-100),
        ( 100,-100),
        ( 100, 100),
        (-100, 100)],

        [(-50,-50),
        ( 50,-50),
        ( 50, 50),
        (-50, 50)],

        [(-25,-50),
        ( 25,-50),
        ( 25, 50),
        (-25, 50)]]


trajPtsArr = [[{'time': 0.00, 'x': 0.000,  'y': 0.000, 'heading': 0.0},
            {'time': 0.05, 'x': 5.000,  'y': 0.000, 'heading': 0.0},
            {'time': 0.10, 'x': 10.000, 'y': 0.000, 'heading': 0.0},
            {'time': 0.15, 'x': 15.000, 'y': 0.000, 'heading': 0.0},
            {'time': 0.20, 'x': 20.000, 'y': 0.000, 'heading': 0.0},
            {'time': 0.25, 'x': 25.001, 'y': 0.000, 'heading': 0.0}],

            [{'time': 0.00, 'x': 0.000, 'y': 0.000,  'heading': 90.0 * 3.1415926535/180},
            {'time': 0.05, 'x': 0.000, 'y': -5.000,  'heading': 90.0 * 3.1415926535/180},
            {'time': 0.10, 'x': 0.000, 'y': -10.000, 'heading': 90.0 * 3.1415926535/180},
            {'time': 0.15, 'x': 0.000, 'y': -15.000, 'heading': 90.0 * 3.1415926535/180},
            {'time': 0.20, 'x': 0.000, 'y': -20.000, 'heading': 90.0 * 3.1415926535/180},
            {'time': 0.25, 'x': 0.000, 'y': -25.000, 'heading': 90.0 * 3.1415926535/180},
            {'time': 0.30, 'x': 0.000, 'y': -30.000, 'heading': 90.0 * 3.1415926535/180}],

            [{'time': 0.00, 'x': 0.000,   'y': 0.000,   'heading': 135.0 * 3.1415926535/180},
            {'time': 0.05, 'x': -5.000,  'y': -5.000,  'heading': 135.0 * 3.1415926535/180},
            {'time': 0.10, 'x': -15.000, 'y': -15.000, 'heading': 135.0 * 3.1415926535/180},
            {'time': 0.15, 'x': -25.000, 'y': -25.000, 'heading': 135.0 * 3.1415926535/180},
            {'time': 0.20, 'x': -35.000, 'y': -35.000, 'heading': 135.0 * 3.1415926535/180},
            {'time': 0.25, 'x': -45.000, 'y': -45.000, 'heading': 135.0 * 3.1415926535/180},
            {'time': 0.30, 'x': -55.000, 'y': -55.000, 'heading': 135.0 * 3.1415926535/180}]]

def geofenceTransgressionTest(_trajectory, _geofence, _shouldPass = False):
    assert core.alive(), "Core terminated unexpectedly"
    assert sup.alive(), "Supervision terminated unexpectedly"

    success = 1

    geofencePoints = _geofence
    trajPts = _trajectory

    maxAbortDelay = 0.1

    traj = ConstructTrajectoryFileData(trajPts, "GeofenceTestTrajectory1")

    # Create geofence
    geofence = ConstructGeofenceFileData(geofencePoints, "GeofenceTestGeofence1", forbidden=False)

    # Upload config
    mscp.ClearTrajectories()
    mscp.ClearGeofences()
    mscp.ClearObjects()

    trajFileName = "trajectoryFileName.traj"
    mscp.UploadFile(trajFileName,traj,"trajectory")
    geofenceFileName = "GeofenceTestGeofence1.geofence"
    mscp.UploadFile(geofenceFileName,geofence,"geofence")

    objID = random.randint(1,100)
    objData = ConstructObjectFileData("127.0.0.1", trajFileName, objID)
    mscp.UploadFile(''.join(random.choice(string.ascii_letters) for i in range(10)) + ".not.obj",objData,"object")

    # Initialize
    mscp.Init()
    try:
        mscp.waitForObjectControlState("INITIALIZED")
    except:
        print("Couldn't enter INITIALIZED state. Test failed.")
        return False

    # Connect
    mscp.Connect()
    try:
        mscp.waitForObjectControlState("CONNECTED")
    except:
        print("Couldn't enter CONNECTED state. Test failed.")
        return False

    # Wait for first HEAB
    connectTime = time.time()
    maxHEABWaitTime = 0.05
    while True:
        try:
            obj.MONR(transmitter_id=objID,position=trajPts[0],heading_deg=trajPts[0]['heading']*180.0/S_PI)
            break
        except ConnectionError:
            pass
        assert time.time() - connectTime < maxHEABWaitTime, f"No HEAB received within {maxHEABWaitTime} s"

    # Arm
    mscp.Arm()
    try:
        mscp.waitForObjectControlState("ARMED", 2)
    except:
        print("Couldn't enter ARMED state. Test failed.")
        return False

    obj.MONR(transmitter_id=objID,position=trajPts[0],heading_deg=(trajPts[0]['heading']*180.0/S_PI))

    # Start
    mscp.Start(0)
    try:
        mscp.waitForObjectControlState("RUNNING")
    except:
        print("Couldn't enter RUNNING state. Test failed.")
        return False

    # Report a number of MONR inside geofence
    print("=== Entered running state, sending test MONR data")
    for x in range(len(trajPts)):
        obj.MONR(transmitter_id=objID,position=trajPts[x])
        sleep(0.01)

        #assert mscp.lastStatusReply["objectControlState"] == "RUNNING", print("Last OBC status is != RUNNING")


    #Check if abort was sent as it should have
    if(_shouldPass):
        if obj.lastCCStatus() != "abort":
            print("Error: No abort was sent.")
            success = 0
    else:
        if obj.lastCCStatus() == "abort":
            print("Error: Abort was sent when it should not have been.")
            success = 0

    # Abort
    mscp.Abort()
    try:
        mscp.waitForObjectControlState("CONNECTED")
    except:
        print("Couldn't enter CONNECTED state. Test failed.")
        return False

    # Disconnect
    mscp.Disconnect()
    try:
        mscp.waitForObjectControlState("IDLE")
    except:
        print("Couldn't enter IDLE state. Test failed.")
        return False



    transgressionTime = time.time()

    # Sleep until max allowed time passed
    time.sleep(maxAbortDelay-(time.time()-transgressionTime))



    if(_shouldPass):
        return success  
    return not success





if __name__ == "__main__":

    try:

        t_end = time.time() + 60 * 15

        while time.time() < t_end:

            #Select a random geofence and traj and test them to eachother
            geofInt = random.randint(0,len(geofArr)-1)
            trajInt = random.randint(0,len(trajPtsArr)-1)
            willPass = trajInt in geofAllowedTrajs[geofInt]     #LUT for pass-fail criteria

            if geofenceTransgressionTest(trajPtsArr[trajInt], geofArr[geofInt], willPass):
                successfulRuns += 1
                print(f"Test successful with geofence[{str(geofInt)}] and trajectory[{str(trajInt)}]")
            else:
                failedRuns += 1
                print(f"Test failed with geofence[{str(geofInt)}] and trajectory[{str(trajInt)}]\n")

            #How many tests that has run so far
            runCounter += 1

            sleep(0.5)
    finally:

        print("\n")
        print(f"The test has been run {str(runCounter)} times \n")
        print(f"The test failed {str(failedRuns)} times and was successful {str(successfulRuns)} times.")

        if mscp:
            mscp.shutdown()
        if sup:
            sup.stop()
        if core:
            core.stop()
        if obj: 
            obj.shutdown()

        assert failedRuns == 0, f"Geofence robustness test failed {failedRuns}/{runCounter} test(s)"