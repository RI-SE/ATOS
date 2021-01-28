from tools.MSCP import MSCP
from tools.Executable import Executable
from tools.ConfigurationFiles import *
from tools.ISO import ISO, ISOObject
import time
import random
import string
import subprocess

core = Executable("../../build/bin/Core",["-m","1"])
sup = Executable("../../build/bin/Supervision")
time.sleep(0.05)
mscp = MSCP("127.0.0.1")
time.sleep(0.25)
obj = ISOObject()

def geofenceTransgressionTest():
    assert core.alive(), "Core terminated unexpectedly"
    assert sup.alive(), "Supervision terminated unexpectedly"

    geofencePoints = [(-50,-50),
                      ( 50,-50),
                      ( 50, 50),
                      (-50, 50)]
    testPts = [(25.000,25.000),
               (30.000,30.000),
               (40.000,40.000),
               (40.000,49.999),
               (50.001,49.999),
               ( 1.000, 1.000),
               ( 0.000, 0.000)]
    maxAbortDelay = 0.1

    # Load trajectory
    trajPts = [{'time': 0.00, 'x': 0.0,  'y': 0.0, 'heading': 0.0},
               {'time': 0.50, 'x': 5.0,  'y': 0.0, 'heading': 0.0},
               {'time': 0.10, 'x': 10.0, 'y': 0.0, 'heading': 0.0},
               {'time': 0.15, 'x': 15.0, 'y': 0.0, 'heading': 0.0},
               {'time': 0.20, 'x': 20.0, 'y': 0.0, 'heading': 0.0},
               {'time': 0.25, 'x': 25.0, 'y': 0.0, 'heading': 0.0}]
    traj = ConstructTrajectoryFileData(trajPts, "GeofenceTestTrajectory1")

    # Create geofence
    geofence = ConstructGeofenceFileData(geofencePoints, "GeofenceTestGeofence1", forbidden=False)

    # Upload config
    mscp.ClearTrajectories()
    mscp.ClearGeofences()
    mscp.ClearObjects()

    trajFileName = "GeofenceTestTrajectory1.traj"
    mscp.UploadFile(trajFileName,traj,"trajectory")
    geofenceFileName = "GeofenceTestGeofence1.geofence"
    mscp.UploadFile(geofenceFileName,geofence,"geofence")

    objID = random.randint(1,100)
    objData = ConstructObjectFileData("127.0.0.1", trajFileName, objID)
    mscp.UploadFile(''.join(random.choice(string.ascii_letters) for i in range(10)) + ".not.obj",objData,"object")

    # Initialize
    mscp.Init()
    mscp.waitForObjectControlState("INITIALIZED")

    # Connect
    mscp.Connect()
    mscp.waitForObjectControlState("CONNECTED")

    # Wait for first HEAB
    connectTime = time.time()
    maxHEABWaitTime = 0.05
    while True:
        try:
            obj.MONR(transmitter_id=objID,position=trajPts[0],heading_deg=trajPts[0]['heading']*180.0/3.14159)
            break
        except ConnectionError:
            pass
        assert time.time() - connectTime < maxHEABWaitTime, f"No HEAB received within {maxHEABWaitTime} s"

    # Arm
    mscp.Arm()
    mscp.waitForObjectControlState("ARMED",timeout=0.5)

    obj.MONR(transmitter_id=objID,position=trajPts[0],heading_deg=trajPts[0]['heading']*180.0/3.14159)

    # Start
    mscp.Start(0)
    mscp.waitForObjectControlState("RUNNING",timeout=0.5)
    time.sleep(0.01)
    obj.waitForHEAB() # Await one new HEAB
    assert obj.lastCCStatus() == "running", "HEAB state not set to running after start"

    # Report a number of MONR inside geofence
    print("=== Entered running state, sending test MONR data")
    obj.MONR(transmitter_id=objID,position=testPts[0])
    time.sleep(0.001*random.randint(1,7))
    obj.MONR(transmitter_id=objID,position=testPts[1])
    time.sleep(0.001*random.randint(1,7))
    obj.MONR(transmitter_id=objID,position=testPts[2])
    time.sleep(0.001*random.randint(1,7))
    obj.MONR(transmitter_id=objID,position=testPts[3])
    time.sleep(0.001*random.randint(1,7))

    # Check last HEAB so it is not ABORT
    print(obj.lastCCStatus())
    assert obj.lastCCStatus() == "running", "HEAB state not kept at running after valid positions"
    print("=== Sending transgressing MONR data")

    # Report one MONR outside geofence
    obj.MONR(transmitter_id=objID,position=testPts[4])
    transgressionTime = time.time()
    time.sleep(0.01)
    obj.MONR(transmitter_id=objID,position=testPts[5])
    time.sleep(0.001*random.randint(1,7))
    obj.MONR(transmitter_id=objID,position=testPts[6])
    obj.waitForHEAB() # temporary - may allow longer time to pass

    # Sleep until max allowed time passed
    time.sleep(maxAbortDelay-(time.time()-transgressionTime))
    print("Sleeping for: " + str(maxAbortDelay-(time.time()-transgressionTime)))
    # Check last HEAB so it is ABORT
    assert obj.lastCCStatus() == "abort", "HEAB state not set to abort after exiting geofence"
    return




if __name__ == "__main__":
    try:
        geofenceTransgressionTest()
    finally:
        if mscp:
            mscp.shutdown()
        if sup:
            sup.stop()
        if core:
            core.stop()
        if obj:
            obj.shutdown()
