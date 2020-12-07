from tools.MSCP import MSCP
from tools.Executable import Executable
#from tools.GeofenceFile import *
from tools.TrajectoryFile import *
from tools.ObjectFile import *
from tools.ISO import ISO, ISOObject
import time
import subprocess
import sys
import os

core = Executable("../../build/bin/Core",["-m","1"])
sup = Executable("../../build/bin/Supervision")
time.sleep(0.05)
mscp = MSCP("127.0.0.1")
time.sleep(0.25)
obj = ISOObject()

def geofenceTransgressionTest():
    assert core.alive()
    assert sup.alive()

    # 2: Load trajectory
    testcases = [x[0] for x in os.walk("resources/geofence-tests")]
    # TODO pick random test case
    [traj,fileName] = ReadTrajectoryFile("/",fileName="random")
    objID = random.randint(1,100)
    objData = ConstructObjectFileData("127.0.0.1", fileName, objID)



if __name__ == "__main__":
    try:
        failed = geofenceTransgressionTest()
    finally:
        if mscp:
            mscp.shutdown()
        if sup:
            sup.stop()
        if core:
            core.stop()
    exit(failed)
