from tools.MSCP import MSCP
from tools.Executable import Executable
#from tools.GeofenceFile import *
from tools.TrajectoryFile import *
from tools.ISO import ISO, ISOObject
import time
import subprocess
import sys

#core = Executable("../../build/bin/Core",["-m","1"])
#sup = Executable("../../build/bin/Supervision")
#time.sleep(0.05)
#mscp = MSCP("127.0.0.1")
#time.sleep(0.25)

def geofenceTransgressionTest():
    assert core.alive()
    assert sup.alive()



if __name__ == "__main__":
    obj = ISOObject()
    print("Sending a bunch of MONR")
    while True:
        try:
            obj.MONR(position={'x':10.0,'y':15.0,'z':4.5})
        except ConnectionError:
            pass
        time.sleep(0.01)
    exit(1)
    failed = False
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
