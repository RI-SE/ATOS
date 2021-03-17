from tools.MSCP import MSCP
from tools.Executable import Executable
import time
import subprocess
import sys

if __name__ == "__main__":

    S = Executable("../../build/bin/Core",["-m","0"])
    time.sleep(1)
    M = MSCP("127.0.0.1")

    time.sleep(1)
    if S.poll():
        S.stop()
        M.shutdown()
        sys.exit(1)
    M.Init()
    time.sleep(1)
    if S.poll():
        S.stop()
        M.shutdown()
        sys.exit(1)
    S.stop()
    M.shutdown()
    sys.exit(0)
