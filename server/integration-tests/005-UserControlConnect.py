from tools.MSCP import MSCP
from tools.Maestro import Maestro
import time
import subprocess

if __name__ == "__main__":

    S = Maestro("../build/TEServer",0)
    time.sleep(0.05)
    M = MSCP("127.0.0.1")

    time.sleep(1)
    M.Init()
    time.sleep(1)
    S.stop()
