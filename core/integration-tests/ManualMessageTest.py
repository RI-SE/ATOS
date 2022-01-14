from tools.MSCP import MSCP
from tools.ISO import ISO
from tools.Executable import Executable
import time
import subprocess
import sys


MSCPTests = ["INIT", 
        "CONNECTOBJECT", 
        "ARM", 
        "START", 
        "ABORT", 
        "GETSERVERSTATUS", 
        "CREATEOBJECTS", 
        "DISARM", 
        "GETSERVERTIME", 
        "UPLOADFILE", 
        "DOWNLOADFILE", 
        "DELETEFILE", 
        "CHECKFILEEXISTS", 
        "REMOTECONTROL", 
        "SETSERVERPARAMETER", 
        "GETSERVERPARAMETERS", 
        "GETDIRECTORYCONTENT",
        "DISCONNECTOBJECT" ]

ISOTests = ["TRAJ",
            "OSEM",
            "OSTM",
            "STRT",
            "HEAB",
            "MONR",
            "MONR2",
            "SOWM",
            "INFO",
            "TRCM",
            "ACCM",
            "TREO",
            "EXAC",
            "CATA",
            "SYPM",
            "MTSP"]

ISOCustomTests = ["OSEM", "OSTM", "TRAJ", "STRT", "OSTM"]

def iso_test():
    print("isotest")
    I = ISO("127.0.0.1")

    for t in ISOCustomTests:

        print("\n Press enter to test {}".format(t))
        s = input()

        if (t == "MONR"):
            I.MONR()
        if (t == "TREO"):
            I.StringTest()
        if (t == "OSEM"):
            I.OSEM()
        if (t == "OSTM"):
            I.OSTM()
        if (t == "STRT"):
            I.STRT()
        if (t == "HEAB"):
            I.HEAB()
        if (t == "TRCM"):
            I.TRCM()
        if (t == "ACCM"):
            I.ACCM()
        if (t == "TRAJ"):
            I.TRAJ()

    I.shutdown()
    return 0

def MSCP_test():
    print("mscptest")
    S = Executable("../build/TEServer", ["-m", "0"])
    time.sleep(0.05)

    M = MSCP("127.0.0.1")
    time.sleep(1)
    for t in MSCPTests:
        print("\n Press enter to test {}".format(t))
        s = input()

        if (t == "INIT"):
            M.Init()

        if (t == "CONNECTOBJECT"):
            M.Connect()

        if (t == "ARM"):
            M.Arm()

        if (t == "START"):
            M.Start(0)

        if (t == "ABORT"):
            M.Abort()

        if (t == "GETSERVERSTATUS"):
            M.GetStatus()

        if (t == "CREATEOBJECTS"):
            M.CreateObjects(1)

        if (t == "DISARM"):
            M.Disarm()

        if (t == "GETSERVERTIME"):
            M.GetServerTime()

        # if(t == "UPLOADFILE"):
        #    M.UploadFile("","")

        if (t == "DOWNLOADFILE"):
            M.DownloadFile()

        if (t == "DELETEFILE"):
            M.DeleteFile()

        if (t == "CHECKFILEEXISTS"):
            M.CheckFileExists()

        if (t == "REMOTECONTROL"):
            M.RemoteControl()

        if (t == "SETSERVERPARAMETER"):
            M.SetServerParameter()

        if (t == "GETSERVERPARAMETERS"):
            M.GetServerParameters()

        if (t == "GETDIRECTORYCONTENT"):
            M.GetDirectoryContent()

        if (t == "DISCONNECTOBJECT"):
            M.Disconnect()

        print("Checking server...")
        time.sleep(1)
        if S.poll():
            S.stop()
            M.shutdown()
            sys.exit(1)

    M.shutdown()
    S.stop()
    return 0

if __name__ == "__main__":

    print("Choose what you would like to test: \n MSCP(1) \n ISO(2) \n:")
    ans = input()

    if(ans == "1" or ans == "MSCP"):
        MSCP_test()

    elif (ans == "2" or ans == "ISO"):
        iso_test()
    sys.exit(1)




