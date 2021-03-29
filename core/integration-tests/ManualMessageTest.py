from tools.MSCP import MSCP
from tools.ISO import ISO
from tools.Executable import Executable
import time
import subprocess
import sys
import getopt

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
             "DISCONNECTOBJECT"]

MSCPCustomTests = [
    "INIT",
    "CONNECTOBJECT",
    "ARM",
    "START",
    "ABORT",
    "DISARM",
    "DISCONNECTOBJECT"
]

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


def old_run():
    S = Executable("../build/TEServer", ["-m", "0"])
    time.sleep(0.05)
    print("Choose what you would like to test: \n MSCP(1) \n ISO(2) \n:")
    ans = input()

    if (ans == "1" or ans == "MSCP"):

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

    elif (ans == "2" or ans == "ISO"):

        I = ISO("127.0.0.1")

        for t in ISOTests:

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

    S.stop()
    sys.exit(1)


def printHelpText(sysArgs):
    print("""usage: %s [OPTION]...
Manual python test of Maestro.

Possible OPTION values are listed below.
	-h, --help           print this text and exit
	""" % sysArgs[0])


def parseArguments(sysArgs):
    try:
        opts, args = getopt.getopt(sysArgs[1:], 'hr:', ["help","remote"])
    except getopt.error as msg:
        sys.stdout = sys.stderr
        print(msg)
        printHelpText(sysArgs)
        sys.exit(0)

    return_val = {
        "ip": "localhost",
        "remote": False
    }

    for o, a in opts:
        print(o)
        if o in ("-h", "--help"):
            printHelpText(args)
            sys.exit(0)
        if o in ("-r", "--remote"):
            return_val["remote"] = True
            return_val["ip"] = a

    return return_val


def connect_maestro(opts):
    m_process = None
    if not opts["remote"]:
        m_process = Executable("../../build/bin/Core", ["-m", "0"])
        time.sleep(0.05)

    m_connection = MSCP(opts["ip"])
    return m_process, m_connection

def shutdown(app, conn):
    if app:
        app.stop()
    if conn:
        conn.shutdown()
    sys.exit(0)

def user_execute(app, m_conn, test):
    for t in test:
        print("\n Press enter to test {}".format(t))
        s = input()

        if (t == "INIT"):
            m_conn.Init()

        if (t == "CONNECTOBJECT"):
            m_conn.Connect()

        if (t == "ARM"):
            m_conn.Arm()

        if (t == "START"):
            m_conn.Start(0)

        if (t == "ABORT"):
            m_conn.Abort()

        if (t == "GETSERVERSTATUS"):
            m_conn.GetStatus()

        if (t == "CREATEOBJECTS"):
            m_conn.CreateObjects(1)

        if (t == "DISARM"):
            m_conn.Disarm()

        if (t == "GETSERVERTIME"):
            m_conn.GetServerTime()

        # if(t == "UPLOADFILE"):
        #    m_conn.UploadFile("","")

        if (t == "DOWNLOADFILE"):
            m_conn.DownloadFile()

        if (t == "DELETEFILE"):
            m_conn.DeleteFile()

        if (t == "CHECKFILEEXISTS"):
            m_conn.CheckFileExists()

        if (t == "REMOTECONTROL"):
            m_conn.RemoteControl()

        if (t == "SETSERVERPARAMETER"):
            m_conn.SetServerParameter()

        if (t == "GETSERVERPARAMETERS"):
            m_conn.GetServerParameters()

        if (t == "GETDIRECTORYCONTENT"):
            m_conn.GetDirectoryContent()

        if (t == "DISCONNECTOBJECT"):
            m_conn.Disconnect()

        if app:
            print("Checking server...")
            time.sleep(1)
            if app.poll():
                app.stop()
                m_conn.shutdown()
                sys.exit(1)


if __name__ == "__main__":
    opts = parseArguments(sys.argv)
    app, conn = connect_maestro(opts)
    user_execute(app, conn, MSCPCustomTests)
    shutdown(app, conn)
