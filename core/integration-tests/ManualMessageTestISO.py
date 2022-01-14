from tools.ISO import ISO

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

if __name__ == "__main__":
    iso_test()