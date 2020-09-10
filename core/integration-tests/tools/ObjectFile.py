def ConstructObjectFileData(objectIP, trajectoryFileName, transmitterID):
    data = ""
    data = data + "IP="     + str(objectIP) + "\n"
    data = data + "traj="   + str(trajectoryFileName) + "\n"
    data = data + "ID="     + str(transmitterID) + "\n"
    
    return data

