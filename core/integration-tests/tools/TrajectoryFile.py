import random
from os import listdir
from os.path import isfile, join

def ReadTrajectoryFile(trajectoryDir, fileName="random"):
    return ReadFile(trajectoryDir, fileName, ".traj")


def ReadGeofenceFile(geofenceDir, fileName="random"):
    return ReadFile(geofenceDir, fileName, ".geofence")


def ReadFile(directory,fileName="random",fileEnding=".traj"):
    if directory[-1] != '/':
        directory += '/'
    allFiles = [f for f in listdir(directory) if isfile(join(directory, f))]
    allFiles[:] = [f for f in allFiles if f[-len(fileEnding):] == fileEnding]
    print(allFiles)
    if fileName == "random":
        fname = join(directory, random.choice(allFiles))
    elif fileName in allFiles:
        fname = join(directory, fileName)
    else:
        raise ValueError("File " + fileName + " in directory " + directory + " does not exist")
    print("=== Reading " + fileEnding + " file " + fname)
    with open(fname, "r") as fp:
        data = fp.read()
    return [data, fname.split('/')[-1]]

