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

def ConstructObjectFileData(objectIP, trajectoryFileName, transmitterID):
    data = ""
    data = data + "IP="     + str(objectIP) + "\n"
    data = data + "traj="   + str(trajectoryFileName) + "\n"
    data = data + "ID="     + str(transmitterID) + "\n"
    
    return data

def ConstructTrajectoryFileData(trajPoints=None,trajectoryName="Unnamed"):
    if trajPoints == None:
        nPoints = 0
    else:
        nPoints = len(trajPoints)

    data = ""
    data = data + "TRAJECTORY;0;" + str(trajectoryName) + ";1.0;" + str(nPoints) + ";\n"
    for trajPoint in trajPoints:
        data = data + "LINE;"
        data = data + str(trajPoint['time']) + ";"
        data = data + str(trajPoint['x']) + ";"
        data = data + str(trajPoint['y']) + ";"
        try:
            data = data + str(trajPoint['z']) + ";"
        except KeyError:
            data = data + "0.0;"
        try:
            data = data + str(trajPoint['heading']) + ";"
        except KeyError:
            data = data + "0.0;"
        try:
            data = data + str(trajPoint['longitudinal_speed']) + ";"
        except KeyError:
            data = data + "0.0;"
        try:
            data = data + str(trajPoint['lateral_speed']) + ";"
        except KeyError:
            data = data + ";"
        try:
            data = data + str(trajPoint['longitudinal_acceleration']) + ";"
        except KeyError:
            data = data + "0.0;"
        try:
            data = data + str(trajPoint['lateral_acceleration']) + ";"
        except KeyError:
            data = data + ";"
        try:
            data = data + str(trajPoint['curvature']) + ";"
        except KeyError:
            data = data + "0.0;"
        try:
            data = data + str(trajPoint['drive_mode']) + ";"
        except KeyError:
            data = data + "0;"
        data = data + "ENDLINE;\n"
    data = data + "ENDTRAJECTORY;"
    return data
    
def ConstructGeofenceFileData(geofencePoints=None,geofenceName="Unnamed",forbidden=True, minHeight=0, maxHeight=100):
    if geofencePoints == None:
        nPoints = 0
    else:
        nPoints = len(geofencePoints)

    data = ""
    data = data + "GEOFENCE;" + str(geofenceName) + ";" + str(nPoints) + ";"
    if forbidden:
        data = data + "forbidden;"
    else:
        data = data + "permitted;"
    data = data + str(minHeight) + ";" + str(maxHeight) + ";\n"
    for vertex in geofencePoints:
        data = data + "LINE;"
        try:
            data = data + str(vertex[0]) + ";"
        except TypeError:
            data = data + str(vertex['x']) + ";"
        try:
            data = data + str(vertex[1]) + ";"
        except TypeError:
            data = data + str(vertex['y']) + ";"
        data = data + "ENDLINE;\n"
    data = data + "ENDGEOFENCE;"
    return data