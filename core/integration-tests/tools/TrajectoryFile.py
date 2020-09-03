import random
from os import listdir
from os.path import isfile, join

def ReadTrajectoryFile(trajectoryDir, fileName="random"):
    if trajectoryDir[-1] != '/':
        trajectoryDir += '/'
    allFiles = [f for f in listdir(trajectoryDir) if isfile(join(trajectoryDir, f))]
    allFiles[:] = [f for f in allFiles if f[-5:] == ".traj"]
    print(allFiles)
    if fileName == "random":
        fname = join(trajectoryDir, random.choice(allFiles))
    elif fileName in allFiles:
        fname = join(trajectoryDir, fileName)
    else:
        raise ValueError("File " + fileName + " in directory " + trajectoryDir + " does not exist")
    print("=== Reading trajectory file " + fname)
    with open(fname, "r") as fp:
        data = fp.read()    
    return [data, fname.split('/')[-1]]

