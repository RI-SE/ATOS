import threading
import subprocess, signal

class Maestro():
    def __init__(self, path, extraModules=0):
        subprocess.run("pwd")
        self.serverArgs = [path, "-m", str(extraModules)]
        self.proc = subprocess.Popen(self.serverArgs)

    def stop(self):
        procName = self.serverArgs[0].split("/")
        args = ["killall","-SIGINT",procName[-1]]
        subprocess.call(args)
