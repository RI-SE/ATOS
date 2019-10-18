import threading
import subprocess, signal
import time

class Maestro():
    def __init__(self, path, extraModules=0):
        subprocess.run("pwd")
        self.serverArgs = [path, "-m", str(extraModules)]
        self.proc = subprocess.Popen(self.serverArgs)
        self.pids = [self.proc.pid]
        time.sleep(0.05)
        pgrep = subprocess.Popen(["pgrep","-P",str(self.proc.pid)],stdout=subprocess.PIPE)
        while True:
            line = pgrep.stdout.readline()
            if not line:
                break
            self.pids.append(int(line))

    def poll(self):
        died = 0
        for pid in self.pids:
            died = died or int(subprocess.call(["kill","-0",str(pid)]))
        return died

    def stop(self):
        procName = self.serverArgs[0].split("/")
        args = ["killall","-SIGINT",procName[-1]]
        subprocess.call(args)
