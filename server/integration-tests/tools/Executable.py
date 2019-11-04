import threading
import subprocess, signal
import time

class Executable():
    def __init__(self, path, argList=[]):
        self.args = [path] + argList
        self.proc = subprocess.Popen(self.args)
        self.pids = []
        
        # Wait for a short time to allow the process to reach "steady-state"
        time.sleep(0.05)

        # Get PIDs of forks (if any)
        pgrep = subprocess.Popen(["pgrep","-P",str(self.proc.pid)],stdout=subprocess.PIPE)
        self.pids.append(self.proc.pid)
        while True:
            line = pgrep.stdout.readline()
            if not line:
                break
            self.pids.append(int(line))
        print("=== Started executable using the following PID(s): " + str(self.pids))

    # Get a list of all processes linked to the Executable which have died
    def poll(self):
        died = []
        if self.proc.poll() is not None:
            died.append(self.proc.pid)
        for pid in self.pids:
            # Kill -0 does not kill the process
            if int(subprocess.call(["kill","-0",str(pid)],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)):
                died.append(pid)
        return died
    
    # Kill all started processes
    def stop(self):
        procName = self.args[0].split("/")
        args = ["killall","-SIGINT",procName[-1]]
        subprocess.call(args)
        print("=== Sent interrupt signal to " + procName[-1])
        
        try:
            self.proc.wait(3)
            print("=== " + procName[-1] + " terminated successfully")
            return
        except TimeoutExpired as e:
            print("=== Executable did not respond to interrupt, forcing shutdown")
            args = ["killall",procName[-1]]
            subprocess.call(args)
            time.sleep(0.05)
            raise TimeoutExpired(e.message + " Forced shutdown for " + procName[-1])
