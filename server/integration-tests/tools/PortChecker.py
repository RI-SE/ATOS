import subprocess, signal
import time
import socket
import sys

host = ''

def WaitForPortAvailable(port,protocol,timeout=5):
    print("=== Checking " + str(protocol) + " port " + str(port))
    if protocol == "UDP":
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    elif protocol == "TCP":
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    else:
        raise ValueError("The only supported network protocols are TCP and UDP")
    
    hasPrinted = False
    timeoutTime = time.time() + timeout
    while True:
        try:
            s.bind((host,port))
            s.close()
            return
        except OSError:
            if not hasPrinted:
                print("=== Port " + str(port) + " occupied, waiting for " + str(timeout) + " seconds")
                hasPrinted = True
            time.sleep(0.25)
            if time.time() > timeoutTime:
                break
    raise TimeoutError("Timed out while waiting for port " + str(port) + " to become available")
