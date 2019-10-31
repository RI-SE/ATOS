import socket
import sys, select
import threading
import re
import time

class MSCP:
    def __init__(self,host,port=54241):
        self.host = host
        self.port = port
        self.socket = socket.socket()
        self.socket.connect((self.host,self.port))
        self.lastStatusReply = {}
        self.lastAbortReply = {}
        self.lastStartReply = {}
        self.lastStatusReply["objectControlState"] = "UNKNOWN"
        self.lastStatusReply["systemControlState"] = "UNKNOWN"
        self.lastStatusReply["systemControlErrorCode"] = 0
        self.lastStatusReply["objectControlErrorCode"] = 0
        self.lastAbortReply["scenarioActive"] = 0
        self.lastStartReply["scenarioActive"] = 0
        self.quit = False
        self.lastResponseCode = "No message received yet"
        self.listener = threading.Thread(target=self.listen)
        self.listener.start()

    def listen(self):
        replyPatterns = [
                {"command": "init", "regex": re.compile(b'(..)InitializeScenario:')},
                {"command": "status", "regex": re.compile(b'(..)GetServerStatus:(.)(.)(.)?(.)?')},
                {"command": "abort", "regex": re.compile(b'(..)AbortScenario:(.)')},
                {"command": "arm", "regex": re.compile(b'(..)ArmScenario:')},
                {"command": "start", "regex": re.compile(b'(..)StartScenario:(.)')},
                {"command": "connect", "regex": re.compile(b'(..)ConnectObject:')},
                {"command": "disconnect", "regex": re.compile(b'(..)DisconnectObject:')}
        ]

        while not self.quit:
            data = self.socket.recv(1024)
            for replyPattern in replyPatterns:
                match = re.search(replyPattern["regex"],data)
                if match is not None:
                    matchPattern = replyPattern
                    self.lastResponseCode = self.interpretResponseCode(match.group(1))
                    break
            if match is not None:
                if matchPattern["command"] == "init":
                    print("Init reply received")
                if matchPattern["command"] == "status":
                    print("Status reply received")
                    num = int.from_bytes(match.group(2),byteorder='big')
                    if num == 1:
                        self.lastStatusReply["systemControlState"] = "INITIALIZED"
                    elif num == 2:
                        self.lastStatusReply["systemControlState"] = "IDLE"
                    elif num == 5:
                        self.lastStatusReply["systemControlState"] = "INWORK"
                    elif num == 6:
                        self.lastStatusReply["systemControlState"] = "ERROR"
                    else:
                        self.lastStatusReply["systemControlState"] = "UNKNOWN"

                    num = int.from_bytes(match.group(3),byteorder='big')
                    if num == 1:
                        self.lastStatusReply["objectControlState"] = "IDLE"
                    elif num == 2:
                        self.lastStatusReply["objectControlState"] = "INITIALIZED"
                    elif num == 3:
                        self.lastStatusReply["objectControlState"] = "CONNECTED"
                    elif num == 4:
                        self.lastStatusReply["objectControlState"] = "ARMED"
                    elif num == 5:
                        self.lastStatusReply["objectControlState"] = "RUNNING"
                    elif num == 6:
                        self.lastStatusReply["objectControlState"] = "ERROR"
                    else:
                        self.lastStatusReply["objectControlState"] = "UNKNOWN"

                    if match.group(3) is not None:
                        self.lastStatusReply["systemControlErrorCode"] = int.from_bytes(match.group(4),byteorder='big')
                    else:
                        self.lastStatusReply["systemControlErrorCode"] = 0
                    if match.group(4) is not None:
                        self.lastStatusReply["objectControlErrorCode"] = int.from_bytes(match.group(5),byteorder='big')
                    else:
                        self.lastStatusReply["objectControlErrorCode"] = 0
                if matchPattern["command"] == "abort":
                    print("Abort reply received")
                    num = int.from_bytes(match.group(2),byteorder='big')
                    if num == 0:
                        self.lastAbortReply["scenarioActive"] = "NOT_ACTIVE"
                    elif num == 1:
                        self.lastAbortReply["scenarioActive"] = "ACTIVE"
                    else:
                        self.lastAbortReply["scenarioActive"] = "UNKNOWN"
                if matchPattern["command"] == "arm":
                    print("Arm reply received")
                if matchPattern["command"] == "start":
                    print("Start reply received")
                    num = int.from_bytes(match.group(2),byteorder='big')
                    if num == 0:
                        self.lastStartReply["scenarioActive"] = "NOT_ACTIVE"
                    elif num == 1:
                        self.lastStartReply["scenarioActive"] = "ACTIVE"
                    else:
                        self.lastStartReply["scenarioActive"] = "UNKNOWN"
                if matchPattern["command"] == "connect":
                    print("Connect reply received")
                if matchPattern["command"] == "disconnect":
                    print("Disconnect reply received")
                

    def GetStatus(self):         
        message = "POST /maestro HTTP/1.1\r\nHost: " + self.host + "\r\n\r\nGetServerStatus();"    
        self.Send(message)
        print("GetServerStatus() sent")
 
    def Abort(self):         
        message = "POST /maestro HTTP/1.1\r\nHost:" + self.host + "\r\n\r\nAbortScenario();"
        self.Send(message)
        print("Abort() sent")

    def Arm(self):         
        message = "POST /maestro HTTP/1.1\r\nHost:" + self.host + "\r\n\r\nArmScenario();"    
        self.Send(message)
        print("ArmScenario() sent")

    def Init(self):       
        message = "POST /maestro HTTP/1.1\r\nHost:" + self.host + "\r\n\r\nInitializeScenario();"
        self.Send(message)
        print("Init() sent")

    def Connect(self):
        message = "POST /maestro HTTP/1.1\r\nHost:" + self.host + "\r\n\r\nConnectObject();"
        self.Send(message)
        print("Connect() sent")
                 
    def Disconnect(self):
        message = "POST /maestro HTTP/1.1\r\nHost:" + self.host + "\r\n\r\nDisconnectObject();"
        self.Send(message)
        print("Disonnect() sent")
    
    def Start(self,delayTime_ms):       
        message = "POST /maestro HTTP/1.1\r\nHost:" + self.host + "\r\n\r\nStartScenario(" + str(delayTime_ms) + ");"
        self.Send(message)
        print("StarScenario() sent")               

    def Send(self,message):
        self.socket.send(message.encode())

    def shutdown(self):
        self.quit = True
        self.socket.close()

    def waitForObjectControlState(self,state,timeout=3.0):
        timeoutTime = time.time() + timeout
        while self.lastStatusReply["objectControlState"] != state and time.time() < timeoutTime:
            time.sleep(0.005)
            print("Expecting: " + state + ", Current: " + self.lastStatusReply["objectControlState"])
            self.GetStatus()
        print("Expecting: " + state + ", Current: " + self.lastStatusReply["objectControlState"])
        if self.lastStatusReply["objectControlState"] != state:
            raise TimeoutError("Timed out while waiting for transition to " + state)

    def interpretResponseCode(self,code):
        num = int.from_bytes(code,byteorder='big')
        if num == 0x0001:
            return "OK"
        elif num == 0x0002:
            return "Logging data"
        elif num == 0x0F10:
            return "Error"
        elif num == 0x0F20:
            return "Function not available"
        elif num == 0x0F25:
            return "Incorrect state"
        elif num == 0x0F30:
            return "Invalid length"
        elif num == 0x0F40:
            return "Busy"
        elif num == 0x0F42:
            return "Timeout"
        elif num == 0x0F50:
            return "Invalid script"
        elif num == 0x0F60:
            return "Invalid encryption code"
        elif num == 0x0F61:
            return "Decryption error"
        elif num == 0x0F62:
            return "No data"
        else:
            raise ValueError("Response code " + str(num) " is not recognized")

