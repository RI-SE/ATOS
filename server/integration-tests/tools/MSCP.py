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
        self.listener = threading.Thread(target=self.listen)
        self.listener.start()

    def listen(self):
        replyPatterns = [
                {"command": "init", "regex": re.compile(b'InitializeScenario:')},
                {"command": "status", "regex": re.compile(b'GetServerStatus:(.)(.)(.)?(.)?')},
                {"command": "abort", "regex": re.compile(b'AbortScenario:(.)')},
                {"command": "arm", "regex": re.compile(b'ArmScenario:')},
                {"command": "start", "regex": re.compile(b'StartScenario:(.)')},
                {"command": "connect", "regex": re.compile(b'ConnectObject:')},
                {"command": "disconnect", "regex": re.compile(b'DisconnectObject:')}
        ]

        while not self.quit:
            data = self.socket.recv(1024)
            for replyPattern in replyPatterns:
                match = re.search(replyPattern["regex"],data)
                if match is not None:
                    matchPattern = replyPattern
                    break
            if match is not None:
                if matchPattern["command"] == "init":
                    print("Init reply received")
                if matchPattern["command"] == "status":
                    print("Status reply received")
                    num = int.from_bytes(match.group(1),byteorder='big')
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

                    num = int.from_bytes(match.group(2),byteorder='big')
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
                        self.lastStatusReply["systemControlErrorCode"] = int.from_bytes(match.group(3),byteorder='big')
                    else:
                        self.lastStatusReply["systemControlErrorCode"] = 0
                    if match.group(4) is not None:
                        self.lastStatusReply["objectControlErrorCode"] = int.from_bytes(match.group(4),byteorder='big')
                    else:
                        self.lastStatusReply["objectControlErrorCode"] = 0
                if matchPattern["command"] == "abort":
                    print("Abort reply received")
                    num = int.from_bytes(match.group(1),byteorder='big')
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
                    num = int.from_bytes(match.group(1),byteorder='big')
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

    def waitForObjectControlState(self,state):
        while self.lastStatusReply["objectControlState"] != state:
            time.sleep(0.005)
            print("Expecting: " + state + ", Current: " + self.lastStatusReply["objectControlState"])
            self.GetStatus()
        print("Expecting: " + state + ", Current: " + self.lastStatusReply["objectControlState"])

