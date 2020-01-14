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
        print("=== MSCP connecting to " + str(self.host) + ":" + str(self.port))
        self.socket.connect((self.host,self.port))
        self.lastStatusReply = {}
        self.lastAbortReply = {}
        self.lastStartReply = {}
        self.lastUploadReply = {}
        self.lastStatusReply["objectControlState"] = "UNKNOWN"
        self.lastStatusReply["systemControlState"] = "UNKNOWN"
        self.lastStatusReply["systemControlErrorCode"] = 0
        self.lastStatusReply["objectControlErrorCode"] = 0
        self.lastAbortReply["scenarioActive"] = 0
        self.lastStartReply["scenarioActive"] = 0
        self.lastUploadReply["status"] = "UNKNOWN"
        self.uploadReplyLock = threading.Lock()
        self.startReplyLock = threading.Lock()
        self.abortReplyLock = threading.Lock()
        self.statusReplyLock = threading.Lock()
        self.responseCodeLock = threading.Lock()
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
                {"command": "disconnect", "regex": re.compile(b'(..)DisconnectObject:')},
                {"command": "upload", "regex": re.compile(b'([^u][^b])UploadFile:(.)')},
                {"command": "subupload", "regex": re.compile(b'(..)SubUploadFile:(.)')}
        ]

        print("=== Starting listener on " + str(self.host) + ":" + str(self.port))
        while not self.quit:
            try:
                data = self.socket.recv(1024)
            except ConnectionResetError as e:
                if not self.quit:
                    raise e
            
            print("Received: " + str(len(data)))
            for replyPattern in replyPatterns:
                match = re.search(replyPattern["regex"],data)
                if match is not None:
                    matchPattern = replyPattern
                    self.responseCodeLock.acquire()
                    self.lastResponseCode = self.interpretResponseCode(match.group(1))
                    self.responseCodeLock.release()
                    break
            if match is not None:
                if matchPattern["command"] == "init":
                    print("=== Init reply received")
                if matchPattern["command"] == "status":
                    print("=== Status reply received")
                    num = int.from_bytes(match.group(2),byteorder='big')
                    self.statusReplyLock.acquire()
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

                    if match.group(4) is not None:
                        self.lastStatusReply["systemControlErrorCode"] = int.from_bytes(match.group(4),byteorder='big')
                    else:
                        self.lastStatusReply["systemControlErrorCode"] = 0
                    if match.group(5) is not None:
                        self.lastStatusReply["objectControlErrorCode"] = int.from_bytes(match.group(5),byteorder='big')
                    else:
                        self.lastStatusReply["objectControlErrorCode"] = 0
                    self.statusReplyLock.release()
                if matchPattern["command"] == "abort":
                    print("=== Abort reply received")
                    num = int.from_bytes(match.group(2),byteorder='big')
                    self.abortReplyLock.acquire()
                    if num == 0:
                        self.lastAbortReply["scenarioActive"] = "NOT_ACTIVE"
                    elif num == 1:
                        self.lastAbortReply["scenarioActive"] = "ACTIVE"
                    else:
                        self.lastAbortReply["scenarioActive"] = "UNKNOWN"
                    self.abortReplyLock.release()
                if matchPattern["command"] == "arm":
                    print("=== Arm reply received")
                if matchPattern["command"] == "start":
                    print("=== Start reply received")
                    num = int.from_bytes(match.group(2),byteorder='big')
                    self.startReplyLock.acquire()
                    if num == 0:
                        self.lastStartReply["scenarioActive"] = "NOT_ACTIVE"
                    elif num == 1:
                        self.lastStartReply["scenarioActive"] = "ACTIVE"
                    else:
                        self.lastStartReply["scenarioActive"] = "UNKNOWN"
                    self.startReplyLock.release()
                if matchPattern["command"] == "connect":
                    print("=== Connect reply received")
                if matchPattern["command"] == "disconnect":
                    print("=== Disconnect reply received")
                if matchPattern["command"] == "upload":
                    print("=== Upload reply 1 received")
                    num = int.from_bytes(match.group(2),byteorder='big')
                    self.uploadReplyLock.acquire()
                    if num == 0x01:
                        self.lastUploadReply["status"] = "SERVER_PREPARED"
                    elif num == 0x02:
                        self.lastUploadReply["status"] = "PACKET_SIZE_ERROR"
                    elif num == 0x03:
                        self.lastUploadReply["status"] = "INVALID_PATH"
                    elif num == 0x04:
                        self.lastUploadReply["status"] = "UPLOAD_SUCCESS"
                    elif num == 0x05:
                        self.lastUploadReply["status"] = "UPLOAD_FAILURE"
                    else:
                        self.lastUploadReply["status"] = "UNKNOWN"
                    self.uploadReplyLock.release()
                if matchPattern["command"] == "subupload":
                    print("=== Upload reply 2 received")
                    num = int.from_bytes(match.group(2),byteorder='big')
                    self.uploadReplyLock.acquire()
                    if num == 0x04:
                        self.lastUploadReply["status"] = "UPLOAD_SUCCESS"
                    elif num == 0x05:
                        self.lastUploadReply["status"] = "UPLOAD_FAILURE"
                    else:
                        self.lastUploadReply["status"] = "UNKNOWN"
                    self.uploadReplyLock.release()
            else:
                print("=== Unable to match against data: " + str(data))

    def GetStatus(self):         
        message = "POST /maestro HTTP/1.1\r\nHost: " + self.host + "\r\n\r\nGetServerStatus();\r\n\r\n"    
        self.Send(message)
        print("=== GetServerStatus() sent")
 
    def Abort(self):         
        message = "POST /maestro HTTP/1.1\r\nHost:" + self.host + "\r\n\r\nAbortScenario();\r\n\r\n"
        self.Send(message)
        print("=== Abort() sent")

    def Arm(self):         
        message = "POST /maestro HTTP/1.1\r\nHost:" + self.host + "\r\n\r\nArmScenario();\r\n\r\n"    
        self.Send(message)
        print("=== ArmScenario() sent")

    def Init(self):       
        message = "POST /maestro HTTP/1.1\r\nHost:" + self.host + "\r\n\r\nInitializeScenario();\r\n\r\n"
        self.Send(message)
        print("=== Init() sent")

    def Connect(self):
        message = "POST /maestro HTTP/1.1\r\nHost:" + self.host + "\r\n\r\nConnectObject();\r\n\r\n"
        self.Send(message)
        print("=== Connect() sent")
                 
    def Disconnect(self):
        message = "POST /maestro HTTP/1.1\r\nHost:" + self.host + "\r\n\r\nDisconnectObject();\r\n\r\n"
        self.Send(message)
        print("=== Disconnect() sent")
    
    def Start(self,delayTime_ms):       
        message = "POST /maestro HTTP/1.1\r\nHost:" + self.host + "\r\n\r\nStartScenario(" + str(delayTime_ms) + ");\r\n\r\n"
        self.Send(message)
        print("=== StartScenario() sent")

    def UploadFile(self,targetPath,fileContents):
        packetSize = 1200
        message = "POST /maestro HTTP/1.1\r\nHost:" + self.host + "\r\n\r\nUploadFile(" + targetPath + "," + str(len(fileContents)) + "," + str(packetSize) + ");\r\n\r\n" 
        self.uploadReplyLock.acquire()
        self.lastUploadReply["status"] = "UNKNOWN"
        self.uploadReplyLock.release()
        self.Send(message)
        print("=== UploadFile() sent")
        self.waitForUploadReply("SERVER_PREPARED")
        print("=== Sending file contents")
        # Send file
        self.Send(fileContents)
        print("=== Sent file contents")
        self.uploadReplyLock.acquire()
        self.lastUploadReply["status"] = "UNKNOWN"
        self.uploadReplyLock.release()
        self.waitForUploadReply("UPLOAD_SUCCESS")
        print("=== File uploaded")

    def Send(self,message):
        self.socket.send(message.encode())

    def shutdown(self):
        self.quit = True
        self.socket.close()

    def waitForUploadReply(self,status,timeout=3.0):
        timeoutTime = time.time() + timeout
        self.uploadReplyLock.acquire()
        ur = self.lastUploadReply["status"]
        self.uploadReplyLock.release()
        print("=== Entering waiting state")
        while ur == "UNKNOWN" and time.time() < timeoutTime:
            self.uploadReplyLock.acquire()
            ur = self.lastUploadReply["status"]
            self.uploadReplyLock.release()
        
        print("=== Exited waiting state")
        if ur != status and time.time() >= timeoutTime:
            raise TimeoutError("Timed out while waiting for reply to UploadFile")
        elif ur != status:
            raise ValueError("Expected status " + status + " but received " + ur)

    def waitForObjectControlState(self,state,timeout=3.0):
        timeoutTime = time.time() + timeout
        self.statusReplyLock.acquire()
        sr = self.lastStatusReply["objectControlState"]
        self.statusReplyLock.release()
        while sr != state and time.time() < timeoutTime:
            time.sleep(0.005)
            self.statusReplyLock.acquire()
            sr = self.lastStatusReply["objectControlState"]
            self.statusReplyLock.release()
            print("=== Expecting: " + state + ", Current: " + sr)
            self.GetStatus()
        print("=== Expecting: " + state + ", Current: " + sr)
        if sr != state:
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
            raise ValueError("Response code " + str(num) + " is not recognized")

