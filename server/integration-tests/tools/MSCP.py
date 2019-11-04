import socket
import sys, select

class MSCP:
    def __init__(self,host,port=54241):
        self.host = host
        self.port = port
        self.socket = socket.socket()
        self.socket.connect((self.host,self.port))
    
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
                 
    def Start(self,delayTime_ms):       
        message = "POST /maestro HTTP/1.1\r\nHost:" + self.host + "\r\n\r\nStartScenario(" + delayTime_ms + ");"
        self.Send(message)
        print("StarScenario() sent")               

    def Send(self,message):
        self.socket.send(message.encode())

