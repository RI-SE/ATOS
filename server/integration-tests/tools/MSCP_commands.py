import socket
import time
import sys, select

def getServerStatus():         
        message = "POST /maestro HTTP/1.1\r\nHost: 127.0.0.1\r\n\r\nGetServerStatus();"    
        
        mySocket.send(message.encode())
        print("GetServerStatus() Sent")

 
def Abort():         
        message = "POST /maestro HTTP/1.1\r\nHost: 127.0.0.1\r\n\r\nAbortScenario();"    
        
        mySocket.send(message.encode())
        print("Abort() Sent")


def Arm():         
        message = "POST /maestro HTTP/1.1\r\nHost: 127.0.0.1\r\n\r\nArmScenario();"    
        
        mySocket.send(message.encode())
        print("ArmScenario() Sent")


def Init():       
        message = "POST /maestro HTTP/1.1\r\nHost: 127.0.0.1\r\n\r\nInitializeScenario();"
         
        mySocket.send(message.encode())
        print("Init() Sent")


def Connect():

        message = "POST /maestro HTTP/1.1\r\nHost: 127.0.0.1\r\n\r\nConnectObject();"
         
        mySocket.send(message.encode())
        print("Connect() Sent")

                 
def StartScenario():       
        message = "POST /maestro HTTP/1.1\r\nHost: 127.0.0.1\r\n\r\nStartScenario(5);"
         
        mySocket.send(message.encode())
        print("StarScenario() Sent")               
      

host = '10.130.24.54' ## add relevant IP here
port = 54241
         
mySocket = socket.socket()
mySocket.connect((host,port))


counter = 0

while True:
        print(counter)
        if (counter == 2):
                Init()
        if (counter == 5):
                Connect()
        if (counter == 8):
                Arm()
        if (counter == 10):
                StartScenario()
        if (counter == 15):
                Abort()
        else:
                time.sleep(1)
                getServerStatus()
        counter += 1
