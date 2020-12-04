import socket
import sys, select
import threading
import re
import time
import encodings
import asyncio
from threading import Thread



def as_hex_little_endian(value):
    value = format(int(value),'x').rjust(8,'0')
    value = [value[i:i+2] for i in range(len(value)-2, -1, -2)]
    return ''.join(value)

class ISO:

    udpPort = 57074
    tcpPort = 54242

    def __init__(self,host="127.0.0.1",port=54241):
        self.host = host
        self.port = port
        
        try:
            self.tcpSocket = socket.socket()
            print("=== ISO connecting to " + str(self.host) + ":" + str(self.port))
            self.tcpSocket.connect((self.host,self.port))
        except:
            print("TCP port: {} is already in use. Continuing...".format(self.tcpPort))

        try:
            print("=== Creating a UDP socket")
            self.udpSocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.udpSocket.bind((self.host, 53240))
        except:
            print("UDP port: {} is already in use. Continuing...".format(self.udpPort))
        



    def StringTest(self):    
        message = bytearray.fromhex("48454c4c4f")
        header = bytearray.fromhex("7e7e00000213002200000080001e00")
        #messageLength = bytearray.fromhex(hex(len(message)))
        #header.extend(messageLength)
        header.extend(message)
        print(header)
        self.SendRawUDP(header)
        print("=== TREO() sent")

    def MONR(self, timestamp=None, position=None, heading_deg=None):
        if not position:
            position = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        if not heading_deg:
            heading_deg = 0.0
        if not timestamp:
            timestamp = "80001e00"
        position['x'] = as_hex_little_endian(position['x']*1000)
        position['y'] = as_hex_little_endian(position['y']*1000)
        position['z'] = as_hex_little_endian(position['z']*1000)
        heading_deg = as_hex_little_endian(heading_deg*100)
        return bytearray.fromhex("7e7e004602060022000000" + str(timestamp) + position['x'] + position['y'] + position['z'] + str(heading_deg) + "0000f93e0000000000000000000301000000")
    
    def HEAB(self):         
        message = bytearray.fromhex("7e7e0000020500090000009000050083c00d4a010000") 
        self.SendRawUDP(message)
        print("=== HEAB() sent")

    def OSEM(self):         
        message = bytearray.fromhex("7e7e00000202004400000020000600d084fd85860021000600f82fc1c11d00220004006f4b0000040004001b3b3401030002002d0802000400629f0d4a70000200ffff72000200ffff7400020000000000") 
        self.SendRawUDP(message)
        print("=== OSEM() sent")

    def OSTM(self):         
        message = bytearray.fromhex("7e7e00000203000500000064000100020000") 
        self.SendRawUDP(message)
        print("=== OSTM() sent")

    def STRT(self):         
        message = bytearray.fromhex("7e7e00000204000e0000000200040038131e4a030002002d080000") 
        self.SendRawUDP(message)
        print("=== STRT() sent")

    def TRAJ(self):         
        message = bytearray.fromhex("7e7e0000020100387f00000101020000000201400047617261676552656b74616e67656c496e726500000000000000000000000000000000000000000000000000000000000000000000000000000000000000000003010200000000a0040000000000") 
        self.SendRawUDP(message)
        print("=== TRAJ() sent")

    def TRCM(self):         
        message = bytearray.fromhex("7e7e00000211002400000001000200000002000200e000110004002200000012000400ffffffff13000400ffffffff0000") 
        self.SendRawUDP(message)
        print("=== TRCM() sent")        
    
    def ACCM(self):         
        message = bytearray.fromhex("7e7e00000212002400000002000200000003000200e000a100040001000000a2000400ffffffffa3000400ffffffff0000") 
        self.SendRawUDP(message)
        print("=== ACCM() sent")

    def TREO(self):         
        message = bytearray.fromhex("7e7e0000020100387f00000101020000000201400047617261676552656b74616e67656c496e726500000000000000000000000000000000000000000000000000000000000000000000000000000000000000000003010200000000a0040000000000") 
        self.SendRawUDP(message)
        print("=== TREO() sent")


    def SendTCP(self,message):
        self.tcpSocket.send(message.encode())

    def SendUDP(self, message):
        self.udpSocket.sendto(bytes(message, "utf-8"), (self.host, self.udpPort))

    def SendRawUDP(self, message):
        self.udpSocket.sendto(message, (self.host, self.udpPort))

    def shutdown(self):
        self.quit = True
        self.tcpSocket.close()

class ISOObject(ISO):
        
    class ObjectProcessChannel:
        def __init__(self,host='localhost'):
            self.port = 53240
            self.host = host
            self.remoteAddr = None
            t = Thread(target=self.create_connection, name='prc-thread')
            t.start()

        def create_connection(self):
            print("=== Creating a UDP socket")
            self.udpSocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.udpSocket.bind((self.host, self.port))
            while True:
                data, remoteAddr = self.udpSocket.recvfrom(2048)
                self.remoteAddr = remoteAddr

        def send(self,message):
            if not self.remoteAddr:
                raise ConnectionError("=== No remote UDP port known for sending on process channel")
            if type(message) == str:
                message = bytes(message, "utf-8")
            self.udpSocket.sendto(message, self.remoteAddr)

        def close(self):
            return self.udpSocket.close()

    class ObjectCommandChannel:
        def __init__(self,host='localhost'):
            self.port = 53241
            self.host = host
            self.transport = None
            self.remoteAddr = None
            self.tcpSocket = None
            t = Thread(target=self.create_connection,name="cmd-thread")
            t.start()

        def create_connection(self):
            self.tcpSocket = socket.socket()
            print("=== ISO connecting to " + str(self.host) + ":" + str(self.port))
            self.tcpSocket.bind((self.host,self.port))
            self.tcpSocket.listen()
            self.transport, self.remoteAddr = self.tcpSocket.accept()
            print("Transport " + str(self.transport) + ", remote: " + str(self.remoteAddr))

        def close(self):
            return self.tcpSocket.close()


    def __init__(self):
        self.host = 'localhost'
        self.commandChannel = self.ObjectCommandChannel(self.host)
        self.processChannel = self.ObjectProcessChannel(self.host)

    def MONR(self, timestamp=None, position=None, heading_deg=None):
        return self.processChannel.send(ISO.MONR(self,timestamp,position,heading_deg))

    def isConnected(self):
        return self.processChannel.remoteAddr != None

    def shutdown(self):
        self.commandChannel.close()
        self.processChannel.close()


if __name__ == "__main__":
    obj = ISOObject()
    while not obj.isConnected():
        pass
    print("Sending a bunch of MONR")
    for i in range(10000):
        obj.MONR(position={'x':10.0,'y':15.0,'z':4.5})
        time.sleep(0.01)
    obj.shutdown()
    exit(1)