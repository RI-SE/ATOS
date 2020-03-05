import socket
import sys, select
import threading
import re
import time
import encodings

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

    def MONR(self):         
        message = bytearray.fromhex("7e7e00460206002200000080001e008140384600710000f983000000000000f93e0000000000000000000301000000") 
        self.SendRawUDP(message)
        print("=== MONR() sent")
    
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