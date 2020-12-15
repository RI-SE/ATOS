import socket
import sys, select
import threading
import re
import time
import encodings
from threading import Thread



def as_hex_little_endian(value,expected_bytes):
    sign = lambda x: (1,-1)[x < 0]
    value = int(value)
    value_sign = sign(value)
    value = format(int(abs(value)),'x').rjust(expected_bytes*2,'0')
    if value_sign < 0:
        msb = int(value[0]) | 0b1000
        value = format(msb,'x') + value[1:]
    value = [value[i:i+2] for i in range(len(value)-2, -1, -2)]
    return ''.join(value)

class ISO:

    udpPort = 57074
    tcpPort = 54242
    ccStatus = {0: "init", 1: "ready", 2: "abort", 3: "running", 4: "testDone", 5: "normalStop"}

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

    def MONR(self, transmitter_id=None, timestamp=None, in_position=None, heading_deg=None, speed=None, acceleration=None, drive_direction=None, object_state=None, ready_to_arm=None, object_error_status=None):
        if not in_position:
            in_position = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        if heading_deg == None:
            heading_deg = 0.0
        if timestamp == None:
            timestamp = (time.time() + 18 - 315964800)*4000 % 2419200000
        if not speed:
            speed = {'lateral': 0.0, 'longitudinal': 0.0}
        if not acceleration:
            acceleration = {'lateral': 0.0, 'longitudinal': 0.0}
        if drive_direction == None:
            drive_direction = 0
        if object_state == None:
            object_state = 4
        if ready_to_arm == None:
            ready_to_arm = 1
        if object_error_status == None:
            object_error_status = 0
        if not transmitter_id:
            transmitter_id = 2

        position = {}
        timestamp = as_hex_little_endian(timestamp, 4)
        try:
            position['x'] = as_hex_little_endian(in_position['x']*1000, 4)
            position['y'] = as_hex_little_endian(in_position['y']*1000, 4)
        except KeyError:
            position['x'] = as_hex_little_endian(in_position[0]*1000, 4)
            position['y'] = as_hex_little_endian(in_position[1]*1000, 4)
            
        position['z'] = as_hex_little_endian(0, 4)
        try:
            position['z'] = as_hex_little_endian(in_position['z']*1000, 4)
        except KeyError:
            try:
                position['z'] = as_hex_little_endian(in_position[2]*1000, 4)
            except (IndexError, KeyError):
                pass
        heading_deg = as_hex_little_endian(((90 - heading_deg + 360) % 360)*100, 2)
        speed['longitudinal'] = as_hex_little_endian(speed['longitudinal'], 2)
        speed['lateral'] = as_hex_little_endian(speed['lateral'], 2)
        acceleration['longitudinal'] = as_hex_little_endian(acceleration['longitudinal'], 2)
        acceleration['lateral'] = as_hex_little_endian(acceleration['lateral'], 2)
        drive_direction = as_hex_little_endian(drive_direction, 1)
        object_state = as_hex_little_endian(object_state, 1)
        ready_to_arm = as_hex_little_endian(ready_to_arm, 1)
        object_error_status = as_hex_little_endian(object_error_status, 1)
        transmitter_id = as_hex_little_endian(transmitter_id, 1)
        
        retval = bytearray.fromhex("7e7e" + transmitter_id + "460206002200000080001e00" + timestamp + position['x'] + position['y'] + position['z'] + heading_deg
                    + speed['longitudinal'] + speed['lateral'] + acceleration['longitudinal'] + acceleration['lateral'] + drive_direction + object_state
                    + ready_to_arm + object_error_status + "0000")
        return retval
    
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
            self.lastHEAB = None
            self.quit = False
            self.thread = Thread(target=self.create_connection, name='prc-thread')
            self.thread.start()

        def create_connection(self):
            print("=== Creating a UDP socket")
            self.udpSocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.udpSocket.bind((self.host, self.port))
            while not self.quit:
                data, remoteAddr = self.udpSocket.recvfrom(2048)
                self.remoteAddr = remoteAddr
                self.lastHEAB = data

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
            self.quit = False
            self.thread = Thread(target=self.create_connection,name="cmd-thread")
            self.thread.start()

        def create_connection(self):
            self.tcpSocket = socket.socket()
            print("=== ISO connecting to " + str(self.host) + ":" + str(self.port))
            self.tcpSocket.bind((self.host,self.port))
            self.tcpSocket.listen()
            self.transport, self.remoteAddr = self.tcpSocket.accept()

        def close(self):
            return self.tcpSocket.close()


    def __init__(self):
        self.host = 'localhost'
        self.commandChannel = self.ObjectCommandChannel(self.host)
        self.processChannel = self.ObjectProcessChannel(self.host)

    def MONR(self, transmitter_id=None, timestamp=None, position=None, heading_deg=None, speed=None, acceleration=None, drive_direction=None, object_state=None, ready_to_arm=None, object_error_status=None):
        return self.processChannel.send(ISO.MONR(self,transmitter_id,timestamp,position,heading_deg,speed,acceleration,drive_direction,object_state,ready_to_arm,object_error_status))

    def lastHEAB(self):
        return self.processChannel.lastHEAB

    def lastCCStatus(self):
        return ISO.ccStatus[self.lastHEAB()[-3]]


    def isConnected(self):
        return self.processChannel.remoteAddr != None

    def shutdown(self):
        self.commandChannel.quit = True
        self.processChannel.quit = True
        self.commandChannel.thread.join()
        self.commandChannel.close()
        self.processChannel.thread.join()
        self.processChannel.close()


if __name__ == "__main__":
    obj = ISOObject()
    while not obj.isConnected():
        pass
    print("Sending a bunch of MONR")
    for i in range(1000):
        obj.MONR(transmitter_id=2,position={'x':10.0,'y':15.0,'z':4.5})
        time.sleep(0.01)
    print("Last HEAB: " + str(obj.lastHEAB()))
    obj.shutdown()
    exit(1)