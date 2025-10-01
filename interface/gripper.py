#!/usr/bin/env python3

import socket
import time, threading, struct

class Gripper():
    def __init__(self, addr, port):
        self.SERVER_SOP = 0xAE
        self.SERVER_EOP = 0xAF

        self.client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client.connect((addr, port))
        print(f"Connected server : {(addr, port)}")
        self.connected = True

    def Init(self):
        code_str = "gripper.Init()"
        self.client.send(code_str.encode('utf-8'))
        self.recv_func()

    def Grip(self):
        code_str = "gripper.Grip()"
        self.client.send(code_str.encode('utf-8'))
        self.recv_func()

    def Release(self):
        code_str = "gripper.Release()"
        self.client.send(code_str.encode('utf-8'))
        self.recv_func()

    def Move(self, pos):
        code_str = f"gripper.Move({pos})"
        self.client.send(code_str.encode('utf-8'))
        self.recv_func()

    def recv_func(self):
        try:
            recv_packet = self.client.recv(1024)
            data = bytearray(recv_packet)
            recvByteLen = len(data)
            
            if recvByteLen >= 3:
                if recvByteLen == (data[1] + 3) and (data[0] == self.SERVER_SOP or data[-1] == self.SERVER_EOP):
                    self.gripper_width = struct.unpack("d", recv_packet[2:2+8])[0]
                    print(f"gripper_width : {self.gripper_width}")
        except socket.error as e:
            print(f"[error] socket error : {e}")
            self.client.close()
            self.connected = False
        
        time.sleep(0.1)

if __name__ == '__main__':
    gripper = Gripper("127.0.0.1", 5002)

    gripper.Init()
    print(f"gripper width : {gripper.gripper_width}")
    gripper.Grip()
    print(f"gripper width : {gripper.gripper_width}")
    gripper.Release()
    print(f"gripper width : {gripper.gripper_width}")

