#!/usr/bin/env python3

import ctypes

class KetiZimmer:
    def __init__(self, so_file):
        self.lib = ctypes.cdll.LoadLibrary(so_file)
        
        self.lib.SetGripper.restype = ctypes.c_void_p
        self.module = self.lib.SetGripper()

    def Connect(self, ip='192.168.0.253', port=502):        
        self.lib.Connect.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_int]
        self.lib.Connect(self.module, ip.encode('utf-8'), port)

    def Disconnect(self):
        self.lib.Disconnect.argtypes = [ctypes.c_void_p]
        self.lib.Disconnect(self.module)

    def Init(self):
        self.lib.Init.argtypes = [ctypes.c_void_p]
        self.lib.Init(self.module)

    def Grip(self, sync=True):
        self.lib.Grip.argtypes = [ctypes.c_void_p, ctypes.c_bool]
        self.lib.Grip(self.module, sync)

    def Release(self, sync=True):
        self.lib.Release.argtypes = [ctypes.c_void_p, ctypes.c_bool]
        self.lib.Release(self.module, sync)

    def IsConnected(self):
        self.lib.IsConnected.argtypes = [ctypes.c_void_p]
        self.lib.IsConnected.restype = ctypes.c_bool
        return self.lib.IsConnected(self.module)
    
    def Move(self, position):
        self.lib.Move.argtypes = [ctypes.c_void_p, ctypes.c_uint16]
        self.lib.Move(self.module, position)

    def CurPos(self):
        self.lib.CurPos.argtypes = [ctypes.c_void_p]
        self.lib.CurPos.restype = ctypes.c_double
        return self.lib.CurPos(self.module)
    
    def CurSW(self):
        self.lib.CurSW.argtypes = [ctypes.c_void_p]
        self.lib.CurSW.restype = ctypes.c_int
        return self.lib.CurSW(self.module)

import time, os
if __name__ == '__main__':
    gripper = KetiZimmer(f'{os.getcwd()}/libzimmergripper.so')
    gripper.Connect('192.168.137.254', 502)
    gripper.Init()
    
    # gripper.Move(1000)
    gripper.Grip()
    print(gripper.CurPos())
    gripper.Release()
    print(gripper.CurPos())
    
    