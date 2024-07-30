#!/usr/bin/env python3

import ctypes

class KetiZimmer:
    def __init__(self, so_file):
        print(so_file)
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

    def Grip(self):
        self.lib.Grip.argtypes = [ctypes.c_void_p]
        self.lib.Grip(self.module)

    def Release(self):
        self.lib.Release.argtypes = [ctypes.c_void_p]
        self.lib.Release(self.module)

    def IsAlive(self):
        self.lib.IsAlive.argtypes = [ctypes.c_void_p]
        self.lib.IsAlive.restype = ctypes.c_bool
        return self.lib.IsAlive(self.module)
    
    def CurPos(self):
        self.lib.CurPos.argtypes = [ctypes.c_void_p]
        self.lib.CurPos.restype = ctypes.c_double
        return self.lib.CurPos(self.module)
    
    def SetInner(self):
        self.lib.SetInner.argtypes = [ctypes.c_void_p]
        self.lib.SetInner(self.module)

    def SetOuter(self):
        self.lib.SetOuter.argtypes = [ctypes.c_void_p]
        self.lib.SetOuter(self.module)