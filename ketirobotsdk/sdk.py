
from operator import mod
import os
import ctypes
# from psutil import cpu_times_percent
TestDummy= 0
RB10    =1
UR10    =2
M1013   =3
Indy7   =4

Base=0
TCP=1

module=None
file_path= f'{os.getcwd()}/librobotsdkv.so'
if os.path.isfile(file_path) is True :
    module=ctypes.cdll.LoadLibrary(file_path)
    
def setLibPath(path):
    global module
    module=ctypes.cdll.LoadLibrary(path)


connect_state=0



class RxDataInfo(ctypes.Structure):
    _fields_=[("Jnt",ctypes.c_double*7),
             ("Mat",ctypes.c_double*16),
             ("State",ctypes.c_double)]
    
class TCPoff(ctypes.Structure):
    _fields_=[('x',ctypes.c_double),
              ('y',ctypes.c_double),
              ('z',ctypes.c_double),
              ('rot1',ctypes.c_double),
              ('rot2',ctypes.c_double),
              ('rot3',ctypes.c_double)]      

class DIInput(ctypes.Structure):
    _fields_=[('DI',ctypes.c_bool*16)]      


def RobotInfo():
    module.RobotInfo.restype=RxDataInfo

    return module.RobotInfo()

# def SetTCP(args):
   
#     module.SetTCPCord(TCP(args[0],args[1],args[2],args[3],args[4],args[5]))

def movel(*args):
    module.movel.argtypes=[ctypes.c_double,ctypes.POINTER(ctypes.c_double)]
    if len(args[1])!=16: 
        print("input arguments must 16")
    
    arg_arr=[args[0],(ctypes.c_double*len(args[1]))(*args[1])]
    
   
    module.movel(*arg_arr)
    
    
def movej(args):
    module.movej.argtypes=[ctypes.POINTER(ctypes.c_double)]
    if len(args)!=6: 
        print("input arguments must 6")
    
    arg_arr=(ctypes.c_double*len(args))(*args)
    module.movej(arg_arr)
                 

def moveb(*args):
    err=0
    args_arr=[args[0],args[1],len(args)-3]
    module.moveb.argtypes=[ctypes.c_double,ctypes.c_double,ctypes.c_double]
    
    
    for i in range(3,len(args)):
    
        if(len(args[i])==16) :
            args_arr.append((ctypes.c_double*len(args[i]))(*args[i]))
            module.moveb.argtypes.append(ctypes.POINTER(ctypes.c_double))
            
        else:
            err=i 
            break
        
    if err!=0 :
        print('CMD Input',err,'Length Not match')
    else : 
        module.moveb(*args_arr)
    

def movec(*args):
    err=0
    args_arr=[args[0]]
    module.movec.argtypes=[ctypes.c_double]
    
    
    for i in range(1,3):
        if(len(args[i])==16) :
            args_arr.append((ctypes.c_double*len(args[i]))(*args[i]))
            module.movec.argtypes.append(ctypes.POINTER(ctypes.c_double))
           
        else:
            err=i 
            break
        
    if err!=0 :
        print('CMD Input',err,'Length Not match')
    else : 
        module.movec(*args_arr)
        
    
def SetRobotConf(*args):
    args_arr=[args[0],args[1].encode('utf-8'),args[2]]
    module.SetRobotConf.argtypes=[ctypes.c_int,ctypes.c_char_p,ctypes.c_int]
    module.SetRobotConf(*args_arr)
    
def RobotConnect():
    # module.RobotConnect()
    module.RobotConnect.restype=ctypes.c_bool
    return module.RobotConnect()
    
def RobotDisconnect():
    module.RobotDisconnect()
    
def SetVelocity(args):
    module.SetVelocity.argtypes=[ctypes.c_double]
    module.SetVelocity(args)
    
def Stop():
    module.Stop()
    

def ControlBoxDigitalOut(args):
    module.ControlBoxDigitalOut.argtypes=[ctypes.c_int]
    module.ControlBoxDigitalOut(args)
    
def ControlBoxDigitalIn():
    # module.ControlBoxDigitalIn.restype=DIInput
    module.ControlBoxDigitalIn.restype=ctypes.c_double
    return module.ControlBoxDigitalIn()

def IsConnected():
    module.IsConnected.restype=ctypes.c_bool    
    return module.IsConnected()

def testf(args):
    module.testf.argtypes=[ctypes.c_int]
    module.testf(args)
    
def check():

    module.check()
    




class Robot:
          
    def SetRobotConf(self,*args):
        args_arr=[args[0],args[1].encode('utf-8'),args[2]]
        module.SetRobotConf.argtypes=[ctypes.c_int,ctypes.c_char_p,ctypes.c_int]
        module.SetRobotConf.restype=ctypes.c_int
        self.ID=module.SetRobotConf(*args_arr)
        
        
    def RobotConnect(self):
        module.RobotConnect.argtypes=[ctypes.c_int]
        module.RobotConnect.restype=ctypes.c_bool
        return module.RobotConnect(self.ID)
    
    def RobotDisconnect(self):
        module.RobotDisconnect.argtypes=[ctypes.c_int]
        module.RobotDisconnect(self.ID)
        
    def movej(self,args):
        module.movej.argtypes=[ctypes.c_int,ctypes.POINTER(ctypes.c_double)]
        if len(args)!=6: 
            print("input arguments must 6")
    
        arg_arr=(ctypes.c_double*len(args))(*args)
        module.movej(self.ID,arg_arr)
        
    def movel(self,*args):
        module.movel.argtypes=[ctypes.c_int,ctypes.c_double,ctypes.POINTER(ctypes.c_double)]
        if len(args[1])!=16: 
            print("input arguments must 16")

        arg_arr=[self.ID,args[0],(ctypes.c_double*len(args[1]))(*args[1])]
        module.movel(*arg_arr)


    def moveb(self,*args):
        err=0
        args_arr=[self.ID,args[0],args[1],len(args)-3]
        module.moveb.argtypes=[ctypes.c_int,ctypes.c_double,ctypes.c_double,ctypes.c_double]
                
        for i in range(3,len(args)):
            if(len(args[i])==16) :
                args_arr.append((ctypes.c_double*len(args[i]))(*args[i]))
                module.moveb.argtypes.append(ctypes.POINTER(ctypes.c_double))

            else:
                err=i 
                break
            
        if err!=0 :
            print('CMD Input',err,'Length Not match')
        else : 
            module.moveb(*args_arr)
            
            
    def movec(self,*args):
        err=0
        args_arr=[self.ID,args[0]]
        module.movec.argtypes=[ctypes.c_int,ctypes.c_double]


        for i in range(1,3):
            if(len(args[i])==16) :
                args_arr.append((ctypes.c_double*len(args[i]))(*args[i]))
                module.movec.argtypes.append(ctypes.POINTER(ctypes.c_double))

            else:
                err=i 
                break
            
        if err!=0 :
            print('CMD Input',err,'Length Not match')
        else : 
            module.movec(*args_arr)
            
    def Stop(self):
        module.Stop(self.ID)
        
    def SetVelocity(self,args):
        module.SetVelocity.argtypes=[ctypes.c_int,ctypes.c_double]
        module.SetVelocity(self.ID,args)
    
    def RobotInfo(self):
        module.RobotInfo.argtypes=[ctypes.c_int]
        module.RobotInfo.restype=RxDataInfo

        return module.RobotInfo(self.ID)
    
    def ControlBoxDigitalOut(self,args):
        module.ControlBoxDigitalOut.argtypes=[ctypes.c_int,ctypes.c_int]
        module.ControlBoxDigitalOut(self.ID,args)
    
    def ControlBoxDigitalIn(self):        
        module.ControlBoxDigitalIn.argtypes=[ctypes.c_int]
        module.ControlBoxDigitalIn.restype=ctypes.c_double
        return module.ControlBoxDigitalIn(self.ID)

    def IsConnected(self):
        module.IsConnected.argtypes=[ctypes.c_int]
        module.IsConnected.restype=ctypes.c_bool    
        return module.IsConnected(self.ID)