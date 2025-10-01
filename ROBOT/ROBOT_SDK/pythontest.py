
from cmath import pi
from ketirobotsdk.sdk import *
from time import *
import threading


Poset1= [-1,   0,   0,  0, 0, 1, 0,  -0.4,0,  0,   -1,  0.350, 0,   0,   0,   1]
Poset2= [-1,   0,   0,  -0.1, 0, 1, 0,  -0.4,0,  0,   -1,  0.350, 0,   0,   0,   1]
Jntt1=[1.237,-1.201,0.904,-1.276,-1.570,-0.336]
Jntt2=[1.002,-1.134,0.805,-1.244,-1.570,-0.570]


testRobot=Robot()
setLibPath("ketirobotsdk/librobotsdk.so")
testRobot.SetRobotConf(M1013,"192.168.137.50",12345)    
def thread():
    while(1):
        Data=testRobot.RobotInfo()
        print("Robot 1 Mat")
        print("%.3f\t%.3f\t%.3f\t%.3f"%(Data.Mat[0],Data.Mat[1],Data.Mat[2],Data.Mat[3]))
        print("%.3f\t%.3f\t%.3f\t%.3f"%(Data.Mat[4],Data.Mat[5],Data.Mat[6],Data.Mat[7]))
        print("%.3f\t%.3f\t%.3f\t%.3f"%(Data.Mat[8],Data.Mat[9],Data.Mat[10],Data.Mat[11]))
        print("%.3f\t%.3f\t%.3f\t%.3f"%(Data.Mat[12],Data.Mat[13],Data.Mat[14],Data.Mat[15]))
        print("%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f"%(Data.Jnt[0],Data.Jnt[1],Data.Jnt[2],Data.Jnt[3],Data.Jnt[4],Data.Jnt[5]))
        
        sleep(1)
    
sub=threading.Thread(target=thread)

if __name__=='__main__':
       
    testRobot.RobotConnect()
    sub.daemon=True
    # sub.start()
    
    while(1):
        sleep(5)
        # testRobot.ControlBoxDigitalOut(1)
        input_Val=testRobot.ControlBoxDigitalIn()
        print("DIGITAL IN %d"%input_Val)
        sleep(5)
        # testRobot.ControlBoxDigitalOut(0)
        input_Val=testRobot.ControlBoxDigitalIn()
        print("DIGITAL IN %d"%input_Val)
        
    testRobot.SetVelocity(40)
    
    # testRobot.movel(0,Poset1)
    sleep(5)
    # testRobot.movel(0,Poset2)
    sleep(5)
    # testRobot.movej(Jntt1)
    sleep(5)
    # testRobot.movej(Jntt2)
    sleep(5)
    
    testRobot.RobotDisconnect()

    
    