import time

import numpy as np
import copy
from ketirobotsdk.ketirobotsdk.sdk import *
from math3d import Orientation

from ketisdk import gripper


class robotsdk(Robot):
    def __init__(self,robotname=RB10,ip='192.168.0.7',port=30003):
        super().SetRobotConf(robotname, ip, port)
        super().RobotConnect()
    def move_home(self):
        robot_z=self.get_pose_1d()[11]
        if robot_z<0.45:
            self.move_translate([0,0,.45-robot_z])
    def move_translate(self,xyz):
        if (np.array(xyz)>1).any():
            return False

        cur_pose=self.get_pose_1d()
        trans_pose=self.translate(cur_pose,xyz)
        self.movel(trans_pose)

    def SetVelocity(self,value):
        super().SetVelocity(value)
    def movel(self, pose,vel=30):
        self.SetVelocity(vel)
        super().movel(Base,pose)

    def move_translate_tcp(self,xyz):
        if (np.array(xyz)>1).any():
            return False

        cur_pose=self.get_pose_1d()
        trans_pose=self.translate(cur_pose,xyz)
        self.movel(trans_pose)
    def movej(self, Jnt,vel=10):
        self.SetVelocity(vel)
        super().movej(Jnt)
    # def moveb_3(self,r,pose1,pose2,pose3):
    #     #type : Reference Coordinate , (Base or TCP)
    #     # r : Blend Radius [m]
    #     # nPnt : number of via point
    #     # Pose(n) : target position & rotation, (4by4 matrix)
    #     super().
    def ControlBoxDigitalOut(self,args):
        super().ControlBoxDigitalOut(args)
    def ControlBoxDigitalIn(self):
        super().ControlBoxDigitalIn()
    def get_pose_1d(self):
        mat=copy.deepcopy(self.RobotInfo().Mat)
        return mat
    def get_pose_matrix4x4(self):
        mat=copy.deepcopy(self.RobotInfo().Mat)
        cur_robot_pose = np.reshape(np.array(mat), (4, 4))
        return cur_robot_pose

    def get_pose_joint(self):
        join=copy.deepcopy(self.RobotInfo().Jnt)
        return join

    def get_rotate_zt_robot(self,angle):
        Orient = Orientation()
        Orient.rotate_zt((angle / 180.0) * np.pi)
        romat = Orient.matrix
        cur_robot_pose = self.get_pose_matrix4x4()
        ro = cur_robot_pose[0:3, 0:3]
        result_matrix = copy.deepcopy(cur_robot_pose)
        result_ro = np.matmul(romat, ro)
        result_matrix[0:3, 0:3] = result_ro
        return result_matrix

    def translate(self,Matrix, xyz):
        cur_matrix = copy.deepcopy(Matrix)
        cur_matrix[3] += xyz[0]
        cur_matrix[7] += xyz[1]
        cur_matrix[11] += xyz[2]
        return cur_matrix

    def print_matrix(self,Matrix):
        print("Mat")
        if Matrix.flatten().shape==1:
            print("1d array")
            print("%.3f\t%.3f\t%.3f\t%.3f" % (Matrix[0], Matrix[1], Matrix[2], Matrix[3]))
            print("%.3f\t%.3f\t%.3f\t%.3f" % (Matrix[4], Matrix[5], Matrix[6], Matrix[7]))
            print("%.3f\t%.3f\t%.3f\t%.3f" % (Matrix[8], Matrix[9], Matrix[10], Matrix[11]))
            print("%.3f\t%.3f\t%.3f\t%.3f" % (Matrix[12], Matrix[13], Matrix[14], Matrix[15]))
        else:
            print("2d array")
            Mat=Matrix.flatten()
            print("%.3f\t%.3f\t%.3f\t%.3f" % (Mat[0], Mat[1], Mat[2], Mat[3]))
            print("%.3f\t%.3f\t%.3f\t%.3f" % (Mat[4], Mat[5], Mat[6], Mat[7]))
            print("%.3f\t%.3f\t%.3f\t%.3f" % (Mat[8], Mat[9], Mat[10], Mat[11]))
            print("%.3f\t%.3f\t%.3f\t%.3f" % (Mat[12], Mat[13], Mat[14], Mat[15]))

if __name__ == '__main__':
    rob=robotsdk(robotname=M1013,ip="192.168.137.50",port=12345)
    # rob = robotsdk(ip="192.168.0.111", robotname=UR10, port=30003)
    cur_pose1=list(rob.get_pose_1d())
    cur_pose2=rob.translate(cur_pose1,[0.1,0,0])
    cur_pose3=rob.translate(cur_pose2,[0.0,0,0.1])
    rob.SetVelocity(5)
    rob.moveb_3(0.03,cur_pose1,cur_pose2,cur_pose3)
    rob.SetVelocity(10)
    init_joint=[3.2276777379299504, 0.4889183928035009, 1.8763281486445098, 0.7612683260246764, 0.19967190088826317,
     -1.554647716892983, 6.91914573813165e-310]
    RB = [
        [6.123233995736766e-17, 0.5735764363510464, -0.8191520442889916, -0.4415000081062317, 0.0, -0.8191520442889916,
         -0.5735764363510464, 0.37158098816871643, -1.0, 3.5121427342182724e-17, -5.0158596452676225e-17,
         0.17621943354606628, 0.0, 0.0, 0.0, 1.0],
        [6.123233995736766e-17, 0.5735764363510464, -0.8191520442889916, -0.4453900158405304, 0.0, -0.8191520442889916,
         -0.5735764363510464, 0.0378200002014637, -1.0, 3.5121427342182724e-17, -5.0158596452676225e-17,
         0.17709992825984955, 0.0, 0.0, 0.0, 1.0],
        [-1.6895982174749097e-05, 0.5735763817608404, -0.8191520823392128, -0.8089030385017395, -4.336892359425421e-06,
         -0.8191520824904621, -0.5735763817772926, 0.04069500043988228, -0.9999999998478586, -6.138561915261865e-06,1.6327918011772917e-05,
         0.1746734082698822, 0.0, 0.0, 0.0, 1.0],
        [6.123233995736766e-17, 0.5735764363510464, -0.8191520442889916, -0.8059999942779541, 0.0, -0.8191520442889916,
         -0.5735764363510464, 0.37457001209259033, -1.0, 3.5121427342182724e-17, -5.0158596452676225e-17,
         0.17239142954349518, 0.0, 0.0, 0.0, 1.0]]

    while 1:
        print(ControlBoxDigitalIn())
        time.sleep(0.5)
    pose_home=[[-4.947944013820744e-06, 0.9999999999781044, -4.394208035884972e-06, -0.356751024723053], [-3.1377056391368486e-06, -4.394223561132726e-06, -0.9999999999854228, 0.22825218737125397], [-0.9999999999828363, -4.9479302260172835e-06, 3.1377273814403116e-06, 0.52621990442276], [0.0, 0.0, 0.0, 1.0]]
    pose_up=[[-4.947944013820744e-06, 0.9999999999781044, -4.394208035884972e-06, -0.6470134854316711],
     [-3.1377056391368486e-06, -4.394223561132726e-06, -0.9999999999854228, 0.22825218737125397],
     [-0.9999999999828363, -4.9479302260172835e-06, 3.1377273814403116e-06, 0.52621990442276], [0.0, 0.0, 0.0, 1.0]]
    pose_down=[6.123233995736766e-17, 0.9999999999999911, -1.3315805429941648e-07, -0.6470130085945129,
               0.0, -1.3315805429941648e-07, -0.9999999999999911, 0.22825199365615845,
               -1.0,6.123233995736712e-17, -8.153579248923492e-24, 0.21553325724601746,
               0.0, 0.0, 0.0, 1.0]
    rob.print_matrix(rob.get_pose_matrix4x4())
    movel(Base, np.array(pose_home).flatten().tolist())
    movel(Base, np.array(pose_up).flatten().tolist())
    movel(Base, np.array(pose_down).flatten().tolist())
    movel(Base, np.array(pose_up).flatten().tolist())
    print("done")