# -*- coding: utf-8 -*-
"""
    + Simultaneous Robot/World and Tool/Flange Calibration:
    Implementation of Shah, Mili. "Solving the robot-world/hand-eye calibration problem using the Kronecker product."
    Journal of Mechanisms and Robotics 5.3 (2013): 031007.

    Batch_Processing solvesfor  X and Y in AX=YB from a set of (A,B) paired measurements.
    (Ai,Bi) are absolute pose measurements with known correspondance

    A: (4x4xn)
    X: (4x4): unknown
    Y: (4x4): unknown
    B: (4x4xn)
    n number of measurements

    + EKF,IEKF solves for AX=XB from a set of (Ai,Bi) relative pose measurements with known correspondance.
    so3 representation was used to represent the state of rotation.

    @author: elif.ayvali
"""
import copy
import numpy as np
import os
import cv2
from APP_Calibration.keticalibsdk.calib.calibration import calib_opencv
from APP_Calibration.keticalibsdk.datautils.grab_data import Sensor
from APP_Base.robot.robot import robotsdk
import datetime
from ketirobotsdk.ketirobotsdk.sdk import M1013
def mouse_event(event, x, y, flags, param):
    global gx, gy
    global display_img
    global fx,fy,ppx,ppy
    if event == cv2.EVENT_FLAG_LBUTTON:
        Xc = x
        Yc = y
        Zc=depth[y,x]

        #draw the point
        img=copy.deepcopy(rgb)
        cv2.circle(img,(x,y),4,(0,0,255),1,-1,0)
        cv2.imshow("show",img)
        key = cv2.waitKey(30)
        #write images
        cur_time_str = datetime.datetime.now().strftime('%Y_%m_%d_%H_%M_%S')
        cv2.imwrite(folderpath + cur_time_str + "_rgb.png", img)
        cv2.imwrite(folderpath + cur_time_str + "_depth.png", depth)

        #convert depth to 3d point (mm)
        print(Xc,Yc,Zc)
        point_x = (Xc - ppx) / fx * Zc
        point_y = (Yc - ppy) / fy * Zc
        point_z = float(Zc)

        #calculattion
        calib=[-0.06194620834952702, 0.03100096813642467, 0.11363061607688473]
        robot_x=-point_y
        robot_y=-point_x
        robot_z=-point_z
        robot_trans=[ robot_x/1000+calib[0],
                      robot_y/1000+calib[1],
                      robot_z/1000+calib[2]]

        robot_pose=list(robot.get_pose_1d())
        robot_pose[3]+=robot_trans[0]
        robot_pose[7]+=robot_trans[1]
        robot_pose[11]+= robot_trans[2]
        print(robot_pose)

        if robot_pose[11]<0.180:
            robot_pose[11]=0.180
        # movel
        robot.SetVelocity(30)
        robot.movel(robot_pose)

        print(robot_pose)


if __name__=='__main__':
    global rgb,depth
    global fx, fy, ppx, ppyh

    try:
        gripper_size=0.285 # m
        calib=calib_opencv()

        robot=robotsdk(ip="192.168.137.50",robotname=M1013,port=12345)
        print(robot.get_pose_1d())

        init_joint=[2.9919661435369993, -0.14149567950770023, 1.6504852971450106, 3.616965391285435e-05, 1.6348476146982287, 1.4206483698006651, 6.76353516188367e-310]

        cv2.namedWindow('result')
        cv2.setMouseCallback('result', mouse_event)

        sensor = Sensor()
        sensor.start()

        ppx, ppy, fx, fy = sensor.intr_params.ppx, sensor.intr_params.ppy, sensor.intr_params.fx, sensor.intr_params.fy

        # save path
        datapath = "../../data/"
        datastr = datetime.datetime.now().strftime('%Y_%m_%d_%H_%M_%S/')
        folderpath = datapath + datastr
        robot.SetVelocity(10)
        robot.movej(init_joint)
        if os.path.isdir(datapath + datastr):
            print(f"{datapath + datastr} exist")
        else:
            print(f"make {datapath + datastr}")
            os.mkdir(datapath + datastr)


            while 1:
                rgb, depth = sensor.get_bgrd()
                cv2.imshow("result", rgb)
                key = cv2.waitKey(30)

                if key==ord("h"):
                    robot.SetVelocity(10)
                    robot.movej(init_joint)
    except Exception as e:
        print("에러 발생:", e)
