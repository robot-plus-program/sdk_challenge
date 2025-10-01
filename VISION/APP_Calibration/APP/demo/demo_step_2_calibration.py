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
import numpy as np
import os
import cv2
import json
from keticalibsdk.detection.aruco_plate import aruco_pattern
from keticalibsdk.calib.calibration import calib_opencv
if __name__=='__main__':
    global rgb,depth
    global fx, fy, ppx, ppy
    gripper_size=0.285 # m
    EPS = 0.00001

    # calibration
    calib=calib_opencv()
    aruco = aruco_pattern()
    print("calibration")

    # load image & data
    datapath = "../../data/set1/"
    files = os.listdir(datapath)
    list_filename = []
    for f in files:
        if "rgb" in f:
            list_filename.append(f[0:-7])
    list_filename = np.sort(list_filename)

    c2w = []
    b2t = []

    RTarget2Cam=[]
    TTarget2Cam=[]
    T_base2EE_list=[]
    for filename in list_filename:
        path_rgb = datapath + filename + "rgb.png"
        path_depth = datapath + filename + "depth.png"
        path_json = datapath + filename + "UR10_Re.json"

        rgb = cv2.imread(path_rgb)
        depth = cv2.imread(path_depth, -1)
        st_json = open(path_json, "r")
        robot = json.load(st_json)
        rvec, tvec, im_with_aruco_board = aruco.calibration_pnp(rgb)
        R, _ = cv2.Rodrigues(rvec)  # R is the rotation matrix from the target frame to the camera frame
        RTarget2Cam.append(R)
        TTarget2Cam.append(tvec)
        T_base2EE_list.append(robot['matrix'])

    # Calculate T_cam2target
    R_cam2target,T_cam2target=    calib.invert_target2cam(RTarget2Cam,TTarget2Cam)

    # Calculate T_Base2EE
    R_vecEE2Base,    tEE2Base=calib.Calculate_T_Base2EE(T_base2EE_list)

    #calibration
    i=0
    print("Method:", i)
    R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
        R_cam2target,
        T_cam2target,
        R_vecEE2Base,
        tEE2Base,
        method=i
    )
    # print and save each results as .npz file
    print("The results for method", i, "are:")
    print("R_cam2gripper:", R_cam2gripper)
    print("t_cam2gripper:", t_cam2gripper.T)
    # Create 4x4 transfromation matrix
    T_cam2TCP = np.concatenate((R_cam2gripper, t_cam2gripper), axis=1)
    T_cam2TCP = np.concatenate((T_cam2TCP, np.array([[0, 0, 0, 1]])), axis=0)

    calib_result={"matrix" : T_cam2TCP.tolist()}
    with open(f"FinalTransforms/T_cam2gripper_Method_{i}", 'w') as sf:
        json.dump(calib_result, sf)
    print("done")
