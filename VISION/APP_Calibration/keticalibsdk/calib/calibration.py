import cv2
import numpy as np
import json
import copy
from math3d import Orientation
class Calib:
    def __init__(self):
        print("Eye in Hand Calibration")
        self.T_cam2gripper=None
    def load_calibmatrix(self,path_json= "./FinalTransforms/T_cam2gripper_Method_0.json"):
        print("        validataion")
        st_json = open(path_json, "r")
        calib_robot = json.load(st_json)
        self.T_cam2TCP=np.array(calib_robot['matrix'])
    def set_grippersize(self,gripper_size): ## m
        self.gripper_size=gripper_size
        self.T_cam2gripper=copy.deepcopy(self.T_cam2TCP)
        self.T_cam2gripper[2, 3] = self.T_cam2gripper[2, 3] + self.gripper_size
    def set_intrinsic(self,ppx,ppy,fx,fy):
        self.ppx=ppx
        self.ppy=ppy
        self.fx=fx
        self.fy=fy
    def convert(self,robot_pose,camera_3dpoint,normal=None,angle_z=0):

        robot_pose=np.array(robot_pose)

        #robot pose >> 4x4
        if robot_pose.size==6:
            pose=np.eye(4)
            pose[0:3,0:3]=cv2.Rodrigues([robot_pose[3::]])[0]
            if (robot_pose[0:3]>3):
                tvec=robot_pose[0:3]/1000
            else:
                tvec=robot_pose[0:3]
            pose[0:3,3]=tvec
        else:
            pose=np.array(copy.deepcopy(robot_pose))

        point_x,point_y,point_z=camera_3dpoint
        point_matrix = np.eye(4)
        if (np.array(normal)==None):
            point_matrix = np.eye(4)
        else:
            point_matrix[0:3, 0:3] = cv2.Rodrigues([normal])[0]

        point_matrix[0:3,3]=np.array([point_x,point_y,point_z])/1000
        gripper_pose=np.matmul(np.linalg.inv(self.T_cam2gripper), point_matrix)
        Orient = Orientation()
        Orient.rotate_zt(np.deg2rad(angle_z))
        romat = Orient.matrix
        ro=gripper_pose[0:3,0:3]
        result_ro = np.matmul(romat, ro)
        rotated_gripper_pose=copy.deepcopy(gripper_pose)
        rotated_gripper_pose[0:3,0:3]=result_ro
        point_pose = np.matmul(pose, rotated_gripper_pose)
        pose_vector=point_pose[0:3,3].tolist()+(cv2.Rodrigues(point_pose[0:3,0:3])[0]).T[0].tolist()
        return pose_vector
class calib_opencv(Calib):
    def __init__(self):
        print("calib_opencv")
    def invert_target2cam(self,RTarget2Cam,TTarget2Cam):
        # Convert to homogeneous transformation matrix
        T_target2cam = [np.concatenate((R, T), axis=1) for R, T in zip(RTarget2Cam, TTarget2Cam)]
        for i in range(len(T_target2cam)):
            T_target2cam[i] = np.concatenate((T_target2cam[i], np.array([[0, 0, 0, 1]])), axis=0)

        # Calculate T_cam2target
        T_cam2target = [np.linalg.inv(T) for T in T_target2cam]
        R_cam2target = [T[:3, :3] for T in T_cam2target]
        T_cam2target = [T[:3, 3] for T in T_cam2target]  # 4x4 transformation matrix

        return R_cam2target,T_cam2target
    def Calculate_T_Base2EE(self,T_base2EE_list):
        # Calculate T_Base2EE
        TEE2Base = [np.linalg.inv(T) for T in T_base2EE_list]
        REE2Base = [T[:3, :3] for T in TEE2Base]
        R_vecEE2Base = [cv2.Rodrigues(R)[0] for R in REE2Base]
        tEE2Base = [T[:3, 3] for T in TEE2Base]
        return R_vecEE2Base,tEE2Base

    def run_calibrateHandEye(self,R_cam2target,T_cam2target,R_vecEE2Base,tEE2Base,method_number=0):
        R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
            R_cam2target,
            T_cam2target,
            R_vecEE2Base,
            tEE2Base,
            method=method_number
        )

        T_cam2gripper = np.concatenate((R_cam2gripper, t_cam2gripper), axis=1)
        T_cam2gripper = np.concatenate((T_cam2gripper, np.array([[0, 0, 0, 1]])), axis=0)
        return T_cam2gripper

    def convert_c2r(self,Xc,Yc,Zc):
        ppx=self.ppx
        ppy=self.ppy
        fx=self.fx
        fy=self.fy

        # camera 3d points
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
        return robot_trans