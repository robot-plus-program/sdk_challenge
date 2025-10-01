#!/usr/bin/env python3

from interface.vision import *
from interface.gripper import *
from interface.robot import *
import copy
import math
import time
import numpy as np

init_joint = [2.9919661435369993, -0.14149567950770023, 1.6504852971450106, 3.616965391285435e-05, 1.6348476146982287, 1.4206483698006651]
init_pose = [0.00041608748554342045, 0.9999998448719181, -0.00037030709442165317, -0.4701260070800781,
                0.9999972054141893, -0.0004169481229974181, -0.0023270835984908872, 0.0360301628112793,
                -0.0023272376363428237, -0.0003693377892035561, -0.999997223773437, 0.6797732543945313,
                0.0, 0.0, 0.0, 1.0]

# angle
check_angle_pose = [0.000561736902368215, 0.9999994275299344, -0.0009107093147668205, -0.527627197265625,
                    0.9999960422571673, -0.000564245175006306, -0.0027562832554316116, 0.803546142578125,
                    -0.002756795540878646, -0.0009091574043956017, -0.9999957867467042, 0.4873616943359375,
                    0.0, 0.0, 0.0, 1.0]

# chuck
chuck_up_pose = [5.643957226186824e-06, 0.9999997613049648, -0.0006909111078788334, -0.09174932861328125,
                    0.9999967496569874, -7.405509350744133e-06, -0.002549631467252072, 0.6716876831054688,
                    -0.002549635975216369, -0.0006908944721697979, -0.9999965110045246, 0.4500216369628906,
                    0.0, 0.0, 0.0, 1.0]
chuck_pose = [5.643957226186824e-06, 0.9999997613049648, -0.0006909111078788334, -0.09174932861328125,
                0.9999967496569874, -7.405509350744133e-06, -0.002549631467252072, 0.6716876831054688,
                -0.002549635975216369, -0.0006908944721697979, -0.9999965110045246, 0.2750216369628906,
                0.0, 0.0, 0.0, 1.0]

# remove
remove_z = 277.8 / 1000.0

# place
place_pose = [-0.000071, 1.000000, -0.000226, -0.517256,
                1.000000, 0.000071, -0.000233, 0.434563,
                -0.000233, -0.000226, -1.000000, 0.320387,
                0.000000, 0.000000, 0.000000, 1.000000]

place_bin_up_pose = [-0.000289, 1.000000, -0.000212, -0.577230,
                    1.000000, 0.000289, -0.000221, 0.590940,
                    -0.000221, -0.000212, -1.000000, 0.235057,
                    0.000000, 0.000000, 0.000000, 1.000000]
place_z = 0.190057
place_obj_offset_x = ((-0.466570) - (-0.577230))/1.0
place_obj_offset_y = ((0.590940) - (0.315530))/4.0

vision=Vision("172.17.0.2",8801)
gripper = Gripper("127.0.0.1", 5002)
robot_info = RobotInfo("127.0.0.1", 3030)
robot_cmd = RobotCmd("127.0.0.1", 3033)

def init():
    cmd_str = f"""
        robot.SetVelocity(25)
        robot.ControlBoxDigitalOut(2)
        robot.movej({init_joint})
        robot.WaitMove()
    """
    robot_cmd.send_cmd(cmd_str)
    gripper.Init()
    print("init complete!!")
    gripper.Release()

def cam_pose():
    cmd_str = f"""
        robot.SetVelocity(25)
        robot.movel(0, {init_pose})
        robot.WaitMove()
    """
    robot_cmd.send_cmd(cmd_str)

def pick(offset_data):
    gripper.Move(1500)
    
    current_pose = robot_info.robot_mat
    target_pose1 = copy.deepcopy(current_pose)
    target_pose1[3] += offset_data[0]
    target_pose1[7] += offset_data[1]
    target_pose2 = copy.deepcopy(target_pose1)
    target_pose2[11] += offset_data[2]
    cmd_str = f"""
        robot.movel(0, {target_pose1})
        robot.WaitMove()

        robot.movel(0, {target_pose2})
        robot.WaitMove()
    """
    robot_cmd.send_cmd(cmd_str)

    gripper.Grip()

    target_pose2[11] = 0.487
    cmd_str = f"""
        robot.movel(0, {target_pose2})
        robot.WaitMove()
    """
    robot_cmd.send_cmd(cmd_str)

def check_angle():
    cmd_str = f"""
        robot.movel(0, {check_angle_pose})
        robot.WaitMove()
        time.sleep(1)
    """
    robot_cmd.send_cmd(cmd_str)

def move_angle(angle):
    mat_offset = np.array([[math.cos(angle), -math.sin(angle), 0],
                [math.sin(angle), math.cos(angle), 0],
                [0, 0, 1]])
    current_pose = robot_info.robot_mat
    target_pose = copy.deepcopy(current_pose)

    target_mat = np.array(target_pose).reshape((4, 4))
    mat = target_mat[0:3, 0:3]
    result = mat @ mat_offset
    target_mat[0:3, 0:3] = result
    target_pose = target_mat.flatten().tolist()
    cmd_str = f"""
        robot.movel(0, {target_pose})
        robot.WaitMove()
    """
    robot_cmd.send_cmd(cmd_str)

def insert():
    target_pose = copy.deepcopy(robot_info.robot_mat)
    target_pose[3] = chuck_up_pose[3]
    target_pose[7] = chuck_up_pose[7]
    target_pose[11] = chuck_up_pose[11]
    cmd_str = f"""
        robot.movel(0, {target_pose})
        robot.WaitMove()
    """
    robot_cmd.send_cmd(cmd_str)

    target_pose[3] = chuck_pose[3]
    target_pose[7] = chuck_pose[7]
    target_pose[11] = chuck_pose[11] + 0.01
    cmd_str = f"""
        robot.movel(0, {target_pose})
        robot.WaitMove()
    """
    robot_cmd.send_cmd(cmd_str)

    target_pose[11] = chuck_pose[11]

    angle = 20*math.pi/180.0
    mat_offset = np.array([[math.cos(angle), -math.sin(angle), 0],
                            [math.sin(angle), math.cos(angle), 0],
                            [0, 0, 1]])
    
    target_mat = np.array(target_pose).reshape((4,4))
    mat = target_mat[0:3, 0:3]
    result = mat @ mat_offset
    target_mat[0:3, 0:3] = result
    
    target_pose2 = target_mat.flatten().tolist()

    cmd_str = f"""
        robot.RobotComplianceCtrlOn()

        fd = [0, 0, -50, 0, 0, 10]
        dir = [0, 0, 1, 0, 0, 1]
        robot.RobotSetToolForce(fd, dir)

        robot.SetVelocity(2)
        robot.movel(0, {target_pose})
        robot.WaitMove()

        robot.SetVelocity(15)
        robot.movel(0, {target_pose2})
        robot.WaitMove()
    """
    robot_cmd.send_cmd(cmd_str)

    gripper.Release()

    cmd_str = f"""
        robot.RobotReleaseForce()
        robot.RobotComplianceCtrlOff()
        robot.SetVelocity(25)
    """
    robot_cmd.send_cmd(cmd_str)

    wait_pose = copy.deepcopy(chuck_up_pose)
    wait_pose[11] += 0.1
    cmd_str = f"""
        robot.movel(0, {wait_pose})
        robot.WaitMove()
    """
    robot_cmd.send_cmd(cmd_str)

def press():
    cmd_str = f"""
        robot.ControlBoxDigitalOut(1)
        while True:
            value = robot.ControlBoxDigitalIn();
            if value & 0x0008:
                time.sleep(1)
                break
            time.sleep(0.1)

        robot.ControlBoxDigitalOut(5)
        time.sleep(2)

        robot.ControlBoxDigitalOut(1)
        while True:
            value = robot.ControlBoxDigitalIn();
            if value & 0x0008:
                time.sleep(1)
                break
            time.sleep(0.1)

        robot.ControlBoxDigitalOut(2)
        while True:
            value = robot.ControlBoxDigitalIn();
            if value & 0x0005:
                time.sleep(1)
                break
            time.sleep(0.1)
    """
    robot_cmd.send_cmd(cmd_str)

def remove():
    remove_pose = copy.deepcopy(chuck_pose)
    remove_pose[11] = remove_z - 0.001
    cmd_str = f"""
        robot.movel(0, {remove_pose})
        robot.WaitMove()
    """
    robot_cmd.send_cmd(cmd_str)

    gripper.Grip()

    wait_pose = copy.deepcopy(chuck_up_pose)
    wait_pose[11] += 0.1
    cmd_str = f"""
        robot.movel(0, {wait_pose})
        robot.WaitMove()
    """
    robot_cmd.send_cmd(cmd_str)

def place(obj_cnt):
    cmd_str = f"""
        robot.movel(0, {place_pose})
        robot.WaitMove()
    """
    robot_cmd.send_cmd(cmd_str)

    offset = [0, 0]
    offset[0] = place_obj_offset_x*(obj_cnt / 5)
    offset[1] = -place_obj_offset_y*(obj_cnt % 5)

    target_pose = copy.deepcopy(place_bin_up_pose)
    target_pose[3] += offset[0]
    target_pose[7] += offset[1]

    target_pose2 = copy.deepcopy(target_pose)
    target_pose2[11] = place_z

    cmd_str = f"""
        robot.SetVelocity(5)
        robot.movel(0, {target_pose})
        robot.WaitMove()

        robot.movel(0, {target_pose2})
        robot.WaitMove()
    """
    robot_cmd.send_cmd(cmd_str)

    gripper.Release()

    target_pose2[11] = place_z*2
    cmd_str = f"""
        robot.movel(0, {target_pose2})
        robot.WaitMove()
        robot.SetVelocity(25)
    """
    robot_cmd.send_cmd(cmd_str)


if __name__ == '__main__':
    vision.get_rs()

    init()
    max_cnt = 2

    for obj_cnt in range(0, max_cnt):
        cam_pose()

        ret_rs,rgb,depth=vision.get_rs()
        ret_pick,ret,ret_img=vision.get_pick(rgb,depth)
        cv2.imshow("ret_img",ret_img)
        cv2.waitKey(100)
        offset_data = ret[0:3]
        
        pick(offset_data)
        check_angle()

        # vision request 2
        ret_Tis,ret_resizeimg,ret_original=vision.get_TIS()
        ret_angle,ret,ret_img =vision.get_angle(ret_original)
        cv2.imshow("ret_img",ret_img)
        cv2.waitKey(100)
        angle = ret['differ_angle']*math.pi/180.0

        move_angle(angle)
        insert()
        press()
        remove()
        place(obj_cnt)
