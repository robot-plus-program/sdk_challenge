#!/usr/bin/env python3

import socket
import time, threading, struct
import signal

class RobotInfo():
    def __init__(self, addr, port):
        self.SERVER_SOP = 0xCE
        self.SERVER_EOP = 0xCF

        self.addr = addr
        self.port = port

        self.robot_state = 0
        self.robot_jnt = [0.0]*6
        self.robot_mat = [0.0]*16
        self.DI = [0, 0]
        self.tool_force = [0.0]*6

        self.connected = False
        self.client = None

        t = threading.Thread(target=self.comm_func)
        t.daemon = True
        t.start()

    def comm_func(self):
        try:
            self.client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.client.connect((self.addr, self.port))
            print(f"Connected server : {(self.addr, self.port)}")
            self.connected = True
            
            while self.connected:
                recv_packet = self.client.recv(230)
                data = bytearray(recv_packet)
                recvByteLen = len(data)

                if recvByteLen >= 3:
                    if recvByteLen == (data[1] + 3) and (data[0] == self.SERVER_SOP or data[-1] == self.SERVER_EOP):
                        self.robot_state = data[2]
                        self.robot_jnt = list(struct.unpack_from("6d", data, offset = 3))
                        self.robot_mat = list(struct.unpack_from("16d", data, offset = 3 + 6*8))
                        self.DI = data[3 + 6*8 + 16*8]*255 + data[3 + 6*8 + 16*8 + 1]
                        self.tool_force = list(struct.unpack_from("6d", data, offset = 3 + 6*8 + 16*8 + 2))

                        # print(f"robot state : {self.robot_state}")
                        # print(f"robot joint : {self.robot_jnt}")
                        # print(f"robot mat : {self.robot_mat}")
                        # print(f"DI : {self.DI}")
                        # print(f"tool force : {self.tool_force}")
            
        except socket.error as e:
            print(f"[error] socket error : {e}")
            self.client.close()
            self.connected = False

        time.sleep(0.1)

    def close(self):
        self.connected = False
        self.client.close()


class RobotCmd():
    def __init__(self, addr, port):
        self.client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client.connect((addr, port))
        print(f"Connected server : {(addr, port)}")
        self.connected = True

    def send_cmd(self, code_str=""):
        self.client.send(code_str.encode('utf-8'))
        recv_data = self.client.recv(1024).decode('utf-8')
        if recv_data == '1':
            return True
        else:
            return False
        
    def close(self):
        self.client.close()


import copy, math
import numpy as np
if __name__ == '__main__':

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


    robot_info = RobotInfo("127.0.0.1", 3030)
    robot_cmd = RobotCmd("127.0.0.1", 3033)

    cmd_str = f"""
        robot.SetVelocity(25)
        robot.ControlBoxDigitalOut(2)
        robot.movej({init_joint})
        robot.WaitMove()
    """
    robot_cmd.send_cmd(cmd_str)

    for obj_cnt in range(0,1):
        # vision request 1
        offset_data = [-0.02, 0.03, -0.3]

        # pick ready
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

        # gripper Grip

        # pick
        target_pose2[11] = 0.487
        cmd_str = f"""
            robot.movel(0, {target_pose2})
            robot.WaitMove()
        """
        robot_cmd.send_cmd(cmd_str)

        # angle
        cmd_str = f"""
            robot.movel(0, {check_angle_pose})
            robot.WaitMove()
            time.sleep(1)
        """
        robot_cmd.send_cmd(cmd_str)

        # vision request 2
        angle = -15*math.pi/180.0
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

        # insert
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

        # gripper Release

        cmd_str = f"""
            robot.RobotReleaseForce()
            robot.RobotComplianceCtrlOff()
            robot.SetVelocity(25)
        """
        robot_cmd.send_cmd(cmd_str)

        # press object
        wait_pose = copy.deepcopy(chuck_up_pose)
        wait_pose[11] += 0.1
        cmd_str = f"""
            robot.movel(0, {wait_pose})
            robot.WaitMove()
        """
        robot_cmd.send_cmd(cmd_str)

        # press
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

        # remove
        remove_pose = copy.deepcopy(chuck_pose)
        remove_pose[11] = remove_z - 0.001
        cmd_str = f"""
            robot.movel(0, {remove_pose})
            robot.WaitMove()
        """
        robot_cmd.send_cmd(cmd_str)

        # gripper.Grip

        cmd_str = f"""
            robot.movel(0, {wait_pose})
            robot.WaitMove()
        """
        robot_cmd.send_cmd(cmd_str)

        # place
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

        # gripper Release

        target_pose2[11] = place_z*2
        cmd_str = f"""
            robot.movel(0, {target_pose2})
            robot.WaitMove()
        """
        robot_cmd.send_cmd(cmd_str)

        # init pose
        cmd_str = f"""
            robot.SetVelocity(25)
            robot.movej({init_joint})
            robot.WaitMove()
        """
        robot_cmd.send_cmd(cmd_str)
    
    cmd_str = "robot.RobotDisconnect()"
    robot_cmd.send_cmd(cmd_str)
