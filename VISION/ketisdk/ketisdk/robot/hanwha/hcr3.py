import configparser
import sys,os
import socket
import time
import threading
from robot.robot_master import Robot

class Hcr3(Robot):
    client_hcr_socket =[]
    client_hcr_addinfo = []
    socket = None

    def load_params(self):
        config = configparser.RawConfigParser()
        dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '../config/' + self.proj + '_' + self.workspace))

        config.read(dir + '/hcr3.cfg')
        self.home_rot     = [float(i) for i in list(config.get('joints', 'home_joints').split(','))]
        self.init_pose    = [float(i) for i in list(config.get('pos', 'init_pos').split(','))]
        self.init_pose2   = [float(i) for i in list(config.get('pos', 'init_pos2').split(','))]
        self.place_f_pose = [float(i) for i in list(config.get('pos', 'place_f_pos').split(','))]

        self.jvel = config.getfloat('time', 'jvel')
        self.jacc = config.getfloat('time', 'jacc')
        self.jtime = config.getfloat('time', 'jtime')

        self.lvel  = config.getfloat('time', 'lvel')
        self.lacc  = config.getfloat('time', 'lacc')
        self.ltime = config.getfloat('time', 'ltime')

        print('Hcr3::load_''params')

    def reload_params(self):
        self.load_params()

    def connect_robot(self):
        try:
            self.connect_socket()
        except:
            print('Hcr3::Can not open new socket(already connected) ')

        time.sleep(1)

        self.hcr3_receive_thread = threading.Thread(target=self.receive_data)
        self.hcr3_receive_thread.daemon = True
        self.hcr3_receive_thread.start()

        self.move_home()

        print('Hcr3::connect_robot')

    def connect_socket(self):
        print('Hcr3::opening socket - server info [ip:%s}] [port:%s]' % (str(self.ip), str(self.port)))

        if self.socket != None:
            self.socket.close()
            self.client_hcr_socket.close()

        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.bind((self.ip, self.port))
        self.socket.listen(1)
        self.client_hcr_socket, self.client_hcr_addinfo = self.socket.accept()

        print('Hcr3::hcr robot info [ip:%s}] [port:%s]' % self.client_hcr_addinfo)
        print('Hcr3::hcr3 robot connected!')

    def disconnect_robot(self):
        out_pos = "End\r\n"
        try:
            self.client_hcr_socket.send(out_pos.encode())
        except:
            print('Hcr3::No send message')

        print('Hcr3::disconnect_robot')

    def receive_data(self):
        # self._connect_socket()
        try:
            recv_data = self.client_hcr_socket.recv(65535)
            str_data = recv_data.decode()
            str_data = str_data.split("\n")

            size = len(str_data)
            index = str_data.index('Joint 6')

            # Joint 6 [0]
            self.c_joint[0] = float(str_data[index + 1])
            self.c_joint[1] = float(str_data[index + 2])
            self.c_joint[2] = float(str_data[index + 3])
            self.c_joint[3] = float(str_data[index + 4])
            self.c_joint[4] = float(str_data[index + 5])
            self.c_joint[5] = float(str_data[index + 6])

            # Pose [7]
            self.c_pose[0] = float(str_data[index + 8])
            self.c_pose[1] = float(str_data[index + 9])
            self.c_pose[2] = float(str_data[index + 10])
            self.c_pose[3] = float(str_data[index + 11])
            self.c_pose[4] = float(str_data[index + 12])
            self.c_pose[5] = float(str_data[index + 13])

            # JointSpeed [14]
            self.c_jspeed[0] = float(str_data[index + 15])
            self.c_jspeed[1] = float(str_data[index + 16])
            self.c_jspeed[2] = float(str_data[index + 17])
            self.c_jspeed[3] = float(str_data[index + 18])
            self.c_jspeed[4] = float(str_data[index + 19])
            self.c_jspeed[5] = float(str_data[index + 20])

            tmp = 0

        except Exception as e:
            print("Hcr3:: Error receive data")

    def move_j(self, joint_params, vel, acc, time, move_wait = False):
        if len(joint_params) != 6:
            print("Hcr3:: move_j function: input param length -> 9")
            return

        out_pos = "movej("
        out_pos = out_pos + "{:.6f}".format(joint_params[0])
        out_pos = out_pos + "0, "
        out_pos = out_pos + "{:.6f}".format(joint_params[1])
        out_pos = out_pos + "0, "
        out_pos = out_pos + "{:.6f}".format(joint_params[2])
        out_pos = out_pos + "0, "
        out_pos = out_pos + "{:.6f}".format(joint_params[3])
        out_pos = out_pos + "0, "
        out_pos = out_pos + "{:.6f}".format(joint_params[4])
        out_pos = out_pos + "0, "
        out_pos = out_pos + "{:.6f}".format(joint_params[5])
        out_pos = out_pos + "0, "
        out_pos = out_pos + "{:.6f}".format(vel)
        out_pos = out_pos + "0, "
        out_pos = out_pos + "{:.6f}".format(acc)
        out_pos = out_pos + "0, "
        out_pos = out_pos + "{:.6f}".format(time)
        out_pos = out_pos + "0)\r\n"

        self.client_hcr_socket.send(out_pos.encode())

        '''
        if move_wait == True:
            self.wait_move_end(joint_params)
        '''

        print('HCR3_Mani :: ' + out_pos)
        return out_pos

    def move_l(self, pose_param, vel, acc, time, move_wait = True):
        print('HCR3 :: movel::' + str(pose_param))

        if len(pose_param) != 6:
            print("Hcr3:: move_l function: input param length -> 9")
            return

        out_pos = "movel(p["
        out_pos = out_pos + "{:.6f}".format(pose_param[0])
        out_pos = out_pos + "0, "
        out_pos = out_pos + "{:.6f}".format(pose_param[1])
        out_pos = out_pos + "0, "
        out_pos = out_pos + "{:.6f}".format(pose_param[2])
        out_pos = out_pos + "0, "
        out_pos = out_pos + "{:.6f}".format(pose_param[3])
        out_pos = out_pos + "0, "
        out_pos = out_pos + "{:.6f}".format(pose_param[4])
        out_pos = out_pos + "0, "
        out_pos = out_pos + "{:.6f}".format(pose_param[5])
        out_pos = out_pos + "0, "
        out_pos = out_pos + "{:.6f}".format(vel)
        out_pos = out_pos + "0, "
        out_pos = out_pos + "{:.6f}".format(acc)
        out_pos = out_pos + "0, "
        out_pos = out_pos + "{:.6f}".format(time)
        out_pos = out_pos + "0)\r\n"

        self.client_hcr_socket.send(out_pos.encode())

        if move_wait == True:
            thread_show = threading.Thread(target=self.wait_move_pose)
            thread_show.daemon = True
            thread_show.start()
            self.pose_param = pose_param

            while True:
                if self.robot_state is 'arrive':
                    self.robot_state = 'moving'
                    break

        return out_pos

    def wait_move_pose(self):
        print('Hcr3::wait_move_end')
        try_count = 0

        while True:
            gapSum = 0
            try_count += 1
            #time.sleep(0.001)

            for i in range(0, 3):
                gapSum += abs(self.pose_param[i] - self.c_pose[i])

            if gapSum < 6:
                time.sleep(0.01)
                self.robot_state = 'arrive'
                return

            if try_count > 2000:
                try_count = 0
                self.robot_state = 'arrive'
                print('Hcr3::Error wait_move_pose ')
                print(str(self.pose_param))
                print(str(self.c_pose))
                return

    def wait_move_joint(self, joint_param):
        print('Hcr3::wait_move_joint')
        '''
        while (True):
            gapSum = 0
            time.sleep(0.01)

            for i in range(0, 3):
                gapSum += abs(joint_param[i] - self.c_pose[i])

            if (gapSum < 6):
                time.sleep(0.01)
                return
        '''

    def set_home_rot(self, joint_param):
        if len(joint_param) != 6:
            print("Hcr3:: joint_param parameter -> 6")
            return

        self.home_rot = joint_param
        print('Hcr3::set_home_rot::'+str(self.home_rot))

    def move_home(self):
        time.sleep(2)
        self.move_j(self.home_rot, self.jvel, self.jacc, self.jtime)
        time.sleep(2)
        print('Hcr3::move_home')

    def set_init_pos(self, pos_param):
        if len(pos_param) != 6:
            print("Hcr3:: pos_param parameter -> 6")
            return

        self.init_pose2 = pos_param
        print('Hcr3::set_init2_pos::' + str(self.init_pose2))

    def move_init2(self):
        self.move_l(self.init_pose2, self.lvel, self.lacc, self.ltime)
        print('Hcr3::move_init2')

    def set_init2_pos(self, pos_param):
        if len(pos_param) != 6:
            print("Hcr3:: pos_param parameter -> 6")
            return

        self.init_pose = pos_param
        print('Hcr3::set_init_pos::' + str(self.init_pose))

    def move_init(self):
        print('Hcr3::move_init')
        self.move_l(self.init_pose, self.lvel, self.lacc, self.ltime)

    def set_drop_pos(self, pos_param):
        if len(pos_param) != 6:
            print("Hcr3:: pos_param parameter -> 6")
            return

        self.drop_pose = pos_param
        print('Hcr3::set_drop_pos::' + str(self.drop_pose))

    def move_drop(self):
        self.move_l(self.drop_pose, self.lvel, self.lacc, self.ltime)
        print('Hcr3::move_drop')

    def stop(self):
        print('Hcr3::disconnect_robot')

    def get_pos(self):
        return self.c_pose

    def get_rot(self):
        return self.c_joint

    def turn_on_io(self):
        print('Hcr3::disconnect_robot')

    def turn_off_io(self):
        print('Hcr3::disconnect_robot')

    def gripper_close(self):
        out_pos = "GOPEN\r\n"
        self.client_hcr_socket.send(out_pos.encode())

    def gripper_open(self):
        out_pos = "GCLOSE\r\n"
        self.client_hcr_socket.send(out_pos.encode())

    def turnGripper_close(self):
        out_pos = "TOPEN\r\n"
        self.client_hcr_socket.send(out_pos.encode())

    def turnGripper_open(self):
        out_pos = "TCLOSE\r\n"
        self.client_hcr_socket.send(out_pos.encode())

    def gk_gripper_close(self):
        out_pos = "GKGOPEN\r\n"
        self.client_hcr_socket.send(out_pos.encode())

    def gk_gripper_open(self):
        out_pos = "GKGCLOSE\r\n"
        self.client_hcr_socket.send(out_pos.encode())