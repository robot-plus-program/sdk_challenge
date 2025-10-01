import socket
import time

def listToByte(list):
    blist = str(list)
    blist = blist.encode()
    return blist

class m1013:
    def __init__(self, ip, port, bufsize, timeout):
        self._ip = ip
        self._port = port
        self._bufsize = bufsize
        self._timeout = timeout
        self._socket = None
        self._client_socket = None
        self._client_addinfo = None
        self._recvData = None
        self._joint6 = [0, 0, 0, 0, 0, 0]
        self._pose = [0, 0, 0, 0, 0, 0]
        self._jspeed = [0, 0, 0, 0, 0, 0]
        self._brun = True

        self.p1 = [356.0, -225.0, 222.0, 0, 180, 90]
        self.p2 = [444, 89, 116.0, 0, 180, 90]
        self.p3 = [444, 491, 116.0, 0, 180, 90]  # init
        self.p4 = [200, -200, 400, 0, 180, 90]

        self.v = 400
        self.a = 600
        self.t = 0

        self.trayGap = 2.0
        self.insize = 0
        self.checkMove = True

        try:    
            #self.OGM = onrobotGripper_master()
            self._connect_socket()
        except:
            print('Already Connected')

        time.sleep(1)

        '''
        self.hcr3_receive_thread = threading.Thread(target=self.receive_data)
        self.hcr3_receive_thread.daemon = True
        self.hcr3_receive_thread.start()
        '''

        print('m1013_INIT :: robot_master')

    def __del__(self):
        _brun = False
        #self.send_end()

        if self._client_socket != None:
            self._client_socket.close()

        if self._socket != None:
            self._socket.close()


    def _connect_socket(self):
        print('opening socket - server info [ip:%s}] [port:%s]' % (str(self._ip), str(self._port)))

        if self._socket != None:
            self._socket.close()
            self._client_socket.close()

        self._client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        print('m1013 robot connected!')
        print('m1013 client info [ip:%s}] [port:%s]' % self._client_addinfo)

    # rnlcksg...
    def move_p1(self, insize):
        p1_up = self.p1.copy()
        p1_up[2] -= (insize * self.trayGap)
        print('p1_up',p1_up)
        self.move_l(p1_up, self.v, self.a, self)

    def move_p1_up(self):
        p1_up = self.p1.copy()
        p1_up[2] += 100
        self.move_l(p1_up, self.v/2, self.a/4, self)

    def move_p2_up(self):
        p2_up = self.p2.copy()
        p2_up[2] += 200
        self.move_l(p2_up, self.v, self.a, self)

    def move_p3_up(self):
        p3_up = self.p3.copy()
        p3_up[2] += 200
        self.move_l(p3_up, self.v, self.a, self)

    def move_p4_up(self):
        p4_up = self.p4.copy()
        p4_up[2] += 100
        self.move_l(p4_up, self.v, self.a, self)

    def move_p2(self, cnt):
        p2_up = self.p2.copy()
        p2_up[2] += (cnt * self.trayGap)
        print('p2_up', p2_up)
        self.move_l(p2_up, self.v, self.a, self)

    def move_p3(self):
        self.move_l(self.p3, self.v, self.a, self)

    def move_p4(self):
        self.move_l(self.p4, self.v, self.a, self)

    def waitRecv(self):
        print('What...')

    def move_l(self, pose_param, vel, acc, time):
        print(pose_param)
        if len(pose_param) != 6:
            print("move_j function: input param length -> 9")
            return

        sendMessage = ['movelw',pose_param,self.v, self.a, self.t]
        sendMessage = listToByte(sendMessage)

        self._client_socket.sendto(sendMessage, ("127.0.0.1", 6666))
        self._client_socket.recvfrom(1024)
        return pose_param

    def move_l_up(self, pose_param, vel, acc, time):
        pose_param_up = pose_param.copy()
        pose_param_up[2] += 200
        self.move_l(pose_param_up, vel, acc, time)

    def stop(self):
        self._brun = False

    def gripper_close(self):
        self.turn_off(int(1))
        return

    def gripper_open(self):
        self.turn_on(int(1))
        return

    def turn_on(self, io):
        sendMessage = ['turnon', io]
        sendMessage = listToByte(sendMessage)
        self._client_socket.sendto(sendMessage, ("127.0.0.1", 6666))
        return

    def turn_off(self, io):
        sendMessage = ['turnoff', io]
        sendMessage = listToByte(sendMessage)
        self._client_socket.sendto(sendMessage, ("127.0.0.1", 6666))
        return

    def send_end(self):
        out_pos = "End"
        self._client_socket.send(out_pos.encode())

    def move_wait(self):
        time.sleep(0.2)
        while True:
            temp = self._jspeed[0] + self._jspeed[1] + self._jspeed[2]+ self._jspeed[3]+ self._jspeed[4]+ self._jspeed[5]
            if temp < 0.0015:
                return
            else:
                time.sleep(0.2)
                print('moving')

if __name__ == '__main__':

    MRM = m1013('192.168.1.55', 6666, 65535, 1)

    cnt = 0






