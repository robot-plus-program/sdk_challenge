from signal import signal, SIGPIPE, SIG_DFL
# Ignore SIG_PIPE and don't throw exceptions on it... (http://docs.python.org/library/signal.html)
signal(SIGPIPE, SIG_DFL)
import socket
import cv2
import time
import datetime
import numpy
import numpy as np
import copy
import base64
from ketisdk.sensor.realsense_sensor import RSSensor
from ketisdk.vision.utils.rgbd_utils_v2 import RGBD
import json


class GraspDetector:
    def __init__(self, ip, port):

        self.wts = np.array([[387, 198], [968, 202], [970, 538], [374, 536]]).flatten()
        self.threshold = 0.8
        self.min_value = 70
        self.max_value = 80
        self.npose = 5000
        self.top_n = 200
        self.angle_step = 10
        self.set_param()

        self.TCP_SERVER_IP = ip
        self.TCP_SERVER_PORT = port
        self.connectCount = 0
        self.rgb_data=[]
        self.connectServer()
    def set_parameter(self,param):
        self.param=copy.deepcopy(param)

    def set_param(self):
        param=copy.deepcopy(self.wts)
        param=np.append(param,self.threshold)
        param=np.append(param,self.min_value)
        param=np.append(param,self.max_value)
        param=np.append(param,self.npose)
        param=np.append(param,self.top_n)
        param=np.append(param,self.angle_step)
        self.param=copy.deepcopy(param)

    def set_workspace(self,wt_array):
        self.wts=wt_array.flatten()

    def set_threshold(self,threshold):
        self.threshold=threshold

    def set_width_range(self,min_value,max_value):
        self.min_value=min_value
        self.max_value=max_value

    def set_npose(self,npose):
        self.npose=npose

    def set_top_n(self,top_n):
        self.top_n=top_n

    def set_angle_set(self,angle_step):
        self.angle_step=angle_step

    def get_data(self,rgb_data, depth):
        start_time = time.perf_counter()


        self.sock.send("/grab".encode('utf-8'))
        # send param
        stringData = base64.b64encode(np.array(self.param))
        length = str(len(stringData))
        # image data
        self.sock.sendall(length.encode('utf-8').ljust(64))
        self.sock.send(stringData)

        # send image
        send_data = numpy.dstack((rgb_data, depth))
        stime = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')
        stringData = base64.b64encode(send_data)
        length = str(len(stringData))

        # image data
        self.sock.sendall(length.encode('utf-8').ljust(64))
        self.sock.send(stringData)
        self.sock.send(stime.encode('utf-8').ljust(64))

        # image size
        stringData = base64.b64encode(np.array(rgb_data.shape))
        length = str(len(stringData))
        self.sock.sendall(length.encode('utf-8').ljust(64))
        self.sock.send(stringData)

        #recv
        #grips
        rec_data = self.sock.recv(64)
        length1 = rec_data.decode('utf-8')
        rec_data = self.recvall(self.sock, int(length1))
        grips = np.frombuffer(base64.b64decode(rec_data), np.float)
        grips=grips.reshape([int(grips.size / 11), 11])
        detected = {'grip': grips}

        # image
        rec_data = self.sock.recv(64)
        length1 = rec_data.decode('utf-8')
        rec_data = self.recvall(self.sock, int(length1))
        img_data = np.frombuffer(base64.b64decode(rec_data), np.uint8)
        # size
        rec_data = self.sock.recv(64)
        length1 = rec_data.decode('utf-8')
        rec_data = self.recvall(self.sock, int(length1))
        img_size = np.frombuffer(base64.b64decode(rec_data), np.int)

        result_image = img_data.reshape(np.append(img_size[0:2], [3]))
        detected.update({'im': result_image})

        # best_index
        rec_data = self.sock.recv(64)
        best_index = rec_data.decode('utf-8')
        detected.update({'best_ind': best_index})

        # best_n_inds
        rec_data = self.sock.recv(64)
        length1 = rec_data.decode('utf-8')
        rec_data = self.recvall(self.sock, int(length1))
        best_n_inds = np.frombuffer(base64.b64decode(rec_data), np.int)
        detected.update({'best_n_inds': best_n_inds})

        # best_grip
        rec_data = self.sock.recv(64)
        length1 = rec_data.decode('utf-8')
        rec_data = self.recvall(self.sock, int(length1))
        best_grip = np.frombuffer(base64.b64decode(rec_data), np.float)
        detected.update({'best': best_grip})

        rec_data = self.sock.recv(64)
        print(rec_data)
        end_time = time.process_time()
        end_time = time.perf_counter()
        print(f"time elapsed : {int(round((end_time - start_time) * 1000))}ms")
        return detected

    def connectServer(self):
        try:
            self.sock = socket.socket()
            self.sock.connect((self.TCP_SERVER_IP, self.TCP_SERVER_PORT))
            print(
                u'Client socket is connected with Server socket [ TCP_SERVER_IP: ' + self.TCP_SERVER_IP + ', TCP_SERVER_PORT: ' + str(
                    self.TCP_SERVER_PORT) + ' ]')
            self.connectCount = 0

        except Exception as e:
            print(e)
            self.connectCount += 1
            if self.connectCount == 10:
                print(u'Connect fail %d times. exit program' % (self.connectCount))
                sys.exit()
            print(u'%d times try to connect with server' % (self.connectCount))
            self.connectServer()

    def recvall(self, sock, count):
        buf = b''
        while count:
            newbuf = sock.recv(count)
            if not newbuf: return None
            buf += newbuf
            count -= len(newbuf)
        return buf
def print_grips(grips):
    print("X,Y,Z", grips[0:3], end=" / ")
    print("box size", grips[3:5], end=" / ")
    print("angle", grips[5], end=" / ")
    print("left point", grips[6:8], end=" / ")
    print("right point", grips[8:10], end=" / ")
    print("score", grips[10])

if __name__=='__main__':
    import sys

    sensor_enable=True
    for arg in sys.argv:
        print(arg)
    rgb_data = cv2.imread("../data/Grasp_RGB.png")
    depth_data = cv2.imread("../data/Grasp_depth.png", -1)
    TCP_IP = '192.168.137.50'
    TCP_PORT = 5000
    GraspPointDetector=GraspDetector(TCP_IP, TCP_PORT)
    GraspPointDetector.set_workspace(np.array([[500, 198], [940,202], [940, 538], [500, 536]]).flatten())
    GraspPointDetector.set_threshold(0.6)
    GraspPointDetector.set_width_range(30,40)
    GraspPointDetector.set_npose(1000)
    GraspPointDetector.set_top_n(500)
    GraspPointDetector.set_angle_set(5)
    count=0

    if sensor_enable:
        sensor = RSSensor()
        sensor.start()
        ppx, ppy, fx, fy = sensor.intr_params.ppx, sensor.intr_params.ppy, sensor.intr_params.fx, sensor.intr_params.fy

    else:
        ppx, ppy, fx, fy = np.load("intr_param.npy")
    # set workspace
    try:
        with open("../configs/workspace/config_roi.json", "r") as st_json:
            ws_pts = json.load(st_json)
    except:
            ws_pts = [(500, 85), (1081, 84), (1036, 663), (870, 659), (768, 557), (716, 577), (753, 675), (235, 667)]
    try:
        while 1:
            if sensor_enable:
                rgb_data, depth_data = sensor.get_data()  # realsense data
            else:
                rgb_data = cv2.imread("../data/Grasp_RGB.png")
                depth_data = cv2.imread("../data/Grasp_depth.png", -1)

            rgbd = RGBD(rgb_data, depth_data, depth_min=400, depth_max=600)
            if ws_pts is not None:
                rgbd.set_workspace(pts=ws_pts)
            rgbimg = rgbd.draw_workspace(rgbd.rgb)
            cv2.imshow("input", rgbimg[:, :, ::-1])
            cv2.imshow("depth", (depth_data.astype(float)/4).astype(np.uint8))
            cv2.waitKey(1000)
            count+=1;
            cur_rgb_data=copy.deepcopy(rgb_data)
            cur_depth_data=copy.deepcopy(depth_data)
            disp_text=str(count)
            cv2.putText(cur_rgb_data, disp_text, (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 5)

            GraspPointDetector.set_param()
            ret=GraspPointDetector.get_data(cur_rgb_data, cur_depth_data)

            grips = np.array(ret["grip"])
            best_index = ret['best_ind']
            best_n_inds = np.array(ret['best_n_inds'])
            best_grip = np.array(ret['best'])
            result_image = np.array(ret['im']).astype(np.uint8)

            print(count)
            # draw the all of grips
            draw_rgb=copy.deepcopy(rgbimg)
            for cur_grip in grips:
                x0,y0,x1,y1=cur_grip[-5:-1].astype(int)
                cv2.line(draw_rgb,(x0,y0),(x1,y1),(255,0,0),1)
            # draw the top n grips
            for cur_grip in grips[best_n_inds,:]:
                x0,y0,x1,y1=cur_grip[-5:-1].astype(int)
                cv2.line(draw_rgb,(x0,y0),(x1,y1),(0,255,0),1)

            # draw the best grips
            if best_grip.__len__()>0:
                x0,y0,x1,y1=best_grip[-5:-1].astype(int)
                cv2.line(draw_rgb,(x0,y0),(x1,y1),(0,0,255),5)
                print("best grips")
                print_grips(best_grip)
                cv2.imshow("result", draw_rgb)
                key = cv2.waitKey(100)
                if key == ord('q'):
                    print("exit")
                    break

    except Exception as e:
        print(e)
        cv2.destroyAllWindows()