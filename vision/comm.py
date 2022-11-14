import threading
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
from demo_03_2_GraspPoint import GraspDetector

robot_connected = False
robot_request = 0
vision_data = []
cal_finish = 0

def comm_func():
    import socket
    global robot_connected, robot_request, vision_data
    ip = '127.0.0.1'
    port = 6000
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((ip, port))
    server.listen(5)

    wait_cnt = 0
    while True:
        try:
            if robot_connected is False:
                print('TCPServer waiting for client on port ', port)
                client, addr = server.accept()
                print('I got a connection from ', addr)
                robot_connected = True

            data = client.recv(1024).decode()
            print(len(data))
            if len(data) == 0:
                robot_connected = False
            elif len(data) >= 1:
                if data == '1':
                    robot_request = 1
                else:
                    robot_request = 0
                print('robot request ', robot_request)

                while robot_request == 1:
                    time.sleep(1)
                    wait_cnt += 1
                    if wait_cnt >= 5:
                        wait_cnt = 0
                        break

                if len(vision_data) >= 3:
                    send_buf = bytearray(24)
                    send_buf[0:8] = bytearray(struct.pack("d", vision_data[0]))
                    send_buf[8:16] = bytearray(struct.pack("d", vision_data[1]))
                    send_buf[16:24] = bytearray(struct.pack("d", vision_data[2]))
                    print(send_buf)
                    client.send(send_buf)
        except:
            robot_connected = False

        time.sleep(1)

if __name__ == '__main__':
    import struct
    import sys

    thread = threading.Thread(target=comm_func)
    thread.deamon = True
    thread.start()

    sensor_enable = False
    for arg in sys.argv:
        print(arg)
    rgb_data = cv2.imread("../data/Grasp_RGB.png")
    depth_data = cv2.imread("../data/Grasp_depth.png", -1)
    TCP_IP = '192.168.137.50'
    TCP_PORT = 5000
    GraspPointDetector = GraspDetector(TCP_IP, TCP_PORT)
    GraspPointDetector.set_workspace(np.array([[500, 198], [940, 202], [940, 538], [500, 536]]).flatten())
    GraspPointDetector.set_threshold(0.6)
    GraspPointDetector.set_width_range(30, 60)
    GraspPointDetector.set_npose(1000)
    GraspPointDetector.set_top_n(500)
    GraspPointDetector.set_angle_set(5)
    count = 0

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

    while True:
        if robot_request == 1:
            if sensor_enable:
                rgb_data, depth_data = sensor.get_data()  # realsense data
            else:
                rgb_data = cv2.imread("../data/Grasp_RGB.png")
                depth_data = cv2.imread("../data/Grasp_depth.png", -1)

            rgbd = RGBD(rgb_data, depth_data, depth_min=400, depth_max=600)
            if ws_pts is not None:
                rgbd.set_workspace(pts=ws_pts)
            rgbimg = rgbd.draw_workspace(rgbd.rgb)
            # cv2.imshow("input", rgbimg[:, :, ::-1])
            # cv2.imshow("depth", (depth_data.astype(float)/4).astype(np.uint8))
            cv2.waitKey(1)
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

            vision_data.clear()
            vision_data.append(best_grip[0])
            vision_data.append(best_grip[1])
            vision_data.append(best_grip[2])
            robot_request = 0

        time.sleep(1)


