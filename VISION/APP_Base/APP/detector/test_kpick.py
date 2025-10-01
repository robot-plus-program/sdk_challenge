import struct
import threading
from signal import signal, SIGPIPE, SIG_DFL

# Ignore SIG_PIPE and don't throw exceptions on it... (http://docs.python.org/library/signal.html)
signal(SIGPIPE, SIG_DFL)
import socket
import cv2
import time
import datetime
import numpy as np
import copy
from APP_Calibration.keticalibsdk.calib.calibration import calib_opencv
from ketisdk.sensor.realsense_sensor import RSSensor
from ketisdk.vision.utils.rgbd_utils_v2 import RGBD
from APP_Base.robot.robot import robotsdk
from ketirobotsdk.ketirobotsdk.sdk import UR10
from APP_Base.robot.keti_zimmer_gripper_jog import KetiZimmer
from APP_Base.APP.detector.kpick_detector import detector
import os


def mouse_event(event, x, y, flags, param):
    global gx, gy
    global display_img

    if event == cv2.EVENT_FLAG_LBUTTON:
        Xc = x
        Yc = y
        Zc = depth_data[y, x]
        print(Xc, Yc, Zc)

def function(cur_rgb_data,cur_depth_data):
    print("method")
if __name__ == '__main__':
    cv2.namedWindow('result')
    cv2.setMouseCallback('result', mouse_event)

    result_save_dir = '/media/keti/5aa8b858-eaa4-4b47-ade7-e2704a1b97da/DB/Yerim/'
    save_dir_folder = result_save_dir + datetime.datetime.now().strftime('%Y_%m_%d_%H_%M_%S')
    if os.path.exists(save_dir_folder) is False:
        os.mkdir(save_dir_folder)

    sensor_enable = True
    robot_enable = False
    grasp_enable = True

    if grasp_enable:
        kpick=detector('/home/keti/Desktop/PoC_Talos/APP_Talos/APP_Base/APP/detector/setting.json')

    #gripper
    if robot_enable:
        gripper = KetiZimmer()
        gripper.connect('192.168.1.254', 502)
        gripper.gripper_init()
        if gripper.grip_distance > 200:
            gripper.gripper_release()


    count = 0

    #camera
    if sensor_enable:
        sensor = RSSensor()
        sensor.get_device_sn()
        sensor.start()
        ppx, ppy, fx, fy = sensor.intr_params.ppx, sensor.intr_params.ppy, sensor.intr_params.fx, sensor.intr_params.fy
        np.save("intr_param.npy",[ppx, ppy, fx, fy])
    else:
        ppx, ppy, fx, fy = np.load("intr_param.npy")


    '''Calibration'''
    calib = calib_opencv()
    calib.set_intrinsic(ppx, ppy, fx, fy)
    #robot
    if robot_enable:
        robot=robotsdk(ip="192.168.0.111",robotname=UR10,port=30003)
        acc = 1
        vel = 1
        init_joint = [0.9778629541397095, -1.8185823599444788, 1.6404414176940918, -1.3920748869525355, -1.5758798758136194, -1.3731969038592737, 6.2817759753808e-310]
        init_pose = [-0.707118010498209, -0.7070955488538756, -6.340419014502592e-05, -0.15606916171152413,
                     -0.7070955516690952, 0.7071180084182227, 5.459326415919072e-05, -0.5299488021643288,
                     6.231590576353444e-06, 8.343670114757943e-05, -0.9999999964997421, 0.7320086616742838,
                     0.0, 0.0, 0.0, 1.0]
    #set workspace
    ws_pts = [(382, 202), (898, 202), (898, 473), (382, 473)]


    # check workspace
    while 1:
        if sensor_enable:
            rgb_data, depth_data = sensor.get_data()  # realsense data
            rgb_data = rgb_data[:, :, ::-1]
            rgbd = RGBD(rgb_data, depth_data, depth_min=400, depth_max=800)
            if ws_pts is not None:
                rgbd.set_workspace(pts=ws_pts)
            rgbimg = rgbd.draw_workspace(rgbd.rgb)
            cv2.imshow("result", rgbimg)
            key = cv2.waitKey(30)
            if key == ord('q'):
                print("set pose")
                break
        else:
            break

    while True:

        try:

            # if robot_enable:
            #     robot.movej(init_joint)

            if sensor_enable:
                rgb_data, depth_data = sensor.get_data()  # realsense data
                rgb_data = rgb_data[:, :, ::-1]

            else:
                rgb_data = cv2.imread(
                    "../data/bin_box_rgb_2.png")
                depth_data = cv2.imread(
                    "../data/bin_box_depth_2.png", -1)

            rgbd = RGBD(rgb_data, depth_data, depth_min=400, depth_max=600)
            if ws_pts is not None:
                rgbd.set_workspace(pts=ws_pts)
            rgbimg = rgbd.draw_workspace(rgbd.rgb)
            cv2.imshow("result", rgbimg)
            msg = cv2.waitKey(100)
            if grasp_enable:
                kpick.load_param()
                ret=kpick.run(rgb_data,depth_data)
                boxes=copy.deepcopy(ret[0].boxes)
                for box in boxes:
                    line_h=int((box[1]+box[3])/2)
                    cv2.line(ret[1],(box[0],line_h),(box[2],line_h),(255,0,0))
                cv2.imshow("ret", ret[1])
                cv2.waitKey(100)

        except Exception as e:
            print(e)
            cv2.destroyAllWindows()
