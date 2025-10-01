import sys,os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from interface.vision import *
import copy
import math
import numpy as np


if __name__ == '__main__':
    root_dir=__file__.split("test_vision")[0]
    # If connection fails:
    # 1. Run `ifconfig` inside the vision container
    # 2. Find the eth0 IP address
    # 3. Set HOST to that IP
    vision=Vision(HOST="172.17.0.2",PORT=8801)
    enable_onair=False
    if enable_onair:
        # realsense data 획득
        ret_rs,rgb,depth=vision.get_rs()
    else:
        imgpath = f"{root_dir}APP_Base/APP/data/dataset/2025_05_12_17_58_29__rgb.png"
        rgb=cv2.imread(imgpath)
        imgpath = f"{root_dir}APP_Base/APP/data/dataset/2025_05_12_17_58_29__depth.png"
        depth=cv2.imread(imgpath,-1)
    # grasping
    ret_pick,ret,ret_img=vision.get_pick(rgb,depth,
                            aDict={
                            # ROI (x, y, w, h) in image
                            "crop_roi": [450, 172, 1040, 513],

                            # Grasp width constraints (pixels)
                            "dmin": 50,   # minimum width
                            "dmax": 70,   # maximum width
                            "df": 50,

                            # Bounding box size ratio relative to image size
                            # ratio = object_width/img_width or object_height/img_height
                            # "min_ratio": 0.01,  # lower bound (currently disabled)
                            "max_ratio": 1,

                            # Object size constraints (pixel area)
                            "min_mass": 500,     # minimum object area
                            "max_mass": 10000,   # maximum object area

                            # Number of top candidate boxes to keep
                            "topn": 100,
                    }
                    )
    [Robot_X, Robot_Y, Robot_Z,robot_z_angle, ret_back]=ret
    print(f"[Robot_X, Robot_Y, Robot_Z]: [{Robot_X}, {Robot_Y}, {Robot_Z}]")
    print(f"robot_z_angle: {robot_z_angle}")
    cv2.imshow("ret_img",ret_img)
    cv2.waitKey(100)

    if enable_onair:
        # TIS RGB data 획득
        ret_tis,img,origin=vision.get_TIS()
    else:
        imgpath = f"{root_dir}APP_Base/APP/data/angle/2025_05_27_19_00_54__tis.png"
        origin=cv2.imread(imgpath)
    # angle data 획득
    ret_angle,ret,ret_img = vision.get_angle(origin,
                            aDict={
                            # ROI 설정 (x, y, w, h) in image
                            "roi":[330, 12, 1393, 1098],

                            # Blur kernel size for HSV preprocessing (must be odd).
                            # Larger values → smoother contours
                            "blur_size": 11,

                            # HSV threshold preprocessing
                            "max_h_threshold": 120,
                            "max_s_threshold": 100,
                            "max_v_threshold": 105,

                            # Circle detection parameters (cv2.HoughCircles 참고)
                            "param1": 30,
                            "param2": 40,
                            "minRadius": 140,
                            "maxRadius": 165,

                            # Polar coordinates angle range
                            "angle_range": [-120, 90],

                            # Median filter size for polar signal (must be odd)
                            "single_med_filtersize": 11,

                            # 기준 각도 (reference angle)
                            "ref_angle":-86.94130242590418
                            }
                            )

    print(f"ret['differ_angle']: {ret['differ_angle']}")
    cv2.imshow("ret_img",ret_img)
    cv2.waitKey(100)