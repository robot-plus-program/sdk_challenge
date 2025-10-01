from signal import signal, SIGPIPE, SIG_DFL

# Ignore SIG_PIPE and don't throw exceptions on it... (http://docs.python.org/library/signal.html)
signal(SIGPIPE, SIG_DFL)
import cv2
import time
import datetime
import numpy as np
import copy
from APP_Calibration.keticalibsdk.calib.calibration import calib_opencv
from ketisdk.sensor.realsense_sensor import RSSensor
from ketisdk.vision.utils.rgbd_utils_v2 import RGBD
from APP_Base.robot.robot import robotsdk
from ketirobotsdk.ketirobotsdk.sdk import M1013
from APP_Base.robot.keti_zimmer_gripper_jog import KetiZimmer
from APP_Base.APP.test.test_02_tiscamera import TISCamera
from APP_Base.APP.detector.kpick_detector import detector
from APP_Base.APP.detector.angle_detection import calculate_dif_angle
import os

def mouse_event(event, x, y, flags, param):
    global gx, gy
    global display_img

    if event == cv2.EVENT_FLAG_LBUTTON:
        Xc = x
        Yc = y
        Zc = depth_data[y, x]
        print(Xc, Yc, Zc)

if __name__ == '__main__':

    # 결과 시각화용 윈도우 생성
    cv2.namedWindow('result')
    cv2.setMouseCallback('result', mouse_event)
    
    
    
    # 로그 저장 폴더 생성
    result_save_dir = '/media/keti/5aa8b858-eaa4-4b47-ade7-e2704a1b97da/DB/Talos/log/'
    save_dir_folder = result_save_dir + datetime.datetime.now().strftime('%Y_%m_%d_%H_%M_%S')
    if os.path.exists(save_dir_folder) is False:
        os.mkdir(save_dir_folder)

    # 센서 및 기능 활성화 여부
    sensor_enable = True
    robot_enable = True
    grasp_enable = True
    place_enable = True
    tis_sensor_enable=True
   
    # 로봇 초기화
    if robot_enable:
        robot=robotsdk(ip="192.168.137.50",robotname=M1013,port=12345)
        acc = 1
        vel = 1

        #picking
        init_joint=[2.9919661435369993, -0.14149567950770023, 1.6504852971450106, 3.616965391285435e-05, 1.6348476146982287, 1.4206483698006651, 6.76353516188367e-310]
        init_pose = [0.00041608748554342045, 0.9999998448719181, -0.00037030709442165317, -0.4701260070800781,
                     0.9999972054141893, -0.0004169481229974181, -0.0023270835984908872, 0.0360301628112793,
                     -0.0023272376363428237, -0.0003693377892035561, -0.999997223773437, 0.6797732543945313,
                     0.0, 0.0, 0.0, 1.0]
        # angle
        check_angle_pose=[4.2804369229876293e-05, 0.9999999942311635, -9.851628877361173e-05, -0.5328749389648437,
                          0.9999999918000846, -4.281625927016197e-05, -0.00012069216595280976, 0.4479977111816406,
                          -0.00012069638335551884, -9.85111218137519e-05, -0.9999999878639709, 0.4878895874023437,
                          0.0, 0.0, 0.0, 1.0]

    # TIS 카메라 설정
    if tis_sensor_enable:
        setting_1 = {
            "device_serial": "20120594",
            "sensor_size": (1920, 1080),

            "auto_focus": "Off",
            "focus": 630,

            "auto_exposure": "Off",
            "exposure": 50000,

            "GainAuto": "Off",
            "Gain": 150,

            "fps": 30
        }
        tis_setting = setting_1
        tisCam = TISCamera()
        tisCam.set_device(tis_setting['device_serial'], tis_setting['sensor_size'], tis_setting['fps'],
                          showvideo=False)
        save_folder = "/media/keti/5aa8b858-eaa4-4b47-ade7-e2704a1b97da/DB/Talos/"
        tisCam.start()
        # tisCam.setting(tis_setting['focus'],tis_setting['exposure'])
        Gain = tis_setting["Gain"]
        exposure = tis_setting["exposure"]
        focus = tis_setting["focus"]
        tisCam.Tis.Set_Property("GainAuto", "Off")
        tisCam.Tis.Set_Property("Gain", Gain)
        tisCam.Tis.Set_Property("ExposureAuto", "Off")
        tisCam.Tis.Set_Property("ExposureTime", exposure)
        tisCam.Tis.Set_Property("FocusAuto", "Off")
        tisCam.Tis.Set_Property("Focus", focus)
        point = []

        while False:
            tis_rgb, orignal = tisCam.get_image()
            # tis_rgb=tisCam.get_image_origin(out_size=(7716, 5360))

            if tis_rgb is not None:
                h, w, c = orignal.shape
                draw_rgb = cv2.cvtColor(orignal, cv2.COLOR_BGRA2BGR)
                cv2.imshow('TIS Cam', draw_rgb)  # Display the result
                key = cv2.waitKey(1)

            if key == ord('q'):
                print("set pose")
                break
    # 그리퍼 초기화
    if robot_enable:
        gripper = KetiZimmer()
        gripper.connect('192.168.137.254', 502)
        gripper.gripper_init()
        gripper_width=1500

        if gripper.grip_distance > 200:
            gripper.gripper_release()


    count = 0

    # Realsense 센서 초기화 및 내부 파라미터 저장
    if sensor_enable:
        sensor = RSSensor()
        sensor.get_device_sn()
        sensor.start()
        ppx, ppy, fx, fy = sensor.intr_params.ppx, sensor.intr_params.ppy, sensor.intr_params.fx, sensor.intr_params.fy
        np.save("intr_param.npy",[ppx, ppy, fx, fy])
    else:
        ppx, ppy, fx, fy = np.load("intr_param.npy")

    min_binbox_mm=0.188


    # 캘리브레이션
    calib = calib_opencv()
    calib.set_intrinsic(ppx, ppy, fx, fy)
    #robot


    # set workspace
    p1=[450,172]
    p2=[    1040,513]
    push_zone=50
    ws_pts = [(p1[0], p1[1]), (p2[0], p1[1]), (p2[0], p2[1]), (p1[0], p2[1])]
    ws_pts_zone = [(p1[0]+push_zone, p1[1]+push_zone), (p2[0]-push_zone, p1[1]+push_zone), (p2[0]-push_zone, p2[1]-push_zone), (p1[0]+push_zone, p2[1]-push_zone)]

    # check workspace
    for i in range(0,100):
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

    # detection	
    kpick=detector('/home/keti/Desktop/PoC_Talos/APP_Talos/APP_Base/APP/detector/setting.json')

    while True:

        try:

	    # 초기 자세로 이동 및 그리퍼 오픈
            if robot_enable:
                robot.movej(init_joint,vel=30)
                gripper.gripper_release()
                time.sleep(4)
	    # 센서 데이터 획득
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

            if 1:

                if grasp_enable:
                    # get_grabdata
                    if sensor_enable:

                        rgb_data, depth_data = sensor.get_data()  # realsense data
                        rgb_data = rgb_data[:, :, ::-1]
                        time_info = datetime.datetime.now().strftime('%Y_%m_%d_%H_%M_%S_')

                        input_path = save_dir_folder + '/' + time_info + "test_input.png"
                        depth_path = save_dir_folder + '/' + time_info + "test_depth.png"
                        ret_path = save_dir_folder + '/' + time_info + "test_result.png"
                        cv2.imwrite(input_path, rgb_data)
                        cv2.imwrite(depth_path, depth_data)

                    else:
                        rgb_data = cv2.imread("../data/bin_box_rgb.png")
                        depth_data = cv2.imread("../data/bin_box_depth.png", -1)

                    cur_rgb_data = copy.deepcopy(rgb_data)
                    cur_depth_data = copy.deepcopy(depth_data)

                    # Grasp Detection 수행
                    kpick.load_param()
                    ret = kpick.run(rgb_data, depth_data)
                    if ret[0] == None:
                        text="Empty box"
                        font=cv2.FONT_HERSHEY_SIMPLEX
                        font_scale=3
                        thickness=6
                        (text_width, text_height), baseline = cv2.getTextSize(text, font, font_scale, thickness)

                        # 텍스트 좌표 (중앙 정렬)
                        text_x = 640 - text_width // 2
                        text_y = 360 + text_height // 2
                        cv2.putText(rgb_data, text, (text_x, text_y),font , font_scale, (0, 255, 0), thickness)
                        cv2.imshow("result", ret[1])
                        cv2.waitKey(100)
                        continue

                    ret_select=kpick.select_best(ret[0],ret[1], depth_data)
                    select_ret=ret_select['select_ret']
                    depth=ret_select['depth_value']
                    depth=590
                    out=ret_select['out_img']
                    boxes = copy.deepcopy(ret[0].boxes)
                    for box in boxes:
                        line_h = int((box[1] + box[3]) / 2)
                        cv2.line(out, (box[0], line_h), (box[2], line_h), (255, 0, 0))
                    box=boxes[0]
                    line_h = int((box[1] + box[3]) / 2)
                    cv2.line(out, (box[0], line_h), (box[2], line_h), ( 0,255, 0),4)
                    grip_x,grip_y=((box[0:2]+box[2::])/2).astype(int)
                    cv2.imshow("result", out)
                    cv2.waitKey(100)
                    cv2.imwrite(ret_path, out)
                    if select_ret==False:
                        continue


                    # 픽킹 위치 -> 로봇 좌표계로 변환
                    robot_pose=list(robot.get_pose_1d())
                    robot_trans=calib.convert_c2r(grip_x,grip_y,depth)
                    robot_z=(robot_trans[2]-0.015)
                    robot_pose[3] += robot_trans[0]
                    robot_pose[7] += robot_trans[1]
                    robot_pose[11] += robot_z

                    up_pose=copy.deepcopy(robot_pose)
                    up_pose[11]=up_pose[11]+0.3

                    # 픽킹 동작 수행
                    robot.movel(up_pose)
                    gripper.grip_custom(gripper_width)
                    time.sleep(1)
                    robot.movel(robot_pose)
                    time.sleep(2)
                    gripper.gripper_grip()
                    gripper.gripper_release()
                    robot.movel(up_pose,30)
                    time.sleep(2)


                if place_enable:
                    robot.movel(check_angle_pose)
                    time.sleep(3)
                    if tis_sensor_enable:
                        tis_rgb, orignal = tisCam.get_image()
                        tis_path = save_dir_folder + '/' + time_info + "_tis.png"
                        tis_ret_path = save_dir_folder + '/' + time_info + "_tis_ret.png"
                        cv2.imwrite(input_path, cv2.cvtColor(orignal, cv2.COLOR_BGRA2RGB))
                    else:
                        orignal = cv2.imread("/home/keti/Desktop/PoC_Talos/APP_Talos/APP_Base/APP/test/ref.png")

                    ret_angle,X,Y,differ_angle,reference_angle,ret_angle_img=calculate_dif_angle(orignal)
                    while ret_angle==False:
                        ret_angle, X, Y, differ_angle, reference_angle, ret_angle_img = calculate_dif_angle(orignal)
                    cv2.imwrite(tis_ret_path,ret_angle_img)
                    cv2.imshow("ret_angle_img", ret_angle_img)
                    cv2.waitKey(100)

                    mat = robot.get_rotate_zt_robot(differ_angle).flatten()
                    robot.movel(mat)
                    time.sleep(2)
                time.sleep(2)

        except Exception as e:
            print(e)
            cv2.destroyAllWindows()
