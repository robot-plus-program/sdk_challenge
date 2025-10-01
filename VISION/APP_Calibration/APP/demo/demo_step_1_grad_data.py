
import os
import time

import cv2
import numpy as np
from ketisdk.sensor.realsense_sensor import RSSensor
import urx
import datetime
import json
import copy

class Sensor(RSSensor):
    def get_bgrd(self):
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)

        color_frame = aligned_frames.get_color_frame()  # bgr
        rgb = np.asanyarray(color_frame.get_data())[:, :, ::-1]

        depth_frame = aligned_frames.get_depth_frame()
        depth = np.asanyarray(depth_frame.get_data())

        rgb=cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR)
        return rgb, depth
if __name__ == '__main__':
    print("grab robot and sensor data")

    '''Robot'''
    # Robot ip
    # URX
    robot=urx.Robot("192.168.1.10")
    robot.set_tcp((0,0,0,0,0,0))


    sensor=Sensor()
    sensor.start()
    intr_params=sensor.intr_params


    datapath = "../../data/set1/"
    datastr=datetime.datetime.now().strftime('%Y_%m_%d_%H_%M_%S/')
    folderpath=datapath+datastr

    if os.path.isdir(datapath+datastr):
        print(f"{datapath+datastr} exist")
    else:
        print(f"make {datapath+datastr}")
        os.mkdir(datapath+datastr)


    '''robot workspace'''

    # X,Y,Z 로봇좌표계
    x_range=np.arange(-0.750,-0.449,step=0.1)
    y_range=np.arange(-0.150,0.180,step=0.10)
    z_range=np.arange(0.45,0.801,step=0.15)

    # roll pitch yaw , 각도 변화량
    angle_minmax=np.deg2rad(15)
    angle_step=angle_minmax/2
    z_angle_minmax=np.deg2rad(30)
    z_angle_step=z_angle_minmax/2

    x_angle_range=np.arange(-angle_step,angle_step+0.01,step=angle_step)
    y_angle_range=np.arange(-angle_step,angle_step+0.01,step=angle_step)
    z_angle_range=np.arange(-z_angle_step,z_angle_step+0.01,step=z_angle_step)
    n_pose=x_range.size*y_range.size*z_range.size*y_angle_range.size*x_angle_range.size


    init_pose=[-0.5783217063257919, 0.014545836915141067, 0.4779862526367722, 2.221439046294238, 2.2213512300962384, 5.4466157583061546e-05]
    print("set")
    acc=1
    vel=1
    robot.movel(init_pose,acc,vel)
    try:
        for temp_z in z_range:
            for temp_y in y_range:
                for temp_x in x_range:
                    for temp_angle_z in z_angle_range:
                        for temp_angle_y in y_angle_range:
                            for temp_angle_x in x_angle_range:
                                print(f"pose {temp_x} {temp_y} {temp_z} {temp_angle_x} {temp_angle_y} {temp_angle_z}")
                                rgb,depth=sensor.get_bgrd()
                                cv2.imshow("rgb",rgb)
                                key=cv2.waitKey(30)
                                time.sleep(1)
                                # 촬영 위치
                                cur_pose=copy.deepcopy(init_pose)
                                cur_pose[0]=temp_x
                                cur_pose[1]=temp_y
                                cur_pose[2]=temp_z
                                cur_pose[4] += temp_angle_x
                                cur_pose[5] += temp_angle_y
                                cur_pose[6] += temp_angle_z
                                robot.movel(cur_pose,acc,vel)

                                time.sleep(2)
                                #데이터 저장
                                cur_time_str = datetime.datetime.now().strftime('%Y_%m_%d_%H_%M_%S')
                                print(cur_time_str,end=" ")
                                cv2.imwrite(folderpath+cur_time_str+"_rgb.png",rgb)
                                cv2.imwrite(folderpath+cur_time_str+"_depth.png",depth)

                                robot_data = {
                                    "id": cur_time_str,
                                    "matrix" : robot.get_pose().array.tolist()
                                }
                                json_string = json.dumps(robot_data)
                                with open(folderpath+cur_time_str+"_UR10.json", 'w') as f:
                                    json.dump(json_string, f)
                                print("done")
    except:
        print("ERROR")
