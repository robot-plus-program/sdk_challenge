# ******************************************************************************
#  Copyright (c) 2023 Orbbec 3D Technology, Inc
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http:# www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.
# ******************************************************************************
import argparse
import sys

import cv2
import numpy as np
from pygame import NOEVENT
from pygame.examples.midi import print_device_info

from pyorbbecsdk import Pipeline, Config, OBSensorType, OBAlignMode, VideoStreamProfile, OBFormat
# from pyorbbecsdk import *
from ketisdk.sensor.femto_utils import frame_to_bgr_image
from ketisdk.sensor.sensor import Sensor as MySensor
from ketisdk.vision.utils.rgbd_utils_v2 import RGBD

ESC_KEY = 27


class FemtoSensor(MySensor):
    def start(self, device_serial=None, size=(1280, 720), **kwargs):
        self.pipeline = Pipeline()
        device = self.pipeline.get_device()
        device_info = device.get_device_info()
        device_pid = device_info.get_pid()
        config = Config()
        align_mode = 'HW'
        enable_sync = True

        try:
            profile_list = self.pipeline.get_stream_profile_list(OBSensorType.COLOR_SENSOR)
            color_profile = profile_list.get_default_video_stream_profile()
            config.enable_stream(color_profile)
            profile_list = self.pipeline.get_stream_profile_list(OBSensorType.DEPTH_SENSOR)
            assert profile_list is not None
            depth_profile = profile_list.get_default_video_stream_profile()
            assert depth_profile is not None
            print("color profile : {}x{}@{}_{}".format(color_profile.get_width(),
                                                       color_profile.get_height(),
                                                       color_profile.get_fps(),
                                                       color_profile.get_format()))
            print("depth profile : {}x{}@{}_{}".format(depth_profile.get_width(),
                                                       depth_profile.get_height(),
                                                       depth_profile.get_fps(),
                                                       depth_profile.get_format()))
            config.enable_stream(depth_profile)
        except Exception as e:
            print(e)
            return
        if align_mode == 'HW':
            if device_pid == 0x066B:
                # Femto Mega does not support hardware D2C, and it is changed to software D2C
                config.set_align_mode(OBAlignMode.SW_MODE)
            else:
                config.set_align_mode(OBAlignMode.HW_MODE)
        elif align_mode == 'SW':
            config.set_align_mode(OBAlignMode.SW_MODE)
        else:
            config.set_align_mode(OBAlignMode.DISABLE)
        if enable_sync:
            try:
                self.pipeline.enable_frame_sync()
            except Exception as e:
                print(e)
        try:
            self.pipeline.start(config)
        except Exception as e:
            print(e)
            return
        
        for _ in range(100):
            if self.get_data() is not None:
                break
        print('Femto initialized ... ')
    def stop(self):
        self.pipeline.stop()
        print('Femto terminated ... ')


    def get_data(self):
        while True:
            frames: FrameSet = self.pipeline.wait_for_frames(100)
            if frames is None:
                continue
            color_frame = frames.get_color_frame()
            if color_frame is None:
                continue
            # covert to RGB format
            color_image = frame_to_bgr_image(color_frame)[...,::-1]
            if color_image is None:
                print("failed to convert frame to image")
                continue
            depth_frame = frames.get_depth_frame()
            if depth_frame is None:
                continue
            width = depth_frame.get_width()
            height = depth_frame.get_height()
            scale = depth_frame.get_depth_scale()

            depth_data = np.frombuffer(depth_frame.get_data(), dtype=np.uint16)
            depth_data = depth_data.reshape((height, width))

            return color_image, depth_data
    def get_rgbd(self, workspace=None, depth_min=300, depth_max=1200, rot_90=False, denoise_ksize=None):
        data = self.get_data()
        if data is None:
            return None
        rgb, depth = data
        rgbd = RGBD(rgb=rgb, depth=depth, workspace=workspace, depth_min=depth_min,
                    depth_max=depth_max, denoise_ksize=denoise_ksize)
        return rgbd

    def run(self):
        self.start()
        while True:
            rgbd = self.get_rgbd()

            if rgbd is None:
                continue
            cv2.imshow('viewer', rgbd.disp(mode='depth')[...,::-1])
            if cv2.waitKey(30)==27:
                intr = self.pipeline.get_camera_param().depth_intrinsic
                cam_info = [intr.fx, intr.fy, intr.cx, intr.cy, 1000, intr.width, intr.height]
                rgbd.show_3D(camera_info=cam_info)
                self.stop()
                return


    

if __name__ == "__main__":
    # print("Please NOTE: This example is NOT supported by the Gemini 330 series.")
    # print("If you want to see the example on Gemini 330 series, please refer to align_filter_viewer.py")
    # main(sys.argv[1:])
    femto = FemtoSensor()
    femto.run()
