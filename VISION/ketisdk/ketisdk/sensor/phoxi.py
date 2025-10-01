import os, sys
sys.path.append(os.getcwd())

from pyphoxi import PhoXiSensor
import numpy as np
from ketisdk.gui.gui import GuiModule, GUI
from ..sensor.sensor import Sensor
from ketisdk.vision.utils.rgbd_utils_v2 import RGBD
import time
import cv2

class PhotoNeo(Sensor):
    def start(self, device_serial=None, size=(1280, 720), addr='127.0.0.1', port=50200):
        # PhoXiControl
        # ./ketisdk/sensor/pyphoxi 2019-12-054-LC3 50200 low
        self.cam = PhoXiSensor(addr, port)
        self.cam.start()
        time.sleep(1)
        print("[!] Successfully connected to camera.")

        print(f"[*] Intrinsics: {self.cam.intrinsics}")
        print(f"[*] Distortion: {self.cam.distortion}")

    def stop(self):
        self.cam.stop()
        print('sensor terminated ... ')

    def get_data(self):
        frame_id, gray, depth = self.cam.get_frame(True)
        return cv2.cvtColor(gray, cv2.COLOR_GRAY2RGB), (1000*depth).astype('uint16')

    def get_rgbd(self, workspace=None, depth_min=300, depth_max=1200, rot_90=False):
        rgb, depth = self.get_data()
        if rot_90: rgb, depth = np.rot90(rgb), np.rot90(depth)
        if rgb is None and depth is None: return None
        rgbd =  RGBD(rgb=rgb, depth=depth,workspace=workspace,depth_min=depth_min, depth_max=depth_max)
        return rgbd


def get_phoxi_module():
    return GuiModule(PhotoNeo, type='phoxi', name='PhotoNeo', short_name='PX',
                     category='vision_sensor', serial=None)


if __name__ == '__main__':
    pass
