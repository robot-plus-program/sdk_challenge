"""
This example shows how to read point cloud data from a ZDF file, convert it to OpenCV format, then extract and
visualize depth map.
The ZDF files for this sample can be found under the main instructions for Zivid samples.
"""

from pathlib import Path
import numpy as np
import cv2
import zivid
from ketisdk.vision.utils.rgbd_utils_v2 import RGBD
from ketisdk.sensor.sensor import Sensor
from datetime import timedelta


class ZividSensor(Sensor):
    def start(self, device_serial=None, size=(1280, 720)):
        self.app = zivid.Application()
        self.camera = self.app.connect_camera()
        self.settings = zivid.Settings(acquisitions=[zivid.Settings.Acquisition()])

        self.settings_2d = zivid.Settings2D()
        self.settings_2d.acquisitions.append(
            zivid.Settings2D.Acquisition(
                aperture=3.0,
                exposure_time=timedelta(microseconds=10000),
                brightness=1.8,
                gain=2.0,
            )
        )
        self.settings.color = self.settings_2d


    def stop(self):
        self.camera.disconnect()
        print('Zivid Camera disconnected ...')
        self.camera.release()
        print('Zivid Camera released ...')
        self.app.release()
        print('Zivid App released ...')

    def get_data(self):
        frame = self.camera.capture(self.settings)
        # frame.save("result.zdf")
        point_cloud = frame.point_cloud()

        return point_cloud.copy_data('rgba')[:, :, :3], point_cloud.copy_data('z').astype('uint16')

    def get_rgbd(self, workspace=None, depth_min=300, depth_max=1200, rot_90=False, denoise_ksize=None):
        rgb, depth = self.get_data()
        if rgb is None and depth is None: return None
        rgbd =  RGBD(rgb=rgb, depth=depth,workspace=workspace,depth_min=depth_min, depth_max=depth_max, denoise_ksize=denoise_ksize)
        return rgbd

def demo_run_zivid():
    cam = ZividSensor()
    cam.start()

    rgbd = cam.get_rgbd(depth_min=500, depth_max=900)
    rgbd.show(mode='rgb')
    rgbd.show(mode='depth_jet')
    cv2.waitKey()

    cam.stop()

def get_zivid_module():
    from ketisdk.gui.gui import GuiModule
    return GuiModule(ZividSensor, type='zivid', name='Zivid', short_name='ZV', category='vision_sensor', serial=None)

def demo_run_zivid_gui():
    from ketisdk.gui.gui import GUI
    GUI(title='Zivid', modules=[get_zivid_module(), ])



if __name__ == "__main__":
    # demo_run_zivid()
    demo_run_zivid_gui()
