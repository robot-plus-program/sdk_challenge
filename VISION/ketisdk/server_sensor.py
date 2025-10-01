from ketisdk.sensor.sensor import Sensor
from ketisdk.base.udp import UDPServerThread
# from ketisdk.base.base_tcp import ServerThread
from ketisdk.vision.utils.rgbd_utils_v2 import RGBD
from threading import Thread


class ServerSensor(Sensor):
    def __init__(self, nbuf=1, host='localhost', port=8888):
        Sensor.__init__(self, nbuf=nbuf)
        self.server = UDPServerThread(host=host, port=port)

    def start(self, **kwargs):
        self.thread = Thread(target=self.server.listen, daemon=True)
        self.thread.start()

    def stop(self, **kwargs):
        self.server.terminate()

    def get_data(self, **kwargs):
        return (self.server.data['rgb'] if 'rgb' in self.server.data else None,
                self.server.data['depth'] if 'depth' in self.server.data else None)

    def get_rgbd(self, workspace=None, depth_min=300, depth_max=1200, denoise_ksize=None):
        rgb, depth = self.get_data()
        return RGBD(rgb=rgb, depth=depth, workspace=workspace, depth_min=depth_min,
                    depth_max=depth_max, denoise_ksize=denoise_ksize)

    def run(self):
        import cv2
        import numpy as np

        self.start()

        viewer_name = 'udp_viewer'
        cv2.namedWindow(viewer_name, cv2.WINDOW_NORMAL)
        w, h = 1080, 720
        cv2.resizeWindow(viewer_name, width=w, height=h)
        fake_im = 127*np.ones((h,w,3), 'uint8')
        cv2.putText(fake_im, 'No server data', (0, 50), cv2.FONT_HERSHEY_COMPLEX, 2, (0,255,0), 1)
        while True:
            try:
                rgbd = self.get_rgbd()
                rgbd_disp = rgbd.disp()
            except:
                rgbd_disp = fake_im

            cv2.imshow(viewer_name, rgbd_disp)
            if cv2.waitKey(10) == 27:
                break
        self.stop()


if __name__=='__main__':
    sensor = ServerSensor()
    sensor.run()







