import numpy as np
class Sensor():
    def __init__(self, nbuf=1, **kwargs):
        self.info = CameraInfo(1280, 720, fx=900,  fy=900,  cx=640, cy=360, scale=np.array([[1000.]]))
        self.nbuf = nbuf
        self.rgb_buf = [[]]*nbuf
        self.depth_buf = [[]]*nbuf
        
    def start(self, device_serial=None, size=(1280, 720), **kwargs):
        pass
    
    def stop(self):
        pass

    def get_data(self):
        pass

    def get_rgbd(self, workspace=None, depth_min=300, depth_max=1200, denoise_ksize=None):
        pass
    
    def get_info(self):
        return self.info

class CameraInfo():
    """ Camera intrisics for point cloud creation. """
    def __init__(self, width, height, fx, fy, cx, cy, scale):
        self.width = width
        self.height = height
        self.fx = fx
        self.fy = fy
        self.cx = cx
        self.cy = cy
        self.scale = scale


if __name__ == '__main__':
    cfg_path = 'configs/DDI/grip_kinect_azure_hcr.cfg'
    realsense = Sensor(cfg_path).run()
