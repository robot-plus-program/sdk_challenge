import sys
sys.path.append("/home/keti/Desktop/PoC_Talos/APP_Talos/tis/python-common")

import gi

gi.require_version("Tcam", "1.0")
gi.require_version("Gst", "1.0")
gi.require_version("GLib", "2.0")

from gi.repository import Tcam, Gst, GLib
import TIS

import cv2
import sys


class TISSensor():
    def __init__(self):
        self.Tis = []

    def __del__(self):
        self.Tis.Stop_pipeline()

    def get_device_sn(self):
        pass

    def start(self, device_serial=None, out_size=(1920, 1080), fps=30):
        self.color_size = out_size
        self.fps = fps
        #
        self.Tis = TIS.TIS()

        if self.Tis.openDevice(serial=None, width=out_size[0], height=out_size[1], framerate='30/1',
                               sinkformat=TIS.SinkFormats.BGRA, showvideo=False):
            print('Error - TIS Camera init')

        camera = Gst.ElementFactory.make("tcambin")
        # in the READY state the camera will always be initialized
        camera.set_state(Gst.State.READY)

        def print_properties(camera):
            """
            Print selected properties
            """
            try:

                property_exposure_auto = camera.get_tcam_property("ExposureAuto")

                print(property_exposure_auto.get_value())

                value = camera.get_tcam_enumeration("ExposureAuto")

                print(f"Exposure Auto has value: {value}")

                value = camera.get_tcam_enumeration("GainAuto")

                print("Gain Auto has value: {}".format(value))

                value = camera.get_tcam_float("ExposureTime")

                print("ExposureTimer has value: {}".format(value))

            except GLib.Error as err:

                print(f"{err.message}")

        print_properties(camera)

        #camera.set_tcam_enumeration('WhitebalanceAuto', "True")
        #camera.set_tcam_enumeration('Exposure Auto', False)
        #
        #camera.set_tcam_enumeration("ExposureAuto", "Off")
        #camera.set_tcam_integer("ExposureTime", 20000)
        #
        #camera.set_tcam_enumeration("GainAuto", "Off")
        #camera.set_tcam_float("Gain", 300)
        #
        focus = 580
        camera.set_tcam_enumeration("FocusAuto", "Off")
        camera.set_tcam_integer("Focus", 580)


        '''
        self.Tis.set_tcam_enumeration("ExposureAuto", "Off")
        self.Tis.Set_Property('Whitebalance Auto', True)
        self.Tis.Set_Property('Exposure Auto', False)
        self.Tis.Set_Property('Exposure', int(20000))
        self.Tis.Set_Property("Gain Auto", False)
        self.Tis.Set_Property("Gain", int(300))
        

        focus = 580

        self.Tis.Set_Property("Focus Auto", False)
        self.Tis.Set_Property("Focus", focus)
        '''

        #print('Exposure', camera.get_tcam_integer("Exposure"))
        #print('Gain', camera.get_tcam_float("Gain"))
        print('Focus', camera.get_tcam_integer("Focus"))

        max_x = 5400
        max_y = 7680

        # left_x = (max_x - self.color_size[0]) / 2
        # up_y = (max_y - self.color_size[1]) / 2

        left_x = (max_x - self.color_size[1]) / 2
        up_y = (max_y - self.color_size[0]) / 2

        self.Tis.Set_Property("Offset X", int(left_x + 300))
        self.Tis.Set_Property("Offset Y", int(up_y))
        print('OffsetXY', int(left_x + 300), int(up_y))
        e = self.Tis.Start_pipeline()
        # cv2.waitKey(1000)
        self.state_Camera = True

        ''' 
        if device_serial is not None:
            self.start_(device_serial=device_serial)
        else:
            realsense_ctx = rs.context()
            for i in range(len(realsense_ctx.devices)):
                detected_serial = realsense_ctx.devices[i].get_info(rs.camera_info.serial_number)
                try:
                    self.start_(device_serial=detected_serial, size=size)
                    break
                except:
                    pass
        '''

    def stop(self):

        pass
        '''
        self.pipeline.stop()
        print('sensor terminated ... ')
        '''

    def get_data(self):
        rgb= []
        depth = []

        while True:
            f = self.Tis.Snap_image(1.0)

            if f == True:
                break
            else:
                print("tis cam: Snap-False")

        '''
        if f == False:
            print('tis_error')
            self.false_count += 1
            del self.Tis
            time.sleep(1)
            self.Camera_Init()
            return [], []
        '''

        self.color_image = self.Tis.Get_image()

        #color_image_ = []

        if self.color_image != []:
            color_image_ = cv2.resize(self.color_image, dsize=(1280, 720))
            rgb = cv2.cvtColor(color_image_, cv2.COLOR_BGRA2BGR)

        return rgb, depth


        '''
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)

        color_frame = aligned_frames.get_color_frame()  # bgr
        rgb = np.asanyarray(color_frame.get_data())[:, :, ::-1]

        depth_frame = aligned_frames.get_depth_frame()
        depth = np.asanyarray(depth_frame.get_data())

        return rgb, depth
        '''

    def get_rgbd(self, workspace=None, depth_min=300, depth_max=1200, rot_90=False):
        pass
        '''
        rgb, depth = self.get_data()
        if rot_90: rgb, depth = np.rot90(rgb), np.rot90(depth)
        if rgb is None and depth is None: return None
        rgbd = RGBD(rgb=rgb, depth=depth, workspace=workspace, depth_min=depth_min, depth_max=depth_max)
        return rgbd
        '''

    def get_intrinsic_params(self, profile):
        pass
        '''
        return profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        '''

    def setExposure(self, Exposure):
        self.Tis.Set_Property('Exposure', Exposure)

    # print('SetExp')

    def setGain(self, Gain):
        self.Tis.Set_Property('Gain', Gain)

    # print('setGain')

    def setFocus(self, Focus):
        self.Tis.Set_Property("Focus", Focus)

    def setOffsetX(self, offx):
        self.Tis.Set_Property("Offset X", offx)

    def setOffsetY(self, offy):
        self.Tis.Set_Property("Offset Y", offy)

    def getExposure(self):
        return self.Tis.Get_Property('Exposure').value

    # print('SetExp')

    def getGain(self):
        return self.Tis.Get_Property('Gain').value

    # print('setGain')

    def getFocus(self):
        return self.Tis.Get_Property('Focus').value

    def getOffsetX(self):
        return self.Tis.Get_Property('Offset X').value

    def getOffsetY(self):
        return self.Tis.Get_Property('Offset Y').value

if __name__ == '__main__':
    # cfg_path = 'configs/sensor/realsense.cfg'
    # # cfg_path = 'configs/grasp_detection/grasp_v2.cfg'
    # RSSensor(cfg_path=cfg_path).run()
    #cfg_path = 'configs/sensor/realsense.cfg'

    tisCam = TISSensor()
    tisCam.start(out_size=(1920, 1080), fps=30)

    key = []

    while True:
        rgb, depth = tisCam.get_data()

        if rgb != []:
            cv2.imshow("tiscam", rgb)
            key = cv2.waitKey(10)

        if key == ord('q') or key == ord('Q'):
            break








    #GUI(modules=get_realsense_modules())
