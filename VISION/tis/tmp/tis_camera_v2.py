import sys
sys.path.append("./camera/tis")

import TIS
import cv2

import sys
sys.path.append('/usr/lib/python3/dist-packages/')
import gi

gi.require_version("Tcam", "1.0")
gi.require_version("Gst", "1.0")

from gi.repository import Gst


class TISSensor():
    def __init__(self):
        self.Tis = []

    def __del__(self):
        self.Tis.Stop_pipeline()

    def get_device_sn(self):
        pass
        ''' 
        realsense_ctx = rs.context()
        for i in range(len(realsense_ctx.devices)):
            detected_camera = realsense_ctx.devices[i].get_info(rs.camera_info.serial_number)

            print(detected_camera)
        '''

    def start(self, device_serial=None, size=(1920, 1080), fps=15):
        self.color_size = size
        self.fps = fps

        self.Tis = TIS.TIS()

        '''
        if self.Tis.openDevice(serial=28020099, width=size[0], height=size[1], framerate='2/1', sinkformat=TIS.SinkFormats.BGRA, showvideo=False):
            print('Error - TIS Camera init')
        '''
        #if self.Tis.openDevice(serial=23810422, width=size[0], height=size[1], framerate='2/1',
        #                       sinkformat=TIS.SinkFormats.BGRA, showvideo=False):
        if self.Tis.openDevice(serial=None, width=size[0], height=size[1], framerate='2/1',
                               sinkformat=TIS.SinkFormats.BGRA, showvideo=False):
            print('Error - TIS Camera init')

        #self.Tis.Set_Property('Focus Auto', False)
        self.Tis.Set_Property('Whitebalance Auto', True)
        self.Tis.Set_Property('Exposure Auto', False)
        self.Tis.Set_Property('Exposure', int(20000))
        self.Tis.Set_Property("Gain Auto", False)
        self.Tis.Set_Property("Gain", int(300))

        focus = 580

        self.Tis.Set_Property("Focus Auto", False)
        self.Tis.Set_Property("Focus", focus)

        print('Exposure', self.Tis.Get_Property('Exposure'))
        print('Gain', self.Tis.Get_Property('Gain'))

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

def callback(appsink, user_data):
    """
    This function will be called in a separate thread when our appsink
    says there is data for us. user_data has to be defined
    when calling g_signal_connect. It can be used to pass objects etc.
    from your other function to the callback.
    """
    sample = appsink.emit("pull-sample")

    if sample:

        caps = sample.get_caps()

        gst_buffer = sample.get_buffer()

        try:
            (ret, buffer_map) = gst_buffer.map(Gst.MapFlags.READ)
        finally:
            gst_buffer.unmap(buffer_map)

    return Gst.FlowReturn.OK

if __name__ == '__main__':
    Gst.init(sys.argv)
    monitor = Gst.DeviceMonitor.new()
    monitor.add_filter("Video/Source/tcam")

    for device in monitor.get_devices():
        struc = device.get_properties()
        print("\tmodel:\t{}\tserial:\t{}\ttype:\t{}".format(struc.get_string("model"),
                                                            struc.get_string("serial"),
                                                            struc.get_string("type")))

    serial = None
    source = Gst.ElementFactory.make("tcambin")

    source.set_state(Gst.State.READY)

    caps = source.get_static_pad("src").query_caps()

    pipeline = Gst.parse_launch("tcambin name=source"
                                " ! videoconvert"
                                " ! appsink name=sink")

    sink = pipeline.get_by_name("sink")

    # tell appsink to notify us when it receives an image
    sink.set_property("emit-signals", True)

    user_data = "This is our user data"

    # tell appsink what function to call when it notifies us
    sink.connect("new-sample", callback, user_data)

    tmp = 0

    '''
    tisCam = TISSensor()
    tisCam.start(size=(1920, 1080), fps=15)

    key = []

    while True:
        rgb, depth = tisCam.get_data()

        if rgb != []:
            cv2.imshow("tiscam", rgb)
            key = cv2.waitKey(10)

        if key == ord('q') or key == ord('Q'):
            break
    '''







    #GUI(modules=get_realsense_modules())
