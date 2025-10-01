import socket
import numpy as np
from pyconnect.utils import byte2dict, dict2byte, data_info, Timer, recvall
import cv2,os,json,copy
from pyinterfaces.utils import vision_mode

class TcpIpClient():
    def __init__(self, host='localhost', port=8888):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.host = host
        self.port = port
        self.server_connected = False
        self.rev_data = None
        print(f'Client connecting to {host} at PORT {port} ...')
        self.send('hello')

    def send(self, data=None):
        """ send a dict to sever"""
        while True:

            try:
                if not self.server_connected:
                    print(f'{"=" * 10} Connecting to server: {self.host}')
                    self.sock.connect((self.host, self.port))
                    print('Connected ....')
                    self.server_connected = True

                timer = Timer()
                self.send_dict(aDict=data)
                timer.pin_time('send_data')
                ret = self.get_return()
                timer.pin_time('get_return')
                print(timer.pin_times_str)
                return ret
            except Exception as e:
                print("에러 타입:", type(e).__name__)
                print("에러 메시지:", e)
                self.sock.close()
                print('socket closed ...')
                return None

    def send_dict(self, aDict):
        print(f'{"=" * 10} Sending byte data')
        byteData = dict2byte(aDict)
        len_byte = str(len(byteData)).rjust(16, '0').encode()
        self.sock.send(len_byte + byteData)
        print('Byte data sent...')

    def get_return(self):
        print(f'{"=" * 10} Waiting for return')
        ret_len = recvall(self.sock, 16)
        byteData = recvall(self.sock, int(ret_len))
        print('Return received ...')
        self.rev_data = byte2dict(byteData)
        print(data_info(self.rev_data))
        return self.rev_data
class Vision():
    crop_roi=(420, 240, 970, 550)
    client=None
    target_ind=None
    dmin=10
    dmax=120
    min_mass=500
    max_mass=10000
    topn=100
    max_ratio=1
    df=50
    nfingers=2
    def __init__(self,HOST='localhost', PORT=8801):
        from VISION.APP_Calibration.keticalibsdk.calib.calibration import calib_opencv
        self.calib = calib_opencv()
        ppx,ppy,fx,fy=np.load(__file__.split("interface")[0]+"VISION/APP_Base/configs/intr_param.npy")
        self.calib.set_intrinsic(ppx,ppy,fx,fy)
        from pyconnect.utils import parse_keys_values
        self.load_param('./VISION/APP_Base/APP/detector/setting.json')
        args = parse_keys_values(optional_args={'host': HOST, 'port': PORT})
        self.client = TcpIpClient(host=args['host'], port=args['port'])

    def get_TIS(self):
        """Get image and original data from the TIS camera.

        Returns:
            tuple:
                - success (bool): True if frames are retrieved, False otherwise.
                - img (numpy.ndarray | None): The image frame if available, None if not connected.
                - origin (numpy.ndarray | None): The original data if available, None if not connected.
        """        
        ret = self.client.send({"mode": vision_mode.MODE_TIS})
        img,origin=ret
        if type(ret[0])==type(None):
            print("TIS 카메라 연결되지 않음")
            return False,img,origin            
        else:
            return True,img,origin

    def get_rs(self) :
        '''Get RGB and depth frames from the Realsense camera.

        Returns:
            tuple:
                - success (bool): True if frames are retrieved, False otherwise.
                - rgb (numpy.ndarray | None): The RGB image in BGR format if available,
                None if not connected.
                - depth (numpy.ndarray | None): The depth image if available, None if not connected.
        '''
        ret = self.client.send({"mode": vision_mode.MODE_RS})
        rgb,depth=ret
        if type(rgb)==type(None):
            print("Realsense 연결되지 않음")
            return False,rgb,depth
        else:
            return True,rgb[:,:,::-1],depth
    def get_pick(self,rgb,depth,aDict=None):
        """Perform grasp detection and compute robot pick coordinates.

        Args:
            rgb (numpy.ndarray): The RGB image from the camera.
            depth (numpy.ndarray): The depth image corresponding to `rgb`.
            aDict (dict, optional): Custom dictionary for vision agent.
                If None, a default grasp request will be sent.

        Returns:
            tuple:
                - ret_pick (bool): True if a valid pick was found, False otherwise.
                - pick_result (list[float]): 
                    [Robot_X, Robot_Y, Robot_Z,robot_z_angle, ret_back] values.
                - out_img (numpy.ndarray): Visualization image with detection results.
        """        
        ret_pick=False
        if aDict==None:
            ret = self.client.send({"mode": vision_mode.MODE_GRASP,
                                    "rgb": rgb,"target_ind": None,
                           "no_model": False,
                           "dmin": self.dmin, "dmax": self.dmax,
                           "max_ratio":self.max_ratio,
                           "topn": self.topn,
                           "df":self.df,
                           "crop_roi": self.crop_roi,
                           "min_mass":self.min_mass,
                           "max_mass":self.max_mass})
        else:
            aDict['mode']=vision_mode.MODE_GRASP
            aDict['rgb']=rgb
            ret = self.client.send(aDict)

        if ret[0] == None:
            outImg=copy.deepcopy(ret[1])
            text = "Empty box"
            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 3
            thickness = 6
            (text_width, text_height), baseline = cv2.getTextSize(text, font, font_scale, thickness)

            # 텍스트 좌표 (중앙 정렬)
            text_x = 640 - text_width // 2
            text_y = 360 + text_height // 2
            cv2.putText(outImg, text, (text_x, text_y), font, font_scale, (0, 255, 0), thickness)
            Robot_X=100000
            Robot_Y=100000
            Robot_Z=100000
            robot_z_angle=100000
            ret_back=-1
            ret_pick=False
        else:
        
            if type(ret[0])!=type(None):
                boxes=ret[0].boxes
                wh=boxes[:,2::]-boxes[:,0:2]
                ratio=wh[:,0]/wh[:,1]
                valid_index=(ratio>0.5)&(ratio<2)
                if (valid_index==False).any():
                    print(valid_index)
                ret[0].masks=np.array(ret[0].masks)[valid_index]
                ret[0].boxes=ret[0].boxes[valid_index]
                for box in ret[0].boxes:
                   pt1=box[0:2]
                   pt2=box[2::]
                   cv2.rectangle(ret[1], pt1, pt2, color=(0, 0, 255), thickness=2)
            ret_select=self.select_best(ret[0],rgb,depth)
            depth = ret_select['depth_value']
            print(f"origin depth:{depth}")
            depth=590
            print(f"fixed depth:{depth}")
            outImg=copy.deepcopy(ret_select['out_img'])
            boxes = copy.deepcopy(ret[0].boxes)
            for box in boxes:
                line_h = int((box[1] + box[3]) / 2)
                cv2.line(outImg, (box[0], line_h), (box[2], line_h), (255, 0, 0))
            box = boxes[0]
            line_h = int((box[1] + box[3]) / 2)
            cv2.line(outImg, (box[0], line_h), (box[2], line_h), (0, 255, 0), 4)
            grip_x, grip_y = ((box[0:2] + box[2::]) / 2).astype(int)
            Robot_X,Robot_Y,Robot_Z = self.calib.convert_c2r(grip_x, grip_y, depth)
            Robot_Z=Robot_Z-0.015
            robot_z_angle=0
            ret_back=-3
            print(
                "[Robot_X, Robot_Y, Robot_Z,robot_z_angle,ret_back] = [{:.2f}, {:.2f}, {:.2f}, {:.2f},{:.2f}]".format(Robot_X, Robot_Y, Robot_Z,robot_z_angle,ret_back))
            ret_pick=True
        return ret_pick,[Robot_X, Robot_Y, Robot_Z,robot_z_angle,ret_back],outImg
    def select_best(self,ret,rgb,depth):
        """Select the best grasp candidate and compute related metrics.

        Args:
            ret: Detection result object with fields such as `centers`, `boxes`,
                `each_obj_target_ind`, and `contact_points_2d`.
            rgb (numpy.ndarray): The RGB image from the camera.
            depth (numpy.ndarray): The depth image aligned with `rgb`.

        Returns:
            dict: A dictionary containing:
                - select_ret (bool): True if a candidate was selected, False otherwise.
                - grip_x (int): Selected grasp x-coordinate in the image.
                - grip_y (int): Selected grasp y-coordinate in the image.
                - depth_value (float): Estimated depth at the grasp location.
                - robot_z_angle (float): Estimated Z rotation angle of the object.
                - out_img (numpy.ndarray): Visualization image with annotations.
        """
        grip_x, grip_y, depth_value, robot_z_angle =0,0,0,0
        if ret.centers.__len__()>0:
            select_index=ret.each_obj_target_ind[0]
            grip_x, grip_y=(ret.centers[select_index]).astype(int)

            # point
            x1, y1, x2, y2 = ret.boxes[0]
            if x1 < x2:
                left_x = x1
                left_y = y1
                right_x = x2
                right_y = y2
            else:
                left_x = x2
                left_y = y2
                right_x = x1
                right_y = y1
            out = copy.deepcopy(rgb)
            cv2.circle(out, (left_x, left_y), 4, (255, 0, 0), -1)
            cv2.circle(out, (right_x, right_y), 4, (255, 0, 255), -1)
            padsize = 5
            if left_y < right_y:
                min_y = left_y
                max_y = right_y
            else:
                min_y = right_y
                max_y = left_y

            depth_candi = copy.deepcopy(
                depth[(min_y - padsize):(max_y + padsize), (left_x - padsize):(right_x + padsize)])
            depth_value = np.min(depth_candi[(depth_candi < 650) & (depth_candi > 500)])
            vector = np.array([right_x - left_x, right_y - left_y])
            robot_z_angle = np.rad2deg(np.arccos(np.dot(vector, [1, 0]) / np.linalg.norm(vector)))
            select_ret=True
        else:
            select_ret = False
        ret={"select_ret":select_ret,
            "grip_x":grip_x,
            "grip_y":grip_y,
            "depth_value":depth_value,
            "robot_z_angle":robot_z_angle,
            "out_img":out}
        return ret
    def get_angle(self,rgb,aDict=None):
        """Estimate object orientation angle from an RGB image.

        Args:
            rgb (numpy.ndarray): The RGB image input.
            aDict (dict, optional): Additional request parameters. If None,
                a default request with mode=MODE_ANGLE will be sent.

        Returns:
            dict: A dictionary with angle estimation results, including:
                - ret (bool): True if the estimation succeeded.
                - reference_r (float): Reference rotation angle.
                - differ_angle (float): Difference angle relative to reference.
                - X (float): X-coordinate of the detected reference point.
                - Y (float): Y-coordinate of the detected reference point.
        """        
        if aDict==None:
            ret = self.client.send({"mode": vision_mode.MODE_ANGLE,
                                    "rgb": rgb})
        else:
            aDict['mode']=vision_mode.MODE_ANGLE
            aDict['rgb']=rgb
            if "blur_size" in aDict.keys():
                blur_size=aDict["blur_size"]
                if blur_size%2==0:
                    print(f"홀수가 아닙니다. blur_size :{blur_size} ")
                    return False,None,rgb
            if "single_med_filtersize" in aDict.keys():
                single_med_filtersize=aDict["single_med_filtersize"]
                if single_med_filtersize%2==0:
                    print(f"홀수가 아닙니다. single_med_filtersize : {single_med_filtersize}")
                    return False,None,rgb
            ret = self.client.send(aDict)
            if type(ret)==type(None):
                return False,None,rgb
            result,ret_img=ret
            if result['ret']:
                del result['ret']
                del result['ret_img']
                return True,result,ret_img
        return True,result,ret_img
    
    def load_param(self,config_path):
        if os.path.isfile(config_path):
            with open(config_path, "r", encoding="utf-8") as f:
                data = json.load(f)
            self.crop_roi = data['crop_roi']
            self.target_ind = data['target_ind']
            self.dmin = data['dmin']
            self.dmax = data['dmax']
            self.topn = data['topn']
            self.max_ratio = data['max_ratio']
            self.df = data['df']


