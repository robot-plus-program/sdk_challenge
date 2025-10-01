import socket, numpy as np
from pyconnect.utils import byte2dict, dict2byte, data_info, Timer, recvall
import cv2,os,json,copy


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

class detector():
    crop_roi=(420, 240, 970, 550)
    client=None
    target_ind=None
    dmin=10
    dmax=120
    min_mass=100
    max_mass=50000
    topn=100
    max_ratio=1
    df=50
    nfingers=2
    def __init__(self,config_path="settings.json"):
        self.config_path=config_path
        self.load_param()
        from pyconnect.utils import parse_keys_values
        HOST, PORT = 'localhost', 8801
        args = parse_keys_values(optional_args={'host': HOST, 'port': PORT})
        self.client = TcpIpClient(host=args['host'], port=args['port'])
    def load_param(self):
        if os.path.isfile(self.config_path):
            with open(self.config_path, "r", encoding="utf-8") as f:
                data = json.load(f)
            self.crop_roi = data['crop_roi']
            self.target_ind = data['target_ind']
            self.dmin = data['dmin']
            self.dmax = data['dmax']
            self.topn = data['topn']
            self.max_ratio = data['max_ratio']
            self.df = data['df']
    def load_param_repose(self):
        repose_config_path='../detector/setting_pose.json'
        if os.path.isfile(repose_config_path):
            with open(repose_config_path, "r", encoding="utf-8") as f:
                data = json.load(f)
            self.crop_roi = data['crop_roi']
            self.target_ind = data['target_ind']
            self.dmin = data['dmin']
            self.dmax = data['dmax']
            self.topn = data['topn']
            self.max_ratio = data['max_ratio']
            self.df = data['df']
    def set_roi(self,crop_roi=(420, 240, 970, 550)):
        self.crop_roi=crop_roi

    def run(self,rgb,depth,aDict=None):
        if aDict==None:
            ret = self.client.send({"rgb": rgb, "depth": depth, "target_ind": None,
                           "no_model": False,
                           "dmin": self.dmin, "dmax": self.dmax,
                           "max_ratio":self.max_ratio,
                           "topn": self.topn,
                           "df":self.df,
                           "crop_roi": self.crop_roi})
        else:
            ret = self.client.send(aDict)
        return ret
    def select_best(self,ret,rgb,depth):

        grip_x, grip_y, depth_value, robot_z_angle =0,0,0,0
        if ret.centers.__len__()>0:
            select_index=ret.each_obj_target_ind[0]
            grip_x, grip_y=ret.centers[0]#(ret.centers[select_index]).astype(int)
            points=ret.contact_points_2d

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


