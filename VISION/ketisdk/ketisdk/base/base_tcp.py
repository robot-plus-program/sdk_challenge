import socket, threading
import numpy as np
import time
# from ketisdk.import_basic_utils import *
from ketisdk.utils.proc_utils import ProcUtils
from ketisdk.base.base import BasObj

from ketisdk.base.utils import show_data, ins2byte, byte2ins, recvall




def send_result(s, inf):
    s.send(inf.encode())


def byte2dict(byteData):
    byte_dict = eval(byteData.decode())
    data_dict = dict()
    for key in byte_dict:
        item = byte_dict[key]
        data = item['data']
        if item['is_array']: data = np.frombuffer(data, dtype=item['dtype']).reshape(item['shape'])
        data_dict.update({key: data})
    return data_dict


def dict2byte(data_dict):
    out_dict = dict()
    for key in data_dict:
        value = data_dict[key]
        is_array = isinstance(value, np.ndarray)
        shape, dtype = None, None
        if is_array:
            shape, dtype = value.shape, value.dtype.name
            value = value.tostring()
        out_dict.update({key: {'is_array': is_array, 'shape': shape, 'dtype': dtype, 'data': value}})
    return str(out_dict).encode()


def ndarray2byteDict(mat):
    return {'size': mat.shape, 'dtype': mat.dtype.name, 'bytes': mat.tostring()}


def byteDict2mat(byteDict):
    return np.frombuffer(byteDict['bytes'], dtype=byteDict['dtype']).reshape(byteDict['size'])


class ServerThread():
    def __init__(self, host='localhost', port=8888):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((host, port))
        print(f'Server {host} at PORT {port} running ....')

    def listen(self):
        print('{} listening a connection'.format('+' * 10))
        self.sock.listen(1)
        while True:
            try:
                self.client, self.address = self.sock.accept()
                self.client.settimeout(1200)  # time out after 20 mins
                self.thread = threading.Thread(target=self.listenToClient, daemon=True)
                self.thread.start()
            except:
                print('sever terminated')
                break

    def listenToClient(self):
        while True:
            try:
                print('{} waiting data'.format('+' * 10))
                self.length = recvall(self.client, 16)

                if self.length is None:
                    raise ('Client disconnected')

                else:
                    print('{} Connected to {}'.format('+' * 10, self.address))

                    byteData = self.receive_data()
                    self.data = byte2ins(byteData=byteData)
                    print('{}  Data received'.format('+' * 10))

                    rets = self.process_received_data()
                    print('{}  Data processed'.format('+' * 10))

                    self.send_return(rets)
                    print('{}  Return sent'.format('+' * 10))
                    time.sleep(2)
                    # ProcUtils().clscr()
            except:
                self.client.close()
                return False

    def receive_data(self):
        byteData = recvall(self.client, int(self.length))
        return byteData
        # return self.get_data(byteData=byteData)

    def process_received_data(self):
        return self.data

    def send_return(self, rets):
        byteRets = ins2byte(ins=rets)
        byteLen = str(len(byteRets)).rjust(16, '0').encode()
        self.client.sendall(byteLen + byteRets)

    def terminate(self):
        self.server_run = False
        self.sock.shutdown(socket.SHUT_RDWR)
        self.sock.close()


class ClientThread():
    def __init__(self, host='localhost', port=8888):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.host = host
        self.port = port
        self.server_connected = False
        print(f'Client connecting to {host} at PORT {port} ...')

    def send_and_get_return(self, aDict):
        """ send a dict to sever"""
        while True:
            try:
                if not self.server_connected:
                    print('{} Connecting to server: {}'.format('+' * 10, self.host))
                    self.sock.connect((self.host, self.port))
                    print('Connected ....')
                    self.server_connected = True

                self.send_dict(aDict=aDict)
                return self.get_return()
            except:
                self.sock.close()
                print('socket closed ...')
                return None

    def send_dict(self, aDict):
        print('{} Sending byte data'.format('+' * 10))
        byteData = ins2byte(ins=aDict)

        len_byte = str(len(byteData)).rjust(16, '0').encode()
        self.sock.send(len_byte + byteData)
        print('Byte data sent...')

    def get_return(self):
        print('{} Waiting for return'.format('+' * 10))
        ret_len = recvall(self.sock, 16)
        byteData = recvall(self.sock, int(ret_len))
        print('Return received ...')
        return byte2ins(byteData=byteData)


def ndarray2bytes(mat):
    info = mat.dtype.name
    for s in mat.shape:
        info += '_' + str(s)
    info = info.ljust(32, '$')
    return info.encode() + mat.tostring()


def bytes2ndarray(byteData):
    info = byteData[:32].decode()
    info = info[:info.find('$')]
    info = tuple(el for el in info.split('_'))
    shape = tuple(int(el) for el in info[1:])
    data_type = info[0]
    return np.frombuffer(byteData[32:], dtype=data_type).reshape(shape)


def get_server_args():
    from ketisdk.utils.proc_utils import CFG
    cfg = CFG()
    cfg.server = CFG()
    cfg.server.host = 'localhost'
    cfg.server.port = 8888

    cfg.module = CFG()
    cfg.module.cfg_path = None
    cfg.module.inputs = None
    cfg.module.runner = 'run'
    cfg.module.runner_inputs = None
    cfg.module.visualizer = None
    cfg.module.visualizer_inputs = None
    cfg.module.show_steps = False

    return cfg


class ZeromqServer():
    def __init__(self, host='*', port=8888):
        import zmq
        context = zmq.Context()
        self.sock = context.socket(zmq.REP)
        self.sock.bind(f'tcp://*:{port}')

        print(f'Server {host} at PORT {port} running ....')

    def listen(self):
        print('{} listening a connection'.format('+' * 10))
        while True:
            #  Wait for next request from client
            byteData = self.sock.recv()
            self.data = byte2ins(byteData=byteData)
            print('{}  Data received'.format('+' * 10))

            rets = self.process_received_data()
            print('{}  Data processed'.format('+' * 10))

            self.sock.send(ins2byte(rets))
            print('{}  Return sent'.format('+' * 10))

    def process_received_data(self):
        return self.data


class ZeromqClient():
    def __init__(self, host='localhost', port=8888):
        import zmq
        context = zmq.Context()
        #  Socket to talk to server
        self.sock = context.socket(zmq.REQ)
        try:
            self.sock.connect(f'tcp://{host}:{port}')
            print(f'Connected to {host} at PORT {port}...')
        except:
            self.sock.close()
            print(f'Cannot connect to {host} at PORT {port}. Socket closed...')

    def send_and_get_return(self, aDict):
        byteData = ins2byte(ins=aDict)
        print('data to byte converted...')
        self.sock.send(byteData)
        print('byte sent...')
        recvbyte = self.sock.recv()
        print('byte received...')
        data = byte2ins(recvbyte)
        print('received data to data...')
        return data


def demo_zeromq():
    from threading import Thread

    server = ZeromqServer()
    Thread(target=server.listen, daemon=True).start()
    client = ZeromqClient()
    for i in range(10):
        data = {1: i, 2: np.ones((3, 3))}
        ret = client.send_and_get_return(data)
        print(ret)


def make_server(cfg=get_server_args(), module=None):
    assert module is not None and cfg.module.runner is not None

    import os, datetime, cv2
    from ketisdk.vision.utils.rgbd_utils_v2 import RGBD
    from ketisdk.utils.proc_utils import WorkSpace

    crop_area, runner, visualizer = None, None, None
    try:
        module = module() if cfg.module.cfg_path is None else module(cfg_path=cfg.module.cfg_path)
    except:
        print('module init failed or previously initialized')
    runner = eval(f'module.{cfg.module.runner}')
    visualizer = eval(f'module.{cfg.module.visualizer}') if cfg.module.visualizer else None

    tcp_data_dir = 'data/tcp_data'
    os.makedirs(tcp_data_dir, exist_ok=True)

    input_names = cfg.module.inputs.replace(' ', '').split(',') if cfg.module.inputs else []
    if cfg.module.runner_inputs == 'all':
        runner_inputs = 'all'
    else:
        runner_inputs = [eq.split('=') for eq in
                         cfg.module.runner_inputs.replace(' ', '').split(',')] if cfg.module.runner_inputs else []
        runner_inputs = {k: v for k, v in runner_inputs}
    visualizer_inputs = [eq.split('=') for eq in
                         cfg.module.visualizer_inputs.replace(' ', '').split(
                             ',')] if cfg.module.visualizer_inputs else []
    visualizer_inputs = {k: v for k, v in visualizer_inputs}

    class ModuleServer(ServerThread):
        def process_received_data(self):
            if self.data is None:
                return None
            if 'rgbd' in runner_inputs:
                module.args.flag.show_steps = cfg.module.show_steps
                self.data['rgbd'] = RGBD(
                    rgb=self.data['rgb'] if 'rgb' in input_names else None,
                    depth=self.data['depth'] if 'depth' in input_names else None,
                    depth_min=module.args.sensor.depth_min,
                    depth_max=module.args.sensor.depth_max)
                self.data['rgbd'].set_workspace(pts=module.args.sensor.crop_poly)
                self.data['rgbd_disp'] = self.data['rgbd'].disp()

            if runner_inputs == 'all':
                runner_inputs_ = self.data
            else:
                runner_inputs_ = {k: self.data[v] if v in self.data else eval(v) for k, v in runner_inputs.items()}
            for k in runner_inputs_:
                assert isinstance(k, str)

            ret = runner(**runner_inputs_)
            if visualizer is not None:
                visualizer_inputs_ = {}
                for k, v in visualizer_inputs.items():
                    visualizer_inputs_.update({k: self.data[v] if v in self.data else eval(v)})
                # visualize_inputs_ = {k: self.data[v] if v in self.data else eval(v) for k, v in
                #                      visualize_inputs.items()}
                ret['im'] = visualizer(**visualizer_inputs_)

            # save
            try:
                filename = datetime.datetime.now().strftime('%y%m%d%H%M%S')
                if visualizer is not None:
                    cv2.imwrite(os.path.join(tcp_data_dir, filename + '_out.png'), ret['im'][:, :, ::-1])
                if 'rgb' in self.data:
                    cv2.imwrite(os.path.join(tcp_data_dir, filename + '_rgb.png'), self.data['rgb'][:, :, ::-1])
                if 'depth' in self.data:
                    cv2.imwrite(os.path.join(tcp_data_dir, filename + '_depth.png'), self.data['depth'])
            except:
                print('Cannot save images...')
            return ret

    return ModuleServer(host=cfg.server.host, port=cfg.server.port)


import hydra
from omegaconf import DictConfig


@hydra.main(version_base=None, config_path='../../configs', config_name='server')
def demo_run_server(cfg: DictConfig):
    from threading import Thread
    class TestModule():
        def run(self, **kwargs):
            show_data(kwargs)
            return {'msg': f'Server {cfg.server.host}:{cfg.server.port} returned...'}

    server = make_server(cfg, TestModule)
    server.listen()


if __name__ == '__main__':
    demo_run_server()
    # client = ClientThread()
    # client.send_and_get_return({1: np.ones(1,1)})
